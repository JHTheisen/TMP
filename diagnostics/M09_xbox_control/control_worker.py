"""Serial owner and autonomous scheduler, independent of the SDL event pump.

The GUI may mutate session models only under ``lock`` and enqueues commands.
Never hold that lock while pumping events, polling SDL, or drawing. The worker
alone reads/writes/closes the port; commands expire instead of being replayed.
"""
from collections import deque
from contextlib import contextmanager
from copy import copy
import threading
import time


class ControlWorker:
    COMMAND_MAX_AGE = .250

    def __init__(self, port, session, auto, encoder_setup, log, *, clock=time.monotonic,
                 safety_input=None):
        self.port, self.session, self.auto = port, session, auto
        self.encoder_setup, self.log, self.clock = encoder_setup, log, clock
        self.safety_input = safety_input
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.condition = threading.Condition(self.lock)
        self.commands = deque()
        self.lines = []
        self.error = None
        self.closed = False
        self.abort = False
        self.cycle = 0
        self.rx = ""
        self.discard_rx_line = False
        self.next_status = clock()
        self.last_ui = clock()
        self.native_takeover = False
        self.native_abort = False
        self.thread = threading.Thread(target=self._run, name="gimbal-control", daemon=True)
        self.thread.start()

    @contextmanager
    def transaction(self):
        with self.lock:
            self.check()
            yield
            self.last_ui = self.clock()
        self.wake.set()

    def check(self):
        if self.error is not None:
            raise RuntimeError(f"Communications worker stopped: {self.error}") from self.error

    def synchronize(self):
        """Wait for one worker pass, without transferring serial ownership.

        Also keeps UI handshakes deterministic with virtual clocks in tests.
        The worker never waits for the GUI and continues while it is blocked.
        """
        with self.condition:
            self.check()
            cycle = self.cycle
            self.wake.set()
            if not self.condition.wait_for(lambda: self.cycle > cycle or self.error or self.closed, 1):
                raise RuntimeError("Communications worker unresponsive")
            self.check()

    def send(self, data, raw_command=False, *, created_at=None):
        with self.lock:
            self.check()
            # STOP/abort are barriers: earlier unsent movement cannot follow them.
            if data.strip() == b"STOP":
                # A later STOP in the same UI batch must not downgrade a B/X
                # latched abort to an ordinary recoverable stop.
                aborts = [entry for entry in self.commands if entry[1].strip() == b"X"]
                self.commands.clear()
                self.commands.extend(aborts)
            elif data.strip() == b"X":
                self.commands.clear()
                self.abort = True
            if len(self.commands) >= 64:
                raise RuntimeError("Communications command queue full")
            self.commands.append((self.clock() if created_at is None else created_at,
                                  data, raw_command, self.session.last_raw_command))
        self.wake.set()

    def snapshots(self):
        # The renderer only reads these detached dictionaries and scalar fields.
        with self.lock:
            session, auto = copy(self.session), copy(self.auto)
            session.sensor_fields = dict(session.sensor_fields)
            session.encoder_fields = {k: dict(v) for k, v in session.encoder_fields.items()}
            session.encoder_status_at = dict(session.encoder_status_at)
            session.celestial_fields = dict(session.celestial_fields)
            auto.frames = dict(auto.frames)
            return session, auto, auto.display_lines()

    def _write(self, data, raw=False, raw_text=""):
        if raw:
            self.log.event("RAW_TX_ATTEMPT", repr(data))
        else:
            self.log.transmitted(data, self.clock())
        if self.port is not None:
            started = self.clock()
            written = self.port.write(data)
            self.log.timing("serial.write", self.clock() - started)
            if written != len(data):
                raise RuntimeError("Incomplete serial write")
        if raw:
            self.log.event("RAW_TX", repr(raw_text))

    def _receive(self, now):
        if self.port is None:
            return
        received = self.port.read(min(self.port.in_waiting, 4096))
        if any(value > 127 for value in received):
            self.log.sampled("non_ascii", "PARSER", f"non-ASCII serial bytes: {received[:96]!r}", now, .5)
        self.rx += received.decode("ascii", errors="replace")
        if self.discard_rx_line and "\n" in self.rx:
            _, self.rx = self.rx.split("\n", 1)
            self.discard_rx_line = False
        while "\n" in self.rx:
            line, self.rx = self.rx.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            self.log.received(line, now)
            was_celestial = self.auto.action == "celestial"
            self.session.receive(line, now)
            self.auto.receive(line, now)
            if (self.session.state == "fault" or line.startswith("M09_xbox_control:") or
                    (was_celestial and self.auto.action != "celestial")):
                # Drop stale motion, preserving operator safety commands. The
                # GUI marks B as locally faulted before its X reaches the port;
                # buffered telemetry must not swallow that unsent abort.
                safe = [entry for entry in self.commands
                        if entry[1].strip() in (b"X", b"STOP", b"STATUS")]
                self.commands.clear()
                self.commands.extend(safe)
            configuring = self.encoder_setup.pending
            self.encoder_setup.receive(line)
            if configuring and self.encoder_setup.phase == "done":
                self.session.readiness_note = "Encoder ratios applied. Center sticks to arm; SET NORTH and SET LEVEL required."
                self.log.event("ENCODER_SETUP", self.session.readiness_note)
            if line != "M09 BUSY":
                self.lines[:] = (self.lines + [line])[-8:]
        if len(self.rx) > 8192:
            self.rx = ""
            self.discard_rx_line = True
            self.log.event("PARSER", "overlong telemetry line discarded (>8192 bytes); command stream continues")
            self.lines[:] = (self.lines + ["Malformed telemetry line discarded; command stream continues."])[-8:]

    def _service(self):
        now = self.clock()
        self._receive(now)
        # Independent Xbox polling matters particularly inside Windows modal SDL
        # window operations. Neutral/focus loss never cancels an automatic mode.
        if self.safety_input is not None:
            safety = self.safety_input()
            if safety == "abort":
                if not self.native_abort:
                    self.commands.clear()
                    self._write(b"X\n")
                    self.auto.receive("M09 ABORTED", now)
                    self.session.receive("M09 ABORTED", now)
                    self.encoder_setup.receive("M09 ABORTED")
                    self.native_abort = self.abort = True
            elif safety == "takeover" and self.auto.overridable:
                self.commands.clear()
                self._write(self.session.stop_raw(now))
                self.auto.cancel(now, "Joystick takeover: braking, then manual JOG.")
                self.native_takeover = True
            elif safety != "abort":
                self.native_abort = False
        while self.commands:
            queued, data, raw, raw_text = self.commands.popleft()
            if self.clock() - queued >= self.COMMAND_MAX_AGE and data.strip() not in (b"STOP", b"X", b"STATUS"):
                self.commands.clear()
                self.auto.cancel(now, "Queued command expired; center sticks and rearm.")
                self.session.disarm("Queued command expired; center sticks and rearm.")
                self._write(b"STOP\n")
                break
            self._write(data, raw, raw_text)
        if now - self.last_ui >= self.COMMAND_MAX_AGE and (
                self.session.state in ("active", "arming") or self.session.pending_stop):
            self.session.disarm("Command stream paused; center sticks and rearm.")
            self.session.pending_stop = False
            self.auto.cancel(now, "Command stream paused; center sticks and rearm.")
            self._write(b"STOP\n")
        if self.encoder_setup.pending:
            command = self.encoder_setup.frame(now)
            if command is not None:
                self._write(command)
        if self.auto.enabled and self.port is not None:
            started = self.clock()
            command = self.auto.frame(now)
            self.log.timing("AutoSession.frame", self.clock() - started)
            if command is not None:
                self.log.event("AUTO_TX", repr(command))
                self._write(command)
            for event in self.auto.take_camera_events():
                self.log.event("CAMERA_TRIGGER", f"segment={event.segment}/{event.segments} epoch={event.epoch} steps={event.steps} at={event.requested_at:.3f} hook=EVENT_ONLY")
        if self.port is not None and now >= self.next_status:
            self._write(b"STATUS\n")
            self.next_status = now + .5

    def _run(self):
        try:
            while True:
                self.wake.wait(.005)
                self.wake.clear()
                with self.condition:
                    if self.closed:
                        break
                    self._service()
                    self.cycle += 1
                    self.condition.notify_all()
        except Exception as error:
            with self.condition:
                self.error = error
                self.commands.clear()
                self.auto.cancel(self.clock(), f"Communications failure: {error}")
                self.session.disarm(f"Communications failure: {error}")
                self.log.exception("Communications worker stopped")
                self.condition.notify_all()
        finally:
            # No retries of motion after a failed read/write. If STOP cannot be
            # delivered, the unchanged firmware lease is the final interlock.
            if self.port is not None:
                try:
                    self._write(b"X\n" if self.abort else b"\nSTOP\n")
                except Exception:
                    self.log.exception("Serial stop/abort write failed during cleanup")
                try:
                    self.port.close()
                    self.log.event("SERIAL", "closed")
                except Exception:
                    self.log.exception("Serial close failed")

    def close(self, abort=False):
        with self.lock:
            self.abort = self.abort or abort
            self.closed = True
            self.commands.clear()
        self.wake.set()
        self.thread.join(1)
        if self.thread.is_alive():
            raise RuntimeError("Communications worker failed to stop; firmware lease remains active")
