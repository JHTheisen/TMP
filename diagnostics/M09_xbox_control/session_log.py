"""Flushed, per-run diagnostic text logs. No serial or motor dependencies."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time
import traceback


SLOW_OPERATION_SECONDS = 0.150


class SessionLog:
    def __init__(self, directory, settings):
        self.path = None
        self.error = ""
        self._file = None
        self._samples = {}
        self._state = None
        self._start = time.monotonic()
        try:
            directory = Path(directory).expanduser().resolve()
            directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            # Exclusive creation never overwrites an earlier run, even on collision.
            for suffix in range(1000):
                path = directory / f"m09_{stamp}_{os.getpid()}_{suffix}.log"
                try:
                    self._file = path.open("x", encoding="utf-8", buffering=1)
                    self.path = path
                    break
                except FileExistsError:
                    continue
            if self._file is None:
                raise OSError("Could not allocate a unique session log")
            self.event("SESSION", "start " + json.dumps(settings, sort_keys=True))
        except OSError as error:
            self._failed(error)

    def _failed(self, error):
        self.error = str(error)
        print(f"Diagnostic logging unavailable: {error}", file=sys.stderr)
        if self._file is not None:
            try:
                self._file.close()
            except OSError:
                pass
            self._file = None

    def event(self, kind, message):
        if self._file is None:
            return
        stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        elapsed = time.monotonic() - self._start
        try:
            write_started = time.monotonic()
            self._file.write(f"{stamp} +{elapsed:.3f}s {kind} {message}\n")
            write_elapsed = time.monotonic() - write_started
            flush_started = time.monotonic()
            self._file.flush()  # Survives Python exceptions/termination; not a power-loss fsync guarantee.
            flush_elapsed = time.monotonic() - flush_started
            # Do not recurse through event(): the diagnostic write itself must
            # remain bounded to one extra record even when storage is slow.
            for operation, duration in (("session_log.write", write_elapsed),
                                        ("session_log.flush", flush_elapsed)):
                if duration >= SLOW_OPERATION_SECONDS:
                    timing_stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                    timing_elapsed = time.monotonic() - self._start
                    self._file.write(
                        f"{timing_stamp} +{timing_elapsed:.3f}s HOST_TIMING "
                        f"operation={operation} elapsed_ms={duration * 1000:.1f} "
                        f"threshold_ms={SLOW_OPERATION_SECONDS * 1000:.0f}\n")
                    self._file.flush()
        except OSError as error:
            self._failed(error)  # Disk trouble does not add a motor interlock.

    def timing(self, operation, elapsed, threshold=SLOW_OPERATION_SECONDS):
        """Record only abnormal synchronous latency; normal loops stay quiet."""
        if elapsed >= threshold:
            self.event("HOST_TIMING", f"operation={operation} elapsed_ms={elapsed * 1000:.1f} "
                       f"threshold_ms={threshold * 1000:.0f}")

    def sampled(self, key, kind, message, now, period, signature=None):
        previous = self._samples.get(key)
        if previous is None or previous[1] != signature or now - previous[0] >= period:
            self.event(kind, message)
            self._samples[key] = (now, signature)

    def received(self, line, now):
        fields = dict(token.split("=", 1) for token in line.split() if "=" in token)
        prefix = line.split(" ", 1)[0]
        if prefix in ("ORIENTATION_STATE", "ENCODER_STATE", "MANUAL_STATE", "POSE_STATE", "STATE"):
            signature = tuple(fields.get(key) for key in (
                "available", "valid", "has_sample", "fresh", "north_set", "level_set",
                "status", "magnet_good", "active", "protocol"))
            self.sampled((prefix, fields.get("bus")), "RX", line,
             now, 5.0, signature)
        elif line in ("M09 READY", "M09 MANUAL", "M09 BUSY", "M09 ABORTED"):
            self.sampled("firmware_mode", "RX", line, now, 5.0, line)
        else:
            self.event("RX", line)

    def transmitted(self, data, now):
        command = data.decode("ascii", errors="replace").strip()
        if command.startswith("JOG "):
            fields = command.split()[1:]
            signs = tuple(0 if int(value) == 0 else (1 if int(value) > 0 else -1) for value in fields)
            # Starts, stops and reversals are immediate; changing magnitude/held
            # velocity is sampled at 2 Hz instead of logging the 50 Hz stream.
            self.sampled("jog", "TX_ATTEMPT", command, now, 2.0, signs)
        elif command == "STATUS":
            self.sampled("status", "TX_ATTEMPT", command, now, 5.0)
        else:
            self.event("TX_ATTEMPT", command)

    def state(self, session):
        value = (session.state, session.ready, session.pending_stop, session.readiness_note)
        if value != self._state:
            self.event("HOST_STATE", f"state={value[0]} ready={value[1]} pending_stop={value[2]} reason={value[3]}")
            self._state = value

    def exception(self, context):
        self.event("EXCEPTION", context + "\n" + traceback.format_exc().rstrip())

    def close(self, result):
        self.event("SESSION", f"exit result={result}")
        if self._file is not None:
            try:
                self._file.close()
            except OSError as error:
                self._failed(error)
            self._file = None
