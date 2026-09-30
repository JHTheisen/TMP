"""M09 host Xbox input. --dry-run never imports pyserial or opens a port."""
import argparse
import math
from pathlib import Path
import textwrap
import time
from session_log import SessionLog
from auto_control import AutoSession

SEND_INTERVAL = 0.020
COMMAND_TIMEOUT = 0.250  # Same command lease as the firmware; a lapse needs a new arm.


def stick_command(value, deadband=0.15, scale=0.25, invert=False):
    """Continuous deadband rescaling; exact zero at rest, quadratic fine control."""
    if not math.isfinite(value):
        raise ValueError("Nonfinite joystick input")
    if abs(value) <= deadband:
        return 0
    magnitude = min(1.0, (abs(value) - deadband) / (1.0 - deadband))
    sign = -1 if value < 0 else 1
    if invert:
        sign = -sign
    return sign * round(1000 * scale * magnitude * magnitude)


class ManualSession:
    """Protocol state, independent of pygame/serial so faults can be tested offline."""
    def __init__(self):
        self.state = "idle"
        self.ready = False
        self.last_rx = 0.0
        self.transition_at = 0.0
        self.last_reason = ""
        self.firmware_phase = "UNKNOWN"
        self.sensor_fields = {}
        self.sensor_status_at = None
        self.encoder_fields = {}
        self.encoder_status_at = {}
        self.bno_raw_fields = {}
        self.bno_raw_at = None
        self.pending_stop = False
        self.readiness_note = "Waiting for firmware status."
        self.raw_mode = False
        self.auto_mode = False
        self.raw_text = ""
        self.raw_waiting = False
        self.last_raw_command = ""
        self.carriage_steps = None
        self.carriage_received_at = None

    def disarm(self, reason, request_stop=False):
        self.state = "idle"
        self.ready = False
        self.readiness_note = reason
        self.pending_stop = self.pending_stop or request_stop

    def receive(self, line, now):
        self.last_rx = now
        fields = dict(token.split("=", 1) for token in line.split() if "=" in token)
        if line.startswith(("MANUAL_STATE ", "POSE_STATE ", "KEYMOVE_STATE ",
                            "KEYFRAME_SNAPSHOT ", "KEYMOVE RESULT ")) and "carriage_steps" in fields:
            try:
                int(fields["carriage_steps"])
            except ValueError:
                pass
            else:
                self.carriage_steps = fields["carriage_steps"]
                self.carriage_received_at = now
        if line.startswith("BNO_STATE "):
            self.sensor_fields = fields
            self.sensor_status_at = now
        elif line.startswith("BNO_RAW "):
            self.bno_raw_fields = fields
            self.bno_raw_at = now
        elif line.startswith("ENCODER_STATE ") and fields.get("bus") in ("A", "B"):
            self.encoder_fields[fields["bus"]] = fields
            self.encoder_status_at[fields["bus"]] = now
        elif line.startswith("STATE "):
            self.firmware_phase = line.split()[1]
            # Accept older firmware's status without overwriting dedicated sensor reports.
            if not self.sensor_fields or "available" not in self.sensor_fields:
                self.sensor_fields = fields
                self.sensor_status_at = now
        if line.startswith("PITCH-ONLY READY:"):
            self.firmware_phase = "PITCH-ONLY READY"
            self.readiness_note = "North reference unavailable for absolute yaw; waiting for manual M09 READY."
        if line.startswith("Reason:"):
            self.last_reason = line
        if line == "M09 ABORTED" or "FINAL RESULT: FAIL" in line:
            self.state = "fault"
            self.ready = False
            self.pending_stop = False
            self.readiness_note = self.last_reason or line
            return
        if line.startswith("MANUAL REJECTED:"):
            self.disarm(line + " Center sticks and rearm after READY.", request_stop=not self.raw_mode)
        elif line.startswith(("MANUAL STOPPED:", "OPERATION FAILED:")):
            self.disarm(line + " Rearm deliberately after READY.")
        elif line.startswith("M09_xbox_control:"):
            self.disarm("Firmware restarted; wait for READY and rearm.")
            self.carriage_steps = self.carriage_received_at = None
            if self.raw_mode:
                self.raw_waiting = True
        if line.startswith("MANUAL READY:") and self.state == "arming":
            self.state = "active"
            self.firmware_phase = "MANUAL"
            self.readiness_note = "Manual control active."
        elif line == "M09 READY":
            if self.state in ("active", "arming"):
                self.disarm("Firmware left manual mode; center sticks and rearm.", request_stop=True)
            self.state = "idle"
            self.ready = True
            self.firmware_phase = "READY"
            if self.raw_mode:
                self.raw_waiting = False
                self.readiness_note = "Raw commands enabled; joystick JOG disabled. F2 exits without arming."
            elif self.auto_mode:
                self.readiness_note = "Automatic action active; sticks cancel and take over. A/Space stops."
            else:
                self.readiness_note = "Center sticks for 0.5 s to enable manual control."
        elif line == "M09 BUSY":
            self.ready = False
            if self.state in ("active", "arming"):
                self.disarm("Firmware unavailable; wait for READY and rearm.", request_stop=True)

    def diagnostic_lines(self, now):
        fields = self.sensor_fields
        age = "no report" if self.sensor_status_at is None else f"received {now - self.sensor_status_at:.1f} s ago"
        rows = [
            f"Firmware: {self.firmware_phase} | {self.readiness_note}",
            f"Last BNO status ({age}): available={fields.get('available', '?')}; accuracy={fields.get('accuracy', '?')}; "
            f"fresh={fields.get('fresh', fields.get('BNO_fresh', '?'))}; sample_age_ms={fields.get('age_ms', '?')}; has_sample={fields.get('has_sample', '?')}",
            f"Last measured heading={fields.get('heading', '?')} deg; physical pitch/{fields.get('pitch_axis', 'ROLL')}={fields.get('physical_pitch', fields.get('pitch_roll', fields.get('pitch', '?')))} deg; "
            f"north_usable={fields.get('north_usable', '?')}",
        ]
        carriage_age = ("no report" if self.carriage_received_at is None else
                        f"received {now - self.carriage_received_at:.1f} s ago")
        rows.append(f"Current carriage_steps={self.carriage_steps if self.carriage_steps is not None else '?'} "
                    f"({carriage_age}; unhomed generated steps)")
        for bus in ("A", "B"):
            encoder = self.encoder_fields.get(bus, {})
            received = self.encoder_status_at.get(bus)
            age = "no report" if received is None else f"received {now - received:.1f} s ago"
            rows.append(f"AS5600 bus {bus} ({age}): available={encoder.get('available', '?')}; "
                        f"valid={encoder.get('valid', '?')}; raw={encoder.get('raw', '?')}; "
                        f"angle={encoder.get('angle_deg', '?')} deg; age_ms={encoder.get('age_ms', '?')}; "
                        f"status={encoder.get('status', '?')}; magnet_good={encoder.get('magnet_good', '?')}")
        if self.bno_raw_fields:
            raw = self.bno_raw_fields
            rows.append(f"Raw BNO (received {now - self.bno_raw_at:.1f} s ago): report={raw.get('report_id', '?')}; "
                        f"status={raw.get('raw_status', '?')}; accepted={raw.get('accepted', '?')}; "
                        f"age_ms={raw.get('age_ms', '?')}; reason={raw.get('reason', '?')}; "
                        f"diagnostic_quality={raw.get('diagnostic_quality', '?')} (quaternion/Euler in log)")
        if self.state == "active" and now - self.last_rx > 1.0:
            rows.append("Telemetry delayed; live joystick commands continue. Firmware still stops on command loss.")
        return rows

    def arm(self, now, centered):
        if self.raw_mode or self.auto_mode or self.state != "idle" or self.pending_stop or not self.ready or not centered:
            return False
        self.state = "arming"
        self.ready = False
        self.transition_at = now
        return True

    def stop(self, now):
        if self.state not in ("active", "arming"):
            return None
        self.state = "stopping"
        self.ready = False
        self.transition_at = now
        return b"STOP\n"

    def stop_raw(self, now):
        """Explicit cancellation, also valid while host is idle during POSE."""
        self.disarm("STOP sent; wait for firmware READY. Manual control stays disarmed.")
        self.pending_stop = False  # Caller sends STOP now; do not leave a deferred duplicate.
        self.raw_waiting = self.raw_mode
        self.transition_at = now
        return b"STOP\n"

    def enter_raw(self, now):
        self.raw_mode = True
        self.raw_text = ""
        return self.stop_raw(now)

    def leave_raw(self, now):
        self.raw_mode = False
        self.raw_text = ""
        return self.stop_raw(now)

    def submit_raw(self):
        """One deliberate line, never an automatically queued/retried command."""
        if not self.raw_mode:
            raise ValueError("Press F2 to enter raw-command mode.")
        if not self.raw_text.strip():
            raise ValueError("Empty command; type a command first.")
        if self.raw_waiting or self.pending_stop:
            raise ValueError("Waiting for READY after STOP; press Enter again when ready. Nothing queued.")
        command = self.raw_text
        self.disarm("Raw command sent; firmware decides admission. Joystick JOG remains disabled.")
        self.last_raw_command = command
        self.raw_text = ""
        return (command + "\n").encode("utf-8")

    def frame(self, now, yaw, pitch, carriage=0):
        if self.state in ("arming", "stopping") and now - self.transition_at > 4.0:
            self.disarm("Manual handshake/stop timed out; wait for READY and rearm.", request_stop=True)
        if self.pending_stop:
            self.pending_stop = False
            self.ready = False  # Require a READY received after this STOP.
            if self.raw_mode:
                self.raw_waiting = True
            return b"STOP\n"
        if self.raw_mode or self.auto_mode:
            return None
        if self.state == "arming":
            return b"JOG 0 0 0\n"
        if self.state == "active":
            return f"JOG {yaw} {pitch} {carriage}\n".encode("ascii")
        return None


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--dry-run", action="store_true", help="show all raw axes and commands; no serial access")
    parser.add_argument("--log-dir", default=str(Path(__file__).resolve().parent / "logs"),
                        help="directory for a new timestamped diagnostic log each run")
    parser.add_argument("--controller", type=int, default=0)
    parser.add_argument("--yaw-axis", type=int, default=0, help="right stick horizontal; verify with --dry-run")
    parser.add_argument("--pitch-axis", type=int, default=1, help="left stick vertical; verify with --dry-run")
    parser.add_argument("--carriage-axis", type=int, default=2, help="left stick horizontal; verify with --dry-run")
    parser.add_argument("--invert-carriage", action="store_true", help="reverse only the left/right carriage direction")
    parser.add_argument("--invert-yaw", action="store_true")
    parser.add_argument("--no-invert-pitch", action="store_true", help="default maps stick up (negative raw) to positive physical pitch")
    parser.add_argument("--deadband", type=float, default=0.15)
    parser.add_argument("--speed-scale", type=float, default=1.0, help="fraction of existing ceilings, default 1.0; valid (0,1]")
    for name, default in (("capture-a", 4), ("capture-b", 5), ("duration", 2),
                          ("play", 10), ("return-a", 8), ("increment", 9)):
        parser.add_argument("--" + name + "-button", type=int, default=default,
                            help="pygame button index; confirm button/hat events with --dry-run")
    parser.add_argument("--move-hat", type=int, default=0, help="D-pad hat index; confirm with --dry-run")
    args = parser.parse_args(argv)
    if not 0.05 <= args.deadband <= 0.5:
        parser.error("--deadband must be between 0.05 and 0.5")
    if not 0 < args.speed_scale <= 1:
        parser.error("--speed-scale must be in (0,1]")
    if min(args.controller, args.yaw_axis, args.pitch_axis, args.carriage_axis) < 0 or len({args.yaw_axis, args.pitch_axis, args.carriage_axis}) != 3:
        parser.error("controller/axis indices must be nonnegative and yaw/pitch/carriage axes must differ")
    buttons = [getattr(args, name + "_button") for name in ("capture_a", "capture_b", "duration", "play", "return_a", "increment")]
    if min(buttons + [args.move_hat]) < 0 or len(set(buttons)) != len(buttons) or any(value in (0, 1) for value in buttons):
        parser.error("AUTO buttons must be distinct, nonnegative, and must not replace A=0 or B=1")
    return args


def main(argv=None):
    args = arguments(argv)
    log = SessionLog(args.log_dir, {**vars(args), "script": str(Path(__file__).resolve()),
                                   "command_interval_s": SEND_INTERVAL, "command_timeout_s": COMMAND_TIMEOUT})
    print(f"M09 diagnostic log: {log.path}" if log.path else "M09 diagnostic logging unavailable; see error above.")
    pygame = None
    port = None
    session = ManualSession()
    auto = AutoSession()
    failed = False
    explicit_abort = False
    try:
        import pygame
        pygame.init()
        if args.controller >= pygame.joystick.get_count():
            raise RuntimeError("No selected controller; connect Xbox, then rerun")
        joystick = pygame.joystick.Joystick(args.controller)
        joystick.init()
        if max(args.yaw_axis, args.pitch_axis, args.carriage_axis) >= joystick.get_numaxes():
            raise RuntimeError("Selected axis unavailable; inspect with valid --yaw-axis/--pitch-axis/--carriage-axis indices")
        screen = pygame.display.set_mode((1100, 960))
        pygame.display.set_caption("M09 Xbox manual control" + (" - DRY RUN (no serial)" if args.dry_run else ""))
        pygame.key.stop_text_input()  # Ordinary control keys are never command text.
        font = pygame.font.SysFont("consolas", 18)
        text_columns = (screen.get_width() - 24) // font.size("M")[0]
        lines = []
        if not args.dry_run:
            import serial
            # Opening this port may reset ESP32; manual control always needs arming.
            log.event("SERIAL", f"opening {args.port} baud=115200; opening may reset ESP32")
            port = serial.Serial(args.port, 115200, timeout=0, write_timeout=0.1)
            log.event("SERIAL", "opened")
        log.event("CONTROLLER", f"name={joystick.get_name()} axes={joystick.get_numaxes()}")
        def send(data, raw_command=False):
            if raw_command:
                # Do not parse or sample arbitrary input as a generated JOG.
                # repr retains exact bytes, whitespace and the final newline.
                log.event("RAW_TX_ATTEMPT", repr(data))
            else:
                log.transmitted(data, time.monotonic())
            if port is not None and port.write(data) != len(data):
                raise RuntimeError("Incomplete serial write")
            if raw_command:
                log.event("RAW_TX", repr(session.last_raw_command))
        now = time.monotonic()
        last_jog = None
        next_send = next_status = now
        centered_since = None
        rx = ""
        discard_rx_line = False
        running = True
        focused = True
        display_frozen = False
        diagnostic_snapshot = None
        response_snapshot = []
        held_buttons = set()
        hat_neutral = True
        input_notice = "MANUAL + KEYFRAMES: LB/RB capture; Home plays. Y is unassigned."
        takeover = False
        while running:
            now = time.monotonic()
            events = pygame.event.get()  # Pumps input before every read, as in v05.
            now = time.monotonic()
            for event in events:
                if event.type == pygame.JOYDEVICEREMOVED and event.instance_id == joystick.get_instance_id():
                    raise RuntimeError("Xbox controller disconnected")
                if event.type == pygame.WINDOWFOCUSLOST:
                    log.event("INPUT", "window focus lost")
                    focused = False
                    centered_since = None
                    takeover = False
                    if session.state in ("active", "arming") or session.raw_mode or auto.enabled:
                        session.disarm("Window lost focus; return, center sticks and rearm.", request_stop=True)
                        auto.cancel(now, "Window lost focus; STOP requested, wait for READY.")
                        if session.raw_mode:
                            session.raw_waiting = True
                elif event.type == pygame.WINDOWFOCUSGAINED:
                    log.event("INPUT", "window focus gained")
                    focused = True
            was_automatic = auto.overridable
            if port is not None:
                received = port.read(min(port.in_waiting, 4096))
                if any(value > 127 for value in received):
                    log.sampled("non_ascii", "PARSER", f"non-ASCII serial bytes: {received[:96]!r}", now, 0.5)
                rx += received.decode("ascii", errors="replace")
                if discard_rx_line and "\n" in rx:
                    _, rx = rx.split("\n", 1)
                    discard_rx_line = False
                while "\n" in rx:
                    line, rx = rx.split("\n", 1)
                    line = line.strip()
                    if line:
                        log.received(line, now)
                        session.receive(line, now)
                        auto.receive(line, now)
                        # Repeated BUSY polls must not push out the startup reason.
                        if line != "M09 BUSY":
                            lines = (lines + [line])[-8:]
                if len(rx) > 8192:
                    rx = ""
                    discard_rx_line = True
                    log.event("PARSER", "overlong telemetry line discarded (>8192 bytes); command stream continues")
                    lines = (lines + ["Malformed telemetry line discarded; command stream continues."])[-8:]
            session.auto_mode = auto.busy
            # Read current axes after receiving status; never replay pre-pause input.
            raw = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            yaw = stick_command(raw[args.yaw_axis], args.deadband, args.speed_scale, args.invert_yaw)
            pitch = stick_command(raw[args.pitch_axis], args.deadband, args.speed_scale, not args.no_invert_pitch)
            carriage = stick_command(raw[args.carriage_axis], args.deadband, args.speed_scale, True)
            now = time.monotonic()
            centered = all(abs(raw[index]) <= args.deadband for index in (args.yaw_axis, args.pitch_axis, args.carriage_axis))
            centered_since = (now if centered_since is None else centered_since) if centered and focused else None
            if session.state in ("active", "arming") and last_jog is not None and now - last_jog >= COMMAND_TIMEOUT:
                session.disarm("Command stream paused; center sticks and rearm.", request_stop=True)
            def selected_button(event, button):
                return (event.type == pygame.JOYBUTTONDOWN and
                        event.instance_id == joystick.get_instance_id() and event.button == button)
            def pressed(event, key):
                return event.type == pygame.KEYDOWN and event.key == key
            # Track physical release even outside AUTO. Holding a control across
            # mode entry/completion cannot turn it into another request.
            fresh_buttons = set()
            hat_actions = {}
            for index, event in enumerate(events):
                if getattr(event, "instance_id", None) != joystick.get_instance_id():
                    continue
                if event.type == pygame.JOYBUTTONUP:
                    held_buttons.discard(event.button)
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button not in held_buttons:
                        fresh_buttons.add(index)
                    held_buttons.add(event.button)
                    input_notice = f"Controller button {event.button} down"
                    log.event("CONTROLLER_INPUT", input_notice)
                elif event.type == pygame.JOYHATMOTION and event.hat == args.move_hat:
                    input_notice = f"Controller hat {event.hat}: {event.value}"
                    log.event("CONTROLLER_INPUT", input_notice)
                    if event.value == (0, 0):
                        hat_neutral = True
                    else:
                        if hat_neutral:
                            hat_actions[index] = {(0, 1): "pitch_up", (0, -1): "pitch_down",
                                                  (1, 0): "yaw_right", (-1, 0): "yaw_left"}.get(event.value)
                        hat_neutral = False
            # Safety actions win over Enter even when queued later in the same
            # pygame batch. No raw move should precede an already-pending stop.
            if any(pressed(event, pygame.K_x) or selected_button(event, 1) for event in events):
                explicit_abort = True
                raise RuntimeError("Operator keyboard X / controller B abort")
            if any(event.type == pygame.QUIT or pressed(event, pygame.K_ESCAPE) for event in events):
                break
            toggling_raw = any(pressed(event, pygame.K_F2) for event in events)
            safety_stop = any(pressed(event, pygame.K_F12) or pressed(event, pygame.K_SPACE) or
                              selected_button(event, 0) for event in events)
            suppress_submit = safety_stop or toggling_raw or any(event.type == pygame.WINDOWFOCUSLOST for event in events)
            if safety_stop:
                takeover = False
                centered_since = None
                stop = session.stop_raw(now)
                auto.cancel(now)
                log.event("INPUT", "operator STOP; raw command submission suppressed for this input batch")
                if not args.dry_run:
                    send(stop)
                else:
                    session.raw_waiting = False
            for index, event in enumerate(events):
                key = event.key if event.type == pygame.KEYDOWN else None
                if key == pygame.K_F3 and focused and not getattr(event, "repeat", False):
                    display_frozen = not display_frozen
                    log.event("DISPLAY", "frozen; controls and telemetry remain live" if display_frozen else "live; showing latest state")
                    continue
                if key == pygame.K_F2 and focused and not getattr(event, "repeat", False):
                    takeover = False
                    auto.leave(now)
                    session.auto_mode = True
                    stop = session.leave_raw(now) if session.raw_mode else session.enter_raw(now)
                    centered_since = None
                    last_jog = None
                    if session.raw_mode:
                        pygame.key.start_text_input()
                    else:
                        pygame.key.stop_text_input()
                    log.event("RAW_MODE", "entered; STOP and disarm" if session.raw_mode else "left; STOP and deliberate rearm required")
                    if not args.dry_run:
                        send(stop)
                    else:
                        session.raw_waiting = False
                    continue
                if session.raw_mode:
                    if not focused:
                        continue
                    if key == pygame.K_TAB:
                        session.raw_text += " "  # Keep Space reserved for STOP in every mode.
                    elif key == pygame.K_BACKSPACE:
                        session.raw_text = session.raw_text[:-1]
                    elif key in (pygame.K_RETURN, pygame.K_KP_ENTER) and not suppress_submit:
                        try:
                            data = session.submit_raw()
                        except ValueError as error:
                            notice = str(error)
                            log.event("RAW_NOT_SENT", notice)
                            lines = (lines + [notice])[-8:]
                        else:
                            if args.dry_run:
                                log.event("RAW_DRY_RUN", repr(data))
                            else:
                                send(data, raw_command=True)
                            echo = f"{'RAW DRY RUN' if args.dry_run else 'RAW TX'}: {session.last_raw_command!r}"
                            print(echo)
                            lines = (lines + [echo])[-8:]
                    elif event.type == pygame.TEXTINPUT and event.text != " ":
                        # Only explicit editor text is accepted, never KEYDOWN
                        # unicode or joystick buttons. Keep input a single line.
                        session.raw_text += "".join(character for character in event.text if character.isprintable())
                    continue
                if auto.enabled:
                    if not focused or suppress_submit:
                        continue
                    action = hat_actions.get(index)
                    if index in fresh_buttons:
                        if event.button == args.increment_button:
                            auto.increment = 2 if auto.increment == 1 else 1
                        elif event.button == args.duration_button:
                            if auto.phase == "READY":
                                auto.duration = {5: 10, 10: 20, 20: 5}[auto.duration]
                        else:
                            action = {args.capture_a_button: "capture_a", args.capture_b_button: "capture_b",
                                      args.return_a_button: "return_a", args.play_button: "play"}.get(event.button)
                    if action:
                        data = auto.request(action, now, centered_since is not None and now - centered_since >= 0.5)
                        if data is not None:
                            session.stop_raw(now)
                            session.auto_mode = True
                            last_jog = None
                            centered_since = None
                            takeover = False
                            log.event("AUTO_TX", repr(data))
                            if args.dry_run:
                                print(f"AUTO DRY RUN: {data!r}")
                                auto.cancel(now, "Dry run: no firmware response or movement; captures require firmware.")
                                auto.receive("M09 READY", now)
                            else:
                                send(data)
                        else:
                            log.event("AUTO_NOT_SENT", auto.note)
                    continue
            # A deliberate stick displacement cancels the entire sequence before
            # any prepared follow-up can be sent. Wait for STOP/READY and the zero
            # JOG acknowledgment, then use only the current live stick value.
            if focused and not session.raw_mode and not suppress_submit and not centered and (auto.overridable or (was_automatic and auto.phase == "READY")):
                stop = session.stop_raw(now)
                if not args.dry_run:
                    send(stop)
                auto.cancel(now, "Joystick takeover: braking, then manual JOG.")
                takeover = True
                last_jog = None
            session.auto_mode = auto.busy
            if focused and not session.raw_mode and not auto.busy and not suppress_submit and now >= next_send:
                centered_ready = centered_since is not None and now - centered_since >= .5
                if session.arm(now, takeover or centered_ready):
                    last_jog = None
                    takeover = False
            if session.pending_stop:
                takeover = False
            if now >= next_send:
                if not args.dry_run:
                    frame = session.frame(now, yaw, pitch, carriage)
                    if frame:
                        if frame == b"STOP\n" and auto.enabled:
                            auto.cancel(now)
                        send(frame)
                        if frame.startswith(b"JOG "):
                            last_jog = now
                next_send = now + SEND_INTERVAL  # No catch-up bursts of old input.
            if auto.enabled and focused and not suppress_submit and not args.dry_run:
                command = auto.frame(now)
                if command is not None:
                    log.event("AUTO_TX", repr(command))
                    send(command)
            if port is not None and now >= next_status:
                send(b"STATUS\n")
                next_status = now + 0.5
            log.state(session)
            if args.dry_run:
                log.sampled("dry_input", "INPUT_ONLY", f"yaw={yaw} pitch={pitch} carriage={carriage} raw={raw}", now, 0.5)
            display = [
                ("DISPLAY FROZEN | F3: resume latest | Controls, serial and logging remain LIVE"
                 if display_frozen else "DISPLAY LIVE | F3: freeze diagnostic values and scrolling responses only"),
                f"{joystick.get_name()} | {'DRY RUN - SERIAL CLOSED' if args.dry_run else args.port + ' | ' + session.state.upper()}",
               "Left stick horizontal = YAW; left stick vertical = PITCH; right stick horizontal = CARRIAGE."
                f"Yaw axis {args.yaw_axis}, pitch axis {args.pitch_axis}, carriage axis {args.carriage_axis} | deadband {args.deadband:.2f} | speed scale {args.speed_scale:.2f}",
                f"Yaw: {yaw:+5d}   Pitch: {pitch:+5d}   Carriage: {carriage:+5d}   Centered: {centered}   Ready: {session.ready}",
                "Raw axes: " + "  ".join(f"{i}:{v:+.2f}" for i, v in enumerate(raw)),
                input_notice,
                "A / Space / F12: STOP. Keyboard X / B: latched abort. Esc / close: stop & exit.",
                "F2: raw command line / leave & STOP. F12: STOP. In raw mode Space / A always STOP; Tab inserts spaces.",
                "Center sticks to enable manual. Stick movement cancels automatic actions. Keep window focused.",
                f"Log: {log.path.name if log.path else 'UNAVAILABLE'} (full path printed at startup)" + (f" ERROR: {log.error}" if log.error else ""),
                "",
            ]
            if session.raw_mode:
                raw_status = "WAITING FOR READY AFTER STOP" if session.raw_waiting else "Enter sends once; firmware decides admission"
                display += [f"RAW MODE — JOG DISABLED | {raw_status}",
                            "Tab: space; Backspace: erase; Enter: send; F2: leave disarmed.",
                            "> " + session.raw_text[-max(1, text_columns - 4):] + "_"]
            else:
                display += auto.display_lines()
            # Cache only presentation text. Protocol state, RX/logging, joystick
            # processing and the raw-command editor above always remain live.
            if not display_frozen or diagnostic_snapshot is None:
                diagnostic_snapshot = session.diagnostic_lines(now) + [""] if not args.dry_run else []
                response_snapshot = list(lines)
            display += diagnostic_snapshot
            display = [row for line in display for row in (textwrap.wrap(line, text_columns) or [""])]
            log_rows = [row for line in response_snapshot for row in (textwrap.wrap(line, text_columns) or [""])]
            available_rows = max(0, (screen.get_height() - 24) // 23 - len(display))
            if available_rows:
                display += log_rows[-available_rows:]
            screen.fill((20, 24, 30))
            if display_frozen:
                screen.fill((255, 195, 45), (0, 0, screen.get_width(), 35))
            for i, line in enumerate(display):
                color = (20, 24, 30) if display_frozen and i == 0 else (230, 235, 242)
                screen.blit(font.render(line, True, color), (12, 12 + i * 23))
            pygame.display.flip()
            time.sleep(0.005)
    except KeyboardInterrupt:
        failed = True
        log.exception("Operator Ctrl+C stop")
        print("Operator Ctrl+C stop")
    except Exception as error:
        failed = True
        log.exception("Python controller stopped")
        print(f"Stopped: {error}")
    finally:
        if port is not None:
            try:
                # Only an explicit operator abort latches the firmware. The
                # command lease also covers failed writes or a disconnected link.
                if explicit_abort:
                    log.transmitted(b"X\n", time.monotonic())
                    port.write(b"X\n")
                else:
                    log.transmitted(b"\nSTOP\n", time.monotonic())
                    port.write(b"\nSTOP\n")
            except Exception:
                log.exception("Serial stop/abort write failed during cleanup")
            try:
                port.close()
                log.event("SERIAL", "closed")
            except Exception:
                log.exception("Serial close failed")
        if pygame is not None:
            try:
                pygame.quit()
            except Exception:
                log.exception("pygame cleanup failed")
        log.state(session)
        log.close(1 if failed else 0)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
