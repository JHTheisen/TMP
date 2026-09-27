"""M09 host Xbox input. --dry-run never imports pyserial or opens a port."""
import argparse
import math
from pathlib import Path
import textwrap
import time
from session_log import SessionLog

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

    def disarm(self, reason, request_stop=False):
        self.state = "idle"
        self.ready = False
        self.readiness_note = reason
        self.pending_stop = self.pending_stop or request_stop

    def receive(self, line, now):
        self.last_rx = now
        fields = dict(token.split("=", 1) for token in line.split() if "=" in token)
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
            self.disarm(line + " Center sticks and rearm after READY.", request_stop=True)
        elif line.startswith(("MANUAL STOPPED:", "OPERATION FAILED:")):
            self.disarm(line + " Rearm deliberately after READY.")
        elif line.startswith("M09_xbox_control:"):
            self.disarm("Firmware restarted; wait for READY and rearm.")
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
            self.readiness_note = "Center sticks for 0.5 s, then press Space/A to arm."
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
            f"Last measured heading={fields.get('heading', '?')} deg; physical pitch/ROLL={fields.get('pitch_roll', fields.get('pitch', '?'))} deg; "
            f"north_usable={fields.get('north_usable', '?')}",
        ]
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
        if self.state != "idle" or self.pending_stop or not self.ready or not centered:
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

    def frame(self, now, yaw, pitch, carriage=0):
        if self.state in ("arming", "stopping") and now - self.transition_at > 4.0:
            self.disarm("Manual handshake/stop timed out; wait for READY and rearm.", request_stop=True)
        if self.pending_stop:
            self.pending_stop = False
            self.ready = False  # Require a READY received after this STOP.
            return b"STOP\n"
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
    parser.add_argument("--yaw-axis", type=int, default=2, help="right stick horizontal; verify with --dry-run")
    parser.add_argument("--pitch-axis", type=int, default=1, help="left stick vertical; verify with --dry-run")
    parser.add_argument("--carriage-axis", type=int, default=0, help="left stick horizontal; verify with --dry-run")
    parser.add_argument("--invert-carriage", action="store_true", help="reverse only the left/right carriage direction")
    parser.add_argument("--invert-yaw", action="store_true")
    parser.add_argument("--no-invert-pitch", action="store_true", help="default maps stick up (negative raw) to positive physical pitch")
    parser.add_argument("--deadband", type=float, default=0.15)
    parser.add_argument("--speed-scale", type=float, default=1.0, help="fraction of existing ceilings, default 1.0; valid (0,1]")
    args = parser.parse_args(argv)
    if not 0.05 <= args.deadband <= 0.5:
        parser.error("--deadband must be between 0.05 and 0.5")
    if not 0 < args.speed_scale <= 1:
        parser.error("--speed-scale must be in (0,1]")
    if min(args.controller, args.yaw_axis, args.pitch_axis, args.carriage_axis) < 0 or len({args.yaw_axis, args.pitch_axis, args.carriage_axis}) != 3:
        parser.error("controller/axis indices must be nonnegative and yaw/pitch/carriage axes must differ")
    return args


def main(argv=None):
    args = arguments(argv)
    log = SessionLog(args.log_dir, {**vars(args), "script": str(Path(__file__).resolve()),
                                   "command_interval_s": SEND_INTERVAL, "command_timeout_s": COMMAND_TIMEOUT})
    print(f"M09 diagnostic log: {log.path}" if log.path else "M09 diagnostic logging unavailable; see error above.")
    pygame = None
    port = None
    session = ManualSession()
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
        screen = pygame.display.set_mode((1100, 720))
        pygame.display.set_caption("M09 Xbox manual control" + (" - DRY RUN (no serial)" if args.dry_run else ""))
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
        def send(data):
            log.transmitted(data, time.monotonic())
            if port is not None and port.write(data) != len(data):
                raise RuntimeError("Incomplete serial write")
        now = time.monotonic()
        last_jog = None
        next_send = next_status = now
        centered_since = None
        rx = ""
        discard_rx_line = False
        running = True
        focused = True
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
                    if session.state in ("active", "arming"):
                        session.disarm("Window lost focus; return, center sticks and rearm.", request_stop=True)
                elif event.type == pygame.WINDOWFOCUSGAINED:
                    log.event("INPUT", "window focus gained")
                    focused = True
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
                        # Repeated BUSY polls must not push out the startup reason.
                        if line != "M09 BUSY":
                            lines = (lines + [line])[-8:]
                if len(rx) > 8192:
                    rx = ""
                    discard_rx_line = True
                    log.event("PARSER", "overlong telemetry line discarded (>8192 bytes); command stream continues")
                    lines = (lines + ["Malformed telemetry line discarded; command stream continues."])[-8:]
            # Read current axes after receiving status; never replay pre-pause input.
            raw = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            yaw = stick_command(raw[args.yaw_axis], args.deadband, args.speed_scale, args.invert_yaw)
            pitch = stick_command(raw[args.pitch_axis], args.deadband, args.speed_scale, not args.no_invert_pitch)
            carriage = stick_command(raw[args.carriage_axis], args.deadband, args.speed_scale, args.invert_carriage)
            now = time.monotonic()
            centered = all(abs(raw[index]) <= args.deadband for index in (args.yaw_axis, args.pitch_axis, args.carriage_axis))
            centered_since = (now if centered_since is None else centered_since) if centered and focused else None
            if session.state in ("active", "arming") and last_jog is not None and now - last_jog >= COMMAND_TIMEOUT:
                session.disarm("Command stream paused; center sticks and rearm.", request_stop=True)
            for event in events:
                key = event.key if event.type == pygame.KEYDOWN else None
                selected_button = event.type == pygame.JOYBUTTONDOWN and event.instance_id == joystick.get_instance_id()
                if key == pygame.K_x or (selected_button and event.button == 1):
                    explicit_abort = True
                    raise RuntimeError("Operator X/B abort")
                if event.type == pygame.QUIT or key == pygame.K_ESCAPE:
                    running = False
                if key == pygame.K_SPACE or (selected_button and event.button == 0):
                    if args.dry_run:
                        continue
                    stop = session.stop(now)
                    if stop:
                        send(stop)
                    elif not session.arm(now, focused and centered_since is not None and now - centered_since >= 0.5):
                        reason = "Arm refused: wait for M09 READY and center both sticks for 0.5 s."
                        log.event("ARM_REFUSED", f"{reason} state={session.state} ready={session.ready} focused={focused} centered_since={centered_since}")
                        lines = (lines + [reason])[-8:]
                    else:
                        last_jog = None
            if not running:
                break
            if now >= next_send:
                if not args.dry_run:
                    frame = session.frame(now, yaw, pitch, carriage)
                    if frame:
                        send(frame)
                        if frame.startswith(b"JOG "):
                            last_jog = now
                next_send = now + SEND_INTERVAL  # No catch-up bursts of old input.
            if port is not None and now >= next_status:
                send(b"STATUS\n")
                next_status = now + 0.5
            log.state(session)
            if args.dry_run:
                log.sampled("dry_input", "INPUT_ONLY", f"yaw={yaw} pitch={pitch} carriage={carriage} raw={raw}", now, 0.5)
            display = [
                f"{joystick.get_name()} | {'DRY RUN - SERIAL CLOSED' if args.dry_run else args.port + ' | ' + session.state.upper()}",
                "Right stick horizontal = YAW; left stick vertical = PITCH; left stick horizontal = CARRIAGE.",
                f"Yaw axis {args.yaw_axis}, pitch axis {args.pitch_axis}, carriage axis {args.carriage_axis} | deadband {args.deadband:.2f} | speed scale {args.speed_scale:.2f}",
                f"Yaw: {yaw:+5d}   Pitch: {pitch:+5d}   Carriage: {carriage:+5d}   Centered: {centered}   Ready: {session.ready}",
                "Raw axes: " + "  ".join(f"{i}:{v:+.2f}" for i, v in enumerate(raw)),
                "Space / A: arm centered or stop & disarm. X / B: latched abort. Esc / close: stop & exit.",
                "Keep this window focused. Command loss stops motion; telemetry delay only warns. Rearm after stopping.",
                f"Log: {log.path.name if log.path else 'UNAVAILABLE'} (full path printed at startup)" + (f" ERROR: {log.error}" if log.error else ""),
                "",
            ]
            if not args.dry_run:
                display += session.diagnostic_lines(now) + [""]
            display = [row for line in display for row in (textwrap.wrap(line, text_columns) or [""])]
            log_rows = [row for line in lines for row in (textwrap.wrap(line, text_columns) or [""])]
            available_rows = max(0, (screen.get_height() - 24) // 23 - len(display))
            if available_rows:
                display += log_rows[-available_rows:]
            screen.fill((20, 24, 30))
            for i, line in enumerate(display):
                screen.blit(font.render(line, True, (230, 235, 242)), (12, 12 + i * 23))
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
