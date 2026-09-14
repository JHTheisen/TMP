"""M09 host Xbox input. --dry-run never imports pyserial or opens a port."""
import argparse
import math
import textwrap
import time

SEND_INTERVAL = 0.020
HOST_PAUSE_LIMIT = 0.100


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
        self.readiness_note = "Waiting for firmware status."

    def receive(self, line, now):
        self.last_rx = now
        if line.startswith("STATE "):
            self.firmware_phase = line.split()[1]
            self.sensor_fields = dict(token.split("=", 1) for token in line.split()[2:] if "=" in token)
            self.sensor_status_at = now
            if self.firmware_phase in ("STARTUP", "BASELINE"):
                self.readiness_note = "Waiting for a stable calibrated baseline; displayed target errors are not valid yet."
            else:
                self.readiness_note = "Waiting for firmware completion and READY."
        if line.startswith("PITCH-ONLY READY:"):
            self.firmware_phase = "PITCH-ONLY READY"
            self.readiness_note = "North reference unavailable for absolute yaw; waiting for manual M09 READY."
        if line.startswith("Reason:"):
            self.last_reason = line
        if line == "M09 ABORTED" or "FINAL RESULT: FAIL" in line or line.startswith("MANUAL REJECTED:"):
            self.state = "fault"
            self.ready = False
            raise RuntimeError(self.last_reason if "FINAL RESULT: FAIL" in line and self.last_reason else line)
        if line.startswith("MANUAL READY:") and self.state == "arming":
            self.state = "active"
            self.firmware_phase = "MANUAL"
            self.readiness_note = "Manual control active."
        elif line == "M09 READY":
            if self.state in ("active", "arming"):
                raise RuntimeError("Firmware left manual mode unexpectedly; rearm deliberately")
            self.ready = True
            self.firmware_phase = "READY"
            self.readiness_note = "Center sticks for 0.5 s, then press Space/A to arm."
            if self.state == "stopping":
                self.state = "idle"
        elif line == "M09 BUSY":
            self.ready = False
            if self.state == "active":
                raise RuntimeError("Firmware restarted or became unavailable during manual control")

    def diagnostic_lines(self, now):
        fields = self.sensor_fields
        age = "no report" if self.sensor_status_at is None else f"received {now - self.sensor_status_at:.1f} s ago"
        return [
            f"Firmware: {self.firmware_phase} | {self.readiness_note}",
            f"Last BNO status ({age}): accuracy={fields.get('accuracy', '?')} / manual accepts 0-3; "
            f"fresh={fields.get('BNO_fresh', '?')}; sample_age_ms={fields.get('age_ms', '?')}",
            f"Last measured heading={fields.get('heading', '?')} deg; physical pitch/ROLL={fields.get('pitch', '?')} deg; "
            f"north_usable={fields.get('north_usable', '?')}",
        ]

    def arm(self, now, centered):
        if self.state != "idle" or not self.ready or not centered or now - self.last_rx > 1.0:
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
        if self.state in ("arming", "active", "stopping") and now - self.last_rx > 1.0:
            raise RuntimeError("Firmware telemetry lost; stopping")
        if self.state in ("arming", "stopping") and now - self.transition_at > 4.0:
            raise RuntimeError("Manual handshake/stop did not complete")
        if self.state == "arming":
            return b"JOG 0 0 0\n"
        if self.state == "active":
            return f"JOG {yaw} {pitch} {carriage}\n".encode("ascii")
        return None


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--dry-run", action="store_true", help="show all raw axes and commands; no serial access")
    parser.add_argument("--controller", type=int, default=0)
    parser.add_argument("--yaw-axis", type=int, default=2, help="right stick horizontal; verify with --dry-run")
    parser.add_argument("--pitch-axis", type=int, default=1, help="left stick vertical; verify with --dry-run")
    parser.add_argument("--carriage-axis", type=int, default=0, help="left stick horizontal; verify with --dry-run")
    parser.add_argument("--invert-carriage", action="store_true", help="reverse only the left/right carriage direction")
    parser.add_argument("--invert-yaw", action="store_true")
    parser.add_argument("--no-invert-pitch", action="store_true", help="default maps stick up (negative raw) to positive physical pitch")
    parser.add_argument("--deadband", type=float, default=0.15)
    parser.add_argument("--speed-scale", type=float, default=1.0, help="fraction of existing ceilings, default 0.25; valid (0,1]")
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
    import pygame
    port = None
    session = ManualSession()
    failed = False
    pygame.init()
    try:
        if args.controller >= pygame.joystick.get_count():
            raise RuntimeError("No selected controller; connect Xbox, then rerun")
        joystick = pygame.joystick.Joystick(args.controller)
        joystick.init()
        if max(args.yaw_axis, args.pitch_axis, args.carriage_axis) >= joystick.get_numaxes():
            raise RuntimeError("Selected axis unavailable; inspect with valid --yaw-axis/--pitch-axis/--carriage-axis indices")
        screen = pygame.display.set_mode((1000, 620))
        pygame.display.set_caption("M09 Xbox manual control" + (" - DRY RUN (no serial)" if args.dry_run else ""))
        font = pygame.font.SysFont("consolas", 18)
        text_columns = (screen.get_width() - 24) // font.size("M")[0]
        lines = []
        if not args.dry_run:
            import serial
            # Opening this port may reset ESP32 and run preserved M08 startup motion.
            port = serial.Serial(args.port, 115200, timeout=0, write_timeout=0.1)
        def send(data):
            if port is not None and port.write(data) != len(data):
                raise RuntimeError("Incomplete serial write")
        now = time.monotonic()
        previous = now
        next_send = next_status = now
        centered_since = None
        rx = ""
        running = True
        while running:
            now = time.monotonic()
            if session.state in ("active", "arming") and now - previous > HOST_PAUSE_LIMIT:
                raise RuntimeError("Host input loop paused over 100 ms; refusing stale stick commands")
            previous = now
            events = pygame.event.get()  # Pumps input before every read, as in v05.
            if session.state in ("active", "arming") and time.monotonic() - now > HOST_PAUSE_LIMIT:
                raise RuntimeError("Host input acquisition paused over 100 ms")
            for event in events:
                if event.type == pygame.JOYDEVICEREMOVED and event.instance_id == joystick.get_instance_id():
                    raise RuntimeError("Xbox controller disconnected")
                if event.type == pygame.WINDOWFOCUSLOST and session.state in ("active", "arming"):
                    raise RuntimeError("Control window lost focus")
            raw = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            yaw = stick_command(raw[args.yaw_axis], args.deadband, args.speed_scale, args.invert_yaw)
            pitch = stick_command(raw[args.pitch_axis], args.deadband, args.speed_scale, not args.no_invert_pitch)
            carriage = stick_command(raw[args.carriage_axis], args.deadband, args.speed_scale, args.invert_carriage)
            centered = all(abs(raw[index]) <= args.deadband for index in (args.yaw_axis, args.pitch_axis, args.carriage_axis))
            centered_since = (now if centered_since is None else centered_since) if centered else None
            if port is not None:
                rx += port.read(min(port.in_waiting, 4096)).decode("ascii", errors="replace")
                while "\n" in rx:
                    line, rx = rx.split("\n", 1)
                    line = line.strip()
                    if line:
                        session.receive(line, now)
                        # Repeated BUSY polls must not push out the startup reason.
                        if line != "M09 BUSY":
                            lines = (lines + [line])[-8:]
                if len(rx) > 8192:
                    raise RuntimeError("Serial response exceeded bounded line buffer")
            for event in events:
                key = event.key if event.type == pygame.KEYDOWN else None
                selected_button = event.type == pygame.JOYBUTTONDOWN and event.instance_id == joystick.get_instance_id()
                if key == pygame.K_x or (selected_button and event.button == 1):
                    raise RuntimeError("Operator X/B abort")
                if event.type == pygame.QUIT or key == pygame.K_ESCAPE:
                    running = False
                if key == pygame.K_SPACE or (selected_button and event.button == 0):
                    if args.dry_run:
                        continue
                    stop = session.stop(now)
                    if stop:
                        send(stop)
                    elif not session.arm(now, centered_since is not None and now - centered_since >= 0.5):
                        lines = (lines + ["Arm refused: wait for M09 READY and center both sticks for 0.5 s."])[-8:]
            if session.state in ("active", "arming") and time.monotonic() - now > HOST_PAUSE_LIMIT:
                raise RuntimeError("Host input frame expired before transmission")
            if not running:
                break
            if now >= next_send:
                if not args.dry_run:
                    frame = session.frame(now, yaw, pitch, carriage)
                    if frame:
                        send(frame)
                next_send = now + SEND_INTERVAL  # No catch-up bursts of old input.
            if port is not None and now >= next_status:
                send(b"STATUS\n")
                next_status = now + 0.5
            display = [
                f"{joystick.get_name()} | {'DRY RUN - SERIAL CLOSED' if args.dry_run else args.port + ' | ' + session.state.upper()}",
                "Right stick horizontal = YAW; left stick vertical = PITCH; left stick horizontal = CARRIAGE.",
                f"Yaw axis {args.yaw_axis}, pitch axis {args.pitch_axis}, carriage axis {args.carriage_axis} | deadband {args.deadband:.2f} | speed scale {args.speed_scale:.2f}",
                f"Yaw: {yaw:+5d}   Pitch: {pitch:+5d}   Carriage: {carriage:+5d}   Centered: {centered}   Ready: {session.ready}",
                "Raw axes: " + "  ".join(f"{i}:{v:+.2f}" for i, v in enumerate(raw)),
                "Space / A: arm centered or stop & disarm. X / B: latched abort. Esc / close: stop & exit.",
                "Keep this window focused. Lost input/link stops; faults require deliberate board reset.",
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
        print("Operator Ctrl+C abort")
    except Exception as error:
        failed = True
        print(f"Stopped: {error}")
    finally:
        if port is not None:
            try:
                # X is immediate even in a partial command. STOP is for normal
                # exit from manual only; the independent lease covers link loss.
                if failed:
                    port.write(b"X\n")
                elif session.state in ("active", "arming", "stopping"):
                    port.write(b"\nSTOP\n")
                elif not session.ready:
                    port.write(b"X\n")  # Closing during automatic startup must stop it too.
            except Exception:
                pass
            port.close()
        pygame.quit()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
