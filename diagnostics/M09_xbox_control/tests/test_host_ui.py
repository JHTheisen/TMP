"""Real pygame headless UI with fake joystick/serial; never accesses hardware."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pygame
import serial
import xbox_control


class HostUiTests(unittest.TestCase):
    def run_ui(self, scenario):
        class FakeTime:
            now = 0.0
            def monotonic(self):
                return self.now
            def sleep(self, duration):
                self.now += duration
        clock = FakeTime()
        writes = []
        write_times = []
        class FakeSerial:
            rx = b""
            active = False
            closed = False
            @property
            def in_waiting(self):
                return len(self.rx)
            def read(self, length):
                if scenario == "serial_error" and clock.now > 0.9:
                    raise OSError("simulated serial read failure")
                data, self.rx = self.rx[:length], self.rx[length:]
                return data
            def write(self, data):
                writes.append(data)
                write_times.append(clock.now)
                if scenario == "telemetry" and clock.now > 0.9:
                    return len(data)
                if data == b"STATUS\n":
                    self.rx += b"M09 MANUAL\n" if self.active else b"M09 READY\n"
                elif data == b"JOG 0 0 0\n" and not self.active:
                    self.active = True
                    if scenario != "no_ack":
                        self.rx += b"MANUAL READY: test\n"
                elif data.strip() == b"STOP":
                    self.active = False
                    self.rx += b"M09 READY\n"
                return len(data)
            def close(self):
                self.closed = True
        port = FakeSerial()
        joystick = mock.Mock()
        joystick.get_numaxes.return_value = 6
        joystick.get_instance_id.return_value = 42
        joystick.get_name.return_value = "Simulated Xbox"
        def axis(index):
            if scenario == "input_error" and clock.now > 0.9:
                return float("nan")
            if scenario == "telemetry" and clock.now > 1.9 and index == 2:
                return 0.7
            if scenario == "long_pause" and clock.now > 1.0 and index == 2:
                return 0.7
            if scenario == "uncentered" and index == 0:
                return 0.7
            if 0.75 < clock.now < 0.85:
                return 0.7 if index == 2 else (-0.7 if index == 1 else 0)
            if 0.85 <= clock.now < 0.95 and index == 0:
                return -0.7
            return 0.0
        joystick.get_axis.side_effect = axis
        sent = set()
        def events():
            result = []
            if clock.now > 0.6 and "arm" not in sent:
                sent.add("arm")
                result.append(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_SPACE))
            if clock.now > 0.9 and "fault" not in sent:
                sent.add("fault")
                if scenario == "disconnect":
                    result.append(pygame.event.Event(pygame.JOYDEVICEREMOVED, instance_id=42))
                elif scenario == "focus":
                    result.append(pygame.event.Event(pygame.WINDOWFOCUSLOST))
                elif scenario == "pause":
                    clock.now += 0.2
                elif scenario == "long_pause":
                    clock.now += 0.3
                elif scenario == "reject":
                    port.rx += b"MANUAL REJECTED: bad input\n"
                elif scenario == "firmware_fault":
                    port.rx += b"M09 ABORTED\n"
                elif scenario == "abort":
                    result.append(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_x))
                elif scenario == "interrupt":
                    raise KeyboardInterrupt
                elif scenario == "overflow":
                    port.rx += b"?" * 20000 + b"\nBNO_STATE available=YES fresh=YES accuracy=0\n"
            if scenario == "focus" and clock.now > 1.0 and "focus_return" not in sent:
                sent.add("focus_return")
                result.append(pygame.event.Event(pygame.WINDOWFOCUSGAINED))
            quit_at = 5.0 if scenario == "no_ack" else (2.1 if scenario == "telemetry" else 1.4)
            if clock.now > quit_at:
                result.append(pygame.event.Event(pygame.QUIT))
            return result
        with tempfile.TemporaryDirectory() as log_dir, \
             mock.patch.object(pygame.joystick, "get_count", return_value=1), \
             mock.patch.object(pygame.joystick, "Joystick", return_value=joystick), \
             mock.patch.object(pygame.event, "get", side_effect=events), \
             mock.patch.object(serial, "Serial", return_value=port) as open_port, \
             mock.patch.object(xbox_control, "time", clock):
            flags = ["--dry-run"] if scenario == "dry_run" else (["--invert-carriage"] if scenario == "inverted" else [])
            result = xbox_control.main(flags + ["--log-dir", log_dir])
            files = list(Path(log_dir).glob("*.log"))
            self.assertEqual(len(files), 1)
            self.last_log = files[0].read_text(encoding="utf-8")
            self.assertIn("SESSION exit result=", self.last_log)
        if scenario == "dry_run":
            open_port.assert_not_called()
            self.assertEqual(writes, [])
            self.assertEqual(result, 0)
        else:
            open_port.assert_called_once_with("COM9", 115200, timeout=0, write_timeout=0.1)
            self.assertTrue(port.closed)
            jogs = [value for value in writes if value.startswith(b"JOG")]
            if scenario == "uncentered":
                self.assertEqual(jogs, [])
                self.assertEqual(result, 0)
                return
            self.assertEqual(jogs[0], b"JOG 0 0 0\n")
            if scenario == "no_ack":
                self.assertTrue(all(value == b"JOG 0 0 0\n" for value in jogs))
                self.assertIn(b"STOP\n", writes)
                self.assertNotIn(b"X\n", writes)
                self.assertEqual(result, 0)
                return
            self.assertTrue(any(int(value.split()[1]) > 0 and int(value.split()[2]) > 0 for value in jogs))
            carriage_jogs = [value.split() for value in jogs if int(value.split()[3]) != 0]
            self.assertTrue(carriage_jogs)
            self.assertTrue(all(int(value[1]) == 0 and int(value[2]) == 0 for value in carriage_jogs))
            self.assertTrue(all((int(value[3]) > 0) == (scenario == "inverted") for value in carriage_jogs))
            if scenario in ("normal", "inverted"):
                self.assertEqual(jogs[-1], b"JOG 0 0 0\n")
            self.assertFalse(any(value.startswith((b"POSE", b"MOVE")) for value in writes))
            if scenario == "abort":
                self.assertEqual(writes[-1], b"X\n")
                self.assertEqual(result, 1)
            else:
                self.assertEqual(writes[-1], b"\nSTOP\n")
                self.assertNotIn(b"X\n", writes)
                self.assertEqual(result, 1 if scenario in ("disconnect", "input_error", "interrupt", "serial_error") else 0)
            if scenario in ("focus", "reject", "firmware_fault", "long_pause"):
                self.assertFalse(any(stamp > 0.9 and data.startswith(b"JOG")
                                     for stamp, data in zip(write_times, writes)))
            if scenario in ("pause", "telemetry", "overflow"):
                resumed_after = 1.9 if scenario == "telemetry" else 1.1
                self.assertTrue(any(stamp > resumed_after and data.startswith(b"JOG")
                                    for stamp, data in zip(write_times, writes)))
            if scenario == "telemetry":
                self.assertTrue(any(stamp > 1.9 and data.startswith(b"JOG") and int(data.split()[1]) > 0
                                    for stamp, data in zip(write_times, writes)))

    def test_dry_run_never_opens_serial(self):
        self.run_ui("dry_run")

    def test_center_arm_motion_release_exit(self):
        self.run_ui("normal")

    def test_disconnect_sends_stop_without_latched_abort(self):
        self.run_ui("disconnect")

    def test_focus_loss_stops_and_does_not_resume_after_focus_returns(self):
        self.run_ui("focus")

    def test_host_pause_below_command_lease_reads_current_input_and_continues(self):
        self.run_ui("pause")

    def test_command_lease_lapse_disarms_before_nonzero_can_resume(self):
        self.run_ui("long_pause")

    def test_receive_only_telemetry_gap_does_not_stop_live_commands(self):
        self.run_ui("telemetry")

    def test_missing_arm_ack_never_allows_nonzero_commands(self):
        self.run_ui("no_ack")

    def test_malformed_telemetry_does_not_stop_command_stream(self):
        self.run_ui("overflow")

    def test_rejection_uses_recoverable_stop(self):
        self.run_ui("reject")

    def test_firmware_fault_does_not_generate_another_abort(self):
        self.run_ui("firmware_fault")

    def test_explicit_abort_still_sends_x(self):
        self.run_ui("abort")

    def test_invalid_input_uses_stop(self):
        self.run_ui("input_error")
        self.assertIn("Traceback (most recent call last)", self.last_log)
        self.assertIn("Nonfinite joystick input", self.last_log)

    def test_serial_failure_is_logged_independently_of_serial(self):
        self.run_ui("serial_error")
        self.assertIn("OSError: simulated serial read failure", self.last_log)
        self.assertIn("TX_ATTEMPT STOP", self.last_log)

    def test_ctrl_c_uses_stop(self):
        self.run_ui("interrupt")

    def test_carriage_direction_inversion(self):
        self.run_ui("inverted")

    def test_uncentered_carriage_prevents_arming(self):
        self.run_ui("uncentered")


if __name__ == "__main__":
    unittest.main()
