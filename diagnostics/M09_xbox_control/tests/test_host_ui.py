"""Real pygame headless UI with fake joystick/serial; never accesses hardware."""
import os
from pathlib import Path
import sys
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
        class FakeSerial:
            rx = b""
            active = False
            closed = False
            @property
            def in_waiting(self):
                return len(self.rx)
            def read(self, length):
                data, self.rx = self.rx[:length], self.rx[length:]
                return data
            def write(self, data):
                writes.append(data)
                if data == b"STATUS\n":
                    self.rx += b"M09 MANUAL\n" if self.active else b"M09 READY\n"
                elif data == b"JOG 0 0 0\n" and not self.active:
                    self.active = True
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
            if clock.now > 1.1:
                result.append(pygame.event.Event(pygame.QUIT))
            return result
        with mock.patch.object(pygame.joystick, "get_count", return_value=1), \
             mock.patch.object(pygame.joystick, "Joystick", return_value=joystick), \
             mock.patch.object(pygame.event, "get", side_effect=events), \
             mock.patch.object(serial, "Serial", return_value=port) as open_port, \
             mock.patch.object(xbox_control, "time", clock):
            flags = ["--dry-run"] if scenario == "dry_run" else (["--invert-carriage"] if scenario == "inverted" else [])
            result = xbox_control.main(flags)
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
            self.assertTrue(any(int(value.split()[1]) > 0 and int(value.split()[2]) > 0 for value in jogs))
            carriage_jogs = [value.split() for value in jogs if int(value.split()[3]) != 0]
            self.assertTrue(carriage_jogs)
            self.assertTrue(all(int(value[1]) == 0 and int(value[2]) == 0 for value in carriage_jogs))
            self.assertTrue(all((int(value[3]) > 0) == (scenario == "inverted") for value in carriage_jogs))
            if scenario in ("normal", "inverted"):
                self.assertEqual(jogs[-1], b"JOG 0 0 0\n")
            self.assertFalse(any(value.startswith((b"POSE", b"MOVE")) for value in writes))
            if scenario in ("normal", "inverted"):
                self.assertEqual(writes[-1], b"\nSTOP\n")
                self.assertEqual(result, 0)
            else:
                self.assertEqual(writes[-1], b"X\n")
                self.assertEqual(result, 1)

    def test_dry_run_never_opens_serial(self):
        self.run_ui("dry_run")

    def test_center_arm_motion_release_exit(self):
        self.run_ui("normal")

    def test_disconnect_aborts(self):
        self.run_ui("disconnect")

    def test_focus_loss_aborts(self):
        self.run_ui("focus")

    def test_host_pause_aborts(self):
        self.run_ui("pause")

    def test_carriage_direction_inversion(self):
        self.run_ui("inverted")

    def test_uncentered_carriage_prevents_arming(self):
        self.run_ui("uncentered")


if __name__ == "__main__":
    unittest.main()
