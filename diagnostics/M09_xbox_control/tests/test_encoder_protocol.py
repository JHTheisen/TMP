"""Compare the real compiled firmware telemetry with both host consumers."""
from pathlib import Path
import shutil
import subprocess
import unittest

from auto_control import AutoSession
from celestial_coordinates import Observer, HeadingReference
from xbox_control import ManualSession

ROOT = Path(__file__).resolve().parents[1]
REPORT = ("ORIENTATION_STATE protocol=2 feedback=AS5600 available=YES has_sample=YES "
          "fresh=YES north_set=YES level_set=YES age_ms=10 heading=100 physical_pitch=30 pitch_axis=PITCH")


class EncoderProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("g++") or r"C:\Strawberry\c\bin\g++.exe"
        cls.executable = ROOT / ".pio/host_tests/encoder_protocol.exe"
        cls.executable.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run([compiler, "-std=c++11", "-DM07_HOST_TEST", "-I", "tests/stubs",
                        "-I", "../../firmware/include", "tests/encoder_motion_test.cpp", "-o", str(cls.executable)],
                       cwd=ROOT, capture_output=True, text=True, check=True)

    def test_firmware_telemetry_is_accepted_by_host(self):
        result = subprocess.run([str(self.executable), "protocol"], cwd=ROOT,
                                capture_output=True, text=True, check=True)
        manual, auto = ManualSession(), AutoSession()
        for line in result.stdout.splitlines():
            manual.receive(line, 1)
            auto.receive(line, 1)
        self.assertTrue(auto.pointing_ready(1.1))
        self.assertEqual(manual.sensor_fields["roll"], "UNAVAILABLE")
        self.assertEqual(manual.sensor_fields["north_set"], "YES")
        self.assertEqual(manual.sensor_fields["level_set"], "YES")
        self.assertEqual(set(manual.encoder_fields), {"A", "B"})
        self.assertIn("KEYFRAME_SNAPSHOT id=9", result.stdout)

    def test_goto_refuses_missing_old_stale_or_uncalibrated_protocol(self):
        for report in (None, REPORT.replace("protocol=2", "protocol=1"),
                       REPORT.replace("north_set=YES", "north_set=NO"),
                       REPORT.replace("level_set=YES", "level_set=NO"),
                       REPORT.replace("fresh=YES", "fresh=NO"),
                       REPORT.replace("heading=100", "heading=nan"),
                       REPORT.replace("age_ms=10", "age_ms=3000")):
            with self.subTest(report=report):
                auto = AutoSession()
                auto.configure_celestial(Observer(42, -83), HeadingReference())
                self.addCleanup(auto.close)
                if report:
                    auto.receive(report, 1)
                with self.assertRaisesRegex(ValueError, "protocol 2"):
                    auto.request_celestial("TRACK_RADEC 1 30", 1.1, True)
                self.assertEqual(auto.phase, "READY")
                self.assertEqual(auto.last_command, "")

    def test_reference_age_and_reboot_invalidate_host_readiness(self):
        auto = AutoSession()
        auto.receive(REPORT, 1)
        self.assertTrue(auto.pointing_ready(1.1))
        self.assertFalse(auto.pointing_ready(3.1))
        auto.receive(REPORT, 4)
        auto.receive("M09_xbox_control: dual AS5600 protocol=2", 4.1)
        self.assertFalse(auto.pointing_ready(4.2))

    def test_calibration_actions_stop_then_send_once_and_return_ready(self):
        for action in ("set_north", "set_level"):
            auto = AutoSession()
            self.assertIsNone(auto.request(action, 0, False))
            self.assertEqual(auto.request(action, 1, True), b"STOP\n")
            auto.receive("M09 READY", 1.1)
            self.assertEqual(auto.frame(1.2), (action.upper() + "\n").encode())
            self.assertIsNone(auto.frame(1.21))
            auto.receive("CALIBRATION " + action.upper() + ": reference stored", 1.3)
            auto.receive("M09 READY", 1.4)
            self.assertEqual(auto.phase, "READY")

    def test_calibration_rejection_does_not_leave_host_busy(self):
        auto = AutoSession()
        auto.request("set_north", 1, True)
        auto.receive("M09 READY", 1.1)
        auto.frame(1.2)
        auto.receive("CALIBRATION REJECTED: encoder unavailable", 1.3)
        self.assertEqual(auto.frame(1.4), b"STATUS\n")
        auto.receive("M09 READY", 1.5)
        self.assertEqual(auto.phase, "READY")
