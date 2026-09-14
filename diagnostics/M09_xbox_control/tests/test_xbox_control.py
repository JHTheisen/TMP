from pathlib import Path
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from xbox_control import ManualSession, arguments, stick_command


class InputTests(unittest.TestCase):
    def test_center_and_noise_are_exact_zero(self):
        for value in (-0.15, -0.05, 0, 0.05, 0.15):
            self.assertEqual(stick_command(value), 0)

    def test_continuous_curve_limits_and_signs(self):
        commands = [stick_command(i / 1000) for i in range(1001)]
        self.assertEqual(commands, sorted(commands))
        self.assertLessEqual(max(b - a for a, b in zip(commands, commands[1:])), 1)
        self.assertEqual(commands[-1], 250)
        self.assertEqual(stick_command(-1), -250)
        self.assertEqual(stick_command(-1, invert=True), 250)
        self.assertEqual(stick_command(1, scale=1), 1000)
        self.assertEqual(stick_command(1.1), 250)

    def test_release_has_no_smoothed_tail(self):
        self.assertEqual([stick_command(v) for v in (1, 1, 0, 0)], [250, 250, 0, 0])

    def test_nonfinite_rejected(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.assertRaises(ValueError):
                stick_command(value)

    def test_cli_defaults_and_bounds(self):
        args = arguments([])
        self.assertEqual((args.yaw_axis, args.pitch_axis, args.speed_scale), (2, 1, 0.25))
        self.assertEqual(args.carriage_axis, 0)
        self.assertFalse(args.invert_carriage)
        self.assertTrue(arguments(["--invert-carriage"]).invert_carriage)
        for flags in (["--speed-scale", "nan"], ["--speed-scale", "1.1"], ["--deadband", "0"],
                      ["--pitch-axis", "2"], ["--controller", "-1"],
                      ["--carriage-axis", "1"], ["--carriage-axis", "-1"]):
            with mock.patch("sys.stderr"), self.assertRaises(SystemExit):
                arguments(flags)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.session = ManualSession()

    def arm(self):
        self.session.receive("M09 READY", 1.0)
        self.assertTrue(self.session.arm(1.1, True))
        self.assertEqual(self.session.frame(1.12, 250, -250, 250), b"JOG 0 0 0\n")
        self.session.receive("MANUAL READY: test", 1.14)

    def test_no_automatic_start(self):
        self.assertIsNone(self.session.frame(0, 250, 250))
        self.assertFalse(self.session.arm(0, True))
        self.session.receive("M09 READY", 1)
        self.assertFalse(self.session.arm(1.1, False))
        self.assertFalse(self.session.arm(3, True))
        self.assertIsNone(self.session.frame(3, 250, 250))

    def test_handshake_stream_stop_and_deliberate_rearm(self):
        self.arm()
        self.assertEqual(self.session.frame(1.16, 120, -50, -75), b"JOG 120 -50 -75\n")
        self.assertEqual(self.session.frame(1.18, 0, 0), b"JOG 0 0 0\n")
        self.assertEqual(self.session.stop(1.2), b"STOP\n")
        self.assertIsNone(self.session.frame(1.22, 250, 250))
        self.session.receive("M09 READY", 1.3)
        self.assertEqual(self.session.state, "idle")
        self.assertIsNone(self.session.frame(1.32, 250, 250))
        self.assertTrue(self.session.arm(1.4, True))

    def test_lost_feedback(self):
        self.arm()
        with self.assertRaisesRegex(RuntimeError, "telemetry lost"):
            self.session.frame(2.2, 250, 250)

    def test_firmware_fault_and_rejection(self):
        for line in ("M09 ABORTED", "FINAL RESULT: FAIL", "MANUAL REJECTED: bad input"):
            with self.assertRaises(RuntimeError):
                self.session.receive(line, 1)
            self.assertEqual(self.session.state, "fault")
            self.assertFalse(self.session.arm(1, True))
        self.session.receive("Reason: BNO085 feedback stale", 2)
        with self.assertRaisesRegex(RuntimeError, "BNO085 feedback stale"):
            self.session.receive("FINAL RESULT: FAIL", 2)

    def test_reset_does_not_resume_stale_input(self):
        self.arm()
        with self.assertRaises(RuntimeError):
            self.session.receive("M09 BUSY", 1.3)

    def test_acknowledgment_timeout(self):
        self.session.receive("M09 READY", 1)
        self.session.arm(1, True)
        self.session.receive("STATE BASELINE", 5.2)
        with self.assertRaisesRegex(RuntimeError, "handshake"):
            self.session.frame(5.3, 0, 0)


if __name__ == "__main__":
    unittest.main()
