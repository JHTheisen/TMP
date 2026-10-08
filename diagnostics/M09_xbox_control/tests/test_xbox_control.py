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
        self.assertEqual((args.yaw_axis, args.pitch_axis, args.speed_scale), (0, 1, 1.0))
        self.assertEqual(arguments(["--speed-scale", "0.25"]).speed_scale, 0.25)
        self.assertEqual(args.carriage_axis, 2)
        self.assertFalse(args.invert_carriage)
        self.assertTrue(arguments(["--invert-carriage"]).invert_carriage)
        self.assertEqual(arguments(["--speed-scale", "1"]).speed_scale, 1.0)
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
        self.assertIsNone(self.session.frame(3, 250, 250))

    def test_telemetry_age_does_not_block_centered_handshake(self):
        self.session.receive("M09 READY", 1)
        self.assertTrue(self.session.arm(3, True))
        # Old READY permits only the zero handshake, never live stick motion.
        self.assertEqual(self.session.frame(3.02, 250, -250, 100), b"JOG 0 0 0\n")
        self.assertEqual(self.session.state, "arming")
        self.session.receive("MANUAL READY: test", 3.04)
        self.assertEqual(self.session.frame(3.06, 250, -250, 100), b"JOG 250 -250 100\n")

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

    def test_lost_telemetry_warns_without_interrupting_live_commands(self):
        self.arm()
        self.assertEqual(self.session.frame(2.2, 250, 250, -50), b"JOG 250 250 -50\n")
        self.assertEqual(self.session.state, "active")
        self.assertIn("Telemetry delayed", "\n".join(self.session.diagnostic_lines(2.2)))
        self.session.receive("M09 MANUAL", 2.3)
        self.assertNotIn("Telemetry delayed", "\n".join(self.session.diagnostic_lines(2.3)))

    def test_firmware_fault_is_reported_without_sending_another_abort(self):
        for line in ("M09 ABORTED", "FINAL RESULT: FAIL"):
            self.session.receive(line, 1)
            self.assertEqual(self.session.state, "fault")
            self.assertFalse(self.session.arm(1, True))
            self.assertIsNone(self.session.frame(1.1, 250, 250))
        self.session.receive("Reason: ENCODER085 feedback stale", 2)
        self.session.receive("FINAL RESULT: FAIL", 2)
        self.assertIn("ENCODER085 feedback stale", self.session.readiness_note)

    def test_rejection_stops_and_requires_new_centered_arm(self):
        self.arm()
        self.session.receive("MANUAL REJECTED: bad input", 1.3)
        self.assertEqual(self.session.state, "idle")
        self.assertEqual(self.session.frame(1.4, 250, 250), b"STOP\n")
        self.assertIsNone(self.session.frame(1.5, 250, 250))
        self.session.receive("M09 READY", 1.6)
        self.assertFalse(self.session.arm(1.7, False))
        self.assertTrue(self.session.arm(1.7, True))
        self.assertEqual(self.session.frame(1.8, 250, 250), b"JOG 0 0 0\n")

    def test_reset_does_not_resume_stale_input(self):
        self.arm()
        self.session.receive("M09 BUSY", 1.3)
        self.assertEqual(self.session.frame(1.4, 250, 250), b"STOP\n")
        self.session.receive("M09 READY", 1.5)
        self.assertIsNone(self.session.frame(1.6, 250, 250))
        self.assertFalse(self.session.arm(1.6, False))

    def test_command_loss_stop_and_axis_error_have_different_scope(self):
        self.arm()
        self.session.receive("MANUAL AXIS ERROR: carriage stopped", 1.3)
        self.assertEqual(self.session.frame(1.4, 25, 50), b"JOG 25 50 0\n")
        self.session.receive("MANUAL STOPPED: command timeout", 1.5)
        self.session.receive("M09 READY", 1.5)
        self.assertIsNone(self.session.frame(1.6, 250, 250))
        self.assertFalse(self.session.arm(1.6, False))

    def test_pose_failure_does_not_latch_manual_fault(self):
        self.session.receive("OPERATION FAILED: ENCODER unavailable", 1)
        self.session.receive("M09 READY", 1.1)
        self.assertTrue(self.session.arm(1.2, True))

    def test_acknowledgment_timeout(self):
        self.session.receive("M09 READY", 1)
        self.session.arm(1, True)
        self.session.receive("STATE BASELINE", 5.2)
        self.assertEqual(self.session.frame(5.3, 250, 250), b"STOP\n")
        self.assertEqual(self.session.state, "idle")
        self.assertIsNone(self.session.frame(5.4, 250, 250))
        self.assertIn("timed out", self.session.readiness_note)

    def test_stop_timeout_remains_recoverable(self):
        self.arm()
        self.session.stop(1.2)
        self.assertEqual(self.session.frame(5.3, 250, 250), b"STOP\n")
        self.assertIsNone(self.session.frame(5.4, 250, 250))
        self.session.receive("M09 READY", 5.5)
        self.assertTrue(self.session.arm(5.6, True))

    def test_dedicated_sensor_reports_preserve_health_and_receipt_age(self):
        self.arm()
        self.session.receive("ORIENTATION_STATE protocol=2 feedback=AS5600 level_set=YES north_set=YES available=NO fresh=NO age_ms=600 heading=nan roll=UNAVAILABLE", 2)
        self.session.receive("ENCODER_STATE bus=A available=YES valid=YES raw=2048 angle_deg=180.000 age_ms=4 status=0x20", 2)
        self.session.receive("ENCODER_STATE bus=B available=YES valid=NO raw=100 angle_deg=8.789 age_ms=700 status=0x10", 2)
        self.session.receive("STATE MANUAL heading=999", 2.1)
        rows = "\n".join(self.session.diagnostic_lines(3.5))
        self.assertIn("available=NO; fresh=NO", rows)
        self.assertIn("received 1.5 s ago", rows)
        self.assertIn("raw=2048; angle=180.000 deg", rows)
        self.assertIn("valid=NO; raw=100", rows)
        self.assertIn("status=0x10", rows)
        self.assertNotIn("heading=999", rows)
        self.assertEqual(self.session.frame(3.5, 100, 0), b"JOG 100 0 0\n")

    def test_raw_encoder_does_not_overwrite_calibrated_orientation(self):
        self.session.receive("ORIENTATION_STATE protocol=2 feedback=AS5600 north_set=YES level_set=NO fresh=NO", 1)
        self.session.receive("ENCODER_STATE bus=A valid=YES raw=4095 angle_deg=359.912 magnet_good=YES", 2)
        self.assertEqual(self.session.sensor_fields["level_set"], "NO")
        self.assertEqual(self.session.encoder_fields["A"]["raw"], "4095")
        self.assertEqual(self.session.sensor_status_at, 1)


    def test_physical_pitch_display_uses_declared_axis_not_raw_roll(self):
        self.arm()
        self.session.receive(
            "ORIENTATION_STATE protocol=2 feedback=AS5600 level_set=YES north_set=YES has_sample=YES fresh=YES heading=301.800 "
            "physical_pitch=-9.989 pitch_axis=PITCH roll=UNAVAILABLE", 2)
        rows = "\n".join(self.session.diagnostic_lines(2.1))
        self.assertIn("physical pitch/PITCH=-9.989 deg", rows)
        self.assertNotIn("physical pitch/ROLL=3.571 deg", rows)
        self.assertEqual(self.session.frame(2.1, 0, 100), b"JOG 0 100 0\n")

    def test_roll_is_unavailable_in_diagnostics(self):
        self.session.receive("ORIENTATION_STATE protocol=2 feedback=AS5600 heading=70 physical_pitch=8.000", 1)
        self.assertIn("roll=UNAVAILABLE", "\n".join(self.session.diagnostic_lines(1.1)))



if __name__ == "__main__":
    unittest.main()
