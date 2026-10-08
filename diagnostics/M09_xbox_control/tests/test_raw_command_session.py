"""Raw command protocol decisions only; no pygame, serial port, or hardware."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from xbox_control import ManualSession


class RawCommandSessionTests(unittest.TestCase):
    def setUp(self):
        self.session = ManualSession()

    def enter_ready(self):
        self.assertEqual(self.session.enter_raw(1.0), b"STOP\n")
        self.session.receive("M09 READY", 1.1)
        self.assertTrue(self.session.raw_mode)
        self.assertFalse(self.session.raw_waiting)

    def assert_submission_error(self):
        with self.assertRaises(ValueError) as caught:
            self.session.submit_raw()
        self.assertTrue(str(caught.exception).strip(), "Show a useful reason for refusing submission")

    def test_initial_state_has_no_invented_carriage_position_or_raw_command(self):
        self.assertFalse(self.session.raw_mode)
        self.assertFalse(self.session.raw_waiting)
        self.assertEqual(self.session.raw_text, "")
        self.assertEqual(self.session.last_raw_command, "")
        self.assertIsNone(self.session.carriage_steps)
        self.assertIsNone(self.session.carriage_received_at)
        self.assertIsNone(self.session.frame(1, 1000, -1000, 1000))

    def test_raw_entry_stops_active_manual_and_requires_new_ready(self):
        self.session.receive("M09 READY", 1)
        self.assertTrue(self.session.arm(1.1, True))
        self.session.receive("MANUAL READY: test", 1.2)
        self.assertEqual(self.session.state, "active")
        self.session.pending_stop = True
        self.assertEqual(self.session.enter_raw(1.3), b"STOP\n")
        self.assertTrue(self.session.raw_mode)
        self.assertTrue(self.session.raw_waiting)
        self.assertEqual(self.session.state, "idle")
        self.assertFalse(self.session.ready)
        self.assertFalse(self.session.pending_stop)
        self.session.raw_text = "POSE 20 8 123"
        self.assert_submission_error()
        self.assertEqual(self.session.raw_text, "POSE 20 8 123")
        for now in (1.4, 2.0, 10.0):
            self.assertIsNone(self.session.frame(now, 1000, -1000, 1000))
        self.session.receive("M09 BUSY", 10.1)
        self.assertTrue(self.session.raw_waiting)
        self.session.receive("MANUAL READY: late prior acknowledgment", 10.2)
        self.assertTrue(self.session.raw_waiting)
        self.assertNotEqual(self.session.state, "active")
        self.session.receive("M09 READY", 10.3)
        self.assertFalse(self.session.raw_waiting)
        self.assertEqual(self.session.submit_raw(), b"POSE 20 8 123\n")

    def test_ready_while_raw_never_arms_or_emits_stick_commands(self):
        self.enter_ready()
        self.assertFalse(self.session.arm(1.2, True))
        self.assertFalse(self.session.arm(1.2, False))
        for now in (1.2, 2.0, 10.0):
            self.assertIsNone(self.session.frame(now, 1000, -1000, 1000))
        self.session.receive("MANUAL READY: unsolicited", 10.1)
        self.assertNotEqual(self.session.state, "active")
        self.assertIsNone(self.session.frame(10.2, 1000, -1000, 1000))

    def test_raw_submission_preserves_every_entered_byte_and_adds_one_lf(self):
        self.enter_ready()
        text = "  POSE\t20.125  -9.989    -2147483648  "
        self.session.raw_text = text
        self.assertEqual(self.session.submit_raw(), text.encode("utf-8") + b"\n")
        self.assertEqual(self.session.last_raw_command, text)
        self.assertEqual(self.session.raw_text, "")
        self.assertFalse(self.session.ready)
        self.assertFalse(self.session.raw_waiting)
        self.assertEqual(self.session.state, "idle")
        self.assertIsNone(self.session.frame(1.2, 500, 500, 500))

    def test_unknown_malformed_jog_long_and_unicode_commands_reach_firmware_unchanged(self):
        self.enter_ready()
        commands = (
            "UNKNOWN command 123",
            "POSE bad-angle 99999 not-a-count",
            "JOG 1000 -1000 1000",
            "JOG not even remotely valid",
            "POSE " + "1" * 300 + " 2 3",
            "diagnostic café 月 🌙",
        )
        for text in commands:
            with self.subTest(text=text):
                self.session.raw_text = text
                self.assertEqual(self.session.submit_raw(), text.encode("utf-8") + b"\n")
                self.assertEqual(self.session.last_raw_command, text)
                self.assertEqual(self.session.state, "idle")
                self.assertIsNone(self.session.frame(2, 1000, 1000, 1000))

    def test_empty_input_is_rejected_without_discarding_edit_buffer(self):
        self.enter_ready()
        for text in ("", "   ", "\t \t", "\u2003"):
            with self.subTest(text=text):
                self.session.raw_text = text
                self.assert_submission_error()
                self.assertEqual(self.session.raw_text, text)
                self.assertEqual(self.session.last_raw_command, "")

    def test_submission_requires_raw_mode(self):
        self.session.receive("M09 READY", 1)
        self.session.raw_text = "STATUS"
        self.assert_submission_error()
        self.assertEqual(self.session.raw_text, "STATUS")
        self.assertEqual(self.session.last_raw_command, "")

    def test_busy_firmware_does_not_defer_or_lose_explicit_subsequent_raw_submission(self):
        self.enter_ready()
        self.session.raw_text = "POSE 21 8 50"
        self.assertEqual(self.session.submit_raw(), b"POSE 21 8 50\n")
        self.session.receive("POSE ACCEPTED yaw_deg=21 pitch_deg=8 carriage_steps=50", 1.2)
        self.session.receive("M09 BUSY", 1.3)
        self.session.raw_text = "POSE 22 8 50"
        self.assertEqual(self.session.submit_raw(), b"POSE 22 8 50\n")
        self.session.receive("POSE REJECTED: command requires READY", 1.4)
        self.assertEqual(self.session.last_raw_command, "POSE 22 8 50")
        for now in (1.5, 2, 10):
            self.assertIsNone(self.session.frame(now, 1000, 1000, 1000))
        self.session.receive("M09 READY", 10.1)
        self.assertIsNone(self.session.frame(10.2, 1000, 1000, 1000))
        self.assertEqual(self.session.raw_text, "")

    def test_pending_stop_can_be_sent_in_raw_mode_but_never_followed_by_jog(self):
        self.enter_ready()
        self.session.disarm("test stop request", request_stop=True)
        self.assertEqual(self.session.frame(1.2, 1000, -1000, 1000), b"STOP\n")
        self.assertFalse(self.session.pending_stop)
        self.assertIsNone(self.session.frame(1.3, 1000, -1000, 1000))

    def test_ready_received_before_pending_stop_cannot_allow_raw_submission_after_stop(self):
        self.enter_ready()
        self.session.disarm("Window lost focus", request_stop=True)
        self.session.raw_waiting = True
        # The UI processes buffered RX before frame() transmits a queued STOP.
        self.session.receive("M09 READY", 1.2)
        self.assertFalse(self.session.raw_waiting)
        self.assertEqual(self.session.frame(1.3, 1000, -1000, 1000), b"STOP\n")
        self.assertTrue(self.session.raw_waiting)
        self.assertFalse(self.session.ready)
        self.session.raw_text = "POSE 21 8 123"
        self.assert_submission_error()
        self.assertEqual(self.session.raw_text, "POSE 21 8 123")
        self.assertIsNone(self.session.frame(1.4, 1000, -1000, 1000))
        self.session.receive("M09 READY", 1.5)
        self.assertFalse(self.session.raw_waiting)
        self.assertEqual(self.session.submit_raw(), b"POSE 21 8 123\n")

    def test_malformed_raw_jog_rejection_does_not_inject_stop_or_resubmit_input(self):
        self.enter_ready()
        self.session.raw_text = "JOG malformed"
        self.assertEqual(self.session.submit_raw(), b"JOG malformed\n")
        self.session.receive("MANUAL REJECTED: use JOG integer_yaw integer_pitch", 1.2)
        self.assertFalse(self.session.pending_stop)
        self.assertTrue(self.session.raw_mode)
        self.assertFalse(self.session.raw_waiting)
        self.assertEqual(self.session.last_raw_command, "JOG malformed")
        self.assertIsNone(self.session.frame(1.3, 1000, -1000, 1000))
        self.session.raw_text = "STATUS"
        self.assertEqual(self.session.submit_raw(), b"STATUS\n")

    def test_raw_stop_keeps_mode_and_requires_new_ready_without_automatic_resubmission(self):
        self.enter_ready()
        self.session.raw_text = "POSE 21 8 123"
        self.assertEqual(self.session.submit_raw(), b"POSE 21 8 123\n")
        self.assertEqual(self.session.stop_raw(1.2), b"STOP\n")
        self.assertTrue(self.session.raw_mode)
        self.assertTrue(self.session.raw_waiting)
        self.assertFalse(self.session.ready)
        self.assertEqual(self.session.state, "idle")
        self.session.raw_text = "STATUS"
        self.assert_submission_error()
        self.assertIsNone(self.session.frame(1.3, 1000, -1000, 1000))
        self.session.receive("M09 READY", 1.4)
        self.assertFalse(self.session.raw_waiting)
        self.assertIsNone(self.session.frame(1.5, 1000, -1000, 1000))
        self.assertEqual(self.session.submit_raw(), b"STATUS\n")

    def test_leaving_raw_stops_and_requires_deliberate_manual_rearm(self):
        self.enter_ready()
        self.session.raw_text = "POSE 21 8 123"
        self.session.pending_stop = True
        self.assertEqual(self.session.leave_raw(1.2), b"STOP\n")
        self.assertFalse(self.session.raw_mode)
        self.assertEqual(self.session.raw_text, "")
        self.assertEqual(self.session.state, "idle")
        self.assertFalse(self.session.ready)
        self.assertFalse(self.session.pending_stop)
        self.assertFalse(self.session.arm(1.3, True))
        self.assertIsNone(self.session.frame(1.3, 1000, -1000, 1000))
        self.session.receive("M09 READY", 1.4)
        self.assertIsNone(self.session.frame(1.5, 1000, -1000, 1000))
        self.assertFalse(self.session.arm(1.5, False))
        self.assertTrue(self.session.arm(1.6, True))
        self.assertEqual(self.session.frame(1.7, 1000, -1000, 1000), b"JOG 0 0 0\n")
        self.session.receive("MANUAL READY: test", 1.8)
        self.assertEqual(self.session.frame(1.9, 120, -40, 50), b"JOG 120 -40 50\n")

    def test_firmware_restart_clears_stale_carriage_and_releases_raw_wait_only_after_ready(self):
        self.enter_ready()
        self.session.receive("POSE_STATE active=NO carriage_steps=-321", 1.2)
        self.session.receive("M09_xbox_control: firmware restart", 1.3)
        self.assertTrue(self.session.raw_mode)
        self.assertTrue(self.session.raw_waiting)
        self.assertIsNone(self.session.carriage_steps)
        self.assertIsNone(self.session.carriage_received_at)
        self.assertFalse(self.session.ready)
        self.session.raw_text = "STATUS"
        self.assert_submission_error()
        self.assertIsNone(self.session.frame(1.4, 1000, -1000, 1000))
        self.session.receive("M09 READY", 1.5)
        self.assertFalse(self.session.raw_waiting)
        self.assertIsNone(self.session.frame(1.6, 1000, -1000, 1000))
        self.assertEqual(self.session.submit_raw(), b"STATUS\n")


class CarriageReceiptTests(unittest.TestCase):
    def setUp(self):
        self.session = ManualSession()

    def test_missing_report_is_unavailable_without_zero_position(self):
        rows = "\n".join(self.session.diagnostic_lines(2))
        carriage_rows = [row for row in rows.splitlines() if "carriage" in row.lower()]
        self.assertTrue(carriage_rows)
        self.assertTrue(any("no report" in row.lower() for row in carriage_rows))
        self.assertFalse(any("steps=0" in row or "steps: 0" in row for row in carriage_rows))
        self.assertIsNone(self.session.carriage_steps)

    def test_manual_and_pose_reports_keep_exact_integer_count_and_receipt_time(self):
        self.session.receive("MANUAL_STATE yaw_cmd=0 pitch_cmd=0 carriage_steps=-2147483648", 2.0)
        self.assertEqual(self.session.carriage_steps, "-2147483648")
        self.assertEqual(self.session.carriage_received_at, 2.0)
        rows = "\n".join(self.session.diagnostic_lines(3.5))
        self.assertIn("-2147483648", rows)
        self.assertIn("received 1.5 s ago", rows)
        self.assertIn("unhomed", rows.lower())
        self.session.receive("POSE_STATE active=YES carriage_steps=2147483647 target_steps=1", 4.0)
        self.assertEqual(self.session.carriage_steps, "2147483647")
        self.assertEqual(self.session.carriage_received_at, 4.0)
        rows = "\n".join(self.session.diagnostic_lines(4.2))
        self.assertIn("2147483647", rows)
        self.assertIn("received 0.2 s ago", rows)

    def test_other_reports_do_not_invent_or_refresh_carriage_count(self):
        self.session.receive("POSE_STATE active=NO carriage_steps=42", 1.0)
        for line in ("STATE MANUAL heading=10", "M09 READY", "ORIENTATION_STATE protocol=2 feedback=AS5600 level_set=YES north_set=YES carriage_steps=999",
                     "POSE ACCEPTED carriage_steps=900", "MANUAL_STATE carriage_cmd=500",
                     "POSE_STATE carriage_steps=not-an-integer", "POSE_STATE carriage_steps=2.5"):
            with self.subTest(line=line):
                self.session.receive(line, 5.0)
                self.assertEqual(self.session.carriage_steps, "42")
                self.assertEqual(self.session.carriage_received_at, 1.0)
        rows = "\n".join(self.session.diagnostic_lines(5.5))
        self.assertIn("received 4.5 s ago", rows)


if __name__ == "__main__":
    unittest.main()
