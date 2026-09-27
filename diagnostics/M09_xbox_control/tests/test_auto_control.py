"""Offline request correlation, keyframe positions and cancellation tests."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from auto_control import AutoSession
from xbox_control import ManualSession, arguments


class AutoControlTests(unittest.TestCase):
    def setUp(self):
        self.auto = AutoSession()
        self.auto.enter(0)
        self.auto.receive("M09 READY", .1)

    def snapshot(self, request_id, steps=(10, -20, 30), epoch=42):
        return (f"KEYFRAME_SNAPSHOT id={request_id} epoch={epoch} yaw_steps={steps[0]} "
                f"pitch_steps={steps[1]} carriage_steps={steps[2]} heading=292.2 physical_pitch=-40.2 "
                "bno_valid=YES bno_age_ms=12 accuracy=1 north_usable=NO unhomed=YES")

    def capture(self, slot, steps=(10, -20, 30), epoch=42):
        command = self.auto.request("capture_" + slot.lower(), .2, True)
        self.assertTrue(command.startswith(b"SNAP "))
        self.auto.receive(self.snapshot(self.auto.pending_id, steps, epoch), .3)

    def test_enter_and_manual_arm_cannot_emit_jog(self):
        manual = ManualSession()
        manual.auto_mode = True
        manual.receive("M09 READY", 0)
        self.assertFalse(manual.arm(1, True))
        manual.state = "active"
        self.assertIsNone(manual.frame(1, 500, -500, 500))
        manual.disarm("stop", request_stop=True)
        self.assertEqual(manual.frame(2, 500, 500, 500), b"STOP\n")

    def test_capture_matches_id_and_uses_fresh_reply_not_cached_display(self):
        self.assertEqual(self.auto.request("capture_a", .2, True), b"SNAP 1\n")
        self.auto.receive(self.snapshot(99), .3)
        self.auto.receive("M09 READY", .4)
        self.assertFalse(self.auto.frames)
        self.assertEqual(self.auto.phase, "SNAPSHOT")
        self.auto.receive(self.snapshot(1), .5)
        self.assertEqual(self.auto.frames["A"].steps, (10, -20, 30))
        self.assertEqual(self.auto.frames["A"].pitch, "-40.2")
        self.assertEqual(self.auto.phase, "READY")

    def test_normal_mode_changes_and_stop_keep_captures(self):
        self.capture("A")
        self.auto.leave(1)
        self.auto.receive("MANUAL STOPPED: operator", 2)
        self.auto.enter(3)
        self.auto.receive("M09 READY", 3.1)
        self.assertIn("A", self.auto.frames)
        self.auto.cancel(4)
        self.auto.receive("M09 READY", 4.1)
        self.assertIn("A", self.auto.frames)

    def test_play_requires_both_captures_and_fresh_exact_position_at_a(self):
        self.assertIsNone(self.auto.request("play", 0, True))
        self.capture("A")
        self.assertIsNone(self.auto.request("play", 0, True))
        self.capture("B", (12, -18, 35))
        self.auto.request("play", .5, True)
        self.auto.receive(self.snapshot(self.auto.pending_id, (10, -20, 31)), .6)
        self.assertIn("not at A", self.auto.note)
        self.assertIsNone(self.auto.frame(.7))

    def test_return_and_play_emit_one_absolute_three_axis_command(self):
        self.capture("A")
        self.capture("B", (12, -18, 35))
        self.auto.duration = 20
        self.auto.request("return_a", .5, True)
        self.auto.receive(self.snapshot(self.auto.pending_id, (12, -18, 35)), .6)
        self.assertEqual(self.auto.frame(.7), b"KEYMOVE 4 42 10 -20 30 20000\n")
        self.assertIsNone(self.auto.frame(.8))
        self.auto.receive("M09 READY", .9)
        self.assertEqual(self.auto.phase, "KEYMOVE_PENDING")
        self.auto.receive("KEYMOVE ACCEPTED id=99 epoch=42 duration_ms=20000", 1)
        self.assertEqual(self.auto.phase, "KEYMOVE_PENDING")
        self.auto.receive("KEYMOVE ACCEPTED id=4 epoch=42 duration_ms=20000", 1.1)
        self.auto.receive("M09 READY", 1.2)
        self.assertEqual(self.auto.phase, "KEYMOVE_ACTIVE")
        self.auto.receive("KEYMOVE RESULT id=4 status=PASS epoch=42 yaw_steps=10 pitch_steps=-20 carriage_steps=30", 20)
        self.auto.receive("M09 READY", 20.1)
        self.assertIn("A", self.auto.frames)
        self.auto.request("play", 20.2, True)
        self.auto.receive(self.snapshot(self.auto.pending_id), 20.3)
        self.assertEqual(self.auto.frame(20.4), b"KEYMOVE 6 42 12 -18 35 20000\n")

    def test_stop_discards_prepared_move_and_late_snapshot(self):
        self.capture("A")
        self.auto.request("return_a", .5, True)
        ident = self.auto.pending_id
        self.auto.receive(self.snapshot(ident), .6)
        self.auto.cancel(.65)
        self.assertIsNone(self.auto.frame(.7))
        self.auto.receive(self.snapshot(ident), .8)
        self.assertIsNone(self.auto.frame(.9))
        self.assertIn("A", self.auto.frames)

    def test_busy_and_uncentered_requests_are_discarded_without_retry(self):
        self.assertIsNone(self.auto.request("pitch_up", .2, False))
        self.assertEqual(self.auto.request("pitch_up", .3, True), b"MOVE 0 1 0\n")
        self.assertIsNone(self.auto.request("pitch_down", .4, True))
        self.auto.receive("PITCH-ONLY ACCEPTED: yaw stationary", .5)
        self.auto.receive("FINAL RESULT: PASS", .6)
        self.auto.receive("M09 READY", .7)
        self.assertIsNone(self.auto.frame(.8))

    def test_all_dpad_signs_leave_carriage_unchanged(self):
        for action, command in (("pitch_up", b"MOVE 0 2 0\n"), ("pitch_down", b"MOVE 0 -2 0\n"),
                                ("yaw_right", b"MOVE 2 0 0\n"), ("yaw_left", b"MOVE -2 0 0\n")):
            self.auto.increment = 2
            self.assertEqual(self.auto.request(action, .2, True), command)
            self.auto.receive("POSE REJECTED: calibrated north required", .3)
            self.auto.receive("M09 READY", .4)

    def test_timeout_stops_once_clears_captures_never_retries(self):
        self.capture("A")
        self.auto.request("pitch_up", 1, True)
        self.auto.receive("M09 READY", 1.1)
        self.assertEqual(self.auto.frame(5.1), b"STOP\n")
        self.assertFalse(self.auto.frames)
        self.auto.receive("M09 READY", 5.2)
        self.assertEqual(self.auto.phase, "FAULT")
        self.assertIsNone(self.auto.frame(20))
        self.assertIsNone(self.auto.request("pitch_up", 21, True))

    def test_firmware_epoch_reset_abort_and_invalidated_clear_captures_even_off(self):
        for line in ("KEYFRAME_INVALIDATED epoch=43 reason=driver_failure", "M09_xbox_control: boot", "M09 ABORTED"):
            with self.subTest(line=line):
                self.setUp()
                self.capture("A")
                self.auto.leave(1)
                self.auto.receive(line, 2)
                self.assertFalse(self.auto.frames)
        self.setUp()
        self.capture("A")
        self.auto.request("capture_b", .5, True)
        self.auto.receive(self.snapshot(self.auto.pending_id, epoch=43), .6)
        self.assertFalse(self.auto.frames)

    def test_failed_playback_invalidates_but_stopped_playback_preserves(self):
        for status in ("FAILED", "STOPPED"):
            self.setUp()
            self.capture("A")
            self.auto.request("return_a", .5, True)
            self.auto.receive(self.snapshot(self.auto.pending_id), .6)
            self.auto.frame(.7)
            self.auto.receive(f"KEYMOVE RESULT id={self.auto.pending_id} status={status}", .8)
            self.auto.receive("M09 READY", .9)
            self.assertEqual(bool(self.auto.frames), status == "STOPPED")

    def test_malformed_snapshot_is_not_capture(self):
        self.auto.request("capture_a", .2, True)
        self.auto.receive(self.snapshot(1).replace("yaw_steps=10", "yaw_steps=nan"), .3)
        self.assertFalse(self.auto.frames)
        self.assertEqual(self.auto.phase, "SNAPSHOT")
        self.auto.receive(self.snapshot(1, epoch=0), .4)
        self.assertFalse(self.auto.frames)

    def test_latched_abort_stays_faulted_despite_status_polling(self):
        self.capture("A")
        self.auto.receive("M09 ABORTED", 1)
        self.assertEqual(self.auto.phase, "FAULT")
        self.assertFalse(self.auto.frames)
        self.auto.receive("M09 ABORTED", 2)
        self.assertIsNone(self.auto.frame(10))
        self.assertIsNone(self.auto.request("pitch_up", 11, True))

    def test_keyframe_telemetry_refreshes_current_carriage_display(self):
        manual = ManualSession()
        for index, line in enumerate((self.snapshot(1),
                                     "KEYMOVE_STATE id=2 carriage_steps=31 running_axes=3",
                                     "KEYMOVE RESULT id=2 status=PASS carriage_steps=32")):
            manual.receive(line, index + 1)
            self.assertEqual(manual.carriage_steps, str(30 + index))
            self.assertEqual(manual.carriage_received_at, index + 1)

    def test_button_configuration_does_not_replace_safety_buttons(self):
        defaults = arguments([])
        self.assertEqual((defaults.auto_button, defaults.capture_a_button, defaults.capture_b_button,
                          defaults.duration_button, defaults.play_button, defaults.return_a_button,
                          defaults.increment_button), (3, 4, 5, 2, 10, 8, 9))
        with self.assertRaises(SystemExit):
            arguments(["--auto-button", "0"])
        with self.assertRaises(SystemExit):
            arguments(["--auto-button", "4"])


if __name__ == "__main__":
    unittest.main()
