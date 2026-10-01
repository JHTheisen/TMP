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

    def request(self, action, now, centered):
        command = self.auto.request(action, now, centered)
        if command == b"STOP\n":
            self.auto.receive("M09 READY", now + .01)
            return self.auto.frame(now + .02)
        return command

    def capture(self, slot, steps=(10, -20, 30), epoch=42):
        command = self.request("capture_" + slot.lower(), .2, True)
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
        self.assertEqual(self.request("capture_a", .2, True), b"SNAP 1\n")
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

    def test_play_from_arbitrary_position_returns_then_verifies_a_and_uses_full_duration(self):
        self.assertIsNone(self.request("play", 0, True))
        self.capture("A")
        self.assertIsNone(self.request("play", 0, True))
        self.capture("B", (12, -18, 35))
        self.auto.duration = 20
        self.request("play", .5, True)
        self.auto.receive(self.snapshot(self.auto.pending_id, (10, -20, 31)), .6)
        self.assertEqual(self.auto.frame(.7), b"KEYRETURN 4 42 10 -20 30\n")
        self.auto.receive("KEYMOVE ACCEPTED id=4", .8)
        self.auto.receive("KEYMOVE RESULT id=4 status=PASS", 2)
        self.auto.receive("M09 READY", 2.01)
        self.assertIsNone(self.auto.frame(2.1))
        self.assertEqual(self.auto.frame(2.3), b"SNAP 5\n")
        self.auto.receive(self.snapshot(5), 2.4)
        self.assertEqual(self.auto.frame(2.5), b"KEYMOVE 6 42 12 -18 35 20000\n")

    def test_return_and_play_emit_one_absolute_three_axis_command(self):
        self.capture("A")
        self.capture("B", (12, -18, 35))
        self.auto.duration = 20
        self.request("return_a", .5, True)
        self.auto.receive(self.snapshot(self.auto.pending_id, (12, -18, 35)), .6)
        self.assertEqual(self.auto.frame(.7), b"KEYRETURN 4 42 10 -20 30\n")
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
        self.request("play", 20.2, True)
        self.auto.receive(self.snapshot(self.auto.pending_id), 20.3)
        self.assertEqual(self.auto.frame(20.4), b"KEYMOVE 6 42 12 -18 35 20000\n")

    def test_stop_discards_prepared_move_and_late_snapshot(self):
        self.capture("A")
        self.request("return_a", .5, True)
        ident = self.auto.pending_id
        self.auto.receive(self.snapshot(ident), .6)
        self.auto.cancel(.65)
        self.assertIsNone(self.auto.frame(.7))
        self.auto.receive(self.snapshot(ident), .8)
        self.assertIsNone(self.auto.frame(.9))
        self.assertIn("A", self.auto.frames)

    def test_busy_and_uncentered_requests_are_discarded_without_retry(self):
        self.assertIsNone(self.request("pitch_up", .2, False))
        self.assertEqual(self.request("pitch_up", .3, True), b"MOVE 0 1 0\n")
        self.assertIsNone(self.request("pitch_down", .4, True))
        self.auto.receive("PITCH-ONLY ACCEPTED: yaw stationary", .5)
        self.auto.receive("FINAL RESULT: PASS", .6)
        self.auto.receive("M09 READY", .7)
        self.assertIsNone(self.auto.frame(.8))

    def test_successful_positioning_matches_keyframe_manual_ready_state(self):
        def manual_reentry():
            manual = ManualSession()
            manual.receive("M09 READY", 1.4)
            self.assertTrue(manual.arm(1.9, True))
            handshake = manual.frame(1.92, 250, 0, 0)
            manual.receive("MANUAL READY: test", 1.94)
            live = manual.frame(1.96, 250, 0, 0)
            return manual.state, handshake, live

        keyframe = AutoSession()
        keyframe.enter(0)
        keyframe.receive("M09 READY", .1)
        keyframe.action = "raw"
        keyframe.phase = "KEYMOVE_ACTIVE"
        keyframe.pending_id = 1
        keyframe.receive("KEYMOVE RESULT id=1 status=PASS", 1.3)
        keyframe.receive("M09 READY", 1.4)
        expected = (keyframe.phase, keyframe.busy)
        expected_manual = manual_reentry()

        for command, accepted, result in (
                ("POSE", "POSE ACCEPTED yaw_deg=1", "FINAL RESULT: PASS"),
                ("LEVEL", "LEVEL ACCEPTED error_deg=-20.0", "LEVEL RESULT: PASS"),
                ("NORTH", "NORTH ACCEPTED error_deg=20.0", "NORTH RESULT: PASS")):
            with self.subTest(command=command):
                auto = AutoSession()
                auto.enter(0)
                auto.receive("M09 READY", .1)
                if command == "POSE":
                    auto.raw_sent(command, .2)
                else:
                    self.assertEqual(auto.request(command.lower(), .2, True), b"STOP\n")
                    auto.receive("M09 READY", .25)
                    self.assertEqual(auto.frame(.3), (command + "\n").encode())
                auto.receive(accepted, .3)
                auto.receive(result, 1.3)
                auto.receive("M09 READY", 1.4)
                self.assertEqual((auto.phase, auto.busy), expected)
                self.assertIsNone(auto.frame(1.5))
                self.assertEqual(manual_reentry(), expected_manual)

    def test_active_positioning_remains_overridable_for_joystick_takeover(self):
        for command, accepted in (
                ("POSE", "POSE ACCEPTED yaw_deg=1"),
                ("LEVEL", "LEVEL ACCEPTED error_deg=-20.0"),
                ("NORTH", "NORTH ACCEPTED error_deg=20.0")):
            with self.subTest(command=command):
                auto = AutoSession()
                auto.enter(0)
                auto.receive("M09 READY", .1)
                if command == "POSE":
                    auto.raw_sent(command, .2)
                else:
                    auto.request(command.lower(), .2, True)
                auto.receive(accepted, .3)
                self.assertTrue(auto.overridable)
                auto.cancel(.4, "Joystick takeover")
                self.assertEqual(auto.phase, "STOPPING")

    def test_interrupted_positioning_does_not_enable_automatic_manual_takeover(self):
        for command, accepted, result in (
                ("POSE", "POSE ACCEPTED yaw_deg=1", "FINAL RESULT: STOPPED"),
                ("LEVEL", "LEVEL ACCEPTED error_deg=-20.0", "LEVEL RESULT: STOPPED"),
                ("NORTH", "NORTH ACCEPTED error_deg=20.0", "NORTH RESULT: STOPPED")):
            with self.subTest(command=command):
                auto = AutoSession()
                auto.enter(0)
                auto.receive("M09 READY", .1)
                if command == "POSE":
                    auto.raw_sent(command, .2)
                else:
                    auto.request(command.lower(), .2, True)
                auto.receive(accepted, .3)
                auto.receive(result, 1.3)
                auto.receive("M09 READY", 1.4)
                self.assertEqual((auto.phase, auto.busy), ("READY", False))

    def test_all_dpad_signs_leave_carriage_unchanged(self):
        for action, command in (("pitch_up", b"MOVE 0 2 0\n"), ("pitch_down", b"MOVE 0 -2 0\n"),
                                ("yaw_right", b"MOVE 2 0 0\n"), ("yaw_left", b"MOVE -2 0 0\n")):
            self.auto.increment = 2
            self.assertEqual(self.request(action, .2, True), command)
            self.auto.receive("POSE REJECTED: calibrated north required", .3)
            self.auto.receive("M09 READY", .4)

    def test_timeout_stops_once_clears_captures_never_retries(self):
        self.capture("A")
        self.request("pitch_up", 1, True)
        self.auto.receive("M09 READY", 1.1)
        self.assertEqual(self.auto.frame(5.1), b"STOP\n")
        self.assertFalse(self.auto.frames)
        self.auto.receive("M09 READY", 5.2)
        self.assertEqual(self.auto.phase, "FAULT")
        self.assertIsNone(self.auto.frame(20))
        self.assertIsNone(self.request("pitch_up", 21, True))

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
        self.request("capture_b", .5, True)
        self.auto.receive(self.snapshot(self.auto.pending_id, epoch=43), .6)
        self.assertFalse(self.auto.frames)

    def test_failed_playback_invalidates_but_stopped_playback_preserves(self):
        for status in ("FAILED", "STOPPED"):
            self.setUp()
            self.capture("A")
            self.request("return_a", .5, True)
            self.auto.receive(self.snapshot(self.auto.pending_id), .6)
            self.auto.frame(.7)
            self.auto.receive(f"KEYMOVE RESULT id={self.auto.pending_id} status={status}", .8)
            self.auto.receive("M09 READY", .9)
            self.assertEqual(bool(self.auto.frames), status == "STOPPED")

    def test_malformed_snapshot_is_not_capture(self):
        self.request("capture_a", .2, True)
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
        self.assertIsNone(self.request("pitch_up", 11, True))

    def test_bno_metadata_changes_never_clear_captures(self):
        self.capture("A")
        self.capture("B")
        for status in ("BNO_STATE fresh=NO accuracy=0 north_usable=NO has_sample=YES",
                       "BNO WARNING: sensor reset; orientation unavailable",
                       "BNO_STATE available=NO has_sample=NO"):
            self.auto.receive(status, 1)
        self.assertEqual(len(self.auto.frames), 2)

    def test_bad_return_result_or_wrong_stopped_a_never_starts_play(self):
        for result in ("STOPPED", "FAILED", "PASS"):
            self.setUp()
            self.capture("A")
            self.capture("B", (99, -90, 100))
            self.request("play", .5, True)
            self.auto.receive(self.snapshot(self.auto.pending_id, (9, -20, 30)), .6)
            self.auto.frame(.7)
            ident = self.auto.pending_id
            self.auto.receive(f"KEYMOVE RESULT id={ident} status={result}", 1)
            self.auto.receive("M09 READY", 1.01)
            command = self.auto.frame(1.3)
            if result == "PASS":
                self.assertTrue(command.startswith(b"SNAP"))
                self.auto.receive(self.snapshot(self.auto.pending_id, (9, -20, 30)), 1.4)
            else:
                self.assertIsNone(command)
            self.assertIsNone(self.auto.frame(1.5))

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
        self.assertEqual((defaults.capture_a_button, defaults.capture_b_button,
                          defaults.duration_button, defaults.play_button, defaults.return_a_button,
                          defaults.increment_button), (4, 5, 2, 10, 8, 9))
        with self.assertRaises(SystemExit):
            arguments(["--capture-a-button", "0"])
        with self.assertRaises(SystemExit):
            arguments(["--capture-a-button", "5"])
        self.assertEqual((defaults.level_button, defaults.north_button), (0, 3))
        with self.assertRaises(SystemExit):
            arguments(["--level-button", "2"])
        with self.assertRaises(SystemExit):
            arguments(["--north-button", "1"])

    def test_alignment_uses_shared_stop_handoff_and_dynamic_firmware_deadline(self):
        for name in ("LEVEL", "NORTH"):
            with self.subTest(name=name):
                self.setUp()
                self.capture("A")
                self.assertEqual(self.request(name.lower(),1,True),(name+"\n").encode())
                self.auto.receive(name+" ACCEPTED deadline_ms=90000",1.1)
                self.assertTrue(self.auto.overridable)
                self.auto.receive(name+" DEADLINE deadline_ms=200000",2)
                self.assertAlmostEqual(self.auto.deadline,206.1) # Relative to admission, not update receipt.
                self.auto.receive(name+" PAUSED: BNO stale; recovery grace_ms=1500",100)
                self.assertIsNone(self.auto.frame(100.1))
                self.auto.receive(name+" RESUMED deadline_ms=201100",101.1)
                self.assertAlmostEqual(self.auto.deadline,207.2)
                self.auto.receive(name+" RESULT: PASS",102)
                self.auto.receive("M09 READY",102.01)
                self.assertFalse(self.auto.busy)
                self.assertIn("A",self.auto.frames)
                self.assertEqual(self.request("capture_b",103,False),b"SNAP 3\n")

    def test_alignment_rejection_and_failure_are_recoverable(self):
        for terminal in ("REJECTED: BNO unavailable", "RESULT: STOPPED"):
            for name in ("LEVEL","NORTH"):
                self.setUp()
                self.capture("A")
                self.request(name.lower(),1,True)
                self.auto.receive(name+" "+terminal,1.1)
                self.auto.receive("M09 READY",1.2)
                self.assertFalse(self.auto.busy)
                self.assertIn("A",self.auto.frames)
                self.assertEqual(self.request("pitch_up",2,True),b"MOVE 0 1 0\n")

    def test_alignment_watchdog_epoch_loss_clears_frames_without_canceling_recovery(self):
        self.capture("A")
        self.request("level",1,True)
        self.auto.receive("LEVEL ACCEPTED deadline_ms=90000",1.1)
        self.auto.receive("KEYFRAME_INVALIDATED epoch=43 reason=forced_stop",1.2)
        self.assertFalse(self.auto.frames)
        self.assertEqual(self.auto.phase,"MOVE_ACTIVE")
        self.auto.receive("LEVEL RESUMED deadline_ms=91000",2)
        self.assertIsNone(self.auto.frame(2.1))


if __name__ == "__main__":
    unittest.main()
