"""Photo sequencing uses only correlated, stopped firmware waypoints."""
import unittest

from auto_control import AutoSession, Keyframe
from stepped_playback import PhotoSettings, SteppedPlayback


class PhotoPlaybackTests(unittest.TestCase):
    def start(self, settings=PhotoSettings()):
        auto = AutoSession()
        auto.frames = {"A": Keyframe(42, (10, -20, 30), "0", "0"),
                       "B": Keyframe(42, (91, -103, 71), "0", "0")}
        auto.epoch = 42
        auto.photo_settings = settings
        self.assertEqual(auto.request("play_stepped", 0, True), b"STOP\n")
        auto.receive("M09 READY", .1)
        self.assertTrue(auto.frame(.2).startswith(b"SNAP "))
        auto.receive(f"KEYFRAME_SNAPSHOT id={auto.pending_id} epoch=42 yaw_steps=10 pitch_steps=-20 carriage_steps=30", .3)
        return auto

    def arrive(self, auto, now):
        command = auto.frame(now)
        self.assertTrue(command.startswith(b"KEYMOVE "))
        fields = command.decode().split()
        ident, epoch, yaw, pitch, carriage = fields[1:6]
        auto.receive(f"KEYMOVE ACCEPTED id={ident}", now + .1)
        self.assertEqual(auto.take_camera_events(), [])
        auto.receive(f"KEYMOVE RESULT id={ident} status=PASS epoch={epoch} yaw_steps={yaw} pitch_steps={pitch} carriage_steps={carriage}", now + .2)
        self.assertEqual(auto.phase, "PHOTO_READY_WAIT")
        self.assertIsNone(auto.frame(now + .21))
        self.assertEqual(auto.take_camera_events(), [])
        auto.receive("M09 READY", now + .3)
        return tuple(map(int, (yaw, pitch, carriage)))

    def test_eight_stops_settle_expose_pause_and_exact_b(self):
        auto = self.start()
        now = .4
        expected = [(20, -30, 35), (30, -41, 40), (40, -51, 45),
                    (51, -62, 51), (61, -72, 56), (71, -82, 61),
                    (81, -93, 66), (91, -103, 71)]
        for segment, target in enumerate(expected, 1):
            self.assertEqual(self.arrive(auto, now), target)
            self.assertIsNone(auto.frame(now + 1.29))
            self.assertEqual(auto.take_camera_events(), [])
            self.assertIsNone(auto.frame(now + 1.31))
            events = auto.take_camera_events()
            self.assertEqual(len(events), 1)
            self.assertEqual((events[0].segment, events[0].segments, events[0].epoch, events[0].steps),
                             (segment, 8, 42, target))
            self.assertIsNone(auto.frame(now + 1.8))
            self.assertEqual(auto.take_camera_events(), [])
            now += 1.82
        self.assertIsNone(auto.frame(now))
        self.assertEqual(auto.phase, "READY")
        self.assertFalse(auto.busy)

    def test_stop_abort_and_takeover_discard_pending_photos(self):
        for phase in ("KEYMOVE_PREPARED", "KEYMOVE_ACTIVE", "PHOTO_READY_WAIT", "PHOTO_SETTLE", "PHOTO_POST"):
            for reason in ("STOP", "joystick takeover", "abort"):
                with self.subTest(phase=phase, reason=reason):
                    auto = self.start()
                    auto.phase = phase
                    auto.settle_until = 1
                    if reason == "abort":
                        auto.receive("M09 ABORTED", .5)
                    else:
                        auto.cancel(.5, reason)
                    auto.receive("M09 READY", .6)
                    self.assertIsNone(auto.frame(2))
                    self.assertEqual(auto.take_camera_events(), [])
                    self.assertIsNone(auto.stepped)

    def test_wrong_endpoint_or_epoch_never_exposes(self):
        for suffix in ("epoch=42 yaw_steps=999 pitch_steps=-30 carriage_steps=35",
                       "epoch=43 yaw_steps=20 pitch_steps=-30 carriage_steps=35", ""):
            auto = self.start()
            auto.frame(.4)
            auto.receive(f"KEYMOVE RESULT id={auto.pending_id} status=PASS {suffix}", .5)
            auto.receive("M09 READY", .6)
            self.assertIsNone(auto.frame(2))
            self.assertEqual(auto.take_camera_events(), [])
            self.assertIsNone(auto.stepped)

    def test_telemetry_loss_cancels_during_photo_wait(self):
        auto = self.start(PhotoSettings(8, 10, 10))
        self.arrive(auto, .4)
        self.assertEqual(auto.frame(4), b"STOP\n")
        self.assertIsNone(auto.stepped)
        self.assertEqual(auto.take_camera_events(), [])

    def test_settings_limits_and_integer_endpoints(self):
        for args in ((0,), (65,), (1.5,), (True,), (8, -1), (8, float("nan")), (8, 1, 61)):
            with self.assertRaises(ValueError):
                PhotoSettings(*args)
        auto = self.start(PhotoSettings(64, 0, 0))
        plan = auto.stepped
        self.assertEqual(plan.segment_ms, 1000)
        targets = []
        for _ in range(64):
            plan.last_target = plan.target()
            targets.append(plan.last_target)
            plan.trigger(0)
        self.assertEqual(targets[-1], auto.frames["B"].steps)
        self.assertTrue(all(b[0] >= a[0] and b[1] <= a[1] for a, b in zip(targets, targets[1:])))


if __name__ == "__main__":
    unittest.main()
