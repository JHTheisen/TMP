"""Celestial lifecycle/leases with deterministic clocks and bounded mailboxes."""
from datetime import datetime, timezone
from pathlib import Path
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from auto_control import AutoSession
from celestial_control import Calculation, CoordinateWorker, parse_tracking_command
from celestial_coordinates import EquatorialTarget, Observer, HeadingReference, HorizontalTarget
from xbox_control import ManualSession


UTC = datetime(2026, 10, 1, tzinfo=timezone.utc)


class FakeWorker:
    def __init__(self):
        self.jobs, self.captures, self.results = [], [], []
        self.closed = False

    def submit(self, generation, target, now):
        self.jobs.append((generation, target, now))

    def capture(self, generation, horizontal, now):
        self.captures.append((generation, horizontal, now))

    def poll(self):
        return self.results.pop(0) if self.results else None

    def close(self):
        self.closed = True


class CelestialControlTests(unittest.TestCase):
    def setUp(self):
        self.auto = AutoSession()
        self.auto.receive("ORIENTATION_STATE protocol=2 feedback=AS5600 available=YES fresh=YES north_set=YES level_set=YES age_ms=10 heading=0 physical_pitch=0", 0)
        self.reference = HeadingReference(magnetic_declination_deg=5, north_reference="magnetic")
        self.auto.configure_celestial(Observer(42, -83, 200), self.reference)
        self.worker = FakeWorker()
        self.auto.celestial.worker = self.worker

    def start(self):
        self.assertEqual(self.auto.request_celestial("TRACK_RADEC 18:36:56.3 +38:47:01", 0, True), b"STOP\n")
        self.assertTrue(self.auto.overridable)

    def result(self, az=100, alt=30, error="", job=None):
        generation, _, now = job or self.worker.jobs[-1]
        horizontal = HorizontalTarget(az, alt)
        mount = self.reference.mount_target(horizontal) if not error else None
        self.worker.results.append(Calculation(generation, now, UTC, horizontal, mount, error))

    def goto(self):
        self.start()
        self.auto.receive("M09 READY", .01)
        self.result()
        self.assertEqual(self.auto.frame(.02), b"CELESTIAL_GOTO 1 95.000000 30.000000\n")
        self.auto.receive("CELESTIAL_ACCEPTED id=1 deadline_ms=90000", .03)
        self.assertEqual(self.auto.phase, "CELESTIAL_GOTO")

    def track(self):
        self.goto()
        self.auto.receive("CELESTIAL_TRACK id=1", .2)
        self.assertEqual(self.auto.phase, "CELESTIAL_TRACK")

    def test_not_configured_malformed_busy_and_uncentered_do_not_start(self):
        for command in ("TRACK_RADEC 1", "TRACK_RADEC nan 0", "TRACK_RADEC 1 -91", "POSE 1 2 3"):
            with self.subTest(command=command), self.assertRaises(ValueError):
                self.auto.request_celestial(command, 0, True)
        with self.assertRaises(ValueError):
            self.auto.request_celestial("TRACK_RADEC 1 -30", 0, False)
        self.assertEqual(self.auto.phase, "READY")
        self.assertFalse(self.worker.jobs)
        self.start()
        with self.assertRaises(ValueError):
            self.auto.request_celestial("TRACK_RADEC 1 -30", 0, True)
        missing = AutoSession()
        with self.assertRaisesRegex(ValueError, "latitude"):
            missing.request_celestial("TRACK_RADEC 1 2", 0, True)

    def test_dry_run_is_local_does_not_compute_or_send(self):
        self.auto.phase = "STOPPING"
        self.assertIsNone(self.auto.request_celestial("TRACK_RADEC 1 -30", 0, True, dry_run=True))
        self.assertFalse(self.worker.jobs)
        self.assertIsNone(self.auto.action)

    def test_track_here_captures_mount_pointing_then_uses_existing_tracker(self):
        fields = {"available": "YES", "has_sample": "YES", "fresh": "YES", "protocol": "2", "feedback": "AS5600", "level_set": "YES", "north_set": "YES",
                  "pitch_axis": "PITCH", "age_ms": "10", "heading": "100", "physical_pitch": "30"}
        self.assertEqual(self.auto.request_track_here(1, True, fields, .95), b"STOP\n")
        self.assertEqual(len(self.worker.captures), 1)
        generation, horizontal, requested_at = self.worker.captures[0]
        self.assertEqual((horizontal.azimuth_deg, horizontal.altitude_deg), (105, 30))
        target = EquatorialTarget(5.25, 22.5)
        mount = self.reference.mount_target(horizontal)
        self.worker.results.append(Calculation(
            generation, requested_at, UTC, horizontal, mount, target=target))
        self.auto.receive("M09 READY", 1.01)
        self.assertEqual(self.auto.frame(1.02), b"CELESTIAL_GOTO 1 100.000000 30.000000\n")
        self.assertIn("TRACK HERE | RA=5.250000 h Dec=+22.500000 deg", self.auto.celestial_status)
        self.auto.receive("CELESTIAL_ACCEPTED id=1 deadline_ms=90000", 1.03)
        self.auto.receive("CELESTIAL_TRACK id=1", 1.1)
        self.assertEqual(self.auto.phase, "CELESTIAL_TRACK")

    def test_here_and_radec_have_identical_steady_state_updates(self):
        streams = []
        fields = {"available": "YES", "has_sample": "YES", "fresh": "YES", "protocol": "2", "feedback": "AS5600", "level_set": "YES", "north_set": "YES",
                  "pitch_axis": "PITCH", "age_ms": "10", "heading": "100", "physical_pitch": "30"}
        target = EquatorialTarget(5.25, 22.5)
        for here in (True, False):
            auto = AutoSession()
            auto.receive("ORIENTATION_STATE protocol=2 feedback=AS5600 available=YES fresh=YES north_set=YES level_set=YES age_ms=10 heading=0 physical_pitch=0", 1)
            auto.configure_celestial(Observer(42, -83, 200), self.reference)
            worker = FakeWorker()
            auto.celestial.worker = worker
            if here:
                auto.request_track_here(1, True, fields, .95)
            else:
                auto.request_celestial("TRACK_RADEC 5.25 22.5", 1, True)
            generation, _, requested_at = (worker.captures if here else worker.jobs)[0]
            horizontal = HorizontalTarget(105, 30, UTC)
            worker.results.append(Calculation(generation, requested_at, UTC, horizontal,
                self.reference.mount_target(horizontal), target=target if here else None))
            auto.receive("M09 READY", 1.01)
            stream = [auto.frame(1.02)]
            auto.receive("CELESTIAL_ACCEPTED id=1 deadline_ms=90000", 1.03)
            auto.receive("CELESTIAL_TRACK id=1", 1.1)
            self.assertIsNone(auto.deadline)
            for second in range(2, 65):
                now = second * 1.1
                auto.receive("CELESTIAL_STATE id=1 mode=TRACK feedback=AS5600 encoder=DEGRADED", now)
                auto.frame(now)
                generation, job_target, requested_at = worker.jobs[-1]
                self.assertEqual(job_target, target)
                horizontal = HorizontalTarget(105 + second * .004, 30 + second * .002, UTC)
                worker.results.append(Calculation(generation, requested_at, UTC, horizontal,
                    self.reference.mount_target(horizontal)))
                stream.append(auto.frame(now + .01))
                self.assertEqual(auto.phase, "CELESTIAL_TRACK")
            streams.append(stream)
            auto.close()
        self.assertEqual(streams[0], streams[1])
        self.assertTrue(all(command.startswith(b"CELESTIAL_UPDATE ") for command in streams[0][1:]))

    def test_track_here_rejects_missing_alignment_without_affecting_manual(self):
        valid = {"available": "YES", "has_sample": "YES", "fresh": "YES", "protocol": "2", "feedback": "AS5600", "level_set": "YES", "north_set": "YES",
                 "pitch_axis": "PITCH", "age_ms": "10", "heading": "100", "physical_pitch": "30"}
        cases = [(None, None), ({**valid, "north_set": "NO"}, .9),
                 ({**valid, "fresh": "NO"}, .9), ({**valid, "heading": "nan"}, .9),
                 ({**valid, "age_ms": "3000"}, .9)]
        for fields, received_at in cases:
            with self.subTest(fields=fields), self.assertRaisesRegex(ValueError, "TRACK HERE"):
                self.auto.request_track_here(1, True, fields, received_at)
            self.assertEqual(self.auto.phase, "READY")
            self.assertFalse(self.worker.captures)
        manual = ManualSession()
        manual.receive("M09 READY", 1)
        self.assertTrue(manual.arm(1.1, True))
        self.assertEqual(manual.frame(1.1, 0, 0, 0), b"JOG 0 0 0\n")
        manual.receive("MANUAL READY: fixture", 1.11)
        self.assertEqual(manual.frame(1.12, 100, -50, 25), b"JOG 100 -50 25\n")

    def test_track_here_worker_start_failure_is_local(self):
        fields = {"available": "YES", "has_sample": "YES", "fresh": "YES", "protocol": "2", "feedback": "AS5600", "level_set": "YES", "north_set": "YES",
                  "pitch_axis": "PITCH", "age_ms": "10", "heading": "100", "physical_pitch": "30"}
        self.worker.capture = lambda *args: (_ for _ in ()).throw(RuntimeError("worker unavailable"))
        with self.assertRaisesRegex(ValueError, "worker unavailable"):
            self.auto.request_track_here(1, True, fields, .99)
        self.assertEqual(self.auto.phase, "READY")
        self.assertIsNone(self.auto.action)
        self.assertIsNone(self.auto.celestial.target)

    def test_initial_result_waits_for_ready_and_blocks_manual(self):
        self.start()
        self.result()
        self.assertIsNone(self.auto.frame(.1))
        manual = ManualSession()
        manual.auto_mode = self.auto.busy
        manual.receive("M09 READY", .1)
        self.assertFalse(manual.arm(.2, True))
        self.assertIsNone(manual.frame(.2, 1000, 1000, 1000))
        self.auto.receive("M09 READY", .2)
        self.assertTrue(self.auto.frame(.3).startswith(b"CELESTIAL_GOTO "))

    def test_worker_failure_is_local_stop_and_manual_recovers_without_encoder(self):
        for reason in ("missing Astropy", "below horizon", "pitch guard", "invalid UTC"):
            with self.subTest(reason=reason):
                self.setUp()
                self.start()
                self.result(error=reason)
                self.assertEqual(self.auto.frame(.1), b"STOP\n")
                self.assertIn(reason, self.auto.note)
                self.auto.receive("M09 READY", .2)
                self.assertFalse(self.auto.busy)
                manual = ManualSession()
                manual.receive("ORIENTATION_STATE protocol=2 feedback=AS5600 level_set=YES north_set=YES available=NO fresh=NO accuracy=0", .2)
                manual.receive("M09 READY", .2)
                self.assertTrue(manual.arm(.3, True))
                self.assertEqual(manual.frame(.3, 0, 0), b"JOG 0 0 0\n")
                manual.receive("MANUAL READY: fixture", .31)
                self.assertEqual(manual.frame(.32, 100, -200, 300), b"JOG 100 -200 300\n")

    def test_goto_track_transitions_follow_correlated_firmware_not_local_angles(self):
        self.goto()
        self.auto.receive("CELESTIAL_TRACK id=2", .1)
        self.assertEqual(self.auto.phase, "CELESTIAL_GOTO")
        self.auto.receive("CELESTIAL_DEADLINE id=1 deadline_ms=130000", .2)
        self.assertGreater(self.auto.deadline, 130)
        self.auto.receive("CELESTIAL_TRACK id=1", .3)
        self.assertEqual(self.auto.phase, "CELESTIAL_TRACK")
        self.assertIsNone(self.auto.deadline)
        self.auto.receive("CELESTIAL_DEADLINE id=1 deadline_ms=100", .4)
        self.assertIsNone(self.auto.deadline)

    def test_tracking_updates_both_axes_once_per_second_no_catch_up(self):
        self.track()
        for i in range(1, 601):
            now = i * 1.01 + .02
            self.auto.receive("M09 BUSY", now)
            self.assertIsNone(self.auto.frame(now))  # Schedule calculation, never wait.
            self.result(az=100 + i * .004, alt=30 + i * .002)
            command = self.auto.frame(now + .001)
            self.assertTrue(command.startswith(f"CELESTIAL_UPDATE 1 {i} ".encode()))
            values = command.decode().split()
            self.assertAlmostEqual(float(values[3]), 95 + i * .004, places=5)
            self.assertAlmostEqual(float(values[4]), 30 + i * .002, places=5)
            for delay in (.002, .01, .2, .99):
                self.assertIsNone(self.auto.frame(now + delay))
        self.assertEqual(len(self.worker.jobs), 601)
        self.assertEqual(self.auto.phase, "CELESTIAL_TRACK")

    def test_wrap_preserves_wrapped_heading_without_full_rotation_command(self):
        self.track()
        for now, az, expected in ((1.1, 4.99, 359.99), (2.2, 5.01, .01)):
            self.auto.receive("M09 BUSY", now)
            self.auto.frame(now)
            self.result(az=az)
            self.assertAlmostEqual(float(self.auto.frame(now + .001).split()[3]), expected)

    def test_cancel_every_phase_discards_pending_and_late_results(self):
        for phase in ("prepare", "goto", "track"):
            with self.subTest(phase=phase):
                self.setUp()
                {"prepare": self.start, "goto": self.goto, "track": self.track}[phase]()
                old_job = self.worker.jobs[-1]
                self.auto.cancel(.4, "Joystick takeover")
                self.result(job=old_job)
                self.auto.receive("CELESTIAL_TRACK id=1", .5)
                self.auto.receive("M09 READY", .6)
                self.assertIsNone(self.auto.frame(.7))
                self.assertEqual(self.auto.phase, "READY")
                self.assertIsNone(self.auto.celestial.target)

    def test_raw_stop_clears_source_in_both_motion_phases(self):
        for start in (self.goto, self.track):
            self.setUp()
            start()
            self.auto.raw_sent("STOP", .3)
            self.assertIsNone(self.auto.celestial.target)
            self.auto.receive("M09 READY", .4)
            self.assertIsNone(self.auto.frame(2))

    def test_stale_missing_low_quality_encoder_terminal_recovers(self):
        for starter in ("goto", "track"):
            for reason in ("encoder_stale", "encoder_unavailable", "encoder_bad_magnet"):
                with self.subTest(starter=starter, reason=reason):
                    self.setUp()
                    getattr(self, starter)()
                    self.auto.receive(f"CELESTIAL_RESULT id=1 status=STOPPED reason={reason}", .4)
                    self.auto.receive("M09 READY", .41)
                    self.assertEqual(self.auto.phase, "READY")
                    self.assertIn(reason, self.auto.note)
                    self.assertIsNone(self.auto.frame(.5))

    def test_abort_and_restart_cannot_resurrect_tracking(self):
        for marker in ("M09 ABORTED", "M09_xbox_control: restart", "KEYFRAME_INVALIDATED epoch=23"):
            self.setUp()
            self.track()
            self.auto.receive(marker, .4)
            self.assertIsNone(self.auto.celestial.target)
            self.assertIsNone(self.auto.frame(.5))

    def test_communication_and_compute_lease_failures_do_not_spam(self):
        self.track()
        self.auto.receive("M09 BUSY", 1.1)
        self.assertIsNone(self.auto.frame(1.1))
        self.auto.receive("M09 BUSY", 2.6)
        self.assertEqual(self.auto.frame(2.6), b"STOP\n")
        self.assertIn("delayed", self.auto.note)
        self.assertIsNone(self.auto.frame(2.7))
        self.setUp()
        self.track()
        self.assertEqual(self.auto.frame(3.5), b"STOP\n")
        self.assertEqual(self.auto.phase, "FAULT")
        self.assertIsNone(self.auto.frame(3.6))

    def test_slow_startup_result_is_recomputed_with_current_time(self):
        self.start()
        self.auto.receive("M09 READY", 2.1)
        self.result()
        self.assertIsNone(self.auto.frame(2.1))
        self.assertEqual(len(self.worker.jobs), 2)
        self.result(az=101)
        self.assertEqual(self.auto.frame(2.2), b"CELESTIAL_GOTO 1 96.000000 30.000000\n")

    def test_missing_goto_ack_stops_without_updates(self):
        self.start()
        self.auto.receive("M09 READY", .01)
        self.result()
        self.auto.frame(.02)
        self.auto.receive("M09 BUSY", 3)
        self.assertEqual(self.auto.frame(4.1), b"STOP\n")
        self.assertEqual(len(self.worker.jobs), 1)

    def test_unexpected_ready_and_lost_track_marker_are_handled(self):
        self.goto()
        self.auto.receive("CELESTIAL_STATE id=1 mode=TRACK yaw_target=95 pitch_target=30 yaw_error=.1 pitch_error=.2", .2)
        self.assertEqual(self.auto.phase, "CELESTIAL_TRACK")
        self.auto.receive("M09 READY", .3)
        self.assertEqual(self.auto.phase, "READY")
        self.assertIsNone(self.auto.celestial.target)
        self.assertIsNone(self.auto.frame(1))

    def test_command_notation_and_ui_diagnostics(self):
        self.assertEqual(parse_tracking_command("track_radec 24 -00:30:00").ra_hours, 0)
        self.track()
        text = " ".join(self.auto.celestial_display_lines())
        for expected in ("CELESTIAL_TRACK", "RA=", "Dec=", "Alt=", "Az=", "UTC=", "observer", "true-north"):
            self.assertIn(expected, text)


class CoordinateWorkerTests(unittest.TestCase):
    def test_slow_library_does_not_block_submit_cancel_or_close(self):
        entered, release, exited = threading.Event(), threading.Event(), threading.Event()

        def factory(observer):
            entered.set()
            release.wait(2)
            exited.set()
            raise ImportError("simulated optional library failure")

        worker = CoordinateWorker(Observer(42, -83), HeadingReference(), converter_factory=factory)
        try:
            worker.submit(1, parse_tracking_command("TRACK_RADEC 1 2"), 0)
            self.assertTrue(entered.wait(1))
            self.assertIsNone(worker.poll())
            worker.close()  # No join on a slow/stalled astronomy operation.
            self.assertFalse(exited.is_set())
            self.assertTrue(worker._thread.daemon)
        finally:
            release.set()
        self.assertTrue(exited.wait(1))

    def test_worker_returns_utc_horizontal_and_reference_mapping(self):
        done = threading.Event()
        class Converter:
            def __init__(self, observer):
                pass
            def altaz(self, target, utc):
                return HorizontalTarget(101, 42, utc)
        worker = CoordinateWorker(Observer(42, -83), HeadingReference(5, 1, 2, north_reference="magnetic"), Converter, lambda: UTC)
        original = worker._replace
        def replace(mailbox, item):
            original(mailbox, item)
            if mailbox is worker._results:
                done.set()
        worker._replace = replace
        try:
            worker.submit(7, parse_tracking_command("TRACK_RADEC 1 2"), .5)
            self.assertTrue(done.wait(1))
            result = worker.poll()
            self.assertEqual(result.generation, 7)
            self.assertEqual(result.utc, UTC)
            self.assertEqual(result.mount.heading_deg, 95)
            self.assertEqual(result.mount.pitch_deg, 40)
        finally:
            worker.close()

    def test_worker_resolves_captured_horizontal_to_fixed_target(self):
        done = threading.Event()
        target = EquatorialTarget(5.25, 22.5)
        class Converter:
            def __init__(self, observer):
                pass
            def equatorial(self, horizontal, utc):
                self.horizontal, self.utc = horizontal, utc
                return target
        reference = HeadingReference(5, 1, 2, north_reference="magnetic")
        worker = CoordinateWorker(Observer(42, -83), reference, Converter, lambda: UTC)
        original = worker._replace
        def replace(mailbox, item):
            original(mailbox, item)
            if mailbox is worker._results:
                done.set()
        worker._replace = replace
        try:
            worker.capture(9, HorizontalTarget(101, 42), .5)
            self.assertTrue(done.wait(1))
            result = worker.poll()
            self.assertEqual(result.target, target)
            self.assertEqual(result.utc, UTC)
            self.assertEqual(result.horizontal.utc, UTC)
            self.assertEqual((result.mount.heading_deg, result.mount.pitch_deg), (95, 40))
        finally:
            worker.close()


if __name__ == "__main__":
    unittest.main()
