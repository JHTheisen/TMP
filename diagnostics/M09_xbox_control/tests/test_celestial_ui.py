"""Headless celestial editor and operator-priority integration; no hardware I/O."""
import contextlib
from datetime import datetime, timezone
import io
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
from celestial_control import Calculation
from celestial_coordinates import EquatorialTarget, HorizontalTarget


TARGET = "TRACK_RADEC 18:36:56.3 +38:47:01"


def key(value):
    return pygame.event.Event(pygame.KEYDOWN, key=value)


def submit(value=TARGET):
    return [pygame.event.Event(pygame.TEXTINPUT, text=value), key(pygame.K_RETURN)]


class CelestialUiTests(unittest.TestCase):
    def test_bno_display_failures_leave_celestial_serial_stream_unchanged(self):
        baseline = self.run_ui([(1.4, submit())], until=4.6, real_session=True)
        def invalid(port):
            port.rx += (b"BNO_STATE available=YES has_sample=YES fresh=YES age_ms=10 "
                        b"accuracy=3 heading=nan physical_pitch=oops pitch_axis=PITCH pitch_roll=inf\n")
        def disconnected(port):
            port.rx += b"BNO_STATE available=NO has_sample=NO fresh=NO age_ms=4294967295\n"
        def stale(port):
            port.rx += (b"BNO_STATE available=YES has_sample=YES fresh=NO age_ms=4000 "
                        b"accuracy=0 heading=123 physical_pitch=12 pitch_axis=PITCH pitch_roll=-5\n")
        result, writes, _, _, auto, rows, _ = self.run_ui(
            [(1.0, invalid), (1.4, submit()), (2.0, disconnected), (2.6, stale)],
            until=4.6, real_session=True)
        self.assertEqual(result, 0)
        self.assertEqual(writes, baseline[1])
        self.assertEqual(auto.phase, "CELESTIAL_TRACK")
        self.assertTrue(any(data.startswith(b"CELESTIAL_UPDATE ") for _, data in writes))
        self.assertIn("UNAVAILABLE", rows)
        self.assertIn("STALE", rows)

    def run_ui(self, actions=(), *, until=3.0, dry_run=False, config=None,
               real_session=False, worker_error=None, gui_entry=False):
        class Clock:
            now = 0.0
            def monotonic(self):
                return self.now
            def sleep(self, amount):
                self.now += amount

        clock = Clock()
        writes, requests, rendered = [], [], []
        session = xbox_control.ManualSession()

        class FixtureAuto(xbox_control.AutoSession):
            """Model only the astronomy worker; exercise the real host event loop."""
            def configure_celestial(self, observer, reference):
                self.observer, self.reference = observer, reference

            def request_celestial(self, command, now, centered, dry_run=False):
                requests.append((command, now, centered, dry_run))
                if command != TARGET:
                    raise ValueError("Invalid celestial coordinates")
                if self.observer is None:
                    raise ValueError("Observer location unavailable")
                if self.phase != "READY" or not centered:
                    raise ValueError("Wait for READY with centered sticks")
                if dry_run:
                    return None
                self.action = "celestial"
                self.phase = "CELESTIAL_GOTO"
                self.started = now
                self.next_update = now + .1
                return b"STOP\n"

            def receive(self, line, now):
                if self.action == "celestial" and line == "M09 READY":
                    self.last_rx = now
                    return
                super().receive(line, now)

            def frame(self, now):
                if self.action != "celestial":
                    return super().frame(now)
                if now - self.started >= .8:
                    self.phase = "CELESTIAL_TRACK"
                if now >= self.next_update:
                    self.next_update = now + .5
                    return b"CELESTIAL_TEST_UPDATE\n"
                return None

            def celestial_display_lines(self):
                return ([self.phase + " | RA=18:36:56.3 Dec=+38:47:01"]
                        if self.action == "celestial" else [])

        class ReadyWorker:
            def __init__(self, reference):
                self.reference, self.result = reference, None
            def submit(self, generation, target, now):
                if worker_error:
                    self.result = Calculation(generation, now, error=worker_error)
                    return
                utc = datetime(2026, 10, 1, tzinfo=timezone.utc)
                horizontal = HorizontalTarget(100 + now * .004, 30 + now * .002, utc)
                self.result = Calculation(generation, now, utc, horizontal, self.reference.mount_target(horizontal))
            def capture(self, generation, horizontal, now):
                utc = datetime(2026, 10, 1, tzinfo=timezone.utc)
                target = EquatorialTarget(5.25, 22.5)
                self.result = Calculation(generation, now, utc, horizontal,
                                          self.reference.mount_target(horizontal), target=target)
            def poll(self):
                value, self.result = self.result, None
                return value
            def close(self):
                pass

        class RealSession(xbox_control.AutoSession):
            def configure_celestial(self, observer, reference):
                super().configure_celestial(observer, reference)
                if self.celestial:
                    self.celestial.worker = ReadyWorker(reference)

        auto = RealSession() if real_session else FixtureAuto()

        class Port:
            rx = b""
            mode = "READY"
            axes = [0.0] * 6
            closed = False
            track_at = None
            celestial_id = None
            @property
            def in_waiting(self):
                return len(self.rx)
            def read(self, size):
                data, self.rx = self.rx[:size], self.rx[size:]
                return data
            def write(self, data):
                writes.append((clock.now, data))
                verb = data.strip().split()[0]
                if verb == b"STOP":
                    if self.celestial_id is not None:
                        self.rx += f"CELESTIAL_RESULT id={self.celestial_id} status=STOPPED reason=operator_STOP\n".encode()
                    self.track_at = self.celestial_id = None
                    self.mode = "READY"
                    self.rx += b"M09 READY\n"
                elif verb == b"STATUS":
                    self.rx += f"M09 {self.mode}\n".encode()
                elif verb == b"CELESTIAL_TEST_UPDATE":
                    self.mode = "BUSY"
                elif verb == b"CELESTIAL_GOTO":
                    self.celestial_id = int(data.split()[1])
                    self.mode = "BUSY"
                    self.track_at = clock.now + .8
                    self.rx += f"CELESTIAL_ACCEPTED id={self.celestial_id} deadline_ms=90000\n".encode()
                elif verb == b"CELESTIAL_UPDATE":
                    if self.mode != "BUSY":
                        raise AssertionError("A late target update escaped cancellation")
                elif verb == b"JOG" and self.mode == "READY":
                    self.mode = "MANUAL"
                    self.rx += b"MANUAL READY: fixture\n"
                return len(data)
            def close(self):
                self.closed = True

        port = Port()
        joystick = mock.Mock()
        joystick.get_numaxes.return_value = 6
        joystick.get_instance_id.return_value = 42
        joystick.get_name.return_value = "Celestial test controller"
        joystick.get_axis.side_effect = lambda index: port.axes[index]
        initial = [] if gui_entry else [(.8, [key(pygame.K_F2)])]
        schedule = sorted(initial + list(actions), key=lambda action: action[0])
        sent = set()

        def events():
            values = []
            if port.track_at is not None and clock.now >= port.track_at:
                port.track_at = None
                port.rx += f"CELESTIAL_TRACK id={port.celestial_id}\n".encode()
            for index, (at, action) in enumerate(schedule):
                if clock.now >= at and index not in sent:
                    sent.add(index)
                    if callable(action):
                        action(port)
                    else:
                        values.extend(action)
            if clock.now >= until:
                values.append(pygame.event.Event(pygame.QUIT))
            return values

        original_font = pygame.font.SysFont
        def make_font(*args):
            font = original_font(*args)
            wrapper = mock.Mock(wraps=font)
            def render(value, *args):
                rendered.append(value)
                return font.render(value, *args)
            wrapper.render.side_effect = render
            return wrapper

        if config is None:
            config = ["--latitude", "40", "--longitude", "-75"]
        with tempfile.TemporaryDirectory() as folder, contextlib.redirect_stdout(io.StringIO()), \
             mock.patch.object(pygame.joystick, "get_count", return_value=1), \
             mock.patch.object(pygame.joystick, "Joystick", return_value=joystick), \
             mock.patch.object(pygame.event, "get", side_effect=events), \
             mock.patch.object(pygame.font, "SysFont", side_effect=make_font), \
             mock.patch.object(serial, "Serial", return_value=port) as opener, \
             mock.patch.object(xbox_control, "ManualSession", return_value=session), \
             mock.patch.object(xbox_control, "AutoSession", return_value=auto), \
             mock.patch.object(xbox_control, "time", clock):
            result = xbox_control.main(config + ["--log-dir", folder] + (["--dry-run"] if dry_run else []))
            log = next(Path(folder).glob("*.log")).read_text(encoding="utf-8")
        if dry_run:
            opener.assert_not_called()
            self.assertEqual(writes, [])
        else:
            self.assertTrue(port.closed)
        self.assertFalse(any(data.startswith(b"TRACK_RADEC") for _, data in writes))
        return result, writes, requests, session, auto, rendered, log

    def test_host_command_and_live_status_during_goto_and_track(self):
        result, writes, requests, session, auto, rows, log = self.run_ui([(1.4, submit())])
        self.assertEqual(result, 0)
        self.assertEqual(len(requests), 1)
        self.assertEqual(session.last_raw_command, TARGET)
        self.assertEqual(session.raw_text, "")
        self.assertEqual(auto.phase, "CELESTIAL_TRACK")
        self.assertTrue(any("CELESTIAL_GOTO" in row for row in rows))
        self.assertTrue(any("CELESTIAL_TRACK" in row for row in rows))
        self.assertIn("CELESTIAL_REQUEST " + TARGET, log)
        self.assertIn("CELESTIAL_CONFIG", log)
        self.assertIn("CELESTIAL CELESTIAL_TRACK", log)
        self.assertLessEqual(log.count(" CELESTIAL "), 4)
        self.assertFalse(any(at >= .8 and data.startswith(b"JOG") for at, data in writes))

    def test_dashboard_target_fields_submit_through_managed_celestial_path(self):
        click = lambda pos: pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=pos)
        result, writes, requests, session, auto, rows, log = self.run_ui([
            (.8, [click((100, 320)), pygame.event.Event(pygame.TEXTINPUT, text="18:36:56.3")]),
            (.9, [click((550, 320)), pygame.event.Event(pygame.TEXTINPUT, text="+38:47:01")]),
            (1.4, [click((990, 320))]),
        ], gui_entry=True)
        self.assertEqual(result, 0)
        self.assertEqual([request[0] for request in requests], [TARGET])
        self.assertEqual(session.last_raw_command, TARGET)
        self.assertFalse(session.raw_mode)
        self.assertEqual(auto.phase, "CELESTIAL_TRACK")
        self.assertFalse(any(data.startswith(b"TRACK_RADEC") for _, data in writes))
        self.assertIn("CELESTIAL_REQUEST " + TARGET, log)

    def test_safety_batch_prevents_celestial_start(self):
        for event in (key(pygame.K_SPACE), key(pygame.K_F12), key(pygame.K_F2),
                      key(pygame.K_x),
                      pygame.event.Event(pygame.JOYBUTTONDOWN, instance_id=42, button=1)):
            with self.subTest(event=event):
                _, writes, requests, _, _, _, _ = self.run_ui([(1.4, submit() + [event])])
                self.assertEqual(requests, [])
                self.assertFalse(any(data == b"CELESTIAL_TEST_UPDATE\n" for _, data in writes))

    def test_focus_loss_and_regain_do_not_change_tracking_or_create_takeover(self):
        baseline = self.run_ui([(1.4, submit())], until=4.2, real_session=True)
        focus = [pygame.event.Event(pygame.WINDOWFOCUSLOST),
                 pygame.event.Event(pygame.JOYAXISMOTION, instance_id=42, axis=0, value=1.0)]
        regain = [pygame.event.Event(pygame.WINDOWFOCUSGAINED)]
        result, writes, _, session, auto, _, log = self.run_ui(
            [(1.4, submit()), (2.5, focus), (3.0, regain)], until=4.2, real_session=True)
        self.assertEqual(result, 0)
        self.assertEqual(writes, baseline[1])
        self.assertEqual(auto.phase, "CELESTIAL_TRACK")
        self.assertTrue(session.auto_mode)
        self.assertIn("focus lost; controller mode unchanged", log)
        self.assertIn("focus gained; controller mode unchanged", log)

    def test_hide_diagnostics_is_ui_only_while_tracking(self):
        baseline = self.run_ui([(1.4, submit())], until=3.6, real_session=True)
        click = lambda pos: pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=pos)
        result, writes, _, _, auto, rows, _ = self.run_ui(
            [(1.4, submit()), (2.4, [click((997, 532))]), (2.7, [click((997, 69))])],
            until=3.6, real_session=True)
        self.assertEqual(result, 0)
        self.assertEqual(writes, baseline[1])
        self.assertEqual(auto.phase, "CELESTIAL_TRACK")
        self.assertTrue(any("Diagnostics" in row for row in rows))

    def test_track_here_button_captures_current_pointing_and_tracks(self):
        def orientation(port):
            port.rx += (b"BNO_STATE available=YES has_sample=YES fresh=YES age_ms=10 accuracy=3 "
                        b"north_usable=YES heading=100 physical_pitch=30 pitch_axis=PITCH pitch_roll=2\n")
        result, writes, _, session, auto, rows, log = self.run_ui(
            [(.9, orientation), (1.4, [pygame.event.Event(
                pygame.MOUSEBUTTONDOWN, button=1, pos=(850, 327))])],
            until=3.4, real_session=True, gui_entry=True)
        self.assertEqual(result, 0)
        self.assertEqual(auto.phase, "CELESTIAL_TRACK")
        self.assertIn("TRACK HERE | RA=5.250000 h Dec=+22.500000 deg", auto.celestial_status)
        self.assertTrue(any(data.startswith(b"CELESTIAL_GOTO ") for _, data in writes))
        self.assertTrue(any("TRACK HERE" in row for row in rows))
        self.assertIn("CELESTIAL_REQUEST TRACK HERE captured", log)
        self.assertTrue(session.auto_mode)

    def test_track_here_missing_orientation_reports_failure_and_manual_continues(self):
        click_here = pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(850, 327))
        result, writes, _, session, auto, rows, log = self.run_ui(
            [(1.4, [click_here]), (1.8, lambda port: port.axes.__setitem__(1, -.8))],
            until=2.3, real_session=True, gui_entry=True)
        self.assertEqual(result, 0)
        self.assertEqual(auto.phase, "READY")
        self.assertTrue(any(at >= 1.8 and data == b"JOG 0 585 0\n" for at, data in writes))
        self.assertTrue(any("TRACK HERE needs" in row for row in rows))
        self.assertIn("CELESTIAL_NOT_SENT TRACK HERE needs", log)
        self.assertEqual(session.state, "active")

    def test_goto_and_track_stop_cancel_future_updates(self):
        for at in (1.8, 2.4):
            with self.subTest(at=at):
                _, writes, _, session, auto, _, _ = self.run_ui([(1.4, submit()), (at, [key(pygame.K_SPACE)])])
                self.assertFalse(any(stamp >= at and data == b"CELESTIAL_TEST_UPDATE\n" for stamp, data in writes))
                self.assertFalse(session.raw_mode)
                self.assertIsNone(auto.action)

    def test_managed_request_leaves_raw_editor_and_ignores_later_text(self):
        _, writes, _, session, auto, _, _ = self.run_ui([
            (1.4, submit()), (1.8, submit("POSE 20 30 0")),
            (2.0, submit("STOP")), (2.2, [key(pygame.K_SPACE)])])
        self.assertFalse(any(data.startswith(b"POSE") for _, data in writes))
        self.assertFalse(any(data == b"STOP\n" and at in (1.8, 2.0) for at, data in writes))
        self.assertEqual(session.last_raw_command, TARGET)
        self.assertFalse(session.raw_mode)
        self.assertIsNone(auto.action)

    def test_track_radec_submission_exits_f2_raw_mode(self):
        _, writes, _, session, auto, _, log = self.run_ui([(1.4, submit())], until=1.7)
        self.assertFalse(session.raw_mode)
        self.assertNotIn("Raw commands enabled; joystick JOG disabled", session.readiness_note)
        self.assertEqual(session.last_raw_command, TARGET)
        self.assertEqual(auto.action, "celestial")
        self.assertIn("Automatic action active", session.readiness_note)
        self.assertFalse(any(data.startswith(b"TRACK_RADEC") for _, data in writes))
        self.assertIn("CELESTIAL_REQUEST " + TARGET, log)

    def test_real_auto_lifecycle_editor_goto_track_and_takeover(self):
        result, writes, _, session, auto, rows, log = self.run_ui([
            (1.4, submit()), (4.8, lambda port: port.axes.__setitem__(1, -.8))],
            until=5.5, real_session=True)
        self.assertEqual(result, 0)
        gotos = [data for _, data in writes if data.startswith(b"CELESTIAL_GOTO ")]
        updates = [data for _, data in writes if data.startswith(b"CELESTIAL_UPDATE ")]
        self.assertEqual(len(gotos), 1)
        self.assertEqual(len(updates), 3)
        self.assertEqual([int(data.split()[2]) for data in updates], [1, 2, 3])
        self.assertTrue(any("CELESTIAL_TRACK" in row for row in rows))
        after = [data for at, data in writes if at >= 4.8]
        self.assertEqual(after[0], b"STOP\n")
        jogs = [data for data in after if data.startswith(b"JOG ")]
        self.assertEqual(jogs[0], b"JOG 0 0 0\n")
        self.assertIn(b"JOG 0 585 0\n", jogs[1:])
        self.assertFalse(any(data.startswith(b"CELESTIAL_") for data in after))
        self.assertFalse(session.raw_mode)
        self.assertEqual(auto.phase, "READY")

    def test_real_auto_goto_and_track_stick_takeover_use_shared_handoff(self):
        for at, expected_phase in ((1.8, "CELESTIAL_GOTO"), (2.8, "CELESTIAL_TRACK")):
            with self.subTest(phase=expected_phase):
                result, writes, _, session, auto, rows, _ = self.run_ui([
                    (1.4, submit()), (at, lambda port: port.axes.__setitem__(1, -.8))],
                    until=3.6, real_session=True)
                self.assertEqual(result, 0)
                self.assertTrue(any(expected_phase in row for row in rows))
                after = [data for stamp, data in writes if stamp >= at]
                self.assertEqual(after[0], b"STOP\n")
                jogs = [data for data in after if data.startswith(b"JOG ")]
                self.assertGreaterEqual(len(jogs), 2)
                self.assertEqual(jogs[0], b"JOG 0 0 0\n")
                self.assertIn(b"JOG 0 585 0\n", jogs[1:])
                self.assertFalse(any(data.startswith(b"CELESTIAL_") for data in after))
                self.assertFalse(session.raw_mode)
                self.assertEqual(auto.phase, "READY")

    def test_real_auto_stop_during_preparation_and_tracking_restores_manual(self):
        for at in (1.405, 2.8):
            with self.subTest(at=at):
                result, writes, _, session, auto, _, _ = self.run_ui([
                    (1.4, submit()), (at, [key(pygame.K_SPACE)])],
                    until=3.8, real_session=True)
                self.assertEqual(result, 0)
                self.assertIsNone(auto.action)
                self.assertFalse(any(stamp >= at and data.startswith(b"CELESTIAL_") for stamp, data in writes))
                self.assertFalse(session.raw_mode)
                self.assertTrue(any(stamp > at + .5 and data == b"JOG 0 0 0\n" for stamp, data in writes))

    def test_below_horizon_and_calculation_failure_restore_manual(self):
        for error in ("ValueError: Target is below the horizon (altitude=-4.000deg)",
                      "RuntimeError: simulated celestial calculation failure"):
            with self.subTest(error=error):
                result, writes, _, session, auto, _, log = self.run_ui(
                    [(1.4, submit())], until=2.8, real_session=True, worker_error=error)
                self.assertEqual(result, 0)
                self.assertFalse(session.raw_mode)
                self.assertEqual(auto.phase, "READY")
                self.assertIsNone(auto.action)
                self.assertIn(error, log)
                celestial_stops = [stamp for stamp, data in writes if stamp >= 1.4 and data == b"STOP\n"]
                self.assertGreaterEqual(len(celestial_stops), 2)
                self.assertTrue(any(stamp > celestial_stops[-1] + .5 and data == b"JOG 0 0 0\n"
                                    for stamp, data in writes))

    def test_celestial_terminal_result_and_ready_restore_manual(self):
        def complete(port):
            ident = port.celestial_id
            port.track_at = port.celestial_id = None
            port.mode = "READY"
            port.rx += (f"CELESTIAL_RESULT id={ident} status=STOPPED reason=operation_complete\n"
                        "M09 READY\n").encode()

        result, writes, _, session, auto, _, _ = self.run_ui([
            (1.4, submit()), (2.8, complete)], until=3.7, real_session=True)
        self.assertEqual(result, 0)
        self.assertFalse(session.raw_mode)
        self.assertEqual(auto.phase, "READY")
        self.assertIsNone(auto.action)
        self.assertTrue(any(stamp > 3.3 and data == b"JOG 0 0 0\n" for stamp, data in writes))

    def test_leaving_f2_after_celestial_use_restores_manual_jog(self):
        result, writes, _, session, auto, _, _ = self.run_ui([
            (1.4, submit()),
            # First F2 cancels celestial and deliberately enters the raw editor;
            # the second leaves it through the existing F2 exit path.
            (1.8, [key(pygame.K_F2)]), (2.1, [key(pygame.K_F2)])], until=3.0,
            real_session=True)
        self.assertEqual(result, 0)
        self.assertFalse(session.raw_mode)
        self.assertEqual(auto.phase, "READY")
        self.assertTrue(any(stamp > 2.6 and data == b"JOG 0 0 0\n" for stamp, data in writes))

    def test_goto_and_track_sticks_stop_then_zero_jog_before_live_jog(self):
        for at in (1.8, 2.4):
            with self.subTest(at=at):
                _, writes, _, session, _, _, _ = self.run_ui([
                    (1.4, submit()), (at, lambda port: port.axes.__setitem__(1, -.8))], until=3.2)
                after = [data for stamp, data in writes if stamp >= at]
                self.assertEqual(after[0], b"STOP\n")
                jogs = [data for data in after if data.startswith(b"JOG")]
                self.assertEqual(jogs[0], b"JOG 0 0 0\n")
                self.assertIn(b"JOG 0 585 0\n", jogs[1:])
                self.assertFalse(session.raw_mode)
                self.assertNotIn(b"CELESTIAL_TEST_UPDATE\n", after)

    def test_b_abort_during_goto_and_track_remains_latched(self):
        for at in (1.8, 2.4):
            with self.subTest(at=at):
                result, writes, _, _, _, _, _ = self.run_ui([(1.4, submit()),
                    (at, [pygame.event.Event(pygame.JOYBUTTONDOWN, instance_id=42, button=1)])])
                self.assertEqual(result, 1)
                self.assertEqual(writes[-1][1], b"X\n")
                self.assertFalse(any(stamp >= at and data == b"CELESTIAL_TEST_UPDATE\n" for stamp, data in writes))

    def test_invalid_command_preserves_editor_and_can_return_to_manual(self):
        bad = "TRACK_RADEC bad bad"
        result, writes, _, session, _, _, log = self.run_ui([(1.4, submit(bad))])
        self.assertEqual(result, 0)
        self.assertEqual(session.raw_text, bad)
        self.assertEqual(session.last_raw_command, "")
        self.assertIn("Invalid celestial coordinates", log)
        self.assertFalse(any(data == b"CELESTIAL_TEST_UPDATE\n" for _, data in writes))

    def test_internal_protocol_cannot_bypass_host(self):
        for command in ("CELESTIAL 1 20 30", "CELESTIAL_UPDATE 20 30", "TRACK_SET 20 30"):
            with self.subTest(command=command):
                result, writes, requests, session, _, _, log = self.run_ui([(1.4, submit(command))])
                self.assertEqual(result, 0)
                self.assertEqual(requests, [])
                self.assertEqual(session.raw_text, command)
                self.assertNotIn((command + "\n").encode(), [data for _, data in writes])
                self.assertIn("host-managed", log)

    def test_bad_optional_config_keeps_manual_available(self):
        for config in ([], ["--latitude", "40"], ["--latitude", "bad", "--longitude", "-75"],
                       ["--latitude", "nan", "--longitude", "-75"],
                       ["--latitude", "40", "--longitude", "-75", "--heading-offset", "nan"]):
            with self.subTest(config=config):
                result, writes, _, _, _, _, log = self.run_ui([(1.4, submit()), (1.8, [key(pygame.K_F2)])], config=config)
                self.assertEqual(result, 0)
                self.assertIn("Celestial unavailable", log)
                self.assertTrue(any(stamp > 2.3 and data == b"JOG 0 0 0\n" for stamp, data in writes))

    def test_dry_run_never_sends_or_starts_tracking(self):
        result, _, requests, _, auto, _, _ = self.run_ui([(1.4, submit())], dry_run=True)
        self.assertEqual(result, 0)
        self.assertTrue(requests[0][3])
        self.assertIsNone(auto.action)


if __name__ == "__main__":
    unittest.main()
