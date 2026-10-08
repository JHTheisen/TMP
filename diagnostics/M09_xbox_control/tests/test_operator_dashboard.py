"""Headless tests for the presentation-only operator dashboard."""
import os
import copy
from pathlib import Path
import sys
import unittest
from unittest import mock

os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pygame

from auto_control import AutoSession
from operator_dashboard import OperatorDashboard
from xbox_control import ManualSession


ENCODER_REPORT = ("ORIENTATION_STATE protocol=2 feedback=AS5600 level_set=YES north_set=YES available=YES has_sample=YES fresh=YES age_ms=10 "
              "heading=301.800 physical_pitch=-9.989 "
              "pitch_axis=PITCH roll=UNAVAILABLE")


class RecordingFont:
    def __init__(self, font, output):
        self.font, self.output = font, output

    def render(self, value, *args, **kwargs):
        self.output.append(str(value))
        return self.font.render(value, *args, **kwargs)

    def size(self, value):
        return self.font.size(value)


class OperatorDashboardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pygame.init()

    @classmethod
    def tearDownClass(cls):
        pygame.quit()

    def make_dashboard(self):
        screen = pygame.display.set_mode((1100, 760), pygame.RESIZABLE)
        dashboard = OperatorDashboard(pygame)
        session, auto = ManualSession(), AutoSession()
        return screen, dashboard, session, auto

    def draw(self, screen, dashboard, session, auto, now=2.0, display_frozen=False):
        dashboard.draw(
            screen, now=now, session=session, auto=auto,
            joystick_name="Test Xbox", serial_label="COM9 | ACTIVE",
            yaw=12, pitch=-34, carriage=0, centered=True,
            raw_axes=[0.0] * 6, input_notice="controller connected",
            log_label="test.log", display_frozen=display_frozen,
            diagnostic_lines=session.diagnostic_lines(2.0), response_lines=[])

    def test_display_flip_has_separate_slow_operation_timing(self):
        screen = pygame.display.set_mode((1100, 760), pygame.RESIZABLE)
        samples = iter((10.0, 10.2))
        timings = []
        dashboard = OperatorDashboard(
            pygame, timing=lambda operation, elapsed: timings.append((operation, elapsed)),
            clock=lambda: next(samples))
        self.draw(screen, dashboard, ManualSession(), AutoSession())
        self.assertEqual(timings[0][0], "pygame.display.flip")
        self.assertAlmostEqual(timings[0][1], .2)

    def test_live_encoder_angles_are_not_replaced_by_targets_or_frozen_diagnostics(self):
        screen, dashboard, session, auto = self.make_dashboard()
        output = []
        dashboard.title = RecordingFont(dashboard.title, output)
        dashboard.font = RecordingFont(dashboard.font, output)
        session.receive(ENCODER_REPORT, 1.9)
        session.receive("CELESTIAL_STATE yaw_target=100 yaw_error=2 pitch_target=50 pitch_error=1", 1.9)
        auto.action, auto.phase = "celestial", "CELESTIAL_TRACK"
        before = copy.deepcopy((vars(session), vars(auto)))
        self.draw(screen, dashboard, session, auto, display_frozen=True)
        self.assertIn("-9.989°", output)
        self.assertIn("301.800°", output)
        self.assertIn("UNAVAILABLE", output)  # roll
        self.assertEqual((vars(session), vars(auto)), before)
        output.clear()
        session.receive(ENCODER_REPORT.replace("-9.989", "12.500"), 2.1)
        self.draw(screen, dashboard, session, auto, now=2.2, display_frozen=True)
        self.assertIn("12.500°", output)


    def test_optional_orientation_failures_never_change_manual_commands_or_buttons(self):
        fields = dict(token.split("=", 1) for token in ENCODER_REPORT.split()[1:])
        cases = [(None, "UNAVAILABLE"), ({}, "UNAVAILABLE")]
        for key, value in (("available", "NO"), ("has_sample", "NO"), ("valid", "NO"),
                           ("pitch_axis", "ROLL"), ("fresh", "?"),
                           ("age_ms", "nan"), ("age_ms", "inf"), ("age_ms", "-1")):
            cases.append(({**fields, key: value}, "UNAVAILABLE"))
        for key in ("physical_pitch", "heading"):
            for value in ("nan", "inf", "-inf", "bad", "", "1e999", "999999999"):
                cases.append(({**fields, key: value}, "UNAVAILABLE"))
            cases.append(({name: value for name, value in fields.items() if name != key}, "UNAVAILABLE"))
        cases += [({**fields, "fresh": "NO"}, "STALE"),
                  ({**fields, "age_ms": "2001"}, "STALE")]
        for report, expected in cases:
            with self.subTest(report=report):
                screen, dashboard, session, auto = self.make_dashboard()
                session.receive("M09 READY", 1)
                session.arm(1.1, True)
                session.receive("MANUAL READY: test", 1.2)
                if report is not None:
                    session.receive("ORIENTATION_STATE protocol=2 feedback=AS5600 level_set=YES north_set=YES " + " ".join(f"{key}={value}" for key, value in report.items()), 1.9)
                before = copy.deepcopy((vars(session), vars(auto)))
                output = []
                dashboard.font = RecordingFont(dashboard.font, output)
                self.draw(screen, dashboard, session, auto)
                self.assertIn(expected, output)
                self.assertEqual((vars(session), vars(auto)), before)
                self.assertEqual(session.frame(2, 120, -50, 75), b"JOG 120 -50 75\n")
                for name in ("track", "level", "north", "capture_a", "capture_b", "play", "stop", "abort"):
                    event = pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1,
                                               pos=dashboard.buttons[name].center)
                    self.assertEqual(dashboard.handle_events([event]), [name])

    def test_orientation_expires_without_reports_and_recovers(self):
        session = ManualSession()
        session.receive(ENCODER_REPORT, 1)
        self.assertEqual(OperatorDashboard._encoder_orientation(session.sensor_fields, session.sensor_status_at, 1.1)[0], "LIVE")
        # Unrelated telemetry cannot refresh ENCODER orientation.
        session.receive("STATE MANUAL heading=999", 3)
        self.assertEqual(OperatorDashboard._encoder_orientation(session.sensor_fields, session.sensor_status_at, 3.1), ("STALE", None))
        session.receive(ENCODER_REPORT.replace("age_ms=10", "age_ms=1900"), 3.1)
        self.assertEqual(OperatorDashboard._encoder_orientation(session.sensor_fields, session.sensor_status_at, 3.3), ("STALE", None))
        session.receive(ENCODER_REPORT.replace("accuracy=3", "accuracy=0").replace("north_usable=YES", "north_usable=NO"), 3.4)
        self.assertEqual(OperatorDashboard._encoder_orientation(session.sensor_fields, session.sensor_status_at, 3.5),
                         ("LIVE", (-9.989, 301.8)))

    def test_orientation_text_fits_existing_card_at_minimum_window_size(self):
        screen = pygame.display.set_mode((820, 620))
        dashboard = OperatorDashboard(pygame)
        session, auto = ManualSession(), AutoSession()
        for report in (ENCODER_REPORT.replace("accuracy=3", "accuracy=0"),
                       ENCODER_REPORT.replace("heading=301.800", "heading=nan")):
            session.receive(report, 1.9)
            with mock.patch.object(dashboard, "_text", wraps=dashboard._text) as draw_text:
                self.draw(screen, dashboard, session, auto)
            # Existing ENCODER card at minimum size: x=416..604, y=126..214.
            boxes = []
            for call in draw_text.call_args_list:
                _, font, value, pos, *_ = call.args
                if 416 <= pos[0] < 604 and 126 <= pos[1] < 214:
                    rect = pygame.Rect(pos, font.size(str(value)))
                    self.assertLessEqual(rect.right, 604)
                    self.assertLessEqual(rect.bottom, 214)
                    self.assertFalse(any(rect.colliderect(other) for other in boxes))
                    boxes.append(rect)
            self.assertGreaterEqual(len(boxes), 3)

    def test_target_fields_support_editing_full_command_paste_and_enter(self):
        screen, dashboard, session, auto = self.make_dashboard()
        self.draw(screen, dashboard, session, auto)
        ra_pos = dashboard.fields["ra"].center
        actions = dashboard.handle_events([
            pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=ra_pos),
            pygame.event.Event(pygame.TEXTINPUT, text="18:36"),
        ])
        self.assertEqual(actions, [])
        self.assertEqual(dashboard.ra.value, "18:36")

        dashboard._clipboard = mock.Mock(return_value="TRACK_RADEC 18:36:56.3 +38:47:01")
        actions = dashboard.handle_events([
            pygame.event.Event(pygame.KEYDOWN, key=pygame.K_v, mod=pygame.KMOD_CTRL),
            pygame.event.Event(pygame.KEYDOWN, key=pygame.K_RETURN, mod=0),
        ])
        self.assertEqual(actions, ["track"])
        self.assertEqual(dashboard.celestial_command(), "TRACK_RADEC 18:36:56.3 +38:47:01")

    def test_dashboard_buttons_emit_existing_controller_action_names(self):
        screen, dashboard, session, auto = self.make_dashboard()
        self.draw(screen, dashboard, session, auto)
        for name in ("track", "track_here", "level", "north", "stop", "abort",
                     "capture_a", "capture_b", "return_a", "play"):
            event = pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1,
                                       pos=dashboard.buttons[name].center)
            self.assertEqual(dashboard.handle_events([event]), [name])

    def test_diagnostics_toggle_does_not_change_controller_state(self):
        screen, dashboard, session, auto = self.make_dashboard()
        session.receive("M09 READY", 1.0)
        before = (session.state, session.ready, auto.phase, auto.action)
        self.draw(screen, dashboard, session, auto)
        event = pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1,
                                   pos=dashboard.buttons["diagnostics"].center)
        self.assertEqual(dashboard.handle_events([event]), [])
        self.assertTrue(dashboard.show_diagnostics)
        self.assertEqual((session.state, session.ready, auto.phase, auto.action), before)
        self.draw(screen, dashboard, session, auto)
        hide = pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1,
                                  pos=dashboard.buttons["diagnostics"].center)
        self.assertEqual(dashboard.handle_events([hide]), [])
        self.assertFalse(dashboard.show_diagnostics)
        self.assertEqual((session.state, session.ready, auto.phase, auto.action), before)

    def test_layout_reflows_within_minimum_and_large_window_sizes(self):
        for size in ((820, 620), (1500, 950)):
            with self.subTest(size=size):
                screen = pygame.display.set_mode(size, pygame.RESIZABLE)
                dashboard = OperatorDashboard(pygame)
                session, auto = ManualSession(), AutoSession()
                self.draw(screen, dashboard, session, auto)
                for rect in list(dashboard.buttons.values()) + list(dashboard.fields.values()):
                    self.assertGreaterEqual(rect.left, 0)
                    self.assertGreaterEqual(rect.top, 0)
                    self.assertLessEqual(rect.right, size[0])
                    self.assertLessEqual(rect.bottom, size[1])

    def test_active_tracking_identifies_encoder_feedback(self):
        screen, dashboard, session, auto = self.make_dashboard()
        output = []
        dashboard.font = RecordingFont(dashboard.font, output)
        dashboard.small = RecordingFont(dashboard.small, output)
        dashboard.title = RecordingFont(dashboard.title, output)
        dashboard.mono = RecordingFont(dashboard.mono, output)
        session.receive("ORIENTATION_STATE protocol=2 feedback=AS5600 level_set=YES north_set=YES available=NO fresh=NO heading=101 physical_pitch=22", 1.9)
        session.receive("ENCODER_STATE bus=A available=YES valid=YES angle_deg=101", 1.9)
        session.receive("ENCODER_STATE bus=B available=YES valid=YES angle_deg=22", 1.9)
        session.receive("CELESTIAL_STATE id=7 mode=TRACK feedback=AS5600 encoder=AVAILABLE", 1.9)
        auto.action, auto.phase = "celestial", "CELESTIAL_TRACK"
        auto.celestial_status = "RA=18.615639 h Dec=+38.783611 deg"
        self.draw(screen, dashboard, session, auto)
        self.assertIn("CELESTIAL TRACKING ACTIVE", output)
        self.assertIn("AS5600 ENCODER FEEDBACK", output)
        self.assertIn("ENC Y:OK  P:OK", output)

    def test_roll_is_explicitly_unavailable(self):
        screen, dashboard, session, auto = self.make_dashboard()
        output = []
        dashboard.small = RecordingFont(dashboard.small, output)
        dashboard.font = RecordingFont(dashboard.font, output)
        session.receive(ENCODER_REPORT, 1.9)
        self.draw(screen, dashboard, session, auto)
        self.assertIn("ROLL", output)
        self.assertIn("UNAVAILABLE", output)


    def test_bad_encoder_magnet_and_stale_sample_are_degraded(self):
        self.assertEqual(OperatorDashboard._health(
            {"available": "YES", "valid": "YES", "magnet_good": "NO"}, 1.9, 2.0)[0], "DEGRADED")
        self.assertEqual(OperatorDashboard._health(
            {"available": "YES", "valid": "YES", "age_ms": "2100"}, 1.9, 2.0)[0], "STALE")



if __name__ == "__main__":
    unittest.main()
