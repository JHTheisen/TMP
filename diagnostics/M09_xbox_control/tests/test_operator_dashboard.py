"""Headless tests for the presentation-only operator dashboard."""
import os
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

    def draw(self, screen, dashboard, session, auto):
        dashboard.draw(
            screen, now=2.0, session=session, auto=auto,
            joystick_name="Test Xbox", serial_label="COM9 | ACTIVE",
            yaw=12, pitch=-34, carriage=0, centered=True,
            raw_axes=[0.0] * 6, input_notice="controller connected",
            log_label="test.log", display_frozen=False,
            diagnostic_lines=session.diagnostic_lines(2.0), response_lines=[])

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
        for name in ("track", "level", "north", "stop", "abort",
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
        self.assertEqual(dashboard.handle_events([event]), ["diagnostics"])
        self.assertTrue(dashboard.show_diagnostics)
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

    def test_active_encoder_propagated_tracking_is_visibly_degraded(self):
        screen, dashboard, session, auto = self.make_dashboard()
        output = []
        dashboard.font = RecordingFont(dashboard.font, output)
        dashboard.small = RecordingFont(dashboard.small, output)
        dashboard.title = RecordingFont(dashboard.title, output)
        dashboard.mono = RecordingFont(dashboard.mono, output)
        session.receive("BNO_STATE available=NO fresh=NO accuracy=0 heading=101 physical_pitch=22", 1.9)
        session.receive("ENCODER_STATE bus=A available=YES valid=YES angle_deg=101", 1.9)
        session.receive("ENCODER_STATE bus=B available=YES valid=YES angle_deg=22", 1.9)
        session.receive("CELESTIAL_STATE id=7 mode=TRACK feedback=AS5600 bno=DEGRADED encoder=AVAILABLE", 1.9)
        auto.action, auto.phase = "celestial", "CELESTIAL_TRACK"
        auto.celestial_status = "RA=18.615639 h Dec=+38.783611 deg"
        self.draw(screen, dashboard, session, auto)
        self.assertIn("CELESTIAL TRACKING ACTIVE", output)
        self.assertIn("ENCODER PROPAGATED / BNO DEGRADED", output)
        self.assertIn("YAW OK", output)
        self.assertIn("PITCH OK", output)

    def test_low_accuracy_bno_and_bad_encoder_magnet_are_degraded(self):
        self.assertEqual(OperatorDashboard._health(
            {"available": "YES", "fresh": "YES", "accuracy": "1"}, 1.9, 2.0, True)[0], "DEGRADED")
        self.assertEqual(OperatorDashboard._health(
            {"available": "YES", "valid": "YES", "magnet_good": "NO"}, 1.9, 2.0)[0], "DEGRADED")


if __name__ == "__main__":
    unittest.main()
