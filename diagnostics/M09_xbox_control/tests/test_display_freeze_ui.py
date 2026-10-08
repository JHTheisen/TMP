"""Exercise rendering, telemetry and controls together with no hardware access."""
import contextlib
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


def key(value, **fields):
    return pygame.event.Event(pygame.KEYDOWN, key=value, **fields)


class DisplayFreezeUiTests(unittest.TestCase):
    def run_ui(self, actions=(), *, raw=False, dry_run=False):
        class Clock:
            now = 0.0
            def monotonic(self):
                return self.now
            def sleep(self, duration):
                self.now += duration

        clock = Clock()
        session = xbox_control.ManualSession()
        writes, frames, rendered, processed = [], [], [], []

        class Port:
            rx = b""
            active = False
            busy = False
            closed = False
            @property
            def in_waiting(self):
                return len(self.rx)
            def read(self, size):
                data, self.rx = self.rx[:size], self.rx[size:]
                return data
            def write(self, data):
                writes.append((clock.now, data))
                if data.strip() == b"STOP":
                    self.active = self.busy = False
                    self.rx += b"M09 READY\n"
                elif data == b"STATUS\n":
                    self.rx += b"M09 BUSY\n" if self.busy else (b"M09 MANUAL\n" if self.active else b"M09 READY\n")
                elif data == b"JOG 0 0 0\n" and not self.active:
                    self.active = True
                    self.rx += b"MANUAL READY: fixture\n"
                elif data.startswith(b"POSE "):
                    self.busy = True
                    self.rx += b"POSE ACCEPTED fixture\nM09 BUSY\n"
                return len(data)
            def close(self):
                self.closed = True

        port = Port()
        joystick = mock.Mock()
        joystick.get_numaxes.return_value = 6
        joystick.get_instance_id.return_value = 42
        joystick.get_name.return_value = "Simulated Xbox freeze test"
        joystick.get_axis.side_effect = lambda index: (0.7 if clock.now < 1.0 else -0.7) if index == 0 and clock.now > 0.72 and not raw else 0.0
        schedule = [(0.6, [key(pygame.K_F2)] if raw else []),
                    (0.8, [key(pygame.K_F3)]),
                    (0.85, [key(pygame.K_F3, repeat=True)]),
                    (1.3, [key(pygame.K_F3)])] + list(actions)
        schedule.sort(key=lambda item: item[0])
        sent = set()
        telemetry_sent = set()
        def events():
            for at, value in ((0.2, 10), (0.9, 20), (1.1, 30)):
                if clock.now >= at and at not in telemetry_sent:
                    telemetry_sent.add(at)
                    port.rx += (f"ORIENTATION_STATE protocol=2 feedback=AS5600 level_set=YES north_set=YES heading={value} physical_pitch={value / 10} pitch_axis=PITCH north_usable=YES\n"
                                f"POSE_STATE carriage_steps={value * 10}\n"
                                f"Telemetry marker {value}\n").encode()
            result = []
            for index, (at, items) in enumerate(schedule):
                if clock.now >= at and index not in sent:
                    sent.add(index)
                    result.extend(items)
            if clock.now >= 1.5:
                result.append(pygame.event.Event(pygame.QUIT))
            return result

        original_font = pygame.font.SysFont
        original_flip = pygame.display.flip
        def font_factory(*args, **kwargs):
            font = original_font(*args, **kwargs)
            wrapper = mock.Mock(wraps=font)
            def render(text, *args):
                rendered.append(text)
                return font.render(text, *args)
            wrapper.render.side_effect = render
            return wrapper
        def flip():
            frames.append((clock.now, tuple(rendered)))
            processed.append((clock.now, dict(session.sensor_fields), session.carriage_steps))
            rendered.clear()
            original_flip()

        with tempfile.TemporaryDirectory() as log_dir, contextlib.redirect_stdout(io.StringIO()), \
             mock.patch.object(pygame.joystick, "get_count", return_value=1), \
             mock.patch.object(pygame.joystick, "Joystick", return_value=joystick), \
             mock.patch.object(pygame.event, "get", side_effect=events), \
             mock.patch.object(pygame.font, "SysFont", side_effect=font_factory), \
             mock.patch.object(pygame.display, "flip", side_effect=flip), \
             mock.patch.object(serial, "Serial", return_value=port) as open_port, \
             mock.patch.object(xbox_control, "ManualSession", return_value=session), \
             mock.patch.object(xbox_control, "time", clock):
            result = xbox_control.main((["--dry-run"] if dry_run else []) + ["--log-dir", log_dir])
            log = next(Path(log_dir).glob("*.log")).read_text(encoding="utf-8")
        if dry_run:
            open_port.assert_not_called()
            self.assertEqual(writes, [])
        else:
            open_port.assert_called_once_with("COM9", 115200, timeout=0, write_timeout=0.1)
            self.assertTrue(port.closed)
        return result, writes, frames, processed, log

    def test_freezes_visible_values_and_scroll_only_then_redraws_latest_immediately(self):
        result, writes, frames, processed, log = self.run_ui()
        self.assertEqual(result, 0)
        frozen = [rows for at, rows in frames if 0.8 <= at < 1.3]
        self.assertTrue(frozen)
        for rows in frozen:
            self.assertIn("DISPLAY FROZEN", rows[0])
            view = "\n".join(rows)
            self.assertIn("heading=10", view)
            self.assertIn("physical pitch/PITCH=1.0", view)
            self.assertIn("Current carriage_steps=100", view)
            self.assertIn("Telemetry marker 10", view)
            self.assertNotIn("Telemetry marker 20", view)
            self.assertNotIn("Telemetry marker 30", view)
        # Even receipt-age text stays fixed, not just the numeric values.
        diagnostics = [rows[rows.index(next(row for row in rows if row.startswith("Firmware:"))):] for rows in frozen]
        self.assertTrue(all(rows == diagnostics[0] for rows in diagnostics))
        latest = next(rows for at, rows in frames if at >= 1.3)
        self.assertIn("DISPLAY LIVE", latest[0])
        self.assertIn("heading=30", "\n".join(latest))
        self.assertIn("Current carriage_steps=300", "\n".join(latest))
        self.assertIn("Telemetry marker 30", "\n".join(latest))
        self.assertTrue(any(1.1 <= at < 1.3 and fields.get("heading") == "30" and carriage == "300"
                            for at, fields, carriage in processed))
        for value in (10, 20, 30):
            self.assertIn(f"RX Telemetry marker {value}", log)
        self.assertEqual(log.count("DISPLAY frozen;"), 1)  # Key repeat did not toggle.
        self.assertEqual(log.count("DISPLAY live;"), 1)
        jogs = [(at, data) for at, data in writes if 0.8 <= at < 1.3 and data.startswith(b"JOG ")]
        self.assertTrue(any(int(data.split()[1]) > 0 for _, data in jogs))
        self.assertTrue(any(int(data.split()[1]) < 0 for _, data in jogs))
        self.assertLess(max(b[0] - a[0] for a, b in zip(jogs, jogs[1:])), 0.03)
        self.assertTrue(any(0.8 <= at < 1.3 and data == b"STATUS\n" for at, data in writes))
        self.assertFalse(any(at < 1.5 and b"STOP" in data for at, data in writes))

    def test_raw_editor_and_submission_remain_live_while_diagnostics_are_frozen(self):
        result, writes, frames, _, log = self.run_ui([
            (0.95, [pygame.event.Event(pygame.TEXTINPUT, text="POSE 20 2 100")]),
            (1.05, [key(pygame.K_RETURN)]),
            (1.15, [key(pygame.K_F2)]),
        ], raw=True)
        self.assertEqual(result, 0)
        self.assertTrue(any(0.95 <= at < 1.05 and "> POSE 20 2 100_" in rows for at, rows in frames))
        self.assertEqual([data for _, data in writes if data.startswith(b"POSE ")], [b"POSE 20 2 100\n"])
        self.assertIn("RAW_TX 'POSE 20 2 100'", log)
        self.assertTrue(any(1.15 <= at < 1.2 and data == b"STOP\n" for at, data in writes))
        self.assertFalse(any(at >= .6 and data.startswith(b"JOG ") for at, data in writes))
        self.assertTrue(all("DISPLAY FROZEN" in rows[0] for at, rows in frames if 0.8 <= at < 1.3))

    def test_stop_abort_and_exit_remain_effective_while_frozen(self):
        for raw in (False, True):
            for event, abort in ((key(pygame.K_SPACE), False), (key(pygame.K_F12), False),
                                 (key(pygame.K_x), True),
                                 (pygame.event.Event(pygame.JOYBUTTONDOWN, instance_id=42, button=1), True),
                                 (key(pygame.K_ESCAPE), False)):
                with self.subTest(raw=raw, event=event):
                    result, writes, _, _, _ = self.run_ui([(1.0, [event])], raw=raw)
                    self.assertEqual(result, int(abort and event.type == pygame.KEYDOWN))
                    self.assertTrue(any(1.0 <= at < 1.03 and
                                        (data == b"X\n" if abort else b"STOP" in data) for at, data in writes))
                    self.assertFalse(any(at >= 1.0 and data.startswith(b"JOG ") for at, data in writes))

    def test_focus_change_does_not_stop_frozen_dashboard_or_generate_input(self):
        focus_events = [pygame.event.Event(pygame.WINDOWFOCUSLOST),
                        pygame.event.Event(pygame.JOYAXISMOTION, instance_id=42, axis=0, value=1.0),
                        pygame.event.Event(pygame.WINDOWFOCUSGAINED)]
        for raw in (False, True):
            with self.subTest(raw=raw):
                result, writes, _, _, _ = self.run_ui([(1.0, focus_events)], raw=raw)
                self.assertEqual(result, 0)
                self.assertFalse(any(1.0 <= at < 1.3 and data.strip() == b"STOP" for at, data in writes))
                if not raw:
                    self.assertTrue(any(at >= 1.0 and data.startswith(b"JOG ") for at, data in writes))

    def test_dry_run_freeze_never_opens_serial(self):
        result, _, frames, _, _ = self.run_ui(dry_run=True)
        self.assertEqual(result, 0)
        self.assertTrue(any("DISPLAY FROZEN" in rows[0] for _, rows in frames))


if __name__ == "__main__":
    unittest.main()
