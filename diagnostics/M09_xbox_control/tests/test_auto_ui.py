"""Xbox AUTO integration using dummy SDL and a single simulated serial port."""
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


def button(value, up=False):
    return pygame.event.Event(pygame.JOYBUTTONUP if up else pygame.JOYBUTTONDOWN, instance_id=42, button=value)


def tap(value):
    return [button(value), button(value, True)]


def key(value):
    return pygame.event.Event(pygame.KEYDOWN, key=value)


def hat(value):
    return pygame.event.Event(pygame.JOYHATMOTION, instance_id=42, hat=0, value=value)


class AutoUiTests(unittest.TestCase):
    def run_ui(self, actions=(), *, until=2.5, dry_run=False, snapshots=True):
        class Clock:
            now = 0.0
            def monotonic(self):
                return self.now
            def sleep(self, amount):
                self.now += amount

        clock = Clock()
        session = xbox_control.ManualSession()
        auto = xbox_control.AutoSession()
        writes, frames, rendered = [], [], []

        class Port:
            rx = b""
            mode = "READY"
            positions = (100, -200, 300)
            completion = None
            closed = False
            @property
            def in_waiting(self):
                return len(self.rx)
            def read(self, size):
                data, self.rx = self.rx[:size], self.rx[size:]
                return data
            def write(self, data):
                writes.append((clock.now, data))
                words = data.decode("ascii").split()
                if words[0] == "STOP":
                    self.completion = None
                    self.mode = "READY"
                    self.rx += b"M09 READY\n"
                elif words[0] == "STATUS":
                    self.rx += f"M09 {self.mode}\n".encode()
                elif words[0] == "SNAP" and snapshots:
                    self.rx += (f"KEYFRAME_SNAPSHOT id={words[1]} epoch=42 yaw_steps={self.positions[0]} "
                                f"pitch_steps={self.positions[1]} carriage_steps={self.positions[2]} "
                                "heading=290 physical_pitch=-40 bno_valid=YES bno_age_ms=10 accuracy=1 north_usable=NO unhomed=YES\n").encode()
                elif words[0] == "KEYMOVE":
                    self.mode = "BUSY"
                    self.rx += f"KEYMOVE ACCEPTED id={words[1]} epoch=42 duration_ms={words[6]}\nM09 BUSY\n".encode()
                    self.completion = (clock.now + .2, words[1], tuple(int(value) for value in words[3:6]))
                elif words[0] == "MOVE":
                    self.mode = "BUSY"
                    self.rx += b"PITCH-ONLY ACCEPTED: fixture\nM09 BUSY\n"
                    self.completion = (clock.now + .2, None, self.positions)
                elif words[0] == "JOG" and self.mode == "READY":
                    self.mode = "MANUAL"
                    self.rx += b"MANUAL READY: fixture\n"
                return len(data)
            def close(self):
                self.closed = True

        port = Port()
        joystick = mock.Mock()
        joystick.get_numaxes.return_value = 6
        joystick.get_instance_id.return_value = 42
        joystick.get_name.return_value = "Simulated Xbox AUTO"
        joystick.get_axis.return_value = 0.0
        schedule = [(0.1, tap(3))] + list(actions)
        schedule.sort(key=lambda item: item[0])
        sent = set()

        def events():
            if port.completion is not None and clock.now >= port.completion[0]:
                _, ident, port.positions = port.completion
                port.completion = None
                port.mode = "READY"
                port.rx += (f"KEYMOVE RESULT id={ident} epoch=42 status=PASS yaw_steps={port.positions[0]} "
                            f"pitch_steps={port.positions[1]} carriage_steps={port.positions[2]} elapsed_ms=10000 concurrent_axes=3\n".encode()
                            if ident else b"FINAL RESULT: PASS\n")
                port.rx += b"M09 READY\n"
            values = []
            for index, (at, items) in enumerate(schedule):
                if clock.now >= at and index not in sent:
                    sent.add(index)
                    if callable(items):
                        items(port)
                    else:
                        values.extend(items)
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
        def flip():
            frames.append((clock.now, list(rendered)))
            rendered.clear()

        with tempfile.TemporaryDirectory() as folder, contextlib.redirect_stdout(io.StringIO()), \
             mock.patch.object(pygame.joystick, "get_count", return_value=1), \
             mock.patch.object(pygame.joystick, "Joystick", return_value=joystick), \
             mock.patch.object(pygame.event, "get", side_effect=events), \
             mock.patch.object(pygame.font, "SysFont", side_effect=make_font), \
             mock.patch.object(pygame.display, "flip", side_effect=flip), \
             mock.patch.object(serial, "Serial", return_value=port) as opener, \
             mock.patch.object(xbox_control, "ManualSession", return_value=session), \
             mock.patch.object(xbox_control, "AutoSession", return_value=auto), \
             mock.patch.object(xbox_control, "time", clock):
            result = xbox_control.main((["--dry-run"] if dry_run else []) + ["--log-dir", folder])
            log = next(Path(folder).glob("*.log")).read_text(encoding="utf-8")
        if dry_run:
            opener.assert_not_called()
            self.assertEqual(writes, [])
        else:
            opener.assert_called_once()
            self.assertTrue(port.closed)
        return result, writes, session, auto, frames, log

    def test_capture_return_play_single_connection_and_no_jog(self):
        result, writes, session, auto, _, log = self.run_ui([
            (.8, tap(4)), (1.0, lambda port: setattr(port, "positions", (112, -191, 318))),
            (1.05, tap(5)), (1.2, tap(8)), (1.3, tap(10)), (1.6, tap(10))])
        self.assertEqual(result, 0)
        commands = [data for _, data in writes if data.startswith(b"KEYMOVE")]
        self.assertEqual(commands, [b"KEYMOVE 4 42 100 -200 300 10000\n", b"KEYMOVE 6 42 112 -191 318 10000\n"])
        self.assertFalse(any(data.startswith(b"JOG") for _, data in writes))
        self.assertEqual(auto.frames["A"].steps, (100, -200, 300))
        self.assertEqual(auto.frames["B"].steps, (112, -191, 318))
        self.assertTrue(session.auto_mode)
        self.assertIn("AUTO_NOT_SENT", log)
        self.assertIn("AUTO_TX b'KEYMOVE 6 42 112 -191 318 10000\\n'", log)

    def test_held_and_busy_buttons_and_hat_do_not_repeat_or_queue(self):
        _, writes, _, auto, _, _ = self.run_ui([
            (.8, [button(4)]), (.9, [button(4)]), (1.0, [button(4)]),
            (1.1, [hat((0, 1))]), (1.15, [hat((0, 1))]),
            (1.2, [hat((0, 0)), hat((0, -1))]), (1.5, [hat((0, -1))]),
            (1.6, [hat((0, 0)), hat((0, -1))])])
        self.assertEqual([data for _, data in writes if data.startswith(b"SNAP")], [b"SNAP 1\n"])
        self.assertEqual([data for _, data in writes if data.startswith(b"MOVE")], [b"MOVE 0 1 0\n", b"MOVE 0 -1 0\n"])
        self.assertEqual(auto.phase, "READY")

    def test_stop_and_abort_win_over_play_in_same_batch(self):
        for safety in (key(pygame.K_SPACE), button(0), key(pygame.K_F12), key(pygame.K_F2),
                       button(1), key(pygame.K_x), pygame.event.Event(pygame.WINDOWFOCUSLOST)):
            with self.subTest(safety=safety):
                _, writes, _, auto, _, _ = self.run_ui([
                    (.8, tap(4)), (1.0, tap(5)), (1.2, tap(10) + [safety])])
                self.assertFalse(any(data.startswith(b"KEYMOVE") for _, data in writes))
                self.assertEqual(len([data for _, data in writes if data.startswith(b"SNAP")]), 2)
                self.assertTrue(any(at >= 1.2 and data.strip() in (b"STOP", b"X") for at, data in writes))

    def test_auto_toggle_does_not_arm_and_raw_mode_disables_auto_buttons(self):
        _, writes, session, auto, _, _ = self.run_ui([
            (.8, tap(4)), (1.0, [key(pygame.K_F2)]),
            (1.1, tap(3) + tap(5) + tap(2) + tap(10) + [hat((0, 1))]), (1.4, [key(pygame.K_F2)])])
        self.assertFalse(auto.enabled)
        self.assertFalse(session.auto_mode)
        self.assertFalse(session.raw_mode)
        self.assertEqual(session.state, "idle")
        self.assertEqual(len([data for _, data in writes if data.startswith(b"SNAP")]), 1)
        self.assertFalse(any(data.startswith((b"JOG", b"MOVE")) for _, data in writes))

    def test_f3_keeps_auto_status_and_capture_live(self):
        _, _, _, auto, frames, log = self.run_ui([
            (.7, [key(pygame.K_F3)]), (.8, tap(4)), (1.0, tap(5)), (1.2, tap(10))])
        frozen = ["\n".join(rows) for stamp, rows in frames if stamp > 1.25]
        self.assertTrue(any("DISPLAY FROZEN" in rows and "KEYMOVE_ACTIVE" in rows for rows in frozen))
        self.assertTrue(any("A=(100, -200, 300)" in rows for rows in frozen))
        self.assertEqual(auto.phase, "READY")
        self.assertIn("KEYMOVE RESULT", log)

    def test_timeout_stops_and_does_not_retry(self):
        _, writes, _, auto, _, _ = self.run_ui([(.8, tap(4))], until=5.5, snapshots=False)
        self.assertEqual([data for _, data in writes if data.startswith(b"SNAP")], [b"SNAP 1\n"])
        self.assertTrue(any(4.7 < at < 5.2 and data == b"STOP\n" for at, data in writes))
        self.assertEqual(auto.phase, "FAULT")

    def test_controller_x_duration_requires_release_and_is_not_abort(self):
        result, writes, _, auto, frames, _ = self.run_ui([
            (.7, [button(2)]), (.8, [button(2)]), (.9, [button(2)]),
            (1.0, [button(2, True)]), (1.1, tap(2))])
        self.assertEqual(result, 0)
        self.assertEqual(auto.duration, 5)  # 10 -> 20 -> 5, only two presses.
        self.assertFalse(any(data.startswith((b"X", b"MOVE", b"KEYMOVE")) for _, data in writes))
        help_text = "\n".join(frames[-1][1])
        for label in ("X button (2): duration", "Home (10): Play A->B", "keyboard X: abort"):
            self.assertIn(label, help_text)
        self.assertNotIn("View", help_text)
        self.assertNotIn("Menu", help_text)

    def test_home_play_requires_release_and_busy_presses_are_not_queued(self):
        result, writes, _, auto, _, _ = self.run_ui([
            (.8, tap(4)), (1.0, tap(5)), (1.2, [button(10)]),
            (1.25, [button(10)]), (1.28, [button(10, True)]),
            (1.3, [button(10)] + tap(2)),  # Fresh presses while busy are discarded.
            (1.6, [button(10)]), (1.7, [button(10, True)]),
            (1.8, tap(10))])
        self.assertEqual(result, 0)
        self.assertEqual(len([data for _, data in writes if data.startswith(b"KEYMOVE")]), 2)
        self.assertEqual(len([data for _, data in writes if data.startswith(b"SNAP")]), 4)
        self.assertEqual(auto.duration, 10)
        self.assertFalse(any(data.startswith(b"JOG") for _, data in writes))

    def test_unused_buttons_and_manual_x_home_do_not_request_motion(self):
        result, writes, session, auto, _, _ = self.run_ui([
            (.8, tap(4)), (1.0, tap(5)), (1.1, tap(6) + tap(7)),
            (1.2, tap(3)), (1.4, tap(2) + tap(10))])
        self.assertEqual(result, 0)
        self.assertFalse(auto.enabled)
        self.assertEqual(auto.duration, 10)
        self.assertEqual(session.state, "idle")
        self.assertEqual(len([data for _, data in writes if data.startswith(b"SNAP")]), 2)
        self.assertFalse(any(data.startswith((b"KEYMOVE", b"MOVE", b"JOG", b"X")) for _, data in writes))

    def test_increment_duration_diagonals_and_dry_input_diagnostics(self):
        _, writes, _, auto, _, log = self.run_ui([
            (.7, tap(9) + tap(2)), (.8, [hat((1, 1))]),
            (1.0, [hat((0, 0)), hat((1, 0))])])
        self.assertEqual([data for _, data in writes if data.startswith(b"MOVE")], [b"MOVE 2 0 0\n"])
        self.assertEqual(auto.duration, 20)
        self.assertIn("Controller hat 0: (1, 1)", log)
        _, _, _, _, frames, log = self.run_ui([(.7, [hat((0, 1))])], dry_run=True)
        self.assertIn("AUTO_TX b'MOVE 0 1 0\\n'", log)
        self.assertTrue(any(any("Controller hat 0: (0, 1)" in row for row in rows) for _, rows in frames))


if __name__ == "__main__":
    unittest.main()
