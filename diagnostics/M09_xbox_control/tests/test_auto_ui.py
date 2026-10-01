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
            axes = [0.0] * 6
            completion = None
            outcome = "PASS"
            motion_duration = .2
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
                elif words[0] in ("KEYMOVE", "KEYRETURN"):
                    self.mode = "BUSY"
                    self.rx += f"KEYMOVE ACCEPTED id={words[1]} epoch=42 duration_ms={words[6] if len(words) == 7 else 200}\nM09 BUSY\n".encode()
                    self.completion = (clock.now + .2, words[1], tuple(int(value) for value in words[3:6]))
                elif words[0] in ("LEVEL", "NORTH"):
                    self.mode = "BUSY"
                    self.rx += f"{words[0]} ACCEPTED deadline_ms=90000\nM09 BUSY\n".encode()
                    self.completion = (clock.now + self.motion_duration, words[0], self.positions)
                elif words[0] in ("MOVE", "POSE"):
                    self.mode = "BUSY"
                    self.rx += b"POSE ACCEPTED: fixture\nM09 BUSY\n"
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
        joystick.get_axis.side_effect = lambda index: port.axes[index]
        schedule = list(actions)
        schedule.sort(key=lambda item: item[0])
        sent = set()

        def events():
            if port.completion is not None and clock.now >= port.completion[0]:
                _, ident, port.positions = port.completion
                port.completion = None
                port.mode = "READY"
                if ident in ("LEVEL", "NORTH"):
                    port.rx += f"{ident} RESULT: {port.outcome}\nFINAL RESULT: {port.outcome}\n".encode()
                else:
                    port.rx += (f"KEYMOVE RESULT id={ident} epoch=42 status=PASS yaw_steps={port.positions[0]} "
                                f"pitch_steps={port.positions[1]} carriage_steps={port.positions[2]} elapsed_ms=10000 concurrent_axes=3\n".encode()
                                if ident else b"FINAL RESULT: PASS\n")
                port.rx += b"M09 READY\n"
                if port.outcome != "PASS":
                    port.rx += b"OPERATION FAILED: BNO recovery expired\n"
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

    def test_capture_manual_reposition_and_play_from_arbitrary_position(self):
        result, writes, session, auto, _, log = self.run_ui([
            (.8, tap(4)), (1.0, lambda port: setattr(port, "positions", (112, -191, 318))),
            (1.1, tap(5)), (1.8, tap(10))], until=3.5)
        self.assertEqual(result, 0)
        commands = [data for _, data in writes if data.startswith((b"KEYMOVE", b"KEYRETURN"))]
        self.assertEqual(commands, [b"KEYRETURN 4 42 100 -200 300\n", b"KEYMOVE 6 42 112 -191 318 10000\n"])
        self.assertEqual(auto.frames["A"].steps, (100, -200, 300))
        self.assertEqual(auto.frames["B"].steps, (112, -191, 318))
        self.assertTrue(any(data.startswith(b"JOG") for _, data in writes))
        self.assertFalse(any(1.8 <= at <= 2.4 and data.startswith(b"JOG") for at, data in writes))
        self.assertIn("KEYMOVE RESULT", log)

    def test_manual_movement_between_captures_preserves_a_and_b(self):
        def move(port):
            port.axes[1] = -.8
            port.positions = (100, -230, 300)  # Generated-position fixture response.
        def center(port):
            port.axes[1] = 0
        result, writes, _, auto, _, _ = self.run_ui([
            (.8, tap(4)), (1.4, move), (1.6, center), (1.7, tap(5))])
        self.assertEqual(result, 0)
        self.assertTrue(any(1.4 < at < 1.6 and data.startswith(b"JOG 0 585 0") for at, data in writes))
        self.assertEqual(auto.frames["A"].steps, (100, -200, 300))
        self.assertEqual(auto.frames["B"].steps, (100, -230, 300))

    def test_stick_override_return_and_play_resumes_manual_without_centering(self):
        for offset in (True, False):
            with self.subTest(returning=offset):
                actions = [(.8, tap(4))]
                if offset:
                    actions.append((1.0, lambda port: setattr(port, "positions", (112, -191, 318))))
                actions += [(1.1, tap(5)), (1.8, tap(10)),
                            (1.9, lambda port: port.axes.__setitem__(1, -.8))]
                result, writes, session, auto, _, _ = self.run_ui(actions, until=2.4)
                self.assertEqual(result, 0)
                self.assertTrue(any(at >= 1.9 and data == b"STOP\n" for at, data in writes))
                self.assertTrue(any(1.9 < at < 2.1 and data.startswith(b"JOG 0 585 0") for at, data in writes))
                self.assertEqual(len(auto.frames), 2)
                self.assertEqual(len([data for _, data in writes if data.startswith((b"KEYMOVE", b"KEYRETURN"))]), 1)

    def test_capture_while_stick_deflected_stops_before_snapshot(self):
        result, writes, _, auto, _, _ = self.run_ui([
            (.7, lambda port: port.axes.__setitem__(1, -.8)), (.8, tap(4))])
        self.assertEqual(result, 0)
        self.assertIn("A", auto.frames)
        before = [data for at, data in writes if .79 < at < 1]
        self.assertEqual(before[:2], [b"STOP\n", b"SNAP 1\n"])
        self.assertFalse(any(data.startswith(b"JOG") for at, data in writes if at >= .8))

    def test_deadband_noise_does_not_override_play(self):
        result, writes, _, auto, _, _ = self.run_ui([
            (.8, tap(4)), (1.1, tap(5)), (1.8, tap(10)),
            (1.9, lambda port: port.axes.__setitem__(1, .14))])
        self.assertEqual(result, 0)
        self.assertFalse(any(1.9 <= at < 2.2 and data == b"STOP\n" for at, data in writes))
        self.assertIn("A", auto.frames)

    def test_stop_and_abort_win_over_play_in_same_batch(self):
        for safety in (key(pygame.K_SPACE), key(pygame.K_F12), key(pygame.K_F2),
                       button(1), key(pygame.K_x), pygame.event.Event(pygame.WINDOWFOCUSLOST)):
            with self.subTest(safety=safety):
                _, writes, _, _, _, _ = self.run_ui([
                    (.8, tap(4)), (1.1, tap(5)), (1.8, tap(10) + [safety])])
                self.assertFalse(any(data.startswith((b"KEYMOVE", b"KEYRETURN")) for _, data in writes))
                self.assertEqual(len([data for _, data in writes if data.startswith(b"SNAP")]), 2)
                self.assertTrue(any(at >= 1.8 and data.strip() in (b"STOP", b"X") for at, data in writes))

    def test_raw_mode_disables_keyframe_buttons(self):
        _, writes, session, auto, _, _ = self.run_ui([
            (.8, tap(4)), (1.0, [key(pygame.K_F2)]),
            (1.1, tap(3) + tap(5) + tap(2) + tap(10) + [hat((0, 1))])])
        self.assertTrue(session.raw_mode)
        self.assertEqual(auto.duration, 10)
        self.assertEqual(len([data for _, data in writes if data.startswith(b"SNAP")]), 1)
        self.assertFalse(any(at >= 1 and data.startswith((b"JOG", b"MOVE", b"KEYMOVE")) for at, data in writes))

    def test_f3_keeps_action_status_and_capture_live(self):
        _, _, _, auto, frames, log = self.run_ui([
            (.7, [key(pygame.K_F3)]), (.8, tap(4)), (1.1, tap(5)), (1.8, tap(10))])
        frozen = ["\n".join(rows) for stamp, rows in frames if stamp > 1.85]
        self.assertTrue(any("DISPLAY FROZEN" in rows and "KEYMOVE_ACTIVE" in rows for rows in frozen))
        self.assertTrue(any("A=(100, -200, 300)" in rows for rows in frozen))
        self.assertIn("KEYMOVE RESULT", log)

    def test_timeout_stops_and_does_not_retry(self):
        _, writes, _, auto, _, _ = self.run_ui([(.8, tap(4))], until=5.5, snapshots=False)
        self.assertEqual([data for _, data in writes if data.startswith(b"SNAP")], [b"SNAP 1\n"])
        self.assertTrue(any(4.7 < at < 5.2 and data == b"STOP\n" for at, data in writes))
        self.assertFalse(auto.frames)

    def test_release_required_duration_home_capture_and_hat(self):
        result, writes, _, auto, frames, _ = self.run_ui([
            (.7, [button(2)]), (.75, [button(2)]), (.8, [button(4)]), (.9, [button(4)]),
            (1.1, tap(5)), (1.8, [button(10)]), (1.9, [button(10)]),
            (2.2, [button(10)]), (2.5, [hat((0, 1))]), (2.55, [hat((0, 1))])], until=3)
        self.assertEqual(result, 0)
        self.assertEqual(auto.duration, 20)
        self.assertEqual(len([data for _, data in writes if data.startswith(b"KEYMOVE")]), 1)
        self.assertEqual(len([data for _, data in writes if data.startswith(b"MOVE")]), 1)
        self.assertEqual(len([data for _, data in writes if data.startswith(b"SNAP")]), 3)
        self.assertIn("Home (10): Play A->B", "\n".join(frames[-1][1]))

    def test_increment_duration_diagonals_and_cardinals(self):
        _, writes, _, auto, _, log = self.run_ui([
            (.7, tap(9) + tap(2)), (.8, [hat((1, 1))]),
            (1.0, [hat((0, 0)), hat((1, 0))])])
        self.assertFalse(any(data in (b"LEVEL\n", b"NORTH\n") for _, data in writes))
        self.assertEqual([data for _, data in writes if data.startswith(b"MOVE")], [b"MOVE 2 0 0\n"])
        self.assertEqual(auto.duration, 20)
        self.assertIn("Controller hat 0: (1, 1)", log)
        _, _, _, _, frames, log = self.run_ui([(.7, [hat((0, 1))])], dry_run=True)
        self.assertIn("AUTO_TX", log)
        self.assertTrue(any(any("Controller hat 0: (0, 1)" in row for row in rows) for _, rows in frames))

    def test_a_level_y_north_are_single_edge_requests_and_diagonals_do_nothing(self):
        _, writes, _, _, frames, _ = self.run_ui([
            (.7, [button(0)]), (.72, [button(0)]), (.9, [button(0, True)]),
            (1.3, [button(3)]), (1.32, [button(3)]), (1.5, [button(3, True)]),
            (1.9, [hat((1, 1))]), (2.0, [hat((0, 0))]),
            (2.1, [hat((-1, 1))]), (2.2, [hat((0, 0))]),
            (2.3, [hat((1, -1))]), (2.4, [hat((0, 0))]),
            (2.5, [hat((-1, -1))])], until=2.9)
        self.assertEqual([data for _, data in writes if data in (b"LEVEL\n", b"NORTH\n")], [b"LEVEL\n", b"NORTH\n"])
        self.assertEqual([data for at, data in writes if .69 <= at < .9 and data == b"STOP\n"], [b"STOP\n"])
        self.assertIn("A (0): LEVEL pitch", "\n".join(frames[-1][1]))

    def test_alignment_and_keyframe_sequences_return_to_manual_without_operator_rearm(self):
        for sequence in (("play",), ("level",), ("north",), ("level","play"), ("north","play"),
                         ("play","level"), ("level","north"), ("north","level"), ("level","north","level","north")):
            with self.subTest(sequence=sequence):
                actions = [(.8, tap(4)), (1.4, tap(5))]
                for index, action in enumerate(sequence):
                    event = {"level": tap(0), "north": tap(3), "play": tap(10)}[action]
                    actions.append((2.0 + index * .8, event))
                manual_at = 2.0 + len(sequence)*.8
                actions.append((manual_at, lambda port: port.axes.__setitem__(1, -.8)))
                result, writes, _, auto, _, _ = self.run_ui(actions, until=manual_at+.4)
                self.assertEqual(result, 0)
                expected = [{"level": b"LEVEL", "north": b"NORTH", "play": b"KEYMOVE"}[action] for action in sequence]
                actual = [data.split()[0] for _,data in writes if data.split()[0] in (b"LEVEL",b"NORTH",b"KEYMOVE")]
                self.assertEqual(actual, expected)
                self.assertEqual(len(auto.frames), 2)
                self.assertTrue(any(at>=manual_at and data==b"JOG 0 585 0\n" for at,data in writes))
                self.assertFalse(any(data==b"X\n" for _,data in writes))

    def test_deliberate_sticks_take_over_alignment_and_raw_autonomous_then_actions_still_work(self):
        for request in ("LEVEL", "NORTH", "POSE 290 -40 300", "MOVE 0 1 0", "KEYMOVE 9 42 101 -202 303 5000"):
            with self.subTest(request=request):
                actions = [(.8,tap(4)), (1.4,tap(5))]
                if request in ("LEVEL","NORTH"):
                    actions.append((2.0, tap(0) if request=="LEVEL" else tap(3)))
                else:
                    actions += [(1.9,[key(pygame.K_F2)]), (2.0,[pygame.event.Event(pygame.TEXTINPUT,text=request),key(pygame.K_RETURN)])]
                actions += [(2.1,lambda port: port.axes.__setitem__(1,-.8)),
                            (2.4,lambda port: port.axes.__setitem__(1,0)), (3.0,tap(0)), (3.8,tap(10))]
                result,writes,session,auto,_,_=self.run_ui(actions,until=4.6)
                self.assertEqual(result,0)
                self.assertFalse(session.raw_mode)
                self.assertTrue(any(2.1<=at<2.3 and data==b"STOP\n" for at,data in writes))
                self.assertTrue(any(2.1<at<2.3 and data==b"JOG 0 585 0\n" for at,data in writes))
                self.assertTrue(any(at>=3 and data==b"LEVEL\n" for at,data in writes))
                self.assertTrue(any(at>=3.8 and data.startswith(b"KEYMOVE ") for at,data in writes))
                self.assertEqual(len(auto.frames),2)
                self.assertFalse(any(data==b"X\n" for _,data in writes))

    def test_sensor_failure_completion_releases_manual_and_keyframes(self):
        for request in (tap(0), tap(3)):
            with self.subTest(request=request):
                actions=[(.8,tap(4)),(1.4,tap(5)),(1.9,lambda port:setattr(port,"outcome","STOPPED")),
                         (2.0,request), (2.8,lambda port:port.axes.__setitem__(1,-.8)),
                         (3.0,lambda port:port.axes.__setitem__(1,0)),
                         (3.4,lambda port:setattr(port,"outcome","PASS")), (3.6,tap(10))]
                result,writes,_,auto,_,_=self.run_ui(actions,until=4.4)
                self.assertEqual(result,0)
                self.assertTrue(any(2.8<at<3 and data==b"JOG 0 585 0\n" for at,data in writes))
                self.assertTrue(any(at>=3.6 and data.startswith(b"KEYMOVE ") for at,data in writes))
                self.assertEqual(len(auto.frames),2)

    def test_stop_and_abort_take_priority_over_alignment_buttons(self):
        for safety in (key(pygame.K_SPACE),key(pygame.K_F12),button(1),key(pygame.K_x)):
            with self.subTest(safety=safety):
                _,writes,_,_,_,_=self.run_ui([(.8,tap(0)+tap(3)+[safety])])
                self.assertFalse(any(data in (b"LEVEL\n",b"NORTH\n") for _,data in writes))


if __name__ == "__main__":
    unittest.main()
