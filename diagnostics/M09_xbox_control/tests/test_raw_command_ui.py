"""Headless raw-command UI through one fake serial connection; no hardware I/O."""
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


def key(value):
    return pygame.event.Event(pygame.KEYDOWN, key=value)


def text(value):
    return pygame.event.Event(pygame.TEXTINPUT, text=value)


def button(value):
    return pygame.event.Event(pygame.JOYBUTTONDOWN, instance_id=42, button=value)


class RawCommandUiTests(unittest.TestCase):
    def run_ui(self, actions, *, until=2.0, dry_run=False, rearm=False, stop_delay=0.2):
        class FakeTime:
            now = 0.0
            def monotonic(self):
                return self.now
            def sleep(self, duration):
                self.now += duration

        clock = FakeTime()
        writes = []
        raw_writes = []
        session = xbox_control.ManualSession()

        class FakeSerial:
            rx = b""
            active = False
            busy = False
            closed = False
            ready_at = None
            @property
            def in_waiting(self):
                return len(self.rx)
            def read(self, length):
                data, self.rx = self.rx[:length], self.rx[length:]
                return data
            def write(self, data):
                writes.append((clock.now, data))
                if data.strip() == b"STOP":
                    self.active = False
                    self.busy = True
                    self.ready_at = clock.now + stop_delay
                elif data == b"STATUS\n":
                    self.rx += b"M09 BUSY\n" if self.busy else (b"M09 MANUAL\n" if self.active else b"M09 READY\n")
                elif data.startswith(b"JOG ") and not session.raw_mode:
                    if not self.active and data == b"JOG 0 0 0\n":
                        self.active = True
                        self.rx += b"MANUAL READY: fixture\n"
                elif data not in (b"X\n", b"\nSTOP\n"):
                    raw_writes.append((clock.now, data))
                    if data.startswith(b"POSE "):
                        self.busy = True
                        self.ready_at = clock.now + 0.25
                        self.rx += b"POSE ACCEPTED fixture\nM09 BUSY\n"
                    else:
                        self.rx += b"POSE REJECTED: fixture firmware syntax check\nM09 READY\n"
                return len(data)
            def close(self):
                self.closed = True

        port = FakeSerial()
        joystick = mock.Mock()
        joystick.get_numaxes.return_value = 6
        joystick.get_instance_id.return_value = 42
        joystick.get_name.return_value = "Simulated Xbox raw console"
        def axis(index):
            if index == 2 and 0.72 < clock.now < (1.35 if rearm else 0.9):
                return 0.7
            return 0.0
        joystick.get_axis.side_effect = axis
        schedule = [(0.9, [key(pygame.K_F2)])] + list(actions)
        schedule.sort(key=lambda action: action[0])
        sent = set()
        def events():
            if port.ready_at is not None and clock.now >= port.ready_at:
                port.ready_at = None
                port.busy = False
                port.rx += b"POSE_STATE active=NO carriage_steps=-154 target_steps=-154 carriage_motor=STOPPED\nM09 READY\n"
            result = []
            for index, (at, values) in enumerate(schedule):
                if index not in sent and clock.now >= at:
                    sent.add(index)
                    result.extend(values)
            if clock.now >= until:
                result.append(pygame.event.Event(pygame.QUIT))
            return result

        with tempfile.TemporaryDirectory() as log_dir, \
             mock.patch.object(pygame.joystick, "get_count", return_value=1), \
             mock.patch.object(pygame.joystick, "Joystick", return_value=joystick), \
             mock.patch.object(pygame.event, "get", side_effect=events), \
             mock.patch.object(serial, "Serial", return_value=port) as open_port, \
             mock.patch.object(xbox_control, "ManualSession", return_value=session), \
             mock.patch.object(xbox_control, "time", clock):
            result = xbox_control.main((["--dry-run"] if dry_run else []) + ["--log-dir", log_dir])
            files = list(Path(log_dir).glob("*.log"))
            self.assertEqual(len(files), 1)
            log = files[0].read_text(encoding="utf-8")
        if dry_run:
            open_port.assert_not_called()
            self.assertEqual(writes, [])
        else:
            open_port.assert_called_once_with("COM9", 115200, timeout=0, write_timeout=0.1)
            self.assertTrue(port.closed)
            self.assertTrue(any(data.startswith(b"JOG ") and data != b"JOG 0 0 0\n"
                                for stamp, data in writes if stamp < 0.9))
        self.assertIn("SESSION exit result=", log)
        return result, writes, raw_writes, session, log

    def assert_no_jog_after(self, writes, after=0.9):
        self.assertFalse(any(stamp >= after and data.startswith(b"JOG ") for stamp, data in writes))

    def test_same_port_stop_handshake_exact_pose_and_no_manual_resume(self):
        command = "POSE  301.800 -9.989 -154"
        result, writes, raw, session, log = self.run_ui([
            (0.95, [text(command), key(pygame.K_RETURN)]),
            (1.25, [key(pygame.K_RETURN)]),
        ])
        self.assertEqual(result, 0)
        self.assertEqual([data for _, data in raw], [(command + "\n").encode()])
        self.assertGreaterEqual(raw[0][0], 1.25)
        self.assertTrue(any(0.9 <= stamp < 0.95 and data == b"STOP\n" for stamp, data in writes))
        self.assert_no_jog_after(writes)
        self.assertTrue(session.raw_mode)
        self.assertEqual(session.last_raw_command, command)
        self.assertEqual(session.carriage_steps, "-154")
        self.assertIsNotNone(session.carriage_received_at)
        self.assertIn("RAW_TX_ATTEMPT " + repr((command + "\n").encode()), log)
        self.assertIn("RAW_TX " + repr(command), log)
        self.assertIn("POSE_STATE active=NO carriage_steps=-154", log)

    def test_early_enter_is_not_queued_for_later_ready(self):
        _, writes, raw, session, _ = self.run_ui([(0.95, [text("POSE 20 8 0"), key(pygame.K_RETURN)])])
        self.assertEqual(raw, [])
        self.assertEqual(session.raw_text, "POSE 20 8 0")
        self.assert_no_jog_after(writes)

    def test_raw_submission_is_verbatim_and_firmware_checks_syntax(self):
        for command in ("JOG nan 0 0", "  MOVE  +1.00 -0.50 -154  ", "mystery café " + "7" * 160):
            with self.subTest(command=command):
                result, writes, raw, session, log = self.run_ui([
                    (1.15, [text(command), key(pygame.K_RETURN)]),
                ])
                self.assertEqual(result, 0)
                self.assertEqual([data for _, data in raw], [(command + "\n").encode("utf-8")])
                self.assertTrue(session.raw_mode)
                self.assertIn("RAW_TX_ATTEMPT " + repr((command + "\n").encode("utf-8")), log)
                # The explicitly entered malformed JOG is permitted; no periodic
                # manual stream can appear around it or after firmware rejection.
                self.assertEqual([(at, data) for at, data in writes if at >= 0.9 and data.startswith(b"JOG ")],
                                 raw if command.startswith("JOG ") else [])

    def test_tab_and_backspace_edit_while_space_stops_without_inserting(self):
        result, writes, raw, _, _ = self.run_ui([
            (1.12, [text("POSE")]),
            (1.15, [key(pygame.K_SPACE), text(" ")]),
            (1.18, [key(pygame.K_TAB), text("20"), key(pygame.K_TAB), text("80"),
                    key(pygame.K_BACKSPACE), key(pygame.K_TAB), text("-154")]),
            (1.5, [key(pygame.K_RETURN)]),
        ])
        self.assertEqual(result, 0)
        self.assertEqual([data for _, data in raw], [b"POSE 20 8 -154\n"])
        self.assertTrue(any(1.15 <= at < 1.2 and data == b"STOP\n" for at, data in writes))
        self.assert_no_jog_after(writes)

    def test_each_operator_status_is_logged_despite_background_status_sampling(self):
        result, _, _, session, log = self.run_ui([
            (1.15, [text("STATUS"), key(pygame.K_RETURN)]),
            (1.25, [text("STATUS"), key(pygame.K_RETURN)]),
        ])
        self.assertEqual(result, 0)
        self.assertTrue(session.raw_mode)
        self.assertEqual(log.count("RAW_TX_ATTEMPT " + repr(b"STATUS\n")), 2)
        self.assertEqual(log.count("RAW_TX " + repr("STATUS")), 2)

    def test_global_stop_beats_enter_in_the_same_event_batch(self):
        for stop in (key(pygame.K_SPACE), key(pygame.K_F12), key(pygame.K_F2)):
            with self.subTest(stop=stop):
                _, writes, raw, _, _ = self.run_ui([
                    (1.15, [text("POSE 20 8 -154"), key(pygame.K_RETURN), stop]),
                ])
                self.assertEqual(raw, [])
                self.assertTrue(any(1.15 <= at < 1.2 and data == b"STOP\n" for at, data in writes))
                if stop.type == pygame.KEYDOWN and stop.key == pygame.K_F2:
                    # This case leaves the editor with centered sticks; preserve
                    # the existing automatic handshake after a fresh half second.
                    self.assertFalse(any(.9 <= at < 1.65 and data.startswith(b"JOG ") for at,data in writes))
                else:
                    self.assert_no_jog_after(writes)

    def test_global_abort_beats_enter_and_writes_latched_abort(self):
        for abort in (key(pygame.K_x), button(1)):
            with self.subTest(abort=abort):
                result, writes, raw, _, log = self.run_ui([
                    (1.15, [text("POSE 20 8 -154"), key(pygame.K_RETURN), abort]),
                ])
                self.assertEqual(result, 1)
                self.assertEqual(raw, [])
                self.assertEqual(writes[-1][1], b"X\n")
                self.assertIn("TX_ATTEMPT X", log)

    def test_exit_and_focus_loss_win_over_enter_without_automatic_submission(self):
        for event in (key(pygame.K_ESCAPE), pygame.event.Event(pygame.QUIT),
                      pygame.event.Event(pygame.WINDOWFOCUSLOST)):
            with self.subTest(event=event):
                result, writes, raw, _, _ = self.run_ui([
                    (1.15, [text("POSE 20 8 -154"), key(pygame.K_RETURN), event]),
                    (1.5, [pygame.event.Event(pygame.WINDOWFOCUSGAINED)]),
                ])
                self.assertEqual(result, 0)
                self.assertEqual(raw, [])
                self.assertEqual(writes[-1][1], b"\nSTOP\n")
                self.assert_no_jog_after(writes)

        # Lose focus after firmware has accepted a POSE and is still BUSY.
        # This must cancel motion even though no manual JOG lease is active.
        result, writes, raw, session, _ = self.run_ui([
            (1.15, [text("POSE 20 8 -154"), key(pygame.K_RETURN)]),
            (1.25, [pygame.event.Event(pygame.WINDOWFOCUSLOST)]),
            (1.6, [pygame.event.Event(pygame.WINDOWFOCUSGAINED)]),
        ])
        self.assertEqual(result, 0)
        self.assertEqual([data for _, data in raw], [b"POSE 20 8 -154\n"])
        self.assertTrue(any(1.25 <= at < 1.3 and data == b"STOP\n" for at, data in writes))
        self.assertTrue(session.raw_mode)
        self.assert_no_jog_after(writes)

    def test_leaving_editor_requires_new_centered_half_second(self):
        _, writes, _, session, _ = self.run_ui([
            (1.15, [key(pygame.K_F2)]),
            (1.5, [key(pygame.K_SPACE)]),  # Not yet centered for half a second.

        ], until=2.3, rearm=True)
        self.assertFalse(session.raw_mode)
        self.assertFalse(any(0.9 <= stamp < 2.0 and data.startswith(b"JOG ") for stamp, data in writes))
        self.assertTrue(any(stamp >= 2.0 and data == b"JOG 0 0 0\n" for stamp, data in writes))

    def test_raw_dry_run_never_opens_port(self):
        result, _, _, _, log = self.run_ui([
            (1.15, [text("POSE 20 8 -154"), key(pygame.K_RETURN)]),
        ], dry_run=True)
        self.assertEqual(result, 0)
        self.assertNotIn("SERIAL opened", log)


if __name__ == "__main__":
    unittest.main()
