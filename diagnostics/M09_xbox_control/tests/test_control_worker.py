"""Scheduler safety tests: no SDL, motors, or actual serial devices."""
import threading
import unittest
from unittest import mock

from auto_control import AutoSession
from control_worker import ControlWorker
from encoder_setup import EncoderSetup
from xbox_control import ManualSession


class Port:
    def __init__(self):
        self.rx = b""
        self.writes = []
        self.owners = set()
        self.fail = False
        self.silent = False
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, size):
        self.owners.add(threading.get_ident())
        if self.fail:
            raise OSError("link lost")
        data, self.rx = self.rx[:size], self.rx[size:]
        return data

    def write(self, data):
        self.owners.add(threading.get_ident())
        self.writes.append(data)
        if data == b"STATUS\n" and not self.silent:
            self.rx += b"M09 BUSY\n"
        return len(data)

    def close(self):
        self.owners.add(threading.get_ident())
        self.closed = True


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.port = Port()
        self.session = ManualSession()
        self.auto = AutoSession()
        self.safety = None
        self.worker = ControlWorker(self.port, self.session, self.auto, EncoderSetup(),
                                    mock.Mock(), clock=lambda: self.now,
                                    safety_input=lambda: self.safety)
        self.addCleanup(self.worker.close)
        self.worker.synchronize()

    def tracking(self):
        with self.worker.transaction():
            self.auto.action = "celestial"
            self.auto.phase = "CELESTIAL_TRACK"
            self.auto.last_rx = self.now
            self.auto.pending_id = 1
            self.auto.celestial = mock.Mock()
            self.auto.celestial.update.return_value = None

    def test_serial_has_one_owner_including_shutdown(self):
        self.worker.send(b"STATUS\n")
        self.worker.synchronize()
        self.worker.close()
        self.assertTrue(self.port.closed)
        self.assertEqual(self.port.owners, {self.worker.thread.ident})
        self.assertNotIn(threading.get_ident(), self.port.owners)

    def test_stop_barrier_discards_unsent_motion(self):
        with self.worker.transaction():
            self.worker.send(b"JOG 100 0 0\n")
            self.worker.send(b"STOP\n")
        self.worker.synchronize()
        self.assertNotIn(b"JOG 100 0 0\n", self.port.writes)
        self.assertIn(b"STOP\n", self.port.writes)
        with self.worker.transaction():
            self.worker.send(b"JOG 200 0 0\n")
            self.worker.send(b"X\n")
            self.session.receive("M09 ABORTED", self.now)
            self.port.rx += b"M09 MANUAL\n"  # Buffered before the operator pressed B.
            self.worker.send(b"STOP\n")
        self.worker.synchronize()
        self.assertNotIn(b"JOG 200 0 0\n", self.port.writes)
        self.assertEqual(self.port.writes[-2:], [b"X\n", b"STOP\n"])

    def test_expired_commands_stop_without_replay(self):
        with self.worker.transaction():
            self.worker.send(b"POSE 10 5 0\n")
            self.now += .3
        self.worker.synchronize()
        self.assertNotIn(b"POSE 10 5 0\n", self.port.writes)
        self.assertIn(b"STOP\n", self.port.writes)
        self.assertEqual(self.session.state, "idle")

    def test_gui_pause_does_not_replay_manual_velocity(self):
        with self.worker.transaction():
            self.session.state = "active"
        self.now += .3
        self.worker.synchronize()
        self.assertEqual(self.session.state, "idle")
        self.assertEqual(self.port.writes.count(b"STOP\n"), 1)
        self.now += .3
        self.worker.synchronize()
        self.assertEqual(self.port.writes.count(b"STOP\n"), 1)

    def test_loss_of_telemetry_still_disarms_tracking(self):
        self.port.silent = True
        self.worker.synchronize()  # Drain any previous STATUS reply first.
        self.tracking()
        self.now += 3.1
        self.worker.synchronize()
        self.assertEqual(self.auto.phase, "FAULT")
        self.assertTrue(self.auto.consume_manual_rearm_required())
        self.assertIn(b"STOP\n", self.port.writes)

    def test_read_failure_ends_owner_and_never_replays_commands(self):
        self.tracking()
        with self.worker.transaction():
            self.port.fail = True
            self.worker.send(b"CELESTIAL_UPDATE 1 2 10 5\n")
        self.worker.thread.join(1)
        self.assertFalse(self.worker.thread.is_alive())
        self.assertTrue(self.port.closed)
        self.assertNotIn(b"CELESTIAL_UPDATE 1 2 10 5\n", self.port.writes)
        with self.assertRaisesRegex(RuntimeError, "link lost"):
            self.worker.send(b"JOG 10 0 0\n")

    def test_firmware_abort_invalidates_queued_motion(self):
        self.tracking()
        with self.worker.transaction():
            self.worker.send(b"CELESTIAL_UPDATE 1 2 10 5\n")
            self.port.rx += b"M09 ABORTED\n"
        self.worker.synchronize()
        self.assertNotIn(b"CELESTIAL_UPDATE 1 2 10 5\n", self.port.writes)
        self.assertEqual(self.auto.phase, "FAULT")

    def test_short_serial_write_is_terminal(self):
        self.tracking()
        with self.worker.transaction():
            self.port.write = mock.Mock(return_value=1)
            self.worker.send(b"CELESTIAL_UPDATE 1 2 10 5\n")
        self.worker.thread.join(1)
        self.assertFalse(self.worker.thread.is_alive())
        self.assertTrue(self.port.closed)
        self.assertIsNone(self.auto.action)
        with self.assertRaisesRegex(RuntimeError, "Incomplete serial write"):
            self.worker.synchronize()

    def test_independent_neutral_takeover_and_b_abort_during_gui_pause(self):
        self.tracking()
        self.now += 1
        self.worker.synchronize()
        self.assertEqual(self.auto.phase, "CELESTIAL_TRACK")
        self.safety = "takeover"
        self.worker.synchronize()
        self.assertIsNone(self.auto.action)
        self.assertTrue(self.worker.native_takeover)
        self.assertIn(b"STOP\n", self.port.writes)
        self.safety = "abort"
        self.worker.synchronize()
        self.assertIn(b"X\n", self.port.writes)
        self.assertEqual(self.session.state, "fault")
        self.assertFalse(any(data.startswith(b"JOG") for data in self.port.writes))
