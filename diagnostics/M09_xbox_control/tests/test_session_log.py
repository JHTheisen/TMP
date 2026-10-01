"""Disk logging only: never accesses serial or a controller."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from session_log import SessionLog


class LoggingTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def logger(self):
        log = SessionLog(self.directory.name, {"test": True})
        self.addCleanup(log.close, 0)
        return log

    def test_unique_files_and_immediately_readable_flush(self):
        first, second = self.logger(), self.logger()
        self.assertNotEqual(first.path, second.path)
        first.event("STOP", "operator")
        data = first.path.read_text(encoding="utf-8")
        self.assertIn("STOP operator", data)
        self.assertRegex(data, r"\d{4}-\d\d-\d\dT.*\+00:00 \+\d+\.\d+s")
        self.assertNotIn("STOP operator", second.path.read_text(encoding="utf-8"))

    def test_quality_transition_is_immediate_and_traces_sampled_by_kind(self):
        log = self.logger()
        log.received("BNO_RAW raw_status=3 diagnostic_quality=PLAUSIBLE", 1.0)
        log.received("BNO_RAW raw_status=3 diagnostic_quality=MALFORMED", 1.01)
        log.received("BNO_TRACE kind=RESET_COMPLETE reset_event=1 at_us=123", 1.02)
        log.received("BNO_TRACE kind=RESET_COMPLETE reset_event=2 at_us=456", 1.03)
        log.received("BNO_TRACE kind=REPORT_ENABLED at_us=789", 1.04)
        log.received("BNO_TRACE kind=RESET_COMPLETE reset_event=3 at_us=999", 6.03)
        data = log.path.read_text(encoding="utf-8")
        self.assertIn("diagnostic_quality=MALFORMED", data)
        self.assertIn("reset_event=1", data)
        self.assertNotIn("reset_event=2", data)
        self.assertIn("kind=REPORT_ENABLED", data)
        self.assertIn("reset_event=3", data)

    def test_command_stream_is_sampled_but_edges_are_immediate(self):
        log = self.logger()
        for n in range(100):
            log.transmitted(f"JOG {100+n} 0 0\n".encode(), n * 0.02)
        log.transmitted(b"JOG 0 0 0\n", 1.981)
        log.transmitted(b"JOG -100 0 0\n", 1.982)
        log.transmitted(b"STOP\n", 1.983)
        log.transmitted(b"X\n", 1.984)
        data = log.path.read_text(encoding="utf-8")
        self.assertLessEqual(data.count("TX_ATTEMPT JOG"), 7)
        for line in ("JOG 0 0 0", "JOG -100 0 0", "STOP", "X"):
            self.assertIn("TX_ATTEMPT " + line, data)

    def test_raw_values_and_sensor_transitions_without_parsed_duplicate(self):
        log = self.logger()
        raw = "BNO_RAW report_id=0x05 raw_status=0 q_w=0.999938965 q_x=0 q_y=0 q_z=0 accepted=YES age_ms=2"
        log.received(raw, 1.0)
        log.received(raw.replace("raw_status=0", "raw_status=3"), 1.01)
        log.received("ENCODER_STATE bus=A available=YES valid=YES raw=2048 status=0x20", 1.02)
        log.received("ENCODER_STATE bus=A available=NO valid=NO raw=2048 status=0x20", 1.03)
        log.received("MANUAL REJECTED: stopping; wait for READY", 1.04)
        data = log.path.read_text(encoding="utf-8")
        self.assertIn(raw, data)
        for value in ("raw_status=0", "raw_status=3", "q_w=0.999938965", "available=NO"):
            self.assertIn(value, data)
        self.assertNotIn(" | parsed=", data)
        self.assertIn("MANUAL REJECTED: stopping; wait for READY", data)

    def test_exception_traceback_survives_close(self):
        log = self.logger()
        try:
            raise RuntimeError("simulated serial failure")
        except RuntimeError:
            log.exception("read failed")
        log.close(1)
        data = log.path.read_text(encoding="utf-8")
        self.assertIn("Traceback (most recent call last)", data)
        self.assertIn("RuntimeError: simulated serial failure", data)
        self.assertIn("exit result=1", data)

    def test_file_failure_does_not_escape_into_motor_control(self):
        log = self.logger()
        with mock.patch.object(log._file, "write", side_effect=OSError("disk full")), mock.patch("sys.stderr"):
            log.event("TEST", "still running")
        self.assertIn("disk full", log.error)
        log.event("STOP", "does not raise")

    def test_open_failure_reports_and_remains_usable(self):
        bad = Path(self.directory.name) / "file"
        bad.write_text("not a directory")
        with mock.patch("sys.stderr"):
            log = SessionLog(bad, {})
        self.assertTrue(log.error)
        self.assertIsNone(log.path)
        log.event("STOP", "does not raise")
        log.close(1)
