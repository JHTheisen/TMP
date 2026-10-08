import unittest

from encoder_setup import EncoderSetup
from xbox_control import arguments

REPORT = "ORIENTATION_STATE protocol=2 feedback=AS5600"


class EncoderSetupTests(unittest.TestCase):
    def start(self, setup):
        self.assertEqual(setup.frame(0), b"STOP\n")
        setup.receive(REPORT)
        self.assertIsNone(setup.frame(.1))
        setup.receive("M09 READY")
        return setup.frame(.2)

    def test_ratios_convert_to_degrees_and_require_ack_then_ready(self):
        setup = EncoderSetup(1, .06666667)
        self.assertEqual(self.start(setup), b"ENCODER_CONFIG 360 24.0000012\n")
        setup.receive("M09 READY")
        self.assertTrue(setup.pending)
        self.assertIsNone(setup.frame(.3))
        setup.receive("CALIBRATION CONFIGURED: session only")
        self.assertTrue(setup.pending)
        setup.receive("M09 READY")
        self.assertFalse(setup.pending)
        self.assertIsNone(setup.frame(100))

    def test_reboot_reapplies_but_status_polls_do_not(self):
        setup = EncoderSetup(-1, -1/15)
        self.assertEqual(self.start(setup), b"ENCODER_CONFIG -360 -24\n")
        setup.receive("CALIBRATION CONFIGURED: session only")
        setup.receive("M09 READY")
        setup.receive(REPORT)
        self.assertIsNone(setup.frame(2))
        setup.receive("M09_xbox_control: restarted")
        self.assertEqual(self.start(setup), b"ENCODER_CONFIG -360 -24\n")

    def test_old_firmware_and_busy_cannot_enable_configuration(self):
        setup = EncoderSetup(1, 1)
        setup.frame(0)
        setup.receive("M09 READY")
        setup.receive(REPORT.replace("protocol=2", "protocol=1"))
        self.assertIsNone(setup.frame(.1))
        setup.receive("M09 BUSY")
        setup.receive(REPORT)
        self.assertIsNone(setup.frame(.2))
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            setup.frame(11)

    def test_rejection_and_abort(self):
        setup = EncoderSetup(1, 1)
        self.start(setup)
        setup.receive("CALIBRATION REJECTED: stopped READY required")
        with self.assertRaisesRegex(RuntimeError, "REJECTED"):
            setup.frame(.3)
        setup.receive("M09 ABORTED")
        self.assertFalse(setup.pending)
        self.assertIsNone(setup.frame(20))

    def test_omitted_configuration_preserves_existing_workflow(self):
        setup = EncoderSetup()
        setup.receive("M09_xbox_control: restarted")
        self.assertFalse(setup.pending)
        self.assertIsNone(setup.frame(1))

    def test_cli_requires_valid_pair(self):
        for values in (("1", None), ("0", "1"), ("nan", "1"), ("1", "inf"), ("2", "1")):
            with self.subTest(values=values), self.assertRaises(SystemExit):
                arguments(["--yaw-encoder-ratio", values[0]] +
                          ([] if values[1] is None else ["--pitch-encoder-ratio", values[1]]))
        args = arguments(["--yaw-encoder-ratio", "1", "--pitch-encoder-ratio", "0.06666667"])
        self.assertEqual(args.pitch_encoder_ratio, .06666667)
