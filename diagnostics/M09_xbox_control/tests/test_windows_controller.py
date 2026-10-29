import unittest
from unittest import mock

import windows_controller


class WindowsControllerTests(unittest.TestCase):
    def make_reader(self):
        self.values = {}
        self.connected = True

        def state(index, pointer):
            self.assertEqual(index, 0)
            for field, value in self.values.items():
                setattr(pointer._obj.pad, field, value)
            return 0 if self.connected else 1167

        dll = mock.Mock()
        dll.XInputGetState.side_effect = state
        with mock.patch.object(windows_controller.sys, "platform", "win32"), \
             mock.patch.object(windows_controller.ctypes, "WinDLL", return_value=dll, create=True):
            return windows_controller.safety_reader(0, .15)

    def test_neutral_noise_and_triggers_do_not_take_over(self):
        read = self.make_reader()
        self.values = dict(lx=1000, ly=-1000, rx=4000, ry=-4000, lt=0, rt=255)
        self.assertIsNone(read())

    def test_sticks_cancel_and_b_has_priority(self):
        read = self.make_reader()
        for field in ("lx", "ly", "rx", "ry"):
            self.values = {field: -10000}
            self.assertEqual(read(), "takeover")
        self.values["buttons"] = 0x2000
        self.assertEqual(read(), "abort")

    def test_disconnection_is_a_failure_after_native_controller_seen(self):
        read = self.make_reader()
        self.assertIsNone(read())
        self.connected = False
        with self.assertRaisesRegex(RuntimeError, "disconnected"):
            read()
