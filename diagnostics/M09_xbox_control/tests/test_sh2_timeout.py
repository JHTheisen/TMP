import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("patch_sh2_timeout", Path(__file__).parents[1] / "tools" / "patch_sh2_timeout.py")
patcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patcher)


class ProductIdTimeoutTests(unittest.TestCase):
    def test_only_product_id_operation_changes_and_repeat_is_safe(self):
        source = "prefix\nconst sh2_Op_t getProdIdOp = {\n    .start = getProdIdStart,\n    .rx = getProdIdRx,\n};\nsuffix\n"
        result = patcher.patch_source(source)
        self.assertEqual(result.replace("    .timeout_us = 1000000,\n", ""), source)
        self.assertEqual(patcher.patch_source(result), result)

    def test_unexpected_dependency_is_not_silently_patched(self):
        with self.assertRaises(RuntimeError):
            patcher.patch_source("different upstream operation")
