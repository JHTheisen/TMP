"""Bound the pinned BNO library's otherwise unlimited product-ID operation.

PlatformIO runs this before compiling dependencies. Keep the change reproducible
after cleaning .pio; reject unexpected upstream source instead of guessing.
"""
from pathlib import Path


def patch_source(source):
    original = "const sh2_Op_t getProdIdOp = {\n    .start = getProdIdStart,"
    patched = "const sh2_Op_t getProdIdOp = {\n    .timeout_us = 1000000,\n    .start = getProdIdStart,"
    if patched in source:
        return source
    if source.count(original) != 1:
        raise RuntimeError("Pinned SH2 product-ID operation changed; review its timeout")
    return source.replace(original, patched, 1)


if "Import" in globals():
    Import("env")
    dependency = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.subst("$PIOENV") / "Adafruit BNO08x" / "src" / "sh2.c"
    original = dependency.read_text(encoding="utf-8")
    patched = patch_source(original)
    if patched != original:
        dependency.write_text(patched, encoding="utf-8", newline="\n")
