"""Bound product-ID initialization and safely close failed SH-2 sessions.

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


def patch_close(source):
    # begin_I2C can fail before opening SH-2. Upstream shtp_close dereferences
    # its argument; allow cleanup of either an unopened or partial session.
    original = "    shtp_close(pSh2->pShtp);"
    patched = "    if (pSh2->pShtp) shtp_close(pSh2->pShtp);"
    if patched in source:
        return source
    if source.count(original) != 1:
        raise RuntimeError("Pinned SH2 close changed; review failed-session cleanup")
    return source.replace(original, patched, 1)


if "Import" in globals():
    Import("env")
    dependency = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.subst("$PIOENV") / "Adafruit BNO08x" / "src" / "sh2.c"
    original = dependency.read_text(encoding="utf-8")
    patched = patch_close(patch_source(original))
    if patched != original:
        dependency.write_text(patched, encoding="utf-8", newline="\n")
