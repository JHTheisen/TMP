"""Small, repeatable instrumentation of the pinned BNO08x 1.2.7 sources.

Hooks are weak and observation-only. Never print from the transport/callback.
No sensor configuration, reset command, bus timing or read ordering is changed.
"""
from pathlib import Path


def replace_once(source, before, after):
    if after in source:
        return source
    if source.count(before) != 1:
        raise RuntimeError("Pinned BNO diagnostic patch anchor changed: " + before[:70])
    return source.replace(before, after, 1)


def patch_hal(source):
    anchor = "static HardwareSerial *uart_dev = NULL;"
    source = replace_once(source, anchor, anchor + '''

// M09_DIAGNOSTICS: boolean BusIO result; numeric ESP-IDF errors remain Wire logs.
extern "C" void m09_bno_io(uint32_t, uint32_t, unsigned, unsigned, int) __attribute__((weak));
static bool m09_i2c_read(uint8_t *data, size_t size, unsigned stage) {
  const uint32_t start = micros();
  const bool ok = i2c_dev->read(data, size);
  if (m09_bno_io) m09_bno_io(start, uint32_t(micros()-start), stage, size, ok);
  return ok;
}
static bool m09_i2c_write(const uint8_t *data, size_t size, unsigned stage) {
  const uint32_t start = micros();
  const bool ok = i2c_dev->write(data, size);
  if (m09_bno_io) m09_bno_io(start, uint32_t(micros()-start), stage, size, ok);
  return ok;
}
''')
    for before, after in (
        ("i2c_dev->write(softreset_pkt, 5)", "m09_i2c_write(softreset_pkt, 5, 4)"),
        ("i2c_dev->read(header, 4)", "m09_i2c_read(header, 4, 1)"),
        ("i2c_dev->read(i2c_buffer, read_size)", "m09_i2c_read(i2c_buffer, read_size, 2)"),
        ("i2c_dev->write(pBuffer, write_size)", "m09_i2c_write(pBuffer, write_size, 3)"),
        ("  if (!m09_i2c_read(header, 4, 1)) {\n    return 0;\n  }\n",
         "  if (!m09_i2c_read(header, 4, 1)) {\n    return 0;\n  }\n"
         "  // Polling HAL: receipt of header approximates interrupt time (no INT pin).\n  *t_us = micros();\n"),
    ):
        source = replace_once(source, before, after)
    return source


def patch_sh2(source):
    # Send the existing request without a blocking SH-2 operation. Replies flow
    # through normal acquisition and the passive product-ID hook below.
    source = replace_once(source, "static int16_t toQ14(double x)", '''int m09_sh2_request_product_id(void) {
    if (_sh2.pOp) return SH2_ERR_OP_IN_PROGRESS;
    ProdIdReq_t req;
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_PROD_ID_REQ;
    return sendCtrl(&_sh2, (uint8_t *)&req, sizeof(req));
}

static int16_t toQ14(double x)''')
    anchor = "sh2_t _sh2;"
    source = replace_once(source, anchor, anchor + '''

// M09_DIAGNOSTICS: record each notification before the Boolean reset latch.
extern void m09_bno_reset(uint32_t) __attribute__((weak));
extern void m09_bno_product(uint32_t, uint8_t, uint32_t, uint32_t) __attribute__((weak));
extern void m09_bno_init_response(uint32_t) __attribute__((weak));
''')
    source = replace_once(source, "            pSh2->resetComplete = true;",
        "            pSh2->resetComplete = true;\n            if (m09_bno_reset) m09_bno_reset(timestamp);")
    source = replace_once(source, "                    // This is an unsolicited INIT message.",
        "                    // This is an unsolicited INIT message.\n                    if (m09_bno_init_response) m09_bno_init_response(timestamp);")
    anchor = "            // Hand off to operation in progress, if any"
    source = replace_once(source, anchor, '''            // Observe complete product-ID responses, including normal startup.
            if (reportId == SENSORHUB_PROD_ID_RESP &&
                reportLen >= sizeof(ProdIdResp_t) && cursor + sizeof(ProdIdResp_t) <= len) {
                const ProdIdResp_t *id = (const ProdIdResp_t *)(payload + cursor);
                if (m09_bno_product) m09_bno_product(timestamp, id->resetCause,
                                                    id->swPartNumber, id->swBuildNumber);
            }

''' + anchor)
    # Correct the offset boundary as well as supplying the HAL receive clock:
    # applying a negative offset in uint32_t adds a spurious 2^32 near wrap.
    source = replace_once(source,
        "    timestamp += hostInt + (referenceDelta + delay) * 100;",
        "    timestamp += hostInt;\n    // M09: apply signed report offset AFTER extending the host clock.\n"
        "    int64_t adjusted = (int64_t)timestamp + ((int64_t)referenceDelta + delay) * 100;\n"
        "    timestamp = adjusted < 0 ? 0 : (uint64_t)adjusted;")
    return source


if "Import" in globals():
    Import("env")
    root = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.subst("$PIOENV") / "Adafruit BNO08x" / "src"
    for name, patch in (("Adafruit_BNO08x.cpp", patch_hal), ("sh2.c", patch_sh2)):
        path = root / name
        original = path.read_text(encoding="utf-8")
        result = patch(original)
        if result != original:
            path.write_text(result, encoding="utf-8", newline="\n")
