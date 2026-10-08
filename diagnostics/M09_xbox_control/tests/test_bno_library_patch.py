"""Compile the actual locally patched timestamp/HAL routines; no hardware I/O."""
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).parents[1]
LIB = ROOT / ".pio/libdeps/esp32dev/Adafruit BNO08x/src"
spec = importlib.util.spec_from_file_location("patch_bno_diagnostics", ROOT / "tools/patch_bno_diagnostics.py")
patcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patcher)
COMPILER = Path("C:/Strawberry/c/bin/g++.exe")


def function(source, signature):
    start = source.index(signature)
    brace = source.index("{", start)
    level = 1
    end = brace + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


class LibraryPatchTests(unittest.TestCase):
    def compile_run(self, source):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "check.cpp").write_text(source, encoding="utf-8")
            subprocess.run([str(COMPILER), "-std=gnu++11", str(path / "check.cpp"),
                            "-o", str(path / "check.exe")], check=True, capture_output=True, text=True)
            subprocess.run([str(path / "check.exe")], check=True, capture_output=True, text=True)

    def test_patch_is_repeatable_and_rejects_unexpected_upstream(self):
        for filename, patch in (("Adafruit_BNO08x.cpp", patcher.patch_hal), ("sh2.c", patcher.patch_sh2)):
            result = patch((LIB / filename).read_text(encoding="utf-8"))
            self.assertEqual(result, patch(result))
            with self.assertRaises(RuntimeError):
                patch("unsupported library")

    def test_actual_sh2_close_releases_partial_session_and_accepts_null(self):
        from test_sh2_timeout import patcher as lifecycle_patcher
        source = lifecycle_patcher.patch_close((LIB / "sh2.c").read_text(encoding="utf-8"))
        routine = function(source, "void sh2_close(void)")
        self.compile_run(r'''
#include <cassert>
#include <cstring>
struct sh2_t { void *pShtp; unsigned other; } _sh2;
unsigned closes = 0;
void shtp_close(void *session) { assert(session); ++closes; }
''' + routine + r'''
int main() {
    sh2_close(); assert(closes == 0 && !_sh2.pShtp);
    _sh2.pShtp = &closes; _sh2.other = 99;
    sh2_close(); assert(closes == 1 && !_sh2.pShtp && !_sh2.other);
    sh2_close(); assert(closes == 1);
}
''')

    def test_actual_timestamp_offsets_and_unsigned_clock_rollover(self):
        source = patcher.patch_sh2((LIB / "sh2.c").read_text(encoding="utf-8"))
        routine = function(source, "static uint64_t touSTimestamp(")
        self.compile_run("#include <cstdint>\n#include <cassert>\n" + routine + r'''
int main() {
    assert(touSTimestamp(100, -23, 0) == 0); // no fictitious 71-minute age at boot
    assert(touSTimestamp(100000, -23, 0) == 97700);
    assert(touSTimestamp(110000, -23, 2) == 107900);
    assert(touSTimestamp(120000, 4, 1) == 120500);
    const uint64_t wrap = uint64_t(1) << 32;
    const auto before = touSTimestamp(0xfffffff0U, -23, 0);
    const auto after = touSTimestamp(0x10U, -23, 0);
    assert(before == wrap - 16 - 2300);
    assert(after == wrap + 16 - 2300 && after - before == 32);
    // Report offsets can straddle the wrap without a spurious extra epoch.
    assert(touSTimestamp(0x20U, 23, 0) == wrap + 32 + 2300);
    assert(touSTimestamp(0x10000U, -23, 0) == wrap + 65536 - 2300);
}
''')

    def test_actual_hal_receipt_timestamp_and_failed_read(self):
        source = patcher.patch_hal((LIB / "Adafruit_BNO08x.cpp").read_text(encoding="utf-8"))
        # Select definition, not forward declaration.
        start = source.index("static int i2chal_read(", source.index("static int i2chal_open(sh2_Hal_t *self) {"))
        routine = function(source[start:], "static int i2chal_read(")
        self.compile_run(r'''
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <cassert>
using std::min;
struct sh2_Hal_t {};
struct Device { size_t maxBufferSize() { return 32; } } device;
Device *i2c_dev = &device;
uint32_t clockUs = 0;
bool failHeader = false, failPayload = false;
uint32_t micros() { return clockUs; }
bool m09_i2c_read(uint8_t *data, size_t size, unsigned stage) {
    clockUs += 100;
    if ((stage == 1 && failHeader) || (stage == 2 && failPayload)) return false;
    memset(data, 0, size); data[0] = 20;
    return true;
}
''' + routine + r'''
int main() {
    sh2_Hal_t hal; uint8_t packet[64]; uint32_t timestamp = 77;
    clockUs = 100000;
    assert(i2chal_read(&hal, packet, sizeof(packet), &timestamp) == 20);
    assert(timestamp == 100100 && clockUs == 100200); // header, not end of payload
    clockUs = 0xffffffceU; // header receipt wraps to 50
    assert(i2chal_read(&hal, packet, sizeof(packet), &timestamp) == 20 && timestamp == 50);
    failHeader = true; timestamp = 77;
    assert(i2chal_read(&hal, packet, sizeof(packet), &timestamp) == 0 && timestamp == 77);
    failHeader = false; failPayload = true;
    assert(i2chal_read(&hal, packet, sizeof(packet), &timestamp) == 0);
}
''')

    def test_actual_reset_callback_preserves_each_notification(self):
        source = patcher.patch_sh2((LIB / "sh2.c").read_text(encoding="utf-8"))
        routine = function(source, "static void executableDeviceHdlr(")
        self.compile_run(r'''
#include <cstdint>
#include <cassert>
#define EXECUTABLE_DEVICE_RESP_RESET_COMPLETE 1
#define SH2_RESET 9
struct Event { int eventId; } sh2AsyncEvent;
struct sh2_t { bool resetComplete=false; unsigned execBadPayload=0;
    void (*eventCallback)(void*, Event*)=nullptr; void *eventCookie=nullptr; };
unsigned callbacks=0, events=0; uint32_t lastTimestamp=0;
void m09_bno_reset(uint32_t timestamp) { ++events; lastTimestamp=timestamp; }
void callback(void*, Event*) { ++callbacks; }
''' + routine + r'''
int main() {
    sh2_t sh2; sh2.eventCallback=callback; uint8_t packet=1;
    executableDeviceHdlr(&sh2, &packet, 1, 123);
    executableDeviceHdlr(&sh2, &packet, 1, 456);
    assert(events==2 && callbacks==2 && lastTimestamp==456 && sh2.resetComplete);
    executableDeviceHdlr(&sh2, &packet, 0, 789);
    packet=2; executableDeviceHdlr(&sh2, &packet, 1, 789);
    assert(events==2 && sh2.execBadPayload==2);
}
''')

    def test_actual_product_request_only_sends_and_preserves_active_operation(self):
        source = patcher.patch_sh2((LIB / "sh2.c").read_text(encoding="utf-8"))
        routine = function(source, "int m09_sh2_request_product_id(")
        self.compile_run(r'''
#include <cstdint>
#include <cstring>
#include <cassert>
#define SH2_ERR_OP_IN_PROGRESS -3
#define SENSORHUB_PROD_ID_REQ 0xf9
struct ProdIdReq_t { uint8_t reportId, reserved; };
struct sh2_t { void *pOp=nullptr; } _sh2;
unsigned calls=0; int sendResult=0;
int sendCtrl(sh2_t*, uint8_t *data, uint16_t len) {
    ++calls; assert(len==2 && data[0]==0xf9 && data[1]==0); return sendResult;
}
''' + routine + r'''
int main() {
    assert(m09_sh2_request_product_id()==0 && calls==1 && !_sh2.pOp);
    _sh2.pOp=&calls;
    assert(m09_sh2_request_product_id()==-3 && calls==1 && _sh2.pOp==&calls);
    _sh2.pOp=nullptr; sendResult=-4;
    assert(m09_sh2_request_product_id()==-4 && calls==2);
}
''')

    def test_actual_product_observer_reports_raw_cause_and_ignores_truncation(self):
        source = patcher.patch_sh2((LIB / "sh2.c").read_text(encoding="utf-8"))
        observer = function(source, "if (reportId == SENSORHUB_PROD_ID_RESP &&")
        self.compile_run(r'''
#include <cstdint>
#include <cassert>
#define SENSORHUB_PROD_ID_RESP 0xf8
struct ProdIdResp_t { uint8_t reportId, resetCause, major, minor; uint32_t swPartNumber, swBuildNumber;
    uint16_t patch; uint8_t reserved0, reserved1; };
unsigned calls=0; uint8_t cause=0; uint32_t part=0, build=0, at=0;
void m09_bno_product(uint32_t timestamp, uint8_t c, uint32_t p, uint32_t b) {
    ++calls; at=timestamp; cause=c; part=p; build=b;
}
void observe(uint8_t *payload, uint16_t len, uint8_t reportLen, uint32_t timestamp) {
    unsigned cursor=0; uint8_t reportId=payload[0];
''' + observer + r'''
}
int main() {
    ProdIdResp_t id={0xf8,3,1,2,1000,2000,4,0,0};
    observe((uint8_t*)&id, sizeof(id), sizeof(id), 123);
    assert(calls==1 && cause==3 && part==1000 && build==2000 && at==123);
    id.resetCause=99; observe((uint8_t*)&id, sizeof(id), sizeof(id), 456);
    assert(calls==2 && cause==99); // unknown values remain raw
    observe((uint8_t*)&id, sizeof(id)-1, sizeof(id), 789);
    observe((uint8_t*)&id, sizeof(id), sizeof(id)-1, 789);
    id.reportId=0x05; observe((uint8_t*)&id, sizeof(id), sizeof(id), 789);
    assert(calls==2);
}
''')
