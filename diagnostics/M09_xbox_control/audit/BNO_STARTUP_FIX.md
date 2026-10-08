# BNO085 startup investigation and fix

No firmware was uploaded and no hardware validation was performed.

## Observed failure versus confirmed software defects

The independent scanner result establishes that both AS5600 addresses and the
BNO address responded with the stated wiring. It does not establish that the
BNO can complete reset, SHTP advertisements, SH-2 product identification and
rotation-vector streaming under the gimbal firmware.

The latest local log, `logs/m09_20261007T235617.819807Z_14180_0.log`, shows:

- Line 34: successful BNO payload read, 128 bytes, at MCU 389 ms.
- Line 41: `i2cRead returned Error 263` at MCU 1399 ms. The installed ESP-IDF
  header defines 263 (`0x107`) as `ESP_ERR_TIMEOUT`.
- Lines 35-37 and subsequent reports: `available=NO`, `has_sample=NO`, no
  rotation-vector events. The printed accuracy zero is a default without a
  sample, not evidence of merely degraded orientation calibration.
- Lines 43-44: both AS5600s subsequently have valid, fresh reads.
- Line 86: BNO I/O reports five calls, one failure, maximum duration 1,007,021 us,
  and no acquisition calls. Initialization did not reach normal acquisition.

Three earlier evening logs show the same startup timeout pattern. These logs
do not prove the electrical/protocol cause of the first read timeout. The
following firmware defects are confirmed in the source:

1. `SensorWorker::initialize()` probed only 0x4A and attempted `begin_I2C()`
   once. After an ACK or handshake failure, `setupDone` prevented initialization
   from ever being retried. All normal BNO service was gated by
   `bnoInitialized`, so a transient startup failure became permanent for the boot.
2. The valid alternate BNO address, 0x4B, was never tried.
3. Diagnostics collapsed bus/handshake/report-enable failures into the same
   unavailable warning. No initialization result or selected address was logged.
4. Failed `begin_I2C()` can leave an allocated SH-2/SHTP session. Retrying without
   cleanup can exhaust the pinned library's session pool. Conversely, blindly
   calling the original `sh2_close()` after a pre-open failure dereferences a
   null SHTP pointer. Safe cleanup is required for the added recovery path.

## Bus, library and recent-change audit

| Item | Verified current implementation |
| --- | --- |
| Bus A | Global `Wire`, controller 0, SDA18/SCL19, AS5600 0x36 |
| Bus B | Global `Wire1`, controller 1, SDA4/SCL5, AS5600 0x36 and BNO 0x4A/0x4B |
| Clock/timeout | Both explicitly begin at 100,000 Hz; Wire timeout configured to 50 ms |
| Ownership | One sensor worker owns both buses, BNO lifecycle and encoder transactions |
| Ordering | Motor/watchdog setup, worker creation, bus A begin, bus B begin, encoder state initialization, BNO probe/handshake, then normal encoder polling |
| Encoder begin | Initializes software state only; no transactions during BNO initialization |
| BNO library | Pinned Adafruit BNO08x 1.2.7, using `begin_I2C(address, &Wire1)` |
| Reset | No hardware reset pin configured (`-1`); existing library software reset and 300 ms delay retained |
| Reports | Existing `SH2_ROTATION_VECTOR` at 10,000 us; reset re-enables that report |
| Product ID | Existing 1,000,000 us operation deadline retained |

Adafruit BusIO calls `Wire1.begin()` again with defaults. In the installed
Arduino-ESP32 implementation, an already initialized master returns before
changing pins/clock; it does not move this bus onto default GPIO21/22. No other
production code creates competing TwoWire objects or reconfigures these buses.
The two encoders share address 0x36 on separate controllers, so they do not
conflict. BNO and the pitch encoder have different addresses on Bus B.

The recent TRACK changes do not touch initialization, pins, Wire objects, clock
or transport calls. `learnStoppedCelestialEncoderScales()` consumes published
encoder snapshots only. The startup worker, sensor support, dependency versions
and library patch scripts matched HEAD before this fix. The one-shot startup
behavior predates the recent TRACK architecture changes. No initialization
regression caused by those TRACK changes was found.

The configured Wire timeout is not a total bound on a library call: the supplied
log contains a roughly one-second blocked read despite that setting. This fix
does not claim to repair that lower-level timeout or change clock/timeout values
without evidence. Slow sensor operations remain isolated from foreground motor
commands and watchdogs.

## Changes

- Probe 0x4A and 0x4B on Bus B; prefer 0x4A when both ACK. Pass the selected
  address to initial setup and the existing accuracy reinitialization path.
- Retry failed initial detection/handshake at most three times, with a one-second
  gap measured from completion of the preceding attempt. Encoder polling and
  ordinary control continue between attempts. Exhaustion is explicitly logged.
- Successful initialization ends connection retries, including when report
  enabling needs its existing separate recovery. Later BNO degradation during
  encoder-primary TRACK cannot start the new retry sequence. Latched abort stops
  pending startup attempts. Existing accuracy/reset recovery remains intact.
- Close failed SH-2 sessions, with a reproducible null guard in the existing
  PlatformIO dependency patch. Neither I2C bus is restarted or reassigned.
- Add `BNO_INIT` status plus `BUS_INIT`, `ADDRESS_PROBE`, `INIT_BEGIN`,
  `INIT_RESULT`, `STARTUP_RETRY_WAIT` and `STARTUP_EXHAUSTED` trace events.

`BNO_INIT` separates these cases:

| Evidence | Meaning |
| --- | --- |
| `stage=BUS_UNAVAILABLE` | ESP32 bus initialization failed; no BNO probe |
| `stage=NO_ACK` | Neither BNO address acknowledged on this attempt |
| `stage=INITIALIZING` | ACK received; library handshake in progress |
| `stage=HANDSHAKE_FAILED` | Address selected, but `begin_I2C()` returned false |
| `stage=REPORT_ENABLE_FAILED initialized=YES` | Library initialized; report request failed |
| `stage=REPORTS_ENABLED` with no fresh `BNO_STATE` sample | Reports requested, but no usable fresh orientation yet |
| Fresh sample, accuracy 0/1 | Communication/reporting works; magnetic orientation remains unqualified |

Probe return codes are raw Wire results (`0` ACK, `255` not yet attempted).
`address=0x00` means no address was selected. Accuracy qualification and safety
continue to use the existing `BNO_STATE`/`BNO_RAW` rules. The new fields do not
grant sensor readiness or renew a motor/target lease.

## Exact files changed for this fix

- `src/sensor_worker.h`: startup attempts, address selection, failed-session
  cleanup and initialization-stage snapshots/traces.
- `src/main.cpp`: `BNO_INIT` telemetry and startup address description only.
- `tools/patch_sh2_timeout.py`: add `patch_close()` and apply it with the existing
  product-ID timeout patch. This is a firmware dependency build script; the
  Python controller, dashboard, B button and photo coordinator were not edited.
- `tests/bno_startup_test.cpp` (new): 15 startup/recovery/isolation scenarios.
- `tests/stubs/Wire.h`, `tests/stubs/Adafruit_BNO08x.h`: address, timing, partial
  session and failure injection for those tests.
- `tests/m09_lifecycle_test.cpp`: require cleanup of a failed accuracy-recovery
  replacement, while preserving the existing bounded recovery expectations.
- `tests/test_sh2_timeout.py`, `tests/test_bno_library_patch.py`: repeatable close
  patch plus compiled actual-library null/partial-session close tests.
- `tests/run_host_tests.ps1`: register startup scenarios.
- `README.md`, `audit/BNO_STARTUP_FIX.md`: startup behavior and investigation.

TRACK feedback/rates, directions, pulses-per-degree constants, carriage,
unrestricted celestial/NORTH yaw, finite-motion protections, 3000 ms celestial
lease, STOP/takeover/rearm, B-button exit hold and stepped photography are retained.

## Validation results

- `tests/run_host_tests.ps1 -SkipPython`: PASS, including all 15 new startup
  scenarios and the existing celestial, primary-AS5600, low-rate, finite-motion,
  NORTH/yaw, STOP/abort/takeover, keyframe and watchdog regressions. All 23 M08
  baseline files remain byte-for-byte unchanged.
- PlatformIO Python with `.pio/python_deps` on `PYTHONPATH`,
  `-m unittest discover -s tests -p 'test_*.py'`: **233 tests passed** in
  209.692 seconds, including the actual compiled library cleanup routine and
  existing B-button/stepped-photo UI regressions.
- `python -m platformio run -e esp32dev`: **SUCCESS**, 11.36 seconds. RAM
  48,308 / 327,680 bytes (14.7%); flash 407,417 / 1,310,720 bytes (31.1%). The
  patched SH-2 library was rebuilt. No upload target was invoked.
- `git diff --check`: PASS.
- `src/celestial_motion.h` remains at SHA-256
  `8BE705CF2181DBD39BBF8B58FA478C46E588FE2A9589FEC41371AB26A96107CE`, identical
  to the completed TRACK architecture change before this investigation.

Evidence files: `.pio/bno_startup_cpp_tests.log`,
`.pio/bno_startup_python_tests.log`, `.pio/bno_startup_esp32_build.log`.

The fix is verified in software, not on the connected gimbal. The initial
read-timeout cause remains unverified. If all bounded attempts fail on hardware,
the new address/stage/I/O diagnostics distinguish where they fail; no claim is
made that retries repair an underlying persistent electrical or protocol fault.
