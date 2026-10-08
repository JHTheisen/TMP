# BNO085 persistent handshake failure: packet transport correction

## Evidence and limits

Reviewed `logs/m09_20261008T001757.170068Z_34164_0.log`. The BNO ACKs
0x4A; its three startup handshakes fail before any orientation report arrives.
At startup the trace records a successful 128-byte stage-2 read and a later
stage-2 timeout. The third attempt explicitly records a failed 28-byte stage-2
read lasting 1,091,201 us. This fits the installed HAL's 152-byte transfer split
into 128 bytes and 28 bytes (24 remaining cargo bytes plus the repeated header).
The trace does not contain the packet headers, so that packet identification is
an inference, not a captured byte-level reconstruction.

This is a transport/handshake failure, not low orientation accuracy. No rotation
vector exists to qualify. An address scanner exercises ACK detection, not SHTP
payload delivery, reset completion, product identification, or report enabling.

The precise electrical reason the second transaction stalls is not established
by this log. Partial SHTP reads are valid per the sensor protocol; no claim is
made that the library's extra four continuation-header bytes are an arithmetic
bug. This change removes the implicated split-read path as a targeted transport
workaround. Hardware success still needs a subsequent run; no upload or hardware
validation was performed.

## Installed implementation traced

- PlatformIO: `espressif32@7.0.1`, ESP-IDF headers identify **4.4.7**.
- Adafruit BNO08x **1.2.7**, BusIO **1.17.4**, Unified Sensor **1.1.15**.
- Worker initializes Wire on SDA18/SCL19 and Wire1 on SDA4/SCL5, at 100 kHz,
  with configured Wire timeout 50 ms. BNO uses Wire1. Encoder I/O begins after
  the initial BNO attempt; both buses have a single worker owner. No motor pin
  overlap or conflicting TwoWire owner was found.
- `Adafruit_I2CDevice::begin()` calls Wire.begin(), but the installed ESP32
  implementation returns immediately for an already-started master bus. It does
  not replace these pins, clock or timeout.
- `DiagnosticBno085::_init()` delegates to Adafruit `_init()`. Reset GPIO is -1:
  there is no assumed/wired reset or INT pin. HAL open sends software reset
  `{5,0,1,0,1}` and waits 300 ms. SH-2 opens SHTP, processes advertisements/reset
  completion, then requests product IDs. The existing product-ID operation
  timeout is 1,000,000 us. On success the worker enables SH2_ROTATION_VECTOR
  at the existing 10,000-us interval. Reset re-enable and qualification remain.
- The installed SHTP open ignores the HAL-open return value, and SH-2 open can
  return success without observing reset completion in its 200-ms window.
  Thus a false `begin_I2C()` is not by itself proof of the exact failed handshake
  substep. Product-ID success remains required; no success bypass was added.
- The original BNO read HAL uses BusIO's ESP32 128-byte maximum and multiple
  STOP-separated reads, stitching out repeated SHTP headers. BusIO also casts
  request lengths to uint8_t, so merely enlarging Wire's buffer cannot make that
  path safely receive all 384 bytes of an SH-2 transfer.
- Installed `Wire::requestFrom()` calls `i2cRead()` with `_timeOutMillis`;
  `esp32-hal-i2c.c` passes that through to the IDF driver. Error 263 is
  ESP_ERR_TIMEOUT (0x107). The matching IDF 4.4.7 driver source clamps its event
  queue wait upward to I2C_CMD_ALIVE_INTERVAL_TICK, **1000 ms**. Consequently the
  configured 50 ms is not an end-to-end wall-clock bound. No framework-wide
  timeout patch was made. The independent sensor worker still isolates motor
  command processing from these waits.

Primary references: [CEVA BNO08x datasheet, I2C and SHTP sections](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf),
[ESP-IDF 4.4.7 I2C driver](https://github.com/espressif/esp-idf/blob/v4.4.7/components/driver/i2c.c).
Adafruit also documents [BNO08x/ESP32 I2C compatibility limitations](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/arduino).

## Changes in this investigation

1. `src/bno_packet_reader.h` (new): reserve the SH-2 maximum 384-byte Wire
   buffer on the selected BNO bus, peek four header bytes with STOP, then receive
   the entire announced packet in one STOP-terminated transaction using the
   size_t Wire overload. No 128-byte payload split, no uint8_t length truncation.
   Check buffer bounds, reserved/malformed lengths, short reads, and header
   consistency before returning anything to SH-2. Preserve receipt timestamps
   and I/O timing/failure hooks (stage 5 denotes packet validation failure).
2. `src/sensor_support.h`: install that reader into `_HAL.read` before the
   inherited SH-2 initialization. Allocation failure stops initialization.
   Retain the existing checked-write wrapper, reset and report paths.
3. `src/main.cpp`: append `transport=packet384` to BNO_INIT telemetry so a
   subsequent log can identify this firmware's receive path.
4. `tests/bno_packet_reader_test.cpp` (new): execute production reader code for
   5/20/128/152/256/384-byte transfers, continuation flags, both addresses,
   malformed/oversized packets, failed and short reads, changed headers,
   allocation failure, timestamps and buffer sentinels. Exercise subclass HAL
   installation as well as the reader directly.
5. `tests/stubs/Wire.h`: packet/read/buffer fixture; existing encoder fixture
   behavior retained.
6. `tests/stubs/Adafruit_BNO08x.h`: model the HAL read callback.
7. `tests/stubs/sh2_hal.h` (new): mirror pinned SH-2 transfer capacity.
8. `tests/run_host_tests.ps1`: register the packet reader regression.
9. `audit/BNO_PACKET_TRANSPORT_FIX.md` (this report).

No Python files or build patch scripts were changed in this investigation.
No TRACK, coordinate, direction, motor, carriage, encoder feedback, lease,
STOP/abort/takeover, B-button or stepped-photography behavior was changed.
`src/celestial_motion.h` retains SHA-256
`8BE705CF2181DBD39BBF8B58FA478C46E588FE2A9589FEC41371AB26A96107CE`.
The earlier yaw-envelope removal, TRACK fixes and all other working-tree
changes were retained. No evidence links a TRACK control branch to the initial
handshake failure, which occurs before a TRACK session exists.

## Validation

- `tests/run_host_tests.ps1 -SkipPython`: PASS. Includes BNO startup/recovery,
  sensor-worker stall isolation, AS5600/TRACK, celestial GOTO/TRACK/lease,
  NORTH/yaw direction, finite-motion fault handling, STOP/abort/takeover and
  keyframe regressions. All 23 protected M08 baseline files remain unchanged.
  The packet test was additionally rebuilt and passed after adding the subclass
  callback/allocation-failure checks.
- Python unittest discovery: **233 tests passed**, 213.507 seconds, including
  celestial and host B-button/stepped-photography coverage. No Python changes.
- Configured `platformio run -e esp32dev`: **SUCCESS**, 31.91 seconds. RAM
  48,316 / 327,680 bytes; flash 408,337 / 1,310,720 bytes.
- Logs: `.pio/bno_packet_cpp_tests.log`, `.pio/bno_packet_python_tests.log`,
  `.pio/bno_packet_esp32_build.log`.

The tests prove the software transaction shape and preserved control behavior,
not that a mocked sensor reproduces the electrical timeout. The driver can
still take about a second on a failed read. A subsequent hardware log must show
successful handshake, report enabling, and actual rotation reports before
hardware initialization can be called fixed. No firmware was uploaded.
