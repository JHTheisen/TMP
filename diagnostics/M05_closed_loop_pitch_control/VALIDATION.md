# Milestone 5 software validation

Validated 2026-09-08. **Physical verification pending.** No hardware upload,
commit, or push was performed.

## ESP32 build: PASS

Command, from the parent `diagnostics` directory:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -d M05_closed_loop_pitch_control -e esp32dev
```

Final build includes the 0.10-degree approach margin and explicitly pinned
M04 transitive dependency versions:

- Board: `esp32dev`, Espressif32 7.0.1, Arduino-ESP32 2.0.17.
- FastAccelStepper 1.2.7; AS5600 0.6.7; Adafruit BNO08x 1.2.7;
  Adafruit BusIO 1.17.4; Adafruit Unified Sensor 1.1.15.
- RAM: 30,436 / 327,680 bytes (9.3%).
- Flash: 336,285 / 1,310,720 bytes (25.7%).
- Image: `.pio/build/esp32dev/firmware.bin`.

## Native software tests: PASS

Command, from this M05 project:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
```

Strict `-std=c++11 -Wall -Wextra -Werror -pedantic` compilation passed.
The final controller passed **48 math/quaternion checks and 41 integration
scenarios**. Integration tests execute the real
firmware `setup()` and `loop()` with deterministic sensor and actuator fixtures.

Successful fixtures cover:

- Actual pulse scaling of 1600 and 3200 pulses/revolution, and reduction values
  of 7, 10, 15, and 30, independently of the controller's planning constants.
- Opposite AS5600 sensor polarity and raw-angle wrap crossings.
- Modest noise and +/-0.20-degree noise accepted by the baseline gates.
- 65% effective pulse delivery with reversal backlash and noise.
- Slow orientation drift and an external +0.8-degree cradle disturbance.
  The disturbance case verifies that the controller returns to measured sensor
  zero with a nonzero net commanded pulse count.
- Brief recoverable sensor interruptions, advisory magnet warnings, and BNO
  reset/report restoration before the startup reference.

Failure fixtures verify:

- Wrong-way cradle movement, frozen cradle, frozen motor, and frozen AS5600.
- Required-sensor outages, encoder read errors and ambiguous half-turn samples,
  invalid quaternions, and an unexpected BNO report type.
- Blocking AS/BNO calls that subsequently return data: the original acquisition
  gap still stops the run before any additional correction is commanded.
- BNO reset after reference, measured travel guards, serial abort during startup
  and motion, rejected moves, a motor burst that never finishes, and a leg that
  cannot settle because measurements become noisy.
- Motor/bus/sensor/report initialization failures, an unstable baseline, and an
  unsafe initial tilt; these prevent the first move.

Every scenario checks pitch-only commands, bounded bursts/speed/acceleration,
a single final result, and subsequent idle behavior with no restart. Successful
scenarios check both measured endpoints, return motion, and preserved Bus B and
pitch pin settings.

## Review and preservation

An independent review checked sensor-gap handling, reference continuity,
measured-position safety, bounded motion, PASS criteria, and the absence of
ratio-based acceptance. Review identified the need to center farther inside the
acceptance band when noise crosses its edge; the final approach margin is
0.10 degrees, with final acceptance unchanged at +/-0.4 degrees.

The copied `src/sensor_support.h` matches M04 byte-for-byte (SHA-256):

```text
6527419E5ABBE8182A59348AC0189BD74C32C1986714EBF1B5B6D1F9CD8D00FE
```

Repository diff checks show all existing tracked files unchanged. All new source,
tests, and documentation are contained in `M05_closed_loop_pitch_control`.

These are software results, not measurements of the gimbal. Native fixtures do
not reproduce ESP32 pulse timing, real queue-drain latency, I2C electrical faults,
motor/load dynamics, or BNO fusion behavior. Use the README's physical test steps
and review the actual outbound/return errors before accepting the milestone.
