# Milestone 2: Dual-I2C Sensor Verification

Status: Milestone complete; second AS5600 mechanical verification remains pending

## Scope

This milestone verifies the two AS5600 sensors and one BNO085 IMU across the
two I2C buses on the current TMP hardware. It is an isolated diagnostic and
does not command any motors.

## Authoritative Bus Configuration

| Bus | SDA pin | SCL pin | Required device |
| --- | ---: | ---: | --- |
| A | GPIO 18 | GPIO 19 | AS5600 at `0x36` |
| B | GPIO 4 | GPIO 5 | AS5600 at `0x36`; BNO085 at `0x4A` |

The diagnostic probes both fixed buses for a BNO085 at `0x4A` or `0x4B` rather
than assuming its location. Physical testing confirmed that the currently
installed BNO085 shares bus B with its AS5600 and responds at `0x4A`.

## Diagnostic Behavior

- Initializes both I2C controllers at 100 kHz using the shared hardware pin
  configuration.
- Initializes one bus-specific AS5600 library object on each bus.
- Probes both buses for a BNO085 at `0x4A` and `0x4B`, then initializes it on
  the responding bus.
- Enables the BNO085 absolute rotation-vector report.
- Prints both AS5600 raw angles, angles in degrees, and magnet status.
- Prints BNO085 heading, pitch, roll, calibration accuracy, and reading age.
- Keeps reporting available devices when another device is missing or fails.

## Build Result

The diagnostic compiled successfully for the `esp32dev` target using PlatformIO
Espressif32 `7.0.1`, Arduino-ESP32 `2.0.17`, AS5600 `0.6.7`, and Adafruit
BNO08x `1.2.7`.

## Physical Test Result

Physical testing confirmed the following on the current TMP hardware:

- I2C bus A initialized successfully on SDA GPIO 18 and SCL GPIO 19.
- I2C bus B initialized successfully on SDA GPIO 4 and SCL GPIO 5.
- Both AS5600 devices were detected and communicated at address `0x36`, one on
  each bus.
- The currently magnetically and mechanically installed AS5600 produced
  logical changing angle readings when its magnet/axis was moved.
- The second AS5600 is electrically verified, but its magnet and mechanical
  installation are not complete. Its absolute mechanical angle response is
  therefore still pending and is not considered verified by this milestone.
- The BNO085 was detected on bus B at address `0x4A`.
- BNO085 heading, pitch, and roll changed logically when the sensor was moved.
- The installed AS5600 and BNO085 operated simultaneously without obvious
  dropouts or bus conflicts.
- The BNO085 startup reset-detected event successfully re-enabled the
  rotation-vector report, after which normal orientation readings followed.
- No motors moved while the diagnostic was running.

This completes Milestone 2 for electrical detection, simultaneous dual-bus
communication, and live verification of the currently installed sensors. The
second AS5600 must be mechanically verified after its magnet and mechanical
installation are complete.
