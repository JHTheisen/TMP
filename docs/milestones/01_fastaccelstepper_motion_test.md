# Milestone 1: Three-Axis FastAccelStepper Motion Test

Status: Physically successful
Test date: 2026-09-07

## Scope

This milestone validates a minimal, open-loop, three-axis STEP/DIR motion test on
the current TMP hardware. It does not include sensors, homing, closed-loop
control, coordinated motion, controller input, networking, or higher-level
command handling.

## Verified Current Hardware Mapping

| Axis | DIR pin | STEP pin |
| --- | ---: | ---: |
| Yaw | GPIO 32 | GPIO 33 |
| Pitch | GPIO 26 | GPIO 12 |
| Linear carriage | GPIO 21 | GPIO 22 |

All three mappings were established by physical testing on the current machine.
The Version 5 pin assignments remain historical reference information only.

## Build Configuration

- PlatformIO board: `esp32dev` (`Espressif ESP32 Dev Module`)
- PlatformIO Espressif32 platform: `7.0.1`
- Arduino-ESP32 core: `2.0.17`
- FastAccelStepper: `1.2.7`
- Test distance: 500 step pulses in each direction
- Maximum configured speed: 1,000 steps per second
- Acceleration and deceleration: 1,000 steps per second squared

The firmware compiled successfully before the physical test.

## Physical Test Result

The user reported that:

- Yaw, pitch, and linear carriage each moved in both directions as expected.
- Only one axis moved at a time.
- No obvious motion problems occurred.
- The firmware completed each negative move and reported a return to the
  starting step count.

The absolute mechanical return to each exact starting position was not
independently measured and therefore remains unverified. This result does not
establish positioning accuracy, repeatability, homing accuracy, or freedom from
missed steps. No sensors or other verification mechanisms were added for this
milestone.

## Milestone Boundary

Milestone 1 ends with this recorded result. No later motion features or sensor
integration are included in this baseline.
