# Milestone 8 software validation

## Verified physical pitch mapping to BNO roll — 2026-09-13

The user's hand-motion test identified BNO roll as physical cradle pitch
(approximately -32 to +38 degrees), with raw BNO pitch near -4 to -2 degrees.
Inspection found the roll substitutions already present in M08. Every physical
pitch feedback path was audited: baseline windows, startup, safety travel limit,
current/error, relative commands, velocity/progress/settling, timing response and
telemetry all use roll directly or consume roll-derived state. Raw Euler conversion
and finite-vector validation remain unchanged. A source comment and startup banner
now make the mapping explicit.

ESP32 build and full host suite: **PASS** (40 startup/control, 30 pose, 25 pitch
readiness scenarios, 85 control-math checks, 362 pose-math checks, and watchdog
tests). Build usage: 34,972 bytes RAM, 357,889 bytes flash. The new `roll_mapping`
regression varies raw BNO pitch independently between -4 and -2 degrees while
checking roll-based readiness, absolute leveling, relative motion, velocity,
learned pulses/degree and physical-pitch telemetry. Existing low-accuracy,
stale/invalid-data, travel-limit, reset and abort scenarios also pass using roll.

Test-only repairs align synthetic motor direction with the existing firmware -1
sign and restore the invalid-baseline, wrong-direction and startup-busy injections
that were missing when inspected. Firmware direction signs and all other motor
constants remain unchanged. The earlier +1 fixture description below is historical.
PITCH-ONLY READY, yaw/heading, carriage, correction math and watchdog behavior were
not changed. No firmware upload or powered motor test was performed.

## Pitch/north readiness separation — 2026-09-13

ESP32 `esp32dev` build: **PASS**. Host suite: **PASS** (40 startup/control
scenarios, 30 pose scenarios, 24 pitch-readiness scenarios, 85 control-math
checks, 362 pose-math checks, and the independent watchdog tests).

Pitch-only coverage includes accuracy 0/1 through leveling and subsequent moves,
stable pitch with unstable heading, every pitch-baseline criterion, invalid/stale/
unstable baseline rejection, calibration recovery before the deadline, no automatic
yaw authorization on later accuracy recovery, yaw/carriage command rejection,
blocked UART readiness output, idle stale/reset faults, active stale/invalid/wrong
reports, blocked BNO reads, resets, operator abort, direction/progress/travel guards,
and the active deadline. The simulated independent watchdog stops at the existing
150 ms deadline even with `referenceSet == false`. Separate cases cover low
accuracy during pitch-only motion after calibrated startup and preservation of
carriage-only accuracy grace. Active yaw now latches a stop on its first accepted
low-accuracy sample; baseline and settling accuracy requirements for north remain.

Before these edits, the existing synthetic pitch motor assumed sign -1 although
the actual firmware is +1. Its response and pose-math regression inputs also
assumed an older pitch correction gain. Test fixtures were updated for the current
unchanged firmware: synthetic pitch response 0.01 deg/pulse, sign +1; progress
regression 1200 pulses/degree and caps 9/10. These are test inputs, not measured
mechanical ratios. No firmware signs, gains, rates, accelerations, correction math,
pins, watchdog implementation or direction diagnostic were changed.

Commands used: `tests/run_host_tests.ps1` and PlatformIO `run -e esp32dev`.
No upload or hardware test was performed. The earlier validation below is retained
as historical context; its fixture values and blanket accuracy policy predate this
readiness separation.

## Earlier validation

Validated 2026-09-12. Physical testing of this revision is pending.
No ESP32 upload, commit, or push was performed.

## Build: PASS

PlatformIO `esp32dev`, Espressif32 7.0.1, Arduino-ESP32 2.0.17:

- FastAccelStepper 1.2.7.
- Adafruit BNO08x 1.2.7, BusIO 1.17.4, Unified Sensor 1.1.15.
- RAM: 34,940 / 327,680 bytes (10.7%).
- Flash: 355,617 / 1,310,720 bytes (27.1%).
- Image: `.pio/build/esp32dev/firmware.bin`.

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
```

## Native tests: PASS

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
```

The final suite passes:

- **40 preserved M07 north/level integration scenarios**, adapted for the existing
  current pitch sign -1, the idle carriage connection, and command-ready BNO service.
- **26 M08 command/control integration scenarios**.
- **85 existing control-math checks**.
- **362 pose timing-math checks**.
- The extended watchdog tests, including two- and three-motor stop requests.

Tests compile the actual firmware with strict C++11 warnings as errors. Only the
standalone watchdog target suppresses unused shared-fixture variable warnings.
The ESP32 timer implementation is replaced by an explicitly advanced independent
host tick in these fixtures; this is decision testing, not ESP32 scheduling proof.

New coverage includes simultaneous three-axis starts, speed scaling, arrival
spread, already-satisfied axes, no-op commands, angle-only/carriage-only commands,
absolute and relative moves, repeated targets, strict parsing and bounds, partial
and oversized lines, busy rejection, missing angular timing response, stale/low
accuracy at command entry, idle manual angular movement with fresh BNO data,
idle reset/outage latching, X/x latching, all-three stopping on blocked BNO reads,
carriage rejection/timeout, and continued BNO acquisition with blocked UART output
through completion and READY.

Timing tests include triangular/trapezoidal profiles and inverses, conservative
slew-braking predictions, precision correction delays, whole-Hz speed caps, invalid
responses, and unchanged watchdog feasibility. A regression rejects pitch error
0.5 degrees at 300 observed pulses/degree and cap 2 Hz: its individual bursts
fit the burst limit, but cumulative progress cannot meet the 15-second watchdog.
The planner selects a faster safe cap and reports limited timing matching.

## Synthetic synchronization observations

All three required axes begin within one 15 ms host observation interval.
For `POSE 45 25 450` after successful north/level startup:

| Synthetic angular response | Planned duration | Yaw stopped | Pitch stopped | Carriage stopped | Arrival spread |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0.020 deg/pulse yaw; 0.025 pitch | 14.147 s | 13.747 s | 13.506 s | 14.094 s | 0.588 s |
| 0.0075 deg/pulse on both axes | 29.493 s | 29.007 s | 28.777 s | 30.017 s | 1.240 s |

Times are relative to command submission in the fixture and exclude the final
shared one-second settling window. They are not hardware measurements. The small
first-test command `MOVE 2 -2 100` also passes with all three axes overlapping.

Fixtures use independently integrated acceleration-limited motors and BNO angle
feedback. They idealize pulse generation/force-stop behavior and do not reproduce
real gearing, torque, missed steps, fusion lag, magnetic errors, backlash, ESP32
queue latency, or I2C electrical faults. The carriage has no physical feedback in
firmware or the tests; count completion must not be reported as measured rail
position. Approximate synchronization and usable angle accuracy require physical
confirmation using README.md.

## Change scope and baseline preservation

New project: `diagnostics/M08_go_to_pose`, following existing sibling milestones.
M07 was untracked during inspection and was not overwritten or committed.

Within the new M08 project:

| File | Change relative to M07 |
| --- | --- |
| `src/main.cpp` | Third motor, POSE/MOVE parser and operation lifecycle, preflight timing/caps, BNO-based timing response, three-axis completion/stops/telemetry, command-ready feedback and queued terminal summaries. |
| `src/pose_math.h` | New bounded timing estimates and speed-cap selection. |
| `src/motion_watchdog.h` | Optional third motor stopped by the existing independent watchdog; M08 timer name. |
| `src/control_math.h` | Copied unchanged, preserving current pitch/yaw signs, gains, speeds and accelerations. |
| `src/sensor_support.h` | Copied unchanged. |
| `platformio.ini` | Copied unchanged; uses the existing shared hardware include and pinned dependencies. |
| `tests/stubs/Arduino.h`, `tests/stubs/FastAccelStepper.h` | Independent third motor and line-input fixtures. |
| `tests/north_level_integration_test.cpp`, `tests/watchdog_test.cpp`, `tests/run_host_tests.ps1` | Preserved behavior checks adapted/extended for M08. |
| `tests/m08_pose_integration_test.cpp`, `tests/pose_math_test.cpp` | New coordination/parser/fault and timing tests. |
| `README.md`, `VALIDATION.md` | M08 identification, encoder audit, references/units, limits, commands and test procedure. |

The remaining inherited test files are unchanged. No edits were made to M05,
M06, M07 source/tests, the shared hardware configuration, or legacy encoder code.
The lack of an off-axis yaw mapping and of a physical carriage datum/mm scale is
explicit; no implementation or absolute position capability was invented.
