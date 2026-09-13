# Standalone pitch direction check

This experiment discovers the relationship between raw motor STEP direction and
the BNO085 cradle pitch used by M08. It does not level the cradle, tune M08, or
use an assumed pitch sign. M07/M08 source files and constants are unchanged.

## Current M08 audit

- `src/main.cpp:387` uses `orientation.pitch` for pitch position and
  `target - current` for error. That orientation comes from the BNO085
  `SH2_ROTATION_VECTOR` quaternion via `sensor_support.h`.
- M08 never initializes or reads either AS5600. Pitch encoder angle/status do
  not enter feedback, corrections, direction logic, or runaway detection.
  An electrical fault on the shared I2C bus could nevertheless disrupt BNO
  communication; missing or bad AS5600 angles alone cannot drive M08 pitch.
- `src/control_math.h:37` multiplies the error sign by
  `POSITIVE_STEP_PITCH_SIGN` in `stepDirection()` for continuous slew.
- `src/control_math.h:89` does the same in `correctionSteps()` for finite
  corrections. Current pitch sign is +1 and pitch gain is 24.0. Negative
  pitch at target zero therefore commands positive motor pulses.
- `src/main.cpp:598` initializes pitch with `confirmed=true`.
  `src/main.cpp:291` explicitly excludes pitch from experimental direction
  confirmation. `resetPoseAxis()` also sets `confirmed=true` for pose moves.
- `learnTimingResponse()` at `src/main.cpp:91` accepts a pitch estimate only
  after successful stopped motion, at least 0.3 degrees, at least 8 net pulses,
  and `pitchSteps * pitchDelta * POSITIVE_STEP_PITCH_SIGN > 0`. It stores
  `abs(pitchSteps / pitchDelta)` for timing, not position feedback. It neither
  discovers nor changes the pitch sign. With +1, positive pulses causing
  decreasing BNO pitch cannot produce a newly accepted timing estimate.
- `src/main.cpp:288` aborts when current absolute error exceeds the smallest
  error observed in that operation by 0.6 degrees. A healthy I2C link does not
  prove the sign assumption or the physical accuracy of BNO pitch.

These references are relative to the sibling `M08_go_to_pose` project, inspected
2026-09-13. That project is not modified by this diagnostic.

## Test movement and hardware

Only pitch FastAccelStepper output is connected: STEP12, DIR26, direction-high
counts up, 200 microseconds DIR setup. Yaw STEP33 and carriage STEP22 are held
low and their motors are never connected to the stepping engine. Only I2C Bus B
is initialized (SDA4, SCL5, 100 kHz, 50 ms Wire timeout): BNO085 0x4A and
optional pitch motor-shaft AS5600 0x36.

Send `G` to run exactly one pair:

1. Collect a stopped, accurate BNO baseline for at least one second, at least
   50 samples, and at most 0.2 degree range. Read optional AS5600 while stopped.
2. Command **+200 raw motor pulses (DIR HIGH)** at **40 Hz**, acceleration
   **240 pulses/s^2**. No pitch-sign multiplier or error-dependent corrections.
3. Wait until the finite pulse move stops, then wait **two seconds**, then
   collect another stable one-second BNO window. Print before/after and delta.
4. Command **-200 raw motor pulses (DIR LOW)** with the same speed/acceleration.
   Repeat the stopped settling/measurement process and print the reverse delta.
5. Stop permanently until reset. No retries, extra backlash take-up moves,
   automatic leveling, or automatic motion on boot.

The earlier `pitch_drivetrain_characterization/src/main.cpp` records 200 full
steps/revolution and 8x microstepping: 1600 input pulses/motor revolution.
Combined with the user's 15:1 reduction, this predicts 66.67 pulses/cradle degree.
Thus 200 pulses is 45 degrees at the motor shaft and nominally 3 degrees at the
cradle before backlash. This assumption is explicitly conditional on unchanged
driver microstepping. It is not a measured calibration. The count also stays
within M08's existing 200-pulse correction cap; speed and acceleration are its
existing minimum correction speed and correction acceleration. Neither M08's
constants nor M08's four-second correction timeout are changed. This separate
40 Hz experiment needs about 5.2 seconds per move and has its own 10-second
move timeout.

Three degrees should be easier to distinguish from sensor noise/backlash than
M08's tiny near-zero corrections, but sufficient response is not guaranteed.
If either response is too small, the result is INCONCLUSIVE. The test does not
automatically increase the count. Equal reverse pulses do not guarantee exact
physical return through backlash, slip, or missed steps.

## Sensor comparison and stops

BNO conversion and rotation-vector report match M08. Direction uses stopped
mean-after minus mean-before, not a single transient sample. A response must
exceed max(0.3 degrees, sum of the two window ranges) to receive a direction.
Only opposite, resolved directions on the two legs produce a candidate sign:

| Positive pulses | Negative pulses | Candidate M08 sign |
|---|---|---|
| BNO increases | BNO decreases | +1 |
| BNO decreases | BNO increases | -1 |
| Unresolved or same direction | Any | Inconclusive |

The candidate is printed only; nothing changes firmware constants. Check
repeatability and observed physical motion before adopting a sign.

AS5600 reads are advisory, performed only at the three stopped snapshots, with
the watchdog disarmed and no motion queued. STATUS (0x0B) and RAW_ANGLE
(0x0C/0x0D) are read without writing device configuration. Output includes raw
angle and all three magnet bits: MD detected, ML too weak, MH too strong. A
detected bit does not override a weak/strong flag. Read failures, short reads,
or bad magnet flags never abort this test or prevent reversal.

The AS comparison reports the shortest wrapped endpoint difference. With the
documented drivetrain the expected shaft motion is only 45 degrees, below a
half turn. Changes under 1 degree or at least 120 degrees are inconclusive;
non-ideal endpoint magnet flags also suppress a direction conclusion. Good
flags do not prove good readings, and stopped endpoints cannot prove continuous
encoder validity or detect intervening full rotations. SAME/OPPOSITE compares
numeric signs only: mounting/view direction can make healthy sensors have
opposite signs. The shaft encoder angle is not cradle angle and is not used
to establish the BNO motor sign.

Independent 10 ms watchdog checks stop pitch if accurate, valid BNO feedback
ages to 150 ms during motion/settling/measurement. A late I2C return cannot
clear that fault. BNO reset, invalid quaternion, absolute pitch reaching 75
degrees, displacement reaching 6 degrees from the initial baseline, endpoint
mismatch, motion timeout, or failure to obtain a stable window also stop the
test. These are direction-neutral guards; this separate experiment does not
use M08's 0.6-degree progress/runaway test. M08 retains that guard unchanged.

`X`/`x` aborts when serial is serviced, including before startup movement.
After a fault there is no automatic reverse. FastAccelStepper forceStop drains
already queued pulses (normally about 20 ms); it does not disconnect power.
AS5600 data cannot abort the test, but a physical shared-bus fault that prevents
fresh BNO feedback still must stop motion. Serial output is buffered and drained
without blocking active acquisition. Motion waits for its command label to
drain and a new accurate BNO reading before starting.

## Build and run

From this folder:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
.\tests\run_host_tests.ps1
```

Close the joystick controller and any other serial-port clients. Position the
cradle with several degrees of clear pitch travel in both directions, away
from mechanical stops and the +/-75 degree BNO limit. Upload this separate
project when ready, then open its serial monitor at 115200 baud. For COM9:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run --target upload --upload-port COM9
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM9 --baud 115200 --filter log2file
```

Observe stationary LIVE pitch first. Send `G` (newline optional) and record the
SNAPSHOT, RESULT, and FINAL lines and actual physical motion. Send `X` if needed;
Ctrl+C exits the monitor, it is not the firmware abort command. Do not use the
M08 joystick/POSE interface with this standalone firmware.

## Verification and file scope

Host tests cover both physical sign mappings, unresolved response, absent/good/
weak/short-read AS5600, serial abort, stale BNO, blocked BNO with independent
watchdog stop, BNO reset, excessive excursion, move timeout, and blocked serial
output. They assert exactly two opposite 200-pulse commands on STEP12 at the
specified speed/acceleration for completed runs, no motion before G, no reverse
after a fault, and no restart after completion. These are synthetic tests, not
physical validation of this platform or its encoders.

New files are confined to this project. `sensor_support.h` is copied unchanged
from M08 to preserve its BNO conversion and transport workaround.
`motion_watchdog.h` is a local copy of M08's watchdog adapted for one motor.
`platformio.ini` retains M08's pinned dependencies and authoritative shared pin
header. The firmware has not been uploaded, committed, or pushed by Codex.
