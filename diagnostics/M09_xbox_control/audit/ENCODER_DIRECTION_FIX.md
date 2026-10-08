# Encoder direction regression — 2026-10-08

Evidence: `logs/m09_20261008T142253.543658Z_28016_0.log` (existing local log,
not a new hardware run). No upload, motor command, commit or push was performed
while making this fix.

## Diagnosis and full direction path

The host sent `CELESTIAL_GOTO 3 126.831845 24.489174` at +24.766 s. Both
calibrated positions were approximately zero, so both target-minus-position
errors were positive. Host north mode was true, boresight offsets were zero,
and firmware geometry was yaw +360 and pitch +24.0000012 degrees/encoder turn.

GOTO selected SLEW for both axes. The previous `axisSlewStepSign()` reused the
manual `-1` direction mapping, selecting `runBackward()` for both positive errors.
Both motors use `setDirectionPin(..., true, 200)`: forward uses DIR high,
backward DIR low, with FastAccelStepper retaining pulse/acceleration ownership.
The manual pitch trace independently records runBackward/negative motor rate
with GPIO26 low, and runForward/positive motor rate with GPIO26 high.

At +25.734 s the measured yaw was -18.80859 degrees (wrapped 341.19141) and pitch
was -7.91016 degrees. At +26.781 s errors had grown to +193.72044 and +50.21430
degrees. Feedback remained fresh, both references valid, magnets good, and
encoder failure counts zero. Target movement was only about 0.004/0.0025 deg/s.
The yaw progress guard requested braking at +26.812 s. This was divergence,
not a lease failure, missing calibration, gear-ratio error or insufficient timeout.

Combining the commanded motor direction from the source with measured log motion
establishes that negative FAS travel decreases both raw encoder coordinates on
this wiring; positive travel increases them. Earlier manual yaw also supports
this: sustained negative JOG (positive FAS direction) increased bus A raw angle
from about 102.2 to 174.1 degrees.

| Stage | Yaw | Pitch |
| --- | --- | --- |
| Host target | Clockwise azimuth from declared north | Altitude from declared level |
| Error | Shortest initial yaw target, then continuous target minus measured yaw | Target minus measured pitch |
| Encoder geometry | Direct output sensor: 360°/encoder turn | Motor-shaft sensor: 360/15 = 24°/encoder turn |
| Raw feedback polarity vs FAS steps | Positive steps increase raw counts | Positive steps increase raw counts |
| Fixed positive-error command with positive scale | Forward / positive finite steps | Forward / positive finite steps |
| Fixed negative-error command with positive scale | Backward / negative finite steps | Backward / negative finite steps |

Yaw's 9:1 **motor** reduction is not an encoder conversion. It is not applied
to encoder readings. Pitch shaft readings unwrap before the 1/15 conversion;
wrapped shaft position alone cannot represent the full cradle angle.

The log establishes motor-to-encoder polarity. It does not independently prove
compass alignment or physical clockwise/upward convention; those require the
operator check below. The positive supplied encoder scales are preserved.

## Implementation

`src/main.cpp` now composes the observed raw-feedback motor sign with the sign
of the configured encoder scale. Slew, finite precision corrections, response
learning, TRACK feedforward/residual correction, and TRACK progress guards all
use that same sign. NORTH/LEVEL/POSE share this measured-position mapping.

Manual JOG still uses its existing -1 mapping on each rotary axis. Keyframes
continue using their original generated-step coordinates; carriage is unchanged.
No encoder magnitude, motor-rate conversion, timeout, acceleration, pulse engine
or safety threshold was changed. `CELESTIAL_STATE` now includes
`yaw_positive_step`, `pitch_positive_step`, `yaw_motor_mHz`, `pitch_motor_mHz`
to make future direction diagnosis explicit (motor rates are commanded/reported
driver rates, not encoder measurements).

The old encoder integration fixture simulated positive FAS steps decreasing raw
counts, masking the regression. Fixtures now explicitly model the observed
positive-step/increasing-count relationship, independent of production signs.
The new hardware-geometry fixture uses direct yaw and 15:1 pitch, including pitch
shaft wrap, rather than the older synthetic geometry.

## Regression checks

Before the production fix, the new yaw-positive and pitch-positive slew tests
both failed their measured-progress assertion. After the fix, all 17 focused
cases pass: both signs on both axes for slew/precision, negative configured
scales, both signs of TRACK residual corrections, the logged simultaneous GOTO
target, actual sensor geometry, and intentionally reversed wiring still stopping.
The positive/negative GOTO cases also settle into TRACK within existing tolerance.

The full C++ runner passed 135 suite/scenario executions, including unchanged
manual direction assertions, keyframes, carriage, STOP/takeover, sensor faults,
and low-rate/zero-rate/reversal behavior. Evidence: `.pio/direction-cpp.log`.
ESP32 compile passed: RAM 33,332 bytes, flash 368,449 bytes.
Evidence: `.pio/direction-build.log`. These are software-only checks.
The complete Python suite also passed all 236 tests; evidence:
`.pio/direction-python.log`. `git diff --check` passed.

## Brief supervised hardware check (after an operator-controlled upload)

1. Start the batch file with yaw ratio `1`, pitch `0.06666667`. Confirm fresh
   encoders. Jog small known movements: clockwise yaw viewed from above must
   increase calibrated yaw, and raising the camera must increase pitch. If this
   physical convention disagrees, stop and resolve that axis's encoder scale
   sign before trying celestial pointing. Do not change the ratio magnitudes.
2. Align to true north and horizontal, then SET NORTH and SET LEVEL. In F2,
   wait for stopped READY and issue `MOVE 1 0 0`, wait for completion, then
   `MOVE -1 0 0`. Repeat with `MOVE 0 1 0` and `MOVE 0 -1 0`. Each request should
   move the displayed axis toward its target, leaving the other axis stationary.
   STOP immediately if error grows. Exit F2 before using the target UI.
3. Choose a nearby above-horizon target away from the Sun. Start GOTO and confirm
   both error magnitudes decrease. With positive encoder scales, positive errors
   should initially show positive motor mHz, negative errors negative motor mHz.
   Confirm transition to TRACK, then test STOP and deliberate joystick takeover.

Physical pointing accuracy and the final powered verification remain outstanding.
