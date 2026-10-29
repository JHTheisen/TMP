# Autonomous yaw direction correction — 2026-10-28

The current mount requires negative FastAccelStepper (FAS) yaw steps to increase
clockwise encoder heading. Autonomous control assumed the opposite. The fix
changes only the yaw entry of `ENCODER_RAW_POSITIVE_STEP_SIGN` in `src/main.cpp`:

```diff
-constexpr int ENCODER_RAW_POSITIVE_STEP_SIGN[2] = {1, 1};
+constexpr int ENCODER_RAW_POSITIVE_STEP_SIGN[2] = {-1, 1};
```

The accompanying source comment explains the newer evidence. No manual mapping,
pitch mapping, encoder scale, motor pin, acceleration, timeout, stop logic or
FastAccelStepper pulse generation was changed. Existing unrelated workspace
edits were preserved. No firmware upload or motor operation was performed.

## Evidence and root cause

Source: `logs/m09_20261028T092214.328503Z_598864_0.log`.

- Line 1 records `invert_yaw=false`, yaw ratio 1 and pitch ratio -0.06666667.
  Firmware accepts scales +360 and -24.0000012 degrees per encoder turn.
- Lines 211–218: Saturn target 265.112481 degrees produces initial yaw error
  -94.888 degrees. This is the correct short negative path from north.
- Lines 221–233: `yaw_positive_step=1`, negative reported motor rates
  (-558032, -1557632, -2000000 mHz), and increasingly negative errors
  (-94.88752, -108.07111, -130.65608, -154.38655). Target updates total only
  about +0.006 degrees; they cannot explain the divergence.
- Raw bus A angle independently increases: 2598 counts / 228.340 degrees
  before the move (line 202), 3688 / 324.141 while braking (line 238), and
  3711 / 326.162 after stopping (line 259). This segment does not even cross
  raw zero. The approximately +97.8-degree raw change agrees with calibrated
  heading. It is not a manufactured full turn in the unwrapping calculation.
- Pitch makes progress using its existing -1 autonomous sign. Its positive
  error falls from 6.912 to 2.687 degrees while yaw diverges.
- The operator independently verified clockwise physical yaw increases the
  encoder angle, and positive manual yaw works. This connects the log's
  motor-to-encoder relationship to physical direction.

The faulty expression was the yaw `+1` in the array consumed by
`axisSlewStepSign()`. October 8 documentation and fixtures asserted positive
FAS yaw increased raw counts. That assumption contradicts the current log.
Why the hardware relationship differs from October 8 is unknown; the fix does
not presume a wiring change or alter working manual control.

## Direction paths

`xbox_control.py:stick_command()` applies deadband/scaling and optional
`invert_yaw` to joystick input. It does not transform celestial targets.
`executeManualCommand()` accepts `JOG`; `serviceManualAxis()` calls
`stepDirection(request, false)`. The retained default manual yaw sign is -1.

Both rotary motors are initialized with `setDirectionPin(..., true, 200)`.
The installed FAS 1.2.7 implementation maps `runForward()` to positive/count-up
motion and DIR high, and `runBackward()` to negative/count-down motion and DIR
low. No separate autonomous pin inversion is applied.

| Request with current encoder scales | Yaw FAS command | Expected measured yaw |
| --- | --- | --- |
| Positive manual JOG | `runBackward()` | Increasing / clockwise |
| Negative manual JOG | `runForward()` | Decreasing / counterclockwise |
| Positive autonomous error or TRACK rate | Negative steps / `runBackward()` | Increasing |
| Negative autonomous error or TRACK rate | Positive steps / `runForward()` | Decreasing |

Thus a **positive FAS motor rate** means decreasing yaw; a positive logical yaw
request means increasing yaw. These are deliberately distinct quantities.
`yaw_positive_step` reports the FAS step sign needed for increasing calibrated
yaw; with +360 scale it now reports -1. Pitch remains -1 with its -24 scale.

Autonomous execution:

1. `executeCelestialCommand()` dispatches GOTO to `beginCelestial()`, which
   chooses `continuousYaw + shortestDifference(target, wrappedHeading)`.
2. `serviceAxes()` computes target minus continuous measured position.
   `axisProgress()` evaluates error and progress; `serviceAxis()` handles
   slew, braking, stopped observation, precision and hold.
3. `commandAxis()` uses `axisSlewStepDirection()` for `runForward/Backward`
   and `axisPrecisionCorrectionSteps()` for signed finite `move()` bursts.
   Both compose the raw motor sign with the configured encoder scale sign.
4. UPDATE enters `updateCelestial()`: bounded shortest target increments are
   applied to the existing continuous target, with compensation of error
   histories. It does not restart the move or reselect a long path.
5. After stopped settling, `serviceCelestialTrackAxis()` uses the same sign
   for trajectory feedforward plus residual correction. Timing-response
   learning and TRACK progress checks use `axisFeedbackStepSign()` too.

The same corrected measured-yaw mapping also applies to NORTH and POSE/MOVE;
generated-step keyframe coordinates and carriage control remain unchanged.

## Encoder and watchdog review

`EncoderAcquisition` reads STATUS and 12-bit RAW_ANGLE from AS5600 registers
starting at 0x0B, alternating buses at approximately 20 ms per encoder.
`EncoderPosition::observe()` unwraps successive raw deltas in the acquisition
worker, so skipped foreground snapshots do not discard revolutions.
`EncoderReference` computes `(ticks - zeroTicks) * signedScale / 4096`;
SET NORTH records the zero tick count without moving the motor.
`serviceEncoders()` uses this value directly for continuous yaw and wraps only
the heading representation. Yaw uses the direct-output +360 scale, not a motor
gear ratio. Pitch shaft unwrapping precedes its signed -24 scale conversion.

Failed reads, bad magnet status, stale gaps of at least 150 ms and an exactly
half-turn sample delta invalidate continuity/references. There is an inherent
limit: more than half a raw revolution between good samples, or a skipped full
revolution, cannot be uniquely recovered by this sensor alone. The observed
yaw speed (roughly 45–50 deg/s) is far below half a turn per nominal 20 ms
sample, and the raw counts support the measured direction in this incident.
This change does not claim to explain every older continuous-angle anomaly.

The progress deadline updates only after at least 0.15 degrees of cumulative
error reduction (or when inside tolerance). With no learned response, SLEW
times out after 2000 ms without progress. The log shows that stop firing.
There is also a wrong-direction/runaway guard for sustained increases above
the best error. Its consecutive-rise requirement can reset on unchanged yaw
samples while the other encoder updates, so its reason string is not a reliable
classifier of stall versus divergence. The fallback progress deadline stopped
this actual divergence; no guard was weakened or retuned in this correction.

Failure invokes `stopCelestial()` / `beginPoseStop()`: prevent new motion,
brake every moving motor, wait for all stopped, then report failure/READY.
`servicePoseStop()` forces a stop if braking exceeds its deadline. Explicit X
remains a latched abort; feedback watchdogs, target leases and pitch limits
remain in place. The final logged yaw error was last calculated before braking
finished, whereas final heading was sampled after braking; those two final
fields need not form a contemporaneous target-minus-position pair.

## Regression verification

New `tests/yaw_direction_test.cpp` drives the real firmware serial dispatcher
with an independent current-mount plant (+360 yaw, -24 pitch, direct yaw and
15:1 pitch encoder geometry). Before the source fix:

- Positive GOTO failed: measured yaw -6.679688, error +96.679688.
- Logged Saturn GOTO failed: measured yaw +6.591797, error -101.479316.
- Positive manual JOG passed unchanged.

After the fix, all ten new cases pass: positive/negative manual yaw, 0 to 90,
0 to 270 via -90, 358 to 2, 2 to 358, the simultaneous Saturn yaw/pitch target,
frozen feedback, reversed response and latched abort. Successful GOTO cases
verify direction, shrinking error, endpoint, stopped settling and STOP; large
moves also verify braking. Existing direction fixtures cover both signs of
slew/precision/TRACK, signed encoder scales and pitch mounting. Old encoder
integration/stop/low-rate fixtures now model negative-FAS/increasing-raw yaw
independently of production constants.

All 154 C++ suite/scenario executions and all 250 Python tests passed.
ESP32 compilation passed with 33,332 bytes RAM and 368,449 bytes flash.
`git diff --check` passed.
Run `tests/run_host_tests.ps1` for the complete C++ and Python suites. Detailed
results are in `.pio/yaw-direction-tests.log`; firmware compilation is recorded
in `.pio/yaw-direction-build.log`. These are simulations and build checks,
not a powered validation of motor response or pointing accuracy.

## Limited hardware verification after operator-controlled upload

1. With clear yaw travel and STOP/X immediately available, confirm scales
   +360 yaw / -24 pitch and fresh valid encoders. Make brief, low-speed manual
   yaw movements in both directions. Right/positive must still increase heading;
   left/negative must decrease it. Stop before continuing.
2. Establish SET NORTH and SET LEVEL at the intended references. Enter the
   existing raw command mode (F2), wait for stopped READY, and send `MOVE 1 0 0`.
   Expect about +1 degree clockwise; then, after completion, `MOVE -1 0 0`
   should return about one degree counterclockwise. Pitch/carriage should stay
   still. STOP immediately if error grows; use X if normal stopping fails.
3. Only after both succeed, verify a small positive/negative yaw GOTO through
   the host's lease-maintaining target flow. Expect yaw error magnitude to
   shrink, braking near the endpoint and settled TRACK. With +360 yaw scale,
   positive yaw error requires negative motor mHz, negative error positive mHz.
4. Verify STOP and a small pitch-only move retain their established behavior.
   Then attempt a nearby celestial target before repeating a large Saturn slew.
   A raw CELESTIAL_GOTO without periodic matching UPDATEs expires after 3 seconds;
   do not bypass that lease for the check.
