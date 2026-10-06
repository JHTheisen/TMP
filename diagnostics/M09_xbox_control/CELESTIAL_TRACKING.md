# Celestial TRACK rate control

## Cause of the former stop/correct cycle

`beginCelestial()` admits GOTO and initializes the existing POSE axes.
`serviceAxes()` runs `axisProgress()` and `serviceAxis()` and, after stopped
settling, calls `enterCelestialTracking()`. Previously TRACK kept calling those
same functions. It only bypassed terminal settling and the GOTO deadline.
`updateCelestial()` shifted positional targets but supplied no angular velocity.

`serviceAxis()` leaves HOLD only when absolute error exceeds 0.4 degrees. It
then commands finite precision moves until error is within 0.1 degrees. The
separate entry/exit thresholds provide hysteresis, and the existing 100 ms
post-stop observation explicitly allows mechanical/BNO filter settling. This
avoids repeatedly commanding bursts on small measurement fluctuations; it is
not a photographic tracking algorithm.

The matching log is `logs/m09_20261004T164306.062823Z_37340_0.log`:

- Line 298: +38.453 s, YAW -> PRECISION, error +0.402 degrees.
- Line 303: +39.172 s, YAW -> HOLD, error +0.025 degrees.
- Line 310: +40.422 s, YAW -> PRECISION, error -0.400 degrees.
- Line 287: BNO fresh, accuracy 3, north usable.
- Lines 410-423: successive updates remain about one second apart.

Yaw heading variation in that run is much faster than the approximately
0.003 deg/s commanded celestial motion. Accuracy status 3 does not establish
arcsecond stability. Pitch stays inside HOLD's tolerance during the observed
interval. Neither a lease failure nor an encoder fallback is needed to produce
this cycle.

## TRACK algorithm

Only CELESTIAL_TRACK dispatches to `serviceCelestialTracking()`; GOTO acquisition
and all noncelestial controllers retain their original dispatch.

1. Each accepted target update computes angular velocity from target delta /
   elapsed ESP32 receive time. Yaw uses the shortest wrapped difference.
   The first velocity initializes the estimate; later estimates use
   `alpha = dt / (2 seconds + dt)` to attenuate arrival jitter. Rejected updates
   change neither velocity nor lease. Serial commands and host astronomy are
   unchanged; no timestamp field is added to the protocol.
2. Configured STEP pulses per cradle degree convert angular velocity to motor rate:
   yaw 52.0, pitch 67.2. These provisional constants are centralized in
   `celestial_motion.h`; learned timing estimates cannot replace them. Existing motor signs,
   hardware pulse generation, speed ceilings and acceleration are retained.
3. On fresh feedback, at most every 100 ms, predict the target at the sample's
   receipt time: `last_target + target_rate * sample_age_from_update`.
   Low-pass the resulting residual with a 10 s time constant. Outside the
   existing 0.1 degree band, apply gain 0.02/s, limited to +/-0.01 deg/s and
   changing by at most 0.002 deg/s per second. This band affects only residual
   correction; it never suppresses celestial feed-forward.
4. The sum of feed-forward and residual correction is bounded by the existing
   1 deg/s target-rate limit and existing motor speed caps. FastAccelStepper
   receives fractional-Hz continuous rates via `setSpeedInMilliHz()`. Updating
   speed does not restart the motor. Direction changes brake, confirm stopped,
   and wait for the existing fresh post-stop observation before reversing.
5. The existing 3000 ms target lease, position/travel guards and STOP/abort paths
   remain active. Extrapolated targets also obey travel guards. A filtered
   residual exceeding the existing 0.4 degree tolerance must improve by
   0.1 degree within 60 seconds or tracking fails safely. This replaces burst
   progress timing only for continuous TRACK, where motion can be below one
   pulse per second. Motor command failure/unexpected stopping also fails safely.
6. Sensor fallback still latches only when both AS5600 axes are control-ready.
   Otherwise motors pause and the session can resume after acceptable BNO
   recovery and confirmed stopping. Continuous TRACK participates in that
   braking path. No optional-sensor requirement is added to manual operation.

## Startup and physical limits

The configured pulses/degree constants are positive magnitudes, not signed
motor conversions. TRACK adds target_dps and correction_dps in BNO coordinates,
then `bnoSlewStepDirection()` applies the existing physical motor sign. During
CELESTIAL, pitch uses `POSITIVE_STEP_PITCH_SIGN = -1`: increasing pitch commands
negative STEP Hz (`runBackward`), decreasing pitch commands positive STEP Hz
(`runForward`). Thus +0.003 pitch deg/s gives nominal -0.2016 STEP Hz with 67.2.
Residual pitch correction follows the identical sign path. Telemetry applies
`bnoFeedbackStepSign()` to nominal/correction rates, also -1 during CELESTIAL.
Do not negate the magnitude constant: direction is already applied separately.
LEVEL's separate sign selection, GOTO, yaw and manual mappings are unchanged.
The supplied GOTO/takeover run did not demonstrate reversed TRACK motion.

A fresh TRACK HERE uses configured conversions immediately upon entering TRACK.
It still needs two successive target positions to establish velocity (normally
the first CELESTIAL_UPDATE); it never waits for a measured motor response or
falls back to positional catch-up. GOTO and its learned deadline estimates are
unchanged. Sensor pauses, stopped reversal handling and lease expiry still apply.

At TRACK entry and every two seconds, two `CELESTIAL_RATE` lines report `axis`,
`source=CONFIGURED`, `pulses_per_degree`, `rate_known`, `feedback`, `target_dps`,
`nominal_hz`, `correction_dps`, `correction_hz`, `commanded_hz`, and `error_deg`.
Hz values are signed STEP rates; angular values follow the BNO axis convention.
`commanded_hz` is the continuous command, zero while stopped/paused/braking;
it is not a measurement of instantaneous pulses during deceleration. Correction
uses AS5600 feedback after a qualified permanent fallback, otherwise BNO.

## Provisional powered evidence (2026-10-04)

All files below are in `logs/`. No nominal gearing or controller gain is used.

| Log | Snapshot lines | STEP A -> B | BNO A -> B | Ratio |
| --- | --- | --- | --- | --- |
| m09_20261004T220825.926621Z_27736_0.log | 78,117 | pitch 0 -> 2769 | -17.530 -> -58.615 | 67.397 |
| m09_20261004T220900.133132Z_33716_0.log | 102,135 | yaw 2141 -> -741 | 171.476 -> 240.382 | 41.825 |
| m09_20261004T221014.320950Z_17932_0.log | 73,101 | yaw 0 -> -1466 | 231.570 -> 262.053 | 48.092 |

All three intervals have fresh accuracy-3 endpoints, no reversal, no other
commanded axis movement, and no intervening reset/stall/malformed report.
Yaw deltas are +68.906 and +30.483 degrees, with no wrap crossing. Pitch's
later pre-reset stopped reading -58.772 gives 67.140; choose 67.2 with earlier
approximately 67.17 supporting evidence. This is not a precision calibration.

Neither yaw capture has a stable heading endpoint. The first later drifts to
234.735 (45.559 pulses/degree); the second to 258.987 (53.470). The second
is a simpler single move, but neither ratio should be treated as exact. Choose
52.0 as a deliberately rounded provisional compromise between the newest later
53.47 estimate and the earlier isolated playback's 51.677 (20261003T030157,
motor lines 4188-4246, angle lines 4184/4257). This is NOT an average of clean
calibrations. Heading drift, especially near pitch -60 degrees in these runs,
remains unresolved. No claim of photographic accuracy is made from these logs.

Foreground BNO acceptance now also enforces the existing plausibility bounds:
finite quaternion norm-squared within 0.05 of one and finite nonnegative heading
accuracy. Rejection uses existing invalid-orientation handling, including
celestial pause/fallback, without gating manual motion or STOP. This does not
repair I2C stalls, sensor resets, or plausible-but-drifting magnetic heading.

The installed ESP32 FastAccelStepper backend cannot represent rates below
5 milliHz with its 32-bit tick interval; those rates are treated as zero rather
than rounded upward. Ordinary rates are rounded to the nearest milliHz.
Microstep/cradle resolution, backlash, BNO drift and provisional calibration accuracy
still limit image stability. Software simulation and compilation do not replace
powered validation. No firmware upload is part of this change.

## Sub-hertz braking and restart correction (2026-10-06)

The trace below is against the installed FastAccelStepper **1.2.7** source,
not inferred from the physical motor's apparent motion. Changes cover celestial
TRACK's transition state, zero-demand progress handling, stop diagnostics and
celestial cancellation's force-stop repetition. The rate/residual calculation,
motor mappings, Python, GOTO, LEVEL and NORTH remain unchanged. No firmware was
uploaded.

| Transition | Firmware / FastAccelStepper path |
| --- | --- |
| Start | Configure acceleration and milliHz; `runForward()` / `runBackward()` calls `RampGenerator::startRun()`. This activates the ramp before the background task fills/starts the hardware queue. |
| Rate update | `setSpeedInMilliHz()` and `applySpeedAcceleration()` publish parameters. An existing long `pause_ticks_left` is still processed before the next step's timing changes. |
| Zero demand | Rates below 5 milliHz enter TRACK_ZERO. A pending period of at least 3 seconds with at most one ramp step remaining is canceled with one `forceStop()` request while the ramp is active; faster motion uses normal deceleration. Queue draining and a fresh stopped sample at least 100 ms later must complete before another run. |
| Reversal | Nonzero direction changes call `stopMove()` and mark the axis BRAKING. An opposite-direction run waits until the motor is idle and a fresh stopped sample at least 100 ms later has arrived. |
| Normal braking | `stopMove()` sets `force_stop`; `_getNextCommand()` emits remaining zero-step pauses before it evaluates deceleration. An integer ramp-down step can retain another full slow period. |
| Running | `isRunning()` is true when the queue runs, the ramp is active, or the queue is nonempty. Tens of seconds without a STEP edge can therefore be entirely normal. |
| Forced stop | `forceStop()` sets `force_immediate_stop`; the ramp task consumes it, clears it and stops generating commands. Already queued short commands still drain. |
| Restart hazard | Calling `forceStop()` again while only the queue is draining can leave a new flag after ramp idle. `startRun()` clears ordinary `force_stop`, but not this immediate-stop flag. The next ramp service consumes the new continuous start without issuing a queue command. |

Previously BRAKING failed after a fixed 3000 ms. TRACK's first successful
`run*()` return also immediately established expected-running state; finding
`isRunning() == false` after 100 ms failed the session. Admission checked all
motors idle and reset the firmware axes, but that did not clear the library's
pending immediate-stop flag.

Normally decelerated internal TRACK braking captures this deadline once, before
`stopMove()`:

`3000 ms + (1 + stepsToStop()) * max(applied period, last requested period)`

Periods are rounded upward to milliseconds. The applied period is read from
`getPeriodInUsAfterCommandsCompleted()` after queue-confirmed launch; the request
is derived from the last nonzero milliHz command. This covers an older slow
interval even if a faster request has just been submitted, plus the remaining
ramp steps and the existing scheduling margin. The deadline is not renewed
while waiting. At 0.020 Hz with one ramp step, it is 103 seconds, not 3 seconds.
Reversal may consequently take that long; this retains normal deceleration and
does not insert dummy steps or alter pulse generation. A stuck brake still
fails at the captured deadline. Intentional braking time is excluded from the
resumed servo's existing 60-second no-progress window.

Zero-demand cancellation of a slow pending pause instead uses the original
3000 ms stop budget. It never resets the motor position or repeatedly requests
a forced stop while the queue drains. TRACK_ZERO continues evaluating the
residual filter and can resume in either direction after stop confirmation.
Zero demand, including feed-forward and residual correction canceling each
other, does not count as a failure to make position progress. The 5 milliHz
minimum is unchanged. Each transition emits `CELESTIAL_AXIS_STOP` with its
reason, stop method, previous requested rate, applied period, remaining ramp
steps and captured timeout.

A start is now unconfirmed until `isQueueRunning()` acknowledges execution,
including pause-only commands; it does not wait for a physical STEP. Before
that acknowledgment only, one retry is permitted after at least 100 ms and
only with `isRunning() == false` (ramp and queue both idle). Both attempts share
a 3000 ms startup deadline. A second idle failure or missing queue acknowledgment
fails with `TRACK motor failed to start`. Once confirmed, the existing
`TRACK motor stopped unexpectedly` check remains and never retries a lost run.
New runs reset the launch confirmation/retry state; new sessions reset all
TRACK state. Celestial cancellation issues forced stops once, rather than
continually relatching them while the queues drain.

Operator STOP, host joystick takeover via STOP, and lease expiry still use the
original cancellation path with its 3000 ms braking limit and forced-stop
fallback. The **3000 ms celestial update lease is unchanged**, including during
long internal brakes. Sensor pause/fallback uses the same normal TRACK brake;
other operation modes retain their prior braking behavior.

Validation includes `fas_low_rate_test.cpp`, which compiles the unmodified
installed ramp implementation with its PC backend (16 MHz ticks): it reproduces
about 100 seconds of normal stopping at 0.020 Hz, verifies an older pending
period survives a rate update, and reproduces/clears the forced-stop restart
flag without dummy steps. `celestial_low_rate_test.cpp` exercises the real
firmware dispatcher with long pauses and asynchronous queue draining: 0.005
and 0.020 Hz motion, zero/below-minimum rates, reversal, pending-period capture,
bounded braking/start failure, confirmed unexpected stops, forced-stop restart,
STOP/manual handoff, lease expiry and sensor pause/recovery. These deterministic
tests do not emulate the ESP32 interrupt scheduler or validate physical tracking
accuracy; powered validation remains outstanding.

`celestial_zero_rate_test.cpp` additionally checks exact feed-forward/correction
cancellation, demand fluctuating within the zero band, a 210-second zero hold,
same-direction restart, both reversal directions through zero, failed stopping,
unexpected running-motor loss, STOP/manual takeover and lease expiry. The
installed-library test also checks cancellation of an unqueued slow pause and
subsequent starts in both directions without relatching an immediate stop.
