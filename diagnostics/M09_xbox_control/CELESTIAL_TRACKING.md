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
