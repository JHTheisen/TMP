# Continuous encoder tracking

The Python astronomy worker supplies wrapped yaw and pitch targets approximately
once per second. `CELESTIAL_GOTO id yaw pitch` starts acquisition;
`CELESTIAL_UPDATE id sequence yaw pitch` updates the same operation. IDs correlate
requests and increasing sequences reject late updates. Updates faster than 100 ms
are rejected; target rates beyond 1 degree/s stop the operation. The lease is
3000 ms. Python keeps scheduling while the dashboard is unfocused.

The ESP32 unwraps each 12-bit AS5600 reading in the sensor task, applies an
explicit signed degrees-per-revolution scale and operator zero offset, and uses
these positions for acquisition, velocity estimation, settling and track errors.
Only complete reads with magnet detected and neither weak nor strong flags are
usable. Each continuity epoch survives mailbox snapshot drops. Motor position
counts never synthesize orientation or teach encoder geometry.

Automatic step direction uses the observed positive-FAS-step/increasing-raw-count
relationship on both axes, composed with each configured encoder scale's sign.
This applies to slew, finite corrections and TRACK; manual stick directions are
separate and unchanged. Yaw feedback is direct output angle and is never divided
by the 9:1 motor reduction. Pitch feedback divides unwrapped shaft angle by 15.
See [direction evidence and hardware checks](audit/ENCODER_DIRECTION_FIX.md).

GOTO retains the existing finite/slew controller and direction/progress checks.
It settles within 0.4 degrees for one second with fresh post-stop observations.
TRACK then uses continuous FastAccelStepper run commands, updating speed rather
than issuing repeated positional bursts. Target velocities are filtered over
2 seconds. Residual error uses a 10-second filter, 0.1-degree deadband, gain
0.02/s, maximum correction 0.01 degree/s, and 0.002 degree/s² correction ramp.

The retained provisional motor conversions are yaw 52.0 and pitch 67.2 STEP
pulses per camera degree. These are feed-forward motor-rate settings from prior
work, not AS5600 conversion factors or newly measured calibration. Revalidate
them physically. GOTO timing estimates learned from encoder-measured displacement
are separate and cannot overwrite TRACK rate conversion.

Low-speed protections retain millihertz rates, a 5 mHz representable floor,
queue-start confirmation with one bounded retry, delayed step-period-aware
reversal braking, bounded zero-demand queue cancellation and explicit STOP.
Subminimum rates become zero, never an upward speed clamp. Numerical tolerance
at exactly 5 mHz prevents floating-point roundoff toggling zero/run. No-progress
checking accounts for slow step periods before rejecting missing response.

The independent 150 ms feedback watchdog stops motors if foreground processing
stalls. Invalid encoder feedback ends automatic operation; reacquired samples do
not restore lost references or restart motors. Manual input retains its separate
250 ms command watchdog. STOP brakes all participating motors and clears pending
motion before feedback processing. X/B abort remains latched until reset.

## Protocol 2 telemetry

`ORIENTATION_STATE` carries protocol=2, feedback=AS5600, available/fresh/age_ms,
heading (wrapped calibrated yaw), yaw_continuous, physical_pitch, north_set,
level_set, yaw_scale and pitch_scale. Uncalibrated axes are NaN, not fabricated
zeroes. ROLL is UNAVAILABLE. The scales are signed output degrees/encoder turn.

`ENCODER_STATE bus=A|B` carries raw, angle_deg (shaft angle only), valid,
magnet_good, status, age_ms and failure count. `CELESTIAL_STATE` distinguishes
targets/errors from measured orientation and reports encoder availability.
It also reports the positive-coordinate step signs and signed driver motor rates.
`CELESTIAL_RATE` distinguishes configured motor conversion, nominal feed-forward,
residual correction and commanded motor rate. Calibration messages report
configuration, explicit reference capture or reference loss. Keyframe snapshots
retain generated step positions and epochs, with optional encoder orientation.

All telemetry is bounded and nonblocking; stale cached reports are never new
control samples. The dashboard displays measured angles directly rather than
reconstructing them from target minus error.
