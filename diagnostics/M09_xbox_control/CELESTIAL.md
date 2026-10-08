# Encoder calibration and celestial pointing

## Configure encoder ratios in the launcher

`Start_Gimbal.bat` now contains your supplied **output-axis revolutions per
encoder revolution**:

```bat
set YAW_ENCODER_RATIO=1
set PITCH_ENCODER_RATIO=0.06666667
```

Yaw is direct 1:1; pitch is approximately 1:15. The batch file passes these as
`--yaw-encoder-ratio` and `--pitch-encoder-ratio`. The host multiplies each by 360,
so firmware receives `ENCODER_CONFIG 360 24.0000012` (approximately 24 degrees
per pitch encoder revolution). Location settings remain separate.

The host sends STOP, waits for stopped READY and protocol-2 telemetry, sends the
configuration once, and waits for confirmation before allowing manual/automatic
commands. It repeats setup after a firmware reboot. A failed setup exits with
STOP. Dry run never sends anything. Configuration clears both zero references;
you must still use SET NORTH and SET LEVEL after launch/reboot. No motor movement
or reference capture is part of automatic configuration.

The supplied positive ratios assume raw counts increase with clockwise yaw and
upward pitch. Verify these directions; use a negative ratio for an axis whose raw
counts increase in the opposite direction. If you omit both CLI options, geometry
can still be configured manually with F2 as described below.

## Firmware geometry units

`ENCODER_CONFIG yaw_scale pitch_scale` specifies **signed camera-axis degrees
per one encoder revolution** for each axis. For a verified direct output-axis
mount the magnitude is 360. For an encoder shaft that turns R times per output
axis revolution, the magnitude is 360/R. Determine R independently from a known
physical output angle and unwrapped encoder travel; motor counts are not an angle
measurement. The pitch encoder is documented as before reduction. Do not copy
the synthetic ratios in tests or use the tracking motor-rate constants here.

Positive scale means increasing raw angle corresponds to clockwise yaw / upward
pitch. Negative scale reverses that conversion. Observe raw readings while
moving a small known physical angle to establish the sign on each axis. Firmware
accepts finite nonzero signed scales with magnitude <=360 degrees/revolution.
It never writes AS5600 OTP, zero or configuration registers. Unknown geometry
starts at zero and blocks automatic angular motion.

Press F2, wait for stopped READY, enter `ENCODER_CONFIG <yaw_scale> <pitch_scale>`
with your measured numbers, then press Enter. Exit F2 for ordinary controls.
Configuration is RAM-only. The configured launcher resends it after every
firmware reset; without those launch options, resend it manually.

## Establish references

Aim the camera's optical axis at **true north** using an independent known
alignment, stop, and press **SET NORTH**. Aim horizontally using an independent
level/reference, stop, and press **SET LEVEL**. These can be performed separately.
Buttons first brake manual motion, wait for READY, and send `SET_NORTH` or
`SET_LEVEL`. The same commands are available in F2. Neither command moves motors.
LEVEL and NORTH are separate movement commands returning to those references.
The dashboard shows NORTH/LEVEL SET or REQUIRED. Repeat either calibration any
time the mount/reference has changed, with all motors stopped.

Default `--north-reference true` requires a true-north physical alignment. A
magnetic compass reading is **not** true north. If intentionally aligning SET
NORTH to magnetic north, launch with `--north-reference magnetic` and a known
local/date-appropriate `--magnetic-declination D` (true minus magnetic, east
positive). Magnetic mode without declination is rejected. True mode rejects an
additional declination to avoid double correction. The north declaration itself
does not measure which method you used; operator alignment must match host setup.

`--heading-offset` and `--pitch-offset` remain optional optical boresight offsets
in degrees. When calibrating the optical axis itself, leave both zero. Encoder
direction is configured in firmware; there is no additional host direction flip.

Offsets and revolution counts are deliberately not restored from flash. Reboot,
encoder-geometry changes, a failed/invalid read, bad magnet or >=150 ms gap
invalidates the affected reference. The sensor continues polling and can recover,
but an automatic operation stops and never resumes itself. Stop, restore valid
readings, repeat the affected SET calibration, then explicitly restart pointing.
This avoids inventing full revolutions lost while a shaft encoder was unavailable.

Unwrapping assumes less than half an encoder revolution between successful
samples (normally 20 ms). An exact half-turn is rejected as ambiguous. Faster
external motion can alias and is not detectable with a single-turn encoder;
keep within that sampling bound. Axis accuracy also depends on mechanical ratio,
encoder linearity, backlash, mounting and north/level alignment.

## Target calculations and use

Supply actual latitude (north positive), longitude (east positive), and elevation
in metres. Keep the PC's date/time accurate; calculations use timezone-aware UTC.
Manual RA/Dec uses ICRS coordinates, with RA in hours or hh:mm:ss and declination
in degrees or signed dd:mm:ss. Astropy uses bundled Earth-orientation data;
warnings about old data are surfaced. No network download occurs while tracking.
Named planets/stars use the offline catalog described in CELESTIAL_TARGETS.md.

Select a target and press GOTO / TRACK, or enter RA/Dec and press TRACK RA/DEC.
TRACK HERE converts the current calibrated encoder pointing to a fixed celestial
coordinate and enters the same tracker. It requires a recent valid orientation
report. The host and firmware both gate admission on valid calibration. Both
host and firmware must use protocol 2; old orientation telemetry cannot enable
celestial pointing in the new host.

For true azimuth A and altitude H, firmware targets are:
`yaw = wrap360(A - declination - heading_offset)` and
`pitch = H - pitch_offset`. Declination is zero in true-north mode. Below-horizon
targets and pitch outside the existing travel guard are refused. GOTO settles
using measured encoder angles, then tracks until STOP, takeover, loss of feedback,
or loss of target updates. Losing dashboard focus does not stop tracking.
