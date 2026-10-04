# Celestial GOTO and tracking

This extends the physically tested `a8c85a1` controller with fixed ICRS RA/Dec
targets. Python computes current local Alt/Az; the ESP32 owns both motors through
the existing autonomous axis controller. This implementation has offline tests
and a compile-only build. Real pointing, tracking drift and mechanical clearance
still require Jeff's supervised validation. No firmware has been uploaded.

## Start and enter a target

Use Python 3.11 or newer. This machine's PlatformIO Python is 3.11.7; its old
`.venv` points to an unavailable Python 3.6 installation. The dependencies are
already installed in `.pio/python_deps` for this implementation. For another
environment, install `requirements.txt` with that environment's Python.

From this workspace in PowerShell, enter your actual observing location:

```powershell
$env:PYTHONPATH = (Resolve-Path .pio\python_deps).Path
$observerLatitude = Read-Host 'Latitude in decimal degrees (north positive)'
$observerLongitude = Read-Host 'Longitude in decimal degrees (west negative)'
$observerElevation = Read-Host 'Elevation in metres (enter 0 if unknown)'
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" xbox_control.py --port COM9 --speed-scale 0.25 --latitude $observerLatitude --longitude $observerLongitude --elevation $observerElevation
```

This initial command reports **raw magnetic NORTH approximation** because it
does not invent magnetic declination. Before evaluating absolute pointing,
restart with the verified reference options described below. The manual
`--speed-scale` changes joystick speed; autonomous GOTO retains existing POSE
profiles. `--dry-run` checks parsing/UI without calculating a live target or
opening serial. Normal tracking needs both this host and the new firmware;
an older board rejects the new commands locally.

Center the sticks, press **F2**, and wait for READY. Enter:

```text
TRACK_RADEC 18:36:56.3 +38:47:01
```

Use **Tab to insert each space**, as Space remains STOP. Press Enter once.
After a valid request is accepted locally, the host automatically leaves the F2
editor and shows the managed celestial state. This is required so every refusal,
calculation failure, STOP, completion, or joystick takeover returns through the
ordinary manual-ready path. Invalid input stays in F2 for correction.
This example is Vega's rounded catalog position; use it only when visible and
within the gimbal's travel envelope. [SIMBAD Vega coordinates](https://simbad.cds.unistra.fr/simbad/sim-id?Ident=Vega).
Another fixed target may use decimal hours/degrees, for example
`TRACK_RADEC 5.5 -22.25`, or compact units such as `05h30m00s -22d15m00s`.
RA is normalized modulo 24 hours; Dec must be in [-90,90] degrees. These are
ICRS catalog coordinates, not apparent RA/Dec of date. This version has no name
resolver or solar-system ephemeris target interface.

To track an object without knowing its coordinates, use the normal joystick to
center the visible star, Moon, or other object in the camera. Release all three
assigned axes and leave them centered for 0.5 seconds, then click **TRACK HERE**.
The host captures the current reported BNO heading and physical pitch, maps them
to optical Alt/Az with the reference settings below, converts that instant to a
fixed ICRS RA/Dec target on the astronomy worker, and uses the same existing
CELESTIAL_GOTO/CELESTIAL_TRACK path. The panel identifies the target as TRACK HERE
and displays its captured RA/Dec. Clicking the Camera app, Alt-Tab, minimizing, or
showing/hiding diagnostics does not stop tracking. Deliberate stick movement,
STOP, abort, controller failure, or an applicable motion/sensor fault still does.

TRACK HERE requires configured observer latitude/longitude and a BNO report that
is recent, valid, and `north_usable=YES`. It refuses stale/missing alignment with
a visible reason while leaving MANUAL available. With no configured magnetic
declination the displayed reference explicitly remains a raw magnetic-north
approximation. Accurate absolute coordinates and longer retention require verified
heading direction, declination, camera boresight offsets, base attitude, host time,
and observer location; the software does not invent those alignments.
The captured target is a fixed ICRS/sidereal direction. The button cannot infer
that a centered object is the Moon, a planet, satellite, or comet, so it cannot
apply that object's independent ephemeris motion; such an object will slowly drift
relative to the captured sidereal direction.

The live celestial panel and session log show normalized RA/Dec, UTC, observer,
reference settings, computed Alt/Az, state, commanded BNO targets and pointing
errors. Actual heading/pitch remain in the existing BNO panel. F3 freezes only
the older diagnostic panel; celestial mode/status and controls stay live.
Malformed input leaves the editor intact. Missing dependencies, invalid observer
settings and astronomical calculation errors affect this operation only.

## Reference settings

Sky azimuth is clockwise from geographic north; altitude is above the horizon.
The board accepts its existing absolute **reported BNO heading and Euler PITCH**.
Generated motor steps and encoder angles remain separate quantities. During a
healthy celestial GOTO, firmware learns the signed relative relationship from
AS5600 A to yaw and AS5600 B to pitch without imposing a fixed gear ratio. No
physical motor sign or controller mapping is changed.

The configurable mapping is:

```text
true optical azimuth = heading_direction * BNO heading + declination + heading_offset
optical altitude    = BNO physical pitch + pitch_offset
```

| Host option | Meaning / default |
| --- | --- |
| `--magnetic-declination D` | Explicit true-minus-magnetic correction, east positive; unconfigured by default |
| `--heading-direction 1` | BNO heading increases with clockwise sky azimuth; default +1 |
| `--heading-direction -1` | BNO heading decreases with clockwise sky azimuth; changes coordinate mapping only |
| `--heading-offset H` | Remaining optical azimuth offset after direction and declination; default 0 degrees |
| `--pitch-offset P` | Optical altitude minus BNO Euler pitch; default 0 degrees |

Check heading handedness with a small supervised manual yaw movement against a
known landmark: the repository's Euler calculation alone does not establish the
camera's geographic handedness. Select +1 if reported heading increases when
the camera turns clockwise viewed from above, -1 otherwise. Check that rising
camera altitude increases reported physical pitch. The current mount's verified
pitch convention is retained; an inverted/remounted camera needs a separately
verified optical alignment before pointing. Constant offsets assume a level
alt-az base and aligned optical/BNO axes; they are not a general 3-D mount model.

To use a reliable supplied declination, append `--magnetic-declination D` to the
startup command, substituting its numeric value. Do not add a second correction
if your measured heading offset already includes true-north error: use explicit
`--magnetic-declination 0 --heading-offset H` for a directly measured true-north
offset. NORTH itself remains magnetic and still requires BNO accuracy >=2.
Diagnostics distinguish an unconfigured magnetic approximation from an explicit
true-north correction and display all direction/offset settings.

## Motion and failure behavior

The host performs STOP and waits for READY while a bounded background worker
loads Astropy and prepares the current target. A correlated `CELESTIAL_GOTO`
starts a single persistent autonomous operation. Both axes reuse POSE's finite
direction checks, slew profiles, braking, post-stop observation, precision
corrections and progress guards. Initial admission also requires fresh BNO
accuracy >=2. Carriage holds its current generated-step position.

GOTO has displacement/learned-response deadlines using both axis profiles and
the existing precision tail. Once both axes are within 0.4 degrees, stopped,
and observed for the existing one-second/30-sample settling window, firmware
announces `CELESTIAL_TRACK`. There is no intervening READY/manual transition.

The host recomputes current UTC Alt/Az approximately once per second during both
GOTO and TRACK. Firmware updates the two targets in place without restarting a
slew. TRACK uses the existing precision controller: HOLD until error exceeds
0.4 degrees, then bounded low-rate finite corrections toward the 0.1-degree
approach deadband. Both axes may correct together. This avoids chasing every
tiny sky change but produces discrete motor corrections; actual camera jitter,
backlash and pointing accuracy must be measured.

Yaw updates use shortest angular differences and an unwrapped target. Crossing
359 to 0 degrees therefore makes a small update. The fixed +/-185-degree yaw
travel envelope and strict +/-75-degree BNO pitch guard remain enforced.
Below-horizon targets, unreachable endpoints, or updates exceeding 1 degree/s
on either axis stop/refuse with a reason; this includes unsuitable zenith paths.
The software does not discover obstacles, cable routing or rail endpoints.

Only matching, increasing sequence updates renew the firmware's 3-second target
lease. STATUS and malformed/stale packets do not. Host calculation delay beyond
2.5 seconds since the last target sends STOP; communication/acknowledgment
timeouts retain the existing host safety path. Old worker results cannot restart
a canceled operation. Both timeouts use monotonic time; astronomy uses system
UTC. Large clock changes can trigger the target-rate guard and cancel tracking.

After admission, missing, stale, reset, malformed, low-accuracy or failed BNO
feedback is a degraded-state event rather than a celestial failure. The session
ID and continually updated target remain active. Firmware permanently latches
that session to the last trustworthy absolute BNO frame plus unwrapped relative
movement measured by AS5600 A (yaw) and AS5600 B (pitch). It disarms only the
celestial session's BNO-stale watchdog; the target lease, encoder progress,
wrong-direction, pitch and yaw travel guards, motor-command failures and every
other mechanical safety check remain active.

The signed encoder-to-cradle scale is learned from simultaneous healthy BNO and
AS5600 movement, so raw shaft degrees are never assumed to be output degrees and
generated step counts never replace encoder feedback. If either required encoder
or its learned scale is unavailable, celestial motion brakes to a preserving
HOLD. The target and session remain alive indefinitely and resume only from fresh
encoder feedback; a BNO outage cannot expire a GOTO deadline while held. This is
also the safe behavior for a first session that loses BNO before an axis has moved
enough to establish its encoder scale.

When valid accuracy-2/3 BNO orientation returns, firmware reports its yaw/pitch
discrepancy from the encoder-propagated frame. It does not re-anchor, blend, issue
a catch-up correction, terminate or restart the session. `CELESTIAL_STATE` reports
the selected feedback, BNO/encoder availability and the last discrepancy.
LEVEL, NORTH and POSE retain their existing BNO requirements and watchdog
behavior. STOP retains its existing 3-second forced-stop fallback.

Move a stick deliberately to cancel either phase: **STOP -> stopped READY ->
zero-JOG acknowledgment -> current live manual input**. No recentering is required
for that takeover. Space/F12 or uppercase `STOP` in F2 cancels ordinarily;
F2 exits with STOP and then requires centered sticks. B/button 1 or keyboard X
remains the separate latched abort requiring reset. Focus/input loss and normal
exit use STOP. The host never transmits literal RA/Dec to firmware and blocks
internal `CELESTIAL_*` commands in the editor.

## Astronomy and accuracy limits

Astropy 7.2.2 performs ICRS `SkyCoord` -> `AltAz` using `EarthLocation` and UTC
`Time`. Pressure is zero, so no weather-dependent refraction is assumed. There
is no handwritten sidereal time/precession/nutation/Earth-orientation transform.
See [Astropy's observing example](https://docs.astropy.org/en/stable/coordinates/example_gallery_plot_obs_planning.html).

Automatic IERS downloads are disabled. Bundled Earth-orientation/leap-second
data support offline operation; unavailable date coverage is reported as degraded
accuracy. Updating `astropy-iers-data` can be done deliberately while online.
See [Astropy IERS configuration](https://docs.astropy.org/en/stable/utils/iers.html).

The goal is several-minute target retention in an alt-az mount. No polar
alignment is required. Field rotation is **not corrected**. BNO magnetic bias,
quality, optical alignment, base tilt, refraction near the horizon, backlash,
motor resolution and the 0.4-degree correction threshold limit results. Fixed
catalog coordinates have no supplied proper motion, distance or radial velocity.
This is not a guarantee of long-exposure or narrow-field astronomical accuracy.

## First supervised physical validation

After Jeff separately installs the new firmware:

1. Put the gimbal on a level base with reasonable cable/travel clearance, camera
   secured, and pitch comfortably inside +/-75 degrees.
2. Start the host normally; confirm manual yaw/pitch/carriage, release braking
   and Space/F12 STOP at the reduced manual scale.
3. Confirm A/LEVEL and Y/NORTH still behave as on `a8c85a1`; confirm accuracy >=2.
4. Check location, elevation and the displayed UTC against the host clock. Verify
   optical/BNO heading direction and offsets, then restart with verified settings.
5. Identify a visible bright target, preferably well above the horizon and away
   from the zenith. Manually center it in the camera, release the sticks for 0.5
   seconds, then click **TRACK HERE**. No RA/Dec entry is required.
6. Watch the captured RA/Dec and the transition through GOTO to TRACK. Separately
   test the existing RA/Dec fields or F2 `TRACK_RADEC` path with a known target.
7. Confirm automatic transition from CELESTIAL_GOTO to CELESTIAL_TRACK. Center the
   phone/camera optical alignment on the target; using the sticks cancels tracking,
   so re-enter the target if mechanical recentering was needed.
8. Track for at least five minutes. Save the session log and record drift in
   degrees/pixels, any stepwise jitter, elapsed time and camera field of view.
9. Move a stick during TRACK and verify braking, READY/zero-JOG and immediate
   normal live manual response. Repeat takeover during a GOTO.
10. Re-enter tracking, use ordinary STOP, leave F2 and confirm centered manual
    control. Re-test LEVEL, NORTH and saved keyframe actions as appropriate.
11. Separately test B's latched abort and reset recovery. In a controlled setup,
    test BNO loss and recovery while both AS5600s remain readable. Confirm the
    session stays in TRACK, `feedback=AS5600` appears, target updates continue,
    and recovery reports discrepancy without a pointing jump. Do not disturb
    shared I2C wiring during motion: BNO and pitch AS5600 share Bus B.

Software simulation cannot establish real optical handedness, magnetic bias,
torque margin, travel clearance or tracking drift. Record those observations
before calling this celestial feature physically validated.
