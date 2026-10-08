# Dashboard target picker

Click **Choose target**, select a tile, then **GOTO / TRACK**. Selection alone
does not request motion. Solar-system targets and bright stars are grouped
separately. STOP and ABORT remain accessible above the picker. Closing the
picker restores the existing dashboard, including manual RA/Dec and TRACK HERE.

The supplied targets are Sun, Moon, Mercury, Venus, Mars, Jupiter, Saturn,
Uranus, Neptune, Pluto, Polaris, Vega, Capella, Arcturus and Sirius. Add another
catalog star to `STAR_TARGETS` in `celestial_targets.py`; no motion changes are
needed. Icons are drawn locally with pygame; there are no downloaded images.

## Astronomy and horizon

`ephem==4.2.1` (PyEphem) is added to `requirements.txt`. Its bundled ephemerides
include Pluto, and its bundled bright-star catalog supplies positions and proper
motions. See the [PyEphem reference](https://rhodesmill.org/pyephem/quick.html)
and [coordinate definitions](https://rhodesmill.org/pyephem/radec.html).
No network, external catalog service or ephemeris download is used at runtime.
The existing Astropy converter for manual ICRS RA/Dec is unchanged.

Named-target calculations use the same validated `Observer` built from the
launcher's `--latitude`, `--longitude` and `--elevation`, and the astronomy
worker's existing timezone-aware host UTC clock. PyEphem computes each object's
current topocentric apparent azimuth and altitude, including changing solar-system
positions and lunar parallax. Apparent RA/Dec is not misinterpreted as fixed ICRS.
Pressure is zero, matching the existing unrefracted horizon convention.

Preview tiles show altitude above/below zero degrees and refresh every 15 seconds
while the picker is open. Positions older than 30 seconds are unavailable for a
new selection request. Below-horizon targets are blocked; the current application
has no horizon-override confirmation mechanism. The existing mount conversion
also checks the newly calculated target before sending GOTO.

## Existing tracking path

The picker passes a `NamedTarget` to the same `AutoSession.request_celestial()`
used by manual RA/Dec. Existing READY/centered-stick admission, preflight STOP,
astronomy preparation and mount-reference conversion remain in place. The host
sends one `CELESTIAL_GOTO`; firmware's `CELESTIAL_TRACK` response drives the same
GOTO-to-TRACK transition. Each normal worker update recalculates the selected
object at current UTC and supplies the existing `CELESTIAL_UPDATE` stream.
Firmware continues deriving continuous tracking rates from that stream.

No new motor command, rate controller, BNO prerequisite, timeout, or automatic
STOP policy is added. The separate preview worker only produces display data:
preview errors, opening the picker and selecting another tile do not cancel or
retarget active tracking. Existing active-session failure handling is unchanged.
Target name, azimuth, altitude and UTC remain in the existing celestial status/log.

Install the updated requirements using the launcher's Python environment. The
current launcher reads project-local packages from `.pio/python_deps`:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m pip install --target .pio/python_deps -r requirements.txt
```

This feature requires no firmware changes or upload.
