# Milestone 8: coordinated three-axis GO TO POSE

M08 extends the working M07 north/level controller with coordinated yaw, pitch,
and carriage commands. Physical verification of this revision is pending.
M07, M05, M06, the shared hardware header, and earlier diagnostics are unchanged.
No firmware upload, commit, or push is part of this development work.

This sibling `diagnostics/M08_go_to_pose` project follows the established milestone
layout. M07 was untracked at inspection, so changing it in place would not preserve
its baseline in Git. Only the diagnostic source/tests/configuration are carried
forward; build caches and unrelated projects are not part of this milestone.

## Actual position feedback and the off-axis yaw encoder

| Axis | Position used by M08 | Reference and limits of knowledge |
| --- | --- | --- |
| Yaw | BNO085 absolute rotation-vector heading and continuous wrap tracking | Same magnetic-heading frame as M07; 0 is its north result, not corrected geographic north. Indoor interference still affects this reading. |
| Pitch | BNO085 **roll** Euler component | Physically verified cradle pitch feedback; zero-roll level target. The shaft AS5600 is not direct cradle-angle feedback. |
| Carriage | FastAccelStepper generated step count | Zero is the power-up count at wherever the carriage physically starts. No homing, measured rail position, calibrated mm conversion, or missed-step detection. |

**The carriage does not have genuine absolute physical positioning.** Its third
POSE field is an integer number of **STEPS**, never millimeters. Reset creates a
new software origin at the current physical location. Manual carriage movement or
missed steps invalidates its correspondence to physical position. A repeatable
physical datum and calibrated travel per pulse are still missing for absolute mm.

Inspection found older yaw AS5600 support in the v05/v06/v07 Arduino sketches:
raw encoder angles are converted with 360/4096, a startup angle is saved, and
wrapped raw-angle error drives return-to-zero. Those sketches use an older
hardware layout and contain no identified mapping, coupling ratio, calibration
curve, or physical datum for the newly installed off-axis yaw mechanism.
M03 reads both AS5600s for bus communication/status; M04 treats Bus A as
communication-only and the pitch encoder as a motor-shaft measurement.

No post-M07 off-axis yaw implementation was found in the inspected project tree.
M08 therefore keeps the BNO as yaw authority and adds no AS5600 reads or invented
encoder calibration. The older raw-angle code does not supersede M07.

The unplugged-motor hand test showed BNO roll following physical cradle pitch
from approximately -32 to +38 degrees while BNO pitch stayed near -4 to -2 degrees.
All M08 physical pitch feedback, baselines, limits, targets/errors, velocity,
settling, learned pulses/degree and `pitch` telemetry therefore use **BNO roll**.
Command names remain `POSE`/`MOVE` with physical pitch as the second field; the
raw Euler conversion and its finite checks remain unchanged. Heading/yaw keeps
its existing component and behavior. This mapping adds no tare or sign change.

## Preserved hardware and motion

| Axis | DIR / STEP GPIO | Slew ceiling | Slew acceleration |
| --- | --- | ---: | ---: |
| Yaw | 32 / 33 | 1000 pulses/s | 1000 pulses/s squared |
| Pitch | 26 / 12 | 600 pulses/s | 600 pulses/s squared |
| Carriage | 21 / 22 | 1000 pulses/s | 1000 pulses/s squared |

Yaw/pitch settings are unchanged from current M07. Carriage settings come from
the earlier verified three-axis diagnostic. All axes retain the 200 us DIR setup
delay. Current yaw and pitch positive-step signs are both **-1**; M08 preserves
the post-M07 pitch sign and corrects its own startup banner to display that value.
M07's stale +1 documentation/banner is left untouched.

BNO remains `Wire1`, Bus B SDA4/SCL5, address 0x4A, 100 kHz, 50 ms per-transaction
I2C timeout, and 10 ms requested `SH2_ROTATION_VECTOR` reports. Bus A SDA18/SCL19
remains defined in the shared header and unused by this controller. The TMC2209
hardware/driver configuration and dependency versions are unchanged.

The startup sequence still performs M07's five-second warning, stable calibrated
baseline, north finding, and pitch leveling, with carriage stationary. Its
PROBE/SLEW/BRAKING/PRECISION/HOLD logic and native speed/acceleration behavior remain.
Startup PASS still requires measured movement of both rotational axes and overlap.
Start modestly off north and level so that this validation and timing observation
can succeed; an already-on-target startup can fail the preserved movement-evidence
check even though the orientation is correct.

## Pitch readiness when north calibration is unavailable

`BNO_MIN_ACCURACY` remains **2** for magnetic north. The normal calibrated
north/level startup is preserved. If it cannot obtain that baseline by the
existing 20-second startup deadline, the next completed baseline window can
instead authorize pitch and report **PITCH-ONLY READY**, with all motors stationary.
An unstable, stale, or invalid pitch baseline still aborts.

Pitch readiness requires a full one-second window with at least 30 accepted
finite orientation samples, both half-windows populated, no gap over 100 ms,
current feedback younger than 150 ms, pitch SD <=0.15 degrees, range <=0.5 degrees,
and half-window mean difference <=0.2 degrees. Accuracy 0 or 1 and heading
instability do not invalidate this pitch baseline. No new sensor report is used.

In this state, `MOVE 0 <pitch_delta_deg> 0` moves only pitch. For example, from a
reported pitch of +8 degrees, `MOVE 0 -8 0` requests level. Alternatively,
`POSE <current_reported_heading_deg> 0 0` requests absolute pitch zero. The yaw
field must request no motion under the existing 0.4-degree tolerance; yaw stays
disabled even if its measured heading drifts during the move. Commands requiring
yaw or carriage motion are rejected. Pitch uses the existing native controller
and limits without inventing a timing response or claiming synchronization.

The independent 150 ms stale watchdog is armed before every pitch move and
refreshed only by accepted BNO vectors. Reset/invalid-data handling, pitch travel,
wrong-direction/progress guards, burst/braking/overall deadlines and X/x remain.
Without calibrated north, the existing +/-185-degree yaw travel check uses the
continuous heading saved at pitch readiness as a conservative startup-relative
reference; it is not labeled magnetic north. Idle stale data or a reset still
latches an abort. Pitch completion requires stopped, fresh feedback for the
existing one-second / 30-sample settling window, regardless of accuracy.

Accuracy recovery alone never starts yaw or creates a north reference. Reset
deliberately to retry calibrated startup. During an authorized yaw operation,
accuracy below 2 now stops and latches the entire operation immediately rather
than allowing yaw to continue through the old accuracy grace. Pitch-only moves
also tolerate low accuracy after a normal calibrated startup. Carriage command
admission still requires calibrated north and accuracy >=2; carriage operations
without yaw retain the existing 1000 ms low-accuracy grace and accurate settling.

## Commands at 115200 baud

Wait for `READY`, then send one newline-terminated command:

```text
POSE <yaw_heading_deg> <pitch_deg> <carriage_position_steps>
MOVE <delta_yaw_deg> <delta_pitch_deg> <delta_carriage_steps>
```

Examples after the startup north/level PASS:

```text
MOVE 2 -2 100
POSE 0 0 0
```

The first requests +2 degrees of BNO heading, -2 degrees of cradle pitch, and
+100 carriage pulses from the current position. The second returns to BNO
north/level and the carriage's startup step count. It does not home the carriage.
`POSE 45 20 200` means **200 startup-relative pulses**, not 200 mm, and is not the
recommended first physical test.

POSE yaw is normalized to 0..360 and uses the existing shortest-path math.
MOVE yaw deltas must be in `[-180,180)` to avoid silently turning a requested
full revolution into no motion. Pitch targets must stay strictly inside +/-75
degrees. The chosen yaw endpoint must stay inside the existing +/-185-degree
continuous north envelope; some wrap-crossing targets are rejected rather than
changing that guard or taking a longer route.

The new carriage software envelope is **-500 through +500 generated pulses**
from startup zero. This conservative test envelope is not a sensed endstop or a
claim of mechanical clearance. Relative commands are checked against the same
resulting-position envelope. Inspect clearance before every reset.

Inputs with missing/extra fields, nonfinite numbers, noninteger carriage values,
overflow, excessive line length, invalid bounds, unavailable timing response,
or unusable feedback are rejected without motor commands. Commands sent while
busy are rejected, not queued. An already-satisfied pose reports
`POSE ALREADY AT TARGET` and issues no motion or new result summary.

**X/x remains an immediate abort character** while serial input is serviced,
including during command entry. Abort/reset faults latch until board reset; no
POSE command clears them. Ctrl+C only closes the serial monitor.

## How coordination works

A completed, stopped north/level run supplies an observed angular timing response:
net generated pulses divided by net BNO angle change. At least 0.3 degrees and
8 pulses with consistent direction are required. Successful later poses can
update that estimate. It is a timing estimate, not a gearbox calibration or a
replacement for BNO feedback. The existing 12/16 pulses-per-error-degree constants
remain correction gains and are not treated as mechanical conversion factors.
A moving angular axis without a usable estimate is explicitly rejected; a
carriage-only command can still run when its requested angles are already satisfied.

For each requested move, `pose_math.h` estimates the effective duration at the
native limits. It accounts for acceleration, M07's measured-rate braking boundary,
finite precision corrections, and stopped-observation delays. Pure pulse moves
use triangular/trapezoidal acceleration time. The longest estimated duration
sets the common target time. Shorter axes receive lower speed caps; acceleration
is unchanged. All required axes receive nonblocking FastAccelStepper commands in
the same control pass, and the loop continues BNO acquisition and serial handling.

Yaw/pitch still finish from measured BNO error, not estimated step endpoints.
A stationary axis starts in HOLD and does not delay the other axes. Carriage
finishes from its generated-count endpoint. Completion requires carriage stopped
at that count and both angular targets stopped within +/-0.4 degrees for the
existing full one-second fresh, accurate settling window.

Speed selection respects the existing burst and progress deadlines. When a very
slow speed would violate them, a faster safe cap is used and `timing_limited=YES`
is reported. That flag also covers material integer-Hz/timing-model mismatch.
It means arrival matching is limited for that request, not that any guard was
relaxed. Requests estimated to exceed the 90-second operation allowance are rejected.

Synchronization is approximate. BNO latency/noise, backlash, varying load,
quantized pulses/speed, and the last precision correction affect arrival time.
Pitch can set the overall duration because of its 600 Hz slew limit, gearing,
and slow precision stage. M08 slows the other axes to match; it does not increase
pitch speed or redesign leveling. This is not a trajectory/interpolation planner.

## Safety and READY behavior

Preserved safeguards include X/x, BNO reset detection, active 150 ms stale-data
protection, yaw/pitch travel
guards, wrong-direction/runaway checks, 2-second slew and 15-second precision
progress checks, 4-second correction and 3-second braking deadlines, and the
90-second active operation deadline. Yaw and carriage command admission requires
fresh BNO accuracy >=2. Pitch-only admission uses the separate orientation
readiness described above; accuracy alone never authorizes a north reference.

The existing independent 10 ms stale timer now stops **all three** motors,
including during a blocked foreground BNO call. Software stopping includes timer
scheduling and FastAccelStepper queue-drain latency; it does not remove driver power.
Carriage cannot detect a mechanical stall from commanded counts alone.

After success, motors remain idle and only an explicit command starts another
move. BNO acquisition continues silently to retain the reference. An idle BNO
outage reaching 150 ms or a BNO reset latches a fault requiring reset; an unseen
manual yaw turn must not silently alter the continuous travel reference. Low
accuracy while idle prevents yaw/carriage command acceptance but permits pitch-only
commands with an intact orientation baseline; it does not itself move anything.
Summaries are buffered so serial backpressure cannot starve that acquisition.

## Telemetry to watch

`POSE ACCEPTED` reports normalized target angles, carriage target **steps**, common
`planned_s`, per-axis `caps_hz`, predicted durations in yaw/pitch/carriage order,
and `timing_limited`. These are estimates, not promises of exact arrival.

The retained `STATE` lines show yaw/pitch errors, mode transitions, pulse and BNO
angular speeds, braking distances, accuracy, and freshness. `POSE_STATE` adds
carriage current/target counts, motor state, speed caps, and `all_three_seen`.
The final MILESTONE 8 summary reports all three targets, BNO errors, carriage
counts, response estimates, two-/three-axis overlap evidence, safeguards, and
`FINAL RESULT: PASS/FAIL`. An axis-only pose may correctly report no three-axis
overlap. Successful completion ends with `READY`.

## Build and conservative first physical test

From `diagnostics/M08_go_to_pose`:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
```

1. Close other serial clients. Keep the existing wiring, driver settings, and
   power configuration. With motion safely inactive, place the carriage away
   from both rail ends, clear cables/travel, and keep motor power removal accessible.
2. Start yaw roughly 15-20 degrees off its BNO north result and pitch about
   5-8 degrees off level, with clear paths. M08 will automatically run the
   preserved north/level routine before accepting commands; carriage stays still.
3. Upload only when ready for that automatic motion, then monitor/log:

   ```powershell
   & "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev -t upload --upload-port COM9
   & "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM9 --baud 115200 --filter log2file
   ```

   Upload, opening the monitor, or reset may restart the startup sequence.
4. Wait for startup `FINAL RESULT: PASS` and `READY`. Confirm nonzero yaw/pitch
   timing estimates and stationary carriage. If startup fails, preserve the
   reason and correct the setup before a deliberate reset.
5. Send **`MOVE 2 -2 100`** with Enter. All three axes should begin together.
   Watch both BNO errors reduce, carriage counts approach 100, and the shorter
   axes run more slowly. Expect approximately matched arrival followed by the
   shared stopped settling window and another PASS/READY.
6. If clear and the first move behaves correctly, send **`POSE 0 0 0`** with
   Enter. Check return to north/level and the original generated carriage count.
   Save the complete log, target/cap/duration lines, transition timing, final
   errors, carriage counts, `timing_limited`, and visible overlap/stillness.
7. For unexpected motion, send X/x or remove motor power. A deliberate abort
   check should stop all three and refuse later poses until reset. A reset changes
   the carriage origin: safely reposition before rerunning if the old physical
   origin matters. Do not enlarge guards to bypass failures.

No homing system, off-axis encoder calibration/control, keyframes, duration-based
pose interpolation, external controller integration, tracking, camera control,
or Milestone 9 features are included.
