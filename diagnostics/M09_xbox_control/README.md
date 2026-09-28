# M09 Xbox manual control

M09 keeps the PC/Python -> serial JOG -> ESP32/FastAccelStepper architecture.
The approved simplification restores both AS5600 readouts and makes manual
velocity independent of orientation/reference availability. M08 is unchanged.

## Hardware and motion

| Axis | DIR / STEP GPIO | Maximum pulses/s | Acceleration pulses/s² |
| --- | --- | ---: | ---: |
| Yaw | 32 / 33 | 2000 | 2000 |
| Pitch | 26 / 12 | 2400 | 2400 |
| Carriage | 21 / 22 | 2000 | 2000 |

Physical pitch feedback for automatic positioning is now **BNO Euler PITCH**,
corrected from the post-relocation powered-log evidence. Existing
positive-step signs remain -1 for yaw and pitch. Your pre-existing M09 precision
settings (`ACCELERATION=250`, `PITCH_PULSES_PER_ERROR_DEG=24.0`) are preserved;
they are separate from the manual acceleration ceilings above.

| Sensor | Bus | Address | Meaning |
| --- | --- | --- | --- |
| AS5600 A | Wire, SDA18/SCL19 | 0x36 | Wrapped sensor/magnet angle; axis mapping is not invented |
| AS5600 B | Wire1, SDA4/SCL5 | 0x36 | Historically pitch motor shaft, before reduction |
| BNO085 | Wire1, SDA4/SCL5 | 0x4A | Moving-cradle orientation; qualified magnetic heading |

Both buses retain 100 kHz and the configured 50 ms transaction timeout (the
underlying ESP-IDF error path can still block longer). AS5600 STATUS and
RAW_ANGLE are read together, one device per scheduled call, alternating at a
10 ms minimum global cadence (about 20 ms/device when calls are quick). No OTP,
zero, scale or calibration registers are written. Read failures are local data
availability changes. No encoder readings command motors or enforce limits.

## Startup and sensor availability

Startup initializes motor control and the existing independent command watchdog,
then starts a dedicated sensor worker. That worker initializes the two buses and
attempts BNO setup once. Manual READY does not wait
for sensor success, magnetic accuracy, a stable reference or an automatic move.
The old 500 ms application delay, 5 s hold and 20 s fallback gate are gone.

The proven BNO reset/report path remains, including the sensor's 300 ms boot delay.
`tools/patch_sh2_timeout.py` reproducibly adds a 1-second product-ID response
operation deadline to the pinned SH2 dependency before compilation. This bounds
that previously unlimited wait; it is not a 1-second total startup guarantee.

While stopped/idle, the original stable one-second windows can qualify pitch and
north references. North still requires rotation-vector accuracy >=2 and the
existing stability checks; accuracy 0/1 is never called calibrated. A sensor gap
or reset invalidates orientation references; healthy idle samples can qualify
new ones without a board reset. Report-enable recovery remains bounded. A BNO
that failed initial setup remains unavailable until reset; no new repeated
initialization loop was added.

The sensor worker exclusively owns both I2C buses, BNO transport/recovery/product
queries, and both AS5600 readers. Foreground command processing never performs or
waits for sensor I/O. Timestamped samples and reset events cross bounded queues;
sensor health and motor context cross nonblocking snapshot mailboxes. The existing
250 ms watchdog still requires valid processed commands, regardless of buffered
UART bytes. See [sensor-worker implementation and validation](audit/SENSOR_WORKER_ISOLATION_2026-09-27.md)
for ownership, handoff details, and the one-second-stall regression results.

`BNO_STATE` and `ENCODER_STATE bus=A/B` report availability, sample age and quality
every 500 ms, including while idle/manual. Python also shows report receipt age.
An encoder's `valid=YES` means a complete readable sample; inspect `magnet_good`
and status separately. These are not calibrated final-axis positions. Last raw
values can remain visible after a failed read and must be interpreted with age
and validity. The pitch encoder and BNO share Bus B; an electrical bus failure
can affect both sensors without making raw manual velocity depend on them.

## Xbox behavior and protocol

- Right stick horizontal = yaw (raw axis 2).
- Left stick vertical = pitch (raw axis 1); up requests positive physical pitch.
- Left stick horizontal = carriage (raw axis 0). Use `--invert-carriage` to reverse it.
- The existing 15% deadband and quadratic velocity curve are unchanged.
- The user's current default scale is **1.0**: full stick requests yaw 2000,
  pitch 2400 and carriage 2000 pulses/s. `--speed-scale 0.25` selects one quarter
  of those rates. This diagnostics pass preserves the increased settings.
- Center both sticks for 0.5 s to enable manual control automatically. No motion is sent before
  the firmware acknowledges the zero-JOG handshake. Telemetry age alone does not
  block that handshake.
- Release an individual stick axis to brake that motor. Reversal retains motor
  acceleration and the existing 100 ms stopped observation.
- A/Space/F12 sends STOP; center sticks again for 0.5 s before manual resumes. keyboard X/controller B deliberately
  sends the existing latched abort; resetting is required after explicit abort.

| Command | Behavior |
| --- | --- |
| `STATUS` | M09 READY, MANUAL, BUSY or ABORTED |
| `JOG 0 0 0` | Enter manual mode from stopped READY |
| `JOG yaw pitch carriage` | Integer fields -1000..1000; host sends every 20 ms |
| `JOG yaw pitch` | Existing two-field compatibility; carriage request is zero |
| `STOP` | Brake manual axes, or cancel POSE/finite carriage and brake all moving axes; return READY when stopped |
| `X` / `x` | Immediate existing latched abort, including within a partial line |
| `MOVE 0 0 steps` | Finite relative carriage motion, independent of BNO/north |
| `POSE yaw pitch steps` / other `MOVE` | BNO-based angular positioning requirements below |

Manual control works with missing, stale, invalid or low-accuracy BNO information
and failed encoders. Manual BNO direction/progress, angular position/margin and
90 s operation guards are removed. Carriage uses continuous run/update/stop calls;
there is **no replacement for the removed +/-500 startup-relative window**.
All manual travel is supervised by the operator. Generated carriage steps are
unhomed counts, not a measured rail position or millimetres.

The existing independent 250 ms valid-command timeout remains. It is armed before
nonzero continuous requests and stays active until zero/STOP has actually reached
motor braking. Stationary centered idle does not need a continuing stream. A lost
stream stops motors, clears requests and permits a new centered arm without reset.
Malformed packets and STATUS do not refresh the command deadline.

Per-axis API failures, unexpected stops and failed braking stop/report the affected
axis; a later valid JOG may retry after stopping. A cleared request is not a new
axis fault latch. Brief telemetry silence only warns while live commands continue.
Focus/input loss, host exceptions and failed session transitions use ordinary STOP;
only deliberate keyboard X/controller B sends X. A host command gap reaching the same 250 ms lease
requires rearming before sending nonzero input again.

## Manual positioning and two-keyframe actions

Manual is the default workflow. Center both sticks for 0.5 seconds; the host
performs the existing zero-JOG/acknowledgment handshake automatically. There is
no AUTO toggle and Y is unassigned. The stick mapping, deadband, quadratic curve,
watchdog, F2 raw editor and F3 display freeze remain in use.

| Control | Action |
| --- | --- |
| Left stick vertical / horizontal | Manual pitch / carriage |
| Right stick horizontal | Manual yaw |
| D-pad up/down | `MOVE 0 +/-increment 0` (physical pitch degrees) |
| D-pad right/left | `MOVE +/-increment 0 0` (reported BNO heading degrees) |
| Right-stick click (9) | Select 1 or 2 degree increments; initial 1 |
| LB (4) / RB (5) | Stop, then capture A / B from a fresh firmware snapshot |
| Controller X (2) | Cycle duration 10 -> 20 -> 5 seconds; initial 10 |
| Left-stick click (8) | Travel to A at normal generated-step travel speed |
| Center/Home (10) | Return to A if necessary, then run the full timed A to B |
| A (0) / Space / F12 | STOP; center sticks again before normal manual resumes |
| B (1) / keyboard X | Latched abort and exit; firmware reset required |
| Esc / close | STOP and exit |

Capture works even while manually moving: it brakes first and captures the stopped
position, not the position at the instant of the button press. Ordinary manual
movement, BNO quality changes, F2 and STOP preserve A/B. Firmware restart, a changed
coordinate epoch, forced/uncertain motor stop, or loss of communication/coordinate
confidence clears captures. They remain in memory only; no persistence is added.

Center sticks before requesting automatic movement. One press initiates one
sequence; release buttons/neutralize the hat for another. Busy movement presses
and hat diagonals are discarded. No JOG is sent during an automatic sequence.
Deliberate stick displacement outside the existing deadband cancels the sequence,
sends STOP, waits for stopped READY and the zero-JOG acknowledgment, then applies
the current live stick value without requiring centering or a mode change.
This is braking plus protocol latency, not an abrupt motor-direction reversal.
Noise within deadband does not cancel motion. Explicit STOP, focus loss and F2
cancel pending continuations and clear this takeover permission.

Play first obtains a fresh stopped snapshot. If away from A it uses `KEYRETURN`,
which has its own travel time independent of the selected playback duration.
After successful return and READY it waits 200 ms, obtains another stopped
snapshot and verifies A exactly before sending the timed `KEYMOVE` to B. This is
motor-count verification, not measured-angle settling. Failed/stopped returns,
invalid epochs, missing acknowledgments and STOP cancel the remaining sequence.
If already at A, the initial stopped snapshot is sufficient to start A to B.

The live MANUAL + KEYFRAMES panel shows action phase, duration, increments and
captures even with F3 frozen. F2 retains exclusive raw-command entry; keyframe
buttons and joystick takeover are inactive in the editor. Exiting F2 sends STOP
and requires centered sticks for 0.5 seconds before normal manual operation.
After a communication fault, STOP provides deliberate recovery; actions do not retry.

Default buttons: A=0, B=1, X=2, LB=4, RB=5, LS click=8, RS click=9, Home=10;
D-pad is hat 0. `--capture-a-button`, `--capture-b-button`, `--duration-button`,
`--play-button`, `--return-a-button`, `--increment-button` and `--move-hat` remain
configurable. `--auto-button` was removed along with the mode toggle. Dry-run
opens no serial and cannot capture real keyframes.

### Finite step-position protocol

These operations are separate from the existing sensor-based POSE controller:

```text
SNAP request_id
KEYRETURN request_id epoch yaw_steps pitch_steps carriage_steps
KEYMOVE request_id epoch yaw_steps pitch_steps carriage_steps duration_ms
```

`SNAP` requires READY and all motors stopped. `KEYFRAME_SNAPSHOT` returns the
request id, coordinate epoch, all three generated-step counts and the latest
BNO heading/physical pitch with validity, receipt age, accuracy and north status.
The epoch is randomized on boot and changes when position confidence is lost.
The host checks this epoch before return/play; firmware checks it again at admission.

`KEYMOVE` uses absolute generated-step targets in that startup-relative epoch.
**All three coordinates are unhomed motor counts**, including yaw and pitch.
Neither snapshots, KEYRETURN nor KEYMOVE require BNO availability, freshness,
accuracy, north qualification, pitch baseline or sensor-derived travel limits.
Sensor stalls/errors/resets remain diagnostic information during generated-step
motion. The sensor worker remains isolated; no foreground sensor I/O was added.
Manual and generated-step travel are operator-supervised and unhomed.

Pitch manual/step travel uses 2400 steps/s and 2400 steps/s?, twice its previous
1200/1200 limits. Yaw and carriage remain at 2000/2000. POSE precision/bootstrap
and angular slew limits are unchanged. KEYRETURN uses these native travel caps
and rejects travel over 60 seconds; short axes may finish before long ones.

Every axis is planned before any start, targeting the requested duration with
approximately 20% acceleration / 60% cruise / 20% deceleration. Integer
acceleration can shorten small-axis ramps; a capped-acceleration profile is used
when the preferred ramp would exceed the cap. Speed is recalculated to retain the
shared duration. All nonzero axes start in one foreground pass using nonblocking
FastAccelStepper finite moves. Starts are near-simultaneous, not pulse-locked.
The separate travel speed/acceleration ceilings remain the bounds. Infeasible durations,
integer overflow and overly sparse steps are rejected before movement. Firmware
accepts 1000..60000 ms; Xbox offers only 5/10/20 seconds. The selected duration is
not a speed limit, and manual `--speed-scale` does not govern autonomous movement.

Sparse moves reject if their cruise step interval would exceed the larger of
250 ms or 5% of duration. Thus extremely small captured displacements may need
a shorter duration; the UI never automatically enlarges a move or changes time.
Runtime endpoint timing must be within the larger of 500 ms or 10% of duration.
An axis stopping short or arriving substantially early stops the whole timed operation.
Rejections identify the limiting axis, requested duration, displacement, speed cap,
acceleration, theoretical minimum duration, and whether timing or step resolution
is the problem. The minimum is a physical lower bound, not a guarantee for every
rounded profile. Sparse moves may need a shorter duration, not a longer one.

`KEYMOVE RESULT` reports PASS/STOPPED/FAILED, final counts, requested/elapsed time,
per-axis endpoint times and maximum concurrent axes. PASS means generated-step
targets reached; `angle_settling=NOT_CHECKED` deliberately makes no measured-angle
settling claim. Backlash, missed motor steps or externally moving the platform
can change actual framing without changing generated counts; recapture if that
occurs. No homing, absolute carriage measurement or learned gearbox ratio is added.

STOP uses native travel braking acceleration and the three-second forced-stop
fallback. Motor/API faults, wrong endpoints and deadline failures stop playback;
BNO quality does not. Host focus loss/disconnection requests STOP. A broken serial
link cannot guarantee delivery; the 250 ms manual JOG lease is not a KEYMOVE lease.

First supervised validation after a separately authorized upload:
1. Run host `--dry-run` to check buttons and the new default workflow.
2. Start powered testing at reduced manual `--speed-scale`, verify axes/signs,
   A/Space/F12 STOP and latched abort/reset with ample physical clearance.
3. Position and press LB; make small visible manual changes, then press RB.
4. Select a feasible duration, move away from A, center sticks and press Home.
   Verify return to A completes before the full timed A to B starts.
5. Test joystick takeover separately during return and playback; verify no queued
   second leg resumes. Test STOP and F2 cancellation too.
6. Repeat with small three-axis displacements and inspect generated counts/results.
   Independently validate pitch at higher travel rates; offline checks cannot
   establish torque margin or detect missed physical steps.

No upload or physical motion was performed during this implementation.

## Freeze diagnostic display

Press **F3** in either manual or F2 raw-command mode to freeze the displayed
diagnostic values, receipt ages and scrolling firmware-response text. A bright
yellow **DISPLAY FROZEN** banner identifies the snapshot. Press F3 again to
immediately show the latest state and resume updates; held-key repeats do not
toggle it repeatedly. The control status and raw-command editor remain live.

This pauses only displayed diagnostic text, not motion or communication. Serial
reception, sensor processing, existing logging, STATUS polling, joystick commands,
F2, STOP/abort and focus-loss handling continue normally. New responses are still
processed and logged under the existing logging policy while hidden by the
snapshot; unfreezing shows the latest response tail, without replaying a backlog.

## Raw commands in the Xbox window

The Xbox application keeps its one existing serial connection. Press **F2** to
open the command line. This sends STOP, disarms manual control and disables all
generated joystick JOG traffic. Wait for firmware READY; pressing Enter early
does not queue anything for later transmission.

Type the command, using **Tab to insert each space** between fields. Space is
deliberately reserved for STOP in every mode. For example, type `POSE`, Tab,
your yaw value, Tab, your physical pitch value, Tab, your carriage count, then
**Enter**. Backspace erases the last character. There are no preset coordinates
or automatic test commands. Whitespace-only input is rejected locally; otherwise
the firmware decides command syntax and admission. Each submission sends exactly
the entered single line plus a newline, with no automatic retry.

Raw mode remains active after submission, rejection, completion and STOP. Moving
sticks does not send JOG. **Space, Xbox A or F12** sends STOP; **X or Xbox B** keeps
the existing latched-abort-and-exit behavior. **Esc/close** sends STOP and exits.
Focus loss also requests STOP. These safety actions take priority over an Enter
queued in the same input batch. Command text is accepted only in the explicit
focused editor; Xbox buttons are never text.

Press **F2 again** to leave the editor; this also sends STOP and leaves manual
control disarmed. Wait for READY and center sticks for 0.5 seconds; the host then
performs the existing manual arming handshake. Manual traffic never
resumes automatically after POSE.

The persistent diagnostics show heading, `physical_pitch` (with its declared
axis), `north_usable`, BNO accuracy, and **Current carriage_steps**. Carriage is
taken from MANUAL_STATE/POSE_STATE telemetry and shows receipt age; unknown is `?`,
and a firmware restart clears the cached count. It remains an unhomed generated
step count. Use the current count, not `0`, to leave carriage unchanged.

Exact submitted text is echoed in the window and console. Unsampled
`RAW_TX_ATTEMPT` (exact bytes including newline) and `RAW_TX` (text after the write
succeeds) are stored in the same existing session log. Firmware responses and
telemetry continue through their existing logging/display path. A successful
write is not firmware acceptance; inspect the firmware response.

This is a single-line editor: no clipboard/history/cursor editing, and long text
shows its trailing portion without truncating the transmission. X remains an
abort shortcut; the firmware also treats any X/x byte as abort, even inside a
line. Unsupported/non-ASCII/overlong commands are left to firmware rejection.
`--dry-run` previews raw text without opening serial. Continuous raw JOG commands
are not streamed; the firmware's manual lease still applies.

This host feature does not update the ESP32. Before powered POSE testing, verify
the board has the restored POSE firmware, including ordinary STOP support.

## BNO-based POSE remains a separate capability

The angular controller uses BNO heading and **Euler PITCH**, with existing motor
signs, target domain, braking and precision correction. It still
requires fresh plausible orientation and a working feedback watchdog. Low accuracy,
missing qualification and an unlearned pitch baseline alone no longer reject yaw
or pitch movement. Heading means the sensor's reported heading, not verified true
north; `north_usable` continues to report magnetic qualification separately.
Loss of essential feedback cancels that operation and returns manual availability.
Encoder acquisition does not replace this feedback or invent a gearbox calibration.

When angular timing is absent, an explicit first POSE can use precision-only
corrections: angular changes within +/-3 degrees, at most 80 pulses/s and 16
steps per angular burst, with acceleration 250 pulses/s² and no continuous slew.
Carriage is capped at 80 pulses/s for that operation. Successful stopped motion
learns timing for later planning; there is no automatic calibration movement.
`MOVE 0 dp ds` explicitly leaves yaw uncontrolled. Absolute POSE always holds its
yaw target, even at low accuracy. Stale/absent feedback, invalid quaternions,
ambiguous heading transitions, direction/progress/angle guards and deadlines still
stop sensor-based control. Historical orientation alone cannot close a live loop.

Absolute yaw remains a target throughout the operation even if its initial
error was already within tolerance. Feedback velocity, ordering and settling
use worker receipt timestamps. Post-stop corrections/settling require samples
received at least 100 ms after stopped motor state was observed. STOP cancellation
does not wait for sensor results; it brakes all moving axes and has a 3-second
force-stop fallback. X still latches abort. Host disconnect alone is not a POSE
stop; the 250 ms command lease applies to manual motion.

`POSE` uses absolute BNO heading, absolute BNO pitch and an absolute unhomed
carriage step count. `MOVE` adds relative deltas; only `MOVE 0 0 steps` is fully
sensor independent. Keep the current `POSE_STATE carriage_steps` value to leave
carriage unchanged. Finite moves retain duration and signed-32-bit checks.
`BNO_STATE physical_pitch` and `pitch_axis=PITCH` identify control feedback;
`pitch_roll` remains raw Euler roll for old log readers. Python displays the
explicit physical pitch, with legacy-firmware fallback.

See [restoration changes, offline evidence and proposed physical test](audit/POSE_RESTORATION_2026-09-27.md).
The restored pitch-only POSE was physically validated before this workflow change.
The new workflow/travel-rate changes require separate physical validation.

Last plausible BNO orientation is retained separately from control eligibility.
`BNO_STATE has_sample=YES` can coexist with `fresh=NO`, `available=NO`, accuracy 0
or `north_usable=NO`; heading/pitch remain visible with sample age. Old handoff
samples never become fresh through foreground consumption. Invalid/nonfinite
quaternions do not replace the retained sample; a full firmware reset starts with
no historical sample. No-data numeric placeholders are marked `has_sample=NO`.

## Persistent diagnostics and BNO audit

Each Python run creates a new UTF-8 text file in `logs/` beside `xbox_control.py`,
named `m09_<UTC timestamp>_<pid>_<suffix>.log`. The full path prints at startup and
the filename stays visible in pygame. Use `--log-dir <directory>` to select a
different location. Files are created exclusively and never silently overwritten.
Every record is timestamped and flushed; exceptions include their traceback,
including when serial/controller initialization or serial I/O fails.

Logs retain raw relevant serial lines and parsed fields, BNO/encoder diagnostics,
host/firmware state changes, rejected-command reasons, STOP/abort/rearm events,
and sampled manual commands. JOG starts, zero transitions and reversals log
immediately; held/changing magnitudes are sampled at 2 Hz, not every 20 ms.
`TX_ATTEMPT` records an attempted write, not proof the board accepted it. Disk
errors are shown without adding a motor interlock. A slow filesystem can consume
the existing host timing budget. Flushing is not a guarantee against power loss.

Firmware adds two records at the existing 500 ms sensor telemetry cadence:

- `BNO_REPORT`: last returned report ID/status/sequence/time, cumulative observed
  status counts/transitions, unexpected reports, reset/failure and telemetry-drop
  counters. These count events returned by the library, not every sensor packet.
- `BNO_RAW`: last 0x05 rotation-vector quaternion (`w=real,x=i,y=j,z=k`), raw library
  status, separate heading accuracy in radians, norm², resulting Euler yaw/pitch/
  roll, timestamps/age, conversion validity and acceptance/rejection reason.

`BNO_STATE` remains the last accepted control data and now explicitly identifies
whether a sample was ever accepted, plus its sequence/timestamps. Before the
first sample, `has_sample=NO` distinguishes the initial zero from a sensor-reported
status zero. Raw and accepted values may differ after a rejected event; match their
timestamps. Inspect `fresh`/age even when `accepted=YES`, since acceptance happened
when the stored event arrived. Python does not recalculate accuracy or Euler angles.

No concrete quaternion conversion bug was found. The conversion math remains
unchanged; POSE's physical feedback selection was subsequently corrected from
roll to pitch using powered-log evidence. The historical
[BNO path and manual restriction inventory](audit/BNO_PATH_AND_MANUAL_LIMITS_2026-09-26.md)
explains all active caps, delays, stops and inactive POSE-only guards. NORTH_LEVEL
is still a legacy enum/summary label plus stationary reference qualification;
there is no North + Level command or button operation.

## Temporary manual pitch direction diagnostics

`PITCH_DIR` records observe manual pitch transitions only: `RUN`,
`BRAKE_REVERSE`, `BRAKE_ZERO`, `STOPPED`, axis-local `FORCE_STOP`, and one
`DIR_CHECK` at least 100 ms after a successful start. There is no added delay;
the later check runs in the normal foreground service and is canceled by braking.
Held JOG packets and same-direction speed changes do not generate extra records.
At most eight records are attempted per one-second diagnostic window. `suppressed`
counts rate-limited records since the previous emission; existing `tx_dropped`
still accounts for a full serial queue. Python's existing session logger records
these lines without a Python change.

Fields: `at_ms` is board time at capture; `cmd` is the signed physical-axis
request; `step_dir` applies the existing -1 pitch sign (zero for a zero request);
`call`/`rc` identify the actual run attempt and its numeric result (`none`/`NA`
when no run call occurred); `active_sign` is the manual state's physical command
sign; `braking`, `observing`, and `ending` expose existing state. `motor_mHz` is
FastAccelStepper's signed pulse-queue/ramp rate, not measured shaft speed.
`RUN` captures the call result before the existing active-sign assignment, so
`active_sign=0` there is normal. `FORCE_STOP` captures the pre-clear request/state
after the existing forceStop call; it is not a new stop or guard.

**`dir26_out` is GPIO26's output-register readback, not voltage measured at the
driver.** No GPIO mode is changed. Output-only GPIO need not have its input buffer
enabled, so the diagnostic deliberately avoids treating `digitalRead()` as valid
pad feedback. FastAccelStepper applies direction asynchronously: the immediate
`RUN` readback may still show the previous level. Compare the later `DIR_CHECK`.
Positive pitch should select `runBackward`, eventually LOW/0 and negative motor
Hz; negative pitch should select `runForward`, HIGH/1 and positive motor Hz.

After **you** flash this build, perform this short pitch-only test:

1. Provide clearance for a brief move either way. Start the usual Xbox client
   with `--port COM9 --speed-scale 0.25` (temporary test override; default remains
   1.0). Keep right stick and left-stick horizontal centered throughout.
2. Center both sticks for at least 0.5 s and confirm the automatic manual handshake.
3. Push left stick straight up for 0.5 s, then center for 1 s. Note the physical
   direction. Expected command +250, `runBackward`, later `dir26_out=0`.
4. Push it straight down for 0.5 s, then center for 1 s. Note the physical
   direction. Expected command -250, `runForward`, later `dir26_out=1`.
5. If movement is controlled and sufficient clearance remains, test one reversal:
   up 0.5 s, directly down 1 s, then center 1 s. Expect `BRAKE_REVERSE`, `STOPPED`,
   then the opposite `RUN`/`DIR_CHECK`. Braking retains the previous direction
   until stopped. If it continues the wrong physical way, stop the test.
6. Press Space/A to STOP/disarm, close normally, and retain the session log plus
   your observed directions. Verify yaw/carriage commands remained zero.

If latch readings change correctly but the shaft still turns one way, the next
distinguishing evidence is a STEP12/DIR26 capture at the driver input, made in a
separate user-controlled test. This diagnostic does not assert electrical delivery.

## Build and offline tests

From this directory:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
& .\tests\run_host_tests.ps1
```

The test runner first tests encoder acquisition separately, then current M09
lifecycle/manual/carriage/POSE scenarios and Python protocol/headless UI tests.
Historical automatic-startup and fallback fixtures run against the unchanged M08
source, separately from M09. The M08 SHA256 snapshot is checked at the end.
See [VALIDATION.md](VALIDATION.md) for the actual results and limitations.

Python needs pygame and pyserial from `requirements.txt`. On this machine the
previously installed packages are in `.pio/python_deps`; the old `.venv` references
an unavailable Python3.6 installation. The offline runner uses PlatformIO's Python
and that existing package directory. The same environment can run input-only:

```powershell
$env:PYTHONPATH = (Resolve-Path .pio\python_deps).Path
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" xbox_control.py --dry-run
```

No serial port is opened in dry-run. For later user-run hardware testing, remove
`--dry-run` and select `--port COM9`. This change was not flashed or physically tested.
The board still contains its earlier firmware until the user deliberately uploads.

## Controlled first hardware check, after a user-approved upload

1. Start with motor power off. Confirm immediate manual READY and live AS5600 A/B
   and BNO status. Verify raw axes in dry-run; no automatic startup travel is expected
   from this firmware. A missing sensor should be reported without losing READY.
2. With travel clear and motor power enabled, select the intended scale explicitly
   (for example `--speed-scale 0.25`). Center, arm, and jog
   one axis a small distance. Release, reverse briefly, and check STOP for each axis.
3. Supervise rail travel; test carriage beyond the former 500-step boundary with
   ample physical clearance. There is no software rail endpoint.
4. During a small jog, close the host or interrupt its command stream. Confirm stop
   and deliberate centered rearm without reset. Check explicit keyboard X/controller B separately;
   that action still deliberately latches abort.
5. Check sensor-failure behavior only with a controlled setup; do not hot-unplug
   shared I2C wiring while moving. Offline fixtures cover missing/stale devices,
   but cannot establish real bus timing, mechanical response or clearance.

Historical sources and the pre-change audit remain under `audit/`; those records
describe earlier behavior and are not the current operating instructions.
