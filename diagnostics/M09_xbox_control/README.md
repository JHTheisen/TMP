# M09 Xbox manual control

M09 keeps the PC/Python -> serial JOG -> ESP32/FastAccelStepper architecture.
The approved simplification restores both AS5600 readouts and makes manual
velocity independent of orientation/reference availability. M08 is unchanged.

## Hardware and motion

| Axis | DIR / STEP GPIO | Maximum pulses/s | Acceleration pulses/s² |
| --- | --- | ---: | ---: |
| Yaw | 32 / 33 | 2000 | 2000 |
| Pitch | 26 / 12 | 1200 | 1200 |
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
  pitch 1200 and carriage 2000 pulses/s. `--speed-scale 0.25` selects one quarter
  of those rates. This diagnostics pass preserves the increased settings.
- Center both sticks for 0.5 s and press Space/A to arm. No motion is sent before
  the firmware acknowledges the zero-JOG handshake. Telemetry age alone does not
  block that handshake.
- Release an individual stick axis to brake that motor. Reversal retains motor
  acceleration and the existing 100 ms stopped observation.
- Space/A during motion sends STOP and disarms after braking. keyboard X/controller B deliberately
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

## Xbox AUTO and two-keyframe playback

The analog manual mappings/arming, F2 raw editor and F3 display freeze are retained.
**Y** enters AUTO, sends STOP and leaves manual control disarmed. Wait for READY
and center both sticks for 0.5 seconds. AUTO never streams joystick JOG commands.
Y exits with STOP; manual operation then requires deliberate centered A/Space
arming. F2 enters raw mode with STOP; AUTO movement buttons are inactive there.

| Control in AUTO | Action |
| --- | --- |
| D-pad up/down | `MOVE 0 +/-increment 0` (physical pitch degrees) |
| D-pad right/left | `MOVE +/-increment 0 0` (yaw degrees; existing north checks apply) |
| Right-stick click | Select 1 or 2 degree increments; default 1 on entry |
| LB / RB | Capture A / B from a fresh stopped firmware snapshot |
| Controller X (button 2) | Cycle duration 5 / 10 / 20 seconds; initial 10 |
| Left-stick click | Position to A using captured generated-step coordinates |
| Center/Home (button 10) | Play A to B; a fresh snapshot must confirm the platform is at A |
| A / Space / F12 | STOP all participating axes; no automatic resume |
| B / keyboard X | Existing latched abort and exit |

One press produces at most one request; buttons must be released and the D-pad
must return to neutral before another request. Busy presses and diagonals are
discarded, never queued or replayed. The live AUTO panel shows pending/active
state, A/B step targets, duration, last result and rejections even under F3.
Capture and playback use new snapshot replies, not the frozen display. A return
or play press performs a stopped snapshot check followed by one movement command;
STOP cancels this continuation before transmission if it has not yet started.

Default pygame mappings: A=0, B=1, X=2, Y=3, LB=4, RB=5, left-stick click=8,
right-stick click=9, Center/Home=10, D-pad hat=0. Verify these with `--dry-run` before powered use:
button/hat events appear in the window and log. Use `--auto-button`,
`--capture-a-button`, `--capture-b-button`, `--duration-button`, `--play-button`,
`--return-a-button`, `--increment-button` and `--move-hat` if needed. A=0 and B=1
remain reserved. Dry-run opens no serial and cannot capture real keyframes.

Only A and B are held in application memory. Ordinary stopped manual
repositioning and gentle STOP preserve them. Firmware restart, changed coordinate
epoch, forced/uncertain stop, or host request/link timeout invalidates captures.
Lost acknowledgments cause STOP and deliberate recovery, never automatic retry.

### Finite step-position protocol

These operations are separate from the existing sensor-based POSE controller:

```text
SNAP request_id
KEYMOVE request_id epoch yaw_steps pitch_steps carriage_steps duration_ms
```

`SNAP` requires READY and all motors stopped. `KEYFRAME_SNAPSHOT` returns the
request id, coordinate epoch, all three generated-step counts and the latest
BNO heading/physical pitch with validity, receipt age, accuracy and north status.
The epoch is randomized on boot and changes when position confidence is lost.
The host checks this epoch before return/play; firmware checks it again at admission.

`KEYMOVE` uses absolute generated-step targets in that startup-relative epoch.
**All three coordinates are unhomed motor counts**, including yaw and pitch.
It needs fresh BNO orientation, an intact pitch baseline and the existing BNO
watchdog/travel guards, but does not need calibrated magnetic north. BNO is used
for protection, not keyframe endpoint correction. Normal yaw-angle POSE/MOVE
retains its qualified-north and accuracy >=2 requirements.

Every axis is planned before any start, targeting the requested duration with
approximately 20% acceleration / 60% cruise / 20% deceleration. Integer
acceleration can shorten small-axis ramps; speed is recalculated to retain the
shared duration. All nonzero axes start in one foreground pass using nonblocking
FastAccelStepper finite moves. Starts are near-simultaneous, not pulse-locked.
Existing axis speed/acceleration ceilings remain the bounds. Infeasible durations,
integer overflow and overly sparse steps are rejected before movement. Firmware
accepts 1000..60000 ms; Xbox offers only 5/10/20 seconds. The selected duration is
not a speed limit, and manual `--speed-scale` does not govern autonomous movement.

Sparse moves reject if their cruise step interval would exceed the larger of
250 ms or 5% of duration. Thus extremely small captured displacements may need
a shorter duration; the UI never automatically enlarges a move or changes time.
Runtime endpoint timing must be within the larger of 500 ms or 10% of duration.
An axis stopping short or arriving substantially early stops the whole operation.

`KEYMOVE RESULT` reports PASS/STOPPED/FAILED, final counts, requested/elapsed time,
per-axis endpoint times and maximum concurrent axes. PASS means generated-step
targets reached; `angle_settling=NOT_CHECKED` deliberately makes no measured-angle
settling claim. Backlash, missed motor steps or externally moving the platform
can change actual framing without changing generated counts; recapture if that
occurs. No homing, absolute carriage measurement or learned gearbox ratio is added.

STOP uses the existing native braking acceleration on each moving axis and the
three-second forced-stop fallback. BNO stale/reset, motor failures and travel
guards cancel playback. The sensor worker remains the sole sensor owner. Host
focus loss/disconnection handling continues to request STOP; a broken serial link
cannot guarantee delivery, and the manual JOG lease is not a KEYMOVE host lease.

For the first demonstration: manually position, STOP/READY, Y and LB to capture A;
Y back to manual, arm and make small visible changes on all three axes; STOP/READY,
Y and RB to capture B. Select duration, left-stick click to return to A, wait for
PASS/READY, then Center/Home to play. First verify STOP during a return move. Reissue
return-to-A deliberately after cancellation. Deployment/physical validation is
still required; this implementation has not been flashed by the agent.

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
control disarmed. Wait for READY, center sticks for 0.5 seconds, then deliberately
press Space/A to use the existing manual arming handshake. Manual traffic never
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
requires fresh valid orientation; absolute yaw also requires qualified north.
Loss of essential feedback cancels that operation and returns manual availability.
Encoder acquisition does not replace this feedback or invent a gearbox calibration.

When angular timing is absent, an explicit first POSE can use precision-only
corrections: angular changes within +/-3 degrees, at most 80 pulses/s and 16
steps per angular burst, with acceleration 250 pulses/s² and no continuous slew.
Carriage is capped at 80 pulses/s for that operation. Successful stopped motion
learns timing for later planning; there is no automatic calibration movement.
Pitch-only operation remains available without qualified north, while yaw
movement requires the existing qualified reference and accuracy >=2.

Qualified yaw remains a target throughout the operation even if its initial
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
This restoration has not been flashed or physically validated, and no new
milestone has been created.

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
2. Center both sticks for at least 0.5 s, press Space/A, and confirm manual arming.
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
