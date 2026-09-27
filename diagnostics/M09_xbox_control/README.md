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

Physical pitch feedback for automatic positioning remains **BNO ROLL**. Existing
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
- Space/A during motion sends STOP and disarms after braking. X/B deliberately
  sends the existing latched abort; resetting is required after explicit abort.

| Command | Behavior |
| --- | --- |
| `STATUS` | M09 READY, MANUAL, BUSY or ABORTED |
| `JOG 0 0 0` | Enter manual mode from stopped READY |
| `JOG yaw pitch carriage` | Integer fields -1000..1000; host sends every 20 ms |
| `JOG yaw pitch` | Existing two-field compatibility; carriage request is zero |
| `STOP` | Brake all manual axes and leave manual mode |
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
only deliberate X/B sends X. A host command gap reaching the same 250 ms lease
requires rearming before sending nonzero input again.

## BNO-based POSE remains a separate capability

The working angular controller still uses BNO heading and **ROLL**, its existing
signs, target domain, braking, precision correction and settling rules. It still
requires fresh valid orientation; absolute yaw also requires qualified north.
Loss of essential feedback cancels that operation and returns manual availability.
Encoder acquisition does not replace this feedback or invent a gearbox calibration.

Removing automatic startup movement removes the normal initial timing evidence
for coordinated angular POSE. When required timing is absent, the command is
explicitly rejected while manual remains ready. No new automatic calibration or
first-POSE bootstrap is included. Existing native pitch-only control remains
available when its pitch reference is usable and north is unavailable; carriage
steps do not require magnetic accuracy. Finite automatic moves retain their
existing duration checks and signed-32-bit target/displacement representation.

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

No concrete quaternion conversion bug was found; math, physical ROLL mapping and
sensor/control admission rules are unchanged. The full
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
   and deliberate centered rearm without reset. Check explicit X/B separately;
   that action still deliberately latches abort.
5. Check sensor-failure behavior only with a controlled setup; do not hot-unplug
   shared I2C wiring while moving. Offline fixtures cover missing/stale devices,
   but cannot establish real bus timing, mechanical response or clearance.

Historical sources and the pre-change audit remain under `audit/`; those records
describe earlier behavior and are not the current operating instructions.
