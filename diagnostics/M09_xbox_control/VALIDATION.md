# Dual AS5600 migration validation — 2026-10-08

Implementation starts from local main `294fc4ea9b91b9450d8be19e6b321398ad932a21`.
The complete pre-edit Git checkpoint is `encoder-migration-checkpoint.bundle`.
See [migration record](audit/ENCODER_MIGRATION.md) for scope and recovery.
No firmware upload, serial connection, physical motion, commit or push was
performed. The separate rollback worktree was not modified.

## Results

- ESP32 `esp32dev` compile: **PASS**, RAM 33,332 / 327,680 bytes, flash
  368,449 / 1,310,720 bytes. Evidence: `.pio/direction-build.log`.
  Dependency graph contains FastAccelStepper 1.2.7 and framework Wire only.
- C++ regression runner: **PASS**, 135 suite/scenario executions, including
  eight unit/diagnostic suites and the installed FastAccelStepper ramp implementation.
  Evidence: `.pio/direction-cpp.log`.
- Complete Python suite: **PASS**, all 236 tests. Evidence: `.pio/direction-python.log`.
- Dashboard rendered and visually inspected at 820 × 620 and 1100 × 760; calibration controls,
  measured yaw/pitch, raw encoder health and ROLL UNAVAILABLE fit the window.
  Render artifacts: `.pio/dashboard-820.png`, `.pio/dashboard-1100.png`.
- `git diff --check`: **PASS**.

## Coverage

Encoder unit tests cover signed scale, reduction, raw conversion, wrapping,
multiple revolutions, repeatable north/level references, unconfigured geometry,
bad magnet, disconnected/stale samples, continuity loss between foreground
snapshots, half-turn ambiguity and millisecond-counter rollover.

Actual firmware integration fixtures cover failed-bus initialization recovery,
NORTH/LEVEL/POSE, calibration admission, GOTO acquisition, continuous TRACK,
wrap crossing, independent measured feedback, disconnection, invalid magnet,
sensor-worker and foreground stalls, target-update lease, STOP, emergency abort,
joystick takeover, stalled motion and wrong direction. The independent watchdog
requests a stop on stale angular feedback even if foreground work is blocked.

Retained motion suites cover manual startup/arming/rates/reversal and command
loss, carriage control, keyframe capture/continuous/stepped playback, braking,
position epochs, and low-rate/zero-rate tracking transitions. Tests exercise
the installed FastAccelStepper implementation's long-period ramp behavior.
An exact-minimum-rate floating-point boundary was corrected to avoid toggling
between zero and 5 mHz for a representational rounding error.

Python coverage includes celestial coordinate conversion, explicit true/magnetic
north mapping, catalog/planet selection including Jupiter, manual RA/Dec,
TRACK HERE, GOTO-to-TRACK lifecycle, low-speed host updates, STOP/takeover,
focus-loss continuity, keyframes, stepped playback, display freeze, calibration
handoff, and admission rejection for old/missing/stale/unreferenced telemetry.
A compatibility test compiles the actual firmware fixture and passes its emitted
protocol-2 telemetry to both production host consumers.

## Reproduce

```powershell
& tests/run_host_tests.ps1
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
```

The runner uses Strawberry GCC with C++11 warnings as errors, the installed
FastAccelStepper library, and PlatformIO Python 3.11. Python dependencies for this
workspace are installed under ignored `.pio/python_deps`. For Python alone:

```powershell
$env:PYTHONUTF8 = '1'
$env:PYTHONPATH = "$PWD\.pio\python_deps;$PWD\tests"
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -B -m unittest discover -s tests -p 'test_*.py' -v
```

## Hardware limits and calibration still required

These are software-only results. Synthetic motor plants and mocked I2C validate
control decisions, not real ESP32 timing, stopping distance, magnetic quality,
torque, backlash or pointing accuracy. The dashboard rendering uses SDL's dummy
driver. No powered validation was performed for this implementation.

The operator subsequently supplied yaw 1:1 and pitch 0.06666667 output revolutions
per encoder revolution. `Start_Gimbal.bat` now stores those ratios and the host
automatically sends their degrees-per-revolution conversion after stopped READY.
The direction signs and physical scaling still require hardware verification.
Establish physical true north and
horizontal with SET NORTH/SET LEVEL, or explicitly configure magnetic north and
declination as described in [calibration](CELESTIAL.md). Geometry is RAM-only;
reboot automatically resends launcher configuration and requires both references
again. A continuity fault
invalidates the affected reference and requires deliberate recalibration.

Verify direction, rate planning, stopping, sensor-loss behavior and pointing on
hardware before relying on automatic motion. The retained 52.0/67.2 motor pulses
per degree are provisional tracking feedforward values, not encoder ratios.
Keep encoder travel below half a revolution between successful reads. Shaft
encoders cannot measure downstream slip/backlash, and no cable-wrap protection
or homing has been added. Two rotary encoders do not measure roll.

Celestial GOTO and TRACK depend only on the calibrated AS5600 feedback for
orientation. No retired orientation-device library, acquisition path,
calibration requirement or fallback remains in the application.

## Launcher ratio follow-up

Added optional paired CLI ratios, nonblocking STOP/READY/configuration/acknowledgment
sequencing, automatic reconfiguration after reboot, and batch-file values supplied
by the operator. Zero references are never captured automatically. Startup timeout
or rejection exits with STOP; dry run opens no serial port.

All 68 relevant setup, real headless host UI, manual session, raw-command session
and firmware/host protocol tests passed, including new startup and reboot cases.
Evidence: `.pio/encoder-launch-final.log`. Firmware was unchanged in this follow-up;
no upload or hardware test was performed.

## GOTO direction follow-up

The supplied powered log exposed a motor-to-encoder sign mismatch masked by the
previous synthetic plant. Automatic angular motion now composes the observed
raw-count motor polarity with the configured scale sign; manual JOG mapping and
encoder scale magnitudes remain unchanged. Finite correction, slew, TRACK and
response/progress checks share that mapping. No deadlines or safeguards changed.

Seventeen new focused tests prove decreasing measured error for positive and
negative targets on both axes, including finite/slew acquisition, signed scales,
TRACK corrections, the logged target pair, direct yaw/15:1 pitch geometry and
stopping on intentionally reversed wiring. The pre-fix positive-axis cases failed.
The full suites and compile were rerun; current results are listed above.
See [direction trace and supervised hardware procedure](audit/ENCODER_DIRECTION_FIX.md).
