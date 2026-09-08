# Milestone 5: Closed-Loop Pitch Position Control

Status: implemented and ESP32 build verified; software validation is recorded in
`VALIDATION.md`. **Physical verification is pending. No upload, commit, or push
has been performed.**

This is a separate PlatformIO project at
`diagnostics/M05_closed_loop_pitch_control`, beside the Milestone 4 project at
`diagnostics/pitch_drivetrain_characterization`. It has its own `src`,
configuration, tests, and build output. The Milestone 4 project and other
diagnostics are unchanged.

## What the test does

After at least five seconds without commanded motion, it collects a stable
one-second sensor baseline and uses the mean BNO085 pitch as relative zero.
It then moves toward **+3 degrees of measured cradle pitch**, settles, reports
the measured error, and returns to that original BNO reference using the same
closed loop. It runs once and stays idle afterward.

The user confirmed that positive pitch motor commands increase BNO pitch.
`POSITIVE_STEP_PITCH_SIGN = +1` records that installation setting. No direction
learning or drivetrain calibration runs during this test.

For each correction, the firmware reads the measured pitch error, issues a
finite FastAccelStepper move, waits for the motor to stop, allows 300 ms for
sensor/mechanical settling, and averages fresh readings for 250 ms before the
next decision. Bursts shrink near the target. There is no integral or derivative
control, no continuous-run command, and no queue of successive corrections.

| Initial setting | Value |
| --- | --- |
| Target relative to startup | +3 degrees, then 0 degrees |
| Final position tolerance | +/-0.4 degrees |
| Final stopped observation | 1 second, all sampled pitches inside tolerance |
| Approach deadband on the averaged error | +/-0.10 degrees; leaves room for noise |
| Maximum correction | 16 STEP pulses |
| Requested speed | 40 to 120 pulses/s, decreasing with error |
| Acceleration | 240 pulses/s squared |
| Approximate planning scale | 1600 pulses/motor revolution, about 15:1 reduction |

The approach deadband is an internal centering margin, not a claim of 0.10-degree
absolute BNO accuracy. The accepted endpoint tolerance remains +/-0.4 degrees.
Stable windows follow M04's observed-noise gates: pitch SD <=0.15 degrees,
range <=0.5 degrees, and first-half to second-half mean drift <=0.2 degrees.
The baseline and final windows each need at least 30 fresh BNO and AS samples;
decision windows need at least 10. Noisy windows cause a stationary retry.

**Measured pitch is the source of position truth.** The 1600/15 estimates only
size the next small burst. Neither PASS nor the travel guards compare measured
motion to those estimates. The return may finish at a different pulse count or
motor-shaft angle because it corrects the cradle's measured position.

## Hardware and reused components

| Component | Preserved connection |
| --- | --- |
| Pitch stepper / TMC2209 / 12 V motor supply | DIR GPIO 26, STEP GPIO 12 |
| Pitch motor-shaft AS5600 | `Wire1`, Bus B, SDA 4 / SCL 5, address `0x36` |
| Cradle-mounted BNO085 | Same Bus B, address `0x4A` |
| Bus B settings | 100 kHz, 50 ms I2C transaction timeout |
| Yaw and carriage | STEP33 and STEP22 held LOW; no stepper objects or commands |

The project reads the existing `firmware/include/hardware_config.h` without
editing or duplicating its pin assignments. Bus A is not opened by this
pitch-only test. The other diagnostics retain their existing Bus A monitoring.

Reused from Milestone 4:

- FastAccelStepper pitch connection, direction polarity `true`, 200 us DIR
  setup delay, and `forceStop()` followed by the verified 25 ms queue drain.
- An **unchanged copy** of `src/sensor_support.h`: rotation-vector report support,
  quaternion-to-Euler conversion, BNO health accounting, and the transport fix
  that converts failed writes into `SH2_ERR_IO` instead of an endless retry.
- AS5600 communication/raw-angle/status acquisition and shortest-delta shaft
  reporting. Magnet flags remain advisory, as in the current M04 diagnostic.
- The native C++ real-`setup()`/`loop()` test harness pattern, with separate M05
  fixtures that vary actual pulse scaling and gearing.

Dependencies remain Espressif32 7.0.1 / Arduino-ESP32 2.0.17,
FastAccelStepper 1.2.7, AS5600 0.6.7, and Adafruit BNO08x 1.2.7. BusIO 1.17.4
and Unified Sensor 1.1.15 are explicitly pinned to the versions installed in
M04's known-good dependency directory.

## Safety and PASS

Every fresh raw BNO pitch sample is checked against **+/-6 degrees from the
startup reference** and **+/-75 degrees absolute Euler pitch**. Starting within
6 degrees of the absolute guard is refused. These are measured orientation
guards, not homing or physical endstop positions; start near level with clear
travel and loose cables in both directions.

During an active sequence, a BNO acquisition gap reaching 150 ms or AS5600 gap
reaching 100 ms latches a stop. Old timestamps are checked after sensor calls
and before accepting recovered data, so a blocking read cannot hide an outage.
A BNO reset after establishing the reference also stops the test because the
orientation frame may have changed. Brief read failures below the deadlines
are counted, and new corrections require fresh valid feedback.

Other stop conditions include a motor burst exceeding 3 seconds, a leg exceeding
90 seconds, no meaningful measured progress (0.15 degrees) for 15 seconds while
outside tolerance, or averaged error growing 0.6 degrees beyond the best error
already reached on that leg. A readable but motionless AS5600 is also detected:
each leg must show at least 0.5 degrees of shaft response, with a check after
8 seconds of an active leg. This is a minimal response check, not a gear ratio
or expected STEP-to-angle comparison. Shaft displacement is diagnostic and
assumes less than half a shaft revolution between samples.

The firmware prints **`FINAL RESULT: PASS`** only after both measured endpoints
settle, both legs show motion toward their targets, both sensors supply samples
during motor movement, and motor-shaft feedback responds. AS magnet warnings
alone do not fail a run. PASS describes this modest relative positioning test;
it does not establish absolute accuracy, a calibrated drivetrain ratio, or
long-term position holding. Active correction ends after the return settles.

**Send `X` or `x` to abort.** The motor stops and the test latches idle; it does
not attempt a return after an abort. Reset is required for another sequence.
Ctrl+C only closes the serial monitor and is not an abort command.

The serial stop is serviced by the foreground loop. If a library call blocks,
serial handling and measured guards wait for it to return, but the motor has
only the current finite burst to execute. Keep motor power removal accessible
as the physical emergency stop. Software stop does not switch off driver power.
The reused queue-drain behavior is also documented in the
[FastAccelStepper API](https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md#forcestop).

## Build, upload, and physical test

1. Put the cradle near level, clear travel for at least 6 degrees in both
   directions, check cables, and keep hands clear. Keep the existing driver,
   microstep settings, wiring, and supply configuration. Close the Python
   controller and other programs using COM9.
2. From the `diagnostics` directory, enter the
   **new** project and build:

   ```powershell
   Set-Location -LiteralPath '.\M05_closed_loop_pitch_control'
   & "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
   ```

3. When ready for motion, upload and open the monitor. Upload/reset starts the
   automatic sequence after initialization, the five-second delay, and the
   stable baseline. Substitute another port only if the board has moved.

   ```powershell
   & "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev -t upload --upload-port COM9
   & "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM9 --baud 115200 --filter log2file
   ```

4. If startup was missed, first confirm clearance, then press EN/reset with the
   monitor open. Look for `TMP M05_closed_loop_pitch_control`, `REFERENCE`, and
   `LEG OUTBOUND target_relative_deg=3.000`. Expect short movements separated
   by pauses. Confirm only pitch moves and BNO `relative` increases toward +3.
5. Look for `SETTLED OUTBOUND`, then `LEG RETURN`, `SETTLED RETURN`, and the final
   summary. Both settled errors must be within +/-0.4 degrees. Check the cradle
   visibly returns near its original orientation and remains still after the
   summary. The pulse count need not return to zero.
6. If motion is unexpected, send `X` (and Enter if your terminal buffers input),
   or remove motor power if necessary. Preserve the failure reason and log;
   do not increase limits or retry automatically. A separate deliberate abort
   test can verify the idle latch after a successful positioning run.

Logs are saved under `logs/device-monitor-*.log`. To locate the result:

```powershell
Select-String -Path '.\logs\device-monitor-*.log' -Pattern 'REFERENCE|SETTLED|FINAL RESULT:|Reason:'
```

Report the complete final summary, outbound/return errors, any sensor warnings,
whether pitch moved smoothly without hunting, and whether yaw/carriage stayed
stationary. Physical acceptance and any commit remain pending your verification.

## Software tests and limits

From this M05 project directory:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
```

The runner uses the installed `C:\Strawberry\c\bin\g++.exe` by default; pass
`-Compiler` for another compatible compiler. Tests compile the real firmware
against deterministic host fixtures with strict C++11 warnings as errors.
See `VALIDATION.md` for the tested revision's results and coverage.

Software fixtures do not verify ESP32 timing, physical pulses/torque, I2C
electrical health, magnetic installation, BNO fusion behavior, or mechanical
clearance. The physical run is the remaining milestone acceptance step.

No yaw/carriage control, joystick/Raspberry Pi communication, keyframes, tracking,
homing, north calibration, drivetrain calibration, PID, or autotuning is added.
