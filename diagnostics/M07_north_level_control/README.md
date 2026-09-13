# M07: simultaneous magnetic north and level control

This revision adds continuous slew followed by precision positioning. Physical
verification is pending. Changes are confined to M07; M05 and M06 remain the
user's verified milestones. Development does not upload, commit, or push.

The moving-cradle BNO085 is the authoritative orientation measurement. Neither
AS5600 is read: the pitch AS5600 measures shaft angle before reduction, not
cradle pitch. FastAccelStepper independently generates both STEP trains.

| Axis | DIR GPIO | STEP GPIO |
| --- | ---: | ---: |
| Yaw | 32 | 33 |
| Pitch | 26 | 12 |
| Carriage, inactive | 21 | 22 |

BNO085: Bus B / `Wire1`, SDA 4, SCL 5, address `0x4A`, 100 kHz I2C,
50 ms transaction timeout, 10 ms requested `SH2_ROTATION_VECTOR` reports.
The shared hardware configuration and pinned dependencies are unchanged.
Yaw's physically verified trial sign stays **-1**, and pitch stays **+1**.

## Why the previous controller crawled

Every move was finite, with another command allowed only after stopping.
At 200 pulses and acceleration 240 pulses/s squared, an ideal triangular burst
peaks around 219 pulses/s and averages about 110 pulses/s, even with a 1000 Hz
speed ceiling. Also, yaw confirmation compared each sample against the continually
updated best error. Gradual motion could stay in four-pulse, 40 Hz trials forever.
Confirmation now uses cumulative improvement from a fixed starting error.

## Motion and tuning

After initialization and a five-second warning, a stable one-second baseline
requires at least 30 valid readings, all with accuracy >=2, and the existing
noise/drift gates. Targets are magnetic heading 0 and cradle pitch 0 degrees.
The continuous north target now consistently uses the baseline mean, removing
the previous last-sample-versus-mean offset. No heading offset or declination is added.

Each axis has its own state machine; there is no finish-yaw-before-pitch sequence:

1. **PROBE** (yaw only): trials of up to four pulses at 40 Hz until cumulative progress
   toward north reaches 0.3 degrees. Polarity never flips automatically.
2. **SLEW**: when confirmed and sufficiently far away, issue one `runForward()`
   or `runBackward()`. FastAccelStepper accelerates and runs continuously while
   the loop acquires BNO data and monitors error reduction.
3. **BRAKING**: request `stopMove()` once inside the predicted braking boundary,
   or when the target direction changes. Continue sensor acquisition while
   polling for stop; no command or reversal is issued during deceleration.
4. **PRECISION**: after the motor stops and a fresh observation at least 100 ms
   later, use small finite corrections. After handing off from slew, this axis
   stays in precision for this run, avoiding repeated slew/stop cycles.
5. **HOLD**: enter after a stopped observation inside +/-0.10 degrees. Stay idle
   within +/-0.4 degrees; resume precision if a fresh sample leaves that band
   while the other axis remains active. Finish only with both motors stopped
   and fresh, accuracy >=2 readings inside +/-0.4 degrees for a full second,
   with >=30 samples and no gap >100 ms. Completion latches idle until reset.

Tune named constants in `src/control_math.h`:

| Constant | Initial value | Purpose |
| --- | ---: | --- |
| `YAW_SLEW_SPEED_HZ` / `PITCH_SLEW_SPEED_HZ` | 1000 / 600 pulses/s | Continuous speed ceilings |
| `YAW_SLEW_ACCELERATION` / `PITCH_SLEW_ACCELERATION` | 1000 / 600 pulses/s squared | Slew acceleration/deceleration |
| `YAW_APPROACH_DEG` / `PITCH_APPROACH_DEG` | 4 / 2 degrees | Minimum handoff distance |
| `SLEW_ENTRY_HYSTERESIS_DEG` | 2 degrees | Extra clearance before slew entry |
| `BRAKING_DISTANCE_FACTOR` | 1.5 | Deceleration distance multiplier |
| `BRAKING_LATENCY_S` | 0.25 seconds | Measurement/queue latency allowance |
| `VELOCITY_WINDOW_MS` | 100 ms | BNO angular-speed measurement interval |
| `PRECISION_OBSERVE_MS` | 100 ms | Fresh stopped-observation delay |
| `BNO_ACCURACY_GRACE_MS` | 1000 ms | Continuous low-accuracy grace |

The braking boundary is:

```text
approach_degrees + peak_measured_degrees_per_second *
    (0.5 * BRAKING_DISTANCE_FACTOR * slew_speed / slew_acceleration
     + BRAKING_LATENCY_S)
```

Angular speed comes from fresh continuous yaw/pitch readings, retaining the
observed absolute peak during the run. A temporarily low speed reading cannot
shrink the stopping margin. Deceleration time uses configured maximum pulse
speed even while accelerating. No shaft-to-cradle ratio is assumed. Noise can
cause an early handoff; actual fusion lag, load, and stopping distance need
physical verification. This estimate cannot guarantee freedom from overshoot.

Precision keeps `MAX_SPEED_HZ=1000`, `MAX_BURST_STEPS=200`, acceleration 240,
original error gains, and smaller yaw probes. Neither old maximum was increased.

## Accuracy and stale data

- Before reference, any accuracy <2 sample invalidates that baseline window.
  A usable stable baseline must arrive within 20 seconds of sensor startup.
- After reference, the first accuracy <2 sample starts a **1000 ms** grace timer.
  Fresh orientation continues driving control, with all travel/direction/progress
  guards active. Accuracy >=2 clears the continuous timer; separate low episodes
  do not accumulate. A continuous 1000 ms low interval stops and latches FAIL.
- Low accuracy prevents settling acceptance. `north_usable=NO` during grace
  describes current accuracy; the established target is retained.
- Missing/invalid vectors do not refresh data or reset the accuracy timer.
  The separate **150 ms** stale deadline remains unchanged.

An independent 10 ms ESP timer task in `src/motion_watchdog.h` requests
`forceStop()` on both axes at BNO age >=150 ms, including while a foreground I2C
call blocks. Recovery cannot erase the outage or clear its latch. The foreground
checks the latch before/after commands and reports failure when it can run.
The timer performs no serial output, I2C, or STEP generation.

Stopping includes timer scheduling latency (nominally up to 10 ms) plus queued
pulses (FastAccelStepper documents approximately 20 ms). This is a software stop,
not instantaneous mechanical stopping or driver power removal. See the pinned
[FastAccelStepper motion/stop API](https://github.com/gin66/FastAccelStepper/blob/1.2.7/src/FastAccelStepper.h).

BNO polling has no intentional wait for motion, but a packet can require multiple
I2C transactions: 50 ms is a per-transaction timeout, not a whole-acquisition limit.
The HAL failed-write workaround is retained. Active control contains no blocking
motion waits. UART diagnostics are buffered and written only when space exists;
optional lines may be dropped under backpressure. Serial input work is bounded
per loop. Startup and terminal stop/summary retain their existing delays outside
active control. The electrical cause of the earlier I2C error and one-second gap
still needs physical investigation.

Other protections:

- Serial **X/x**, BNO reset detection, and latched idle after completion/abort.
- Measured yaw guard **+/-185 degrees from the fixed continuous north target**.
  This is not a +/-6-degree startup envelope. Shortest differences still use
  `[-180, 180)`; accumulated travel does not disappear across 0/360.
- Pitch absolute guard **+/-75 degrees**, including before the first command.
- Wrong-direction/runaway: error >0.6 degrees above the best measured error.
- Progress: >=0.15-degree improvement within 15 seconds outside tolerance;
  continuous slew tightens this to **2 seconds**. Finite moves time out at
  4 seconds, braking at 3 seconds, and active control at 90 seconds from reference.

## Software checks

From this M07 directory:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
```

Native tests execute actual firmware decisions with independent synthetic motors
and fault injection. They do not validate real pulse timing, torque, magnetic
accuracy, fusion lag, electrical faults, or stopping distance. See `VALIDATION.md`.

## Expected telemetry

`STATE` snapshots appear about every 750 ms. `AXIS YAW/PITCH -> ...` lines mark
transitions; `YAW DIRECTION CONFIRMED` follows cumulative response. Expect
`SLEW -> BRAKING -> PRECISION -> HOLD` per axis when starting far enough away.
Small-error starts may use only precision.

Snapshots show heading, both errors, pitch, per-axis modes and MOVING/IDLE state,
live `concurrent=YES/NO`, signed FAS `yaw_hz/pitch_hz`, signed BNO
`yaw_dps/pitch_dps`, `brake_deg=yaw/pitch`, `BNO_fresh`, `age_ms`, `max_gap_ms`,
`accuracy`, `accuracy_low_ms`, and `north_usable`. Pulse and angular-speed signs
can differ because the yaw positive-step mapping is -1.

A transient accuracy drop prints `BNO ACCURACY LOW`, followed by
`BNO ACCURACY RECOVERED` if it recovers before expiry. The final summary includes
settled errors, burst/slew counts, overlap evidence, sensor counters, accuracy
episodes/recoveries, stale watchdog status, reason, and one `FINAL RESULT: PASS/FAIL`.

PASS still requires observed movement of **both** axes and actual overlap.
Starting exactly on target can settle safely but fail to demonstrate this milestone.
Successful runs report both axes settled, north usable, simultaneous motion YES,
and then remain quiet and stationary until reset.

## Conservative first physical test (operator only)

1. Close the Python controller and other serial clients. Verify existing wiring
   and driver settings; clear mechanism/cables and keep motor power removal
   accessible. North can be up to 180 degrees away: check the actual intended
   path before upload/reset. The 185-degree guard does not establish clearance.
2. While safely inactive, place yaw roughly **15-20 degrees from magnetic north**
   and pitch **5-8 degrees from level**, if mechanically clear. Keep defaults.
   Avoid a first test near 180 degrees or the pitch guard.
3. Ensure usable calibration using the established safe procedure while inactive.
   Keep ferrous tools and strong current cables away. Do not manually move the
   cradle after this diagnostic establishes reference: motion starts automatically.
   If baseline fails, correct calibration while inactive and reset deliberately.
4. Upload **M07 only when ready**, then monitor at 115200 baud with logging:

   ```powershell
   & "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev -t upload --upload-port COM9
   & "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM9 --baud 115200 --filter log2file
   ```

   Upload, opening the monitor, or reset may restart the automatic sequence.
   Watch the banner, five-second wait, and `NORTH REFERENCE ... accuracy=2/3`.
5. First check abort: send **X** during motion and confirm both axes stop, report
   `Operator X abort`, and remain idle. **Ctrl+C only exits the monitor.**
   Use power removal if serial abort is ineffective. Reposition safely and reset
   deliberately for the positioning run.
6. Confirm both errors decrease and both axes can overlap. Watch smooth slew,
   one braking transition per slewing axis, then small precision corrections.
   Abort on wrong-way motion, binding, excessive overshoot, knocking, or unexplained
   sensor behavior. Never enlarge guards/timeouts to bypass a failure.
7. Save the complete log and final summary, including speeds, brake boundaries,
   BNO gaps/accuracy events, and final errors. Verify stillness after completion.
   Increase starting yaw distance only after the short run behaves well.

No carriage motion, external controller protocol, tracking, true-north correction,
keyframes, elaborate PID, homing, or long-term holding is included.
