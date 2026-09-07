# Milestone 4: Pitch Drivetrain Characterization

Status: implementation, PlatformIO build, and software checks passed; **physical verification PENDING**.

## Purpose and scope

The new project is `diagnostics/pitch_drivetrain_characterization`. It measures
the relationship between commanded pitch steps, FastAccelStepper's reported
position, motor-shaft motion measured by the AS5600, and cradle pitch measured
by the BNO085. It learns the direction relationships and estimates the effective
motor-to-cradle reduction in both directions. It also records repeatability
and a bounded, noise-aware reversal experiment.

This is an **open-loop characterization diagnostic**. Sensor measurements do
not correct the commanded trajectory. No closed-loop control, PID, homing,
Xbox input, Raspberry Pi communication, keyframes, tracking, or north calibration
is implemented. Yaw and carriage are held stationary. The physically verified
Milestone 3 diagnostic and its verification record remain unchanged.

## Confirmed physical configuration

The user identified the mechanically installed pitch encoder as the AS5600 on
**bus B / `Wire1`, SDA GPIO 4, SCL GPIO 5, address `0x36`**. It shares this bus
with the BNO085 at the verified address `0x4A`. Earlier Milestone 1-3 records
established these bus connections but did not identify which encoder had the
installed motor-shaft magnet; this association comes from the user's subsequent
hardware confirmation.

| Component | Connection and physical measurement |
| --- | --- |
| Pitch NEMA 17 | DIR GPIO 26, STEP GPIO 12 |
| Pitch AS5600 | Bus B, SDA 4 / SCL 5, `0x36`; magnet on pitch motor rotor/output shaft |
| BNO085 | Bus B, SDA 4 / SCL 5, `0x4A`; mounted on final moving pitch cradle |
| Second AS5600 | Bus A, SDA 18 / SCL 19, `0x36`; communication-only, mechanical installation pending |
| Yaw | DIR GPIO 32, STEP GPIO 33; no move commands |
| Carriage | DIR GPIO 21, STEP GPIO 22; no move commands |

**The pitch AS5600 measures motor-shaft angle, not final cradle angle.**
The BNO085 measures orientation on the output side of the mechanical reduction.
Approximately 15 motor revolutions produce one cradle revolution. **15:1 is a
nominal travel estimate, not a calibrated or enforced ratio.**

The user confirmed **200 full motor steps/revolution and 3200 driver input
pulses/revolution**, equivalent to 16 microsteps per full step. One commanded
step in this diagnostic is one driver pulse: 0.1125 nominal motor degrees.
FastAccelStepper positions count generated steps; they are not independent
measurements of rotor or cradle position.

The project retains the shared `firmware/include/hardware_config.h` pin map,
100 kHz I2C buses, 50 ms I2C transaction timeouts, and the verified pinned
dependencies: Espressif32 `7.0.1`, Arduino-ESP32 `2.0.17`, FastAccelStepper
`1.2.7`, AS5600 `0.6.7`, and Adafruit BNO08x `1.2.7`.

## Motion and physical clearance

The user's requested **+1000 to -1000** movement is implemented as absolute
firmware targets relative to the starting position: the test remains within
**[-1000, +1000] pulses**, starts at zero, and finishes at zero. It does not
mean repeatedly adding 1000 pulses beyond those endpoints.

At the nominal 15:1 reduction:

| Movement | Nominal motor angle | Approximate cradle angle |
| --- | ---: | ---: |
| 25-pulse reversal probe | 2.8125 degrees | 0.1875 degrees |
| 100-pulse initial check | 11.25 degrees | 0.75 degrees |
| 450-pulse measurement segment | 50.625 degrees | 3.375 degrees |
| Start to either 1000-pulse limit | 112.5 degrees | 7.5 degrees |
| Full -1000 to +1000 span | 225 degrees | 15 degrees |

The final travel depends on the actual gearing, driver settings, installation,
and mechanical response. Before uploading or resetting, position the mechanism
so both directions have clearance for this expanded excursion. Remove cable
snags and keep hands clear. Firmware count limits and orientation guards do not
establish physical endstop positions; this diagnostic has no homing or hardware
limit-switch guarantee.

Speed remains **1000 pulses/s**, acceleration **1000 pulses/s squared**, and
direction delay **200 microseconds**, matching the verified pitch motion setup.
Movement is split into 36 segments:

| Stage | Relative segment commands | End position |
| --- | --- | ---: |
| Initial small check and positive measurements | +100, +450, +450 | +1000 |
| First negative reversal and traverse | -25 four times; -450 four times; -100 | -1000 |
| First positive reversal and traverse | +25 four times; +450 four times; +100 | +1000 |
| Second negative reversal and traverse | -25 four times; -450 four times; -100 | -1000 |
| Second positive reversal and return | +25 four times; +450, +450 | 0 |

The repeated 450-pulse segments form the main reduction/repeatability pool.
The 100-pulse preload/trim segments and 25-pulse reversal probes are printed
but excluded from that pool. There are eight planned main segments in each
direction and four reversal experiments. Only the pitch stepper receives
movement commands. No yaw/carriage stepper objects are allocated; their STEP
outputs are held LOW. Actual physical inactivity must still be observed.
The normal sequence takes roughly 2.5 minutes, longer if settling windows retry.

## Startup, acquisition, and settling

1. Initialize both buses, the pitch AS5600, and BNO085 rotation-vector reports.
   The pitch AS5600 must be readable with a usable magnet indication. The
   optional bus A encoder's magnet status cannot block or fail this test.
2. Print the pin map, driver assumptions, motion bounds, and a five-second
   warning before motion. Acquire a one-second baseline window and establish
   starting step count, wrapped/unwrapped motor angle, and cradle pitch. A
   failed required-sensor or baseline gate prevents motion.
3. Run the fixed segment sequence. Service the pitch AS5600 on a 10 ms target
   schedule and BNO085 every loop with 10 ms requested rotation-vector reports.
   Poll the optional bus A AS5600 at 100 ms. These are foreground service
   targets, not hard real-time guarantees under I2C stalls.
4. After every segment, wait at least two seconds, then collect a one-second
   endpoint window. Accept only stable windows with at least 30 fresh BNO
   samples and 30 usable shaft samples, pitch standard deviation at most 0.15 degrees, range at most
   0.5 degrees, and absolute drift between first/second half-window means at
   most 0.2 degrees, and shaft-angle range at most 1 degree. Sample gaps in an
   endpoint window must not exceed 100 ms. An eight-second bounded settling deadline prevents an
   unstable sensor from leaving the sequence waiting indefinitely.
5. Store command acceptance/completion, target and reported step counts,
   endpoint measurements, health counters, and per-move fresh acquisition
   evidence. Print cached telemetry roughly every 750 ms and one segment
   measurement report after settling.

The stability limits are practical diagnostic heuristics, not calibration
certificates or BNO085 accuracy specifications. A new vector is counted only
when a valid report arrives; repeated use of a cached value does not inflate
endpoint sample counts or sensor-during-motion evidence.

Pitch retains the verified rotation-vector quaternion calculation:
`asin(clamp(2 * (w*y - z*x) / (w*w+x*x+y*y+z*z), -1, 1))`, converted to degrees. This is the BNO
orientation frame's Euler pitch, whose physical sign is learned experimentally.
It is not an independent mechanical axis calibration and becomes unsuitable
near the Euler pitch singularity. The baseline must be within 65 degrees of
zero pitch. Absolute pitch of 80 degrees or a change of 12 degrees from the
baseline triggers an abort during the test. These guards provide additional
observed-orientation bounds; sensor outages and mounting geometry limit their
ability to protect the mechanism.

## Angle unwrapping and direction

The same pinned AS5600 library reads the unscaled `RAW_ANGLE` register through
`rawAngle()` and checks `lastError()` immediately, rather than using the
potentially scaled `ANGLE` output. No sensor configuration or OTP is changed.
The [AS5600 datasheet](https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf)
describes the raw and scaled angle registers. The raw output provides 12-bit
wrapped angles (0-4095 ticks per revolution). For
each accepted reading the diagnostic subtracts the previous raw value, then
adds/subtracts 4096 when needed to choose the signed change smaller than half
a revolution. A 359-to-0-degree transition therefore adds approximately one
degree instead of subtracting 359 degrees. Signed changes accumulate in a
64-bit tick count; both wrapped angle and accumulated motor degrees/revolutions
are reported. The accumulated value is relative to the starting encoder reading.

Exactly half a revolution is ambiguous. Acquisition timing and commanded
travel are checked so an outage capable of hiding half a motor turn cannot
silently produce a plausible but incorrect unwrap. Ambiguous continuity
aborts the diagnostic. This method assumes the motor does not independently
move more than half a turn between accepted reads; manually forcing the shaft
during a test can violate that assumption. Summed absolute raw-angle travel
can also accumulate noise and must not be interpreted as exact mechanical
distance traveled.

The software independently learns the sign of AS5600 change and BNO pitch
change for positive motor commands. Opposite encoder and cradle signs can be
valid because their mounting frames differ. A disagreement means subsequent
qualified movements contradict the learned mapping, not simply that the two
sensor numbers have opposite signs.

## Reduction and repeatability

For stable, continuous endpoint pairs:

`effective reduction = abs(AS5600 motor degrees / BNO085 cradle pitch degrees)`

This dimensionless value is motor revolutions per cradle revolution. It uses
the measured motor angle, not the assumed 3200-pulse conversion, as its
numerator. Commanded pulses and reported step displacement are also shown;
multiply them by 0.1125 nominal motor degrees per pulse to compare with the
encoder response.

The denominator must exceed a conservative observed-noise gate:

`max(1 degree, 5*hypot(endpoint standard deviations),`
`    2*sum(endpoint ranges), 2*sum(abs(endpoint half-window drift)))`.

Measured motor displacement must also be at least one degree. Nonfinite data,
unstable endpoints, discontinuity across a BNO reset, or insufficient cradle
displacement make the ratio **INCONCLUSIVE**, avoiding division by a tiny
number. Recovered interruptions remain recorded and can exclude affected
measurements even when sensor health ultimately passes.

Forward, reverse, and combined qualified 450-pulse measurements are summarized
with sample counts, spread/repeatability information, and coarse ratio output
(one decimal place). Sensitivity bounds use the observed pitch-noise gate
around the measured denominator; they are **not statistical confidence
intervals** and do not include all orientation bias, mounting error, mechanical
compliance, or systematic sensor error. A measured ratio different from 15:1
is not automatically a failure. Repeated same-direction measurements and
their spread matter more than extra decimal places.
The summary also compares qualified repeated segments with identical commanded
start/end counts and reports the largest motor/cradle displacement difference.
This separates repeatability evidence from variation across different positions.
The [BNO08X datasheet](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf)
describes fusion error and latency; quiet endpoint windows alone do not establish
absolute orientation accuracy or remove magnetic and mounting bias.

## Reversal evidence and its limits

Each reversal begins with four 25-pulse probes and a stable endpoint after
each probe. Relative to the reversal baseline, the diagnostic looks for
measurable motor-shaft travel before BNO pitch exceeds an observed-noise gate
with a 0.25-degree floor and the same standard-deviation/range/drift terms used
above. The expected signs follow the independently learned direction mapping.

A first above-noise cradle response must be confirmed by a second consecutive
endpoint. When confirmed, the last below-threshold motor travel and first
above-threshold motor travel bracket the **observed cradle-response detection
delay**. That interval includes sensor resolution, fusion/filtering, compliance,
and actual cradle motion. Subsequent larger measurement segments may confirm
or first reveal the response, producing a correspondingly coarse motor-angle
interval. It is **not a direct measurement of gear lash**, and
the firmware does not subtract nominal gearing to invent a precise lash value.

One 25-pulse probe is only approximately 0.19 degrees at the cradle, below the
0.25-degree minimum detection gate. A nonzero detection interval is therefore
expected even with negligible mechanical backlash. No confirmed response,
reset-discontinuous endpoints, inconsistent direction, or excessive noise
leaves that experiment **INCONCLUSIVE**. Inconclusive reversal evidence alone
does not fail otherwise successful motion, sensor, direction, and ratio checks.
Use the logs to decide whether a future higher-resolution output encoder or
external measurement is needed; this milestone does not compensate backlash.

## Failure and recovery policy

The Milestone 3 BNO085 history remains relevant: an earlier approximately
1060 ms interruption recovered, and the final revised Milestone 3 physical
run later passed with zero stale events, zero observable/transport errors,
and zero active-test resets. Those results are not measurements of this new
Milestone 4 diagnostic.

- BNO freshness uses ESP32 reception time of valid, finite `SH2_ROTATION_VECTOR`
  samples. More than **500 ms** without one records a stale event: 50 requested
  10 ms intervals. This allows shared-bus scheduling and several 50 ms I2C
  timeouts while retaining a meaningful record of interrupted delivery.
- An active-test gap of **2000 ms or longer** (200 report intervals) latches a
  persistent required-sensor failure and stops pitch output, including when
  fresh data immediately follows the gap. These are diagnostic recovery limits,
  not manufacturer guarantees or suitable closed-loop control deadlines.
- A recovered gap such as 1060 ms can remain a warning without automatically
  failing the whole run, provided remaining motion and measurement evidence
  qualify. Counters and maximum age retain the event. A reset invalidates cached
  orientation, invokes bounded report re-enable retries, and requires a new
  valid vector for recovery. Estimates spanning the reset are excluded.
- BNO reset events, re-enable attempts/successes/failures, observable failures,
  transport write failures, stale observations/events, and fresh samples remain
  visible. Empty `getSensorEvent` polls are not mislabeled as read failures.
  The local adapter preserves the Milestone 3 finite I2C write-failure behavior
  instead of allowing a failed SHTP write to retry forever.
- The required pitch encoder must communicate and have a usable magnet. A
  required encoder read/magnet outage lasting **500 ms or longer**, or ambiguous
  unwrap continuity, aborts. This differs intentionally from Milestone 3's
  communication-only encoder acceptance: a mechanically usable pitch angle is
  needed here. The previous acquisition gap is checked before accepting recovery
  so a blocking call cannot hide the outage. Missing/weak magnet on optional
  bus A remains informational.
- A rejected step command, unexpected firmware position,
  movement watchdog expiry, settling deadline, orientation guard, or received
  uppercase **`X`** also aborts. The firmware stops pitch pulses and prints a
  failed final summary. An abort does not attempt a sensor-dependent return
  to zero; the mechanism can remain displaced. Serial abort is a software
  convenience, not an independent emergency stop.

## Final result and physical acceptance

The diagnostic prints one searchable final block ending in exactly
`FINAL RESULT: PASS` or `FINAL RESULT: FAIL`. Verbose telemetry stops afterward,
sensor counters are frozen, and the sequence never restarts automatically.
A deliberate board reset begins a new run and its warning/countdown.

PASS requires accepted/completed pitch moves at their commanded firmware
positions, return to the recorded starting count, no yaw/carriage commands,
required sensor health with no persistent failure, and fresh required-sensor
acquisition during every actual move. It also requires at least two qualified
main reduction measurements in each direction with a consistent learned
direction relationship. A motor count returning to zero does not prove exact
physical return or absence of missed steps. Reversal results can remain
INCONCLUSIVE without invalidating other measurements.

Record the following from the real hardware run; leave blanks until observed:

| Physical result | Recorded value |
| --- | --- |
| Run date and saved log | PENDING |
| Exact `FINAL RESULT:` line | PENDING |
| Pitch motion completion / final reported position | PENDING |
| Yaw and carriage stationary throughout | PENDING |
| Nominal step angle versus observed AS5600 angle | PENDING |
| Final motor-shaft unwrapped displacement / revolutions | PENDING |
| Final BNO cradle pitch displacement | PENDING |
| Positive command -> AS5600 direction | PENDING |
| Positive command -> cradle pitch direction | PENDING |
| Forward ratio / sample count / spread | PENDING |
| Reverse ratio / sample count / spread | PENDING |
| Combined ratio and sensitivity limits | PENDING |
| Four reversal outcomes and detection intervals | PENDING |
| Pitch AS5600 failures / magnet warnings | PENDING |
| BNO stale / reset / observable failure / write failure counters | PENDING |
| BNO report re-enable and recovery results | PENDING |
| Sensor acquisition during every move | PENDING |
| Warnings, unexpected motion, or binding | PENDING |
| Summary remains visible; no automatic restart | PENDING |

**Physical verification remains pending until the user runs this revision and
reviews its measurements.** Bus A AS5600 mechanical verification remains pending
separately. No values from synthetic tests are physical measurements.

## Software validation

PlatformIO build: **PASS**, `esp32dev`, with 40,172 bytes RAM (12.3%) and
344,033 bytes flash (26.2%). Firmware was built, not uploaded by this development
session. No commits or pushes were made.

Software checks: **PASS**, strict native C++11 with warnings treated as errors:

- 1,110 synthetic math checks cover forward/reverse 360-degree unwrapping,
  repeated turns, ambiguous half turns, direction handling, valid/invalid
  endpoint ratios, near-zero protection, noise sensitivity, reversal response
  confirmation, and inconclusive cases.
- 1,163 checks across 12 executions of the actual firmware `setup()`/`loop()`
  cover the 36-segment motion envelope and return to zero, pitch-only commands,
  opposite sensor signs, a non-15:1 synthetic ratio, a recoverable BNO gap,
  reset/report-write recovery, persistent BNO failure, encoder failure and
  recovery after a blocking gap, missing magnet, unstable baseline, stepper
  initialization failure, and operator abort. Every scenario checks one final
  summary and 100 simulated seconds of silence with no restart afterward.

Run from the new diagnostic directory:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
```

The runner defaults to the installed `C:\Strawberry\c\bin\g++.exe`; use its
`-Compiler` argument for another compatible compiler. Software
simulation verifies bookkeeping and arithmetic; it cannot verify real pulses,
motor torque, driver microstep switches, actual gearing, magnetic installation,
BNO accuracy, or physical clearance.

## Build, upload, and capture

From PowerShell, use the new project directory (not the Milestone 3 directory):

```powershell
Set-Location -LiteralPath 'C:\Users\jhthe\Documents\Arduino\Gimbal_Project\v05_Add Additional Sensors\diagnostics\pitch_drivetrain_characterization'
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev -t upload --upload-port COM9
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM9 --baud 115200 --filter log2file
```

Close other programs using COM9 first. Uploading or resetting can start the
diagnostic after the warning, so establish mechanical clearance beforehand.
If startup was missed, press ESP32 EN/reset once after opening the monitor
with the mechanism clear. Uppercase `X` requests an abort while the application
is servicing serial input; Ctrl+C exits the monitor and does not command an
abort. Logs are saved under `logs/device-monitor-*.log`.

```powershell
Select-String -Path '.\logs\device-monitor-*.log' -Pattern 'FINAL RESULT:'
```

On the physical machine, verify that only pitch moves smoothly, the first
100-pulse check is small, all travel stays clear, and the expected forward/reverse
segments return the firmware count to zero. Check logical motor angle and
cradle response, approximately plausible direction/reduction, and increasing
sensor counters during movement. Save the entire final block and segment
measurements, including warnings and inconclusive results. Confirm the final
summary stays visible and the motor does not restart. Report those observations
before marking Milestone 4 complete or beginning closed-loop work.
