# M07 slew/precision software validation

Date: 2026-09-08. Physical verification of this revision is pending.
No firmware upload, commit, or push was performed. All development files are
inside M07. Repository checks found no changes to M05, M06, or shared firmware.

## ESP32 build: PASS

From this project directory:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
```

Final source built successfully for `esp32dev`, Espressif32 7.0.1,
Arduino-ESP32 2.0.17, FastAccelStepper 1.2.7, Adafruit BNO08x 1.2.7,
BusIO 1.17.4, and Unified Sensor 1.1.15.

- RAM: 32,156 / 327,680 bytes (9.8%).
- Flash: 337,965 / 1,310,720 bytes (25.8%).
- Firmware image: `.pio/build/esp32dev/firmware.bin`.

PlatformIO needed its normal user-profile package-cache/lock access to build;
no package version or project configuration was changed.

## Native regression result: PASS

The final runner passed **85 math checks, the independent-watchdog tests, and
40 integration scenarios**. The final suite includes the recommended first-test
geometry (20 degrees yaw, 8 degrees pitch) and a separate lower synthetic gain
of 0.0075 cradle degrees per pulse on each axis; both reached and settled.
These values describe test fixtures, not a calibrated drivetrain.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
```

The runner compiles math, independent-watchdog, and actual `setup()`/`loop()`
integration tests with C++11, `-Wall -Wextra -Werror -pedantic`. Only the standalone
watchdog target suppresses unused fixture-variable warnings because it includes
shared Arduino fixtures without exercising sensor/UART paths.

Coverage includes:

- Unchanged 0/360 shortest-heading math and continuous heading tracking;
  braking-margin monotonicity; accuracy timer reset, expiry, and clock rollover.
- Independent continuous yaw/pitch commands, overlapping motion, actual stop
  before precision, and fresh feedback before any post-stop command.
- Cumulative yaw confirmation when each individual BNO change is less than
  0.3 degrees, fixing the former perpetual-probe behavior.
- Full stopped settling, near-north correction from both sides of zero,
  transient and repeated accuracy recovery, and sustained-accuracy expiry.
- Target-valued feedback and an expired settling timer cannot complete while
  the asynchronous motors are still running. Frozen but freshly delivered
  orientation readings trigger a slew progress stop on the first expiring axis.
- Baseline refusal for persistent/intermittent low accuracy; acquisition gaps,
  invalid vectors, wrong report types, and a one-second blocked BNO read.
- Independent stale stopping during that blocked read, with the old freshness
  timestamp and latched trip retained after the call returns.
- Startup/active serial abort, startup and active BNO reset, wrong direction,
  runaway, positive/negative yaw/pitch guards, and motion/progress/overall deadlines.
- Rejected finite and continuous commands, initialization failures, and UART
  backpressure without starving the synthetic BNO service.
- Both axes stopped and one summary only, followed by idle with no new commands,
  sensor acquisition, or telemetry after every terminal scenario.

The watchdog-specific fixture checks the 149/150 ms boundary, both stop requests,
reassertion after a trip, late fresh-sample rejection, disarm/rearm latch behavior,
initial stale timestamps, a concurrent future timestamp, and millis rollover.

## Interpretation and remaining physical checks

The integration fixture has independent acceleration-limited motor states and a
1 ms simulated clock. It models `forceStop()` as immediate and explicitly advances
the watchdog during a blocked sensor call. Thus the demonstrated 150 ms is the
simulated stop-request decision, not measured ESP32 scheduling or physical stop
latency. Production uses a 10 ms timer cadence and FastAccelStepper's queued-pulse
drain. The fixture does not emulate FreeRTOS task scheduling or prove thread
interleavings on hardware.

Synthetic plant gains are independent of the control's error-to-pulse constants.
No simulated result establishes a real gearbox ratio, motor torque, pulse quality,
magnetic accuracy, BNO fusion lag, I2C electrical health, or mechanical clearance.
The conservative braking estimate, smoothness, final errors, and response to X/x
must be checked on the actual cradle using the README procedure.

The earlier I2C error/acquisition gap has not been reproduced or electrically
fixed by a host test. The new independent watchdog limits continued commanded
motion if foreground acquisition stalls; physical logs are still required.
