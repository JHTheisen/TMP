# Milestone 3: Integrated Motor and Sensor Coexistence

Status: Build and software validation PASSED; physical hardware verification PASSED.

## Scope and hardware observations

This isolated diagnostic combines the verified Milestone 1 three-axis
FastAccelStepper sequence and Milestone 2 dual-I2C sensor configuration. It tests
open-loop stepping and foreground sensor acquisition together. Milestone 4 has
not begun. No closed-loop control, homing, Raspberry Pi communication, or
coordinated motion is included.

## Physical verification results: PASSED

The user completed the physical hardware rerun of the revised diagnostic and
confirmed the definitive final summary result:

```text
FINAL RESULT: PASS
```

| Final-summary check | Measured result reported by the user |
| --- | --- |
| YAW forward/reverse motion | PASS; returned to firmware step count 0 |
| PITCH forward/reverse motion | PASS; returned to firmware step count 0 |
| CARRIAGE forward/reverse motion | PASS; returned to firmware step count 0 |
| AS5600 bus A communication | PASS; 0 read failures |
| AS5600 bus B communication | PASS; 0 read failures |
| BNO085 communication | PASS |
| BNO085 stale events during the test | 0 |
| BNO085 observable read failures | 0 |
| BNO085 transport write failures | 0 |
| BNO085 resets during the test | 0 |
| Sensor telemetry during motor motion | PASS |
| Recorded sensor warnings | NONE |

This completes physical verification of Milestone 3's open-loop motor/sensor
coexistence diagnostic. The second AS5600 remains **communication-only**: its
magnet and mechanical installation, and therefore its mechanical angle-response
verification, are still pending. Firmware step-count return does not establish
exact mechanical positioning accuracy. Milestone 4 has not begun.

## Earlier physical observations

The user ran the previous Milestone 3 diagnostic on the physical hardware:

- All three motors appeared to perform their intended motion sequence.
- Sensor telemetry continued during motion.
- The installed AS5600 and BNO085 produced logical readings.
- The second AS5600 remains electrically verified only; its magnet and mechanical
  installation are incomplete.
- No obvious motor or sensor malfunction was observed, but a final move/result
  summary could not be reliably located in the fast serial scrollback.
- Earlier testing showed a BNO085 reading age of about **1060 ms** and temporary
  `FAILED` status, immediately followed by normal fresh data. This is retained
  as a real observed interruption, not dismissed or retroactively marked passed.

Those earlier observations alone did not establish a definitive recorded
Milestone 3 PASS. The subsequent revised-diagnostic run recorded above now does.
The earlier 1060 ms interruption remains part of the test history; no stale
event occurred during the successful revised test.

## Preserved hardware and motion

The project is `diagnostics/integrated_motor_sensor_coexistence`; its source
includes the authoritative `firmware/include/hardware_config.h`.

| Function | Pin or address |
| --- | --- |
| Yaw DIR / STEP | GPIO 32 / GPIO 33 |
| Pitch DIR / STEP | GPIO 26 / GPIO 12 |
| Linear carriage DIR / STEP | GPIO 21 / GPIO 22 |
| I2C bus A SDA / SCL | GPIO 18 / GPIO 19 |
| I2C bus B SDA / SCL | GPIO 4 / GPIO 5 |
| AS5600 on each bus | `0x36` |
| BNO085 on bus B | `0x4A` |

FastAccelStepper still commands **500 steps forward and 500 steps reverse** at
**1000 steps/s**, with **1000 steps/s squared** acceleration. Axis order remains
**yaw, pitch, carriage**, one axis at a time. Direction delay remains 200 us,
startup warning 5 seconds, direction pause 2 seconds, axis pause 3 seconds,
and each move retains its 10-second watchdog. Both I2C buses remain at 100 kHz
with 50 ms transaction timeouts. BNO rotation-vector requests remain at 10 ms.

The original initialization/motion gate remains: all steppers and buses, BNO085
at the verified bus/address with report enabled, at least one initialized
AS5600, and fresh/healthy required sensor data at the end of startup. Every
initialized encoder must be readable at that point. Allowing motion with one
encoder preserves the existing gate; **the revised final PASS requires successful
communication with both AS5600s**. A missing encoder is an explicit final FAIL.
Missing/weak/strong magnet indications alone never fail or prevent motion.

## Why the previous summary was hard to find

The previous sketch had reachable `MOVE END` messages and a one-time final
`RESULT PASS` or `RESULT FAIL` summary. Completion follows the final carriage
reverse move and its existing 3-second axis pause. However, six-line telemetry
snapshots continued every 250 ms indefinitely, including after completion or
abort: roughly 24 lines per second could quickly push the summary out of the
terminal scrollback. There was no exact `FINAL RESULT:` marker. Without a saved
log, we cannot prove whether the previous physical run emitted its summary.

The old result also required every printed snapshot to be healthy, so one
temporary stale BNO sample could permanently fail the test. Its failed report
re-enable path stopped further BNO servicing, preventing recovery retries.
Review also found a separate potential hang in the pinned BNO library: its I2C
write failure returns zero, which SHTP interprets as an instruction to retry
indefinitely. This could block a final summary during a write failure, although
there is no evidence that it happened in the user's previous physical run.

## Acquisition, counters, and display

- Four-line cached telemetry snapshots print roughly every **750 ms** while
  startup and the test are active. They show phase, axis, all three firmware
  step counts, encoder angles/magnet indications/read counts, and BNO
  heading/pitch/roll, accuracy, age, fresh samples, stale events, and resets.
- AS5600 angle/status reads and health checks run independently on a **50 ms**
  schedule. BNO polling runs every loop and between encoder reads. These are
  foreground service targets, not hard real-time guarantees during I2C stalls.
- Both AS5600s record successful reads and failures, separately for the total
  acquisition period and the active sequence (including its pauses). A success
  requires ACK plus successful angle and status register reads, with library
  error checks after each register operation. Magnet quality is informational.
- Each axis stores forward/reverse command acceptance, completion, target/end
  counts, and return to its starting count. Final actual firmware positions
  are rechecked when the summary is produced.
- Each individual move counts successful reads from each encoder, newly
  decoded valid BNO rotation vectors, and displayed snapshots **only while
  that axis's FastAccelStepper object reports running**. Cached BNO data and
  samples acquired after a move stops cannot substitute for this evidence.
- Sensors continue to be serviced and failures recorded during the pauses.
  Total counters include startup after device initialization; test counters
  cover first move through the final axis pause. Initialization/probe outcomes
  are printed separately.

There is one summary on normal completion or a handled abort. It contains
individual YAW/PITCH/CARRIAGE motion results, both AS5600 communication results,
BNO communication/health counters, per-move evidence, sensor telemetry during
motion, and the exact searchable line **`FINAL RESULT: PASS`** or
**`FINAL RESULT: FAIL`**. All application telemetry and sensor servicing stop
afterward, leaving the counters frozen and the summary visible. The sequence
does not automatically restart; a deliberate board reset starts a new test.

## BNO085 stale and recovery policy

Freshness measures elapsed ESP32 time since receiving a valid, finite
`SH2_ROTATION_VECTOR` through the library. It is reception freshness, not an
independent measurement of the sensor's internal capture time or of orientation
accuracy. No game rotation vector is used.

| Condition | Recorded outcome |
| --- | --- |
| Gap at most 500 ms | Fresh, provided the report is enabled and valid after any reset |
| Gap greater than 500 ms | One stale event on entry; repeated 50 ms stale checks counted separately |
| Fresh vector returns before a 2000 ms gap | Stale episode marked recovered; warning remains visible |
| Gap at least 2000 ms during the sequence or pauses | Persistent failure latched, even if data later recovers |
| No fresh data, disabled report, or reset awaiting fresh data at the end | BNO communication FAIL |
| Any move has no new BNO vector while running | Sensor telemetry during motion FAIL |

The stale threshold is **50 requested 10 ms report intervals = 500 ms**. This
deliberately tolerates scheduling, shared-bus traffic, and several 50 ms I2C
timeouts, yet treats dozens of missed reports as an interruption worth recording.
The persistent threshold is **200 intervals = 2000 ms**: a conservative recovery
allowance, not a claim that multi-second gaps are normal. These are diagnostic
acceptance limits, not sensor manufacturer guarantees or a closed-loop design.

A recurrence of the observed **1060 ms** gap therefore records a stale episode,
maximum gap, and recovery. It can still yield PASS if it recovers, every move
has sensor evidence, and all other criteria pass. Repeated sub-2000 ms recovered
events are all counted and flagged for review; this diagnostic does not impose
a separate cumulative outage budget. A persistent gap always causes final FAIL.
Startup gaps/reset events remain in the total counters; a recovered startup gap
alone does not latch an active-sequence failure, but the startup freshness gate
must still pass.

The previous timestamp is checked **before accepting each new sample**, so a
blocking library call followed immediately by fresh data cannot erase a long
gap. Stale checks count observations, not unique sensor packets or exact missed
reports; an outage entirely inside a blocking call may produce a stale event
and maximum gap even with zero scheduled stale checks.

BNO resets invalidate the cached orientation and trigger report re-enabling.
The summary records total resets (including library startup reset notifications),
resets during the sequence, re-enable attempts/successes/failures, and reset
episodes followed by fresh data. Failed re-enabling is retried on a 500 ms
schedule, and event polling continues while the report is disabled. Successful
configuration alone is insufficient: a subsequent valid rotation vector must
confirm recovery. Closely spaced resets before fresh data may belong to one
fresh-recovery episode.

A diagnostic-local adapter maps the pinned Adafruit I2C HAL's failed-write
return of zero to `SH2_ERR_IO`. This lets report enabling return failure to the
retry logic instead of hanging inside SHTP. Successful writes are unchanged;
the installed library and hardware configuration are not edited. Transport
write failures are separately counted in the final summary. The event buffer
has persistent storage because the library retains its callback pointer.

The library's `getSensorEvent(false)` does not distinguish ordinary empty polls
from internal transport/decode failures. Empty polls are counted separately and
are **not** mislabeled as read failures. The diagnostic records observable
failures as invalid/nonfinite rotation vectors plus failed independent BNO ACK
probes (every 500 ms). ACK only proves address response; fresh rotation vectors
prove report delivery. Hidden transport failures are detected through the gap
policy rather than claimed as an exact low-level error count. Recovered BNO
observable errors or re-enable failures remain warnings; fresh final data,
per-move evidence, and absence of a persistent outage determine BNO acceptance.

## Revised final pass criteria

`FINAL RESULT: PASS` requires all of the following:

1. All six moves were accepted and completed at their targets; each axis's
   reverse move and final firmware position equal that axis's recorded start.
2. Both AS5600s initialized, had successful reads during the sequence, had
   **zero read failures during the sequence and pauses**, and remain readable
   with a last successful read no older than 500 ms at completion. Startup-only
   recovered errors remain visible in total counters. Magnet status is excluded.
3. BNO085 delivered valid new rotation vectors during the sequence, is fresh
   at completion, has its report enabled, has no reset awaiting fresh data,
   and never reached the persistent gap threshold during the sequence/pauses.
4. Every forward and reverse move has at least one successful read from each
   AS5600, at least one new valid BNO vector, and at least one displayed snapshot
   while its stepper reports running. Internal sensor checks also occurred
   between moves. Device communication failures are reported separately even
   if this acquisition-evidence check passes.
5. The sequence completed normally. Initialization failures, rejected commands,
   position mismatches, or watchdog aborts always produce final FAIL.

These checks verify firmware counts and sensor communication/coexistence. They
do not prove physical travel, exact mechanical return, missed-step absence,
encoder mechanical calibration, or closed-loop positioning accuracy.

## Build and software validation

The revised diagnostic builds for generic `esp32dev` using PlatformIO
Espressif32 `7.0.1`, Arduino-ESP32 `2.0.17`, FastAccelStepper `1.2.7`, AS5600
`0.6.7`, and Adafruit BNO08x `1.2.7`. The final build uses **30,484 bytes RAM
(9.3%)** and **336,765 bytes flash (25.7%)**. The user has uploaded and physically
verified this revision with the PASS results recorded above.

Native C++11 validation passed with warnings treated as errors:

- 87 health/evidence checks cover the stale boundary, retained 1060 ms history,
  persistent failure after recovery, startup distinction, timer wraparound, and
  required motion/sensor evidence.
- 345 checks across six simulated executions of the actual `setup()`/`loop()`
  cover normal completion with weak/missing magnets, recovered 1060 ms and
  persistent 2010 ms gaps, an encoder read failure, failed BNO report write
  followed by reset/re-enable recovery, and initialization abort. Each execution
  checks one summary and no output, acquisitions, or new motor commands during
  100 simulated seconds afterward.

Host stubs test bookkeeping and reporting, not hardware pulses, real I2C
transactions, library timing, or physical travel. Rerun these software checks
from the diagnostic directory with the installed compiler:

```powershell
& 'C:\Strawberry\c\bin\g++.exe' -std=c++11 -Wall -Wextra -Werror -pedantic tests/diagnostic_health_test.cpp -o .pio/diagnostic_health_test.exe
& '.pio\diagnostic_health_test.exe'
& 'C:\Strawberry\c\bin\g++.exe' -std=c++11 -Wall -Wextra -Werror -pedantic -I tests/stubs -I ../../firmware/include tests/diagnostic_integration_test.cpp -o .pio/diagnostic_integration_test.exe
foreach ($scenario in @('normal', 'transient', 'persistent', 'encoder_failure', 'reset_retry', 'init_abort')) {
    & '.pio\diagnostic_integration_test.exe' $scenario
    if ($LASTEXITCODE -ne 0) { throw "Integration check failed: $scenario" }
}
```

From PowerShell in the diagnostic directory:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
```

## Upload and capture for repeat verification

COM9 was enumerated as the Silicon Labs CP210x USB-to-UART bridge during this
revision. Close other programs using COM9. From the diagnostic directory, run:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev -t upload --upload-port COM9
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM9 --baud 115200 --filter log2file
```

The monitor saves output to `logs/device-monitor-*.log`. To capture startup if
upload finished before the monitor attached, press the ESP32 EN/reset button
once after the monitor is open and the machine is clear. This intentionally
starts a new run with the same five-second warning. Let the entire sequence
and final three-second pause finish. Exit the monitor with Ctrl+C when done.
The saved result is searchable with:

```powershell
Select-String -Path '.\logs\device-monitor-*.log' -Pattern 'FINAL RESULT:'
```

For any future repeat run, use the same verification checklist:

- Startup pins, sensor locations, and initialization match the hardware.
- Yaw, pitch, and carriage move forward/reverse in that order with the existing
  distances, speed, pauses, and smooth behavior; no unexpected movement occurs.
- The slower telemetry shows changing positions and increasing acquisition
  counters during motion. The installed AS5600 and BNO085 readings remain
  physically sensible.
- The one final summary has all three motion checks, both encoder communication
  checks, BNO communication, and sensor telemetry during motion marked PASS;
  all six move records show began/completed YES and nonzero sensor/display
  evidence, and all final step counts equal their starts.
- Review stale/reset/error counters even if the final result passes. Any
  repeated 1060 ms-like gap must remain recorded with its recovery and maximum
  age; persistent failure must say YES and force FAIL when applicable. Save
  the complete summary/log rather than relying on individual scrolling lines.
- The exact final line is `FINAL RESULT: PASS`, and afterward the terminal
  stays quiet and the motors do not restart.

The second AS5600 passes this milestone through communication even with no or
weak magnet. Its mechanical angle response remains pending until magnet and
mechanical installation are complete. **Physical verification of the revised
Milestone 3 diagnostic is PASSED; only the second AS5600's mechanical
verification remains pending.**
