# M09 sensor-worker isolation — 2026-09-27

## Scope

Implemented the approved correction for optional sensor I/O blocking foreground
JOG/STOP processing long enough to expire the existing 250 ms command lease.
The approximately one-second I2C error path is isolated, not shortened or hidden.
No serial port was opened, no firmware was flashed, and no powered run occurred.
Existing logs remain unmodified PRE- or POST-RELOCATION evidence as recorded.

## Ownership and execution

`src/sensor_worker.h` contains `m09::SensorWorker`. Its FreeRTOS task is pinned to
core 0, priority 1, with an 8192-byte stack. It yields one RTOS tick between
iterations. The foreground retains serial parsing, motor commands, manual and
POSE control, reference qualification, safety checks, and application telemetry.

The worker's private BNO and encoder objects exclusively perform all Wire/Wire1
initialization and transactions: bus probes, BNO initialization and reads, reset
consumption, report re-enabling, product-ID requests, and both AS5600 readers.
Thus Bus-B BNO and AS5600 transactions remain serialized by ownership, without a
foreground I2C mutex or foreground wait. AS5600 register reads, alternating
cadence, bus speed, and configured timeouts remain unchanged.

Setup starts motor control/watchdogs and launches sensor initialization
asynchronously. Manual READY does not wait for optional sensors. A worker-start
failure reports unavailable sensors and leaves sensor-independent manual control
available. The worker never holds motor pointers or changes manual/POSE state.
After explicit abort it stops starting sensor work once it observes the halted
context; an already-running transaction may finish first.

## Thread-safe handoff

`src/sensor_handoff.h` supplies bounded single-producer/single-consumer queues and
snapshot mailboxes. Queue indices use acquire/release atomics; payload slots are
exclusively owned while copied. Mailboxes try an atomic flag once and skip a
contended copy, retrying on subsequent iterations. Neither side spins, waits for
I2C, allocates memory, or invokes callbacks while copying a snapshot.

- Worker to foreground: 32-entry raw sample queue, 32-entry diagnostic event
  queue, and a snapshot containing encoder state, BNO availability, and counters.
- Foreground to worker: copied diagnostic motor/phase/JOG context and flags for
  query-idle and explicit abort. Query scheduling uses the latest available
  snapshot; an in-flight query can overlap a subsequent foreground state change.
- A separate atomic reset generation advances at every reset callback. Queue
  overflow cannot suppress orientation-reference invalidation.

Samples retain their acquisition receipt time and reset generation. Foreground
`serviceBno()` consumes results without I2C, applies the existing quaternion/BNO
ROLL conversion and authority rules, and updates freshness using receipt time.
Samples from a previous reset generation or already older than the existing
150 ms freshness bound cannot become fresh merely by being dequeued later.
The worker's persistent SH-2 event object also keeps the callback destination
valid between library calls.

Existing reset-cause, product-query, transport-duration/error, malformed-vector,
timestamp, and encoder diagnostics remain. `context_at_ms` identifies the time
of the copied motor context attached to a sensor event; it is not asserted to be
an instantaneous motor read inside the callback. `sample_handoff_dropped` counts
sample-queue overflow, and trace overflow remains counted. Application telemetry
is formatted/emitted by the foreground; existing framework I2C error printing is
not replaced. The pinned BNO library patches were not changed in this pass.

## Control semantics preserved

No changes to `manual_control.h`, `motion_watchdog.h`, motor settings, Python,
joystick mappings/arming, BNO report 0x05 at 100 Hz, POSE feedback, or sensor
authority were made in this pass. Yaw/pitch/carriage manual speed and acceleration
ceilings remain 2000/1200/2000, and Python default scale remains 1.0. BNO ROLL stays
the physical pitch feedback axis for POSE. M08 remains byte-for-byte unchanged.

Only valid processed commands renew the existing command lease. Bytes waiting in
UART do not renew it. STOP and centered JOG use their existing braking/disarming
semantics. No new travel limits, guards, sensor dependencies, or automatic moves
were added. Sensor stalls may still delay other sensor results, including the
shared-bus encoder; they no longer occupy foreground command processing.

## Validation

Production `platformio run -e esp32dev`: PASS. Static RAM 47,180 / 327,680 bytes;
flash 375,645 / 1,310,720 bytes. The task stack is a separate runtime allocation.
Complete `tests/run_host_tests.ps1`: PASS, including every existing C++ suite,
all 50 Python/headless/library-patch tests, and all 23 M08 baseline hashes.

The stall fixture executes the production worker body. A simulated blocking
sensor read yields CPU time to the real foreground loop; independent millisecond
ticks inject host commands and run watchdog checks while the read remains pending.
It does not refresh the lease directly or accept buffered bytes as commands.

| Injected scenario | Result |
| --- | --- |
| 1000 ms BNO stall, continuous valid JOG, change all three requests | Handled in 2 ms; axes remain active; no disarm/watchdog trip |
| Same stall, host commands stop | All three stop requests at 250 ms after last valid command |
| Same stall, STOP | Handled in 1 ms; all axes brake/stop; manual mode ends |
| Same stall, centered JOG | Handled in 2 ms; all axes stop; manual remains armed |
| 1000 ms Bus-B AS5600 stall, continuous JOG | Updated requests handled in 2 ms; no disarm |
| BNO stall, valid commands stop but invalid UART traffic continues | All three stop requests at 250 ms; bytes do not renew lease |

Additional checks cover 100,000 queue records and snapshot coherence between two
native threads, bounded overflow, no acquisition during foreground result
consumption, stale sample receipt times, pre-reset sample exclusion, and reset
invalidation despite an overflowing trace queue. Existing diagnostic callback,
product-cause/query, malformed-sample, and observer tests pass through the new
handoff. The observer fixture was adapted to dispatch the worker during host
simulation; its M08 branch remains unchanged. No remaining test failures.

These tests establish software behavior under the simulated scheduling model.
They do not measure actual ESP32 scheduling, electrical recovery, or physical
stopping time. A subsequent user-run powered test remains the hardware check.
Outputs: `.pio/worker_build.txt`, `.pio/worker_tests.txt`.

## Files changed in this pass

This list excludes pre-existing uncommitted changes from earlier passes.

- New firmware: `src/sensor_worker.h`, `src/sensor_handoff.h`,
  `src/bno_trace_types.h`.
- Firmware integration: `src/main.cpp`, `src/bno_lifecycle_diagnostics.h`.
- New tests: `tests/sensor_worker_stall_test.cpp`,
  `tests/sensor_handoff_test.cpp`, `tests/sensor_result_test.cpp`.
- Existing test integration: `tests/bno_trace_test.cpp`,
  `tests/run_host_tests.ps1`, `tests/stubs/Arduino.h`,
  `tests/stubs/Adafruit_BNO08x.h`, `tests/stubs/Wire.h`.
- Observer integration: `tools/bno_observer/main.cpp`.
- Documentation: `README.md`, `VALIDATION.md`, this report.
