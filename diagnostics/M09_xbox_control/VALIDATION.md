# M09 offline validation — 2026-09-26

## Current work: manual-first keyframes and optional-sensor independence

Restore baseline: `0467dfac3ce33df7dd67f8c9781e746669df3cf0`, tag
`M10_pre_workflow_redesign`, branch `main`. The complete pre-change working tree
was committed before editing. No push or post-change milestone tag was made.

Manual is now the default operator workflow. Centered sticks for 0.5 seconds
trigger the existing zero-JOG/acknowledgment handshake automatically. A/Space/F12
always STOP; B/keyboard X retains latched abort. Y and the AUTO toggle are removed.
LB/RB capture, controller X duration, RS increment, LS return and Home Play are
available directly from manual. Capturing brakes before requesting a fresh stopped
snapshot. Normal movement and control-state/BNO-quality changes preserve A/B.

Home performs STOP/snapshot, KEYRETURN to A if necessary, then a 200 ms stopped
observation and another correlated snapshot before timed KEYMOVE to B. The selected
5/10/20-second duration applies only to A-to-B. KEYRETURN uses native travel caps
with its own 60-second bound. Generated-step return verification never claims
measured-angle settling. A failed/stopped return or mismatched A cannot start B.

Stick displacement outside the existing deadband cancels automatic actions and
pending continuations. STOP/READY and the zero-JOG acknowledgment precede applying
the current live stick value; no centering or mode toggle is needed for takeover.
Explicit STOP/focus loss/F2 cancels takeover permission. Normal recovery requires
centered sticks. There is still one outstanding operation; busy presses do not queue.
F2 keeps exclusive raw-command ownership, and F3 remains presentation-only.

KEYMOVE/KEYRETURN no longer arm the BNO watchdog or depend on BNO freshness,
accuracy, pitch qualification, north, reset state or sensor-derived angle guards.
Motor initialization, stopped admission, epochs, signed count/delta bounds,
finite profiles, endpoints, deadlines and stop handling remain enforced.
The sensor-worker source and ownership architecture are unchanged.

Orientation POSE/MOVE still needs fresh plausible samples and its feedback watchdog,
but no longer requires a learned pitch baseline, accuracy >=2 or qualified north
just to admit/control a reported-angle target. Absolute POSE always holds yaw;
MOVE with zero yaw delta explicitly leaves yaw uncontrolled. Sensor quality remains
visible, not described as calibrated. Freshness, quaternion validation, heading
continuity, direction/progress, travel-angle limits and post-stop settling remain.
Historical samples are retained separately for display with age, accuracy,
availability, freshness and north qualification. Invalid vectors cannot replace
the retained sample. Old queued data cannot become fresh at consumption time.

Pitch manual/generated-step travel now has separate 2400 steps/s and 2400 steps/s²
caps (previously 1200/1200); yaw/carriage remain 2000/2000. POSE's 80 Hz bootstrap,
16-step first bursts, 250 steps/s² precision acceleration, correction/settling
logic and 1200/1200 angular slew settings are retained. Timed planning can use
capped acceleration when the preferred ramp would be infeasible, provided the
duration remains feasible. Rejections identify axis, duration, displacement, cap,
acceleration, minimum duration and speed/acceleration versus step-resolution cause.
The earlier id=13 rejection cannot be reconstructed without its displacement;
the new diagnostics distinguish those possible causes.

Verification: complete `tests/run_host_tests.ps1` passes, including all C++ cases,
all 111 Python tests and all 23 protected M08 hashes. Added coverage includes
manual capture/reposition, Play's independent return and playback durations,
joystick takeover/noise, failed return verification, BNO-independent motion,
stale/invalid retained samples, imperfect-quality POSE, faster travel and planner
feasibility. The expanded all-axis rejection case also passes independently.
Full output: `.pio/workflow_tests.txt`; compile output: `.pio/workflow_build.txt`.
The final ESP32 compile passes (47,420 bytes RAM, 384,281 bytes flash).
`git diff --check` passes. Earlier failures from obsolete arm/quality/rate test
expectations were updated; testing also exposed and fixed a startup handshake
race by arming on the transmit tick. No failures remain.

Files changed:
- Host: `xbox_control.py`, `auto_control.py`.
- Firmware: `src/main.cpp`, `src/manual_control.h`, `src/keyframe_math.h`,
  `src/keyframe_motion.h`.
- C++ checks: `tests/keyframe_math_test.cpp`, `tests/keyframe_motion_test.cpp`,
  `tests/m08_pose_integration_test.cpp`, `tests/manual_integration_test.cpp`,
  `tests/pitch_direction_diagnostics_test.cpp`, `tests/pose_restoration_test.cpp`,
  `tests/sensor_result_test.cpp`, `tests/sensor_worker_stall_test.cpp`.
- Python checks: `tests/test_auto_control.py`, `tests/test_auto_ui.py`,
  `tests/test_display_freeze_ui.py`, `tests/test_host_ui.py`,
  `tests/test_raw_command_ui.py`.
- Runner/docs: `tests/run_host_tests.ps1`, `README.md`, `VALIDATION.md`.

No serial hardware, upload or physical motion was used. Remaining physical limits:
higher pitch rates need torque/stop verification; counts remain startup-relative
and unhomed; missed steps/backlash are not measured; low-quality reported heading
is not qualified north; a broken serial link can prevent delivery of STOP during
finite motion. README contains the staged physical procedure. New firmware and
host must be deployed together later for KEYRETURN support.

## Previous host work: physical controller button mapping (2026-09-27)

Replaced only the unavailable AUTO duration/play bindings: controller X (button 2)
cycles duration and Center/Home (button 10) plays A to B. Y, LB/RB, stick clicks,
D-pad, A/STOP and B/latched abort retain their existing assignments. Keyboard X
still aborts; controller X does not. Displayed help and README distinguish them.
The event handling, release latches, one-request protocol, mode gates, captures,
serial ownership and firmware are unchanged.

Updated the AUTO event tests to use the actual buttons. Additional cases verify
X and Home release requirements, busy-press discard, no accidental X abort,
inactive old buttons, and inactive X/Home in manual mode. Existing cases exercise
the new bindings with raw mode, F3, STOP/abort priority and no AUTO JOG traffic.
The 25 focused AUTO tests pass. A headless screenshot of the updated cheat sheet
was inspected: `.pio/controller_mapping_ui.png`.

The complete `tests/run_host_tests.ps1` suite passes: all 108 Python tests,
all C++ unit/integration scenarios, and all 23 protected M08 hashes. Full output:
`.pio/controller_remap_tests.txt`. `git diff --check` passes. No failures remain.
No ESP32 rebuild was needed for this Python mapping-only change.

Files changed in this host-only pass: `xbox_control.py`, `auto_control.py`,
`tests/test_auto_control.py`, `tests/test_auto_ui.py`, `README.md`, `VALIDATION.md`.
Firmware source SHA-256 hashes match the pre-remapping snapshot. No real serial
port was opened, firmware flashed, hardware moved, milestone or Git tag created.

## Previous work: Xbox AUTO and timed step keyframes (2026-09-27)

Added stopped, request-correlated `SNAP` and finite three-axis `KEYMOVE` in an
independent generated-step operation. Captures/playback carry a boot/confidence
epoch; forced or uncertain stops invalidate old captures. Normal gentle STOP
preserves captured coordinates. The existing angular POSE/MOVE admission,
feedback and settling controller, manual JOG, and sensor worker remain in use.
Carriage acceleration is explicitly restored before subsequent POSE carriage
motion, so a preceding slow keyframe cannot change that existing operation.

The Xbox AUTO interface provides pitch/yaw MOVE increments, A/B capture,
5/10/20-second duration, return-to-A and deliberate A-to-B playback. No JOG is
emitted while AUTO is selected. Fresh/released controls, one outstanding request,
correlated snapshots, cancellation and timeout handling prevent repeated or
deferred button motions. F2 and F3 remain available; AUTO status stays live under
display freeze. Keyframe telemetry updates the current carriage display.

Focused firmware coverage includes independent profile timing/progress math and
16 operation scenarios: stopped snapshots, admission/malformed/epoch rejection,
configuration and partial-start failure, low-accuracy three-axis and two-axis
completion/reverse, zero motion, STOP/braking timeout, abort, stale/reset/pitch
guard, sensor-worker-stall STOP latency, and unexpected individual motor stop.
The completion scenario also exercises later relative carriage and pitch/carriage
POSE operations to check the original acceleration is retained. New headless
Python tests cover protocol correlation, capture validity, button release/busy
discard, mode changes, STOP/abort precedence, dry-run, and F3 behavior. Tests use
fake serial/controllers only. A simulated AUTO screenshot was inspected.

Full regression output is `.pio/keyframe_regression.txt`; compile-only output is
`.pio/keyframe_build.txt`. The complete suite passes: all C++ unit/integration
scenarios (including the new timing math and 16 keyframe cases), all 105 Python
tests, and all 23 protected M08 file hashes. `git diff --check` passes. No test
failures remain. PlatformIO's first sandboxed attempt could not write
its user cache lock; the authorized compile-only retry succeeded. The esp32dev
image uses 47,396 bytes RAM and 383,905 bytes flash. No serial port was opened,
firmware uploaded, physical motion performed, milestone created or Git tag made.

Runtime success means generated-step endpoints reached, with
`angle_settling=NOT_CHECKED`. Motors share a requested duration and start in one
foreground pass; ESP32 scheduling, actual pulse timing and physical repeatability
still need the supervised physical checks in README. Runtime endpoint timing
tolerance is max(500 ms, 10% of requested duration), not pulse-exact synchronization.

Files changed for this implementation:
`src/main.cpp`, `src/manual_control.h`, new `src/keyframe_math.h`,
new `src/keyframe_motion.h`, `xbox_control.py`, new `auto_control.py`,
`tests/run_host_tests.ps1`, `tests/stubs/Arduino.h`, `tests/stubs/FastAccelStepper.h`,
new `tests/keyframe_math_test.cpp`, new `tests/keyframe_motion_test.cpp`,
new `tests/test_auto_control.py`, new `tests/test_auto_ui.py`, `README.md`,
and `VALIDATION.md`. Earlier uncommitted POSE/F2/F3 changes are retained.

## Previous host work: F3 diagnostic display freeze (2026-09-27)

F3 snapshots only the diagnostic and scrolling response text. The yellow
DISPLAY FROZEN banner is visible in manual and raw-command modes; controls and
the raw editor remain live. Unfreezing redraws the latest processed state on the
same frame, without replaying accumulated responses.

`test_display_freeze_ui.py` uses real headless pygame rendering with simulated
serial/controller objects. It checks fixed values/receipt ages/response text,
background sensor state and logs, uninterrupted JOG timing and direction changes,
STATUS polling, ignored key repeats, immediate latest-state redraw, F2 editing
and submission, STOP/abort/focus loss/exit in both modes, and serial-free dry-run.
A simulated frozen raw-mode screenshot was inspected for banner/layout clarity.
The complete regression output is `.pio/display_freeze_tests.txt`.

`tests/run_host_tests.ps1` passes: all C++ unit/integration scenarios, all 83
Python tests (including four new freeze tests with safety subcases), and all 23
protected M08 file hashes. `git diff --check` passes.

Firmware source hashes match those before this display-only feature. No real
serial port was opened, no firmware changed or flashed, no physical motion
performed, and no milestone/tag created.

## Previous host work: raw command entry (2026-09-27)

The Python Xbox window now provides an F2 single-line editor through its existing
serial connection. STOP/READY disarming, persistent JOG suppression, exact raw
logging, reserved safety keys and current carriage telemetry are covered by new
offline tests in `test_raw_command_session.py` and `test_raw_command_ui.py`.
All pre-existing host/manual tests remain enabled. The complete regression output
is `.pio/raw_command_tests.txt`.

`tests/run_host_tests.ps1` passes: all C++ unit/integration scenarios, all 79
Python tests (including 27 new raw-command tests), and all 23 protected M08 file
hashes. `git diff --check` also passes.

Headless pygame checks use fake joystick and serial objects and assert a single
port construction; dry-run asserts none. They cover early Enter/no deferred send,
malformed/Unicode/overlong raw forwarding, Tab separators, Space/A/F12 STOP,
X/B abort, Esc/close/focus-loss precedence, exact logs, and deliberate rearming.
Session tests also cover a buffered READY before a pending STOP: transmitting
that STOP restores the wait for a subsequent READY. A simulated editor screenshot
was inspected for layout; it is not a powered test.

Firmware source hashes match those before this host feature. No firmware build
was needed for the Python-only feature; no serial port was opened, no ESP32
flashed, no physical test performed, and no milestone/tag created.

## Firmware work: targeted POSE restoration (2026-09-27)

See [POSE restoration](audit/POSE_RESTORATION_2026-09-27.md) for the current
feedback mapping, first-move limits, STOP behavior, timestamp checks and proposed
physical procedure. The current checkout remains M09; no new milestone was made.

Complete offline regression suite and ESP32 compile-only build pass. Added 13
POSE restoration scenarios and 7 POSE STOP scenarios; all 52 Python tests pass.
All 23 protected M08 files are unchanged. Output is in
`.pio/pose_restore_tests.txt` and `.pio/pose_restore_build.txt`.

No serial connection, firmware upload or physical POSE validation occurred.
Earlier records below describe their respective historical builds, including
the now-superseded BNO roll feedback assumption.

## Previous build: temporary pitch-direction diagnostics (2026-09-27)

- ESP32 `esp32dev` build PASS: static RAM 47,204 / 327,680 bytes; flash
  376,473 / 1,310,720 bytes.
- Full `tests/run_host_tests.ps1` PASS: all existing C++ suites, new pitch-direction
  diagnostics test, all 50 Python/headless/library-patch tests, all 23 M08 hashes.
- New checks: both signed commands and forward/backward selections; actual return
  codes including rejected starts; independent GPIO latch readback (deliberately
  inconsistent with the command); reversal, zero/STOP and stopped state; delayed
  DIR checks; no held-command chatter; bounded output with unavailable UART space.
- Existing one-second sensor-stall cases still pass: JOG/center 2 ms, STOP 1 ms,
  true host loss and invalid-only traffic stop requests at 250 ms, encoder stall
  JOG 2 ms. These are simulation times, not powered measurements.
- No remaining failures. Initial test compilation exposed mixed `auto` types;
  production compilation exposed a scoped library return enum; both corrected
  before the final complete suite/build.
- Outputs: `.pio/pitch_direction_build.txt`, `.pio/pitch_direction_tests.txt`.
  Firmware `.pio/build/esp32dev/firmware.bin` SHA256:
  `a78913b6f33f20ec48451e71446e95fb40432c764cfa63c01601ecf741551aaa`.

Changed in this pass only: new `src/pitch_direction_diagnostics.h` and
`tests/pitch_direction_diagnostics_test.cpp`; hooks in `src/manual_control.h`;
ESP32 register includes in `src/main.cpp`; independent GPIO fixture variable in
`tests/stubs/Arduino.h`; `tests/run_host_tests.ps1`; README and this record.
No Python, worker, watchdog, settings, pin/sign, sensor-authority or library changes.
No serial connection, flash, or powered test. GPIO readback is explicitly an
output latch, not an external driver-input voltage measurement. Test procedure
and field definitions are in README's temporary pitch diagnostics section.

## Previous build: sensor-worker isolation (2026-09-27)

Implementation, ownership, handoff details, exact file list, and evidence limits:
[Sensor-worker isolation](audit/SENSOR_WORKER_ISOLATION_2026-09-27.md).

- ESP32 `esp32dev` build PASS: static RAM 47,180 / 327,680 bytes; flash
  375,645 / 1,310,720 bytes. The worker additionally allocates an 8 KiB task stack.
- Complete `tests/run_host_tests.ps1` PASS: all existing C++ units/scenarios,
  new worker-stall/handoff/sample-age/reset tests, and all 50 Python/headless/
  library-patch tests. All 23 M08 baseline file hashes PASS.
- One-second simulated BNO stall: changing JOG and centered JOG handled in 2 ms;
  STOP handled in 1 ms. Continuous JOG does not disarm manual control.
- Host loss during the same stall stops all three axes at 250 ms. Continuing
  invalid UART traffic does not renew that lease. A one-second Bus-B encoder
  stall also permits JOG updates in 2 ms.
- Concurrent native-thread handoff test passes 100,000 coherent records;
  stale/pre-reset samples and trace-queue overflow cannot hide reset invalidation.
- Output: `.pio/worker_build.txt`, `.pio/worker_tests.txt`; binary:
  `.pio/build/esp32dev/firmware.bin`. No remaining validation failures.

No serial port opened, firmware flashed, or powered test performed. The reported
latencies are simulation results, not measurements of ESP32 scheduling or physical
stopping time. Motion settings, Python scale 1.0, mappings, sensor authority,
report type/rate, and manual/watchdog semantics are preserved.

## Previous build: relocation comparison diagnostics (2026-09-27)

Scope, log-field definitions, evidence boundaries, and limitations:
[BNO relocation diagnostics](audit/BNO_RELOCATION_DIAGNOSTICS_2026-09-27.md).
Both existing powered logs are PRE-RELOCATION. The next powered evidence is
POST-RELOCATION; no improvement is assumed and no powered run was performed here.

- ESP32 `esp32dev` build PASS: RAM 39,580 / 327,680 bytes; flash
  372,273 / 1,310,720 bytes. Firmware symbol inspection confirms all diagnostic
  callback hooks and the asynchronous product-ID request are linked.
- Complete `tests/run_host_tests.ps1` PASS: encoder/math/watchdog/observer/BNO
  units and all lifecycle/manual/carriage/POSE/M08 scenarios, including the new
  trace callback, query scheduling, unavailable response, queue pressure and
  malformed-sample checks.
- All 50 Python/headless UI/library-patch tests PASS. The library tests compile
  actual patched routines to check receive timestamps, signed report offsets,
  unsigned clock rollover, failed reads, individual reset callbacks, product-ID
  requests/responses, unknown reset causes and truncated response exclusion.
- All 23 M08 baseline file hashes PASS; `git diff --check` PASS.
- Build output: `.pio/build_diagnostics_results.txt`; complete host-suite output:
  `.pio/host_diagnostics_results.txt`; binary: `.pio/build/esp32dev/firmware.bin`.
- Initial development checks caught an ambiguous patch anchor and a display-text
  compatibility failure; both were corrected before this complete final run.
  No remaining validation failures. An initial sandbox cache-access build error
  was resolved by running the authorized offline build with cache access.

No flash, serial connection, motor action, speed/acceleration/joystick/safety change
or BNO mounting transform. Python default scale remains 1.0; report 0x05 remains
100 Hz. The old logs are unmodified; their hashes are recorded in the linked audit.

## Previous build: persistent logging and raw BNO diagnostics

This is the diagnostics-only follow-up. The user confirmed the earlier apparent
failure was an old firmware image and reported working manual motion after their
upload. This pass did not reopen that investigation, change motion control, upload
firmware, open live serial, or exercise hardware.

The user's newer yaw/pitch/carriage ceilings of 2000/1200/2000 pulses/s, matching
accelerations, and Python runtime scale default 1.0 are preserved. Existing manual
watchdog, STOP/rearm, explicit abort, pin/sign mapping and POSE math are unchanged.

Files changed **in this pass**, separate from the previous uncommitted work:

- `session_log.py` (new), `xbox_control.py`, `.gitignore`: per-session timestamped,
  flushed text logs, raw/parsed telemetry, sampled commands and immediate state/
  sensor transitions, exception tracebacks, visible log path and `--log-dir`.
  Pygame display retained and enlarged for the additional diagnostic rows.
- `src/bno_diagnostics.h` (new), `src/main.cpp`: pre-filter raw BNO snapshots,
  observed report/status counters, raw/accepted timestamps, quaternion components,
  separate categorical status and radian accuracy, resulting Euler values and
  rejection reason. Added `has_sample` and telemetry enqueue-drop count. Sensor
  acquisition/acceptance, quaternion math and manual motion behavior unchanged.
- `tests/bno_diagnostics_test.cpp` and `tests/test_session_log.py` (new): focused
  instrumentation, math interpretation, file persistence/sampling/error tests.
- `tests/stubs/Adafruit_BNO08x.h`, `tests/test_host_ui.py`,
  `tests/test_xbox_control.py`, `tests/run_host_tests.ps1`: realistic decoded report
  metadata and integration coverage for raw/accepted status, logs and serial errors.
- `tests/manual_integration_test.cpp`, `tests/manual_carriage_test.cpp`,
  `tests/pose_math_test.cpp`: replace obsolete hard-coded rate/acceleration
  expectations with the user's current settings; adjust the near-deadline math
  fixture so it still tests acceleration overhead at the increased acceleration.
  Python's default-value test now expects the user's 1.0. Production settings were
  not modified to make these tests pass.
- README, this record, and
  `audit/BNO_PATH_AND_MANUAL_LIMITS_2026-09-26.md`: current instructions, complete BNO
  path, manual restriction inventory, NORTH_LEVEL status and diagnostic limitations.

Final `platformio run`: **PASS**, RAM 35,372 / 327,680 bytes (10.8%), flash
364,413 / 1,310,720 bytes (27.8%). Image `.pio/build/esp32dev/firmware.bin` SHA-256:
`e8a286029beb49e694188c1c91825c029d5cc45fe0c4e6d012c0ff034c3d4968`.

Final `tests/run_host_tests.ps1`: **PASS**, exit 0:

- 72 current M09 scenarios: 20 lifecycle, 26 manual, 9 carriage, 17 BNO POSE.
- 65 historical M08 scenarios against unchanged M08 source.
- 43 Python tests, including new per-run/flush/traceback, log failure isolation,
  sampled command edges, raw/parsed sensor transitions and serial-failure logging.
- 31 new BNO diagnostic checks: known quaternion rotations, q/-q and scaling,
  raw/accepted categorical status, separate radian accuracy, ignored report type,
  invalid input/reason, timestamps/sequences, stale data and optional queue drops.
- Existing standalone encoders, 85 control-math checks, 361 POSE-math checks,
  independent watchdog and sensor-only observer all pass.
- All 23 M08 snapshot hashes unchanged; no remaining build/test failures.

Logs: `.pio/diagnostics_full_tests.txt` and `.pio/python_diagnostics_tests.txt`.
Initial failures were stale test expectations for the user's increased settings
and a host-compiler zero-initializer warning in the new diagnostic fixture; both
were corrected without altering motor tuning or control behavior.

Still requires powered evidence: actual quaternion/frame alignment, origin of BNO
status changes, real sensor/transport and filesystem timing, raw encoder readings,
and command/STOP behavior at the user's higher speeds. No quaternion math fix was
justified offline. Telemetry is sampled, library callbacks may coalesce reports,
and file flushing does not guarantee survival of OS/power failure. No North + Level
operation or new guard was implemented.

## Previous build: AS5600 acquisition and approved manual simplification

Implemented only in M09. No serial port was opened, no upload was performed, and
no powered hardware was exercised for this change. The board still has its prior
firmware. All 23 files in the M08 baseline snapshot remain byte-for-byte unchanged
(excluding build artifacts).

### Changes

- `src/encoder_acquisition.h`: restored read-only acquisition from both AS5600s
  at address 0x36, using Bus A SDA18/SCL19 and Bus B SDA4/SCL5. Each scheduled call
  reads one device's STATUS and RAW_ANGLE; calls alternate at a minimum 10 ms
  interval. Availability, validity, age, raw angle and magnet status are reported.
  No calibration writes, inferred axis conversion, encoder motor feedback or
  encoder-triggered shutdown was added. The reader has its own standalone test,
  run before the integrated simplification tests.
- `src/main.cpp`: motor setup and the existing command watchdog establish manual
  readiness after one bounded optional sensor setup attempt. Removed the fixed
  application startup waits, calibration readiness gate and automatic startup
  movement. Pitch/north references can qualify from healthy samples while idle.
  BNO loss invalidates dependent references and cancels an affected angular
  operation recoverably; it does not disable manual control or carriage-only MOVE.
  Added periodic BNO/encoder health telemetry during idle and manual operation.
- `tools/patch_sh2_timeout.py` and `platformio.ini`: reproducibly bound the pinned
  SH2 product-ID operation to one second. The shared SH2 operation loop, BNO
  rotation-vector report, 300 ms reset wait and calibration handling are unchanged.
  This is an operation deadline, not a measured total boot-time guarantee.
- `src/manual_control.h`: retained two-/three-field JOG, mappings, direction signs,
  speed/acceleration limits, braking, reversal and stopped observation. Removed
  manual dependence on BNO freshness/accuracy, BNO-derived direction/progress and
  angular/margin guards, the 90-second manual duration guard, and the arbitrary
  +/-500 generated-step carriage window. Carriage uses continuous velocity calls.
  Axis API/unexpected-stop/braking failures stop and report the affected axis;
  a later valid JOG can retry without a new axis fault latch.
- `src/motion_watchdog.h`: kept the existing independent 250 ms valid-command
  deadline and added stopped-state recovery using the same timer. Timeout clears
  continuous requests and allows a new centered arm. Callback/rearm/refresh state
  is serialized so an expired command interval is not overwritten by a late
  packet. The timer remains active until STOP/zero has reached actual motor
  braking. Explicit operator X remains a latched abort.
- `xbox_control.py`: default speed scale now matches the documented 0.25;
  `--speed-scale 1` is retained. Existing stick mapping, deadband, velocity curve,
  inversion options and 20 ms JOG stream are unchanged. Receive-only telemetry
  gaps warn without stopping live commands. Routine session/input failures use
  STOP and centered rearm, without automatically sending X. A command gap reaching
  the existing 250 ms deadline disarms before motion can resume. The UI reports
  BNO and encoder health/receipt age; malformed telemetry is discarded without
  interrupting the command stream. Unrelated telemetry age no longer blocks
  centered arming; live motion still requires the zero-JOG acknowledgement.
- Test fixtures, stubs and runner cover the new behavior. The sensor-only observer
  follows the new idle polling path and still cannot dispatch movement commands.
  README documents the current operation and a controlled future hardware check.

BNO heading/ROLL remains the feedback for POSE, with the verified physical pitch
mapping and existing automatic convergence/settling rules. North still requires
the existing accuracy/stability qualification; accuracy 0/1 is not relabeled as
calibrated. No replacement carriage limit, homing, encoder-to-axis calibration or
new watchdog was introduced.

The local `src/control_math.h` settings `ACCELERATION=250` and
`PITCH_PULSES_PER_ERROR_DEG=24.0` predated this implementation and were preserved.
An old math fixture's hard-coded 240 expectation was updated to use the current
constant; firmware tuning was not changed to satisfy that test.

### Results

`platformio run`: **PASS**, compile only for `esp32dev`.

- RAM: 35,236 / 327,680 bytes (10.8%).
- Flash: 362,421 / 1,310,720 bytes (27.7%).
- Firmware: `.pio/build/esp32dev/firmware.bin`.
- SHA-256: `b4e40d7f39dd8a337e2f58fd260c306f517bef883397b04919f089cb900d2e9d`.

`tests/run_host_tests.ps1`: **PASS**, exit code 0.

| Offline suite | Result |
| --- | --- |
| Standalone AS5600 reader | Pass: register reads, status, cadence and isolated failures |
| Control and POSE math | Pass: 85 and 361 checks |
| Independent watchdog and sensor-only observer | Pass, including stopped recovery and no-motion observer |
| M09 startup/sensor lifecycle | 20 scenarios pass |
| M09 manual control | 26 scenarios pass |
| M09 manual carriage | 9 scenarios pass |
| M09 preserved BNO POSE controller | 17 scenarios pass |
| Historical M08 startup and pitch readiness | 40 + 25 scenarios pass against unchanged M08 source |
| Python protocol, headless UI and dependency patch | 35 tests pass |
| M08 SHA-256 snapshot | All 23 baseline files unchanged |

The 72 current M09 scenarios cover sensor-independent manual operation, per-device
encoder failure, command-loss stop/rearm, blocked foreground work, individual axis
errors, carriage beyond the former window, manual operation beyond 90 seconds,
explicit abort, BNO-only POSE failure and missing timing evidence. Historical M08
startup tests are clearly separate; they are not evidence that M09 still performs
the removed automatic startup movement. Raw output is in the ignored local
`.pio/host_test_results.txt` build/test artifact. The final Python-only arming
correction was checked with the full 35-test Python suite; its output is in
`.pio/python_test_results.txt`. Firmware was unchanged by that final correction.

### Remaining limitations and failures

No remaining offline test or build failures. These tests simulate sensors and motor
behavior; they do not establish electrical bus timing, physical encoder attachment,
mechanical direction, stopping distance or powered operation.

Removing automatic startup travel also removes the initial angular timing evidence
normally used by coordinated POSE. Required missing timing is explicitly rejected,
with manual still available. Full coordinated POSE regression fixtures inject
synthetic timing evidence only in the test harness; no firmware calibration or
first-POSE bootstrap was invented. Native pitch-only control remains available
when its pitch reference is usable and north is unavailable.

The old `.venv` points to an unavailable Python 3.6 installation. The passing runner
uses PlatformIO's Python and the existing `.pio/python_deps` packages; README gives
the corresponding dry-run command. That unrelated environment was not rebuilt.

Both encoders remain acquisition/telemetry only. Carriage remains unhomed and has
no software rail endpoint. The first powered test must be supervised; this change
has not been flashed. See README for the controlled test sequence.

## Historical validation — prior firmware, not the current build

The records below are preserved as history. Earlier limits, startup behavior,
upload claims and binary hashes describe earlier versions only.

### Manual carriage extension — 2026-09-13

Added left-stick horizontal/raw axis 0 through the existing deadband, quadratic
velocity curve, speed scale and 20 ms stream. Right horizontal yaw/raw axis 2 and
left vertical pitch/raw axis 1 are unchanged. `--invert-carriage` reverses only
carriage; `--carriage-axis` permits explicit index selection. Arming requires all
three assigned inputs centered. The host now sends `JOG yaw pitch carriage`;
firmware still accepts two-value JOG as a zero-carriage request.

Production changes are limited to `xbox_control.py`, the extra manual state and
abort-request clearing in `src/main.cpp`, and `src/manual_control.h` protocol,
carriage service, STOP completion and telemetry. Carriage retains DIR21/STEP22,
1000 pulses/s, 1000 pulses/s² and the +/-500 startup-relative generated-step
envelope. At default host scale its maximum requested rate is 250 pulses/s.
It uses a finite boundary target with live speed updates, so it decelerates at
the endpoint without repeated move commands. Release/STOP/reversal use normal
deceleration and stopped observation; when already near an endpoint the existing
finite braking target is retained, because FastAccelStepper stopMove rewrites
that target. A boundary STOP regression reproduced overshoot in the simulated
plant before this correction and now passes in both directions.

The existing watchdogs already include carriage; their implementation is unchanged.
STOP waits for all three motors. Manual still accepts accuracy 0–3; north validity,
absolute POSE/MOVE admission, BNO report selection, signs and calibration are unchanged.
Carriage has no physical position feedback or homing; generated-step limits do not
measure actual rail clearance or missed steps.

- ESP32 build: PASS, RAM 35,180 bytes, flash 362,857 bytes.
- All 95 inherited firmware scenarios, math/watchdog tests and 41 yaw/pitch manual
  scenarios: PASS.
- 21 new carriage scenarios: PASS. Covers independent/simultaneous axes, rate
  changes, release/reversal, STOP, endpoints, near-endpoint STOP at multiple speeds,
  rearming without recentering travel, legacy JOG, malformed commands, stale/invalid/
  wrong-report BNO, blocked BNO, reset, command loss, abort and remaining guards.
- 18 Python tests: PASS, including actual headless host carriage mapping/inversion,
  centered-input admission, disconnect/focus/pause abort and no-serial dry run.
- Python bytecode check and all 23 M08 baseline file hashes: PASS.

New fixture: `tests/manual_carriage_test.cpp`; updated runner and host tests in
`tests/run_host_tests.ps1`, `tests/test_xbox_control.py`, `tests/test_host_ui.py`.
README now describes the three-axis mapping, protocol and controlled first test.
The initial carriage-extension turn performed no upload or hardware action.
The user subsequently requested flashing after the old two-value firmware rejected
the updated client's third JOG value. The matching carriage image was built and
flashed to COM9 with esptool hash verification. At 2026-09-14 02:48 UTC, the real
`xbox_control.ManualSession` generated 32 zero-velocity `JOG 0 0 0` frames accepted
by the board. Three-axis telemetry showed zero commands/rates and carriage steps
0, followed by `MANUAL STOPPED` and `M09 READY` after STOP. COM9 closed with manual
control stopped/disarmed. No nonzero manual commands were sent during this check.
Raw evidence: [JOG protocol capture](audit/JOG_PROTOCOL_20260914T024831Z.log).
The existing JOG protocol was extended with an optional carriage field; no new
MANUAL command was introduced. Build image SHA-256:
`30ae17fea6c59bf7ade544a0b7a24ad2c47a898893233420b4f84d4806885e83`.

## Earlier validation history

Host display follow-up: the user's screenshot showed BASELINE with zero-initialized
error fields, but accuracy/freshness were clipped offscreen. The host now wraps
messages and keeps phase, last BNO accuracy/freshness/sample age, report receipt
age and pitch-only readiness visible. Repeated BUSY polls no longer displace
diagnostic messages. All 16 Python tests and bytecode compilation pass after this
display change. All 23 M08 files still match their hashes. Firmware is unchanged
by this follow-up. Subsequent screenshots identified fresh BNO data with accuracy
0 as the north-readiness blocker; the hardware audit below tested that finding.

The user identifies the earlier M08 hardware behavior as tested and correct.
Initial implementation and UI validation used no hardware. The user subsequently
authorized COM9 access and firmware A/B uploads, confirming powered motors and
clear startup travel. The [hardware audit](audit/HARDWARE_RESULTS.md) records those
tests separately. Powered Xbox motion remains unverified; no commit or push was
performed.

## Authorized BNO startup audit follow-up

- Stock M08 and M09 were captured twice each with matching serial/reset options.
  All four reported accuracy 0 throughout baseline telemetry and pitch-only
  readiness around 21.3 seconds after reset.
- Separate sensor-only wrappers using each stock setup built successfully and
  passed simulated no-motion/abort checks. In 60-second physical captures both
  reached accuracy 1, neither reached 2 or 3.
- Opening COM9 using the Xbox host's default RTS/DTR states produced a fresh M09
  boot banner without an explicit reset command.
- Actual-source synthetic timing tests confirmed the same 20-second deadline
  and late-calibration lockout in both versions.
- Audit tooling, fixtures and timestamped logs were added under `tools/`,
  `tests/` and `audit/`. Production firmware was not edited during this audit.
  Normal M09 was restored after diagnostic uploads. All 23 M08 hashes still pass.

## Manual-only accuracy prerequisite change

At the user's request, `src/manual_control.h` now admits JOG and reports manual
READY after either completed startup path, without requiring north or accuracy
>=2. Manual operations clear only their own yaw/accuracy requirements. Absolute
POSE/MOVE admission and north validity are unchanged. Without north, the manual
stopping margin uses the existing pitch-only startup yaw reference, matching the
unchanged hard travel guard. `xbox_control.py` display text reflects manual 0–3
acceptance. No `src/main.cpp`, shared safety code or M08 source was edited.

Validation: ESP32 build passes (RAM 35,124; flash 361,685 bytes), all 95 inherited
firmware scenarios, math/watchdog tests, 41 manual scenarios and 16 Python tests
pass. Manual tests cover arming/moving at each accuracy 0–3, accuracy transitions,
unverified-north POSE/carriage rejection, and low-accuracy stale/invalid/wrong-report,
reset, input-loss, abort, direction/progress and travel protections. M08's 23-file
preservation check passes. These are simulated motion checks; physical JOG motion
has not been exercised by the agent.

## Initial implementation results

- ESP32 `esp32dev` build: PASS with unchanged pinned dependencies/configuration.
  RAM: 35,124 / 327,680 bytes. Flash: 361,693 / 1,310,720 bytes.
- All 95 inherited firmware scenarios: PASS (40 startup/control, 30 POSE/MOVE,
  25 pitch-readiness). Includes carriage, BNO roll, pitch sign -1, stale/invalid/
  reset, blocked I2C/UART, abort and closed-loop settling cases.
- Original 85 control-math checks, 362 pose-math checks and watchdog tests: PASS.
- 29 new manual firmware scenarios: PASS. Covers centered/no motion, admission,
  readiness, correct independent axes/signs, rates/ceilings/acceleration,
  release/reversal, busy rejection, and POSE/carriage after manual exit.
- Fault cases: PASS for host loss, partial/malformed/late commands, blocked host,
  stale/invalid/wrong BNO reports, blocked BNO calls, accuracy drop, reset, X inside
  a partial line, wrong direction, no progress, angular guards, braking/overall
  deadlines and rejected speed settings.
- 16 Python tests: PASS. Eleven cover deadband/curve, zero release, signs, bounds,
  handshake, deliberate arming/rearming, telemetry loss, reset and fault handling.
  Five run the actual pygame host with dummy video/audio and mocked Xbox/serial,
  checking dry run never opens serial, motion/release/exit, disconnect, focus loss
  and an input-pump stall.
- Python bytecode compilation and `--help`: PASS.
- All 23 M08 files match their pre-edit SHA-256 hashes (excluding build artifacts).
  Copied control/sensor/watchdog/pose math and PlatformIO configuration also match
  M08 byte for byte. Git reports no M08 or shared hardware-header modifications.

The stalled-input UI test found that checking elapsed time only at loop entry
could permit stale input after a blocked input pump. The final host also checks
after input acquisition and before transmission. That regression now aborts
without replaying stale motion commands.

## Reproduction and limits

See README for environment setup. Development used PlatformIO Python 3.11,
pygame 2.6.1 and pyserial 3.5 installed into ignored `.pio/python_deps`, plus
Strawberry GCC with strict C++11 warnings as errors.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests\run_host_tests.ps1
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
$env:PYTHONPATH = (Resolve-Path .pio\python_deps).Path
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m unittest discover -s tests -p 'test_*.py' -v
powershell -NoProfile -ExecutionPolicy Bypass -File tests\verify_m08_baseline.ps1
```

Firmware test output: ignored `.pio/host_test_results.txt`. Compiled firmware:
`.pio/build/esp32dev/firmware.bin`.

Simulated watchdog ticks and idealized acceleration-limited motors test decisions.
The 150/250 ms assertions measure stop-request time in the fixture, not physical
stop latency. Real ESP32 scheduling, motor queues, gearing, torque, backlash,
magnetic conditions and joystick driver mapping need the README powered procedure.
Tiny manual rates can fail progress deadlines if BNO cannot resolve motion; no
limits were relaxed to avoid this.

The installed FastAccelStepper 1.2.7 header was inspected for rate-update and stop
semantics. The upstream [FastAccelStepper API](https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md)
also documents applying changed rates to an active ramp and normal deceleration.
Input architecture was checked against historical source and
[pygame documentation](https://www.pygame.org/docs/ref/sdl2_controller.html).
M09 retains the historical raw joystick API with configurable indices; actual
mapping is explicitly left for the no-serial dry-run check.
