# Targeted POSE restoration — 2026-09-27

Implemented the five authorized fixes in the attached M09 checkout. The request
referred to M10, but no M10 workspace exists here; no rename, milestone, tag or
checkpoint was created. Sensor-worker ownership, hardware stepping, manual
motion, manual lease, motor signs and unhomed carriage representation remain.
No ESP32 upload or live serial access was performed. Physical validation is pending.

## Changes

1. `physicalPitch()` selects BNO Euler pitch throughout POSE reference
   qualification, target/error, guards, velocity, settling, timing and telemetry.
   The powered log at
   `audit/milestones/M09_manual_sensor_worker_verified_2026-09-27/powered.log`,
   lines 256/264, records isolated positive pitch JOG: Euler pitch
   -9.988755° -> -1.397953°, roll 3.570875° -> 3.536259°. A regression converts
   those recorded quaternions independently of the synthetic motor model.
   Positive physical-angle error still selects negative steps/backward/LOW;
   no motor direction convention or quaternion conversion formula changed.
2. Missing timing no longer causes circular first-POSE rejection. After all
   existing feedback/reference checks, a requested change within +/-3° on each
   enabled angular axis can use finite precision corrections at <=80 pulses/s,
   <=16 steps/burst and acceleration 250 pulses/s². Continuous slew is disabled.
   Carriage speed is <=80 pulses/s for this fallback and its finite duration is
   checked. No gearbox ratio is invented. Timing is learned only on successful
   stopped completion; larger changes without timing still reject. Low-accuracy
   pitch-only admission is retained; yaw still requires qualified north/status>=2.
3. Ordinary STOP latches POSE cancellation before issuing `stopMove()` to all
   moving motors, discards pending carriage motion and prevents new corrections.
   BNO watchdog is disarmed after braking requests have been issued; cancellation
   polls motor state independently of sensor results. At 3 seconds, failed braking
   falls back to force-stop. READY returns only after all motors report stopped.
   Repeated STOP cannot extend the deadline; X remains a latched abort. The existing
   recoverable summary reports STOPPED/OPERATION FAILED for the uncompleted target,
   followed by explicit `POSE CANCELLED`. No canceled target automatically resumes.
4. A yaw target with qualified feedback remains required even when initially
   inside +/-0.4°. It is corrected if it drifts outside tolerance and checked at
   completion. A pitch-only move that regains an already-qualified magnetic
   reference stops recoverably if measured yaw misses its requested tolerance;
   it does not silently enable previously disabled yaw movement or report PASS.
5. Feedback ordering, velocity, progress timestamps, reference-window span and
   settling use worker receipt time. Duplicate/backward receipts are rejected.
   POSE ignores pre-command feedback for control decisions, and post-stop
   corrections require receipt >= stopped-observation time +100 ms. Settling
   also requires all motors stopped and post-stop samples, then >=1 second and
   >=30 samples with gaps <=100 ms. Motor timeout clocks remain wall time.

`BNO_STATE` now includes `physical_pitch` and `pitch_axis=PITCH`. Raw
`pitch_roll` and BNO_RAW Euler values remain honestly labeled. The Python display
prefers the new physical-pitch fields and retains compatibility with older logs
and firmware. No joystick mapping, rates, arming or command-stream behavior changed.

The worker, bus acquisition, watchdog implementation and sensor authority were
not redesigned. Freshness remains <150 ms, with reference requalification after
stale/reset events. AS5600 values remain telemetry, not POSE feedback. Carriage
remains a generated signed step count from arbitrary startup zero: no homing,
endstops, absolute carriage feedback or rail-length assumptions were added.

## Offline verification

`tests/run_host_tests.ps1` passes the complete existing suite plus:

- 13 restoration cases: first angular POSE without injected timing, first
  carriage coordination, pitch-only fallback, fallback bounds, sensor/reference
  admission, initially satisfied yaw correction, recovered-yaw rejection,
  receipt-based velocity, post-stop ordering, receipt-based settling,
  duplicate/backward receipts, millis rollover, and reference receipt span.
- 7 STOP cases: angular, finite carriage, pending carriage, repeated STOP,
  failed braking, STOP during a 1000 ms BNO worker stall, and X during braking.
  Tests assert no resumed motion and normal manual rearm. Stall STOP handling
  is within 20 ms in the deterministic fixture, not a measured ESP32 guarantee.
- Recorded quaternion mapping evidence, physical pitch qualification/limits/
  control/timing/telemetry, and Python display/legacy compatibility checks.
- All 52 Python tests and all 23 M08 baseline hashes pass. Historical M08 tests
  retain their original roll-mounted simulated sensor; current tests use the
  corrected pitch mounting. This fixture distinction does not claim new hardware
  calibration or validation.

ESP32 `platformio run -e esp32dev` passes (compile only). Build/test output:
`.pio/pose_restore_build.txt`, `.pio/pose_restore_tests.txt`.

## Proposed physical test — after a deliberate later upload

1. Provide clearance and an immediately reachable physical stop. Use a single
   logging serial command interface at 115200 baud; do not keep the Xbox client
   armed or competing for the port. Opening serial may reset this board. Wait for
   stopped READY, `fresh=YES`, `pitch_axis=PITCH`, and the pitch reference message.
   For yaw tests also require the qualified magnetic reference message and
   `north_usable=YES`/accuracy>=2. A successful manual session alone does not prove
   those references are ready. Do not defeat admission rejection.
2. Before autonomous pitch, use the already-working low-speed manual control for
   a very small pitch movement and confirm `physical_pitch` follows physical
   pitch with the expected sign. STOP/disarm and wait for READY. This verifies
   feedback selection on the current mounting, not the resolved driver issue.
3. Record fresh heading H, physical pitch P and current `POSE_STATE carriage_steps`
   C while stopped. Substitute actual numbers into `POSE H P C`; expect no motion.
   Never substitute carriage 0 unless C actually is 0.
4. Send `POSE (H+1) P C`, with yaw wrapped modulo 360°. Expect increasing heading,
   small finite motion, stopped/settled PASS and READY. Return with `POSE H P C`.
   Issue one command at a time and record actual start/end feedback and counts.
5. Refresh H/P/C after stopping. Send `POSE H (P+1) C`; confirm physical pitch
   and its selected feedback both increase. Wait for stopped/settled PASS, then
   return with `POSE H P C`. Completion tolerance is +/-0.4°; corrections normally
   approach the 0.1° HOLD entry threshold. Both sequences leave carriage at C.
6. During another small move, send STOP. Confirm `POSE STOPPING`, actual braking,
   `POSE CANCELLED` and READY without renewed motion. Reissue a target explicitly
   only after inspecting stopped state. X is the existing latched abort; host
   disconnect alone is not a POSE stop. The manual 250 ms lease does not govern
   finite POSE commands.
7. Only after those checks, optionally use a nearby carriage count with visible
   clearance, such as C+10, then return to C. This verifies generated pulses,
   not a homed rail position. Defer longer/simultaneous travel and milestone
   creation until the operator accepts physical results.

If feedback moves away from the target, does not respond to visible motion,
becomes stale, or the operation rejects/fails, stop and inspect the recorded
reason. Do not increase travel to overcome a failed first test. No sensor wiring
changes or induced powered I2C fault are needed for this first procedure.
