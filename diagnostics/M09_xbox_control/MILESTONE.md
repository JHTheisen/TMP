# M09 checkpoint: verified manual control with sensor-worker isolation

Identifier/tag: **M09_manual_sensor_worker_verified_2026-09-27**.
This freezes the current M09 source before any new POSE work, following the
project's numbered milestone-directory and Git-commit convention. It includes
the existing temporary pitch-direction diagnostics; no functional change was
made to create this checkpoint.

## Powered verification

The operator reports physically verified yaw, pitch, and carriage Xbox manual
motion, and continued manual responsiveness through BNO acquisition stalls.
Sensor-worker isolation is implemented and physically exercised. The current
[POST-RELOCATION powered log](audit/milestones/M09_manual_sensor_worker_verified_2026-09-27/powered.log)
is an unchanged copy of `logs/m09_20260927T112840.129476Z_31248_0.log`.

- At board time 25145 ms the log records a 1,009,618 us BNO acquisition stall
  while manual mode remains active; subsequent commands/motion continue.
- Positive pitch selects `runBackward` and later GPIO26 output-latch LOW;
  negative pitch selects `runForward` and later output-latch HIGH, with successful
  return codes. Examples: log lines 236/238 and 289/296.
- The pitch software/GPIO output path is verified. These are output-latch
  readings, not driver-terminal voltage measurements. The intermittent physical
  one-direction event has no proven software root cause; no corrective software
  change was made for it. The diagnostic logging remains in this checkpoint.
- The log ends with the existing explicit operator X/B abort behavior, including
  the Python exception/serial close; this was not changed for the milestone.

## Offline verification and retained behavior

The complete current regression suite and ESP32 build pass. All 50 Python tests
and all 23 protected M08 file hashes pass. Existing one-second-stall simulations
process JOG/center in 2 ms and STOP in 1 ms; genuine command loss or invalid-only
UART traffic stops all three axes at 250 ms. The exact timing values and
command-loss fault injection are offline evidence, not new powered measurements.

The existing 250 ms valid-command watchdog is preserved. Sensor-worker ownership,
AS5600/BNO acquisition and diagnostics, serial logging, arming/STOP/abort behavior,
pin assignments, BNO ROLL-to-physical-pitch mapping and direction signs are retained.
Manual yaw/pitch/carriage speed and acceleration ceilings remain 2000/1200/2000;
Python default speed scale remains 1.0. Optional sensor state does not gate manual
control. No new limits, guards, dependencies or automatic moves were introduced.

**POSE has not yet been revalidated on hardware for this M09 checkpoint.** Its
existing code and offline coverage are retained; prior M08 success does not
constitute M09 hardware revalidation. **Moon tracking has not been implemented.**
BNO reset causes/electrical behavior and the intermittent physical pitch event
remain outside the claimed verification.

## Reproducible record

The [evidence directory](audit/milestones/M09_manual_sensor_worker_verified_2026-09-27/)
contains the powered log, complete offline test/build outputs, evidence SHA256
hashes, and raw working-file SHA256 hashes of all 80 existing M09 files before
checkpoint creation. Those existing files were verified unchanged afterward;
Git retains its existing line-ending policy. Generated caches and the live logs
directory remain ignored. The repository commit also retains shared headers and
earlier milestones unchanged.

Build: RAM 47,204 / 327,680 bytes; flash 376,473 / 1,310,720 bytes.
Firmware SHA256: `a78913b6f33f20ec48451e71446e95fb40432c764cfa63c01601ecf741551aaa`.
The Git tag identifies the commit containing this note and source snapshot.
No firmware flash, live serial access, or powered action was performed while
creating the checkpoint.
