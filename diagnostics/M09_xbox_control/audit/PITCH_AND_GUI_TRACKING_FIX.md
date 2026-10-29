# Physical pitch and GUI-independent tracking — 2026-10-12

Work used the current TMP working tree, including its encoder-only firmware,
existing low-rate TRACK fixes, host, tests, documentation and logs. The existing
dirty `diagnostics/GimbalRollbackTest` submodule and `TMP-source.zip` were left
alone. No firmware upload, live serial connection, or physical motion was made.

## Pitch: evidence and correction

The October 8 direction audit established **positive motor steps increase raw
AS5600 counts on both axes**. It explicitly did not establish physical up/down.
The operator's current observation supplies the missing fact: raising the camera
decreases the reported pitch with the previously positive encoder scale.
Thus increasing raw pitch counts means physical downward movement on this mount.
This physical observation has not been independently repeated during this work.

The complete path is:

1. AS5600 raw counts are unwrapped into continuous ticks by `EncoderPosition`.
2. `EncoderReference.angle()` applies `(ticks-zeroTicks)*signedScale/4096`.
3. `serviceEncoders()` assigns that angle to `orientation.pitch`;
   `physicalPitch()` returns it without another sign change.
4. GOTO, LEVEL, POSE/MOVE and TRACK form physical target minus physical feedback.
5. `axisSlewStepSign()` composes motor-to-raw polarity with the signed scale.
   Slew, finite precision correction, continuous tracking and response learning
   all use this composition.
6. Negative pitch steps now physically raise the camera, reduce raw counts,
   increase calibrated pitch, and reduce a positive target error.

The minimal correction is the launcher configuration: pitch ratio
`-0.06666667`, producing approximately **-24 degrees per raw encoder turn**.
Yaw stays +360 degrees per turn. The firmware already implements the required
signed feedback and direction consistently; negating `physicalPitch()` or
changing the raw motor polarity would introduce a second, incorrect inversion.
No motor-control algorithm or direction constant was changed.

SET LEVEL still stores a zero tick reference. Pitch limits still compare the
same physical angle. TRACK HERE consumes that calibrated angle without a host
sign flip. Manual JOG keeps its existing motor-step mapping; keyframe coordinates
remain generated step counts. Carriage and yaw behavior remain unchanged.
Launching without the batch file requires the equivalent signed CLI options or
`ENCODER_CONFIG 360 -24`, followed by explicit SET NORTH and SET LEVEL.

## Tracking: evidence and correction

Encoder-era log `logs/m09_20261008T153412.727518Z_31292_0.log`, lines 344–358,
records a **3344 ms `pygame.event.get()` stall**, buffered AS5600 telemetry,
celestial calculation/update delay cancellation, and a 3000 ms firmware target
lease expiration. `m09_20261008T154151.858893Z_21816_0.log`, lines 2477–2488,
records another event-pump stall of **6734 ms**, followed by the same lease stop.
These prove host event-pump starvation; identifying the particular Windows
window operation as the trigger relies on the operator's reproduction report.
Focus-loss handling itself only logs an event; it does not cancel autonomy.

The original loop serviced serial RX, `AutoSession.frame()`, and serial TX after
the event pump. A blocked pump therefore stopped all three. A thread alone was
insufficient: pinned [pygame 2.6.1 event.c](https://github.com/pygame/pygame/blob/2.6.1/src_c/event.c)
calls `SDL_PumpEvents()` with the GIL held in `event.get()`, whereas
`event.wait(timeout)` releases the GIL. The host now waits for one event with a
1 ms timeout, then drains `get(pump=False)`, preserving all events and their order.
Pygame's display flip also releases the GIL in the pinned version.

`ControlWorker` owns serial reads, writes, shutdown STOP/abort and port close.
It services autonomous state, fresh celestial calculations, encoder setup and
STATUS every independent worker pass. The GUI prepares actions under a shared
lock and puts bounded, timestamped commands into its queue. Event pumping,
joystick polling, text-input switching and rendering occur outside that lock;
the renderer receives detached state dictionaries. Logging is serialized too.

Safety behavior:

- STOP/abort discard older unsent motion. Commands older than 250 ms stop and
  disarm instead of replaying. Firmware reset/fault invalidates prepared commands.
- Stale GUI input stops manual JOG; no worker repeats a cached stick velocity.
  Healthy celestial updates remain independent of GUI activity and focus.
- Windows XInput polls B and physical thumbstick displacement independently of
  SDL. Neutral sticks, resting triggers and focus changes do not cancel tracking.
  This is a cancellation-only path: live SDL input and the existing handshake
  are still required for manual motion. A native controller disconnection after
  connection is a terminal host failure. Verify the selected XInput slot matches
  `--controller` when using multiple controllers; other platforms retain SDL input.
- Read errors, write errors and partial writes terminate the serial worker,
  cancel autonomy, disarm, attempt STOP, and close the link without motion retries.
- Genuine telemetry loss still invokes the existing host timeout. The ESP32's
  **3000 ms celestial update lease is unchanged**, as are its sensor, stall,
  braking/restart and motor-control safety paths. Killing or suspending the host
  still causes the firmware lease to expire.

## Changed files

| Files | Change |
| --- | --- |
| `Start_Gimbal.bat`, `src/main.cpp` | Correct configured pitch sign; document the existing shared firmware sign convention. |
| `xbox_control.py`, new `control_worker.py` | Single serial owner, independent scheduler, bounded command queue, locked model actions, detached render state, GIL-releasing event pump. |
| new `windows_controller.py` | Independent cancellation-only Xbox safety polling. |
| `session_log.py` | Serialize shared logging state and file writes. |
| `tests/encoder_direction_test.cpp`, `tests/run_host_tests.ps1` | Nine mount-specific physical direction/geometry/LEVEL/reference/limit scenarios. |
| `tests/test_encoder_setup.py` | Assert the shipped launcher actually transmits the negative pitch scale. |
| `tests/test_celestial_ui.py` | Actual 3.3-second blocked GUI regression with real AutoSession and scheduler; distinct fresh updates continue throughout. |
| new `tests/test_control_worker.py`, `tests/test_windows_controller.py` | Serial ownership, expiration, STOP barrier, manual pause, read/write/telemetry/fault failures, neutral input, takeover and abort. |
| new `tests/host_test_support.py`; `test_auto_ui.py`, `test_celestial_targets.py`, `test_display_freeze_ui.py`, `test_host_ui.py`, `test_raw_command_ui.py` | Preserve existing UI regressions with non-pumping event reads and tightly bounded independent-thread scheduling tolerance. |
| `README.md`, `CELESTIAL.md`, `audit/ENCODER_DIRECTION_FIX.md`, this audit | Correct current configuration and explain behavior, evidence, validation and physical checks. |

`auto_control.py`, `operator_dashboard.py`, firmware algorithms and the celestial
lease needed no functional changes.

## Validation results

- Full Python discovery: **250 tests passed** (`Ran 250 tests in 434.337s`, `OK`).
  This includes the measured GUI stall longer than 3000 ms, fresh target updates
  throughout it, signed launcher configuration, and existing TRACK HERE tests.
  UI fixtures mock the native input reader as well as pygame/serial; dedicated
  native-reader tests use synthetic XInput states, so controller movement cannot
  affect simulated tests.
- `tests/run_host_tests.ps1 -SkipPython`: **144 C++ invocations passed** — 8 unit
  executables, 22 encoder motion cases, 26 direction cases (9 new), 24 manual
  cases, 7 pose-stop cases, 10 carriage cases, 20 keyframe cases, 15 low-rate
  celestial cases, 11 zero-rate cases, and the installed FastAccelStepper test.
- PlatformIO `run`, ESP32 / Espressif32 7.0.1 / FastAccelStepper 1.2.7:
  **build passed**. RAM 33,332 bytes (10.2%); flash 368,449 bytes (28.1%).
  The first sandboxed attempt could not write PlatformIO's existing cache;
  the authorized build with cache access succeeded. No upload target was used.
- `git diff --check`: passed.

Intermediate testing caught and corrected an abort-queue ordering bug: buffered
telemetry must not discard an unsent B/X abort, and a later STOP must preserve a
queued latched abort. A regression now covers both. Timing assertions allow
Windows clock quantization while still requiring a measured pause over 3 seconds.
No failures remain in the final full run.

## Safe physical checks still required

1. With travel clear, launch the updated batch file and STOP. Confirm fresh AS5600
   feedback, yaw scale +360 and pitch scale approximately -24. Set north and level
   references while physically aligned and stationary. Changing configuration
   invalidates previous references; it does not move the motors.
2. Use a very short, low-speed manual pitch jog. Confirm physical upward movement
   increases pitch and downward movement decreases it. STOP immediately if not.
   Verify yaw/carriage and manual stick directions remain as before.
3. Near level, request a small positive relative pitch move (for example F2
   `MOVE 0 2 0` with pitch travel clear). Verify physical pitch
   rises and its target error decreases. STOP, then test a small downward
   correction and LEVEL. Keep the abort control within reach.
4. Select a safe, above-horizon target close to current pointing, or use TRACK HERE.
   During TRACK, drag/hold another window and the dashboard for 5–10 seconds,
   release focus, and hold a mouse button. Confirm fresh target sequence numbers
   continue and lease age stays below 3000 ms. Check neutral sticks do not cancel.
5. In separate runs, verify deliberate stick takeover, STOP, and B abort while
   dragging. Confirm B remains latched and that the correct controller supplies
   the independent safety input. Finally disconnect the host link during a safe
   slow track and confirm firmware stops on loss of updates. Re-establish all
   required references before resuming; do not automatically restart motion.

Software simulations do not establish physical polarity, backlash, pulse timing,
native Windows modal behavior or controller-slot association on the actual rig.
