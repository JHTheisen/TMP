# TRACK feedback audit and operator changes (2026-10-07)

## Findings and command path

This audit used the current `src/main.cpp` PlatformIO firmware and
`xbox_control.py` application. The older Arduino sketch described by the
workspace notes is not the active firmware entry point.

`AutoSession.request_celestial()` parses RA/Dec or accepts an existing named
target. `request_track_here()` requires a qualified BNO boresight and converts
that pointing to an equatorial target once. Both use `CelestialTracker` and
`CoordinateWorker`, then the same `AutoSession.frame()` path sends
`CELESTIAL_GOTO id yaw pitch` followed by `CELESTIAL_UPDATE id sequence yaw pitch`.
The worker computes sky coordinates; it does not drive motors.

ESP32 `executeCommand()` dispatches these to `executeCelestialCommand()` and
`beginCelestial()` / `updateCelestial()`. `serviceAxes()` owns finite acquisition
and calls `enterCelestialTracking()` after both axes stop and settle. It then
dispatches to `serviceCelestialTracking()` / `serviceCelestialTrackAxis()`.
FastAccelStepper on the ESP32 owns pulse generation, acceleration, braking and
execution. Target-rate filtering, residual correction and fault handling also
remain on the ESP32. No serial protocol or celestial coordinate math changed.

Before this change, healthy BNO reports remained the primary position/error
feedback for both axes throughout TRACK. AS5600 feedback latched only after a
BNO fault and only if both encoder scales had been learned. A stationary
TRACK HERE could enter TRACK without learning either scale. In addition,
`celestialRecordBnoRecovery()` consumed the encoder feedback generation before
`serviceAxes()` could use it, potentially starving control with healthy BNO
reports even after selecting encoders.

## Feedback after this change

| Stage | Yaw and pitch control feedback | BNO role |
| --- | --- | --- |
| Admission/GOTO | Existing BNO acquisition, with its existing qualified encoder fallback | Required admission/reference; acquisition and scale learning |
| TRACK entry | Freeze encoder reference and signed scale on each axis | Last acquisition reference |
| Continuous TRACK | Unwrapped AS5600 Bus A yaw and Bus B pitch; fresh, valid, good magnet, known scale required | Reference and discrepancy reporting only |
| BNO degradation | Same encoder controller and trajectory | Degraded telemetry; no reanchor or motor-rate change |
| BNO recovery | Same encoder controller and trajectory | Report-only discrepancy; no consumption of encoder control samples |
| Encoder invalid/stale/bad magnet | Existing braking/HOLD; session and updates retained | Cannot replace encoder control in TRACK |
| Encoder recovery | Existing stopped observation and resume path | No BNO-driven catch-up |

AS5600-derived `axis.current` and `axis.error` feed the residual filter,
correction magnitude/sign, response watchdog and pitch guard. The existing
motor sign helpers are unchanged. BNO orientation is still maintained for
telemetry and future reference-dependent commands, but it cannot overwrite the
latched TRACK reference or error. `celestialSafety()` continues reporting BNO
health even while encoders control motion.

There is no documented fixed encoder-to-cradle ratio in this project.
`learnStoppedCelestialEncoderScales()` therefore observes ordinary completed
movement, using two stopped endpoints and the existing signed STEP/cradle-degree
calibration to learn encoder ticks per axis degree. It never requests movement.
It requires at least 32 generated steps and 16 encoder ticks, fresh valid
magnet readings and a 150 ms stopped interval. Missing samples invalidate the
learning interval. The existing acquisition-based BNO/encoder learner remains
available if no earlier scale exists. Learned scales are cached for the current
boot; TRACK freezes them and never relearns them from BNO noise.

If either scale is still unknown, TRACK enters a reported HOLD instead of
guessing a ratio or reverting to BNO control. STOP, deliberately move the axes
manually, stop, then retry. No scale is persisted across power cycles and no
configured pulses-per-degree constant changed (yaw 52.0, pitch 67.2).

Both host entry paths already converged; no duplicate engine was introduced.
A regression compares their complete steady-state update streams beyond one
minute. Firmware scenarios exercise both immediate HERE acquisition and a
nonzero RA/Dec acquisition before the same two-minute encoder-driven TRACK.

## Preserved safety and diagnostics

- Celestial target lease remains exactly 3000 ms, including while held.
- STOP, latched abort, joystick takeover, centered rearm and carriage handling
  retain their established paths.
- Finite GOTO/POSE convergence, direction and execution deadlines remain.
- Continuous TRACK does not use finite convergence/completion deadlines. Its
  existing commanded-motion/no-response watchdog, start/queue failure,
  unexpected stop, reversal/braking timeout, pitch guard and lease still apply.
- Low-rate fractional-Hz handling, zero-rate cancellation, acceleration,
  configured pulse calibration and yaw/pitch directions are unchanged.
- Celestial/NORTH yaw remains unrestricted; no replacement cable-wrap envelope
  was added. Other finite POSE/MOVE yaw guards and motor rate limits remain.
- `CELESTIAL_STATE`, `CELESTIAL_RATE`, encoder status and BNO discrepancy
  diagnostics remain available. Large residual errors remain visible; TRACK
  does not report finite-target settled completion.

## B-button exit hold

On B down, the application sends the existing latched abort immediately and
cancels pending automatic work. The dashboard remains open. `ExitHold` uses
monotonic time; holding continuously for two seconds exits, releasing sooner
cancels the exit timer. Repeated down events do not reset the timer. The display
shows the countdown and logs press/release/completion. Keyboard X, dashboard
ABORT, STOP and joystick behavior retain their established semantics. A tap
still aborts motion; it no longer closes the window.

## Stepped photos

The separate **Stepped photos...** panel defaults to eight movement segments,
1.0 s settle and 0.5 s after-photo delay. Segments are adjustable from 1 to 64;
each delay accepts 0 to 60 seconds. Existing smooth **PLAY A to B** is retained.
The existing movement duration is divided among segments, with the protocol's
one-second minimum per segment. Settle/photo delays are additional. Firmware
still rejects physically infeasible requested durations.

`SteppedPlayback` creates evenly spaced integer generated-step waypoints across
all three axes; rounding is deterministic and the final endpoint is exactly B.
`AutoSession` uses the existing stop/snapshot/return-to-A verification, then
one ordinary firmware `KEYMOVE` per waypoint. It requires a correlated PASS
with matching epoch and exact endpoint, followed by READY. Only then does it
wait for settle, expose one camera event, wait after the event and proceed.
No JOG stream or motor pulse schedule is generated by the photo coordinator.
STOP, abort, telemetry failure and takeover cancel remaining moves and events.

`AutoSession.take_camera_events()` drains immutable `CameraTrigger` records
containing segment/total, position epoch, three-axis steps and monotonic event
time. The default host logs `CAMERA_TRIGGER ... hook=EVENT_ONLY`; it does not
operate a camera. A future nonblocking camera adapter can consume these events.
Actual exposure acknowledgment/focus timing would need that future adapter.

## Exact files and functions changed for this audit

Earlier uncommitted target-selector/watchdog work was retained. This task adds:

| File | Change |
| --- | --- |
| `src/celestial_motion.h` | `observeCelestialEncoder()`, `celestialEncoderAxisReady()`, new `learnStoppedCelestialEncoderScales()`, `initializeCelestialEncoders()`, `updateCelestialEncoderCalibration()`, `enterCelestialTracking()`, `celestialRecordBnoRecovery()`, `celestialSafety()` |
| `src/main.cpp` | Call stopped encoder observer/learner from `loop()` |
| `auto_control.py` | `__init__()`, `request()`, `receive()`, `frame()`, `cancel()`, `_finish()`, new `_prepare_photo_move()` / `take_camera_events()`, `display_lines()` |
| `stepped_playback.py` (new) | `PhotoSettings`, `SteppedPlayback`, `CameraTrigger` |
| `xbox_control.py` | `ExitHold`, B press/release handling, photo UI dispatch and camera-event logging in `main()` |
| `operator_dashboard.py` | `__init__()`, `_paste()`, new `photo_settings()`, `handle_events()`, `draw()`: photo panel/editor and primary encoder label |
| `tests/celestial_primary_feedback_test.cpp` (new) | HERE/acquisition, BNO noise/loss/recovery, encoder invalid/magnet recovery, unknown scales/manual learning/gaps |
| `tests/celestial_motion_test.cpp` | Encoder-based TRACK fixtures, persistent/large errors and existing safety scenarios |
| `tests/celestial_low_rate_test.cpp` | Known encoder geometry; encoder-loss pause scenario |
| `tests/stubs/Arduino.h`, `tests/stubs/Wire.h` | Encoder plant independent of BNO noise; physical stalls remain observable |
| `tests/keyframe_motion_test.cpp` | Eight firmware moves with exact stops and stationary photo pauses |
| `tests/run_host_tests.ps1` | Register new C++ scenarios |
| `tests/test_stepped_playback.py` (new) | Sequence timing, exact B, cancellation, bad acknowledgment, timeout and settings |
| `tests/test_auto_ui.py` | Real event-loop B hold/release, photo panel, camera logging and takeover tests |
| `tests/test_celestial_control.py` | HERE/RA-Dec update-stream equivalence |
| `tests/test_celestial_ui.py` | Immediate B abort with window retained; modeled firmware abort latch |
| `tests/test_raw_command_ui.py`, `tests/test_display_freeze_ui.py` | B window-exit expectations; immediate abort still asserted |
| `CELESTIAL_TRACKING.md`, `TRACK_ARCHITECTURE_AUDIT.md` (new) | Current behavior and this audit/report |

## Validation results

- `tests/run_host_tests.ps1 -SkipPython`: PASS. All registered C++ suites and
  scenarios completed, including primary encoder feedback, BNO loss/recovery,
  unknown scale/HOLD, invalid reads/magnets, low-rate/zero/reversal handling,
  genuine execution/stall failures, persistent and large errors, 3000 ms lease,
  finite GOTO/POSE, NORTH/unrestricted yaw, STOP/abort/takeover, and eight stopped
  photo waypoints. M08's 23 baseline files remain byte-for-byte unchanged.
- PlatformIO Python with `.pio/python_deps` on `PYTHONPATH`,
  `-m unittest discover -s tests -p 'test_*.py'`: **231 tests passed** in
  209.456 seconds. This includes existing smooth playback and celestial tests,
  HERE/RA-Dec equivalence, B hold/release/abort, photo UI/event logging, exact
  endpoints, cancellation and telemetry failure.
- `python -m platformio run -e esp32dev`: **SUCCESS**. RAM 48,268 / 327,680 bytes
  (14.7%); flash 406,301 / 1,310,720 bytes (31.0%). The generated binary is newer
  than both changed firmware sources. No upload target was invoked.
- Photo panel rendered and inspected at 1100x760 and the minimum 820x620.
  STOP/ABORT remain visible and the settings/start controls fit.
- `git diff --check`: PASS.

Evidence: `.pio/architecture_cpp_tests.log`, `.pio/architecture_python_tests.log`,
`.pio/architecture_esp32_build.log` and `.pio/stepped_photo_dashboard*.png`.
Tests use simulated hardware; no powered motor or camera test was performed.

## Hardware limits

Software fixtures cannot certify optical tracking accuracy, gear ratio, encoder
mounting, missed steps or mechanical slip. Stopped-step scale learning depends
on the existing provisional motor calibration and normal mechanical execution;
a partial mechanical slip while learning can bias the inferred scale. A measured
encoder ratio remains preferable when hardware data becomes available.
AS5600 is single-turn: movement over half a shaft turn between valid samples
is ambiguous, especially across outages. The preserved resume path cannot
recover unknown whole turns. BNO acquisition/reference errors are frozen into
the reference and are reported rather than corrected during TRACK.

Real hardware should verify each encoder's signed scale, both-axis low-rate
tracking, BNO loss/recovery without reference jumps, encoder HOLD/resume,
STOP/takeover, backlash/settle delays and all three photo waypoint positions.
KEYMOVE endpoints are generated-step coordinates, not homed/optically verified
positions. No shutter hardware or photographic accuracy is claimed.

No firmware upload is part of this task.
