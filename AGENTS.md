# AGENTS

## Project overview
This workspace contains a joystick-to-ESP32 gimbal controller.

- `v05_Add Additional Sensors.py` is the host-side Python controller.
  - Uses `pygame` for joystick input and `pyserial` for serial communication.
  - Sends `yaw,pitch` commands to the ESP32 every 20 ms.
  - Sends single-letter mode requests: `L`, `N`, `M`, `C`, `Z`, `K`, `B`, `T`.
  - Reads telemetry from the ESP32 and prints AS5600/BNO085 values.
  - Clears smoothed stick state when entering/canceling auto modes so stale joystick ramps do not leak into firmware modes.
- `v05_Add_Additional_Sensors/v05_Add_Additional_Sensors.ino` is the ESP32 firmware.
  - Controls two stepper motors via GPIO step/dir pins.
  - STEP pulse generation has been moved out of the foreground loop and is now hardware-timer driven for both motors.
  - Uses AS5600 magnetic encoders and optional BNO085 IMU.
  - Supports manual motion, return-to-zero, north/level modes, and keyframe playback.
  - BNO085 is on `Wire1` with the pitch AS5600; expected addresses are AS5600 `0x36` and BNO085 `0x4A` or `0x4B`.

## Run instructions
- Python host: run `python "v05_Add Additional Sensors.py"` from the workspace root.
- ESP32 firmware: open `v05_Add_Additional_Sensors/v05_Add_Additional_Sensors.ino` in Arduino IDE or PlatformIO.
- Serial port is configured in Python via `SERIAL_PORT = "COM9"` and baud `115200`.

## What agents should know
- The primary work is in the Arduino sketch, not the Python script.
- The Python script is mostly I/O and smoothing; real motor behavior is determined by the ESP32 sketch.
- Commands from Python are bound to stepper rates in the Arduino code, not direct step pulses.
- Internal logical modes currently use `motor1` for yaw/north/zero and `motor2` for pitch/level, but the physical output pins are swapped in the hardware-timer step service:
  - `motor1Rate` is sent to `STEP2/DIR2`.
  - `motor2Rate` is sent to `STEP1/DIR1`.
  - Manual joystick routing is also crossed in `readSerial()` to match the physical axes after that output swap.
- Hardware-timed stepping is working and made both motors dramatically smoother. Do not casually replace it with foreground-loop `runMotor()` stepping.
- `levelMode` is working on the correct physical motor using BNO pitch (`LEVEL_USE_ROLL = false`), but the current sketch has fixed `LEVEL_TARGET_DEG = 0.0`; Python sends `T`, while firmware currently does not handle `T` or define `tareLevelTarget()`.
- `northMode` moves the correct physical yaw motor and uses only fresh `SH2_ROTATION_VECTOR` heading, not game rotation. It is currently off by about 20-30 degrees and likely needs a north tare/heading offset.
- `SH2_GAME_ROTATION_VECTOR` was removed because it is relative yaw and should not be used for north. BNO accelerometer remains available as a level fallback.
- Manual stick movement above `MANUAL_OVERRIDE_THRESHOLD` cancels active auto modes in firmware.
- Manual stick control was made closer to v04 behavior: Python does the smoothing for manual control; firmware ramping is used for automated modes.
- New/remaining issue: the physical pitch motor can occasionally begin moving without intentional user input. The physical pitch axis is driven by `motor2Rate`; `desiredMotor2Rate` can be written by manual yaw input in `readSerial()`, level mode, north+level mode, and keyframe playback.
- Most likely pitch drift cause is resting/noisy Python `yaw_cmd` crossing the firmware manual deadzone, because `readSerial()` maps incoming `yaw` to `desiredMotor2Rate = (yaw / 1000.0) * MOTOR2_MAX_RATE`.
- Secondary pitch-motion suspects: stuck `levelMode`/`northLevelMode`, keyframe playback, and `stopKeyframePlayback()` clearing `motor2Rate` but not `desiredMotor2Rate`.

## Useful code pointers
- `readSerial()` interprets joystick commands and mode letters.
- `initStepTimer()`, `onStepTimer()`, `serviceTimerMotor()`, and `updateTimerMotorRates()` implement hardware-timed STEP pulse generation.
- `updateLevel()`, `updateNorth()`, and `updateReturnToZero()` compute automated control outputs.
- `rampPlaybackRate()` and the global rate variables influence motor acceleration.
- `tareLevelTarget()` is not present in the current sketch even though Python still sends `T`.
- `NORTHDBG`/`LEVELDBG` references may be stale; the current sketch prints some status lines but does not currently emit full debug lines with target/error/settle details.

## Notes for debugging
- If buttons appear to disable stick axes, check whether mode flags are stuck active; manual override should print `Manual override: auto mode canceled`.
- If the steppers knock, preserve the hardware-timer pulse generator and inspect rate updates, acceleration/ramping, interrupt load, and any accidental reintroduction of loop-timed stepping.
- If level moves the wrong physical motor, re-check the timer service mapping and the crossed manual assignments in `readSerial()`.
- If level reaches a visible offset, `T` tare needs firmware support restored/implemented; it is not currently handled in `readSerial()`.
- If the pitch motor moves with no intended input, first inspect Python's displayed `Yaw:` value and incoming serial `yaw` values; the physical pitch motor follows the incoming `yaw` field through crossed manual routing.
- If north lands inconsistently, inspect `NORTHDBG`, BNO heading age, magnetic calibration/interference, and add/tune a north heading offset.
- The existing batch file `Run Gimbal Controller.bat` references `v04_Add Key Frames.py`, so it may be outdated.
