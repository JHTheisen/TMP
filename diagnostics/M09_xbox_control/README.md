# ESP32 camera gimbal — dual AS5600 control

The ESP32 owns motor pulses, acceleration, braking, angular feedback, GOTO
acquisition and continuous tracking. Python owns Xbox input, the dashboard,
celestial calculations and target updates. Firmware entry point: `src/main.cpp`.
Host entry point: `xbox_control.py`. There is no automatic startup movement.

## Hardware

| Device | GPIO | Interface |
| --- | --- | --- |
| Yaw stepper | DIR32 / STEP33 | FastAccelStepper |
| Pitch stepper | DIR26 / STEP12 | FastAccelStepper |
| Carriage | DIR21 / STEP22 | FastAccelStepper |
| Yaw AS5600 | SDA18 / SCL19 | Wire, address 0x36 |
| Pitch AS5600 | SDA4 / SCL5 | Wire1, address 0x36 |

Both buses operate at 100 kHz. One acquisition task alternates read-only
STATUS/RAW_ANGLE transactions every 10 ms (approximately 20 ms per encoder).
Motor stepping remains independent of Python and I2C. Manual JOG retains its
existing -1 step mapping on yaw/pitch. Automatic angular control instead uses
the measured motor-to-encoder polarity, composed with each signed encoder scale.
Positive calibrated yaw is clockwise looking down on the mount; positive pitch
raises the camera. See the [direction regression and checks](audit/ENCODER_DIRECTION_FIX.md).

**Mounting matters:** the pitch encoder is before reduction. The operator-supplied
ratios are yaw 1:1 and pitch 0.06666667 output turns per encoder turn (about 1:15).
These are configured in `Start_Gimbal.bat`; verify their direction signs on the
mount. No encoder ratio or direction is inferred from motor counts. See
[calibration](CELESTIAL.md). A shaft encoder measures
shaft motion, so downstream backlash, slip and flex cannot be observed by it.

## Run and build

Use Python 3.11+ and install `requirements.txt` into its environment. Run:

```powershell
python xbox_control.py --port COM9 --latitude 43.9425 --longitude -86.0394 --north-reference true
```

Use your actual observing location. The existing `Start_Gimbal.bat` uses the
installed PlatformIO Python and `.pio/python_deps`; update its location if needed.
Serial is 115200 baud. `--dry-run` opens no serial port.

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
& tests/run_host_tests.ps1
```

The build pins Espressif32 7.0.1 and FastAccelStepper 1.2.7. Only Wire and
FastAccelStepper are firmware dependencies. `run` builds without uploading.

## Operator workflow

1. Launch `Start_Gimbal.bat` to apply its encoder ratios automatically (or use F2
   `ENCODER_CONFIG` if running without the ratio options). Align the camera to north
   and horizontal, then use **SET NORTH** and **SET LEVEL**. Both references are
   required for angular automatic control. Calibration commands never move motors.
2. Manual joystick control and carriage/keyframe motion remain available without
   calibrated encoders. Center sticks for 0.5 seconds to arm manual control.
3. Choose a celestial object (including Jupiter), enter manual RA/Dec, or use
   TRACK HERE to capture the current pointing. A successful GOTO transitions
   into continuous TRACK; this application has no separate GOTO-only UI action.
4. Use STOP to brake, or deliberate stick displacement to cancel an automatic
   operation and take over after the STOP/READY/centered-JOG handshake.

Xbox defaults: A/0 LEVEL, Y/3 NORTH, LB/4 capture A, RB/5 capture B, X/2 duration,
Home/10 play A to B, button 8 return A, button 9 increment, D-pad relative angles.
B/1 is immediate latched abort. Space/F12 STOP; F2 raw console; F3 freezes only
diagnostic displays. Window focus loss does not cancel autonomous operation.
Mappings remain configurable through command-line options; use dry run to verify
your controller's indices. Manual rates retain yaw/carriage 2000 and pitch 2400
pulses/s ceilings and their existing acceleration limits.

Dashboard yaw/pitch are calibrated encoder measurements. Raw shaft angles remain
in diagnostics. ROLL explicitly reads UNAVAILABLE: two axis encoders do not measure
roll. Calibration and feedback status remain live when diagnostics are frozen.

## Preserved motion and limits

Keyframes store generated yaw/pitch/carriage step counts and a boot/position epoch.
They are not absolute encoder poses. Normal STOP preserves captures; reboot or a
forced stop invalidates them. Continuous and stepped photo playback remain
implemented. Stepped playback produces logged camera-trigger events; there is
no physical shutter adapter in this project. Carriage is unhomed generated steps.

LEVEL/NORTH return to the manually declared references. POSE/MOVE retain their
existing planning and guarded first precision moves. Celestial pointing limits
pitch to strictly within +/-75 degrees; LEVEL retains its +/-89 degree recovery
guard. Celestial yaw follows the shortest initial path and can track across
multiple revolutions; it has no cable-wrap or homing protection. POSE retains its
existing +/-185 degree continuous yaw envelope from the north reference.

See [celestial setup](CELESTIAL.md), [tracking control](CELESTIAL_TRACKING.md),
[target catalog](CELESTIAL_TARGETS.md), and [validation](VALIDATION.md).
