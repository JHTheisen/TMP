# Closed-Loop Zero Integration Skill

## Goal
Create the smallest possible Version 3 change on top of the known-working Version 2 behavior.

## Workflow
1. Inspect the existing Version 2 joystick-to-serial Python bridge and the Arduino firmware.
2. Preserve the current manual thumbstick mapping and serial telemetry behavior.
3. Add only one new capability: a single-axis AS5600-based return-to-zero closed-loop mode.
4. Initialize the AS5600 at startup to a saved zero angle.
5. Keep motor 2 on its existing manual path and leave the other hardware paths untouched.
6. Use the AS5600 as live feedback during return-to-zero mode and stop once the measured error is near zero.

## Decision Points
- If the AS5600 is missing, keep startup and manual commands alive but mark the sensor as unavailable.
- If the new command is pressed, activate closed-loop mode for the AS5600 axis only.
- If the measured angle crosses the 0/360 wrap boundary, normalize the delta before issuing motor commands.
- If the measured deviation is within tolerance, stop the return-to-zero drive and return to the existing command handling path.

## Completion Checks
- Manual joystick control still operates the same as Version 2.
- One AS5600 axis is zeroed at startup and used as feedback.
- The other motor is not rewritten or reworked.
- The new mode is command-driven and localized to the current firmware and bridge code.
