# BNO axis check

Standalone ESP32 sensor-only diagnostic for identifying which reported BNO085
orientation component changes when the pitch cradle is moved by hand.
No motor library, motor GPIO initialization, motion commands or encoder access.
M08 and the direction diagnostic are not used or modified.

- BNO085: Bus B, SDA4 / SCL5, address 0x4A, 100 kHz I2C.
- Serial: 115200 baud, one line approximately every 100 ms.
- Uses the same SH2_ROTATION_VECTOR quaternion-to-Euler convention as M08:
  yaw/heading 0..360 degrees, pitch -90..90 degrees, roll -180..180 degrees.
- Accuracy 0/1 does not suppress orientation output or require calibration.

Example output (illustrative):

```text
Yaw=  72.45  Pitch=   3.12  Roll= -18.36  Acc=0  FRESH   Age=2 ms
```

`Age` is milliseconds since the last accepted finite rotation vector. At 150 ms
it is marked `STALE`; displayed angles then remain the last valid values.
`INVALID` means the latest rotation vector was invalid; displayed values/accuracy
still belong to the last valid sample. Before any valid sample, or after a reset,
fields show `--` / `NO DATA`. Sensor resets automatically re-enable reports.

With the pitch motor unplugged, move the pitch cradle slowly by hand in both
directions. Watch which component changes most, and whether it increases or
decreases for each physical direction. Keep the other physical axes still.
If multiple components change, record all three; this diagnostic applies no
axis remapping, tare, sign correction or calibration gate.

From this directory, using PowerShell:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev -t upload --upload-port COM9
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM9 --baud 115200
```

Uploading this diagnostic replaces the firmware running on the ESP32. The source
projects remain separate. No upload is performed as part of creating this project.