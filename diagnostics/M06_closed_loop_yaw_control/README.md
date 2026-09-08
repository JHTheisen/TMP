# Milestone 6: Closed-Loop Relative Yaw Position Control

Status: implementation and software verification; **M06 physical verification
is pending**. No upload, commit, or push is part of this development session.

This separate PlatformIO project is `diagnostics/M06_closed_loop_yaw_control`,
beside `diagnostics/M05_closed_loop_pitch_control`. M05 was physically verified
by the user and recorded in commit `c810581`; its files remain untouched.

## Purpose and sequence

Prove a modest relative yaw move using the cradle-mounted BNO085:

1. Keep the platform stationary through initialization and a five-second delay.
2. Capture a stable one-second startup heading window as relative yaw zero.
3. Move toward measured **+3 degrees** using small motor bursts and fresh BNO
   feedback. Positive relative yaw means increasing BNO heading, including
   359 -> 0 degrees; it does not prescribe clockwise motion as viewed by a person.
4. Settle within **+/-0.4 degrees** for a complete one-second stopped window.
5. Return to the original measured heading and settle with the same tolerance.
6. Print one PASS/FAIL summary and latch idle.

No magnetic-north or true-north target is commanded. M06 keeps M05's
`SH2_ROTATION_VECTOR` configuration and subtracts the startup orientation.
The BNO fusion remains the sensor's existing implementation; this milestone
does not add compass calibration, declination, or absolute heading alignment.

## Hardware and reuse

| Item | Configuration |
| --- | --- |
| Yaw stepper | DIR GPIO 32, STEP GPIO 33, FastAccelStepper |
| BNO085 on moving cradle | `Wire1`, Bus B, SDA 4 / SCL 5, address `0x4A` |
| I2C and reports | 100 kHz, 50 ms transaction timeout, 10 ms rotation-vector reports |
| Pitch and carriage | Their STEP outputs are LOW; no stepper objects or motion commands |
| AS5600 | Neither encoder is required, initialized, read, or used for PASS |
| Bus A | Unused |

The existing `firmware/include/hardware_config.h` supplies the pin map without
modification. M06 preserves M05's FastAccelStepper setup, 200 us DIR setup delay,
finite nonblocking moves, acceleration, measurement/settling windows, and
`forceStop()` plus the verified 25 ms queue-drain delay.

`src/sensor_support.h` is copied byte-for-byte from M05. It preserves the BNO
HAL failed-write workaround, quaternion-to-Euler conversion, and health
accounting. M06 uses the conversion's heading output instead of pitch. Its
active feedback deadline remains M05's tighter 150 ms, checked before a
recovered sample can replace the old timestamp.

Pinned dependencies match M05: Espressif32 7.0.1 / Arduino-ESP32 2.0.17,
FastAccelStepper 1.2.7, Adafruit BNO08x 1.2.7, Adafruit BusIO 1.17.4, and
Adafruit Unified Sensor 1.1.15. The unused AS5600 dependency is omitted from
this project; M05's dependencies are unchanged.

## Simple control and heading wrap

Each finite burst finishes before another is issued. After stopping, the
controller waits 300 ms and averages fresh heading observations for 250 ms.
Bursts and requested speed shrink as the measured target error decreases.
There is no continuous-run command, PID, or queued multi-burst trajectory.

| Initial setting | Value |
| --- | --- |
| Final tolerance | +/-0.4 degrees, all samples in a stable 1 s stopped window |
| Approach margin on averaged error | +/-0.10 degrees, leaving room for sensor noise |
| Normal burst limit | 16 pulses |
| Unconfirmed-direction burst limit | 4 pulses at 40 pulses/s |
| Normal requested speed | 40 to 120 pulses/s |
| Acceleration | 240 pulses/s squared |
| Relative measured travel guard | +/-6 degrees from startup |
| Active BNO freshness deadline | 150 ms |
| Burst / leg / no-progress deadlines | 3 s / 90 s / 15 s |

The error-to-pulse gain is a conservative starting value carried over from
M05's general behavior. It is not a yaw drivetrain model. No theoretical
ratio, expected angle per STEP, encoder relationship, or return-to-zero pulse
count is an acceptance or travel-safety criterion.

Heading samples are unwrapped by accumulating the shortest signed difference
between consecutive readings. Statistics use this continuous coordinate, so
359.9 and 0.1 average near zero rather than 180. Target errors use the signed
shortest difference in [-180, 180). An exact half-turn transition is ambiguous
and rejected. Continuous measured travel is guarded without resetting at
0/360; a full turn cannot erase its accumulated displacement. As with M05,
this small test relies on fresh sensor samples, not prediction of motion
during a missing report.

Stable baseline and final windows need at least 30 BNO samples; decision
windows need at least 10. Every accepted window requires sample gaps <=100 ms,
heading SD <=0.15 degrees, range <=0.5 degrees, and first-half to second-half
mean drift <=0.2 degrees. An unstable window causes a stationary retry, subject
to the startup or leg timeout. The approach margin is not a claim of
0.10-degree absolute sensor accuracy.

## Direction verification and safe stops

**Yaw polarity is initially unverified.** The default
`M06_TRIAL_POSITIVE_STEP_YAW_SIGN=1` is a trial command mapping, not a confirmed
installation fact. Each leg is restricted to four-pulse moves at 40 pulses/s
until at least 0.3 degrees of measured movement toward that leg's target is
confirmed by two fresh, stable stopped windows.

If the measured response instead exceeds 0.3 degrees in the opposite direction,
the controller stops issuing commands and takes a second observation while
stationary. Persistent contrary motion produces **`DIRECTION MISMATCH`** and
FAIL. The firmware never automatically flips the sign or retries.

After direction confirmation, an error increase greater than 0.6 degrees above
the best error reached also triggers a stopped confirmation and then a
`DIRECTION MISMATCH / RUNAWAY` abort if it persists. Smaller failure to progress
is bounded by the 15-second no-progress watchdog (at least 0.15 degrees of
improvement while outside tolerance) and the 90-second leg deadline.

All fresh yaw readings are checked against the +/-6-degree measured envelope
before averaging. There is no absolute north-sector guard. BNO pitch at or
above +/-75 degrees stops/refuses the test because Euler heading becomes poorly
conditioned near vertical; this only checks orientation and never drives pitch.

A 150 ms BNO gap, BNO reset after establishing the reference, rejected motor
command, or expired motion deadline latches a stop. A reset before the baseline
can restore reports and start fresh stationary observations. After the reference,
the firmware never silently re-zeros the heading.

Send **`X` or `x`** to abort. It stops and latches idle, without an automatic
return. **Ctrl+C closes the serial monitor; it does not abort motion.**
Keep motor power removal accessible for a physical emergency stop. Serial
handling and software guards wait if a foreground library call blocks, while
the motor can execute only its current finite burst. A software stop does not
switch off driver power or provide mechanical endstops/homing.

PASS requires both measured targets settled, direction confirmed on each leg,
at least one degree of observed motion toward each target, accepted yaw moves,
and BNO reports during movement. Without a shaft encoder, firmware cannot
independently attribute all measured movement to motor torque; the operator
must confirm that yaw actually responds and that other axes remain still.
Active control ends after the final settle; this test does not establish
long-term position holding.

## Build, upload, and physical verification

1. Start with the platform stationary and reasonably level. Clear yaw travel
   for at least 6 degrees in both directions, check cables, and keep hands clear.
   Use the existing verified motor wiring, driver, and supply setup. Close the
   Python controller and any other program using COM9.
2. From the repository's `diagnostics` directory, enter **M06** and build:

   ```powershell
   Set-Location -LiteralPath '.\M06_closed_loop_yaw_control'
   & "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
   ```

3. When ready for motion, upload and open a logging serial monitor:

   ```powershell
   & "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev -t upload --upload-port COM9
   & "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM9 --baud 115200 --filter log2file
   ```

   Upload/reset starts the sequence automatically after initialization, the
   startup delay, and a stable baseline. If the monitor missed startup, confirm
   clearance and press EN/reset once. Use the actual board port if it has changed.

4. Confirm the banner says `M06_closed_loop_yaw_control`, DIR32/STEP33, and
   `UNVERIFIED trial positive STEP yaw sign=1`. Look for `REFERENCE` and
   `LEG OUTBOUND`. Expect initially very small movements separated by pauses.
   Observe that only yaw moves and `relative_yaw` increases toward +3 degrees.
5. Watch for `DIRECTION CONFIRMED`, `SETTLED OUTBOUND`, `LEG RETURN`, another
   direction confirmation, and `SETTLED RETURN`. Each settled error should be
   within +/-0.4 degrees. The final line should be `FINAL RESULT: PASS`.
   Confirm yaw visibly returns near the start and stays still after the summary.
   The displayed pulse count need not return to zero.
6. If the first trials move the wrong way, send `X` if necessary and preserve
   the mismatch log. After verifying that the physical yaw response and BNO
   heading have opposite signs to the trial setting, change only the default
   macro in M06's `src/control_math.h` from `1` to `-1`, rebuild, and upload for
   a new deliberate test. Do not change pins or M05. A stall/noisy-sensor failure
   is not evidence that polarity needs reversing.
7. If any motion is unexpected, send `X` (plus Enter if your terminal buffers
   input), or remove motor power if needed. Do not increase the travel guards
   to bypass a failure. A separate deliberate serial-abort run can confirm the
   idle latch after successful positioning.
8. If the startup heading happens to be near 359 degrees, +3 should cross
   through 0 and still show a small positive relative yaw. Do not aim at north
   or rearrange hardware merely to force this case; software tests cover it.

Logs are saved under `logs/device-monitor-*.log`. To find key lines:

```powershell
Select-String -Path '.\logs\device-monitor-*.log' -Pattern 'REFERENCE|DIRECTION|SETTLED|FINAL RESULT:|Reason:'
```

Report the complete final summary, outbound/return errors, chosen trial sign,
any BNO warnings, whether yaw moves smoothly without hunting, and whether pitch
and carriage stay stationary. Physical verification remains the user's next
step; **do not commit or push M06 until that verification is confirmed**.

## Non-hardware checks

From the M06 project directory:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
```

The runner defaults to the installed `C:\Strawberry\c\bin\g++.exe`.
Use `-Compiler` to select another compatible compiler. It uses strict C++11
warnings as errors, tests the real firmware setup/loop with deterministic
fixtures, and separately compiles an operator-selected negative trial polarity.
There is no AS5600 stub/library in the tests, and Bus A never acknowledges an
encoder. See `VALIDATION.md` for build and software results.

Simulation verifies decisions and bookkeeping, not real ESP32 pulses, stop
latency, I2C wiring, BNO fusion performance, motor dynamics, or mechanical clearance.
North calibration, celestial heading, encoder closed-loop feedback, pitch
control, carriage control, external controllers, keyframes, tracking, homing,
automatic calibration, and elaborate PID remain out of scope.
