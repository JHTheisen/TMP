# Milestone 09: Xbox manual yaw/pitch/carriage

M09 adds a PC/Python Xbox interface to the **tested M08 control baseline**.
M08 is unchanged. M09 builds and offline tests pass. The subsequently authorized
powered BNO startup A/B reproduced low-accuracy/pitch-only readiness under both
stock M08 and M09; powered Xbox motion verification remains pending. See the
[startup audit](audit/BNO_STARTUP_COMPARISON.md) and
[hardware captures](audit/HARDWARE_RESULTS.md).

## Historical code and architecture

The repository retains `../../v05_Add Additional Sensors.py` and
`../../v05_Add_Additional_Sensors/v05_Add_Additional_Sensors.ino`, archived in commit
`2121682`. Xbox input ran on the **PC** using pygame and pyserial at 115200 baud,
sending normalized commands every 20 ms. M09 reuses that host/firmware split,
input pumping, cadence, deadband and velocity-command concepts. The ESP32 has no
new USB/Bluetooth controller-host responsibilities.

The old script labels raw axis 1 `yaw` and axis 2 `pitch`, but its firmware crosses
the fields: `yaw` drives physical pitch and `-pitch` drives physical yaw. Its old
pin layout and sensor mapping do not supersede M08. M09 uses clear fields and
defaults to **raw axis 2 for right-stick horizontal yaw** and **raw axis 1 for
left-stick vertical pitch**. Verify these with dry run: indices can differ across
drivers and USB/Bluetooth connections. The old batch launcher references missing
v04 code and Python 3.6; it is unchanged and should not launch M09.

The host requests rates; ESP32 retains FastAccelStepper pulse generation,
acceleration, BNO acquisition and safety. This Python host can later run on
Raspberry Pi with its serial port and joystick mapping verified. No camera,
keyframe, astronomy, tracking or other host features are included.

## Preserved baseline

| Axis | DIR / STEP GPIO | Maximum pulses/s | Acceleration pulses/s² |
| --- | --- | ---: | ---: |
| Yaw | 32 / 33 | 1000 | 1000 |
| Pitch | 26 / 12 | 600 | 600 |
| Carriage | 21 / 22 | 1000 | 1000 |

BNO085 remains on Wire1, SDA4/SCL5, address 0x4A. Physical pitch is **BNO ROLL**.
Both existing positive-step signs remain `-1`, including
`POSITIVE_STEP_PITCH_SIGN = -1`. M08's control math, pose math, sensor support,
watchdog implementation, shared pin header and pinned PlatformIO configuration
are unchanged. Startup north/level, closed-loop POSE/MOVE, pitch-only readiness
and carriage behavior are retained.

**Power-up, reset, upload, or opening serial can run the preserved automatic
north/level startup.** Manual arming does not gate that existing startup.
Carriage stays stationary during startup and when its stick axis is centered. Existing explicit
POSE/MOVE carriage commands and its +/-500 startup-relative step envelope remain;
it still has no homing or measured rail-position feedback.

## Manual behavior

- Right horizontal stick requests yaw velocity. Left vertical stick requests
  pitch velocity; up defaults to positive physical pitch/BNO roll.
- Left horizontal stick (raw axis 0) requests carriage velocity: left is negative
  step direction, right positive. Add `--invert-carriage` if physical direction
  is backwards; `--carriage-axis N` changes its input index only.
- A 15% deadband produces exact zero at rest. Remaining displacement is rescaled
  continuously and squared for fine control near center.
- Default full stick requests **25%** of the unchanged firmware ceilings: yaw
  250, pitch 150 and carriage 250 pulses/s. `--speed-scale 1` permits the original ceilings
  after verification; it changes host requests, not firmware limits.
- FastAccelStepper supplies acceleration/deceleration. Release commands zero
  immediately, then the motor brakes at its existing acceleration and stays
  stopped without a position correction. Ideal braking at the default maximum
  is about 0.25 s plus acquisition/queue latency; at full ceilings about 1 s.
  These are estimates, not physical stopping-time measurements.
- Reversal brakes fully and waits for fresh stopped feedback plus the existing
  100 ms observation allowance before starting the other direction. Same-direction
  rate changes update the active ramp without repeated move/restart calls.
- Carriage moves at the requested rate toward the existing +/-500-step boundary,
  where FastAccelStepper brakes and stops. Holding outward stays stopped; reversing
  permits inward motion. STOP/release near a boundary retains that braking target
  to avoid extending it. The startup step reference is never reset by manual arming.
- Wait for manual `M09 READY`, center both sticks for 0.5 s, then press
  **Space or A** to arm. Press again to stop and disarm.
- **X key or B button** aborts and latches all axes. **Esc/window close** stops
  manual motion and exits; closing during unfinished startup aborts. **Ctrl+C**
  aborts. Keyboard controls require the control window to be focused.
- Controller/focus/telemetry/serial loss or a host pause over 100 ms ends the host
  session and attempts X. Reconnection never automatically resumes motion.

Manual mode requires completed startup, fresh valid BNO feedback, stopped motors
and clear watchdogs. Accuracy 0–3 is accepted for manual yaw/pitch/carriage JOG, including
after PITCH-ONLY READY. The original startup wait remains. This does not calibrate
north: POSE/MOVE retain their existing north/accuracy requirements. Without a north
reference, manual yaw travel and stopping margins use the existing pitch-only
startup orientation as their reference; rearming does not recenter that envelope.

The window keeps the firmware phase, last BNO accuracy/freshness/sample age and
readiness explanation above the wrapped serial messages. During STARTUP/BASELINE,
the original firmware's zero error fields are not yet calculated target errors.
PITCH-ONLY READY remains visible even when STATUS replies BUSY. The last BNO
report's receipt age is shown because the firmware does not emit STATE continuously
in every mode; old values must not be mistaken for current sensor health.

## Protocol and protections

New commands are newline-terminated ASCII at 115200 baud:

| Command | Behavior |
| --- | --- |
| `STATUS` | Returns `M09 READY`, `M09 BUSY`, `M09 MANUAL`, or `M09 ABORTED`; no motion. |
| `JOG 0 0 0` from READY | Arms stationary and reports `MANUAL READY`. |
| `JOG <yaw> <pitch> <carriage>` while armed | Integer requests -1000..1000, scaled to each native speed ceiling. Send every 20 ms, including centered zeros. Older two-value JOG remains supported and commands zero carriage velocity. |
| `STOP` | Brakes/disarms manual mode; reports `MANUAL STOPPED` and `M09 READY` after stopped observation. |
| `X` / `x` | Existing immediate serial abort, including inside partial commands; reset required. |

POSE/MOVE are rejected while manual is active. JOG cannot interrupt startup or
a pose. STOP applies to manual only; X aborts every operation. Malformed, partial
and oversized JOG lines do not refresh the manual lease. Old `yaw,pitch` CSV and
old mode-button letters are not the M09 protocol.

The unchanged independent **150 ms BNO stale watchdog** remains armed throughout
manual mode, including braking and centered idle. Invalid/wrong reports do not
refresh it. Sensor resets retain the inherited abort path. Accuracy changes alone
do not abort manual JOG; absolute-heading operations retain their original checks.
A second instance of the unchanged watchdog adds a **250 ms command lease**:
lost commands independently request stop and latch abort, even if loop() blocks.
Late packets cannot revive it. STOP transfers control to bounded BNO-guarded
braking, allowing normal host exit without a command-loss fault. Force-stop
retains FastAccelStepper queue-drain latency; timeout is not an instantaneous
physical stop guarantee.

Existing +/-185° yaw, +/-75° roll, carriage and 90 s motion guards remain.
Manual continuous motion is limited to 90 s; all three assigned inputs centered and all three motors
fully stopped reset only its manual clock. BNO displacement monitoring uses the
existing 0.6° reverse excursion and 0.15° progress thresholds, with a 2 s progress
deadline at >=40 Hz and the existing 15 s deadline below that; braking is bounded
by 3 s. Tiny rates without detectable BNO progress may abort. Manual outbound
motion also aborts at the existing measured-rate stopping margin before the hard
angular guard. Do not enlarge limits to bypass failures.

## Setup and offline checks

Use Python 3.11+ from this M09 folder. The old Python 3.6 launcher is unsuitable.
This machine's installed PlatformIO Python can create a local environment:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m venv .venv
& .\.venv\Scripts\python.exe -m pip install -r requirements.txt
& .\.venv\Scripts\python.exe xbox_control.py --dry-run
```

Dry run opens **no serial port**. Right horizontal stick must change only Yaw;
left vertical stick must change only Pitch. Both commands must stay zero at rest.
Left horizontal stick must change only Carriage. Inspect all raw axes in the window.
If needed use `--yaw-axis N --pitch-axis M --carriage-axis L`
and retain those options for the powered run. `--invert-carriage`, `--invert-yaw` and
`--no-invert-pitch` change only host preferences, never firmware motor signs.
Increase `--deadband` if centered raw input falls outside 0.15. Right-stick vertical
and triggers have no assigned function.

Compile/test without upload or serial access:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
powershell -NoProfile -ExecutionPolicy Bypass -File tests\run_host_tests.ps1
& .\.venv\Scripts\python.exe -m unittest discover -s tests -p 'test_*.py' -v
powershell -NoProfile -ExecutionPolicy Bypass -File tests\verify_m08_baseline.ps1
```

Python UI tests use dummy video/audio and mocked Xbox/serial devices. C++ tests
compile actual firmware with simulated sensors/motors. These do not validate
ESP32 scheduling, torque, backlash, driver mapping or physical travel.

## Controlled first powered test — operator only

1. Complete the unpowered dry-run mapping check. Use the same supported, clear
   setup as the known-good M08 test. Allow startup yaw/pitch travel and cable
   clearance; keep carriage clear and motor power removal immediately available.
   Start modestly off north and level as in M08 so its movement-evidence check
   can succeed.
2. When ready for automatic startup motion, upload **M09 yourself**:

   ```powershell
   & "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev -t upload --upload-port COM9
   ```

3. Close other serial monitors. Start the host, appending any axis options verified
   in dry run:

   ```powershell
   & .\.venv\Scripts\python.exe xbox_control.py --port COM9 --speed-scale 0.25
   ```

   Opening serial may reset ESP32 and repeat M08 startup. Leave sticks centered;
   wait for completed startup and `M09 READY` / `Ready: True`. PITCH-ONLY READY
   now also permits manual yaw/pitch/carriage at accuracy 0–3; north remains unverified.
   Confirm carriage is still. Startup faults still prevent arming.
4. Focus the control window, center both sticks for 0.5 s, and press Space/A.
   Confirm MANUAL READY and no motion for several seconds.
5. Briefly deflect **only right horizontal stick** slightly, then release. Check
   yaw alone moves smoothly, positive command increases BNO heading, and release
   brakes to a complete stop. Repeat gently in the opposite direction.
6. Repeat with **only left vertical stick**. Pitch alone must move; positive
   command increases BNO roll, and release stops. Keep left horizontal centered
   so carriage remains stationary during this pitch-only check.
7. Briefly deflect only **left horizontal stick** and release. Check carriage
   alone moves and brakes to a stop. Check both directions; use `--invert-carriage`
   after stopping/exiting if needed. Only after independent axes pass, try small
   simultaneous commands and a gentle reversal. The existing travel envelope is
   +/-500 generated steps, with no physical homing or rail-position sensor.
8. Press Space/A to stop/disarm; confirm `MANUAL STOPPED` / `M09 READY`. Sticks
   must do nothing while disarmed. Rearm centered for a final low-speed **B/X abort**
   check: subsequent arming must be refused until deliberate reset. For any
   unexpected motion, use B/X or remove motor power immediately.

Record displayed heading, pitch_roll, requested rates and any failure reason.
Keep the conservative scale until axis mapping, direction, release and stillness pass.

## Exact change scope

| Files | M09 change |
| --- | --- |
| `src/main.cpp` | Manual state/dispatch/service integration and separate command watchdog; M09 banner/summary. Carriage adds only manual state and clearing its request on abort. |
| `src/manual_control.h` | Manual velocity, admission, stop/reversal, BNO direction/progress/travel supervision and telemetry; optional third JOG value and carriage service within existing step limits. |
| `xbox_control.py`, `requirements.txt` | pygame/pyserial host, mapping, dry run, deadband/curve, arming and fault handling; pinned dependencies. |
| `tests/manual_integration_test.cpp`, `tests/run_host_tests.ps1` | Manual scenarios alongside every inherited test. |
| `tests/stubs/FastAccelStepper.h` | Adds the velocity-update API to the existing simulated motor. |
| `tests/test_xbox_control.py`, `tests/test_host_ui.py` | Input/protocol tests and actual headless UI tests with fake devices. |
| `tests/manual_carriage_test.cpp` | Carriage mapping, rates, release/reversal, endpoints, near-boundary STOP and fault coverage. |
| `tests/m08_baseline_hashes.json`, `tests/verify_m08_baseline.ps1` | Before/after SHA-256 preservation check. |
| `.gitignore`, `README.md`, `VALIDATION.md` | Ignore Python artifacts; M09 instructions and validation. |

Other copied M09 files remain identical to M08. Full M08 command and baseline
documentation remains in [M08 README](../M08_go_to_pose/README.md).
