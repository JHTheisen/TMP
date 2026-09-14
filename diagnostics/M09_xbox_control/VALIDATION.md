# M09 offline validation — 2026-09-13

## Manual carriage extension — current build

Added left-stick horizontal/raw axis 0 through the existing deadband, quadratic
velocity curve, speed scale and 20 ms stream. Right horizontal yaw/raw axis 2 and
left vertical pitch/raw axis 1 are unchanged. `--invert-carriage` reverses only
carriage; `--carriage-axis` permits explicit index selection. Arming requires all
three assigned inputs centered. The host now sends `JOG yaw pitch carriage`;
firmware still accepts two-value JOG as a zero-carriage request.

Production changes are limited to `xbox_control.py`, the extra manual state and
abort-request clearing in `src/main.cpp`, and `src/manual_control.h` protocol,
carriage service, STOP completion and telemetry. Carriage retains DIR21/STEP22,
1000 pulses/s, 1000 pulses/s² and the +/-500 startup-relative generated-step
envelope. At default host scale its maximum requested rate is 250 pulses/s.
It uses a finite boundary target with live speed updates, so it decelerates at
the endpoint without repeated move commands. Release/STOP/reversal use normal
deceleration and stopped observation; when already near an endpoint the existing
finite braking target is retained, because FastAccelStepper stopMove rewrites
that target. A boundary STOP regression reproduced overshoot in the simulated
plant before this correction and now passes in both directions.

The existing watchdogs already include carriage; their implementation is unchanged.
STOP waits for all three motors. Manual still accepts accuracy 0–3; north validity,
absolute POSE/MOVE admission, BNO report selection, signs and calibration are unchanged.
Carriage has no physical position feedback or homing; generated-step limits do not
measure actual rail clearance or missed steps.

- ESP32 build: PASS, RAM 35,180 bytes, flash 362,857 bytes.
- All 95 inherited firmware scenarios, math/watchdog tests and 41 yaw/pitch manual
  scenarios: PASS.
- 21 new carriage scenarios: PASS. Covers independent/simultaneous axes, rate
  changes, release/reversal, STOP, endpoints, near-endpoint STOP at multiple speeds,
  rearming without recentering travel, legacy JOG, malformed commands, stale/invalid/
  wrong-report BNO, blocked BNO, reset, command loss, abort and remaining guards.
- 18 Python tests: PASS, including actual headless host carriage mapping/inversion,
  centered-input admission, disconnect/focus/pause abort and no-serial dry run.
- Python bytecode check and all 23 M08 baseline file hashes: PASS.

New fixture: `tests/manual_carriage_test.cpp`; updated runner and host tests in
`tests/run_host_tests.ps1`, `tests/test_xbox_control.py`, `tests/test_host_ui.py`.
README now describes the three-axis mapping, protocol and controlled first test.
The initial carriage-extension turn performed no upload or hardware action.
The user subsequently requested flashing after the old two-value firmware rejected
the updated client's third JOG value. The matching carriage image was built and
flashed to COM9 with esptool hash verification. At 2026-09-14 02:48 UTC, the real
`xbox_control.ManualSession` generated 32 zero-velocity `JOG 0 0 0` frames accepted
by the board. Three-axis telemetry showed zero commands/rates and carriage steps
0, followed by `MANUAL STOPPED` and `M09 READY` after STOP. COM9 closed with manual
control stopped/disarmed. No nonzero manual commands were sent during this check.
Raw evidence: [JOG protocol capture](audit/JOG_PROTOCOL_20260914T024831Z.log).
The existing JOG protocol was extended with an optional carriage field; no new
MANUAL command was introduced. Build image SHA-256:
`30ae17fea6c59bf7ade544a0b7a24ad2c47a898893233420b4f84d4806885e83`.

## Earlier validation history

Host display follow-up: the user's screenshot showed BASELINE with zero-initialized
error fields, but accuracy/freshness were clipped offscreen. The host now wraps
messages and keeps phase, last BNO accuracy/freshness/sample age, report receipt
age and pitch-only readiness visible. Repeated BUSY polls no longer displace
diagnostic messages. All 16 Python tests and bytecode compilation pass after this
display change. All 23 M08 files still match their hashes. Firmware is unchanged
by this follow-up. Subsequent screenshots identified fresh BNO data with accuracy
0 as the north-readiness blocker; the hardware audit below tested that finding.

The user identifies the earlier M08 hardware behavior as tested and correct.
Initial implementation and UI validation used no hardware. The user subsequently
authorized COM9 access and firmware A/B uploads, confirming powered motors and
clear startup travel. The [hardware audit](audit/HARDWARE_RESULTS.md) records those
tests separately. Powered Xbox motion remains unverified; no commit or push was
performed.

## Authorized BNO startup audit follow-up

- Stock M08 and M09 were captured twice each with matching serial/reset options.
  All four reported accuracy 0 throughout baseline telemetry and pitch-only
  readiness around 21.3 seconds after reset.
- Separate sensor-only wrappers using each stock setup built successfully and
  passed simulated no-motion/abort checks. In 60-second physical captures both
  reached accuracy 1, neither reached 2 or 3.
- Opening COM9 using the Xbox host's default RTS/DTR states produced a fresh M09
  boot banner without an explicit reset command.
- Actual-source synthetic timing tests confirmed the same 20-second deadline
  and late-calibration lockout in both versions.
- Audit tooling, fixtures and timestamped logs were added under `tools/`,
  `tests/` and `audit/`. Production firmware was not edited during this audit.
  Normal M09 was restored after diagnostic uploads. All 23 M08 hashes still pass.

## Manual-only accuracy prerequisite change

At the user's request, `src/manual_control.h` now admits JOG and reports manual
READY after either completed startup path, without requiring north or accuracy
>=2. Manual operations clear only their own yaw/accuracy requirements. Absolute
POSE/MOVE admission and north validity are unchanged. Without north, the manual
stopping margin uses the existing pitch-only startup yaw reference, matching the
unchanged hard travel guard. `xbox_control.py` display text reflects manual 0–3
acceptance. No `src/main.cpp`, shared safety code or M08 source was edited.

Validation: ESP32 build passes (RAM 35,124; flash 361,685 bytes), all 95 inherited
firmware scenarios, math/watchdog tests, 41 manual scenarios and 16 Python tests
pass. Manual tests cover arming/moving at each accuracy 0–3, accuracy transitions,
unverified-north POSE/carriage rejection, and low-accuracy stale/invalid/wrong-report,
reset, input-loss, abort, direction/progress and travel protections. M08's 23-file
preservation check passes. These are simulated motion checks; physical JOG motion
has not been exercised by the agent.

## Initial implementation results

- ESP32 `esp32dev` build: PASS with unchanged pinned dependencies/configuration.
  RAM: 35,124 / 327,680 bytes. Flash: 361,693 / 1,310,720 bytes.
- All 95 inherited firmware scenarios: PASS (40 startup/control, 30 POSE/MOVE,
  25 pitch-readiness). Includes carriage, BNO roll, pitch sign -1, stale/invalid/
  reset, blocked I2C/UART, abort and closed-loop settling cases.
- Original 85 control-math checks, 362 pose-math checks and watchdog tests: PASS.
- 29 new manual firmware scenarios: PASS. Covers centered/no motion, admission,
  readiness, correct independent axes/signs, rates/ceilings/acceleration,
  release/reversal, busy rejection, and POSE/carriage after manual exit.
- Fault cases: PASS for host loss, partial/malformed/late commands, blocked host,
  stale/invalid/wrong BNO reports, blocked BNO calls, accuracy drop, reset, X inside
  a partial line, wrong direction, no progress, angular guards, braking/overall
  deadlines and rejected speed settings.
- 16 Python tests: PASS. Eleven cover deadband/curve, zero release, signs, bounds,
  handshake, deliberate arming/rearming, telemetry loss, reset and fault handling.
  Five run the actual pygame host with dummy video/audio and mocked Xbox/serial,
  checking dry run never opens serial, motion/release/exit, disconnect, focus loss
  and an input-pump stall.
- Python bytecode compilation and `--help`: PASS.
- All 23 M08 files match their pre-edit SHA-256 hashes (excluding build artifacts).
  Copied control/sensor/watchdog/pose math and PlatformIO configuration also match
  M08 byte for byte. Git reports no M08 or shared hardware-header modifications.

The stalled-input UI test found that checking elapsed time only at loop entry
could permit stale input after a blocked input pump. The final host also checks
after input acquisition and before transmission. That regression now aborts
without replaying stale motion commands.

## Reproduction and limits

See README for environment setup. Development used PlatformIO Python 3.11,
pygame 2.6.1 and pyserial 3.5 installed into ignored `.pio/python_deps`, plus
Strawberry GCC with strict C++11 warnings as errors.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests\run_host_tests.ps1
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
$env:PYTHONPATH = (Resolve-Path .pio\python_deps).Path
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m unittest discover -s tests -p 'test_*.py' -v
powershell -NoProfile -ExecutionPolicy Bypass -File tests\verify_m08_baseline.ps1
```

Firmware test output: ignored `.pio/host_test_results.txt`. Compiled firmware:
`.pio/build/esp32dev/firmware.bin`.

Simulated watchdog ticks and idealized acceleration-limited motors test decisions.
The 150/250 ms assertions measure stop-request time in the fixture, not physical
stop latency. Real ESP32 scheduling, motor queues, gearing, torque, backlash,
magnetic conditions and joystick driver mapping need the README powered procedure.
Tiny manual rates can fail progress deadlines if BNO cannot resolve motion; no
limits were relaxed to avoid this.

The installed FastAccelStepper 1.2.7 header was inspected for rate-update and stop
semantics. The upstream [FastAccelStepper API](https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md)
also documents applying changed rates to an active ramp and normal deceleration.
Input architecture was checked against historical source and
[pygame documentation](https://www.pygame.org/docs/ref/sdl2_controller.html).
M09 retains the historical raw joystick API with configurable indices; actual
mapping is explicitly left for the no-serial dry-run check.
