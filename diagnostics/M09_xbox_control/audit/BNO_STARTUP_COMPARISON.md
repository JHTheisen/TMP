# M08 versus M09 BNO startup audit

## Status

Source/dependency audit, synthetic timing tests and the authorized COM9 hardware
A/B are complete on 2026-09-13. The user confirmed motors powered, startup travel
clear and COM9 free. Stock M08 reproduced stock M09's accuracy-0/pitch-only result
on both repeats. This does not establish an M09-specific regression or a physical
setup fault. See [hardware results and capture logs](HARDWARE_RESULTS.md).

## Exact baseline and files

M08 source matches Git HEAD `d0f2553f274aa5fd0b6c8cd274e9398596eabf22`
(`M08 stable closed-loop baseline with corrected BNO pitch-axis mapping`). There
are no M08 working-tree changes; all 23 files match the pre-M09 SHA-256 snapshot.
There is no archived serial trace identifying how quickly accuracy rose during
the earlier successful physical run. The current existing M08 image is identified
in `bno_startup_comparison.json`; matching that image to the historical upload
cannot be proven without an upload record or flash readback.

The sensor wrapper, control math, watchdog, pose math and PlatformIO configuration
are byte-identical. All 198 files in the four installed library directories match:
Adafruit BNO08x 1.2.7 (32), BusIO 1.17.4 (31), Unified Sensor 1.1.15 (8),
FastAccelStepper 1.2.7 (127). Bootloader and partition images also match.

`enableReport`, `handleReset`, `serviceBno`, `stableBaseline` and
`stablePitchBaseline` match exactly. Automated evidence is recorded by
`tools/audit_bno_startup.py` in `bno_startup_comparison.json`.

## Initialization and reset chain in BOTH versions

1. Serial at 115200; 500 ms delay; original GPIO and FastAccelStepper setup.
2. Wire1 SDA4/SCL5 at 100 kHz, 50 ms transaction timeout; probe address 0x4A.
3. `DiagnosticBno085(-1)` means no host-controlled BNO reset GPIO.
4. `begin_I2C` sets I2C HAL callbacks and invokes the identical wrapper `_init`.
5. Adafruit `_init` invokes hardwareReset (no-op for reset pin -1), then
   `sh2_open` -> `shtp_open` -> `i2chal_open`.
6. **i2chal_open sends the BNO executable soft-reset packet `{5,0,1,0,1}`**.
   Up to five write attempts, 30 ms after failed attempts, then 300 ms after success.
7. SH-2 waits for reset notifications, queries product IDs and registers the sensor
   callback. Firmware requests **SH2_ROTATION_VECTOR, 10000 us (100 Hz)**.
8. Only after successful report enabling does firmware set `startedAt`. BNO
   acquisition runs during the 5-second startup wait. Baseline windows then run
   for one second each. BNO startup reset notifications re-enable the same report.

Neither firmware calls `sh2_setCalConfig`, `sh2_getCalConfig`, `sh2_saveDcdNow`,
`sh2_setDcdAutoSave` or `sh2_clearDcdAndReset`; neither changes DCD through FRS.
Thus calibration enablement/persistence relies on the BNO's own defaults/state.
This audit establishes equal host commands, not the actual internal calibration
mask or DCD contents in the physical sensor.

## Does opening serial clear calibration?

The actual `M09_HOST_OPEN` capture recorded a fresh M09 boot banner 0.719 seconds
after opening COM9 with the host's default line states, with no explicit reset.
Thus that opening sequence did reset this ESP32 in the test. Accuracy was first
0 at 1.188 seconds and 1 at 1.938 seconds; it did not reach 2 during that 10-second
capture. A second, full-duration host-open capture is recorded in the hardware report.

The Python host opens pyserial without overriding RTS/DTR. Installed pyserial
3.5 initializes both states true and applies them on Windows port open. ESP32
reset behavior depends on the board's USB/UART reset circuit and line transitions;
an open does not universally mean exactly one reset. If ESP32 resets, the sequence
above soft-resets the BNO as well. Host close during unfinished startup also sends
X; it does not issue a BNO calibration-clear command.

**Restarting sensor fusion/status is not equivalent to erasing saved DCD.** SH-2
reference section 6.4.10 states that a non-power-up reset persists the most recent
RAM DCD to flash. Clearing it requires a distinct sequence. Therefore neither a
soft reset nor accuracy initially reading 0 proves that calibration was erased.
Power cycling and a powered soft reset must be distinguished in the A/B log.

Sources: [Espressif reset circuitry](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html),
[SH-2 reference, section 6.4.10](https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf).
The installed Adafruit source at `src/Adafruit_BNO08x.cpp:287` contains the actual
I2C soft-reset transaction used by these builds.

## Startup timeout: a real shared limitation

Both set `BASELINE_TIMEOUT_MS = 20000` from successful report enabling, including
the five-second startup wait. Acceptance needs a whole stable one-second window
with accuracy >=2 on every accepted sample (at least 30). It does not require 3
or a stepwise 0 -> 1 -> 2 -> 3 progression. Any accuracy 0/1 in that window marks
it unsuitable for north. At the first failed window ending after the deadline,
stable pitch is accepted instead and north stays unauthorized until reset.

The same simulated 0 -> 1 -> 2 -> 3 timelines through actual M08 and M09 source
(with M09 STATUS polling every 500 ms) produced:

| Accuracy first reaches 2 | M08 | M09 |
| --- | --- | --- |
| 10.0 s | North reference at 11.001 s | Same |
| 18.0 s | North reference at 19.001 s | Same |
| 19.5 s | Pitch-only at 20.001 s | Same |
| 22.0 s | Pitch-only at 20.001 s | Same |
| 30.0 s | Pitch-only at 20.001 s | Same |

In all late-recovery cases BNO accuracy was 3 at 40 s, but north remained disabled.
The sensor continues being read in pitch-only idle; **STATE telemetry stops**, so
the Python window shows an explicitly aged last report. A frozen displayed 0 can
therefore hide later accuracy recovery.

These are decision tests, not physical calibration-time measurements. There is
no justified guarantee that 20 seconds is enough on this sensor at this startup.
Manufacturer calibration procedures are motion/status based, not a promise of
convergence within 20 seconds: [BNO085 calibration procedure](https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/assets/component_documentation/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf).
No timeout or safety threshold has been changed during this audit.

## Indirect differences and limits

M09 adds a second 10 ms ESP timer before BNO initialization. It is disarmed during
startup and returns without touching sensors or motors, but still incurs a timer
callback. M09 also adds a different banner and parses/responds to STATUS every
500 ms from the Python host. The host's serial-open/close sequence may differ
from whichever monitor was used for the successful M08 run. These can affect
real timing; equal source paths and host tests cannot rule them out. The hardware
captures found no M08-versus-M09 split in north readiness. Both sensor-only builds
reached accuracy 1, whereas both stock images reported 0 throughout their baseline
telemetry in the matched explicit-reset runs. Reset history, elapsed time and
the observer's different loop workload remain confounders; neither contrast proves
which factor caused accuracy to differ.

## Physical A/B method

Use identical motor power, orientation, sensor power and wiring for each run;
confirm startup travel clearance if motors are powered. Close other serial apps.
Do not run the Xbox host during the initial A/B: its STATUS command is absent in
M08, and arming/exit behavior would make it an unequal test.

The existing stock M08 and M09 firmware images, bootloaders, partitions and hashes
were recorded before testing. The actual sequence was M09 -> M08 -> M08 observer
-> M09 observer -> M08 -> M09, followed by M09 host-open checks. No wiring,
orientation or power-cycle action was requested or performed by the agent.
A repeatable image-dependent
difference would strongly implicate software/startup; a single sequential pair
can still be influenced by sensor calibration history/order effects.

`tools/capture_bno_startup.py` records complete timestamped lines and first observed
accuracy levels, boot banners and terminal readiness. It sends no pose, jog or
calibration commands, and sends X on exit. An explicit RTS reset is optional;
opening or resetting still may trigger the stock automatic north/level motion.

Example after physical setup is confirmed and the corresponding image is installed:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools\capture_bno_startup.py --label M08_A1 --reset rts
```

Use the same capture options for each corresponding image. Stock firmware only
exposes accuracy until completion/pitch-only/abort. The temporary observer in
`tools/bno_observer/main.cpp` includes each unchanged stock source, executes its
exact setup, then substitutes a sensor-only loop with continuous telemetry and X
handling. It never dispatches movement commands or runs automatic control. Both
observer variants passed a 70-second simulated no-motion/abort test, including
injected POSE/MOVE/JOG commands, and built for ESP32 before being installed for
60-second captures. Neither reached accuracy 2 or 3. The observer changes loop
workload, so it is not equivalent to merely extending the stock timeout.

The normal M09 application image was restored after the observers; no production
firmware source, calibration settings, limits or watchdogs were changed by this
audit. All 23 M08 baseline files still match their original hashes.
