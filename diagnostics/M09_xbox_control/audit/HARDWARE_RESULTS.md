# Authorized M08/M09 BNO hardware comparison — 2026-09-13

## Result

**Stock M08 reproduced the low-accuracy/pitch-only result in both repeats.**
Stock M09 did the same. We did not reproduce the earlier successful M08 north
calibration, so these tests do not establish an M09-specific regression or prove
a physical setup fault. The reason accuracy remains below 2 is still unresolved.

Normal M09 is restored. The final capture acknowledged `Reason: Operator X abort`
and `Latched abort; reset required`; COM9 was closed afterward. Its final summary
reported zero slew starts, carriage position/target 0/0, no invalid BNO reports,
one BNO reset notification and no stale-watchdog trip. No Xbox motion was tested.
Reopening serial can reset the board and restart automatic north/level startup.

## Conditions and method

The user explicitly authorized the comparison and confirmed: motors powered,
startup travel clear, COM9 free. COM9 is the CP210x USB/UART connection to the
ESP32. The agent issued no wiring, orientation or power-cycle actions and no
POSE, MOVE, JOG or calibration commands. Stock automatic startup remained enabled.
All recorded motor telemetry was idle at zero requested rates; this is firmware
telemetry, not independent visual observation of the hardware.

Matching stock captures used 115200 baud, inactive RTS/DTR before opening, an
explicit RTS reset, 35 seconds of observation, and X on exit. Uploads replaced
only the application at 0x10000; esptool verified each write. Observer captures
used the same serial/reset options for 60 seconds. A roughly 70-minute interruption
between the initial stock pair and observer pair is visible in UTC timestamps;
sensor calibration history and elapsed time were not controlled across that gap.

The two host-open tests used the Xbox host's default active RTS/DTR settings,
no explicit reset, and STATUS every 500 ms. They sent no manual arming commands.

## Recorded results

Times below are seconds from capture start, not from the firmware's internal
`startedAt`. The approximately 21.3-second observed fallback includes reset/boot
and BNO initialization before the firmware's 20-second deadline begins.
Accuracy values are report status levels, not degree-error estimates.

| UTC start | Image/test and raw log | Accuracy observed | Readiness/result |
| --- | --- | --- | --- |
| 20:31:39 | [Stock M09 B0](M09_B0_20260913T203139Z.log) | 0 only; first 1.281 s | Pitch-only at 21.297 s |
| 20:32:51 | [Stock M08 A1](M08_A1_20260913T203251Z.log) | 0 only; first 1.265 s | Pitch-only at 21.296 s |
| 21:42:47 | [M08 sensor observer](M08_OBSERVE_20260913T214247Z.log) | 0 at 1.266 s; 1 at 2.016 s | Never 2/3 in 60 s; final complete STATE at 59.891 s |
| 21:44:42 | [M09 sensor observer](M09_OBSERVE_20260913T214442Z.log) | 0 at 1.266 s; 1 at 2.016 s | Never 2/3 in 60 s; final complete STATE at 59.250 s |
| 21:46:28 | [Stock M08 A2](M08_A2_20260913T214628Z.log) | 0 only; first 1.282 s | Pitch-only at 21.297 s |
| 21:50:20 | [Stock M09 B1](M09_B1_20260913T215020Z.log) | 0 only; first 1.281 s | Pitch-only at 21.297 s |
| 21:51:10 | [M09 host-open, 10 s](M09_HOST_OPEN_20260913T215110Z.log) | 0 at 1.188 s; 1 at 1.938 s | Fresh boot at 0.719 s; no readiness before exit |
| 21:51:56 | [M09 host-open, 35 s](M09_HOST_OPEN_FULL_20260913T215156Z.log) | 0 only; first 1.172 s | Fresh boot at 0.719 s; pitch-only at 21.188 s; exit abort acknowledged |

Every capture has an adjacent `.json` summary. All four matching stock runs
provided 27 STATE samples through baseline. Stock STATE output stops after
pitch-only readiness, although BNO acquisition continues. Therefore those rows
do not prove accuracy stayed at 0 for the entire 35 seconds. The final host-open
abort summary additionally reported accuracy 0 at the end of that run.

Both observers execute their respective stock setup, including all sensor init
and timer setup, but use the same replacement sensor-only loop. They continuously
report past the stock deadline and never run automatic control or dispatch motion
commands. Their different loop workload means they cannot isolate timeout duration
alone. Both built for ESP32 and passed 70-second simulated no-motion/abort checks.

## Answers supported by the comparison

1. **BNO initialization is the same in the current M08 baseline and M09.** Report
   type, interval, library files, reset transaction, roll mapping and baseline
   functions match. All 23 M08 files remain unchanged. There is no archived serial
   trace/upload record proving the exact calibration state of the earlier
   successful M08 run; this limitation prevents reconstructing that prior state.
2. **Serial opening reset this board in both host-default tests.** Each recorded
   a fresh boot banner without an explicit reset request. Both firmware setup
   paths then issue the same BNO I2C software reset. This restarts the sensor but
   does not demonstrate erasure of saved calibration/DCD. The ESP32 boot ROM's
   `POWERON_RESET` text does not measure whether the separately connected BNO lost
   supply power. Actual BNO internal calibration flags/DCD were not read.
3. **The same timeout can reject late calibration in both versions.** The 20-second
   budget includes the initial 5-second acquisition wait and requires a full
   stable 1-second window with accuracy >=2. Synthetic actual-source tests reject
   accuracy first reaching 2 at 19.5 seconds, despite reaching 3 later. North then
   remains disabled until reset. Neither 60-second sensor observer reached 2, so
   simply changing the timeout is not a demonstrated fix for this hardware run.
4. **No M09 calibration/report/DCD changes were found.** Neither stock firmware
   explicitly configures calibration enablement or saves/clears DCD. M09 adds a
   disarmed command-watchdog timer and host STATUS traffic; those are real timing
   differences, but no M08/M09 readiness split emerged from these tests.
5. **The requested M08 hardware retest was performed twice and did not recover
   accuracy 2–3.** Shared startup behavior, calibration history/internal settings
   and sensor conditions remain possible causes. These captures do not justify
   changing pins, motor signs, BNO axis mapping, accuracy thresholds or watchdogs.

The 10-second host-open run reaching 1 while the repeat remained at 0 also shows
run-to-run variation with the same M09 image. It does not isolate the effect of
RTS/DTR timing or STATUS polling. The next discriminating diagnostic would read
the sensor's actual calibration-enable mask and reset/product information using
the sensor-only observer, before changing any calibration configuration.

See [source audit and manufacturer references](BNO_STARTUP_COMPARISON.md) for the
reset/DCD distinction and detailed timing comparisons.

## Firmware identity and changes made for this audit

| Installed application | SHA-256 |
| --- | --- |
| Stock M08 | `80f6043e62b964b22c4ad7e78244fee4e63aa83f403f4121745ec1792b1456d3` |
| Stock M09, restored at end | `04db93c98349f3e00339ef36a38d2d304ac11d63b308c664115ef46d32520d81` |

The complete source/dependency/image manifest is
[bno_startup_comparison.json](bno_startup_comparison.json). Bootloader and partition
images matched between milestones. No production firmware source was edited.

Added audit files: `tools/audit_bno_startup.py`, `tools/capture_bno_startup.py`,
`tools/bno_observer/main.cpp`, `tests/startup_timing_ab.cpp`,
`tests/bno_observer_test.cpp`, and the reports/logs in `audit/`. README and
VALIDATION now distinguish the completed authorized startup audit from pending
powered Xbox movement tests. Python bytecode checks, timing tests, observer tests
and the 23-file M08 preservation check pass.
