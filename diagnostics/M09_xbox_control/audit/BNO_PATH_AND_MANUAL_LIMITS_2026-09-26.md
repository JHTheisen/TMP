# M09 diagnostics, BNO path and manual restriction audit

This pass instruments the existing control system. The earlier failed test used
old firmware and is resolved by the user's later upload; it is not investigated
as a new firmware defect. The user reports the simplified manual control works.
This diagnostics image was built and tested offline only: no live serial, upload
or hardware movement. M08 and verified physical pitch = BNO ROLL remain unchanged.

## Current settings preserved

| Axis | DIR / STEP | Manual ceiling, pulses/s | Manual acceleration, pulses/s² |
| --- | --- | ---: | ---: |
| Yaw | 32 / 33 | 2000 | 2000 |
| Pitch | 26 / 12 | 1200 | 1200 |
| Carriage | 21 / 22 | 2000 | 2000 |

The user's current Python CLI default is **1.0**, including its help text. This
pass preserves that change. `--speed-scale 0.25` remains available. The standalone
`stick_command` helper's default is still 0.25, but the UI always passes the CLI
value explicitly; it is not a hidden 25% runtime cap. Shared yaw/pitch slew
constants also affect POSE. Precision settings 250 steps/s² and pitch gain 24.0
were not changed. Earlier README/validation values are historical.

## End-to-end BNO path

The application sources are [main.cpp](../src/main.cpp),
[sensor_support.h](../src/sensor_support.h),
[bno_diagnostics.h](../src/bno_diagnostics.h), and
[xbox_control.py](../xbox_control.py). Dependency paths below are under
`.pio/libdeps/esp32dev/Adafruit BNO08x/src/`, pinned version 1.2.7.

1. `setup()` opens Wire1, SDA4/SCL5, 100 kHz, 50 ms Wire timeout; probes address
   0x4A, calls `DiagnosticBno085::begin_I2C`, then `enableReport()`.
   The library's I²C open performs a software reset and a 300 ms delay.
   `DiagnosticBno085` preserves the earlier checked-write wrapper; the existing
   project patch bounds product-ID waiting. None was changed in this pass.
2. `enableReport()` requests **SH2_ROTATION_VECTOR / 0x05**, interval **10,000 µs**.
   No game, geomagnetic or AR/VR report is requested. Reset detection reenables
   the same report. M09 calls no calibration-enable, save/clear DCD, tare or FRS
   routine. Consequently source inspection cannot establish what calibration or
   orientation configuration was previously saved in the physical device.
3. `sh2.c` unpacks SHTP sensor reports and invokes the Adafruit sensor callback.
   Its microsecond timestamp combines host reception timing with SH-2 report
   reference/delay information. It is not UTC. The decoder's separate `delay`
   struct field is not populated in this pinned implementation and is not logged
   as a trustworthy measurement.
4. `sh2_SensorValue.c`, `sh2_decodeSensorEvent`, copies report ID, sequence byte,
   timestamp, and **`status = report[2] & 0x03`**. This categorical field means
   0 unreliable, 1 low, 2 medium, 3 high. It is distinct from the floating-point
   heading-error estimate carried inside the rotation-vector payload.
5. The rotation-vector decoder reads signed 16-bit **i, j, k, real** from offsets
   4, 6, 8 and 10, scales by 2^-14, and returns library floats. Offset 12 scaled by
   2^-12 is `rotationVector.accuracy`, in **radians**. These formats match CEVA's
   [SH-2 manual, sections 6.5.1 and 6.5.18](https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf).
6. Adafruit `getSensorEvent()` sets the output timestamp to zero, services SH-2,
   and returns whether a decoded event was delivered. The callback can overwrite
   that output several times while processing one packet. M09 therefore observes
   the last returned event, not necessarily every report produced by the sensor.
   Sequence gaps reveal missing observations but do not identify why they occurred.
7. `serviceBno()` now snapshots each returned event's metadata **before** existing
   state/reset/type/conversion/heading filters. It reads quaternion payload only
   for report 0x05. Unexpected report status never overwrites the stored rotation
   snapshot or the accepted control accuracy.
8. Existing control acceptance is unchanged: expected report, enabled report,
   usable state/reset outcome, finite nonzero quaternion, then accepted continuous
   heading update. Only then are `orientation = result` and
   `bnoAccuracy = event.status` assigned together. Accuracy is not estimated,
   truncated from a quaternion, mapped to a different scale, or copied from an
   unrelated report. An invalid/ignored report can leave the preceding accepted
   status visible; the new raw snapshot makes that distinction observable.
9. `quaternionToEuler()` uses **w=real, x=i, y=j, z=k**, conventional Z-Y-X Euler
   extraction: yaw about Z, pitch about Y, roll about X. The asin argument is
   normalized by norm² and clamped; the atan2 expressions are homogeneous and
   do not require separate normalization. Outputs are degrees; negative yaw is
   wrapped into 0–360. Its algebra matches the pinned Adafruit
   `examples/quaternion_yaw_pitch_roll` example. Physical gimbal pitch intentionally
   uses **roll**, not Euler pitch. No axis remap, conjugation, north offset or
   new sensor-to-camera transform was added.
10. Every 500 ms `sensorTelemetry()` queues accepted BNO state, raw diagnostics,
    and both encoders. UART draining remains nonblocking. Python separates the
    raw and accepted dictionaries, parses key/value strings without converting
    accuracy, and displays the same status string. The log preserves both the
    received serial text and parsed fields; it does not recompute Euler angles.

**No concrete quaternion order, sign, units or formula bug was found.** Focused
tests cover identity, X/Y/Z rotations, negative yaw wrapping, scaled and sign-
equivalent quaternions, and zero/nonfinite rejection. Those establish software
interpretation, not the actual sensor-to-camera mounting alignment. Euler angle
coupling and the singularity near ±90° Euler pitch still apply. Sensor translation
away from the axis alone is not an explanation for stationary 0/3 status changes.
Physical cause requires the new recorded evidence; none is asserted here.

## Raw, accepted and stale information

| Record | Meaning |
| --- | --- |
| `BNO_STATE` | Last accepted control orientation/status, existing freshness and local age; now also `has_sample`, accepted sequence, library timestamp and MCU receive time |
| `BNO_REPORT` | Last returned report ID/status/sequence/time; cumulative observed event, rotation, other-report, status0/1/2/3, status-change, invalid-vector, reset and report-enable-failure counts; optional telemetry drop count |
| `BNO_RAW` | Last returned **rotation-vector** snapshot: ID, sequence, timestamp, receive time, age, raw library status, `q_w/x/y/z`, separate `heading_accuracy_rad`, norm², conversion validity, yaw/pitch/roll, acceptance and reason |
| `ENCODER_STATE bus=A/B` | Existing raw 12-bit reading, angle, availability, read validity, age, magnet flags and failure count; acquisition unchanged |

Quaternion components use nine significant decimal digits, enough to preserve the
library's binary32 values. This is raw **library output**, not a newly instrumented
I²C byte capture. Norm² is diagnostic only. `BNO_RAW.accepted` describes what
happened when that snapshot arrived, not whether it is still fresh now.
`has_sample` means a sample was seen/accepted at least once since boot; reset and
staleness do not erase that history. Always inspect freshness/ages too.

Before any accepted event, the old numeric accuracy variable is zero. New
`has_sample=NO` makes clear that this is not a measured status zero. On invalid
input/reset/staleness, previously accepted values remain historical; `fresh=NO`,
age and raw rejection reason expose this without changing control policy.
Python's receipt age measures the time since telemetry arrived at the PC; firmware
`age_ms` measures local sample age. Neither should be confused with sensor UTC.

The following comparisons answer the requested diagnostic questions:

- `BNO_RAW has_sample=YES raw_status=0` proves the returned 0x05 library event had
  categorical status zero. If accepted, correlate its sequence/timestamp with
  `BNO_STATE.accuracy`. If rejected, the accepted state can legitimately differ.
- Compare each logged raw serial line with its `parsed=` fields and UI status.
  A mismatch here identifies host interpretation rather than a physical cause.
- Compare quaternion and same-snapshot Euler results, `euler_valid`, norm² and
  acceptance reason. Do not compare a new raw quaternion to an old accepted pose.
- Observed status counters preserve evidence of transient status changes between
  500 ms snapshots. They cannot reconstruct every intermediate quaternion or
  events overwritten inside the library. No full 100 Hz wire capture is claimed.
- A stalled sensor can retain a constant sequence/timestamp while receipt ages
  grow. Control freshness still uses local receipt time; delayed queued packets
  could appear locally fresh. This pass exposes timestamps without adding a new
  rejection rule. Reset/report-failure counters help correlate interruptions.
- `tx_dropped` reports optional lines rejected by the existing 4096-byte buffer.
  It does not count intentional summary-buffer clearing. A PC log cannot recover
  telemetry that never left the board. Latched abort
  still stops normal sensor polling/telemetry through the existing early return.

## Persistent logging behavior

[session_log.py](../session_log.py) creates `logs/m09_<UTC timestamp>_<pid>_<suffix>.log`
with exclusive creation. `--log-dir` selects another directory. Logs are plain
UTF-8 text with UTC and elapsed-time timestamps; every written event is flushed.
The path prints before controller/serial initialization; the pygame UI shows the
filename and logging errors. Exceptions, including serial/startup/input failures,
include tracebacks before best-effort STOP/cleanup. Logging needs no healthy serial
connection. Disk/open failures produce a visible warning without a new interlock.

Records include raw relevant RX plus parsed telemetry, state/rearm/rejection
reasons, focus changes, STOP/abort attempts, protocol events and exceptions.
`TX_ATTEMPT` deliberately means an attempted write, not proof of board receipt.
The existing accepted-command/firmware telemetry provides independent evidence.
Repeated JOG magnitudes/held commands are sampled at 2 Hz; starts, zero transitions
and reversals are immediate. Stable telemetry is sampled at most 2 Hz per record
type/device; quality/availability changes are immediate. Repeated mode/STATUS
heartbeats are coalesced to 5 s. The 20 ms command stream itself is unchanged.
Dry-run logs input only and never opens a serial port.

Flush protects evidence from ordinary Python exceptions/process termination, not
OS power loss or every native crash. No per-frame fsync or logging thread was
added. Slow filesystem I/O can affect the host's existing timing budget; powered
validation should watch actual command cadence. An unavailable disk cannot retain
new evidence, and that condition is explicitly shown rather than silently ignored.

## Complete manual motion restriction inventory

All rows below are currently active unless marked otherwise. File names refer to
M09. “All” means yaw, pitch and carriage. Existing mechanisms were inventoried,
not tightened or removed in this instrumentation pass.

| File / symbol | Current behavior | Axes / reason |
| --- | --- | --- |
| `xbox_control.py` `arguments`, controller selection | Nonnegative controller/axis indices; distinct axes; controller and indices must exist | All; prevents unusable input configuration |
| `arguments`, axis mapping/inversion | Yaw raw2, pitch raw1 with inversion, carriage raw0; explicit inversion switches | Selected axis; routing/sign choice can make a wrongly selected stick appear ineffective |
| `stick_command`, `--deadband` | 0.15 default, allowed 0.05–0.5; exact zero within it | All; suppresses resting stick noise |
| `stick_command` | Rescale outside deadband, square magnitude, clamp to1, multiply by1000×scale, round to integer | All; fine velocity control, saturation/quantization; a small additional quantized zero region exists |
| `arguments`, `--speed-scale` | Current CLI default1.0; allowed (0,1]; helper default0.25 overridden explicitly by main | All; user speed scaling, no hidden second multiplier |
| `stick_command` | Rejects nonfinite selected input | Affects whole host session; unusable input causes existing best-effort STOP |
| `ManualSession.arm`, main | Idle, READY, no pending STOP, focused window, all selected raw axes centered ≥0.5s, deliberate Space/A | All; intentional centered arming; no telemetry-recency gate |
| `ManualSession.frame/receive` | While arming, sends only zero JOG; nonzero requires MANUAL READY acknowledgement | All; mode handshake |
| `frame` | Arming/stopping longer than4s → recoverable STOP/disarm | All; pending handshake timeout |
| `receive` | Rejection, BUSY/mode loss, restart, MANUAL STOPPED/OPERATION FAILED cancel relevant host session; some request STOP | All; rearm after state disagreement; no automatic X |
| `receive` | ABORTED / FINAL RESULT FAIL → host fault | All; reflects board's actual latched state, does not send another X |
| main event processing | Focus loss → STOP/disarm; focus return does not rearm | All; retained operator control behavior |
| main operator events | Space/A stops active/arming; Esc/close exits with STOP; X/controllerB exits with X | All; explicit operator commands |
| main failure/cleanup | Disconnect, input/I/O/incomplete-write exception or Ctrl+C → best-effort STOP and exit | All; independent firmware lease covers failed delivery |
| `SEND_INTERVAL`, loop scheduling | 20ms stream, 5ms GUI sleep, current input reacquired, no catch-up bursts | All; update cadence; host stalls can cause command loss |
| `COMMAND_TIMEOUT`, main | ≥250ms since last host JOG → disarm before nonzero resumes | All; same existing command lease, centered rearm required |
| serial constructor | 115200 baud, nonblocking read,100ms write timeout; open can reset ESP32 | All; transport behavior/latency, not a position guard |
| `diagnostic_lines` | Active RX silence >1s warns only | All; **not a movement prerequisite or stop** |
| RX buffering | >8192 unframed chars discarded through newline; malformed/non-ASCII logged | All; **no direct motor stop**; parser exceptions follow existing exception cleanup |
| `src/control_math.h` `slewSpeed/slewAcceleration` | Yaw2000/2000, pitch1200/1200 | Yaw/pitch; motor rate and acceleration ceilings, also shared with POSE |
| `src/main.cpp` carriage constants | 2000 pulses/s and2000 pulses/s² | Carriage; established user tuning |
| `src/manual_control.h` `serviceManualAxis/Carriage` | Nonzero rate = max(1,round(abs(JOG)×ceiling/1000)); zero bypasses run | All; integer-Hz driver API; no manual40Hz floor |
| `control_math.h` `stepDirection` | Pitch sign−1, yaw default−1; carriage command sign used directly | Axis-specific; verified wiring/motion direction, no feedback check |
| `manualReady` | Control initialized, commandIdle/COMPLETE, watchdog configured, **all3 motors stopped** | All; excludes POSE, braking and latched abort during arming |
| `manualReady`, watchdog clear | Drain optional BNO callback and prior command trip while stopped; transient lock contention can return BUSY | All; prevents an old callback stopping a rearmed session; absent BNO watchdog does not block manual |
| `setup` | Any motor allocation/configuration or command-watchdog initialization failure → latched setup failure | All; required motor/control infrastructure |
| `executeManualCommand` | JOG2/3 integer-valued numeric fields in[-1000,1000]; two fields force carriage0; first packet must be zero; reject while ending | All; valid command protocol. Rejection alone does not immediately stop prior motion; Python reacts and lease still expires |
| `serviceCommands` | Up to32 bytes per call;127-character line, printable input, execute only on CR/LF; malformed/overflow rejected | All; framing/processing budget; backlog can delay valid JOG |
| `serviceCommands` X/x | **Any X/x byte** immediately aborts before tokenization, even inside malformed text | All; retained out-of-band abort quirk, not silently redesigned |
| `MotionWatchdog`, `MANUAL_COMMAND_TIMEOUT_MS` | 250ms valid-command lease, checked by independent10ms timer; old lease checked before accepting late JOG | All; intentional communication-loss stop |
| `executeManualCommand`, `serviceManual` | Lease arms before first nonzero; stationary zero idle needs no lease; zero/STOP disarms only after actual braking handoff | All; receiving zero alone does not prove motion stopped |
| `serviceManualBraking` | Zero or reversal first calls stopMove; wait for actual stopped motor | Each axis; smooth braking/reversal sequencing |
| `PRECISION_OBSERVE_MS` | **100ms stopped observation** after braking | Each axis; retained legacy response delay, also affects reversal |
| `BRAKING_TIMEOUT_MS`, `failManualAxis` | **3000ms** failed braking → local forceStop/request clear/report | Affected axis; later valid JOG may retry, no axis/global latch |
| `failManualAxis` | Unexpected continuous stop or rejected speed/acceleration/run call → same local stop/report | Affected axis; handles driver API failure without blanket abort |
| `serviceManual`, session completion | All motors stopped and braking/observation complete before READY | All; rearm cannot overlap prior motion |
| `stopMotors` | forceStop all,25ms queued-command drain, STEP LOW | All; watchdog/abort stop mechanism, not instantaneous physical braking |
| `beginPose`, manual admission | Mutually exclusive manual and POSE ownership | All; joystick does not override an active POSE |
| `serviceBno`, `encoders.service`, loop | Foreground bounded sensor calls; serial/manual service before/after | All; **no sensor prerequisite**, but slow I²C/SH2 can indirectly delay commands until command watchdog expires |
| `queueText/serviceSerialOutput` |4096byte optional buffer; ≤64byte UART drain per call; full buffer drops diagnostics | All; no direct motor cap; dropped READY can delay host handshake |
| New logging | Flushed host disk I/O; errors warn only | No new motion rule; slow disk can indirectly consume the existing host command budget |

The following retained mechanisms are **not active manual restrictions**:

| File / symbol | Value/behavior | Actual scope |
| --- | --- | --- |
| `main.cpp` `safety` BNO guards | Freshness150ms, accuracy≥2/grace1000ms, yaw±185°, pitch±75° | Angular POSE only; manual exits before these checks |
| `LEG_TIMEOUT_MS` |90s | Finite POSE operation, not continuous JOG |
| `axisProgress` | Error increases>0.6°, direction/progress deadlines | POSE only; loop calls serviceAxes only for poseActive |
| `control_math.h` precision constants |0.4° tolerance,0.1° approach deadband,4/200step bursts,40–1000Hz,250steps/s²,2s slew/15s precision progress,4s burst deadline | POSE; neither manual minimum speed nor manual acceleration |
| `limitedSpeed`, planned synchronization caps | Timing-model rate caps | POSE only; manual uses direct ceiling constants |
| `beginPose` carriage checks | Signed32-bit finite target/delta; estimated finite duration before90s | POSE/MOVE only; continuous JOG has no finite target window |
| `serviceReference`, stable windows |≥1s, sample count/variance/range/drift; fresh BNO; accuracy≥2 for north | Optional reference qualification/POSE availability; never manual arming |
| AS5600 validity/magnet/raw angle | Read/status telemetry | No encoder motor control or manual position guard |
| Removed legacy limits |±500 carriage window, manual BNO direction/progress/envelope, manual90s limit | Absent; no replacement limits added |

Required driver constraints are distinct from the application ceilings:

| File / symbol | Constraint and effect |
| --- | --- |
| `main.cpp` `setDirectionPin(...,true,200)` |200µs direction-settle time, all axes |
| FAS1.2.7 `pd_esp32/esp32_queue.h`, `pd_config.h` |16MHz ticks;80tick minimum step period yields200kHz library ceiling, above application rates |
| FAS `FastAccelStepper.cpp`, ramp configuration | Requires positive acceleration, nonzero/representable legal speed and initialized ramp configuration; rejection produces local axis error |
| FAS `pd_config.h` |32queue entries; minimum command duration3200ticks=200µs; this is **not** a5kHz step-rate cap |
| FAS `FastAccelStepper.h` queue planning |Default forward planning about20ms, task about4ms; forceStop drains queued steps rather than instantly removing all pulses |
| `setup`, FAS resource allocation |All three STEP channels must allocate successfully; driver current/microstepping/mechanical limits are not determined by this source audit |

## NORTH_LEVEL status

`Operation::NORTH_LEVEL` is an enum member, initial operation value and fallback
summary label. It does not establish an available operation. `serviceReference()`
does qualify stationary optional pitch/north references in the background and
never moves motors. Command dispatch accepts STATUS, STOP, JOG, POSE, MOVE and
out-of-band X/x. No N, L, NORTH_LEVEL or equivalent reference command exists, and
Python has no button path for one. Absolute POSE prerequisites remain explicit;
no North + Level operation, calibration movement or automatic startup movement
was added.

## Validation and next physical evidence

See [VALIDATION.md](../VALIDATION.md) for build and test results. Before making
further BNO changes, a future user-authorized powered session should retain the
new log from startup through stationary status changes and small deliberate
single-axis movements. Correlate report/accepted timestamps, raw quaternions,
Euler results, status counters, resets, both encoders and actual motor commands.
Verify the current speed settings, command cadence, STOP/rearm and encoder readouts
on hardware. These observations, especially camera/sensor frame alignment and the
physical cause of changing BNO confidence, cannot be established offline.
