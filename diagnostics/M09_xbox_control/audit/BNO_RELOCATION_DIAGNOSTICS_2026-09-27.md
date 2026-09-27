# BNO relocation comparison: diagnostic implementation

The user reports relocating the BNO085 after both existing powered runs, from a
rigid mounting approximately six inches from the cradle rotation axis to a
mounting directly on the cradle, closer to that axis. Relocation is not evidence
that the reset problem is fixed. No mounting transform or sensor authority changed.

## Evidence boundary

These files are **PRE-RELOCATION**, unchanged:

| Log in `logs/` | SHA-256 |
| --- | --- |
| `m09_20260927T003223.645752Z_16468_0.log` | `1E88FB92B09AD5F17DBE30843A987B0937131F1645C1008F91466083178B26A9` |
| `m09_20260927T052220.433960Z_10152_0.log` | `CFFC2960EB6773E17AE3798B733067D34874E0AF07CB5F363FA031B820C25C09` |

Subsequent powered evidence is **POST-RELOCATION**. This build announces
`BNO_DIAGNOSTICS revision=relocation-1 comparison=POST_RELOCATION_USER_REPORTED`.
That is experiment metadata based on the user's statement, not automatic
mounting detection. There has been no post-relocation powered test by the agent.
Old `sensor_us` fields remain defective evidence; use their MCU receive ages and
host reception timestamps. Do not compare their absolute values to corrected ones.

## Exact implementation scope

- `tools/patch_bno_diagnostics.py` and `platformio.ini`: reproducible, anchor-checked,
  idempotent local patches to the existing pinned Adafruit BNO08x 1.2.7 dependency.
  No dependency update. Instrument I2C header/payload reads and writes, individual
  SH-2 reset-complete notifications, unsolicited initialize responses, and complete
  product-ID responses. Preserve the earlier product-ID timeout patch.
- The HAL now supplies `*t_us` at successful I2C header receipt using `micros()`.
  This polling implementation has no interrupt timestamp; the field is a host
  receive-time approximation, not the sensor's internal uptime.
- SH-2 `touSTimestamp()` now extends the host clock before applying the signed
  report offset. This prevents an additional false 2^32 epoch when an offset
  crosses the 32-bit clock boundary. An unrepresentable negative boot time clamps
  to zero. Existing timestamp consumers in motor freshness/control are unchanged.
- `src/bno_lifecycle_diagnostics.h` and `src/main.cpp`: bounded observation-only
  trace records, cumulative transaction statistics, acquisition/service timing,
  and motor/freshness/encoder context. No printing or I2C inside callbacks.
- Reset-cause queries send the existing SH-2 product-ID request once per observed
  runtime reset, deferred until COMPLETE, disarmed, outside POSE, with all motors
  stopped. A small local SH-2 entry point sends the request without starting a
  blocking response operation. Replies are captured by normal acquisition.
  There is no synchronous `sh2_getProdIds()` wait in the runtime loop. The write
  itself retains the existing transport timeout/recovery behavior.
- `src/encoder_acquisition.h`: adds only a read-plus-failure count accessor so the
  caller can time actual Bus-B service attempts. Register reads, cadence, bus
  configuration, and encoder behavior are unchanged.
- `src/bno_diagnostics.h`: labels quaternion data `MALFORMED` when norm squared is
  nonfinite or outside [0.95, 1.05], or heading uncertainty is negative/nonfinite.
  `PLAUSIBLE` is a mathematical/data-quality label, not a calibration claim.
  Status 0 with a unit quaternion can therefore still be `PLAUSIBLE`.
  The existing conversion/acceptance path remains unchanged. `accepted=YES` can
  coexist with `diagnostic_quality=MALFORMED`; that explicitly describes existing
  control handling rather than implying healthy data. Immediate MALFORMED_RV
  traces retain norm, uncertainty, sequence and status even between 500 ms samples.
- `xbox_control.py`: displays `diagnostic_quality` beside existing raw BNO fields.
  `session_log.py`: retains quality transitions immediately; new trace records
  already use the persistent logger's unsampled path. X/B exception/exit behavior
  remains unchanged. Python runtime speed scale remains 1.0.
- Tests: new C++ trace/scheduling tests and actual-library-routine compilation
  tests; the full runner includes them. Arduino/BNO fixtures gained only the
  diagnostic clock/request support; logging tests cover immediate preservation.

No changes to report 0x05/100 Hz, pins, yaw/pitch mapping, verified BNO roll feedback,
motion speeds/accelerations, joystick routing, POSE authority, watchdog decisions,
safety/abort behavior, or AS5600 acquisition. M08 is unchanged. No flash/serial
connection/hardware action was performed.

## Reading the new records

Every `BNO_TRACE` contains:

- `at_ms`, `at_us`: MCU callback/observation time. `at_us` wraps as a uint32;
  `at_ms` disambiguates it. Host log time may be later because UART is queued.
- `start_us`, `duration_us`: operation interval where applicable, modulo-32-bit
  elapsed arithmetic. Context is sampled when the trace is recorded, not necessarily
  at the beginning of a stalled transaction.
- `reset_event`: individual SH-2 callback count, separate from existing `resets`
  (consumed Boolean reset flags).
- `bno_age_ms`, `fresh`, `raw_status`, `seq`, `norm_sq`, `heading_accuracy_rad`:
  current diagnostic/control snapshot. Reset/I2C records can refer to the last
  pre-event quaternion; their freshness/age must be considered.
- `phase`, `manual`, `pose`, `jog=yaw/pitch/carriage`, `motor_mHz=yaw/pitch/carriage`.
- `encB_attempt_ms`, `encB_age_ms`, `encB_failures` and most recently completed
  BNO transaction's `last_io_start_us`, `last_io_duration_us`, `last_io_stage`,
  `last_io_ok`.

The compact `a/b/c/result` fields are defined by `kind`:

| kind | a | b | c | result |
| --- | --- | --- | --- | --- |
| RESET_COMPLETE | SHTP packet receive timestamp (us) | 0 | 0 | 0 |
| RESET_CONSUMED | existing consumed-reset counter | 0 | 0 | 0 |
| I2C | stage: 1 header read, 2 payload read, 3 write, 4 startup reset write | requested bytes | 0 | BusIO success Boolean |
| REPORT_ENABLE | report ID | report interval us | 0 | enable success Boolean |
| ACQUIRE_SLOW | 0 | 0 | 0 | getSensorEvent returned an event |
| ENCODER_B | last stored status | last stored raw angle | cumulative failures | most recent read valid |
| MALFORMED_RV | sequence | raw SH-2 status | 0 | 0 |
| UNSOLICITED_INIT | 0 | 0 | 0 | 0; start_us is packet receive time |
| PRODUCT_ID | raw reset cause | software part number | software build number | 0; start_us is packet receive time |
| PRODUCT_QUERY_SENT | requested reset generation | 0 | 0 | SH-2 send return code |
| PRODUCT_QUERY_WINDOW_END | requested generation | current generation | received response count | 0 if at least one response, -6 if none |

`cause` translates product responses: 0 UNSPECIFIED, 1 POWER_ON, 2 INTERNAL,
3 WATCHDOG, 4 EXTERNAL, 5 OTHER; unknown numeric values remain UNKNOWN and the raw
value is retained. This is the sensor's reported latest reset cause, not independent
proof of an electrical mechanism. The startup library deliberately software-resets
the BNO before its normal product-ID query. Do not interpret that startup cause as
evidence of a spontaneous runtime reset.

No query runs during manual/POSE movement. Several resets before an idle query can
therefore leave earlier causes unknown. A reset overlapping a response window is
visible through differing generation numbers; do not assign all replies to the
older reset. A response count is not a guarantee that every product entry arrived.
Failure/unavailability remains explicit; no fabricated cause or unlimited retries.
An explicit latched abort retains the existing early return, so further sensor
servicing/query completion is unavailable until reset.

I2C failures and transactions/acquisitions >=10,000 us are traced. Ordinary short
successful transactions only update counters/maxima. Slow Bus-B encoder attempts
are traced; repeated encoder failures without long duration are limited to 2 Hz.
`BNO_IO` at 2 Hz reports totals/maxima, query generations, and `trace_dropped`.
The ring holds 32 records and retains queued records under UART backpressure;
overflow increments `trace_dropped` rather than blocking motion. Existing UART
queue overflow remains `tx_dropped`. Neither counter means zero I2C errors.

BusIO exposes only success/failure, not the numeric ESP-IDF error. Correlate the
trace's operation interval with the existing `[...][E][Wire.cpp...]` error line
(263 means timeout). Framework error output can still interrupt queued telemetry
lines; retain the original raw log and check for interleaved/truncated lines. This
patch does not change the Arduino framework or its UART/error behavior.

## Validation and comparison limits

Complete `tests/run_host_tests.ps1`: firmware math/encoder/observer/diagnostic
units, all existing lifecycle/manual/carriage/POSE/M08 regression scenarios, Python
and headless pygame tests, and the 23-file M08 hash check. New cases cover distinct
reset notifications, clock/offset/wrap, failed HAL reads, asynchronous request
payload/busy/error behavior, product cause extraction and truncation, idle-only
query scheduling, unavailable replies, UART backpressure, malformed labels, and
unchanged motor commands. Results are recorded in VALIDATION.md after the final run.

The next powered run must identify this diagnostic revision and preserve its
complete log, including the startup banner. Compare duration-normalized reset/I2C
failure rates, timing/order of stalls and reset events, cause codes, stationary
versus moving intervals, and post-reset status recovery. Old and new locations also
differ in this diagnostic revision; added trace work and idle query traffic are
explicit experimental differences. A successful new run alone does not prove
relocation caused the improvement. No new powered evidence exists yet.

For that future user-operated run, use the existing normal STOP/disarm and leave
the client connected for at least two seconds while fully idle before X/B or
closing it. This gives deferred product-ID requests and queued diagnostics an
opportunity to complete. An explicit X/B abort still latches the firmware and
closes the Python display as before; this change does not alter that path.
