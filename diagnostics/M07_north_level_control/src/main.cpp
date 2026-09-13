#include <Arduino.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <FastAccelStepper.h>
#include <Wire.h>
#include "hardware_config.h"
#include "sensor_support.h"
#include "control_math.h"
#include "motion_watchdog.h"

namespace {
using milestone4::BnoHealth;
using milestone4::DiagnosticBno085;
using milestone4::EulerAngles;
using milestone4::quaternionToEuler;
using namespace milestone7;
constexpr uint32_t STARTUP_MS = 5000, BASELINE_MS = 1000, BASELINE_TIMEOUT_MS = 20000;
constexpr uint32_t BNO_STALE_MS = 150, WINDOW_GAP_MS = 100, SETTLE_MS = 1000;
constexpr uint32_t LEG_TIMEOUT_MS = 90000, PROGRESS_TIMEOUT_MS = 15000;
constexpr double RELATIVE_LIMIT_DEG = 185.0, MAX_USABLE_PITCH_DEG = 75.0;

enum class Phase { STARTUP, BASELINE, MOVING, SETTLING, COMPLETE, ABORTED };
enum class Motion { PRECISION, SLEW, BRAKING, HOLD };
struct Axis {
    FastAccelStepper *motor = nullptr;
    bool pitch = false, settled = false, confirmed = false;
    double current = 0, target = 0, error = 0, bestError = 1e9, progressError = 1e9;
    Motion motion = Motion::PRECISION;
    bool finiteActive = false, observing = false, slewFinished = false, velocityReady = false;
    double initialError = 0, velocityPosition = 0, angularSpeed = 0, peakAngularSpeed = 0, brakeAtDeg = 0;
    int slewDirection = 0;
    uint32_t bursts = 0, slewStarts = 0, motionSamples = 0, lastProgress = 0;
    uint32_t commandAt = 0, stoppedAt = 0, velocityAt = 0;
};
struct Window {
    bool active = false, interrupted = false;
    uint32_t started = 0, lastBno = 0;
    Statistics heading, pitch, firstHeading, secondHeading, firstPitch, secondPitch;
};
FastAccelStepperEngine engine;
FastAccelStepper *yawMotor = nullptr, *pitchMotor = nullptr;
DiagnosticBno085 bno;
MotionWatchdog motionWatchdog;
AccuracyGrace accuracyGrace;
BnoHealth bnoHealth;
EulerAngles orientation = {0, 0, 0};
HeadingTracker heading;
Axis yawAxis, pitchAxis;
Window window;
Phase phase = Phase::ABORTED;
bool finalPrinted = false, referenceSet = false, bnoInitialized = false, reportEnabled = false, bnoValid = false;
bool northUsable = false, concurrentMotion = false, sawYawMotion = false, sawPitchMotion = false;
uint8_t bnoAccuracy = 0;
double northTargetContinuous = 0, baselineHeading = 0, baselinePitch = 0;
uint32_t startedAt = 0, controlStartedAt = 0, lastBnoGood = 0, lastReportAttempt = 0, lastDisplay = 0, bnoMaxGap = 0;
uint32_t lastControlSample = 0, settleStarted = 0, settleSamples = 0, settleLastSample = 0;
uint32_t invalidVectors = 0, bnoResets = 0, reportFailures = 0, accuracyGood = 0;
char txBuffer[1536];
size_t txLength = 0, txOffset = 0;

// Never wait for UART space while active. Drop optional diagnostics if the
// buffer fills; terminal summaries print only after the motors are stopped.
void queueText(const char *line) {
    const size_t length = strlen(line);
    if (txOffset == txLength) txOffset = txLength = 0;
    if (length <= sizeof(txBuffer) - txLength) {
        memcpy(txBuffer + txLength, line, length); txLength += length;
    }
}
void serviceSerialOutput() {
    const int available = Serial.availableForWrite();
    if (available <= 0 || txOffset == txLength) return;
    const size_t count = std::min(txLength - txOffset, static_cast<size_t>(std::min(available, 64)));
    txOffset += Serial.write(reinterpret_cast<const uint8_t *>(txBuffer + txOffset), count);
}
const char *motionText(const Axis &axis) {
    switch (axis.motion) {
    case Motion::PRECISION: return axis.confirmed ? "PRECISION" : "PROBE";
    case Motion::SLEW: return "SLEW";
    case Motion::BRAKING: return "BRAKING";
    case Motion::HOLD: return "HOLD";
    }
    return "UNKNOWN";
}
void setMotion(Axis &axis, Motion motion) {
    axis.motion = motion;
    char line[160];
    snprintf(line, sizeof(line), "AXIS %s -> %s error_deg=%.3f brake_at_deg=%.3f\n",
        axis.pitch ? "PITCH" : "YAW", motionText(axis), axis.error, axis.brakeAtDeg);
    queueText(line);
}

const char *phaseText() {
    switch (phase) { case Phase::STARTUP: return "STARTUP"; case Phase::BASELINE: return "BASELINE";
    case Phase::MOVING: return "MOVING"; case Phase::SETTLING: return "SETTLING";
    case Phase::COMPLETE: return "COMPLETE"; case Phase::ABORTED: return "ABORTED"; }
    return "UNKNOWN";
}
bool fresh(uint32_t now) { return bnoValid && reportEnabled && now - lastBnoGood < BNO_STALE_MS; }
void stopMotors() {
    if (yawMotor) yawMotor->forceStop();
    if (pitchMotor) pitchMotor->forceStop();
    delay(25);
    digitalWrite(tmp_hardware::YAW_STEP_PIN, LOW);
    digitalWrite(tmp_hardware::PITCH_STEP_PIN, LOW);
    digitalWrite(tmp_hardware::CARRIAGE_STEP_PIN, LOW);
}
void finish(bool passed, const char *reason) {
    if (finalPrinted) return;
    stopMotors(); motionWatchdog.disarm(); finalPrinted = true; window.active = false;
    if (txLength > txOffset) Serial.write(reinterpret_cast<const uint8_t *>(txBuffer + txOffset), txLength - txOffset);
    txOffset = txLength = 0;
    phase = passed ? Phase::COMPLETE : Phase::ABORTED;
    Serial.println("=== MILESTONE 7 NORTH / LEVEL SUMMARY ===");
    Serial.print("BNO heading_deg="); Serial.print(orientation.heading, 3);
    Serial.print(" accuracy="); Serial.print(bnoAccuracy);
    Serial.print(" north_usable="); Serial.println(northUsable ? "YES" : "NO");
    Serial.print("Target heading_deg=0 pitch_deg=0; baseline heading/pitch=");
    Serial.print(baselineHeading, 3); Serial.print('/'); Serial.println(baselinePitch, 3);
    Serial.print("YAW settled="); Serial.print(yawAxis.settled ? "YES" : "NO");
    Serial.print(" error_deg="); Serial.print(yawAxis.error, 3); Serial.print(" bursts="); Serial.println(yawAxis.bursts);
    Serial.print("PITCH settled="); Serial.print(pitchAxis.settled ? "YES" : "NO");
    Serial.print(" error_deg="); Serial.print(pitchAxis.error, 3); Serial.print(" bursts="); Serial.println(pitchAxis.bursts);
    Serial.print("Slew starts yaw/pitch="); Serial.print(yawAxis.slewStarts); Serial.print('/'); Serial.println(pitchAxis.slewStarts);
    Serial.print("Simultaneous two-axis motion occurred="); Serial.println(concurrentMotion ? "YES" : "NO");
    Serial.print("Both axes reached and settled="); Serial.println(yawAxis.settled && pitchAxis.settled ? "YES" : "NO");
    Serial.print("BNO fresh/invalid/resets/report_failures/write_failures=");
    Serial.print(bnoHealth.freshSamples); Serial.print('/'); Serial.print(invalidVectors); Serial.print('/');
    Serial.print(bnoResets); Serial.print('/'); Serial.print(reportFailures); Serial.print('/'); Serial.println(DiagnosticBno085::writeFailures);
    Serial.print("Maximum BNO acquisition_gap_ms="); Serial.println(bnoMaxGap);
    Serial.print("BNO accuracy low episodes/recoveries/longest_ms=");
    Serial.print(accuracyGrace.episodes); Serial.print('/'); Serial.print(accuracyGrace.recoveries); Serial.print('/');
    Serial.println(std::max(accuracyGrace.longestMs, accuracyGrace.age(millis())));
    Serial.print("Independent BNO stale watchdog tripped="); Serial.println(motionWatchdog.tripped() ? "YES" : "NO");
    Serial.print("Reason: "); Serial.println(reason);
    Serial.print("FINAL RESULT: "); Serial.println(passed ? "PASS" : "FAIL");
    Serial.println("Latched idle; reset starts a new test. X/x aborts without automatic retry."); Serial.flush();
}
void abortTest(const char *reason) { finish(false, reason); }
void safety() {
    if (!referenceSet || finalPrinted) return;
    const uint32_t now = millis(); bnoMaxGap = std::max(bnoMaxGap, now - lastBnoGood);
    if (motionWatchdog.tripped() || now - lastBnoGood >= BNO_STALE_MS) abortTest("BNO085 feedback stale: acquisition gap reached 150 ms");
    else if (accuracyGrace.expired(now)) abortTest("BNO085 accuracy below 2 continuously for 1000 ms");
    else if (bnoValid && fabs(heading.continuous - northTargetContinuous) >= RELATIVE_LIMIT_DEG)
        abortTest("Measured yaw travel guard exceeded +/-185 degrees from continuous north target");
    else if (bnoValid && fabs(orientation.pitch) >= MAX_USABLE_PITCH_DEG)
        abortTest("Measured pitch reached +/-75 degree absolute guard");
    else if (now - controlStartedAt >= LEG_TIMEOUT_MS)
        abortTest("Two-axis control timeout");
}
bool enableReport() {
    lastReportAttempt = millis(); reportEnabled = bno.enableReport(SH2_ROTATION_VECTOR, milestone4::BNO_REPORT_INTERVAL_US);
    if (!reportEnabled) ++reportFailures;
    return reportEnabled;
}
bool handleReset() {
    if (!bno.wasReset()) return false;
    ++bnoResets; bnoValid = false; window.interrupted = true;
    if (referenceSet) abortTest("BNO085 reset after north reference: orientation continuity lost");
    else { heading = {}; northUsable = false; enableReport(); }
    return true;
}
void serviceBno() {
    if (!bnoInitialized) return;
    handleReset(); if (finalPrinted) return;
    if (!reportEnabled && millis() - lastReportAttempt >= 500) enableReport();
    sh2_SensorValue_t event = {};
    const bool gotEvent = bno.getSensorEvent(&event), reset = handleReset();
    safety();
    if (finalPrinted || reset || !gotEvent || !reportEnabled || event.sensorId != SH2_ROTATION_VECTOR) return;
    EulerAngles result;
    if (!quaternionToEuler(event.un.rotationVector, result)) { ++invalidVectors; return; }
    if (!heading.update(result.heading)) { abortTest("Ambiguous/nonfinite BNO heading transition"); return; }
    const uint32_t now = millis();
    bnoHealth.recordFresh(now, referenceSet); orientation = result; bnoAccuracy = event.status;
    bnoValid = true; lastBnoGood = now;
    if (referenceSet) {
        motionWatchdog.recordFresh(now);
        const bool wasLow = accuracyGrace.low;
        accuracyGrace.observe(event.status, now);
        northUsable = event.status >= BNO_MIN_ACCURACY;
        if (!wasLow && accuracyGrace.low) queueText("BNO ACCURACY LOW: grace=1000 ms; stale deadline remains 150 ms\n");
        if (wasLow && !accuracyGrace.low) queueText("BNO ACCURACY RECOVERED: continuous low timer cleared\n");
    }
    safety(); if (finalPrinted) return;
    if (yawMotor && yawMotor->isRunning()) { ++yawAxis.motionSamples; sawYawMotion = true; }
    if (pitchMotor && pitchMotor->isRunning()) { ++pitchAxis.motionSamples; sawPitchMotion = true; }
    if (yawMotor && yawMotor->isRunning() && pitchMotor && pitchMotor->isRunning()) concurrentMotion = true;
    if (window.active) {
        if (event.status < BNO_MIN_ACCURACY) window.interrupted = true;
        else ++accuracyGood;
        if (now - window.lastBno > WINDOW_GAP_MS) window.interrupted = true;
        window.lastBno = now; window.heading.add(heading.continuous); window.pitch.add(result.pitch);
        if (now - window.started < BASELINE_MS / 2) { window.firstHeading.add(heading.continuous); window.firstPitch.add(result.pitch); }
        else { window.secondHeading.add(heading.continuous); window.secondPitch.add(result.pitch); }
    }
}
void startBaseline() { window = {}; accuracyGood = 0; window.active = true; window.started = window.lastBno = millis(); }
bool stableBaseline() {
    const uint32_t now = millis();
    return !window.interrupted && window.heading.n >= 30 && window.pitch.n >= 30 && accuracyGood >= 30 &&
        window.firstHeading.n && window.secondHeading.n && window.firstPitch.n && window.secondPitch.n &&
        now - window.lastBno <= WINDOW_GAP_MS && fresh(now) && bnoAccuracy >= BNO_MIN_ACCURACY && window.heading.sd() <= 0.15 &&
        window.pitch.sd() <= 0.15 && window.heading.range() <= 0.5 && window.pitch.range() <= 0.5 &&
        fabs(window.secondHeading.mean() - window.firstHeading.mean()) <= 0.2 &&
        fabs(window.secondPitch.mean() - window.firstPitch.mean()) <= 0.2;
}
bool axisProgress(Axis &axis, double error) {
    const double magnitude = fabs(error); const uint32_t now = millis(); axis.error = error;
    if (magnitude > axis.bestError + 0.6) { abortTest(axis.pitch ? "Pitch wrong-direction/runaway guard" : "Yaw wrong-direction/runaway guard"); return false; }
    // Compare cumulative response to the fixed run start, not bestError, which
    // advances every sample and used to leave gradual yaw motion in trials forever.
    if (!axis.pitch && axis.bursts && !axis.confirmed && magnitude <= axis.initialError - DIRECTION_RESPONSE_DEG) {
        axis.confirmed = true;
        queueText("YAW DIRECTION CONFIRMED: cumulative measured heading moved toward north\n");
    }
    axis.bestError = fmin(axis.bestError, magnitude);
    if (magnitude <= axis.progressError - 0.15) { axis.progressError = magnitude; axis.lastProgress = now; }
    if (magnitude <= TOLERANCE_DEG) axis.lastProgress = now;
    const uint32_t timeout = axis.motion == Motion::SLEW ? SLEW_PROGRESS_TIMEOUT_MS : PROGRESS_TIMEOUT_MS;
    if (magnitude > TOLERANCE_DEG && now - axis.lastProgress >= timeout) {
        abortTest(axis.pitch ? "No measured pitch progress: axis deadline expired" : "No measured yaw progress: axis deadline expired"); return false;
    }
    return true;
}
void updateVelocity(Axis &axis, uint32_t now) {
    if (!axis.velocityReady) {
        axis.velocityReady = true; axis.velocityAt = now; axis.velocityPosition = axis.current;
    } else if (now - axis.velocityAt >= VELOCITY_WINDOW_MS) {
        axis.angularSpeed = (axis.current - axis.velocityPosition) * 1000.0 / (now - axis.velocityAt);
        // Retain the peak through this run. Noise can make us brake early, but
        // a late or temporarily flattened BNO velocity must not shrink the margin.
        axis.peakAngularSpeed = fmax(axis.peakAngularSpeed, fabs(axis.angularSpeed));
        axis.velocityPosition = axis.current; axis.velocityAt = now;
    }
    axis.brakeAtDeg = brakingThreshold(axis.pitch, axis.peakAngularSpeed);
}
void commandAxis(Axis &axis) {
    safety();
    if (finalPrinted || !fresh(millis()) || motionWatchdog.tripped() || axis.motor->isRunning()) return;
    if (axis.confirmed && !axis.slewFinished &&
        fabs(axis.error) > axis.brakeAtDeg + SLEW_ENTRY_HYSTERESIS_DEG) {
        axis.slewDirection = stepDirection(axis.error, axis.pitch);
        if (axis.motor->setAcceleration(slewAcceleration(axis.pitch)) != 0 ||
            axis.motor->setSpeedInHz(slewSpeed(axis.pitch)) != 0 ||
            (axis.slewDirection > 0 ? axis.motor->runForward() : axis.motor->runBackward()) != MOVE_OK) {
            abortTest(axis.pitch ? "FastAccelStepper pitch slew rejected" : "FastAccelStepper yaw slew rejected"); return;
        }
        ++axis.slewStarts; axis.commandAt = millis(); axis.lastProgress = millis();
        setMotion(axis, Motion::SLEW);
    } else {
        const int32_t steps = correctionSteps(axis.error, axis.confirmed, axis.pitch);
        if (!steps) return;
        if (axis.motor->setAcceleration(ACCELERATION) != 0 ||
            axis.motor->setSpeedInHz(correctionSpeed(axis.error, axis.confirmed)) != 0 ||
            axis.motor->move(steps) != MOVE_OK) {
            abortTest(axis.pitch ? "FastAccelStepper pitch correction rejected" : "FastAccelStepper yaw correction rejected"); return;
        }
        ++axis.bursts; axis.finiteActive = true; axis.commandAt = millis();
    }
    // If the independent watchdog raced this command, stop again immediately.
    safety();
}
void serviceAxis(Axis &axis, uint32_t now) {
    updateVelocity(axis, now);
    if (axis.motion == Motion::SLEW) {
        if (!axis.motor->isRunning()) { abortTest("Continuous slew stopped unexpectedly"); return; }
        if (fabs(axis.error) <= axis.brakeAtDeg || stepDirection(axis.error, axis.pitch) != axis.slewDirection) {
            // stopMove keeps the slew acceleration already applied by run*().
            // Do not change rates or issue a reversal until isRunning is false.
            axis.motor->stopMove(); axis.commandAt = now; axis.slewFinished = true;
            setMotion(axis, Motion::BRAKING);
        }
        return;
    }
    if (axis.motion == Motion::BRAKING || axis.finiteActive) {
        if (axis.motor->isRunning()) {
            const uint32_t timeout = axis.motion == Motion::BRAKING ? BRAKING_TIMEOUT_MS : BURST_TIMEOUT_MS;
            if (now - axis.commandAt >= timeout) abortTest("Axis braking or finite correction timed out");
            return;
        }
        axis.finiteActive = false; axis.observing = true; axis.stoppedAt = now;
        if (axis.motion == Motion::BRAKING) setMotion(axis, Motion::PRECISION);
        return;
    }
    // Called on fresh BNO samples only: this is a fresh observation after stop,
    // with a short nonblocking allowance for the mechanical response to settle.
    if (axis.observing) {
        if (now - axis.stoppedAt < PRECISION_OBSERVE_MS) return;
        axis.observing = false;
    }
    if (axis.motion == Motion::HOLD) {
        if (fabs(axis.error) <= TOLERANCE_DEG) return;
        axis.settled = false; setMotion(axis, Motion::PRECISION);
    }
    if (fabs(axis.error) <= APPROACH_DEADBAND_DEG) {
        setMotion(axis, Motion::HOLD); return;
    }
    commandAxis(axis);
}
void serviceAxes() {
    if (!referenceSet || !fresh(millis())) return;
    // Cached values are useful for telemetry, but never constitute a new
    // control decision, direction confirmation, velocity or settling sample.
    if (lastControlSample == bnoHealth.freshSamples) return;
    lastControlSample = bnoHealth.freshSamples;
    const uint32_t now = millis();
    yawAxis.current = heading.continuous; yawAxis.error = northTargetContinuous - yawAxis.current;
    pitchAxis.current = orientation.pitch; pitchAxis.error = -pitchAxis.current;
    if (!axisProgress(yawAxis, yawAxis.error) || !axisProgress(pitchAxis, pitchAxis.error)) return;
    serviceAxis(yawAxis, now); if (finalPrinted) return;
    serviceAxis(pitchAxis, now); if (finalPrinted) return;
    const bool stoppedInTolerance = yawAxis.motion == Motion::HOLD && pitchAxis.motion == Motion::HOLD &&
        !yawMotor->isRunning() && !pitchMotor->isRunning() &&
        fabs(yawAxis.error) <= TOLERANCE_DEG && fabs(pitchAxis.error) <= TOLERANCE_DEG &&
        bnoAccuracy >= BNO_MIN_ACCURACY;
    if (!stoppedInTolerance) { phase = Phase::MOVING; settleSamples = 0; return; }
    if (!settleSamples || now - settleLastSample > WINDOW_GAP_MS) {
        settleStarted = now; settleSamples = 0;
    }
    phase = Phase::SETTLING; ++settleSamples; settleLastSample = now;
    if (now - settleStarted >= SETTLE_MS && settleSamples >= 30) {
        yawAxis.settled = pitchAxis.settled = true;
        const bool evidence = sawYawMotion && sawPitchMotion && concurrentMotion;
        finish(evidence, evidence ? "Both measured axes stopped and settled at magnetic north and level" :
            "Targets settled, but simultaneous two-axis motion was not demonstrated");
    }
}
void telemetry() {
    const uint32_t now = millis();
    const bool yawRunning = yawMotor && yawMotor->isRunning(), pitchRunning = pitchMotor && pitchMotor->isRunning();
    char line[650];
    snprintf(line, sizeof(line),
        "STATE %s heading=%.3f yaw_error=%.3f pitch=%.3f pitch_error=%.3f "
        "yaw_mode=%s pitch_mode=%s yaw_motor=%s pitch_motor=%s concurrent=%s "
        "yaw_hz=%.1f pitch_hz=%.1f yaw_dps=%.2f pitch_dps=%.2f brake_deg=%.2f/%.2f "
        "BNO_fresh=%s age_ms=%lu max_gap_ms=%lu accuracy=%u accuracy_low_ms=%lu north_usable=%s\n",
        phaseText(), orientation.heading, yawAxis.error, orientation.pitch, pitchAxis.error,
        motionText(yawAxis), motionText(pitchAxis), yawRunning ? "MOVING" : "IDLE", pitchRunning ? "MOVING" : "IDLE",
        yawRunning && pitchRunning ? "YES" : "NO",
        yawMotor ? yawMotor->getCurrentSpeedInMilliHz() / 1000.0 : 0,
        pitchMotor ? pitchMotor->getCurrentSpeedInMilliHz() / 1000.0 : 0,
        yawAxis.angularSpeed, pitchAxis.angularSpeed, yawAxis.brakeAtDeg, pitchAxis.brakeAtDeg,
        fresh(now) ? "YES" : "NO", static_cast<unsigned long>(now - lastBnoGood),
        static_cast<unsigned long>(bnoMaxGap), bnoAccuracy, static_cast<unsigned long>(accuracyGrace.age(now)),
        northUsable ? "YES" : "NO");
    queueText(line);
}
} // namespace

void setup() {
    Serial.begin(115200); delay(500);
    for (const uint8_t pin : {tmp_hardware::YAW_STEP_PIN, tmp_hardware::PITCH_STEP_PIN, tmp_hardware::CARRIAGE_STEP_PIN}) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
    Serial.println("M07_north_level_control: simultaneous magnetic north + level");
    Serial.println("Yaw DIR32/STEP33; pitch DIR26/STEP12; BNO085 Bus B SDA4/SCL5 address 0x4A");
    Serial.println("BNO accuracy >=2 throughout baseline; after reference: 1000 ms low-accuracy grace, independent 150 ms stale stop.");
    Serial.print("Yaw trial sign="); Serial.print(TRIAL_POSITIVE_STEP_YAW_SIGN); Serial.println("; verified M05 pitch sign=+1");
    Serial.println("X/x aborts; yaw may take up to 180 deg shortest path to north. Clear the intended path and cables.");
    Serial.print("Measured yaw guard +/-"); Serial.print(RELATIVE_LIMIT_DEG, 0);
    Serial.println(" deg from continuous north target; pitch absolute guard +/-75 deg. Power removal is emergency stop.");
    Serial.print("Slew yaw/pitch Hz="); Serial.print(YAW_SLEW_SPEED_HZ); Serial.print('/'); Serial.print(PITCH_SLEW_SPEED_HZ);
    Serial.print(" accel="); Serial.print(YAW_SLEW_ACCELERATION); Serial.print('/'); Serial.println(PITCH_SLEW_ACCELERATION);
    Serial.print("Approach yaw/pitch deg="); Serial.print(YAW_APPROACH_DEG); Serial.print('/'); Serial.print(PITCH_APPROACH_DEG);
    Serial.println(" plus measured-rate braking margin. Continuous slew then stopped precision corrections.");
    engine.init();
    yawMotor = engine.stepperConnectToPin(tmp_hardware::YAW_STEP_PIN); pitchMotor = engine.stepperConnectToPin(tmp_hardware::PITCH_STEP_PIN);
    if (!yawMotor || !pitchMotor) { abortTest("FastAccelStepper axis initialization failed"); return; }
    yawMotor->setDirectionPin(tmp_hardware::YAW_DIR_PIN, true, 200); pitchMotor->setDirectionPin(tmp_hardware::PITCH_DIR_PIN, true, 200);
    if (yawMotor->setSpeedInHz(MIN_SPEED_HZ) != 0 || pitchMotor->setSpeedInHz(MIN_SPEED_HZ) != 0 ||
        yawMotor->setAcceleration(ACCELERATION) != 0 || pitchMotor->setAcceleration(ACCELERATION) != 0) { abortTest("FastAccelStepper configuration rejected"); return; }
    yawAxis = {}; pitchAxis = {}; yawAxis.motor = yawMotor; yawAxis.pitch = false; pitchAxis.motor = pitchMotor; pitchAxis.pitch = true; pitchAxis.confirmed = true;
    if (!motionWatchdog.begin(yawMotor, pitchMotor, BNO_STALE_MS)) { abortTest("Independent BNO watchdog initialization failed"); return; }
    if (!Wire1.begin(tmp_hardware::I2C_BUS_B_SDA_PIN, tmp_hardware::I2C_BUS_B_SCL_PIN, 100000)) { abortTest("Required I2C Bus B initialization failed"); return; }
    Wire1.setTimeOut(50); Wire1.beginTransmission(0x4A);
    if (Wire1.endTransmission() != 0) { abortTest("Required BNO085 at Bus B 0x4A did not ACK"); return; }
    bnoInitialized = bno.begin_I2C(0x4A, &Wire1);
    if (!bnoInitialized || !enableReport()) { abortTest("BNO085 initialization/report enabling failed"); return; }
    startedAt = lastBnoGood = millis(); bnoHealth.lastFreshOrStartMs = startedAt; phase = Phase::STARTUP;
}
void loop() {
    if (finalPrinted) { delay(20); return; }
    // Bound input work too, so a busy serial sender cannot starve acquisition.
    for (unsigned n = 0; n < 32 && Serial.available(); ++n) {
        const int input = Serial.read(); if (input == 'X' || input == 'x') { abortTest("Operator X abort"); return; }
    }
    safety(); if (finalPrinted) return;
    serviceBno(); if (finalPrinted) return;
    if (phase == Phase::STARTUP) { if (millis() - startedAt >= STARTUP_MS) { phase = Phase::BASELINE; startBaseline(); } }
    else if (phase == Phase::BASELINE) {
        if (millis() - startedAt >= BASELINE_TIMEOUT_MS) abortTest("Stable calibrated BNO baseline unavailable; calibrate and reset");
        else if (millis() - window.started >= BASELINE_MS) {
            if (!stableBaseline()) { accuracyGood = 0; startBaseline(); }
            else {
                baselineHeading = wrap360(heading.first + window.heading.mean()); baselinePitch = window.pitch.mean();
                northTargetContinuous = window.heading.mean() + shortestDifference(0, baselineHeading);
                yawAxis.target = northTargetContinuous; pitchAxis.target = 0; referenceSet = northUsable = true;
                window.active = false; accuracyGrace = {};
                controlStartedAt = millis();
                yawAxis.error = northTargetContinuous - heading.continuous;
                yawAxis.initialError = yawAxis.bestError = yawAxis.progressError = fabs(yawAxis.error);
                pitchAxis.error = -baselinePitch;
                pitchAxis.initialError = pitchAxis.bestError = pitchAxis.progressError = fabs(pitchAxis.error);
                yawAxis.lastProgress = pitchAxis.lastProgress = millis();
                if (!motionWatchdog.arm(lastBnoGood)) { abortTest("BNO watchdog could not arm"); return; }
                char line[120];
                snprintf(line, sizeof(line), "NORTH REFERENCE heading_deg=%.3f accuracy=%u\n", baselineHeading, bnoAccuracy);
                queueText(line);
                phase = Phase::MOVING;
            }
        }
    } else if (phase == Phase::MOVING || phase == Phase::SETTLING) serviceAxes();
    if (!finalPrinted && millis() - lastDisplay >= 750) { lastDisplay = millis(); telemetry(); }
    serviceSerialOutput();
    delay(1);
}
