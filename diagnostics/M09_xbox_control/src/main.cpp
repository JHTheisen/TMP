#include <Arduino.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <climits>
#include <FastAccelStepper.h>
#include <Wire.h>
#include "hardware_config.h"
#include "sensor_support.h"
#include "control_math.h"
#include "motion_watchdog.h"
#include "pose_math.h"

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
// Earlier three-axis diagnostics used these carriage speed/acceleration values.
// Position is generated pulses from power-up, not a homed or measured distance.
constexpr uint32_t CARRIAGE_MAX_SPEED_HZ = 1000;
constexpr int32_t CARRIAGE_ACCELERATION = 1000, CARRIAGE_LIMIT_STEPS = 500;
enum class Operation { NORTH_LEVEL, POSE, MANUAL };

enum class Phase { STARTUP, BASELINE, MOVING, SETTLING, COMPLETE, ABORTED, MANUAL };
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
    bool active = false, interrupted = false, calibrationInterrupted = false;
    uint32_t started = 0, lastBno = 0;
    Statistics heading, pitch, firstHeading, secondHeading, firstPitch, secondPitch;
};
FastAccelStepperEngine engine;
FastAccelStepper *yawMotor = nullptr, *pitchMotor = nullptr;
FastAccelStepper *carriageMotor = nullptr;
DiagnosticBno085 bno;
MotionWatchdog motionWatchdog;
// A separate instance adds a host-command lease without changing M08's BNO watchdog.
MotionWatchdog commandWatchdog;
constexpr uint32_t MANUAL_COMMAND_TIMEOUT_MS = 250;
bool manualActive = false, manualEnding = false;
struct ManualAxis {
    int request = 0, direction = 0;
    uint32_t rate = 0, brakeAt = 0, stoppedAt = 0, progressAt = 0;
    bool braking = false, observing = false;
    double start = 0, best = 0, progress = 0;
};
ManualAxis manualYaw, manualPitch, manualCarriage;
AccuracyGrace accuracyGrace;
BnoHealth bnoHealth;
EulerAngles orientation = {0, 0, 0};
// Physical gimbal PITCH was verified to follow BNO roll. Raw Euler pitch
// remains available for vector validation, but is not pitch-axis feedback.
HeadingTracker heading;
Axis yawAxis, pitchAxis;
Window window;
Phase phase = Phase::ABORTED;
bool finalPrinted = false, referenceSet = false, bnoInitialized = false, reportEnabled = false, bnoValid = false;
bool northUsable = false, concurrentMotion = false, sawYawMotion = false, sawPitchMotion = false;
// Orientation safety is independent of a calibrated magnetic north reference.
bool pitchReady = false, yawRequired = true, accuracyRequired = true;
double pitchReadyYaw = 0;
uint8_t bnoAccuracy = 0;
double northTargetContinuous = 0, baselineHeading = 0, baselinePitch = 0;
uint32_t startedAt = 0, controlStartedAt = 0, lastBnoGood = 0, lastReportAttempt = 0, lastDisplay = 0, bnoMaxGap = 0;
uint32_t lastControlSample = 0, settleStarted = 0, settleSamples = 0, settleLastSample = 0;
uint32_t invalidVectors = 0, bnoResets = 0, reportFailures = 0, accuracyGood = 0;
char txBuffer[4096];
size_t txLength = 0, txOffset = 0;
Operation operation = Operation::NORTH_LEVEL;
bool poseActive = false, carriagePending = false, timingLimited = false;
bool concurrentThreeMotion = false;
int32_t carriageTarget = 0;
uint32_t yawRateCap = YAW_SLEW_SPEED_HZ, pitchRateCap = PITCH_SLEW_SPEED_HZ, carriageRateHz = CARRIAGE_MAX_SPEED_HZ;
double plannedDurationSeconds = 0, yawPulsesPerDegree = 0, pitchPulsesPerDegree = 0;
double yawTimingPeak = 0, pitchTimingPeak = 0, yawStartAngle = 0, pitchStartAngle = 0;
int32_t yawStartSteps = 0, pitchStartSteps = 0;
char commandLine[128];
size_t commandLength = 0;
bool commandOverflow = false;

bool commandIdle() { return finalPrinted && phase == Phase::COMPLETE; }
uint32_t limitedSpeed(const Axis &axis, uint32_t native) {
    return poseActive ? std::min(native, axis.pitch ? pitchRateCap : yawRateCap) : native;
}
void recordTimingStarts() {
    yawStartAngle = heading.continuous; pitchStartAngle = orientation.roll;
    yawStartSteps = yawMotor->getCurrentPosition(); pitchStartSteps = pitchMotor->getCurrentPosition();
}
void learnTimingResponse() {
    const double yawDelta = heading.continuous - yawStartAngle, pitchDelta = orientation.roll - pitchStartAngle;
    const double yawSteps = static_cast<double>(yawMotor->getCurrentPosition()) - yawStartSteps;
    const double pitchSteps = static_cast<double>(pitchMotor->getCurrentPosition()) - pitchStartSteps;
    // Completed, stopped BNO motion supplies a timing estimate only. These
    // counts never replace BNO angle feedback or claim a calibrated gearbox.
    if (fabs(yawDelta) >= DIRECTION_RESPONSE_DEG && fabs(yawSteps) >= 8 &&
        yawSteps * yawDelta * TRIAL_POSITIVE_STEP_YAW_SIGN > 0) {
        yawPulsesPerDegree = fabs(yawSteps / yawDelta); yawTimingPeak = yawAxis.peakAngularSpeed;
    }
    if (fabs(pitchDelta) >= DIRECTION_RESPONSE_DEG && fabs(pitchSteps) >= 8 &&
        pitchSteps * pitchDelta * POSITIVE_STEP_PITCH_SIGN > 0) {
        pitchPulsesPerDegree = fabs(pitchSteps / pitchDelta); pitchTimingPeak = pitchAxis.peakAngularSpeed;
    }
}

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
    case Phase::COMPLETE: return "COMPLETE"; case Phase::ABORTED: return "ABORTED";
    case Phase::MANUAL: return "MANUAL"; }
    return "UNKNOWN";
}
bool fresh(uint32_t now) { return bnoValid && reportEnabled && now - lastBnoGood < BNO_STALE_MS; }
void stopMotors() {
    if (yawMotor) yawMotor->forceStop();
    if (pitchMotor) pitchMotor->forceStop();
    if (carriageMotor) carriageMotor->forceStop();
    delay(25);
    digitalWrite(tmp_hardware::YAW_STEP_PIN, LOW);
    digitalWrite(tmp_hardware::PITCH_STEP_PIN, LOW);
    digitalWrite(tmp_hardware::CARRIAGE_STEP_PIN, LOW);
}
void finish(bool passed, const char *reason) {
    if (finalPrinted) return;
    if (passed && !manualActive) learnTimingResponse();
    stopMotors(); motionWatchdog.disarm(); finalPrinted = true; window.active = false;
    commandWatchdog.disarm(); manualActive = manualEnding = false;
    manualYaw.request = manualPitch.request = manualCarriage.request = 0;
    phase = passed ? Phase::COMPLETE : Phase::ABORTED;
    // READY continues BNO acquisition: queue the summary instead of waiting on
    // UART space. Discard optional pending snapshots, including any partial line.
    txOffset = txLength = 0;
    char summary[2600];
    snprintf(summary, sizeof(summary),
        "\n=== MILESTONE 9 %s SUMMARY ===\n"
        "BNO heading_deg=%.3f accuracy=%u north_usable=%s\n"
        "Target heading_deg=%.3f pitch_deg=%.3f; baseline heading/pitch=%.3f/%.3f\n"
        "YAW settled=%s error_deg=%.3f bursts=%lu\n"
        "PITCH settled=%s error_deg=%.3f bursts=%lu\n"
        "Slew starts yaw/pitch=%lu/%lu\n"
        "CARRIAGE position/target_steps=%ld/%ld\n"
        "Timing estimate pulses_per_deg yaw/pitch=%.3f/%.3f\n"
        "Simultaneous two-axis motion occurred=%s\n"
        "Simultaneous three-axis motion observed=%s\n"
        "Both axes reached and settled=%s\n"
        "BNO fresh/invalid/resets/report_failures/write_failures=%lu/%lu/%lu/%lu/%lu\n"
        "Maximum BNO acquisition_gap_ms=%lu\n"
        "BNO accuracy low episodes/recoveries/longest_ms=%lu/%lu/%lu\n"
        "Independent BNO stale watchdog tripped=%s\n"
        "Reason: %s\nFINAL RESULT: %s\n%s\n",
        operation == Operation::MANUAL ? "XBOX MANUAL" : (operation == Operation::POSE ? "GO TO POSE" : "PRESERVED M08 NORTH / LEVEL"),
        orientation.heading, bnoAccuracy, northUsable ? "YES" : "NO",
        wrap360(heading.first + yawAxis.target), pitchAxis.target, baselineHeading, baselinePitch,
        yawAxis.settled ? "YES" : "NO", yawAxis.error, static_cast<unsigned long>(yawAxis.bursts),
        pitchAxis.settled ? "YES" : "NO", pitchAxis.error, static_cast<unsigned long>(pitchAxis.bursts),
        static_cast<unsigned long>(yawAxis.slewStarts), static_cast<unsigned long>(pitchAxis.slewStarts),
        static_cast<long>(carriageMotor ? carriageMotor->getCurrentPosition() : 0), static_cast<long>(carriageTarget),
        yawPulsesPerDegree, pitchPulsesPerDegree, concurrentMotion ? "YES" : "NO",
        concurrentThreeMotion ? "YES" : "NO", yawAxis.settled && pitchAxis.settled ? "YES" : "NO",
        static_cast<unsigned long>(bnoHealth.freshSamples), static_cast<unsigned long>(invalidVectors),
        static_cast<unsigned long>(bnoResets), static_cast<unsigned long>(reportFailures),
        static_cast<unsigned long>(DiagnosticBno085::writeFailures), static_cast<unsigned long>(bnoMaxGap),
        static_cast<unsigned long>(accuracyGrace.episodes), static_cast<unsigned long>(accuracyGrace.recoveries),
        static_cast<unsigned long>(std::max(accuracyGrace.longestMs, accuracyGrace.age(millis()))),
        motionWatchdog.tripped() ? "YES" : "NO", reason, passed ? "PASS" : "FAIL",
        passed ? (referenceSet ? "READY: POSE yaw_deg pitch_deg carriage_steps; MOVE delta_yaw_deg delta_pitch_deg delta_steps. No automatic motion." :
                 "PITCH-ONLY READY: north unavailable; MOVE 0 <pitch_delta_deg> 0. No automatic motion; reset for calibrated north startup.") :
                 "Latched abort; reset required. X/x aborts without automatic retry.");
    poseActive = false; carriagePending = false;
    queueText(summary);
}
void abortTest(const char *reason) {
    if (commandIdle()) finalPrinted = false;
    finish(false, reason);
}
void safety() {
    if (manualActive) commandWatchdog.check(millis());
    if (manualActive && commandWatchdog.tripped()) {
        abortTest("Manual command stream lost for 250 ms; reset required"); return;
    }
    if (!pitchReady && !referenceSet) return;
    if (finalPrinted) {
        // Continuing to accept poses needs a continuous heading reference.
        // An idle outage can hide manual motion just as an active outage can.
        if (commandIdle() && millis() - lastBnoGood >= BNO_STALE_MS)
            abortTest("BNO085 stale while idle: heading continuity unverified; reset required");
        return;
    }
    const uint32_t now = millis(); bnoMaxGap = std::max(bnoMaxGap, now - lastBnoGood);
    if (motionWatchdog.tripped() || now - lastBnoGood >= BNO_STALE_MS) abortTest("BNO085 feedback stale: acquisition gap reached 150 ms");
    else if (yawRequired && bnoAccuracy < BNO_MIN_ACCURACY)
        abortTest("BNO085 accuracy below 2: yaw operation stopped; reset required");
    else if (accuracyRequired && accuracyGrace.expired(now)) abortTest("BNO085 accuracy below 2 continuously for 1000 ms");
    else if (bnoValid && fabs(heading.continuous - (referenceSet ? northTargetContinuous : pitchReadyYaw)) >= RELATIVE_LIMIT_DEG)
        abortTest(referenceSet ? "Measured yaw travel guard exceeded +/-185 degrees from continuous north target" :
            "Measured yaw travel guard exceeded +/-185 degrees from pitch-only startup orientation");
    else if (bnoValid && fabs(orientation.roll) >= MAX_USABLE_PITCH_DEG)
        abortTest("Measured pitch reached +/-75 degree absolute guard");
    else if (carriageMotor && (carriageMotor->getCurrentPosition() < -CARRIAGE_LIMIT_STEPS || carriageMotor->getCurrentPosition() > CARRIAGE_LIMIT_STEPS))
        abortTest("Carriage startup-relative step guard exceeded +/-500 steps");
    else if (now - controlStartedAt >= LEG_TIMEOUT_MS)
        abortTest(poseActive ? "Three-axis pose control timeout" : "Two-axis control timeout");
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
    else if (pitchReady) abortTest("BNO085 reset after pitch baseline: orientation continuity lost");
    else { heading = {}; northUsable = false; enableReport(); }
    return true;
}
void serviceBno() {
    if (!bnoInitialized) return;
    safety(); if (finalPrinted && !commandIdle()) return;
    handleReset(); if (finalPrinted && !commandIdle()) return;
    if (!reportEnabled && millis() - lastReportAttempt >= 500) enableReport();
    sh2_SensorValue_t event = {};
    const bool gotEvent = bno.getSensorEvent(&event), reset = handleReset();
    safety();
    if ((finalPrinted && !commandIdle()) || reset || !gotEvent || !reportEnabled || event.sensorId != SH2_ROTATION_VECTOR) return;
    EulerAngles result;
    if (!quaternionToEuler(event.un.rotationVector, result)) { ++invalidVectors; window.interrupted = true; return; }
    if (!heading.update(result.heading)) { abortTest("Ambiguous/nonfinite BNO heading transition"); return; }
    const uint32_t now = millis();
    bnoHealth.recordFresh(now, pitchReady || referenceSet); orientation = result; bnoAccuracy = event.status;
    bnoValid = true; lastBnoGood = now;
    if ((pitchReady || referenceSet) && !finalPrinted) {
        motionWatchdog.recordFresh(now);
    }
    if (accuracyRequired && referenceSet && !finalPrinted) {
        const bool wasLow = accuracyGrace.low;
        accuracyGrace.observe(event.status, now);
        northUsable = event.status >= BNO_MIN_ACCURACY;
        if (!wasLow && accuracyGrace.low) queueText("BNO ACCURACY LOW: yaw disabled; carriage accuracy grace=1000 ms; stale deadline remains 150 ms\n");
        if (wasLow && !accuracyGrace.low) queueText("BNO ACCURACY RECOVERED: continuous low timer cleared\n");
    }
    northUsable = bnoAccuracy >= BNO_MIN_ACCURACY && referenceSet;
    safety(); if (finalPrinted) return;
    if (yawMotor && yawMotor->isRunning()) { ++yawAxis.motionSamples; sawYawMotion = true; }
    if (pitchMotor && pitchMotor->isRunning()) { ++pitchAxis.motionSamples; sawPitchMotion = true; }
    if (yawMotor && yawMotor->isRunning() && pitchMotor && pitchMotor->isRunning()) concurrentMotion = true;
    if (poseActive && yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning()) concurrentThreeMotion = true;
    if (window.active) {
        if (event.status < BNO_MIN_ACCURACY) window.calibrationInterrupted = true;
        else ++accuracyGood;
        if (now - window.lastBno > WINDOW_GAP_MS) window.interrupted = true;
        window.lastBno = now; window.heading.add(heading.continuous); window.pitch.add(result.roll);
        if (now - window.started < BASELINE_MS / 2) { window.firstHeading.add(heading.continuous); window.firstPitch.add(result.roll); }
        else { window.secondHeading.add(heading.continuous); window.secondPitch.add(result.roll); }
    }
}
void startBaseline() { window = {}; accuracyGood = 0; window.active = true; window.started = window.lastBno = millis(); }
bool stablePitchBaseline() {
    const uint32_t now = millis();
    return !window.interrupted && now - window.started >= BASELINE_MS && window.pitch.n >= 30 &&
        window.firstPitch.n && window.secondPitch.n && now - window.lastBno <= WINDOW_GAP_MS && fresh(now) &&
        window.pitch.sd() <= 0.15 && window.pitch.range() <= 0.5 &&
        fabs(window.secondPitch.mean() - window.firstPitch.mean()) <= 0.2;
}
bool stableBaseline() {
    const uint32_t now = millis();
    return !window.interrupted && !window.calibrationInterrupted && window.heading.n >= 30 && window.pitch.n >= 30 && accuracyGood >= 30 &&
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
    if (!axis.pitch && (!yawRequired || !referenceSet || bnoAccuracy < BNO_MIN_ACCURACY)) return;
    if (axis.confirmed && !axis.slewFinished &&
        fabs(axis.error) > axis.brakeAtDeg + SLEW_ENTRY_HYSTERESIS_DEG) {
        axis.slewDirection = stepDirection(axis.error, axis.pitch);
        if (axis.motor->setAcceleration(slewAcceleration(axis.pitch)) != 0 ||
            axis.motor->setSpeedInHz(limitedSpeed(axis, slewSpeed(axis.pitch))) != 0 ||
            (axis.slewDirection > 0 ? axis.motor->runForward() : axis.motor->runBackward()) != MOVE_OK) {
            abortTest(axis.pitch ? "FastAccelStepper pitch slew rejected" : "FastAccelStepper yaw slew rejected"); return;
        }
        ++axis.slewStarts; axis.commandAt = millis(); axis.lastProgress = millis();
        setMotion(axis, Motion::SLEW);
    } else {
        const int32_t steps = correctionSteps(axis.error, axis.confirmed, axis.pitch);
        if (!steps) return;
        if (axis.motor->setAcceleration(ACCELERATION) != 0 ||
            axis.motor->setSpeedInHz(limitedSpeed(axis, correctionSpeed(axis.error, axis.confirmed))) != 0 ||
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
    if ((!pitchReady && !referenceSet) || !fresh(millis())) return;
    // Cached values are useful for telemetry, but never constitute a new
    // control decision, direction confirmation, velocity or settling sample.
    if (lastControlSample == bnoHealth.freshSamples) return;
    lastControlSample = bnoHealth.freshSamples;
    const uint32_t now = millis();
    yawAxis.current = heading.continuous; yawAxis.error = yawAxis.target - yawAxis.current;
    pitchAxis.current = orientation.roll; pitchAxis.error = pitchAxis.target - pitchAxis.current;
    if ((yawRequired && !axisProgress(yawAxis, yawAxis.error)) || !axisProgress(pitchAxis, pitchAxis.error)) return;
    if (yawRequired) serviceAxis(yawAxis, now);
    if (finalPrinted) return;
    serviceAxis(pitchAxis, now); if (finalPrinted) return;
    if (carriagePending) {
        safety(); if (finalPrinted) return;
        if (carriageMotor->setSpeedInHz(carriageRateHz) != 0 ||
            carriageMotor->moveTo(carriageTarget) != MOVE_OK) {
            abortTest("FastAccelStepper carriage pose command rejected"); return;
        }
        carriagePending = false;
        safety(); if (finalPrinted) return;
    }
    const bool carriageStoppedAtTarget = !carriageMotor->isRunning() && carriageMotor->getCurrentPosition() == carriageTarget;
    const bool stoppedInTolerance = yawAxis.motion == Motion::HOLD && pitchAxis.motion == Motion::HOLD &&
        !yawMotor->isRunning() && !pitchMotor->isRunning() &&
        carriageStoppedAtTarget &&
        (!yawRequired || fabs(yawAxis.error) <= TOLERANCE_DEG) && fabs(pitchAxis.error) <= TOLERANCE_DEG &&
        (!accuracyRequired || bnoAccuracy >= BNO_MIN_ACCURACY);
    if (!stoppedInTolerance) { phase = Phase::MOVING; settleSamples = 0; return; }
    if (!settleSamples || now - settleLastSample > WINDOW_GAP_MS) {
        settleStarted = now; settleSamples = 0;
    }
    phase = Phase::SETTLING; ++settleSamples; settleLastSample = now;
    if (now - settleStarted >= SETTLE_MS && settleSamples >= 30) {
        yawAxis.settled = referenceSet && fabs(yawAxis.error) <= TOLERANCE_DEG;
        pitchAxis.settled = true;
        const bool evidence = sawYawMotion && sawPitchMotion && concurrentMotion;
        if (poseActive) finish(true, !referenceSet ? "Pitch target stopped and settled; yaw disabled, north unverified" :
            "POSE complete: BNO angle targets settled; carriage generated-step target reached");
        else finish(evidence, evidence ? "Both measured axes stopped and settled at magnetic north and level" :
            "Targets settled, but simultaneous two-axis motion was not demonstrated");
    }
}
void rejectPose(const char *reason) {
    queueText("POSE REJECTED: "); queueText(reason); queueText("\n");
}
void resetPoseAxis(Axis &axis, double target, double timingPeak) {
    FastAccelStepper *motor = axis.motor;
    const bool pitch = axis.pitch;
    axis = {}; axis.motor = motor; axis.pitch = pitch; axis.confirmed = true;
    axis.target = target; axis.current = pitch ? orientation.roll : heading.continuous;
    axis.error = target - axis.current;
    axis.initialError = axis.bestError = axis.progressError = fabs(axis.error);
    axis.lastProgress = millis(); axis.peakAngularSpeed = timingPeak;
    if (fabs(axis.error) <= TOLERANCE_DEG) axis.motion = Motion::HOLD;
}
void beginPose(double yawValue, double pitchValue, int64_t carriageValue, bool relative) {
    if (!commandIdle()) { rejectPose(finalPrinted ? "abort latched; reset required" : "busy; wait for READY"); return; }
    if (!pitchReady || !fresh(millis()) || motionWatchdog.tripped()) {
        rejectPose("fresh valid BNO orientation and intact pitch baseline required"); return;
    }
    if (relative && (yawValue < -180.0 || yawValue >= 180.0)) {
        rejectPose("MOVE yaw delta must be in [-180,180) degrees; no implicit full-turn motion"); return;
    }
    const double requestedYaw = relative ? static_cast<double>(orientation.heading) + yawValue : yawValue;
    const double requestedPitch = relative ? static_cast<double>(orientation.roll) + pitchValue : pitchValue;
    const int64_t requestedCarriage = relative ? static_cast<int64_t>(carriageMotor->getCurrentPosition()) + carriageValue : carriageValue;
    if (!isfinite(requestedYaw) || !isfinite(requestedPitch) || fabs(requestedPitch) >= MAX_USABLE_PITCH_DEG) {
        rejectPose("finite yaw/pitch required; pitch target must be inside +/-75 degrees"); return;
    }
    const double targetYaw = heading.continuous + shortestDifference(requestedYaw, orientation.heading);
    if (referenceSet && fabs(targetYaw - northTargetContinuous) >= RELATIVE_LIMIT_DEG) {
        rejectPose("shortest yaw path endpoint exceeds existing +/-185-degree continuous north guard"); return;
    }
    if (requestedCarriage < -CARRIAGE_LIMIT_STEPS || requestedCarriage > CARRIAGE_LIMIT_STEPS) {
        rejectPose("carriage target outside +/-500 startup-relative STEPS; millimeters unavailable"); return;
    }
    const double yawError = targetYaw - heading.continuous, pitchError = requestedPitch - orientation.roll;
    const bool moveYaw = fabs(yawError) > TOLERANCE_DEG, movePitch = fabs(pitchError) > TOLERANCE_DEG;
    const double carriageDistance = fabs(static_cast<double>(requestedCarriage) - carriageMotor->getCurrentPosition());
    const bool pitchOnly = !referenceSet || bnoAccuracy < BNO_MIN_ACCURACY;
    if (pitchOnly && moveYaw) {
        rejectPose("yaw disabled: calibrated north baseline and BNO accuracy >=2 required"); return;
    }
    if (pitchOnly && carriageDistance != 0) {
        rejectPose("pitch-only readiness: carriage motion still requires calibrated north and accuracy >=2"); return;
    }
    if (pitchOnly) {
        if (!movePitch) { queueText("POSE ALREADY AT TARGET: no motion; yaw disabled\n"); return; }
        // A startup timing measurement may not exist here. Run the existing
        // native pitch controller without inventing a pulses/degree estimate.
        resetPoseAxis(yawAxis, heading.continuous, yawTimingPeak);
        resetPoseAxis(pitchAxis, requestedPitch, pitchTimingPeak);
        yawRequired = accuracyRequired = false;
        yawRateCap = YAW_SLEW_SPEED_HZ; pitchRateCap = PITCH_SLEW_SPEED_HZ;
        plannedDurationSeconds = 0; timingLimited = true;
        operation = Operation::POSE; poseActive = true; finalPrinted = false; phase = Phase::MOVING;
        settleSamples = 0; window.active = false; accuracyGrace = {};
        sawYawMotion = sawPitchMotion = concurrentMotion = concurrentThreeMotion = false;
        controlStartedAt = millis(); lastControlSample = bnoHealth.freshSamples;
        recordTimingStarts();
        if (!motionWatchdog.arm(lastBnoGood)) { abortTest("BNO watchdog could not arm for pitch-only motion"); return; }
        queueText("PITCH-ONLY ACCEPTED: native pitch control; yaw disabled; carriage stationary\n");
        safety(); return;
    }
    if ((moveYaw && yawPulsesPerDegree <= 0) || (movePitch && pitchPulsesPerDegree <= 0)) {
        rejectPose("angular timing response unavailable: reset with modest yaw/pitch offsets for the existing north/level run"); return;
    }
    if (!moveYaw && !movePitch && carriageDistance == 0) {
        queueText("POSE ALREADY AT TARGET: no motion; carriage units=STEPS from startup zero\n"); return;
    }
    const double yawSeconds = moveYaw ? milestone8::estimateAxisSeconds(yawError, false, yawPulsesPerDegree, yawTimingPeak, YAW_SLEW_SPEED_HZ) : 0;
    const double pitchSeconds = movePitch ? milestone8::estimateAxisSeconds(pitchError, true, pitchPulsesPerDegree, pitchTimingPeak, PITCH_SLEW_SPEED_HZ) : 0;
    const double carriageSeconds = milestone8::finiteSeconds(carriageDistance, CARRIAGE_MAX_SPEED_HZ, CARRIAGE_ACCELERATION);
    const double duration = fmax(fmax(yawSeconds, pitchSeconds), carriageSeconds);
    if (!isfinite(yawSeconds) || !isfinite(pitchSeconds) || !isfinite(carriageSeconds) || duration <= 0 ||
        duration * 1000 >= LEG_TIMEOUT_MS - SETTLE_MS) {
        rejectPose("estimated move cannot meet existing motion deadlines; use a smaller pose change"); return;
    }
    const auto yawTiming = moveYaw ? milestone8::chooseAxisCap(yawError, false, yawPulsesPerDegree, yawTimingPeak, duration) : milestone8::AxisTiming{true, 0, 0};
    const auto pitchTiming = movePitch ? milestone8::chooseAxisCap(pitchError, true, pitchPulsesPerDegree, pitchTimingPeak, duration) : milestone8::AxisTiming{true, 0, 0};
    if ((moveYaw && (!isfinite(yawTiming.seconds) || !yawTiming.capHz)) ||
        (movePitch && (!isfinite(pitchTiming.seconds) || !pitchTiming.capHz))) {
        rejectPose("no safe synchronized speed under existing finite-correction timeout"); return;
    }
    // Preflight is complete before any axis state or motor command is changed.
    yawRateCap = moveYaw ? yawTiming.capHz : YAW_SLEW_SPEED_HZ;
    pitchRateCap = movePitch ? pitchTiming.capHz : PITCH_SLEW_SPEED_HZ;
    plannedDurationSeconds = duration;
    timingLimited = (moveYaw && !yawTiming.achievable) || (movePitch && !pitchTiming.achievable);
    const double timingTolerance = fmax(0.5, duration * 0.1);
    if ((moveYaw && fabs(yawTiming.seconds - duration) > timingTolerance) ||
        (movePitch && fabs(pitchTiming.seconds - duration) > timingTolerance)) timingLimited = true;
    if (carriageDistance > 0) {
        const double speed = milestone8::syncSpeedForDuration(carriageDistance, CARRIAGE_ACCELERATION, duration);
        carriageRateHz = static_cast<uint32_t>(fmax(1.0, fmin(static_cast<double>(CARRIAGE_MAX_SPEED_HZ), round(speed))));
        const double actualSeconds = milestone8::finiteSeconds(carriageDistance, carriageRateHz, CARRIAGE_ACCELERATION);
        if (fabs(actualSeconds - duration) > timingTolerance) timingLimited = true;
    }
    resetPoseAxis(yawAxis, targetYaw, yawTimingPeak); resetPoseAxis(pitchAxis, requestedPitch, pitchTimingPeak);
    yawRequired = moveYaw;
    accuracyRequired = moveYaw || carriageDistance > 0;
    carriageTarget = static_cast<int32_t>(requestedCarriage); carriagePending = carriageDistance > 0;
    operation = Operation::POSE; poseActive = true; finalPrinted = false; phase = Phase::MOVING;
    settleSamples = 0; window.active = false; accuracyGrace = {};
    sawYawMotion = sawPitchMotion = concurrentMotion = false;
    concurrentThreeMotion = false;
    controlStartedAt = millis(); lastControlSample = bnoHealth.freshSamples;
    recordTimingStarts();
    if (!motionWatchdog.arm(lastBnoGood)) { abortTest("BNO watchdog could not arm for POSE"); return; }
    char line[340];
    snprintf(line, sizeof(line), "POSE ACCEPTED yaw_deg=%.3f pitch_deg=%.3f carriage_steps=%ld planned_s=%.3f caps_hz=%lu/%lu/%lu predicted_s=%.3f/%.3f/%.3f timing_limited=%s\n",
        wrap360(requestedYaw), requestedPitch, static_cast<long>(carriageTarget), duration,
        static_cast<unsigned long>(yawRateCap), static_cast<unsigned long>(pitchRateCap), static_cast<unsigned long>(carriageRateHz),
        moveYaw ? yawTiming.seconds : 0, movePitch ? pitchTiming.seconds : 0,
        milestone8::finiteSeconds(carriageDistance, carriageRateHz, CARRIAGE_ACCELERATION), timingLimited ? "YES" : "NO");
    queueText(line);
    safety();
}
bool parseAngle(const char *token, double &value) {
    char *end = nullptr; errno = 0; value = strtod(token, &end);
    return end != token && *end == '\0' && errno != ERANGE && isfinite(value);
}
#include "manual_control.h"
void executeCommand() {
    char *tokens[5] = {}; unsigned count = 0; char *context = nullptr;
    for (char *token = strtok_r(commandLine, " \t", &context); token && count < 5; token = strtok_r(nullptr, " \t", &context)) tokens[count++] = token;
    if (!count) return;
    if (executeManualCommand(tokens, count)) return;
    if (count != 4 || (strcmp(tokens[0], "POSE") != 0 && strcmp(tokens[0], "MOVE") != 0)) {
        rejectPose("use POSE yaw_deg pitch_deg carriage_steps or MOVE delta_yaw_deg delta_pitch_deg delta_steps"); return;
    }
    double yawValue = 0, pitchValue = 0;
    char *end = nullptr; errno = 0; const long long carriageValue = strtoll(tokens[3], &end, 10);
    if (end == tokens[3] || *end || errno == ERANGE || carriageValue < INT32_MIN || carriageValue > INT32_MAX ||
        !parseAngle(tokens[1], yawValue) || !parseAngle(tokens[2], pitchValue)) {
        rejectPose("angles must be finite numbers; carriage must be an integer step count"); return;
    }
    beginPose(yawValue, pitchValue, carriageValue, strcmp(tokens[0], "MOVE") == 0);
}
void serviceCommands() {
    for (unsigned n = 0; n < 32 && Serial.available(); ++n) {
        const int input = Serial.read();
        if (input == 'X' || input == 'x') {
            commandLength = 0; commandOverflow = false; abortTest("Operator X abort"); return;
        }
        if (input == '\r' || input == '\n') {
            if (commandOverflow) rejectPose("command line too long");
            else if (commandLength) { commandLine[commandLength] = '\0'; executeCommand(); }
            commandLength = 0; commandOverflow = false;
        } else if (!commandOverflow) {
            if ((input < 32 && input != '\t') || input > 126 || commandLength >= sizeof(commandLine) - 1) commandOverflow = true;
            else commandLine[commandLength++] = static_cast<char>(input);
        }
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
        phaseText(), orientation.heading, yawAxis.error, orientation.roll, pitchAxis.error,
        motionText(yawAxis), motionText(pitchAxis), yawRunning ? "MOVING" : "IDLE", pitchRunning ? "MOVING" : "IDLE",
        yawRunning && pitchRunning ? "YES" : "NO",
        yawMotor ? yawMotor->getCurrentSpeedInMilliHz() / 1000.0 : 0,
        pitchMotor ? pitchMotor->getCurrentSpeedInMilliHz() / 1000.0 : 0,
        yawAxis.angularSpeed, pitchAxis.angularSpeed, yawAxis.brakeAtDeg, pitchAxis.brakeAtDeg,
        fresh(now) ? "YES" : "NO", static_cast<unsigned long>(now - lastBnoGood),
        static_cast<unsigned long>(bnoMaxGap), bnoAccuracy, static_cast<unsigned long>(accuracyGrace.age(now)),
        northUsable ? "YES" : "NO");
    queueText(line);
    if (poseActive || commandIdle()) {
        char poseLine[220];
        snprintf(poseLine, sizeof(poseLine), "POSE_STATE active=%s carriage_steps=%ld target_steps=%ld carriage_motor=%s caps_hz=%lu/%lu/%lu planned_s=%.2f timing_limited=%s all_three_seen=%s\n",
            poseActive ? "YES" : "NO", static_cast<long>(carriageMotor->getCurrentPosition()), static_cast<long>(carriageTarget),
            carriageMotor->isRunning() ? "MOVING" : "IDLE", static_cast<unsigned long>(yawRateCap),
            static_cast<unsigned long>(pitchRateCap), static_cast<unsigned long>(carriageRateHz), plannedDurationSeconds,
            timingLimited ? "YES" : "NO", concurrentThreeMotion ? "YES" : "NO");
        queueText(poseLine);
    }
}
} // namespace

void setup() {
    Serial.begin(115200); delay(500);
    for (const uint8_t pin : {tmp_hardware::YAW_STEP_PIN, tmp_hardware::PITCH_STEP_PIN, tmp_hardware::CARRIAGE_STEP_PIN}) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
    Serial.println("M09_xbox_control: host Xbox velocity commands; preserved M08 startup and POSE/MOVE");
    Serial.println("Yaw DIR32/STEP33; pitch DIR26/STEP12; carriage DIR21/STEP22; BNO085 Bus B SDA4/SCL5 address 0x4A");
    Serial.println("Physical PITCH feedback = BNO ROLL; pitch targets and telemetry use roll degrees.");
    Serial.println("Carriage zero is startup step count only; units are STEPS, not mm. Software envelope +/-500 steps; no homing.");
    Serial.println("North requires calibrated baseline and BNO accuracy >=2; low accuracy stops yaw operations. Independent 150 ms stale stop for all motion.");
    Serial.println("Stable fresh pitch can enter PITCH-ONLY READY after north-baseline timeout, regardless of accuracy. No automatic fallback motion.");
    Serial.print("Yaw trial sign="); Serial.print(TRIAL_POSITIVE_STEP_YAW_SIGN); Serial.print("; current pitch sign="); Serial.println(POSITIVE_STEP_PITCH_SIGN);
    Serial.println("X/x aborts; yaw may take up to 180 deg shortest path to north. Clear the intended path and cables.");
    Serial.print("Measured yaw guard +/-"); Serial.print(RELATIVE_LIMIT_DEG, 0);
    Serial.println(" deg from continuous north target; pitch absolute guard +/-75 deg. Power removal is emergency stop.");
    Serial.print("Slew yaw/pitch Hz="); Serial.print(YAW_SLEW_SPEED_HZ); Serial.print('/'); Serial.print(PITCH_SLEW_SPEED_HZ);
    Serial.print(" accel="); Serial.print(YAW_SLEW_ACCELERATION); Serial.print('/'); Serial.println(PITCH_SLEW_ACCELERATION);
    Serial.print("Approach yaw/pitch deg="); Serial.print(YAW_APPROACH_DEG); Serial.print('/'); Serial.print(PITCH_APPROACH_DEG);
    Serial.println(" plus measured-rate braking margin. Continuous slew then stopped precision corrections.");
    engine.init();
    yawMotor = engine.stepperConnectToPin(tmp_hardware::YAW_STEP_PIN); pitchMotor = engine.stepperConnectToPin(tmp_hardware::PITCH_STEP_PIN);
    carriageMotor = engine.stepperConnectToPin(tmp_hardware::CARRIAGE_STEP_PIN);
    if (!yawMotor || !pitchMotor || !carriageMotor) { abortTest("FastAccelStepper axis initialization failed"); return; }
    yawMotor->setDirectionPin(tmp_hardware::YAW_DIR_PIN, true, 200); pitchMotor->setDirectionPin(tmp_hardware::PITCH_DIR_PIN, true, 200);
    carriageMotor->setDirectionPin(tmp_hardware::CARRIAGE_DIR_PIN, true, 200);
    if (yawMotor->setSpeedInHz(MIN_SPEED_HZ) != 0 || pitchMotor->setSpeedInHz(MIN_SPEED_HZ) != 0 ||
        yawMotor->setAcceleration(ACCELERATION) != 0 || pitchMotor->setAcceleration(ACCELERATION) != 0 ||
        carriageMotor->setSpeedInHz(CARRIAGE_MAX_SPEED_HZ) != 0 || carriageMotor->setAcceleration(CARRIAGE_ACCELERATION) != 0) { abortTest("FastAccelStepper configuration rejected"); return; }
    yawAxis = {}; pitchAxis = {}; yawAxis.motor = yawMotor; yawAxis.pitch = false; pitchAxis.motor = pitchMotor; pitchAxis.pitch = true; pitchAxis.confirmed = true;
    if (!motionWatchdog.begin(yawMotor, pitchMotor, BNO_STALE_MS, carriageMotor)) { abortTest("Independent BNO watchdog initialization failed"); return; }
    if (!commandWatchdog.begin(yawMotor, pitchMotor, MANUAL_COMMAND_TIMEOUT_MS, carriageMotor)) { abortTest("Independent manual command watchdog initialization failed"); return; }
    if (!Wire1.begin(tmp_hardware::I2C_BUS_B_SDA_PIN, tmp_hardware::I2C_BUS_B_SCL_PIN, 100000)) { abortTest("Required I2C Bus B initialization failed"); return; }
    Wire1.setTimeOut(50); Wire1.beginTransmission(0x4A);
    if (Wire1.endTransmission() != 0) { abortTest("Required BNO085 at Bus B 0x4A did not ACK"); return; }
    bnoInitialized = bno.begin_I2C(0x4A, &Wire1);
    if (!bnoInitialized || !enableReport()) { abortTest("BNO085 initialization/report enabling failed"); return; }
    startedAt = lastBnoGood = millis(); bnoHealth.lastFreshOrStartMs = startedAt; phase = Phase::STARTUP;
}
void loop() {
    serviceCommands();
    if (finalPrinted) {
        if (commandIdle()) serviceBno();
        serviceSerialOutput(); delay(1); return;
    }
    safety(); if (finalPrinted) return;
    serviceBno(); if (finalPrinted) return;
    if (phase == Phase::STARTUP) { if (millis() - startedAt >= STARTUP_MS) { phase = Phase::BASELINE; startBaseline(); } }
    else if (phase == Phase::BASELINE) {
        if (millis() - window.started >= BASELINE_MS) {
            if (!stableBaseline()) {
                if (millis() - startedAt < BASELINE_TIMEOUT_MS) startBaseline();
                else if (!stablePitchBaseline()) abortTest("Stable fresh pitch baseline unavailable; inspect BNO and reset");
                else {
                    baselinePitch = window.pitch.mean(); pitchReadyYaw = heading.continuous;
                    pitchReady = true; yawRequired = accuracyRequired = false; referenceSet = northUsable = false;
                    window.active = false; controlStartedAt = millis();
                    resetPoseAxis(yawAxis, heading.continuous, 0);
                    resetPoseAxis(pitchAxis, orientation.roll, 0);
                    safety(); if (finalPrinted) return;
                    finalPrinted = true; phase = Phase::COMPLETE;
                    txOffset = txLength = 0; // Preserve READY even after optional telemetry filled the UART buffer.
                    queueText("PITCH-ONLY READY: stable fresh pitch; calibrated north unavailable, yaw disabled. MOVE 0 <pitch_delta_deg> 0; POSE <current_heading_deg> 0 0 levels pitch. Reset for calibrated north startup.\n");
                }
            }
            else {
                baselineHeading = wrap360(heading.first + window.heading.mean()); baselinePitch = window.pitch.mean();
                northTargetContinuous = window.heading.mean() + shortestDifference(0, baselineHeading);
                yawAxis.target = northTargetContinuous; pitchAxis.target = 0; referenceSet = northUsable = pitchReady = true;
                window.active = false; accuracyGrace = {};
                controlStartedAt = millis();
                yawAxis.error = northTargetContinuous - heading.continuous;
                yawAxis.initialError = yawAxis.bestError = yawAxis.progressError = fabs(yawAxis.error);
                pitchAxis.error = -baselinePitch;
                pitchAxis.initialError = pitchAxis.bestError = pitchAxis.progressError = fabs(pitchAxis.error);
                yawAxis.lastProgress = pitchAxis.lastProgress = millis();
                recordTimingStarts();
                if (!motionWatchdog.arm(lastBnoGood)) { abortTest("BNO watchdog could not arm"); return; }
                char line[120];
                snprintf(line, sizeof(line), "NORTH REFERENCE heading_deg=%.3f accuracy=%u\n", baselineHeading, bnoAccuracy);
                queueText(line);
                phase = Phase::MOVING;
            }
        }
    } else if (phase == Phase::MANUAL) serviceManual();
    else if (phase == Phase::MOVING || phase == Phase::SETTLING) serviceAxes();
    if (!finalPrinted && millis() - lastDisplay >= 750) { lastDisplay = millis(); telemetry(); }
    serviceSerialOutput();
    delay(1);
}
