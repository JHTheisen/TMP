#include <Arduino.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <climits>
#include <FastAccelStepper.h>
#include <Wire.h>
#ifndef M07_HOST_TEST
#include <soc/gpio_reg.h>
#include <soc/soc.h>
#endif
#include "hardware_config.h"
#include "sensor_support.h"
#include "control_math.h"
#include "motion_watchdog.h"
#include "pose_math.h"
#include "encoder_acquisition.h"
#include "bno_diagnostics.h"
#include "sensor_worker.h"
extern "C" int m09_sh2_request_product_id(void);

namespace {
using milestone4::BnoHealth;
using milestone4::DiagnosticBno085;
using milestone4::EulerAngles;
using milestone4::quaternionToEuler;
using namespace milestone7;
constexpr uint32_t BASELINE_MS = 1000;
constexpr uint32_t BNO_STALE_MS = 150, WINDOW_GAP_MS = 100, SETTLE_MS = 1000;
constexpr uint32_t LEG_TIMEOUT_MS = 90000, PROGRESS_TIMEOUT_MS = 15000;
constexpr double RELATIVE_LIMIT_DEG = 185.0, MAX_USABLE_PITCH_DEG = 75.0;
// Earlier three-axis diagnostics used these carriage speed/acceleration values.
// Position is generated pulses from power-up, not a homed or measured distance.
constexpr uint32_t CARRIAGE_MAX_SPEED_HZ = 2000;
constexpr int32_t CARRIAGE_ACCELERATION = 2000;
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
m09::SensorWorker sensorWorker;
m09::EncoderSnapshot encoders;
m09::SensorSnapshot sensorSnapshot;
uint32_t sensorResetEpoch = 0, sensorSampleDrops = 0, bnoWriteFailures = 0;
uint32_t foregroundTraceDrops = 0;
bool sensorSetupReported = false;
BnoDiagnostics bnoDiagnostics;
uint8_t acceptedBnoSequence = 0;
uint64_t acceptedBnoTimestamp = 0;
uint32_t telemetryDrops = 0;
uint32_t lastSensorDisplay = 0;
MotionWatchdog motionWatchdog;
// A separate instance adds a host-command lease without changing M08's BNO watchdog.
MotionWatchdog commandWatchdog;
constexpr uint32_t MANUAL_COMMAND_TIMEOUT_MS = 250;
bool manualActive = false, manualEnding = false;
bool controlReady = false, bnoWatchdogReady = false, poseNeedsBno = true;
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
void stopManualSession(const char *reason);
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
    } else ++telemetryDrops;
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
#include "bno_lifecycle_diagnostics.h"
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
    const bool recoverable = !passed && poseActive;
    if (passed && poseActive && poseNeedsBno) learnTimingResponse();
    stopMotors(); motionWatchdog.disarm(); finalPrinted = true; window.active = false;
    commandWatchdog.disarm(); manualActive = manualEnding = false;
    manualYaw.request = manualPitch.request = manualCarriage.request = 0;
    phase = (passed || recoverable) ? Phase::COMPLETE : Phase::ABORTED;
    // Clearing a trip is permitted only after the independent stop has ended.
    if (phase == Phase::COMPLETE) motionWatchdog.clearTripWhenStopped();
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
        static_cast<unsigned long>(bnoWriteFailures), static_cast<unsigned long>(bnoMaxGap),
        static_cast<unsigned long>(accuracyGrace.episodes), static_cast<unsigned long>(accuracyGrace.recoveries),
        static_cast<unsigned long>(std::max(accuracyGrace.longestMs, accuracyGrace.age(millis()))),
        motionWatchdog.tripped() ? "YES" : "NO", reason, passed ? "PASS" : (recoverable ? "STOPPED" : "FAIL"),
        (passed || recoverable) ? "M09 READY" : "Latched abort; reset required. X/x aborts without automatic retry.");
    poseActive = false; carriagePending = false;
    queueText(summary);
    if (recoverable) { queueText("OPERATION FAILED: "); queueText(reason); queueText("\n"); }
}
void abortTest(const char *reason, bool latch = false) {
    if (latch) poseActive = false; // Explicit operator abort always remains latched.
    if (commandIdle()) finalPrinted = false;
    finish(false, reason);
}
bool checkSensorReset();
void safety() {
    checkSensorReset();
    if (manualActive) commandWatchdog.check(millis());
    if (manualActive && commandWatchdog.tripped()) {
        stopManualSession("command stream lost for 250 ms; rearm centered"); return;
    }
    if (manualActive || finalPrinted || !poseActive) return;
    if (millis() - controlStartedAt >= LEG_TIMEOUT_MS) { abortTest("Pose control timeout"); return; }
    if (!poseNeedsBno) return;
    const uint32_t now = millis(); bnoMaxGap = std::max(bnoMaxGap, now - lastBnoGood);
    if (motionWatchdog.tripped() || now - lastBnoGood >= BNO_STALE_MS) abortTest("BNO085 feedback stale: acquisition gap reached 150 ms");
    else if (yawRequired && bnoAccuracy < BNO_MIN_ACCURACY)
        abortTest("BNO085 accuracy below 2: yaw operation stopped");
    else if (accuracyRequired && accuracyGrace.expired(now)) abortTest("BNO085 accuracy below 2 continuously for 1000 ms");
    else if (bnoValid && fabs(heading.continuous - (referenceSet ? northTargetContinuous : pitchReadyYaw)) >= RELATIVE_LIMIT_DEG)
        abortTest(referenceSet ? "Measured yaw travel guard exceeded +/-185 degrees from continuous north target" :
            "Measured yaw travel guard exceeded +/-185 degrees from pitch-only startup orientation");
    else if (bnoValid && fabs(orientation.roll) >= MAX_USABLE_PITCH_DEG)
        abortTest("Measured pitch reached +/-75 degree absolute guard");
}
void invalidateOrientation(const char *reason) {
    const bool hadData = bnoValid || pitchReady || referenceSet;
    bnoValid = pitchReady = referenceSet = northUsable = false;
    heading = {}; window.active = false;
    if (hadData) { queueText("BNO WARNING: "); queueText(reason); queueText("\n"); }
    if (poseActive && poseNeedsBno) abortTest(reason);
}
bool checkSensorReset() {
    const uint32_t epoch = sensorWorker.resetEpoch.load(std::memory_order_acquire);
    if (epoch == sensorResetEpoch) return false;
    sensorResetEpoch = epoch;
    invalidateOrientation("sensor reset; orientation reference unavailable");
    return true;
}
void consumeSensorStatus() {
    if (sensorWorker.status.read(sensorSnapshot)) {
        bnoInitialized = sensorSnapshot.bnoInitialized;
        reportEnabled = sensorSnapshot.reportEnabled;
        bnoResets = sensorSnapshot.consumedResets;
        reportFailures = sensorSnapshot.reportFailures;
        bnoWriteFailures = sensorSnapshot.writeFailures;
        sensorSampleDrops = sensorSnapshot.sampleDrops;
        encoders = sensorSnapshot.encoders;
        static_cast<BnoIoStatistics &>(bnoTrace) = sensorSnapshot.io;
        bnoTrace.dropped += foregroundTraceDrops;
        if (sensorSnapshot.setupDone && !sensorSetupReported) {
            sensorSetupReported = true;
            if (!encoders.state(0).busAvailable) queueText("ENCODER WARNING: Bus A unavailable\n");
            if (!encoders.state(1).busAvailable) queueText("SENSOR WARNING: Bus B unavailable\n");
            if (!bnoInitialized || !reportEnabled)
                queueText("BNO WARNING: orientation unavailable; manual control remains available\n");
        }
    }
    BnoTraceEvent trace;
    for (unsigned n=0; n<8 && sensorWorker.traces.pop(trace); ++n) {
        if (bnoTrace.size == 32) { ++foregroundTraceDrops; ++bnoTrace.dropped; }
        else bnoTrace.events[(bnoTrace.head + bnoTrace.size++) % 32] = trace;
    }
}
void serviceBno() {
    consumeSensorStatus();
    checkSensorReset();
    if (bnoValid && millis() - lastBnoGood >= BNO_STALE_MS)
        invalidateOrientation("data stale; orientation reference unavailable");
    safety();
    m09::SensorSample sample;
    if (!sensorWorker.samples.pop(sample)) return;
    const auto &event = sample.event;
    bnoDiagnostics.observe(event, sample.receivedMs);
    if ((finalPrinted && !commandIdle()) || !reportEnabled ||
        sample.epoch != sensorWorker.resetEpoch.load(std::memory_order_acquire) ||
        millis() - sample.receivedMs >= BNO_STALE_MS || event.sensorId != SH2_ROTATION_VECTOR) {
        if (event.sensorId == SH2_ROTATION_VECTOR)
            bnoDiagnostics.reason = sample.epoch != sensorResetEpoch ? "reset" :
                (millis()-sample.receivedMs >= BNO_STALE_MS ? "stale_handoff" :
                 (!reportEnabled ? "report_disabled" : "inactive"));
        return;
    }
    EulerAngles result;
    if (!quaternionToEuler(event.un.rotationVector, result)) {
        bnoDiagnostics.reason = "invalid_quaternion";
        ++invalidVectors; window.interrupted = true; return;
    }
    bnoDiagnostics.euler = result; bnoDiagnostics.eulerValid = true;
    if (!heading.update(result.heading)) {
        bnoDiagnostics.reason = "ambiguous_heading";
        invalidateOrientation("ambiguous heading transition"); return;
    }
    const uint32_t now = sample.receivedMs; // Never make queued old feedback fresh at consumption.
    bnoHealth.recordFresh(now, pitchReady || referenceSet); orientation = result; bnoAccuracy = event.status;
    bnoDiagnostics.accepted = true; bnoDiagnostics.reason = "accepted";
    acceptedBnoSequence = event.sequence; acceptedBnoTimestamp = event.timestamp;
    bnoValid = true; lastBnoGood = now;
    if (poseActive && poseNeedsBno) {
        motionWatchdog.recordFresh(now);
    }
    if (accuracyRequired && referenceSet && !finalPrinted) {
        const bool wasLow = accuracyGrace.low;
        accuracyGrace.observe(event.status, now);
        northUsable = event.status >= BNO_MIN_ACCURACY;
        if (!wasLow && accuracyGrace.low) queueText("BNO ACCURACY LOW: north-dependent positioning unavailable\n");
        if (wasLow && !accuracyGrace.low) queueText("BNO ACCURACY RECOVERED: continuous low timer cleared\n");
    }
    northUsable = bnoAccuracy >= BNO_MIN_ACCURACY && referenceSet;
    safety(); if (phase == Phase::ABORTED) return;
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
// Qualify optional orientation references while idle; never command movement.
void serviceReference() {
    if (!commandIdle() || !fresh(millis()) || referenceSet) { window.active = false; return; }
    if (!window.active) { startBaseline(); return; }
    if (millis() - window.started < BASELINE_MS) return;
    if (stablePitchBaseline()) {
        if (!pitchReady) queueText("REFERENCE: stable pitch available; manual readiness unchanged\n");
        pitchReady = true; baselinePitch = window.pitch.mean(); pitchReadyYaw = heading.continuous;
    }
    if (stableBaseline()) {
        baselineHeading = wrap360(heading.first + window.heading.mean());
        northTargetContinuous = window.heading.mean() + shortestDifference(0, baselineHeading);
        referenceSet = northUsable = pitchReady = true;
        queueText("REFERENCE: qualified magnetic north available; no automatic movement\n");
        window.active = false;
    } else startBaseline();
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
    if (!poseNeedsBno) {
        if (carriagePending) {
            if (carriageMotor->setAcceleration(CARRIAGE_ACCELERATION) != 0 ||
                carriageMotor->setSpeedInHz(carriageRateHz) != 0 || carriageMotor->moveTo(carriageTarget) != MOVE_OK) {
                abortTest("FastAccelStepper carriage pose command rejected"); return;
            }
            carriagePending = false;
        }
        if (!carriageMotor->isRunning()) {
            if (carriageMotor->getCurrentPosition() == carriageTarget)
                finish(true, "Carriage generated-step target reached; no orientation requested");
            else abortTest("Carriage stopped before its requested target");
        }
        return;
    }
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
        if (poseActive) finish(true, !referenceSet ? "Pitch target stopped and settled; yaw disabled, north unverified" :
            "POSE complete: BNO angle targets settled; carriage generated-step target reached");
        else finish(true, "Measured axes stopped and settled");
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
    if (!controlReady || yawMotor->isRunning() || pitchMotor->isRunning() || carriageMotor->isRunning()) {
        rejectPose("initialized motor control and stopped motors required"); return;
    }
    const int64_t requestedCarriage = relative ? static_cast<int64_t>(carriageMotor->getCurrentPosition()) + carriageValue : carriageValue;
    const int64_t carriageDelta = requestedCarriage - carriageMotor->getCurrentPosition();
    if (requestedCarriage < INT32_MIN || requestedCarriage > INT32_MAX || carriageDelta < INT32_MIN || carriageDelta > INT32_MAX) {
        rejectPose("carriage target and displacement must fit signed 32-bit step counts"); return;
    }
    if (relative && yawValue == 0 && pitchValue == 0) {
        if (!carriageDelta) { queueText("POSE ALREADY AT TARGET: no motion\n"); return; }
        const double duration = milestone8::finiteSeconds(fabs(static_cast<double>(carriageDelta)), CARRIAGE_MAX_SPEED_HZ, CARRIAGE_ACCELERATION);
        if (!isfinite(duration) || duration * 1000 >= LEG_TIMEOUT_MS - SETTLE_MS) {
            rejectPose("estimated carriage move cannot meet existing motion deadline"); return;
        }
        poseNeedsBno = false; yawRequired = accuracyRequired = false;
        operation = Operation::POSE; poseActive = true; finalPrinted = false; phase = Phase::MOVING;
        carriageTarget = static_cast<int32_t>(requestedCarriage); carriagePending = true;
        carriageRateHz = CARRIAGE_MAX_SPEED_HZ; plannedDurationSeconds = duration;
        controlStartedAt = millis(); window.active = false;
        queueText("POSE ACCEPTED: relative carriage steps; angular axes stationary\n"); return;
    }
    if (!pitchReady || !fresh(millis()) || !bnoWatchdogReady || !motionWatchdog.clearTripWhenStopped()) {
        rejectPose("fresh valid BNO orientation and intact pitch baseline required"); return;
    }
    if (relative && (yawValue < -180.0 || yawValue >= 180.0)) {
        rejectPose("MOVE yaw delta must be in [-180,180) degrees; no implicit full-turn motion"); return;
    }
    const double requestedYaw = relative ? static_cast<double>(orientation.heading) + yawValue : yawValue;
    const double requestedPitch = relative ? static_cast<double>(orientation.roll) + pitchValue : pitchValue;
    if (!isfinite(requestedYaw) || !isfinite(requestedPitch) || fabs(requestedPitch) >= MAX_USABLE_PITCH_DEG) {
        rejectPose("finite yaw/pitch required; pitch target must be inside +/-75 degrees"); return;
    }
    const double targetYaw = heading.continuous + shortestDifference(requestedYaw, orientation.heading);
    if (referenceSet && fabs(targetYaw - northTargetContinuous) >= RELATIVE_LIMIT_DEG) {
        rejectPose("shortest yaw path endpoint exceeds existing +/-185-degree continuous north guard"); return;
    }
    const double yawError = targetYaw - heading.continuous, pitchError = requestedPitch - orientation.roll;
    const bool moveYaw = fabs(yawError) > TOLERANCE_DEG, movePitch = fabs(pitchError) > TOLERANCE_DEG;
    const double carriageDistance = fabs(static_cast<double>(requestedCarriage) - carriageMotor->getCurrentPosition());
    const bool pitchOnly = !referenceSet || bnoAccuracy < BNO_MIN_ACCURACY;
    if (pitchOnly && moveYaw) {
        rejectPose("yaw disabled: calibrated north baseline and BNO accuracy >=2 required"); return;
    }
    if (pitchOnly) {
        if (!movePitch && carriageDistance == 0) { queueText("POSE ALREADY AT TARGET: no motion; yaw disabled\n"); return; }
        if (milestone8::finiteSeconds(carriageDistance, CARRIAGE_MAX_SPEED_HZ, CARRIAGE_ACCELERATION) * 1000 >= LEG_TIMEOUT_MS - SETTLE_MS) {
            rejectPose("estimated carriage move cannot meet existing motion deadline"); return;
        }
        // A startup timing measurement may not exist here. Run the existing
        // native pitch controller without inventing a pulses/degree estimate.
        resetPoseAxis(yawAxis, heading.continuous, yawTimingPeak);
        resetPoseAxis(pitchAxis, requestedPitch, pitchTimingPeak);
        yawRequired = accuracyRequired = false;
        poseNeedsBno = true;
        carriageTarget = static_cast<int32_t>(requestedCarriage); carriagePending = carriageDistance > 0;
        carriageRateHz = CARRIAGE_MAX_SPEED_HZ;
        yawRateCap = YAW_SLEW_SPEED_HZ; pitchRateCap = PITCH_SLEW_SPEED_HZ;
        plannedDurationSeconds = 0; timingLimited = true;
        operation = Operation::POSE; poseActive = true; finalPrinted = false; phase = Phase::MOVING;
        settleSamples = 0; window.active = false; accuracyGrace = {};
        sawYawMotion = sawPitchMotion = concurrentMotion = concurrentThreeMotion = false;
        controlStartedAt = millis(); lastControlSample = bnoHealth.freshSamples;
        recordTimingStarts();
        if (!motionWatchdog.arm(lastBnoGood)) { abortTest("BNO watchdog could not arm for pitch-only motion"); return; }
        queueText("PITCH-ONLY ACCEPTED: native pitch control; yaw stationary; carriage uses requested steps\n");
        safety(); return;
    }
    if ((moveYaw && yawPulsesPerDegree <= 0) || (movePitch && pitchPulsesPerDegree <= 0)) {
        rejectPose("angular timing response unavailable; manual control remains ready (no automatic calibration movement)"); return;
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
    accuracyRequired = moveYaw;
    poseNeedsBno = true;
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
            commandLength = 0; commandOverflow = false; abortTest("Operator X abort", true); return;
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
void sensorTelemetry() {
    const uint32_t now = millis();
    char line[850];
    snprintf(line, sizeof(line), "BNO_STATE available=%s fresh=%s age_ms=%lu accuracy=%u north_usable=%s heading=%.3f pitch_roll=%.3f has_sample=%s accepted_seq=%u accepted_sensor_us=%llu accepted_rx_ms=%lu\n",
        bnoInitialized && reportEnabled ? "YES" : "NO", fresh(now) ? "YES" : "NO",
        static_cast<unsigned long>(now - lastBnoGood), bnoAccuracy, northUsable && fresh(now) ? "YES" : "NO",
        orientation.heading, orientation.roll, bnoHealth.freshSamples ? "YES" : "NO", acceptedBnoSequence,
        static_cast<unsigned long long>(acceptedBnoTimestamp), static_cast<unsigned long>(lastBnoGood));
    queueText(line);
    const auto &raw = bnoDiagnostics;
    snprintf(line, sizeof(line), "BNO_REPORT seen=%s report_id=0x%02X raw_status=%u seq=%u sensor_us=%llu age_ms=%lu observed_events=%lu rotation_events=%lu other_events=%lu status0=%lu status1=%lu status2=%lu status3=%lu status_changes=%lu invalid_vectors=%lu resets=%lu report_failures=%lu tx_dropped=%lu\n",
        raw.events ? "YES" : "NO", raw.lastReport, raw.lastStatus, raw.lastSequence,
        static_cast<unsigned long long>(raw.lastTimestamp), static_cast<unsigned long>(raw.events ? now - raw.lastEventAt : UINT32_MAX),
        static_cast<unsigned long>(raw.events), static_cast<unsigned long>(raw.rotations), static_cast<unsigned long>(raw.otherReports),
        static_cast<unsigned long>(raw.statusCounts[0]), static_cast<unsigned long>(raw.statusCounts[1]),
        static_cast<unsigned long>(raw.statusCounts[2]), static_cast<unsigned long>(raw.statusCounts[3]),
        static_cast<unsigned long>(raw.statusChanges), static_cast<unsigned long>(invalidVectors),
        static_cast<unsigned long>(bnoResets), static_cast<unsigned long>(reportFailures), static_cast<unsigned long>(telemetryDrops));
    queueText(line);
    const auto &q = raw.rotation;
    // Nine significant digits preserve library float components for comparison.
    // No quaternion is synthesized for unrelated reports or a never-seen sensor.
    const double norm = raw.rotations ? static_cast<double>(q.real)*q.real + static_cast<double>(q.i)*q.i +
        static_cast<double>(q.j)*q.j + static_cast<double>(q.k)*q.k : NAN;
    snprintf(line, sizeof(line), "BNO_RAW has_sample=%s report_id=0x%02X seq=%u sensor_us=%llu rx_ms=%lu age_ms=%lu raw_status=%u q_w=%.9g q_x=%.9g q_y=%.9g q_z=%.9g heading_accuracy_rad=%.9g norm_sq=%.9g euler_valid=%s yaw_deg=%.9g pitch_deg=%.9g roll_deg=%.9g accepted=%s reason=%s diagnostic_quality=%s malformed_total=%lu\n",
        raw.rotations ? "YES" : "NO", SH2_ROTATION_VECTOR, raw.sequence, static_cast<unsigned long long>(raw.timestamp),
        static_cast<unsigned long>(raw.rotationAt), static_cast<unsigned long>(raw.rotations ? now - raw.rotationAt : UINT32_MAX),
        raw.status, raw.rotations ? q.real : NAN, raw.rotations ? q.i : NAN, raw.rotations ? q.j : NAN, raw.rotations ? q.k : NAN,
        raw.rotations ? q.accuracy : NAN, norm, raw.eulerValid ? "YES" : "NO",
        raw.euler.heading, raw.euler.pitch, raw.euler.roll, raw.accepted ? "YES" : "NO", raw.reason,
        !raw.rotations ? "NO_SAMPLE" : (raw.plausible ? "PLAUSIBLE" : "MALFORMED"), (unsigned long)raw.malformed);
    queueText(line);
    snprintf(line, sizeof(line), "BNO_IO at_ms=%lu calls=%lu failures=%lu slow=%lu max_us=%lu acquire_max_us=%lu encB_service_max_us=%lu reset_events=%lu product_generation=%lu queried_generation=%lu trace_dropped=%lu sample_handoff_dropped=%lu\n",
        (unsigned long)now, (unsigned long)bnoTrace.ioCalls, (unsigned long)bnoTrace.ioFailures,
        (unsigned long)bnoTrace.ioSlow, (unsigned long)bnoTrace.ioMaxUs, (unsigned long)bnoTrace.acquireMaxUs,
        (unsigned long)bnoTrace.encoderMaxUs, (unsigned long)bnoTrace.resetEvents,
        (unsigned long)bnoTrace.productGeneration, (unsigned long)bnoTrace.queriedGeneration, (unsigned long)bnoTrace.dropped, (unsigned long)sensorSampleDrops);
    queueText(line);
    for (unsigned index = 0; index < 2; ++index) {
        const auto &sample = encoders.state(index);
        snprintf(line, sizeof(line), "ENCODER_STATE bus=%c available=%s valid=%s raw=%u angle_deg=%.3f age_ms=%lu status=0x%02X magnet_good=%s failures=%lu\n",
            index ? 'B' : 'A', sample.available ? "YES" : "NO", sample.valid ? "YES" : "NO",
            sample.raw, sample.angleDegrees(), static_cast<unsigned long>(sample.ageMs(now)), sample.status,
            sample.magnetGood() ? "YES" : "NO", static_cast<unsigned long>(sample.failures));
        queueText(line);
    }
}
} // namespace

// Called synchronously by the locally instrumented pinned library. The callback
// timestamp is MCU observation time, not an assertion about physical reset time.
extern "C" void m09_bno_io(uint32_t start, uint32_t duration, unsigned stage, unsigned bytes, int ok) {
    sensorWorker.onIo(start, duration, stage, bytes, ok);
}
extern "C" void m09_bno_reset(uint32_t transportUs) {
    sensorWorker.onReset(transportUs);
}
extern "C" void m09_bno_product(uint32_t transportUs, uint8_t cause, uint32_t part, uint32_t build) {
    sensorWorker.onProduct(transportUs, cause, part, build);
}
extern "C" void m09_bno_init_response(uint32_t transportUs) {
    sensorWorker.onInitResponse(transportUs);
}

#ifdef M07_HOST_TEST
void loop();
void sensorWorkerTestWait(uint32_t duration) {
    const uint32_t start = millis();
    while (millis()-start < duration) loop();
}
#endif

void setup() {
    Serial.begin(115200);
    for (const uint8_t pin : {tmp_hardware::YAW_STEP_PIN, tmp_hardware::PITCH_STEP_PIN, tmp_hardware::CARRIAGE_STEP_PIN}) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
    Serial.println("M09_xbox_control: manual velocity; optional AS5600/BNO telemetry; no automatic startup movement");
    Serial.println("BNO_DIAGNOSTICS revision=sensor-worker-1 comparison=POST_RELOCATION_USER_REPORTED timestamp=header_receipt_us");
    Serial.println("Yaw DIR32/STEP33; pitch DIR26/STEP12; carriage DIR21/STEP22. Physical pitch feedback = BNO ROLL.");
    Serial.println("AS5600 Bus A SDA18/SCL19 and Bus B SDA4/SCL5 at 0x36; BNO Bus B at 0x4A.");
    Serial.println("Manual travel is operator-supervised. Carriage steps are unhomed; no software position window.");
    Serial.println("Centered JOG arms; STOP brakes; X/x latches abort. Continuous commands expire after 250 ms.");
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
    if (!commandWatchdog.begin(yawMotor, pitchMotor, MANUAL_COMMAND_TIMEOUT_MS, carriageMotor)) { abortTest("Independent manual command watchdog initialization failed"); return; }
    bnoWatchdogReady = motionWatchdog.begin(yawMotor, pitchMotor, BNO_STALE_MS, carriageMotor);
    if (!bnoWatchdogReady) queueText("BNO WARNING: feedback watchdog unavailable; angular POSE disabled\n");
    controlReady = true;
    startedAt = lastBnoGood = millis(); bnoHealth.lastFreshOrStartMs = startedAt;
    yawRequired = accuracyRequired = false;
    phase = Phase::COMPLETE; finalPrinted = true;
    publishSensorContext();
    if (!sensorWorker.start()) queueText("SENSOR WARNING: worker unavailable; manual control remains available\n");
#ifdef M07_HOST_TEST
    simulated::sensorWait = sensorWorkerTestWait;
#endif
    queueText("M09 READY\n");
}
void loop() {
    serviceCommands();
    if (phase == Phase::ABORTED) { publishSensorContext(); consumeSensorStatus(); serviceTraceOutput(); serviceSerialOutput(); delay(1); return; }
    safety();
    if (manualActive) serviceManual(); // STOP reaches motor braking before sensor calls.
    serviceBno();
    serviceCommands(); safety();
    if (manualActive) serviceManual();
    serviceCommands(); safety();
    if (manualActive) serviceManual();
    if (poseActive) serviceAxes();
    serviceReference();
    publishSensorContext();
    serviceTraceOutput();
    const uint32_t now = millis();
    if (now - lastSensorDisplay >= 500) { lastSensorDisplay = now; sensorTelemetry(); }
    if (!manualActive && now - lastDisplay >= 750) { lastDisplay = now; telemetry(); }
    serviceSerialOutput();
#ifdef M07_HOST_TEST
    sensorWorker.testDispatch();
#endif
    delay(1);
}
