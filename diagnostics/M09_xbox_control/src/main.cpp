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
#include <esp_system.h>
#endif
#include "hardware_config.h"
#include "sensor_support.h"
#include "control_math.h"
#include "motion_watchdog.h"
#include "pose_math.h"
#include "keyframe_math.h"
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
constexpr double RUNAWAY_ERROR_RISE_DEG = 0.6;
constexpr uint8_t RUNAWAY_CONFIRM_SAMPLES = 3;
constexpr uint32_t RUNAWAY_CONFIRM_MS = 100;
constexpr double NORTH_RUNAWAY_ERROR_RISE_DEG = 1.5;
constexpr uint8_t NORTH_RUNAWAY_CONFIRM_SAMPLES = 4;
constexpr uint32_t NORTH_RUNAWAY_CONFIRM_MS = 200;
// First operator-commanded POSE: bounded precision corrections until measured
// motion supplies timing. No assumed gearbox ratio or automatic startup move.
constexpr double FIRST_POSE_MAX_ERROR_DEG = 3.0;
constexpr uint32_t FIRST_POSE_SPEED_HZ = 80;
constexpr int32_t FIRST_POSE_BURST_STEPS = 16;
// Earlier three-axis diagnostics used these carriage speed/acceleration values.
// Position is generated pulses from power-up, not a homed or measured distance.
constexpr uint32_t CARRIAGE_MAX_SPEED_HZ = 2000;
constexpr int32_t CARRIAGE_ACCELERATION = 2000;
// Generated-step/manual travel is independent of the proven POSE precision caps.
constexpr uint32_t PITCH_TRAVEL_SPEED_HZ = 2400;
constexpr int32_t PITCH_TRAVEL_ACCELERATION = 2400;
enum class Operation { NORTH_LEVEL, POSE, MANUAL, LEVEL, NORTH, CELESTIAL };

enum class Phase { STARTUP, BASELINE, MOVING, SETTLING, COMPLETE, ABORTED, MANUAL };
enum class Motion { PRECISION, SLEW, BRAKING, HOLD, TRACK, TRACK_ZERO };
struct Axis {
    FastAccelStepper *motor = nullptr;
    bool pitch = false, settled = false, confirmed = false;
    double current = 0, target = 0, error = 0, bestError = 1e9, progressError = 1e9;
    double previousErrorMagnitude = 0;
    Motion motion = Motion::PRECISION;
    bool finiteActive = false, observing = false, slewFinished = false, velocityReady = false;
    bool previousErrorValid = false;
    double initialError = 0, velocityPosition = 0, angularSpeed = 0, peakAngularSpeed = 0, brakeAtDeg = 0;
    int slewDirection = 0;
    uint32_t bursts = 0, slewStarts = 0, motionSamples = 0, lastProgress = 0;
    uint8_t divergenceSamples = 0;
    uint32_t divergenceSince = 0;
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
    uint32_t rate = 0, brakeAt = 0, stoppedAt = 0, startedAt = 0, progressAt = 0;
    bool braking = false, observing = false, runningObserved = false;
    double start = 0, best = 0, progress = 0;
};
ManualAxis manualYaw, manualPitch, manualCarriage;
AccuracyGrace accuracyGrace;
BnoHealth bnoHealth;
EulerAngles orientation = {0, 0, 0};
EulerAngles lastPlausibleOrientation = {0, 0, 0};
bool hasPlausibleOrientation = false;
uint32_t lastPlausibleAt = 0;
uint8_t lastPlausibleAccuracy = 0;
// The post-relocation powered log shows pitch-only motion in Euler pitch,
// with roll nearly unchanged. Keep the verified motor signs; select feedback
// consistently for this mounting, including qualification and travel guards.
double physicalPitch(const EulerAngles &sample) { return sample.pitch; }
double physicalPitch() { return physicalPitch(orientation); }
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
bool keyframeActive = false;
bool poseStopping = false, posePrecisionOnly = false, poseStoppedObserved = false;
uint32_t poseStoppedAt = 0;
bool concurrentThreeMotion = false;
int32_t carriageTarget = 0;
uint32_t yawRateCap = YAW_SLEW_SPEED_HZ, pitchRateCap = PITCH_SLEW_SPEED_HZ, carriageRateHz = CARRIAGE_MAX_SPEED_HZ;
double plannedDurationSeconds = 0, yawPulsesPerDegree = 0, pitchPulsesPerDegree = 0;
double yawTimingPeak = 0, pitchTimingPeak = 0, yawStartAngle = 0, pitchStartAngle = 0;
int32_t yawStartSteps = 0, pitchStartSteps = 0;
char commandLine[128];
size_t commandLength = 0;
bool commandOverflow = false;

bool celestialTracking = false, celestialFailed = false;
uint32_t celestialId = 0;
bool celestialActive() { return poseActive && operation == Operation::CELESTIAL; }
void stopCelestial(const char *reason, bool failed = true);
void celestialSafety();
void updateCelestialTiming(Axis &axis);
uint32_t celestialProgressTimeout(const Axis &axis);
void enterCelestialTracking();
void celestialResult(const char *reason, bool latched);
void celestialBnoUnavailable(const char *reason);
void celestialRecordTrustedBno();
void celestialRecordBnoRecovery(const EulerAngles &sample, uint8_t accuracy, uint32_t sampleAt);
void resumeCelestialBnoFeedback(uint32_t now);
bool celestialUsingEncoderFeedback();
bool celestialControlFeedbackReady();
bool serviceCelestialEncoderFeedback(uint32_t now, uint32_t &sampleAt, bool &newSample);
void serviceCelestialTracking(uint32_t now, uint32_t sampleAt);

bool alignmentActive() { return poseActive && (operation == Operation::LEVEL || operation == Operation::NORTH); }
bool pitchControlRequired() { return !alignmentActive() || operation == Operation::LEVEL; }
const char *alignmentName() { return operation == Operation::LEVEL ? "LEVEL" : "NORTH"; }
// LEVEL continuous slew and finite precision use separate FastAccelStepper APIs.
// Powered tests established the current slew correction below, while the later
// near-target test established that finite move() must retain the legacy signed
// step convention. Manual and POSE/MOVE continue to use their existing mapping.
constexpr int LEVEL_POSITIVE_BNO_PITCH_STEP_SIGN = -1;
int bnoSlewStepSign(bool pitch) {
    return pitch && alignmentActive() && operation == Operation::LEVEL ?
        LEVEL_POSITIVE_BNO_PITCH_STEP_SIGN :
        (pitch ? POSITIVE_STEP_PITCH_SIGN : TRIAL_POSITIVE_STEP_YAW_SIGN);
}
int bnoSlewStepDirection(double error, bool pitch) {
    return (error > 0 ? 1 : -1) * bnoSlewStepSign(pitch);
}
int32_t bnoPrecisionCorrectionSteps(double error, bool confirmed, bool pitch) {
    // Signed relative move() convention remains the physically established
    // POSITIVE_STEP_PITCH_SIGN path; do not reuse LEVEL's continuous-run sign.
    return correctionSteps(error, confirmed, pitch);
}
int bnoFeedbackStepSign(const Axis &axis) {
    if (axis.pitch && alignmentActive() && operation == Operation::LEVEL &&
        (axis.motion == Motion::SLEW || axis.motion == Motion::BRAKING)) {
        return LEVEL_POSITIVE_BNO_PITCH_STEP_SIGN;
    }
    return axis.pitch ? POSITIVE_STEP_PITCH_SIGN : TRIAL_POSITIVE_STEP_YAW_SIGN;
}
void abortAlignment(const char *reason);
void pauseAlignment(const char *reason);
bool alignmentSafety();
void updateAlignmentTiming(Axis &axis);
uint32_t alignmentProgressTimeout(const Axis &axis);
bool alignmentPaused = false;
bool alignmentSampleInvalid = false;
double alignmentPulsesPerDegree = 0;

bool commandIdle() { return finalPrinted && phase == Phase::COMPLETE; }
// All compared intervals are far below half the millis() range.
bool sampleAtOrAfter(uint32_t sampleAt, uint32_t boundary) {
    return sampleAt - boundary < 0x80000000UL;
}
void stopManualSession(const char *reason);
void invalidateKeyframes(const char *reason);
void stopKeyframe(const char *reason, bool failure = false, bool latch = false);
void keyframeSafety();
uint32_t limitedSpeed(const Axis &axis, uint32_t native) {
    return poseActive ? std::min(native, axis.pitch ? pitchRateCap : yawRateCap) : native;
}
void recordTimingStarts() {
    yawStartAngle = heading.continuous; pitchStartAngle = physicalPitch();
    yawStartSteps = yawMotor->getCurrentPosition(); pitchStartSteps = pitchMotor->getCurrentPosition();
}
void learnTimingResponse() {
    const double yawDelta = heading.continuous - yawStartAngle, pitchDelta = physicalPitch() - pitchStartAngle;
    const double yawSteps = static_cast<double>(yawMotor->getCurrentPosition()) - yawStartSteps;
    const double pitchSteps = static_cast<double>(pitchMotor->getCurrentPosition()) - pitchStartSteps;
    // Completed, stopped BNO motion supplies a timing estimate only. These
    // counts never replace BNO angle feedback or claim a calibrated gearbox.
    if (fabs(yawDelta) >= DIRECTION_RESPONSE_DEG && fabs(yawSteps) >= 8 &&
        yawSteps * yawDelta * TRIAL_POSITIVE_STEP_YAW_SIGN > 0) {
        yawPulsesPerDegree = fabs(yawSteps / yawDelta); yawTimingPeak = yawAxis.peakAngularSpeed;
    }
    if (operation == Operation::LEVEL && alignmentPulsesPerDegree > 0) {
        pitchPulsesPerDegree = alignmentPulsesPerDegree;
        pitchTimingPeak = pitchAxis.peakAngularSpeed;
    } else if (fabs(pitchDelta) >= DIRECTION_RESPONSE_DEG && fabs(pitchSteps) >= 8 &&
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
    case Motion::TRACK: return "TRACK";
    case Motion::TRACK_ZERO: return "TRACK_ZERO";
    }
    return "UNKNOWN";
}
void setMotion(Axis &axis, Motion motion) {
    const Motion previous = axis.motion;
    axis.motion = motion;
    // A continuous slew's best error is not a useful baseline for precision:
    // the motor can coast while braking, then BNO/filter noise settles after
    // it stops. Seed each side of that transition from its own measured error.
    if (alignmentActive() && operation == Operation::NORTH &&
        (motion == Motion::BRAKING || (previous == Motion::BRAKING && motion == Motion::PRECISION))) {
        axis.bestError = fabs(axis.error);
        axis.previousErrorMagnitude = fabs(axis.error);
        axis.previousErrorValid = true;
        axis.divergenceSamples = 0;
        axis.divergenceSince = 0;
    }
    char line[160];
    snprintf(line, sizeof(line), "AXIS %s -> %s error_deg=%.3f brake_at_deg=%.3f\n",
        axis.pitch ? "PITCH" : "YAW", motionText(axis), axis.error, axis.brakeAtDeg);
    queueText(line);
}
#include "pitch_direction_diagnostics.h"

const char *phaseText() {
    if (celestialActive()) return celestialTracking ? "CELESTIAL_TRACK" : "CELESTIAL_GOTO";
    switch (phase) { case Phase::STARTUP: return "STARTUP"; case Phase::BASELINE: return "BASELINE";
    case Phase::MOVING: return "MOVING"; case Phase::SETTLING: return "SETTLING";
    case Phase::COMPLETE: return "COMPLETE"; case Phase::ABORTED: return "ABORTED";
    case Phase::MANUAL: return "MANUAL"; }
    return "UNKNOWN";
}
bool fresh(uint32_t now) { return bnoValid && reportEnabled && now - lastBnoGood < BNO_STALE_MS; }
const char *northUnavailableReason(uint32_t now) {
    if (!bnoInitialized) return "bno_unavailable";
    if (!reportEnabled) return "rotation_report_disabled";
    if (alignmentSampleInvalid) return "invalid_orientation_sample";
    if (!hasPlausibleOrientation) return "heading_unavailable";
    if (now - lastPlausibleAt >= BNO_STALE_MS) return "orientation_stale";
    if (!bnoValid) return "orientation_not_accepted_current_epoch";
    if (!heading.initialized || !isfinite(orientation.heading)) return "heading_invalid";
    if (bnoAccuracy < BNO_MIN_ACCURACY) return "accuracy_below_2";
    return "usable";
}
bool northHeadingUsable(uint32_t now) { return strcmp(northUnavailableReason(now), "usable") == 0; }
#include "bno_lifecycle_diagnostics.h"
void stopMotors() {
    const bool interrupted = (yawMotor && yawMotor->isRunning()) || (pitchMotor && pitchMotor->isRunning()) ||
        (carriageMotor && carriageMotor->isRunning());
    if (yawMotor && yawMotor->isRunning()) yawMotor->forceStop();
    if (pitchMotor && pitchMotor->isRunning()) pitchMotor->forceStop();
    if (carriageMotor && carriageMotor->isRunning()) carriageMotor->forceStop();
    delay(25);
    digitalWrite(tmp_hardware::YAW_STEP_PIN, LOW);
    digitalWrite(tmp_hardware::PITCH_STEP_PIN, LOW);
    digitalWrite(tmp_hardware::CARRIAGE_STEP_PIN, LOW);
    if (interrupted) invalidateKeyframes("forced motor stop; recapture required");
}
void finish(bool passed, const char *reason) {
    if (finalPrinted) return;
    const bool wasAlignment = operation == Operation::LEVEL || operation == Operation::NORTH;
    const bool wasCelestial = operation == Operation::CELESTIAL;
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
        wasCelestial ? "CELESTIAL" : (wasAlignment ? alignmentName() : (operation == Operation::MANUAL ? "XBOX MANUAL" : (operation == Operation::POSE ? "GO TO POSE" : "PRESERVED M08 NORTH / LEVEL"))),
        orientation.heading, bnoAccuracy, northUsable ? "YES" : "NO",
        wrap360(heading.first + yawAxis.target), pitchAxis.target, baselineHeading, baselinePitch,
        wasAlignment && operation == Operation::LEVEL ? "NOT_REQUESTED" : (yawAxis.settled ? "YES" : "NO"), yawAxis.error, static_cast<unsigned long>(yawAxis.bursts),
        wasAlignment && operation == Operation::NORTH ? "NOT_REQUESTED" : (pitchAxis.settled ? "YES" : "NO"), pitchAxis.error, static_cast<unsigned long>(pitchAxis.bursts),
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
    poseActive = poseStopping = alignmentPaused = false; carriagePending = false;
    if (wasCelestial) celestialResult(reason, phase == Phase::ABORTED);
    // Terminal operation marker precedes READY so the existing host handoff
    // never has to guess whether this particular request finished.
    if (wasAlignment) {
        queueText(alignmentName()); queueText(" RESULT: ");
        queueText(passed ? "PASS\n" : (recoverable ? "STOPPED\n" : "FAIL\n"));
    }
    queueText(summary);
    if (recoverable) { queueText("OPERATION FAILED: "); queueText(reason); queueText("\n"); }
}
void abortTest(const char *reason, bool latch = false) {
    if (keyframeActive) { stopKeyframe(reason, true, latch); return; }
    if (celestialActive() && !latch) { stopCelestial(reason); return; }
    if (alignmentActive() && !latch) { abortAlignment(reason); return; }
    if (latch) poseActive = false; // Explicit operator abort always remains latched.
    if (commandIdle()) finalPrinted = false;
    finish(false, reason);
    invalidateKeyframes(reason);
}
#include "pose_stop.h"
#include "orientation_commands.h"
bool checkSensorReset();
void safety() {
    checkSensorReset();
    if (keyframeActive) { keyframeSafety(); return; }
    if (manualActive) commandWatchdog.check(millis());
    if (manualActive && commandWatchdog.tripped()) {
        stopManualSession("command stream lost for 250 ms; rearm centered"); return;
    }
    if (manualActive || finalPrinted || !poseActive || poseStopping) return;
    if (celestialActive()) { celestialSafety(); return; }
    // LEVEL/NORTH have selected-axis guards and recoverable feedback handling.
    // Do not apply combined-POSE yaw/pitch guards to their inactive axes.
    if (alignmentActive()) { alignmentSafety(); return; }
    if (!alignmentActive() && millis() - controlStartedAt >= LEG_TIMEOUT_MS) { abortTest("Pose control timeout"); return; }
    if (!poseNeedsBno) return;
    const uint32_t now = millis(); bnoMaxGap = std::max(bnoMaxGap, now - lastBnoGood);
    if (motionWatchdog.tripped() || now - lastBnoGood >= BNO_STALE_MS) abortTest("BNO085 feedback stale: acquisition gap reached 150 ms");
    else if (bnoValid && fabs(heading.continuous - (referenceSet ? northTargetContinuous : pitchReadyYaw)) >= RELATIVE_LIMIT_DEG)
        abortTest(referenceSet ? "Measured yaw travel guard exceeded +/-185 degrees from continuous north target" :
            "Measured yaw travel guard exceeded +/-185 degrees from pitch-only startup orientation");
    else if (bnoValid && fabs(physicalPitch()) >= MAX_USABLE_PITCH_DEG)
        abortTest("Measured pitch reached +/-75 degree absolute guard");
}
void invalidateOrientation(const char *reason) {
    const bool hadData = bnoValid || pitchReady || referenceSet;
    if (celestialActive() && !poseStopping) {
        bnoValid = pitchReady = referenceSet = northUsable = false;
        window.active = false;
        if (hadData) { queueText("BNO WARNING: "); queueText(reason); queueText("\n"); }
        celestialBnoUnavailable(reason);
        return;
    }
    bnoValid = pitchReady = referenceSet = northUsable = false;
    heading = {}; window.active = false;
    if (hadData) { queueText("BNO WARNING: "); queueText(reason); queueText("\n"); }
    if (alignmentActive() && !poseStopping) pauseAlignment(reason);
    else if (poseActive && poseNeedsBno && !poseStopping) abortTest(reason);
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
    EulerAngles plausible;
    if (sample.epoch == sensorWorker.resetEpoch.load(std::memory_order_acquire) &&
        event.sensorId == SH2_ROTATION_VECTOR && bnoDiagnostics.plausible &&
        quaternionToEuler(event.un.rotationVector, plausible) &&
        (!hasPlausibleOrientation || sampleAtOrAfter(sample.receivedMs, lastPlausibleAt))) {
        lastPlausibleOrientation = plausible; lastPlausibleAt = sample.receivedMs;
        lastPlausibleAccuracy = event.status; hasPlausibleOrientation = true;
    }
    if ((finalPrinted && !commandIdle()) || !reportEnabled ||
        sample.epoch != sensorWorker.resetEpoch.load(std::memory_order_acquire) ||
        millis() - sample.receivedMs >= BNO_STALE_MS || event.sensorId != SH2_ROTATION_VECTOR) {
        if (event.sensorId == SH2_ROTATION_VECTOR)
            bnoDiagnostics.reason = sample.epoch != sensorResetEpoch ? "reset" :
                (millis()-sample.receivedMs >= BNO_STALE_MS ? "stale_handoff" :
                 (!reportEnabled ? "report_disabled" : "inactive"));
        return;
    }
    if (bnoValid && (sample.receivedMs == lastBnoGood || !sampleAtOrAfter(sample.receivedMs, lastBnoGood))) {
        bnoDiagnostics.reason = "out_of_order_receipt"; return;
    }
    EulerAngles result;
    if (!bnoDiagnostics.plausible || !quaternionToEuler(event.un.rotationVector, result)) {
        bnoDiagnostics.reason = "invalid_quaternion";
        alignmentSampleInvalid = true;
        if (alignmentActive() || celestialActive()) invalidateOrientation("invalid BNO orientation");
        ++invalidVectors; window.interrupted = true; return;
    }
    bnoDiagnostics.euler = result; bnoDiagnostics.eulerValid = true;
    if (celestialActive() && event.status < BNO_MIN_ACCURACY) {
        bnoAccuracy = event.status;
        bnoValid = false;
        bnoDiagnostics.reason = "accuracy_below_2";
        celestialBnoUnavailable("BNO accuracy below 2");
        return;
    }
    if (celestialActive() && celestialUsingEncoderFeedback()) {
        celestialRecordBnoRecovery(result, event.status, sample.receivedMs);
        bnoDiagnostics.accepted = true; bnoDiagnostics.reason = "accepted_recovery_report_only";
        acceptedBnoSequence = event.sequence; acceptedBnoTimestamp = event.timestamp;
        return;
    }
    if (!heading.update(result.heading)) {
        bnoDiagnostics.reason = "ambiguous_heading";
        invalidateOrientation("ambiguous heading transition"); return;
    }
    const uint32_t now = sample.receivedMs; // Never make queued old feedback fresh at consumption.
    bnoHealth.recordFresh(now, pitchReady || referenceSet); orientation = result; bnoAccuracy = event.status;
    bnoDiagnostics.accepted = true; bnoDiagnostics.reason = "accepted";
    acceptedBnoSequence = event.sequence; acceptedBnoTimestamp = event.timestamp;
    bnoValid = true; lastBnoGood = now;
    alignmentSampleInvalid = false;
    if (celestialActive()) {
        celestialRecordTrustedBno();
        resumeCelestialBnoFeedback(millis());
    }
    if (poseActive && poseNeedsBno && !poseStopping && !alignmentPaused) {
        motionWatchdog.recordFresh(now);
    }
    if (accuracyRequired && referenceSet && !finalPrinted) {
        const bool wasLow = accuracyGrace.low;
        accuracyGrace.observe(event.status, now);
        northUsable = northHeadingUsable(millis());
        if (!wasLow && accuracyGrace.low) queueText("BNO ACCURACY LOW: north-dependent positioning unavailable\n");
        if (wasLow && !accuracyGrace.low) queueText("BNO ACCURACY RECOVERED: continuous low timer cleared\n");
    }
    // Direct NORTH readiness describes this accepted magnetic-heading sample.
    // referenceSet remains the separate, idle-qualified baseline used by POSE.
    northUsable = northHeadingUsable(now);
    safety(); if (phase == Phase::ABORTED) return;
    if (yawMotor && yawMotor->isRunning()) { ++yawAxis.motionSamples; sawYawMotion = true; }
    if (pitchMotor && pitchMotor->isRunning()) { ++pitchAxis.motionSamples; sawPitchMotion = true; }
    if (yawMotor && yawMotor->isRunning() && pitchMotor && pitchMotor->isRunning()) concurrentMotion = true;
    if (poseActive && yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning()) concurrentThreeMotion = true;
    if (window.active && sampleAtOrAfter(now, window.started)) {
        if (event.status < BNO_MIN_ACCURACY) window.calibrationInterrupted = true;
        else ++accuracyGood;
        if (now - window.lastBno > WINDOW_GAP_MS) window.interrupted = true;
        window.lastBno = now; window.heading.add(heading.continuous); window.pitch.add(physicalPitch(result));
        if (now - window.started < BASELINE_MS / 2) { window.firstHeading.add(heading.continuous); window.firstPitch.add(physicalPitch(result)); }
        else { window.secondHeading.add(heading.continuous); window.secondPitch.add(physicalPitch(result)); }
    }
}
void startBaseline() { window = {}; accuracyGood = 0; window.active = true; window.started = window.lastBno = millis(); }
bool stablePitchBaseline() {
    const uint32_t now = millis();
    return !window.interrupted && sampleAtOrAfter(window.lastBno, window.started + BASELINE_MS) && window.pitch.n >= 30 &&
        window.firstPitch.n && window.secondPitch.n && now - window.lastBno <= WINDOW_GAP_MS && fresh(now) &&
        window.pitch.sd() <= 0.15 && window.pitch.range() <= 0.5 &&
        fabs(window.secondPitch.mean() - window.firstPitch.mean()) <= 0.2;
}
bool stableBaseline() {
    const uint32_t now = millis();
    return !window.interrupted && !window.calibrationInterrupted && sampleAtOrAfter(window.lastBno, window.started + BASELINE_MS) && window.heading.n >= 30 && window.pitch.n >= 30 && accuracyGood >= 30 &&
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
    if (!window.interrupted && !sampleAtOrAfter(window.lastBno, window.started + BASELINE_MS)) return;
    if (stablePitchBaseline()) {
        if (!pitchReady) queueText("REFERENCE: stable pitch available; manual readiness unchanged\n");
        pitchReady = true; baselinePitch = window.pitch.mean(); pitchReadyYaw = heading.continuous;
    }
    if (stableBaseline()) {
        baselineHeading = wrap360(heading.first + window.heading.mean());
        northTargetContinuous = window.heading.mean() + shortestDifference(0, baselineHeading);
        referenceSet = pitchReady = true;
        northUsable = northHeadingUsable(millis());
        queueText("REFERENCE: qualified magnetic north available; no automatic movement\n");
        window.active = false;
    } else startBaseline();
}
bool axisProgress(Axis &axis, double error) {
    const double magnitude = fabs(error); const uint32_t now = millis(); axis.error = error;
    const bool northRunaway = alignmentActive() && operation == Operation::NORTH;
    const double riseThreshold = northRunaway ? NORTH_RUNAWAY_ERROR_RISE_DEG : RUNAWAY_ERROR_RISE_DEG;
    const uint8_t requiredSamples = northRunaway ? NORTH_RUNAWAY_CONFIRM_SAMPLES : RUNAWAY_CONFIRM_SAMPLES;
    const uint32_t requiredMs = northRunaway ? NORTH_RUNAWAY_CONFIRM_MS : RUNAWAY_CONFIRM_MS;
    const bool continuingRise = axis.previousErrorValid && magnitude > axis.previousErrorMagnitude + 0.05;
    const bool canObserveDivergence = axis.motion != Motion::BRAKING && !axis.observing;
    if (canObserveDivergence && magnitude > axis.bestError + riseThreshold && continuingRise) {
        if (!axis.divergenceSamples) axis.divergenceSince = now;
        ++axis.divergenceSamples;
        if (axis.divergenceSamples >= requiredSamples && now - axis.divergenceSince >= requiredMs) {
            abortTest(axis.pitch ? "Pitch wrong-direction/runaway guard" : "Yaw wrong-direction/runaway guard"); return false;
        }
    } else {
        axis.divergenceSamples = 0; axis.divergenceSince = 0;
    }
    axis.previousErrorMagnitude = magnitude; axis.previousErrorValid = true;
    // Compare cumulative response to the fixed run start, not bestError, which
    // advances every sample and used to leave gradual yaw motion in trials forever.
    if (!axis.pitch && axis.bursts && !axis.confirmed && magnitude <= axis.initialError - DIRECTION_RESPONSE_DEG) {
        axis.confirmed = true;
        queueText("YAW DIRECTION CONFIRMED: cumulative measured heading moved toward north\n");
    }
    axis.bestError = fmin(axis.bestError, magnitude);
    const uint32_t feedbackAt = celestialActive() && celestialUsingEncoderFeedback() ? millis() : lastBnoGood;
    if (magnitude <= axis.progressError - 0.15) { axis.progressError = magnitude; axis.lastProgress = feedbackAt; }
    if (magnitude <= TOLERANCE_DEG) axis.lastProgress = feedbackAt;
    if (alignmentActive()) updateAlignmentTiming(axis);
    if (celestialActive()) updateCelestialTiming(axis);
    const uint32_t timeout = celestialActive() ? celestialProgressTimeout(axis) :
        (alignmentActive() ? alignmentProgressTimeout(axis) :
        (axis.motion == Motion::SLEW ? SLEW_PROGRESS_TIMEOUT_MS : PROGRESS_TIMEOUT_MS));
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
    if (finalPrinted || poseStopping || alignmentPaused || !celestialControlFeedbackReady() ||
        motionWatchdog.tripped() || axis.motor->isRunning()) return;
    if (!axis.pitch && !yawRequired) return;
    if (!posePrecisionOnly && axis.confirmed && !axis.slewFinished &&
        fabs(axis.error) > axis.brakeAtDeg + SLEW_ENTRY_HYSTERESIS_DEG) {
        axis.slewDirection = bnoSlewStepDirection(axis.error, axis.pitch);
        const int32_t acceleration = alignmentActive() && axis.pitch ? PITCH_TRAVEL_ACCELERATION : slewAcceleration(axis.pitch);
        const uint32_t speed = alignmentActive() && axis.pitch ? PITCH_TRAVEL_SPEED_HZ : slewSpeed(axis.pitch);
        int result = static_cast<int>(axis.motor->setAcceleration(acceleration));
        if (result == 0) result = static_cast<int>(axis.motor->setSpeedInHz(limitedSpeed(axis, speed)));
        const char *call = axis.slewDirection > 0 ? "runForward" : "runBackward";
        if (result == 0) result = static_cast<int>(axis.slewDirection > 0 ?
            axis.motor->runForward() : axis.motor->runBackward());
        if (axis.pitch) traceLevelPitchDirection("COMMAND", axis, "SLEW", axis.error > 0 ? 1 : -1,
            axis.slewDirection, call, result);
        if (result != static_cast<int>(MOVE_OK)) {
            abortTest(axis.pitch ? "FastAccelStepper pitch slew rejected" : "FastAccelStepper yaw slew rejected"); return;
        }
        if (axis.pitch) armLevelPitchDirectionCheck(axis.error > 0 ? 1 : -1, axis.slewDirection);
        ++axis.slewStarts; axis.commandAt = millis(); axis.lastProgress = millis();
        setMotion(axis, Motion::SLEW);
    } else {
        int32_t steps = bnoPrecisionCorrectionSteps(axis.error, axis.confirmed, axis.pitch);
        if (posePrecisionOnly) steps = std::max(-FIRST_POSE_BURST_STEPS, std::min(FIRST_POSE_BURST_STEPS, steps));
        if (!steps) return;
        int result = static_cast<int>(axis.motor->setAcceleration(ACCELERATION));
        if (result == 0) result = static_cast<int>(axis.motor->setSpeedInHz(
            limitedSpeed(axis, correctionSpeed(axis.error, axis.confirmed))));
        if (result == 0) result = static_cast<int>(axis.motor->move(steps));
        if (axis.pitch) traceLevelPitchDirection("COMMAND", axis, "PRECISION", axis.error > 0 ? 1 : -1,
            steps, "move", result);
        if (result != static_cast<int>(MOVE_OK)) {
            abortTest(axis.pitch ? "FastAccelStepper pitch correction rejected" : "FastAccelStepper yaw correction rejected"); return;
        }
        if (axis.pitch) armLevelPitchDirectionCheck(axis.error > 0 ? 1 : -1, steps > 0 ? 1 : -1);
        ++axis.bursts; axis.finiteActive = true; axis.commandAt = millis();
    }
    // If the independent watchdog raced this command, stop again immediately.
    safety();
}
void serviceAxis(Axis &axis, uint32_t now, uint32_t sampleAt) {
    updateVelocity(axis, sampleAt);
    if (axis.motion == Motion::SLEW) {
        if (!axis.motor->isRunning()) {
            if (now - axis.commandAt < PRECISION_OBSERVE_MS) return;
            abortTest("Continuous slew stopped unexpectedly"); return;
        }
        if (fabs(axis.error) <= axis.brakeAtDeg || bnoSlewStepDirection(axis.error, axis.pitch) != axis.slewDirection) {
            // stopMove keeps the slew acceleration already applied by run*().
            // Do not change rates or issue a reversal until isRunning is false.
            axis.motor->stopMove(); axis.commandAt = now; axis.slewFinished = true;
            if (axis.pitch) traceLevelPitchDirection("BRAKE", axis, "BRAKE", axis.error > 0 ? 1 : -1,
                axis.slewDirection, "stopMove", static_cast<int>(MOVE_OK));
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
        if (axis.motion == Motion::BRAKING) {
            axis.bestError = axis.progressError = fabs(axis.error); axis.divergenceSamples = 0; axis.divergenceSince = 0;
            axis.previousErrorMagnitude = fabs(axis.error); axis.previousErrorValid = true; axis.lastProgress = sampleAt;
            setMotion(axis, Motion::PRECISION);
            if (axis.pitch && alignmentActive() && operation == Operation::LEVEL) {
                alignmentTimingAngle = axis.current;
                alignmentTimingSteps = axis.motor->getCurrentPosition();
            }
        }
        return;
    }
    // Queue delivery time cannot establish a post-stop observation. Require a
    // sample RECEIVED after the actual stopped observation plus mechanical wait.
    if (axis.observing) {
        if (!sampleAtOrAfter(sampleAt, axis.stoppedAt + PRECISION_OBSERVE_MS)) return;
        axis.observing = false;
    }
    if (axis.motion == Motion::HOLD) {
        if (fabs(axis.error) <= TOLERANCE_DEG) return;
        axis.settled = false; setMotion(axis, Motion::PRECISION);
    }
    if (fabs(axis.error) <= (alignmentActive() ? TOLERANCE_DEG : APPROACH_DEADBAND_DEG)) {
        setMotion(axis, Motion::HOLD); return;
    }
    commandAxis(axis);
}
void serviceAxes() {
    serviceLevelPitchDirectionCheck();
    if (poseStopping || alignmentPaused) return;
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
    const uint32_t now = millis();
    const bool celestialEncoderFeedback = celestialActive() && celestialUsingEncoderFeedback();
    if (!celestialEncoderFeedback && !fresh(now)) return;
    if (carriagePending || yawMotor->isRunning() || pitchMotor->isRunning() || carriageMotor->isRunning()) {
        poseStoppedObserved = false; settleSamples = 0;
    } else if (!poseStoppedObserved) {
        poseStoppedObserved = true; poseStoppedAt = now;
    }
    uint32_t sampleAt = lastBnoGood;
    if (celestialEncoderFeedback) {
        bool newSample = false;
        if (!serviceCelestialEncoderFeedback(now, sampleAt, newSample) || !newSample) return;
    } else {
        // Cached values are useful for telemetry, but never constitute a new
        // control decision, direction confirmation, velocity or settling sample.
        if (lastControlSample == bnoHealth.freshSamples) return;
        lastControlSample = bnoHealth.freshSamples;
        yawAxis.current = heading.continuous; yawAxis.error = yawAxis.target - yawAxis.current;
        pitchAxis.current = physicalPitch(); pitchAxis.error = pitchAxis.target - pitchAxis.current;
    }
    if (!sampleAtOrAfter(sampleAt, controlStartedAt)) return;
    if (celestialActive() && celestialTracking) {
        serviceCelestialTracking(now, sampleAt);
        return;
    }
    // Absolute POSE always checks yaw; MOVE 0 dp ds explicitly leaves yaw uncontrolled.
    if ((yawRequired && !axisProgress(yawAxis, yawAxis.error)) ||
        (pitchControlRequired() && !axisProgress(pitchAxis, pitchAxis.error))) return;
    if (yawRequired) serviceAxis(yawAxis, now, sampleAt);
    if (finalPrinted) return;
    if (pitchControlRequired()) serviceAxis(pitchAxis, now, sampleAt);
    if (finalPrinted || poseStopping || alignmentPaused) return;
    // One persistent angular controller owns TRACK. Updating its targets never
    // creates a new POSE, restarts a slew, or re-enters a terminal settling loop.
    if (celestialActive() && celestialTracking) return;
    if (carriagePending) {
        safety(); if (finalPrinted) return;
        if (carriageMotor->setAcceleration(CARRIAGE_ACCELERATION) != 0 || carriageMotor->setSpeedInHz(carriageRateHz) != 0 ||
            carriageMotor->moveTo(carriageTarget) != MOVE_OK) {
            abortTest("FastAccelStepper carriage pose command rejected"); return;
        }
        carriagePending = false;
        safety(); if (finalPrinted) return;
    }
    const bool carriageStoppedAtTarget = !carriageMotor->isRunning() && carriageMotor->getCurrentPosition() == carriageTarget;
    const bool stoppedInTolerance = yawAxis.motion == Motion::HOLD && pitchAxis.motion == Motion::HOLD &&
        poseStoppedObserved && sampleAtOrAfter(sampleAt, poseStoppedAt + PRECISION_OBSERVE_MS) &&
        !yawAxis.observing && !pitchAxis.observing &&
        !yawMotor->isRunning() && !pitchMotor->isRunning() &&
        carriageStoppedAtTarget &&
        (!yawRequired || fabs(yawAxis.error) <= TOLERANCE_DEG) &&
        (!pitchControlRequired() || fabs(pitchAxis.error) <= TOLERANCE_DEG) &&
        (!accuracyRequired || bnoAccuracy >= BNO_MIN_ACCURACY);
    if (!stoppedInTolerance) { phase = Phase::MOVING; settleSamples = 0; return; }
    if (!settleSamples || sampleAt - settleLastSample > WINDOW_GAP_MS) {
        settleStarted = sampleAt; settleSamples = 0;
    }
    phase = Phase::SETTLING; ++settleSamples; settleLastSample = sampleAt;
    if (sampleAt - settleStarted >= SETTLE_MS && settleSamples >= 30) {
        yawAxis.settled = ((alignmentActive() || celestialActive()) ? yawRequired : referenceSet) && fabs(yawAxis.error) <= TOLERANCE_DEG;
        pitchAxis.settled = pitchControlRequired();
        if (celestialActive()) enterCelestialTracking();
        else if (alignmentActive()) finish(true, operation == Operation::LEVEL ? "LEVEL: physical pitch zero settled" : "NORTH: magnetic heading zero settled");
        else if (poseActive) finish(true, !yawRequired ? "Pitch target stopped and settled; yaw control disabled" :
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
    axis.target = target; axis.current = pitch ? physicalPitch() : heading.continuous;
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
        posePrecisionOnly = false; poseStoppedObserved = false;
        operation = Operation::POSE; poseActive = true; finalPrinted = false; phase = Phase::MOVING;
        carriageTarget = static_cast<int32_t>(requestedCarriage); carriagePending = true;
        carriageRateHz = CARRIAGE_MAX_SPEED_HZ; plannedDurationSeconds = duration;
        controlStartedAt = millis(); window.active = false;
        queueText("POSE ACCEPTED: relative carriage steps; angular axes stationary\n"); return;
    }
    if (!fresh(millis()) || !bnoWatchdogReady || !motionWatchdog.clearTripWhenStopped()) {
        rejectPose("fresh plausible BNO orientation and feedback watchdog required"); return;
    }
    if (relative && (yawValue < -180.0 || yawValue >= 180.0)) {
        rejectPose("MOVE yaw delta must be in [-180,180) degrees; no implicit full-turn motion"); return;
    }
    const double requestedYaw = relative ? static_cast<double>(orientation.heading) + yawValue : yawValue;
    const double requestedPitch = relative ? physicalPitch() + pitchValue : pitchValue;
    if (!isfinite(requestedYaw) || !isfinite(requestedPitch) || fabs(requestedPitch) >= MAX_USABLE_PITCH_DEG) {
        rejectPose("finite yaw/pitch required; pitch target must be inside +/-75 degrees"); return;
    }
    const double targetYaw = heading.continuous + shortestDifference(requestedYaw, orientation.heading);
    if (referenceSet && fabs(targetYaw - northTargetContinuous) >= RELATIVE_LIMIT_DEG) {
        rejectPose("shortest yaw path endpoint exceeds existing +/-185-degree continuous north guard"); return;
    }
    const double yawError = targetYaw - heading.continuous, pitchError = requestedPitch - physicalPitch();
    const bool moveYaw = fabs(yawError) > TOLERANCE_DEG, movePitch = fabs(pitchError) > TOLERANCE_DEG;
    const double carriageDistance = fabs(static_cast<double>(requestedCarriage) - carriageMotor->getCurrentPosition());
    const bool pitchOnly = relative && yawValue == 0;
    if (!referenceSet && !pitchReady) pitchReadyYaw = heading.continuous;
    if (!northUsable) queueText("POSE NOTE: using live reported orientation; magnetic north not qualified\n");
    if (!moveYaw && !movePitch && carriageDistance == 0) {
        queueText("POSE ALREADY AT TARGET: no motion; carriage units=STEPS from startup zero\n"); return;
    }
    // An absolute yaw target remains required even when initially satisfied.
    // Missing timing on that holding axis must not permit an unplanned slew.
    const bool missingTiming = pitchPulsesPerDegree <= 0 || (!pitchOnly && yawPulsesPerDegree <= 0);
    if (missingTiming && (fabs(pitchError) > FIRST_POSE_MAX_ERROR_DEG ||
        (!pitchOnly && fabs(yawError) > FIRST_POSE_MAX_ERROR_DEG))) {
        rejectPose("first precision POSE requires angular changes within +/-3 degrees until timing is learned"); return;
    }
    if (pitchOnly || missingTiming) {
        const uint32_t carriageCap = missingTiming ? FIRST_POSE_SPEED_HZ : CARRIAGE_MAX_SPEED_HZ;
        if (milestone8::finiteSeconds(carriageDistance, carriageCap, CARRIAGE_ACCELERATION) * 1000 >= LEG_TIMEOUT_MS - SETTLE_MS) {
            rejectPose("estimated carriage move cannot meet existing motion deadline"); return;
        }
        // Use the existing finite controller, bounded in pulse rate/burst size;
        // learn timing only after a stopped, successfully measured completion.
        resetPoseAxis(yawAxis, targetYaw, yawTimingPeak);
        resetPoseAxis(pitchAxis, requestedPitch, pitchTimingPeak);
        yawRequired = !pitchOnly; accuracyRequired = false;
        posePrecisionOnly = missingTiming; poseStoppedObserved = false;
        poseNeedsBno = true;
        carriageTarget = static_cast<int32_t>(requestedCarriage); carriagePending = carriageDistance > 0;
        carriageRateHz = carriageCap;
        yawRateCap = missingTiming ? FIRST_POSE_SPEED_HZ : YAW_SLEW_SPEED_HZ;
        pitchRateCap = missingTiming ? FIRST_POSE_SPEED_HZ : PITCH_SLEW_SPEED_HZ;
        plannedDurationSeconds = 0; timingLimited = true;
        operation = Operation::POSE; poseActive = true; finalPrinted = false; phase = Phase::MOVING;
        settleSamples = 0; window.active = false; accuracyGrace = {};
        sawYawMotion = sawPitchMotion = concurrentMotion = concurrentThreeMotion = false;
        controlStartedAt = millis(); lastControlSample = bnoHealth.freshSamples;
        recordTimingStarts();
        if (!motionWatchdog.arm(lastBnoGood)) { abortTest("BNO watchdog could not arm for precision motion"); return; }
        queueText(pitchOnly ? "PITCH-ONLY ACCEPTED: yaw stationary; carriage uses requested steps\n" :
            "POSE ACCEPTED: first precision move; reported yaw target remains required\n");
        if (missingTiming) queueText("POSE PRECISION: caps_hz=80/80/80 max_burst_steps=16; no slew; timing learned only on success\n");
        safety(); return;
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
    yawRateCap = moveYaw ? yawTiming.capHz : FIRST_POSE_SPEED_HZ;
    pitchRateCap = movePitch ? pitchTiming.capHz : FIRST_POSE_SPEED_HZ;
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
    yawRequired = true; accuracyRequired = false;
    posePrecisionOnly = false; poseStoppedObserved = false;
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
#include "keyframe_motion.h"
#include "celestial_motion.h"
void executeCommand() {
    char *tokens[8] = {}; unsigned count = 0; char *context = nullptr;
    for (char *token = strtok_r(commandLine, " \t", &context); token && count < 8; token = strtok_r(nullptr, " \t", &context)) tokens[count++] = token;
    if (!count) return;
    if (executeManualCommand(tokens, count)) return;
    if (executeKeyframeCommand(tokens, count)) return;
    if (executeOrientationCommand(tokens, count)) return;
    if (executeCelestialCommand(tokens, count)) return;
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
        "BNO_fresh=%s age_ms=%lu max_gap_ms=%lu accuracy=%u accuracy_low_ms=%lu north_usable=%s north_reason=%s\n",
        phaseText(), orientation.heading, yawAxis.error, physicalPitch(), pitchAxis.error,
        motionText(yawAxis), motionText(pitchAxis), yawRunning ? "MOVING" : "IDLE", pitchRunning ? "MOVING" : "IDLE",
        yawRunning && pitchRunning ? "YES" : "NO",
        yawMotor ? yawMotor->getCurrentSpeedInMilliHz() / 1000.0 : 0,
        pitchMotor ? pitchMotor->getCurrentSpeedInMilliHz() / 1000.0 : 0,
        yawAxis.angularSpeed, pitchAxis.angularSpeed, yawAxis.brakeAtDeg, pitchAxis.brakeAtDeg,
        fresh(now) ? "YES" : "NO", static_cast<unsigned long>(now - lastBnoGood),
        static_cast<unsigned long>(bnoMaxGap), bnoAccuracy, static_cast<unsigned long>(accuracyGrace.age(now)),
        northHeadingUsable(now) ? "YES" : "NO", northUnavailableReason(now));
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
    const char *northReason = northUnavailableReason(now);
    snprintf(line, sizeof(line), "BNO_STATE available=%s fresh=%s age_ms=%lu accuracy=%u north_usable=%s north_reason=%s heading=%.3f physical_pitch=%.3f pitch_axis=PITCH pitch_roll=%.3f has_sample=%s accepted_seq=%u accepted_sensor_us=%llu accepted_rx_ms=%lu\n",
        bnoInitialized && reportEnabled ? "YES" : "NO", fresh(now) ? "YES" : "NO",
        static_cast<unsigned long>(hasPlausibleOrientation ? now - lastPlausibleAt : UINT32_MAX), lastPlausibleAccuracy,
        strcmp(northReason, "usable") == 0 ? "YES" : "NO", northReason,
        lastPlausibleOrientation.heading, physicalPitch(lastPlausibleOrientation), lastPlausibleOrientation.roll, hasPlausibleOrientation ? "YES" : "NO", acceptedBnoSequence,
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
#ifndef M07_HOST_TEST
    keyframeEpoch = esp_random();
    if (!keyframeEpoch) keyframeEpoch = 1;
#endif
    for (const uint8_t pin : {tmp_hardware::YAW_STEP_PIN, tmp_hardware::PITCH_STEP_PIN, tmp_hardware::CARRIAGE_STEP_PIN}) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
    Serial.println("M09_xbox_control: manual velocity; optional AS5600/BNO telemetry; no automatic startup movement");
    Serial.println("BNO_DIAGNOSTICS revision=sensor-worker-1 comparison=POST_RELOCATION_USER_REPORTED timestamp=header_receipt_us");
    Serial.println("Yaw DIR32/STEP33; pitch DIR26/STEP12; carriage DIR21/STEP22. Physical pitch feedback = BNO PITCH.");
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
    servicePoseStop();
    serviceKeyframe();
    safety();
    if (manualActive) serviceManual(); // STOP reaches motor braking before sensor calls.
    serviceBno();
    serviceCommands(); safety();
    servicePoseStop();
    serviceKeyframe();
    if (manualActive) serviceManual();
    serviceCommands(); safety();
    servicePoseStop();
    serviceKeyframe();
    if (manualActive) serviceManual();
    if (poseActive) serviceAxes();
    serviceReference();
    publishSensorContext();
    serviceTraceOutput();
    const uint32_t now = millis();
    if (now - lastSensorDisplay >= 500) { lastSensorDisplay = now; sensorTelemetry(); celestialTelemetry(); }
    if (!manualActive && now - lastDisplay >= 750) { lastDisplay = now; telemetry(); }
    serviceSerialOutput();
#ifdef M07_HOST_TEST
    sensorWorker.testDispatch();
#endif
    delay(1);
}
