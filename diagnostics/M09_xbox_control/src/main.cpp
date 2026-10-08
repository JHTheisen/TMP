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
#include "control_math.h"
#include "motion_watchdog.h"
#include "pose_math.h"
#include "keyframe_math.h"
#include "encoder_acquisition.h"
#include "sensor_worker.h"

namespace {
using namespace milestone7;
constexpr uint32_t ENCODER_STALE_MS = m09::ENCODER_STALE_MS, WINDOW_GAP_MS = 100, SETTLE_MS = 1000;
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
enum class Operation { POSE, MANUAL, LEVEL, NORTH, CELESTIAL };

enum class Phase { MOVING, SETTLING, COMPLETE, ABORTED, MANUAL };
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
FastAccelStepperEngine engine;
FastAccelStepper *yawMotor = nullptr, *pitchMotor = nullptr;
FastAccelStepper *carriageMotor = nullptr;
m09::SensorWorker sensorWorker;
m09::EncoderSnapshot encoders;
m09::SensorSnapshot sensorSnapshot;
m09::EncoderReference encoderReferences[2];
uint32_t feedbackGeneration = 0;
uint32_t encoderReads[2] = {};
struct AxisOrientation { double heading = NAN, pitch = NAN; } orientation;
uint32_t telemetryDrops = 0;
uint32_t lastSensorDisplay = 0;
MotionWatchdog motionWatchdog;
// Independent leases for feedback and manual host commands.
MotionWatchdog commandWatchdog;
constexpr uint32_t MANUAL_COMMAND_TIMEOUT_MS = 250;
bool manualActive = false, manualEnding = false;
bool controlReady = false, feedbackWatchdogReady = false, poseNeedsFeedback = true;
struct ManualAxis {
    int request = 0, direction = 0;
    uint32_t rate = 0, brakeAt = 0, stoppedAt = 0, startedAt = 0, progressAt = 0;
    bool braking = false, observing = false, runningObserved = false;
    double start = 0, best = 0, progress = 0;
};
ManualAxis manualYaw, manualPitch, manualCarriage;
double physicalPitch() { return orientation.pitch; }
HeadingTracker heading;
Axis yawAxis, pitchAxis;
Phase phase = Phase::ABORTED;
bool finalPrinted = false, referenceSet = false;
bool northUsable = false, concurrentMotion = false, sawYawMotion = false, sawPitchMotion = false;
// References are explicitly declared by the operator, never inferred while idle.
bool pitchReady = false, yawRequired = true;
constexpr double northTargetContinuous = 0;
uint32_t controlStartedAt = 0, lastFeedbackAt = 0, lastDisplay = 0;
uint32_t lastControlSample = 0, settleStarted = 0, settleSamples = 0, settleLastSample = 0;
char txBuffer[4096];
size_t txLength = 0, txOffset = 0;
Operation operation = Operation::MANUAL;
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
bool celestialControlFeedbackReady();
void serviceCelestialTracking(uint32_t now, uint32_t sampleAt);

bool alignmentActive() { return poseActive && (operation == Operation::LEVEL || operation == Operation::NORTH); }
bool pitchControlRequired() { return !alignmentActive() || operation == Operation::LEVEL; }
const char *alignmentName() { return operation == Operation::LEVEL ? "LEVEL" : "NORTH"; }
// Powered AS5600 log 20261008T142253: negative FAS steps decreased raw
// counts on BOTH axes. Positive steps therefore increase each raw coordinate.
// Compose that hardware relationship with the signed encoder scale; do not
// reuse the manual stick mapping, which deliberately remains unchanged.
constexpr int ENCODER_RAW_POSITIVE_STEP_SIGN[2] = {1, 1};
int axisSlewStepSign(bool pitch) {
    const unsigned index = pitch ? 1 : 0;
    return ENCODER_RAW_POSITIVE_STEP_SIGN[index] *
        (encoderReferences[index].degreesPerRevolution < 0 ? -1 : 1);
}
int axisSlewStepDirection(double error, bool pitch) {
    return (error > 0 ? 1 : -1) * axisSlewStepSign(pitch);
}
int32_t axisPrecisionCorrectionSteps(double error, bool confirmed, bool pitch) {
    // Keep the existing burst magnitude/caps; all measured-position control
    // (finite, slew, TRACK and response guards) must share one feedback sign.
    const int32_t magnitude = std::abs(correctionSteps(error, confirmed, pitch));
    return magnitude * axisSlewStepDirection(error, pitch);
}
int axisFeedbackStepSign(const Axis &axis) {
    return axisSlewStepSign(axis.pitch);
}
void abortAlignment(const char *reason);
bool alignmentSafety();
void updateAlignmentTiming(Axis &axis);
uint32_t alignmentProgressTimeout(const Axis &axis);
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
    // Completed, stopped encoder motion supplies a timing estimate only. These
    // counts never replace encoder angle feedback or claim a calibrated gearbox.
    if (fabs(yawDelta) >= DIRECTION_RESPONSE_DEG && fabs(yawSteps) >= 8 &&
        yawSteps * yawDelta * axisFeedbackStepSign(yawAxis) > 0) {
        yawPulsesPerDegree = fabs(yawSteps / yawDelta); yawTimingPeak = yawAxis.peakAngularSpeed;
    }
    if (operation == Operation::LEVEL && alignmentPulsesPerDegree > 0) {
        pitchPulsesPerDegree = alignmentPulsesPerDegree;
        pitchTimingPeak = pitchAxis.peakAngularSpeed;
    } else if (fabs(pitchDelta) >= DIRECTION_RESPONSE_DEG && fabs(pitchSteps) >= 8 &&
        pitchSteps * pitchDelta * axisFeedbackStepSign(pitchAxis) > 0) {
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
    // the motor can coast while braking, then encoder/filter noise settles after
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

bool fresh(uint32_t now) {
    return encoderReferences[0].ready(sensorSnapshot.positions[0], now) &&
           encoderReferences[1].ready(sensorSnapshot.positions[1], now);
}
const char *northUnavailableReason(uint32_t now) {
    if (!encoderReferences[0].degreesPerRevolution) return "yaw_encoder_scale_unconfigured";
    if (!sensorSnapshot.positions[0].fresh(now)) return "yaw_encoder_unavailable";
    if (!encoderReferences[0].ready(sensorSnapshot.positions[0], now)) return "set_north_required";
    return "usable";
}
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
    if (passed && poseActive && poseNeedsFeedback) learnTimingResponse();
    stopMotors(); motionWatchdog.disarm(); finalPrinted = true;
    commandWatchdog.disarm(); manualActive = manualEnding = false;
    manualYaw.request = manualPitch.request = manualCarriage.request = 0;
    phase = (passed || recoverable) ? Phase::COMPLETE : Phase::ABORTED;
    // Clearing a trip is permitted only after the independent stop has ended.
    if (phase == Phase::COMPLETE) motionWatchdog.clearTripWhenStopped();
    // READY continues encoder acquisition: queue the summary instead of waiting on
    // UART space. Discard optional pending snapshots, including any partial line.
    txOffset = txLength = 0;
    char summary[500];
    snprintf(summary, sizeof(summary),
        "ORIENTATION_RESULT feedback=AS5600 yaw=%.3f pitch=%.3f yaw_error=%.3f pitch_error=%.3f\n"
        "Reason: %s\nFINAL RESULT: %s\n%s\n",
        orientation.heading, physicalPitch(), yawAxis.error, pitchAxis.error, reason,
        passed ? "PASS" : (recoverable ? "STOPPED" : "FAIL"),
        (passed || recoverable) ? "M09 READY" : "Latched abort; reset required.");
    poseActive = poseStopping = false; carriagePending = false;
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
void safety() {
    if (keyframeActive) { keyframeSafety(); return; }
    if (manualActive) commandWatchdog.check(millis());
    if (manualActive && commandWatchdog.tripped()) {
        stopManualSession("command stream lost for 250 ms; rearm centered"); return;
    }
    if (manualActive || finalPrinted || !poseActive || poseStopping) return;
    if (motionWatchdog.tripped()) invalidateKeyframes("feedback watchdog forced stop; recapture required");
    if (celestialActive()) { celestialSafety(); return; }
    // LEVEL/NORTH have selected-axis guards and recoverable feedback handling.
    // Do not apply combined-POSE yaw/pitch guards to their inactive axes.
    if (alignmentActive()) { alignmentSafety(); return; }
    if (!alignmentActive() && millis() - controlStartedAt >= LEG_TIMEOUT_MS) { abortTest("Pose control timeout"); return; }
    if (!poseNeedsFeedback) return;
    if (!fresh(millis()) || motionWatchdog.tripped()) {
        abortTest("encoder feedback/reference unavailable; recalibrate before retrying"); return;
    }
    if (fabs(physicalPitch()) >= MAX_USABLE_PITCH_DEG)
        abortTest("Measured pitch reached +/-75 degree absolute guard");
}
void serviceEncoders() {
    sensorWorker.status.read(sensorSnapshot);
    encoders = sensorSnapshot.encoders;
    const uint32_t now = millis();
    bool changed = false;
    for (unsigned n = 0; n < 2; ++n) {
        const bool wasCalibrated = encoderReferences[n].calibrated;
        encoderReferences[n].observe(sensorSnapshot.positions[n], now);
        if (wasCalibrated && !encoderReferences[n].calibrated)
            queueText(n ? "CALIBRATION LOST axis=PITCH; SET_LEVEL required\n" :
                          "CALIBRATION LOST axis=YAW; SET_NORTH required\n");
        if (encoderReads[n] != sensorSnapshot.positions[n].reads) changed = true;
        encoderReads[n] = sensorSnapshot.positions[n].reads;
    }
    const double yaw = encoderReferences[0].angle(sensorSnapshot.positions[0], now);
    orientation.heading = wrap360(yaw);
    orientation.pitch = encoderReferences[1].angle(sensorSnapshot.positions[1], now);
    heading.initialized = isfinite(yaw); heading.first = 0; heading.continuous = yaw;
    heading.previous = orientation.heading;
    referenceSet = encoderReferences[0].ready(sensorSnapshot.positions[0], now);
    pitchReady = encoderReferences[1].ready(sensorSnapshot.positions[1], now);
    northUsable = referenceSet;
    if (!fresh(now)) return;
    lastFeedbackAt = now - std::max(encoders.state(0).ageMs(now), encoders.state(1).ageMs(now));
    if (changed) ++feedbackGeneration;
    if (poseActive && poseNeedsFeedback && !poseStopping) motionWatchdog.recordFresh(lastFeedbackAt);
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
    const uint32_t feedbackAt = lastFeedbackAt;
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
        // a late or temporarily flattened encoder velocity must not shrink the margin.
        axis.peakAngularSpeed = fmax(axis.peakAngularSpeed, fabs(axis.angularSpeed));
        axis.velocityPosition = axis.current; axis.velocityAt = now;
    }
    axis.brakeAtDeg = brakingThreshold(axis.pitch, axis.peakAngularSpeed);
}
void commandAxis(Axis &axis) {
    safety();
    if (finalPrinted || poseStopping || !celestialControlFeedbackReady() ||
        motionWatchdog.tripped() || axis.motor->isRunning()) return;
    if (!axis.pitch && !yawRequired) return;
    if (!posePrecisionOnly && axis.confirmed && !axis.slewFinished &&
        fabs(axis.error) > axis.brakeAtDeg + SLEW_ENTRY_HYSTERESIS_DEG) {
        axis.slewDirection = axisSlewStepDirection(axis.error, axis.pitch);
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
        int32_t steps = axisPrecisionCorrectionSteps(axis.error, axis.confirmed, axis.pitch);
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
        if (fabs(axis.error) <= axis.brakeAtDeg || axisSlewStepDirection(axis.error, axis.pitch) != axis.slewDirection) {
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
    if (poseStopping) return;
    if (!poseNeedsFeedback) {
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
    const uint32_t sampleAt = lastFeedbackAt;
    if (carriagePending || !poseMotorsStopped()) {
        poseStoppedObserved = false; settleSamples = 0;
    } else if (!poseStoppedObserved) {
        poseStoppedObserved = true; poseStoppedAt = now;
    }
    if (!fresh(now) || lastControlSample == feedbackGeneration) return;
    lastControlSample = feedbackGeneration;
    yawAxis.current = heading.continuous; yawAxis.error = yawAxis.target - yawAxis.current;
    pitchAxis.current = physicalPitch(); pitchAxis.error = pitchAxis.target - pitchAxis.current;
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
    if (finalPrinted || poseStopping) return;
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
        (!pitchControlRequired() || fabs(pitchAxis.error) <= TOLERANCE_DEG);
    if (!stoppedInTolerance) { phase = Phase::MOVING; settleSamples = 0; return; }
    if (!settleSamples || sampleAt - settleLastSample > WINDOW_GAP_MS) {
        settleStarted = sampleAt; settleSamples = 0;
    }
    phase = Phase::SETTLING; ++settleSamples; settleLastSample = sampleAt;
    if (sampleAt - settleStarted >= SETTLE_MS && settleSamples >= 30) {
        yawAxis.settled = ((alignmentActive() || celestialActive()) ? yawRequired : referenceSet) && fabs(yawAxis.error) <= TOLERANCE_DEG;
        pitchAxis.settled = pitchControlRequired();
        if (celestialActive()) enterCelestialTracking();
        else if (alignmentActive()) finish(true, operation == Operation::LEVEL ? "LEVEL: physical pitch zero settled" : "NORTH: calibrated north zero settled");
        else if (poseActive) finish(true, !yawRequired ? "Pitch target stopped and settled; yaw control disabled" :
            "POSE complete: encoder angle targets settled; carriage generated-step target reached");
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
        poseNeedsFeedback = false; yawRequired = false;
        posePrecisionOnly = false; poseStoppedObserved = false;
        operation = Operation::POSE; poseActive = true; finalPrinted = false; phase = Phase::MOVING;
        carriageTarget = static_cast<int32_t>(requestedCarriage); carriagePending = true;
        carriageRateHz = CARRIAGE_MAX_SPEED_HZ; plannedDurationSeconds = duration;
        controlStartedAt = millis();
        queueText("POSE ACCEPTED: relative carriage steps; angular axes stationary\n"); return;
    }
    if (!fresh(millis()) || !feedbackWatchdogReady || !motionWatchdog.clearTripWhenStopped()) {
        rejectPose("fresh plausible encoder orientation and feedback watchdog required"); return;
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
        yawRequired = !pitchOnly;
        posePrecisionOnly = missingTiming; poseStoppedObserved = false;
        poseNeedsFeedback = true;
        carriageTarget = static_cast<int32_t>(requestedCarriage); carriagePending = carriageDistance > 0;
        carriageRateHz = carriageCap;
        yawRateCap = missingTiming ? FIRST_POSE_SPEED_HZ : YAW_SLEW_SPEED_HZ;
        pitchRateCap = missingTiming ? FIRST_POSE_SPEED_HZ : PITCH_SLEW_SPEED_HZ;
        plannedDurationSeconds = 0; timingLimited = true;
        operation = Operation::POSE; poseActive = true; finalPrinted = false; phase = Phase::MOVING;
        settleSamples = 0;
        sawYawMotion = sawPitchMotion = concurrentMotion = concurrentThreeMotion = false;
        controlStartedAt = millis(); lastControlSample = feedbackGeneration;
        recordTimingStarts();
        if (!motionWatchdog.arm(lastFeedbackAt)) { abortTest("encoder watchdog could not arm for precision motion"); return; }
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
    yawRequired = true;
    posePrecisionOnly = false; poseStoppedObserved = false;
    poseNeedsFeedback = true;
    carriageTarget = static_cast<int32_t>(requestedCarriage); carriagePending = carriageDistance > 0;
    operation = Operation::POSE; poseActive = true; finalPrinted = false; phase = Phase::MOVING;
    settleSamples = 0;
    sawYawMotion = sawPitchMotion = concurrentMotion = false;
    concurrentThreeMotion = false;
    controlStartedAt = millis(); lastControlSample = feedbackGeneration;
    recordTimingStarts();
    if (!motionWatchdog.arm(lastFeedbackAt)) { abortTest("encoder watchdog could not arm for POSE"); return; }
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
#include "encoder_commands.h"
#include "manual_control.h"
#include "keyframe_motion.h"
#include "celestial_motion.h"
void executeCommand() {
    char *tokens[8] = {}; unsigned count = 0; char *context = nullptr;
    for (char *token = strtok_r(commandLine, " \t", &context); token && count < 8; token = strtok_r(nullptr, " \t", &context)) tokens[count++] = token;
    if (!count) return;
    if (executeCalibrationCommand(tokens, count)) return;
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
void sensorTelemetry() {
    const uint32_t now = millis();
    char line[650];
    snprintf(line, sizeof(line),
        "ORIENTATION_STATE protocol=2 feedback=AS5600 available=%s fresh=%s age_ms=%lu has_sample=%s "
        "heading=%.5f yaw_continuous=%.5f physical_pitch=%.5f pitch_axis=PITCH roll=UNAVAILABLE "
        "north_set=%s level_set=%s yaw_scale=%.9g pitch_scale=%.9g\n",
        sensorSnapshot.positions[0].fresh(now) && sensorSnapshot.positions[1].fresh(now) ? "YES" : "NO",
        fresh(now) ? "YES" : "NO",
        static_cast<unsigned long>(std::max(encoders.state(0).ageMs(now), encoders.state(1).ageMs(now))),
        encoders.state(0).hasSample && encoders.state(1).hasSample ? "YES" : "NO",
        orientation.heading, heading.continuous, physicalPitch(),
        referenceSet ? "YES" : "NO", pitchReady ? "YES" : "NO",
        encoderReferences[0].degreesPerRevolution, encoderReferences[1].degreesPerRevolution);
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

#ifdef M07_HOST_TEST
void loop();
void sensorWorkerTestWait(uint32_t duration) {
    const uint32_t start = millis();
    while (millis()-start < duration) loop();
}
#endif

void setup() {
    Serial.begin(115200);
    Serial.println("M09_xbox_control: dual AS5600 protocol=2; configure encoder geometry, SET_NORTH and SET_LEVEL; no startup motion");
#ifndef M07_HOST_TEST
    keyframeEpoch = esp_random();
    if (!keyframeEpoch) keyframeEpoch = 1;
#endif
    for (const uint8_t pin : {tmp_hardware::YAW_STEP_PIN, tmp_hardware::PITCH_STEP_PIN, tmp_hardware::CARRIAGE_STEP_PIN}) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
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
    feedbackWatchdogReady = motionWatchdog.begin(yawMotor, pitchMotor, ENCODER_STALE_MS, carriageMotor);
    if (!feedbackWatchdogReady) queueText("encoder WARNING: feedback watchdog unavailable; angular POSE disabled\n");
    controlReady = true;
    yawRequired = false;
    phase = Phase::COMPLETE; finalPrinted = true;

    if (!sensorWorker.start()) queueText("SENSOR WARNING: worker unavailable; manual control remains available\n");
#ifdef M07_HOST_TEST
    simulated::sensorWait = sensorWorkerTestWait;
#endif
    queueText("M09 READY\n");
}
void loop() {
    serviceCommands();
    if (phase == Phase::ABORTED) {  serviceEncoders();  serviceSerialOutput(); delay(1); return; }
    servicePoseStop();
    serviceKeyframe();
    safety();
    if (manualActive) serviceManual(); // STOP reaches motor braking before sensor calls.
    serviceEncoders();
    serviceCommands(); safety();
    servicePoseStop();
    serviceKeyframe();
    if (manualActive) serviceManual();
    serviceCommands(); safety();
    servicePoseStop();
    serviceKeyframe();
    if (manualActive) serviceManual();
    if (poseActive) serviceAxes();


    const uint32_t now = millis();
    if (now - lastSensorDisplay >= 500) { lastSensorDisplay = now; sensorTelemetry(); celestialTelemetry(); }
    serviceSerialOutput();
#ifdef M07_HOST_TEST
    sensorWorker.testDispatch();
#endif
    delay(1);
}
