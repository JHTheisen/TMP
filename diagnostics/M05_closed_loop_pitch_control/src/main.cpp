#include <Arduino.h>
#include <algorithm>
#include <AS5600.h>
#include <FastAccelStepper.h>
#include <Wire.h>
#include "hardware_config.h"
#include "sensor_support.h"
#include "control_math.h"

namespace {
using milestone4::DiagnosticBno085;
using milestone4::EulerAngles;
using milestone4::BnoHealth;
using milestone4::quaternionToEuler;
using namespace milestone5;
constexpr uint32_t STARTUP_MS = 5000;
constexpr uint32_t BASELINE_TIMEOUT_MS = 15000;
constexpr uint32_t ENCODER_INTERVAL_MS = 10;
constexpr uint32_t ENCODER_STALE_MS = 100;
constexpr uint32_t BNO_STALE_MS = 150;
constexpr uint32_t WINDOW_GAP_MS = 100;
constexpr uint32_t OBSERVE_DELAY_MS = 300;
constexpr uint32_t OBSERVE_WINDOW_MS = 250;
constexpr uint32_t SETTLE_WINDOW_MS = 1000;
constexpr uint32_t MOVE_TIMEOUT_MS = 3000;
constexpr uint32_t LEG_TIMEOUT_MS = 90000;
constexpr uint32_t PROGRESS_TIMEOUT_MS = 15000;
constexpr double PROGRESS_DEG = 0.15;
constexpr double DIVERGENCE_DEG = 0.6;
constexpr double MIN_SHAFT_RESPONSE_DEG = 0.5;

enum class Phase { STARTUP, BASELINE, MOVING, OBSERVING, SETTLING, COMPLETE, ABORTED };
struct LegResult {
    bool settled = false;
    double finalPitch = 0, error = 0;
    double startPitch = 0, target = 0, shaftMinimum = 0, shaftMaximum = 0;
    uint32_t bursts = 0, bnoMotionSamples = 0, encoderMotionSamples = 0;
};
struct Window {
    bool active = false, interrupted = false;
    uint32_t started = 0, duration = 0, lastBno = 0, lastEncoder = 0;
    uint32_t encoderSamples = 0;
    Statistics pitch, firstHalf, secondHalf, shaft;
};

FastAccelStepperEngine engine;
FastAccelStepper *pitch = nullptr;
AS5600 pitchEncoder(&Wire1);
DiagnosticBno085 bno;
BnoHealth bnoHealth;
EulerAngles orientation = {0, 0, 0};
Phase phase = Phase::ABORTED;
LegResult legs[2];
Window window;
unsigned legIndex = 0;
bool finalPrinted = false, referenceSet = false;
bool bnoInitialized = false, reportEnabled = false, bnoValid = false;
bool encoderValid = false, shaftInitialized = false;
uint16_t encoderRaw = 0, previousRaw = 0;
uint8_t encoderMagnet = 0, bnoAccuracy = 0;
double shaftDegrees = 0, referencePitch = 0, measuredPitch = 0;
double baselineSd = 0, baselineRange = 0, bestError = 0, progressError = 0;
uint32_t startedAt = 0, phaseStartedAt = 0, legStartedAt = 0, lastProgressAt = 0;
uint32_t lastBnoGood = 0, lastEncoderGood = 0, lastEncoderService = 0;
uint32_t lastReportAttempt = 0, lastDisplay = 0;
uint32_t encoderReads = 0, encoderReadFailures = 0, encoderMagnetWarnings = 0;
uint32_t encoderMaxGap = 0, bnoMaxGap = 0, invalidBnoVectors = 0;
uint32_t bnoResets = 0, reportFailures = 0;

const char *phaseText() {
    switch (phase) {
    case Phase::STARTUP: return "STARTUP";
    case Phase::BASELINE: return "BASELINE";
    case Phase::MOVING: return "MOVING";
    case Phase::OBSERVING: return "OBSERVING";
    case Phase::SETTLING: return "SETTLING";
    case Phase::COMPLETE: return "COMPLETE";
    case Phase::ABORTED: return "ABORTED";
    }
    return "UNKNOWN";
}
const char *magnetText() {
    if (encoderMagnet & 0x10U) return "TOO-WEAK";
    if (encoderMagnet & 0x08U) return "TOO-STRONG";
    return encoderMagnet & 0x20U ? "DETECTED" : "NOT-DETECTED";
}
void stopPitch() {
    if (pitch) pitch->forceStop();
    delay(25); // Same verified FastAccelStepper queue drain as M04.
    digitalWrite(tmp_hardware::PITCH_STEP_PIN, LOW);
    digitalWrite(tmp_hardware::YAW_STEP_PIN, LOW);
    digitalWrite(tmp_hardware::CARRIAGE_STEP_PIN, LOW);
}
void finish(bool passed, const char *reason) {
    if (finalPrinted) return;
    stopPitch();
    finalPrinted = true;
    window.active = false;
    phase = passed ? Phase::COMPLETE : Phase::ABORTED;
    Serial.println("=== MILESTONE 5 CLOSED-LOOP PITCH SUMMARY ===");
    Serial.print("Reference BNO pitch_deg="); Serial.print(referencePitch, 3);
    Serial.print(" established="); Serial.println(referenceSet ? "YES" : "NO");
    Serial.print("Baseline sd/range_deg="); Serial.print(baselineSd, 3);
    Serial.print('/'); Serial.print(baselineRange, 3);
    Serial.print(" tolerance_deg=+/-"); Serial.println(TOLERANCE_DEG, 2);
    for (unsigned i = 0; i < 2; ++i) {
        Serial.print(i == 0 ? "OUTBOUND" : "RETURN");
        Serial.print(" settled="); Serial.print(legs[i].settled ? "YES" : "NO");
        if (legs[i].settled) {
            Serial.print(" relative_pitch_deg="); Serial.print(legs[i].finalPitch - referencePitch, 3);
            Serial.print(" final_error_deg="); Serial.print(legs[i].error, 3);
        }
        Serial.print(" bursts="); Serial.print(legs[i].bursts);
        Serial.print(" shaft_response_range_deg="); Serial.print(legs[i].shaftMaximum - legs[i].shaftMinimum, 2);
        Serial.print(" BNO/AS_motion_samples="); Serial.print(legs[i].bnoMotionSamples);
        Serial.print('/'); Serial.println(legs[i].encoderMotionSamples);
    }
    Serial.print("Last BNO relative_pitch_deg="); Serial.print(orientation.pitch - referencePitch, 3);
    Serial.print(" AS_motor_displacement_deg="); Serial.print(shaftDegrees, 2);
    Serial.print(" pulse_count_informational="); Serial.println(pitch ? pitch->getCurrentPosition() : 0);
    Serial.print("BNO fresh/invalid/resets/report_failures/write_failures=");
    Serial.print(bnoHealth.freshSamples); Serial.print('/'); Serial.print(invalidBnoVectors);
    Serial.print('/'); Serial.print(bnoResets); Serial.print('/'); Serial.print(reportFailures);
    Serial.print('/'); Serial.println(DiagnosticBno085::writeFailures);
    Serial.print("AS reads/read_failures/magnet_warnings="); Serial.print(encoderReads);
    Serial.print('/'); Serial.print(encoderReadFailures); Serial.print('/'); Serial.println(encoderMagnetWarnings);
    Serial.print("Maximum active BNO/AS acquisition_gap_ms="); Serial.print(bnoMaxGap);
    Serial.print('/'); Serial.println(encoderMaxGap);
    Serial.println("AS magnet flags are advisory. No measured/expected drivetrain ratio acceptance gate.");
    Serial.print("Reason: "); Serial.println(reason);
    Serial.print("FINAL RESULT: "); Serial.println(passed ? "PASS" : "FAIL");
    Serial.println("Latched idle; no automatic retry or return after abort. Reset starts a new test.");
    Serial.println("Physical verification remains pending operator review.");
    Serial.flush();
}
void abortTest(const char *reason) { finish(false, reason); }
bool sensorsFresh(uint32_t now) {
    return bnoValid && reportEnabled && encoderValid &&
        now - lastBnoGood < BNO_STALE_MS && now - lastEncoderGood < ENCODER_STALE_MS;
}
void checkSafety() {
    if (!referenceSet || finalPrinted) return;
    const uint32_t now = millis();
    bnoMaxGap = std::max(bnoMaxGap, now - lastBnoGood);
    encoderMaxGap = std::max(encoderMaxGap, now - lastEncoderGood);
    if (now - lastBnoGood >= BNO_STALE_MS)
        abortTest("BNO085 feedback stale: acquisition gap reached 150 ms");
    else if (now - lastEncoderGood >= ENCODER_STALE_MS)
        abortTest("AS5600 feedback stale: acquisition gap reached 100 ms");
    else if (bnoValid && (fabs(orientation.pitch) >= ABSOLUTE_LIMIT_DEG ||
             fabs(orientation.pitch - referencePitch) >= RELATIVE_LIMIT_DEG))
        abortTest("Measured pitch travel guard: relative 6 deg or absolute 75 deg");
    else if (now - legStartedAt >= LEG_TIMEOUT_MS)
        abortTest("Leg timeout: target did not settle within 90 seconds");
}
bool enableReport() {
    lastReportAttempt = millis();
    reportEnabled = bno.enableReport(SH2_ROTATION_VECTOR, milestone4::BNO_REPORT_INTERVAL_US);
    if (!reportEnabled) ++reportFailures;
    return reportEnabled;
}
bool handleReset() {
    if (!bno.wasReset()) return false;
    ++bnoResets;
    bnoValid = false;
    window.interrupted = true;
    if (referenceSet) {
        // A reset can change the orientation frame. Never silently re-zero it.
        abortTest("BNO085 reset after reference: orientation continuity lost");
    } else enableReport();
    return true;
}
void serviceBno() {
    if (!bnoInitialized) return;
    handleReset();
    if (finalPrinted) return;
    if (!reportEnabled && millis() - lastReportAttempt >= 500) enableReport();
    sh2_SensorValue_t event = {};
    const bool gotEvent = bno.getSensorEvent(&event);
    const bool reset = handleReset();
    // Check OLD timestamps after the library call, BEFORE accepting recovery.
    checkSafety();
    if (finalPrinted || reset || !gotEvent || !reportEnabled || event.sensorId != SH2_ROTATION_VECTOR) return;
    EulerAngles result;
    if (!quaternionToEuler(event.un.rotationVector, result)) { ++invalidBnoVectors; return; }
    const uint32_t now = millis();
    bnoHealth.recordFresh(now, referenceSet);
    orientation = result;
    bnoAccuracy = event.status;
    bnoValid = true;
    lastBnoGood = now;
    checkSafety(); // Every fresh raw pitch is guarded, not just the window mean.
    if (finalPrinted) return;
    if (referenceSet && phase == Phase::MOVING && pitch->isRunning()) ++legs[legIndex].bnoMotionSamples;
    if (window.active) {
        if (now - window.lastBno > WINDOW_GAP_MS) window.interrupted = true;
        window.lastBno = now;
        window.pitch.add(result.pitch);
        if (now - window.started < window.duration / 2) window.firstHalf.add(result.pitch);
        else window.secondHalf.add(result.pitch);
    }
}
void serviceEncoder() {
    if (millis() - lastEncoderService < ENCODER_INTERVAL_MS) return;
    lastEncoderService = millis();
    bool ok = pitchEncoder.isConnected();
    uint16_t raw = encoderRaw;
    uint8_t status = encoderMagnet;
    if (ok) { raw = pitchEncoder.rawAngle(); ok = pitchEncoder.lastError() == AS5600_OK && raw < 4096; }
    if (ok) { status = pitchEncoder.readStatus(); ok = pitchEncoder.lastError() == AS5600_OK; }
    checkSafety(); // A recovered blocking read must not hide its elapsed gap.
    if (finalPrinted) return;
    if (!ok) { ++encoderReadFailures; encoderValid = false; return; }
    const uint32_t now = millis();
    int delta = shaftInitialized ? static_cast<int>(raw) - previousRaw : 0;
    if (delta == 2048 || delta == -2048) { ++encoderReadFailures; encoderValid = false; return; }
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    // Shortest signed sensor delta, for shaft feedback only. There is no
    // comparison to commanded steps, motor microsteps, or cradle reduction.
    shaftDegrees += delta * (360.0 / 4096.0);
    shaftInitialized = true;
    previousRaw = encoderRaw = raw;
    encoderMagnet = status;
    ++encoderReads;
    if (!(status & 0x20U) || (status & 0x18U)) ++encoderMagnetWarnings;
    encoderValid = true;
    lastEncoderGood = now;
    if (referenceSet) {
        LegResult &leg = legs[legIndex];
        leg.shaftMinimum = fmin(leg.shaftMinimum, shaftDegrees);
        leg.shaftMaximum = fmax(leg.shaftMaximum, shaftDegrees);
        if (phase == Phase::MOVING && pitch->isRunning()) ++leg.encoderMotionSamples;
    }
    if (window.active) {
        if (now - window.lastEncoder > WINDOW_GAP_MS) window.interrupted = true;
        window.lastEncoder = now;
        ++window.encoderSamples;
        window.shaft.add(shaftDegrees);
    }
}
void startWindow(uint32_t duration) {
    window = {};
    window.active = true;
    window.started = window.lastBno = window.lastEncoder = millis();
    window.duration = duration;
}
bool usableWindow() {
    const uint32_t now = millis();
    const uint32_t minimumSamples = window.duration == SETTLE_WINDOW_MS ? 30 : 10;
    return !window.interrupted && window.pitch.n >= minimumSamples &&
        window.encoderSamples >= minimumSamples && window.firstHalf.n && window.secondHalf.n &&
        now - window.lastBno <= WINDOW_GAP_MS && now - window.lastEncoder <= WINDOW_GAP_MS &&
        sensorsFresh(now) && window.pitch.sd() <= 0.15 && window.pitch.range() <= 0.5 &&
        fabs(window.secondHalf.mean() - window.firstHalf.mean()) <= 0.2;
}
void startLeg(unsigned index, double startPitch) {
    legIndex = index;
    LegResult &leg = legs[index];
    leg.startPitch = startPitch;
    leg.target = referencePitch + (index == 0 ? TARGET_DELTA_DEG : 0);
    leg.shaftMinimum = leg.shaftMaximum = shaftDegrees;
    measuredPitch = startPitch;
    bestError = progressError = fabs(leg.target - startPitch);
    legStartedAt = lastProgressAt = millis();
    phase = Phase::OBSERVING;
    phaseStartedAt = millis();
    window.active = false;
    Serial.print("LEG "); Serial.print(index == 0 ? "OUTBOUND" : "RETURN");
    Serial.print(" target_relative_deg="); Serial.println(leg.target - referencePitch, 3);
}
bool checkProgress(double error) {
    const uint32_t now = millis();
    const double magnitude = fabs(error);
    if (magnitude > bestError + DIVERGENCE_DEG) {
        abortTest("Measured pitch diverged/wrong-way: check POSITIVE_STEP_PITCH_SIGN and mechanics");
        return false;
    }
    bestError = fmin(bestError, magnitude);
    if (magnitude <= progressError - PROGRESS_DEG) {
        progressError = magnitude;
        lastProgressAt = now;
    }
    if (magnitude > TOLERANCE_DEG && now - lastProgressAt >= PROGRESS_TIMEOUT_MS) {
        abortTest("No measured pitch progress for 15 seconds: inspect drive/load/sensors");
        return false;
    }
    if (legs[legIndex].bursts && now - legStartedAt >= 8000 &&
        legs[legIndex].shaftMaximum - legs[legIndex].shaftMinimum < MIN_SHAFT_RESPONSE_DEG) {
        abortTest("AS5600 readable but no motor-shaft response observed");
        return false;
    }
    return true;
}
void commandCorrection(double error) {
    checkSafety();
    if (finalPrinted) return;
    if (!sensorsFresh(millis()) || pitch->isRunning()) return;
    const int32_t delta = correctionSteps(error);
    if (!delta) return;
    if (pitch->setSpeedInHz(correctionSpeed(error)) != 0 || pitch->move(delta) != MOVE_OK) {
        abortTest("FastAccelStepper rejected correction"); return;
    }
    ++legs[legIndex].bursts;
    window.active = false;
    phase = Phase::MOVING;
    phaseStartedAt = millis();
}
void recordSettled() {
    LegResult &leg = legs[legIndex];
    if (!leg.bursts || !leg.bnoMotionSamples || !leg.encoderMotionSamples ||
        leg.shaftMaximum - leg.shaftMinimum < MIN_SHAFT_RESPONSE_DEG ||
        (legIndex == 0 ? measuredPitch - leg.startPitch : leg.startPitch - measuredPitch) < 1.0) {
        abortTest("Insufficient measured cradle/shaft response or sensor samples during motion"); return;
    }
    leg.settled = true;
    leg.finalPitch = measuredPitch;
    leg.error = leg.target - measuredPitch;
    Serial.print("SETTLED "); Serial.print(legIndex == 0 ? "OUTBOUND" : "RETURN");
    Serial.print(" relative_deg="); Serial.print(measuredPitch - referencePitch, 3);
    Serial.print(" error_deg="); Serial.println(leg.error, 3);
    if (legIndex == 0) startLeg(1, measuredPitch);
    else finish(true, "Both measured pitch targets reached and settled with shaft response and fresh sensors");
}
void serviceState() {
    const uint32_t now = millis();
    if (phase == Phase::STARTUP || phase == Phase::BASELINE) {
        if (now - startedAt >= BASELINE_TIMEOUT_MS) { abortTest("Stable startup sensor baseline unavailable"); return; }
        if (phase == Phase::STARTUP) {
            if (now - startedAt < STARTUP_MS) return;
            phase = Phase::BASELINE;
            startWindow(SETTLE_WINDOW_MS);
        }
        if (now - window.started < window.duration) return;
        if (!usableWindow() || window.shaft.range() > 1.0) { startWindow(SETTLE_WINDOW_MS); return; }
        referencePitch = window.pitch.mean();
        baselineSd = window.pitch.sd(); baselineRange = window.pitch.range();
        if (fabs(referencePitch) + RELATIVE_LIMIT_DEG >= ABSOLUTE_LIMIT_DEG) {
            abortTest("Startup pitch too close to absolute guard; reposition nearer level"); return;
        }
        referenceSet = true;
        Serial.print("REFERENCE pitch_deg="); Serial.print(referencePitch, 3);
        Serial.print(" sd/range_deg="); Serial.print(baselineSd, 3);
        Serial.print('/'); Serial.println(baselineRange, 3);
        startLeg(0, referencePitch);
        return;
    }
    if (phase == Phase::MOVING) {
        if (now - phaseStartedAt >= MOVE_TIMEOUT_MS) { abortTest("Finite motor burst timed out"); return; }
        if (pitch->isRunning()) return;
        phase = Phase::OBSERVING;
        phaseStartedAt = now;
        window.active = false;
        return;
    }
    if (phase != Phase::OBSERVING && phase != Phase::SETTLING) return;
    if (!window.active) {
        if (now - phaseStartedAt < OBSERVE_DELAY_MS) return;
        startWindow(OBSERVE_WINDOW_MS);
    }
    if (now - window.started < window.duration) return;
    if (!usableWindow()) {
        // No motion on noisy windows. Overall leg deadline still applies.
        startWindow(phase == Phase::SETTLING ? SETTLE_WINDOW_MS : OBSERVE_WINDOW_MS);
        return;
    }
    measuredPitch = window.pitch.mean();
    const double error = legs[legIndex].target - measuredPitch;
    if (!checkProgress(error)) return;
    const bool allInTolerance =
        fabs(legs[legIndex].target - window.pitch.minimum) <= TOLERANCE_DEG &&
        fabs(legs[legIndex].target - window.pitch.maximum) <= TOLERANCE_DEG;
    if (phase == Phase::SETTLING && allInTolerance) { recordSettled(); return; }
    if (fabs(error) <= APPROACH_DEADBAND_DEG) {
        phase = Phase::SETTLING;
        startWindow(SETTLE_WINDOW_MS);
    } else commandCorrection(error);
}
void printTelemetry() {
    Serial.print("STATE "); Serial.print(phaseText());
    Serial.print(" leg="); Serial.print(legIndex == 0 ? "OUTBOUND" : "RETURN");
    Serial.print(" BNO_pitch="); Serial.print(orientation.pitch, 3);
    if (referenceSet) {
        Serial.print(" relative="); Serial.print(orientation.pitch - referencePitch, 3);
        Serial.print(" target="); Serial.print(legs[legIndex].target - referencePitch, 3);
        Serial.print(" error="); Serial.print(legs[legIndex].target - orientation.pitch, 3);
    }
    Serial.print(" age_ms="); Serial.print(millis() - lastBnoGood);
    Serial.print(" accuracy="); Serial.print(bnoAccuracy);
    Serial.print(" AS_raw="); Serial.print(encoderRaw);
    Serial.print(" shaft_deg="); Serial.print(shaftDegrees, 2);
    Serial.print(" age_ms="); Serial.print(millis() - lastEncoderGood);
    Serial.print(" magnet_status="); Serial.print(magnetText());
    Serial.print(" pulse_count_info="); Serial.println(pitch ? pitch->getCurrentPosition() : 0);
}
} // namespace

void setup() {
    Serial.begin(115200);
    delay(500);
    for (const uint8_t pin : {tmp_hardware::YAW_STEP_PIN, tmp_hardware::PITCH_STEP_PIN, tmp_hardware::CARRIAGE_STEP_PIN}) {
        pinMode(pin, OUTPUT); digitalWrite(pin, LOW);
    }
    Serial.println("TMP M05_closed_loop_pitch_control - PITCH ONLY");
    Serial.println("Pitch DIR26 STEP12; TMC2209; 12 V supply; FastAccelStepper.");
    Serial.println("Bus B SDA4 SCL5: AS5600 0x36 motor shaft; BNO085 0x4A moving cradle.");
    Serial.println("Sequence: stable startup zero -> measured +3 deg -> settle -> measured zero.");
    Serial.println("Tolerance +/-0.4 deg for 1 s; maximum burst 16 pulses; 40..120 pulses/s; accel 240 pulses/s^2.");
    Serial.println("1600 pulses/rev and about 15:1 size bursts only; no ratio-based safety/PASS checks.");
    Serial.print("POSITIVE_STEP_PITCH_SIGN="); Serial.println(POSITIVE_STEP_PITCH_SIGN);
    Serial.println("Measured guards: +/-6 deg from reference, absolute +/-75 deg; no homing/endstops.");
    Serial.println("Clear travel both ways. X/x aborts; power removal is the physical emergency stop.");
    Serial.println("At least 5 s without motion, then stable baseline. Reset will start a new sequence.");
    engine.init();
    pitch = engine.stepperConnectToPin(tmp_hardware::PITCH_STEP_PIN);
    if (!pitch) { abortTest("Pitch FastAccelStepper initialization failed"); return; }
    pitch->setDirectionPin(tmp_hardware::PITCH_DIR_PIN, true, 200);
    if (pitch->setSpeedInHz(MIN_SPEED_HZ) != 0 || pitch->setAcceleration(ACCELERATION) != 0) {
        abortTest("Pitch speed/acceleration configuration rejected"); return;
    }
    if (!Wire1.begin(tmp_hardware::I2C_BUS_B_SDA_PIN, tmp_hardware::I2C_BUS_B_SCL_PIN, 100000)) {
        abortTest("Required I2C Bus B initialization failed"); return;
    }
    Wire1.setTimeOut(50);
    if (!pitchEncoder.begin() || !pitchEncoder.isConnected()) {
        abortTest("Pitch AS5600 0x36 initialization failed"); return;
    }
    Wire1.beginTransmission(0x4A);
    if (Wire1.endTransmission() != 0) { abortTest("Required BNO085 at Bus B 0x4A did not ACK"); return; }
    bnoInitialized = bno.begin_I2C(0x4A, &Wire1);
    if (!bnoInitialized || !enableReport()) { abortTest("BNO085 initialization/report enabling failed"); return; }
    startedAt = lastBnoGood = lastEncoderGood = millis();
    bnoHealth.lastFreshOrStartMs = startedAt;
    phase = Phase::STARTUP;
}
void loop() {
    if (finalPrinted) { delay(20); return; }
    while (Serial.available()) {
        const int input = Serial.read();
        if (input == 'X' || input == 'x') { abortTest("Operator X abort"); return; }
    }
    checkSafety();
    if (finalPrinted) return;
    serviceEncoder();
    if (finalPrinted) return;
    serviceBno();
    if (finalPrinted) return;
    serviceEncoder();
    checkSafety();
    if (finalPrinted) return;
    serviceState();
    if (!finalPrinted && millis() - lastDisplay >= 750) { lastDisplay = millis(); printTelemetry(); }
    delay(1);
}
