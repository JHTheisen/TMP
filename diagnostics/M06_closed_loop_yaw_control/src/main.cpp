#include <Arduino.h>
#include <algorithm>
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
using namespace milestone6;
constexpr uint32_t STARTUP_MS = 5000;
constexpr uint32_t BASELINE_TIMEOUT_MS = 15000;
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

enum class Phase { STARTUP, BASELINE, MOVING, OBSERVING, SETTLING, COMPLETE, ABORTED };
struct LegResult {
    bool settled = false, directionConfirmed = false;
    double startYaw = 0, target = 0, finalYaw = 0, error = 0;
    uint32_t bursts = 0, bnoMotionSamples = 0;
};
struct Window {
    bool active = false, interrupted = false;
    uint32_t started = 0, duration = 0, lastBno = 0;
    Statistics yaw, firstHalf, secondHalf;
};
FastAccelStepperEngine engine;
FastAccelStepper *yaw = nullptr;
DiagnosticBno085 bno;
BnoHealth bnoHealth;
EulerAngles orientation = {0, 0, 0};
HeadingTracker heading;
Phase phase = Phase::ABORTED;
LegResult legs[2];
Window window;
unsigned legIndex = 0;
bool finalPrinted = false, referenceSet = false;
bool bnoInitialized = false, reportEnabled = false, bnoValid = false;
uint8_t bnoAccuracy = 0;
// All windows average this continuous coordinate. Raw 359.9/0.1 readings
// therefore average near zero, never 180. No step count contributes to it.
double referenceCoordinate = 0, referenceHeading = 0, measuredYaw = 0;
double baselineSd = 0, baselineRange = 0, bestError = 0, progressError = 0;
uint32_t startedAt = 0, phaseStartedAt = 0, legStartedAt = 0, lastProgressAt = 0;
uint32_t lastBnoGood = 0, lastReportAttempt = 0, lastDisplay = 0;
uint32_t bnoMaxGap = 0, invalidBnoVectors = 0, bnoResets = 0, reportFailures = 0;
unsigned directionGoodWindows = 0, directionBadWindows = 0, divergenceWindows = 0;

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
double relativeYaw() { return heading.continuous - referenceCoordinate; }
void stopYaw() {
    if (yaw) yaw->forceStop();
    delay(25); // Preserve M05's verified FastAccelStepper queue-drain behavior.
    digitalWrite(tmp_hardware::YAW_STEP_PIN, LOW);
    digitalWrite(tmp_hardware::PITCH_STEP_PIN, LOW);
    digitalWrite(tmp_hardware::CARRIAGE_STEP_PIN, LOW);
}
void finish(bool passed, const char *reason) {
    if (finalPrinted) return;
    stopYaw();
    finalPrinted = true;
    window.active = false;
    phase = passed ? Phase::COMPLETE : Phase::ABORTED;
    Serial.println("=== MILESTONE 6 CLOSED-LOOP RELATIVE YAW SUMMARY ===");
    Serial.print("Reference BNO heading_deg="); Serial.print(referenceHeading, 3);
    Serial.print(" established="); Serial.println(referenceSet ? "YES" : "NO");
    Serial.print("Baseline sd/range_deg="); Serial.print(baselineSd, 3);
    Serial.print('/'); Serial.print(baselineRange, 3);
    Serial.print(" tolerance_deg=+/-"); Serial.println(TOLERANCE_DEG, 2);
    Serial.print("Trial positive STEP yaw sign="); Serial.println(TRIAL_POSITIVE_STEP_YAW_SIGN);
    for (unsigned i = 0; i < 2; ++i) {
        Serial.print(i == 0 ? "OUTBOUND" : "RETURN");
        Serial.print(" settled="); Serial.print(legs[i].settled ? "YES" : "NO");
        Serial.print(" direction_confirmed="); Serial.print(legs[i].directionConfirmed ? "YES" : "NO");
        if (legs[i].settled) {
            Serial.print(" relative_yaw_deg="); Serial.print(legs[i].finalYaw, 3);
            Serial.print(" final_error_deg="); Serial.print(legs[i].error, 3);
        }
        Serial.print(" bursts="); Serial.print(legs[i].bursts);
        Serial.print(" BNO_motion_samples="); Serial.println(legs[i].bnoMotionSamples);
    }
    Serial.print("Last BNO heading_deg="); Serial.print(orientation.heading, 3);
    Serial.print(" relative_yaw_deg="); Serial.print(relativeYaw(), 3);
    Serial.print(" pulse_count_informational="); Serial.println(yaw ? yaw->getCurrentPosition() : 0);
    Serial.print("BNO fresh/invalid/resets/report_failures/write_failures=");
    Serial.print(bnoHealth.freshSamples); Serial.print('/'); Serial.print(invalidBnoVectors);
    Serial.print('/'); Serial.print(bnoResets); Serial.print('/'); Serial.print(reportFailures);
    Serial.print('/'); Serial.println(DiagnosticBno085::writeFailures);
    Serial.print("Maximum active BNO acquisition_gap_ms="); Serial.println(bnoMaxGap);
    Serial.println("AS5600 yaw feedback NOT REQUIRED. No ratio-based acceptance; no north target.");
    Serial.print("Reason: "); Serial.println(reason);
    Serial.print("FINAL RESULT: "); Serial.println(passed ? "PASS" : "FAIL");
    Serial.println("Latched idle; no retry, automatic sign change, or return after abort. Reset starts a new test.");
    Serial.println("M06 physical verification remains pending operator review.");
    Serial.flush();
}
void abortTest(const char *reason) { finish(false, reason); }
bool sensorFresh(uint32_t now) {
    return bnoValid && reportEnabled && now - lastBnoGood < BNO_STALE_MS;
}
void checkSafety() {
    if (!referenceSet || finalPrinted) return;
    const uint32_t now = millis();
    bnoMaxGap = std::max(bnoMaxGap, now - lastBnoGood);
    if (now - lastBnoGood >= BNO_STALE_MS)
        abortTest("BNO085 feedback stale: acquisition gap reached 150 ms");
    // Use continuous measured travel: crossing 0/360 never resets the guard.
    else if (bnoValid && fabs(relativeYaw()) >= RELATIVE_LIMIT_DEG)
        abortTest("Measured yaw travel guard: +/-6 degrees from startup");
    else if (bnoValid && fabs(orientation.pitch) >= MAX_USABLE_PITCH_DEG)
        abortTest("BNO pitch too near vertical for usable Euler heading; no pitch motion commanded");
    else if (now - legStartedAt >= LEG_TIMEOUT_MS)
        abortTest("Leg timeout: yaw target did not settle within 90 seconds");
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
    if (referenceSet) abortTest("BNO085 reset after reference: yaw continuity lost");
    else {
        heading = {};
        enableReport();
    }
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
    // Preserve the OLD gap even when a blocked library call returns valid data.
    checkSafety();
    if (finalPrinted || reset || !gotEvent || !reportEnabled || event.sensorId != SH2_ROTATION_VECTOR) return;
    EulerAngles result;
    if (!quaternionToEuler(event.un.rotationVector, result)) { ++invalidBnoVectors; return; }
    if (!heading.update(result.heading)) {
        abortTest("Ambiguous/nonfinite BNO heading transition"); return;
    }
    const uint32_t now = millis();
    bnoHealth.recordFresh(now, referenceSet);
    orientation = result;
    bnoAccuracy = event.status;
    bnoValid = true;
    lastBnoGood = now;
    checkSafety(); // Guard every raw sensor update, before averaging.
    if (finalPrinted) return;
    if (referenceSet && phase == Phase::MOVING && yaw->isRunning()) ++legs[legIndex].bnoMotionSamples;
    if (window.active) {
        if (now - window.lastBno > WINDOW_GAP_MS) window.interrupted = true;
        window.lastBno = now;
        window.yaw.add(heading.continuous);
        if (now - window.started < window.duration / 2) window.firstHalf.add(heading.continuous);
        else window.secondHalf.add(heading.continuous);
    }
}
void startWindow(uint32_t duration) {
    window = {};
    window.active = true;
    window.started = window.lastBno = millis();
    window.duration = duration;
}
bool usableWindow() {
    const uint32_t now = millis();
    const uint32_t minimumSamples = window.duration == SETTLE_WINDOW_MS ? 30 : 10;
    return !window.interrupted && window.yaw.n >= minimumSamples &&
        window.firstHalf.n && window.secondHalf.n &&
        now - window.lastBno <= WINDOW_GAP_MS && sensorFresh(now) &&
        window.yaw.sd() <= 0.15 && window.yaw.range() <= 0.5 &&
        fabs(window.secondHalf.mean() - window.firstHalf.mean()) <= 0.2;
}
void reobserve() {
    // Confirm a direction warning with a fresh stopped window before allowing
    // another motor command. A single noisy observation cannot reverse polarity.
    phase = Phase::OBSERVING;
    startWindow(OBSERVE_WINDOW_MS);
}
void startLeg(unsigned index, double startYaw) {
    legIndex = index;
    LegResult &leg = legs[index];
    leg.startYaw = startYaw;
    leg.target = index == 0 ? TARGET_DELTA_DEG : 0;
    measuredYaw = startYaw;
    bestError = progressError = fabs(shortestDifference(leg.target, startYaw));
    legStartedAt = lastProgressAt = millis();
    directionGoodWindows = directionBadWindows = divergenceWindows = 0;
    phase = Phase::OBSERVING;
    phaseStartedAt = millis();
    window.active = false;
    Serial.print("LEG "); Serial.print(index == 0 ? "OUTBOUND" : "RETURN");
    Serial.print(" target_relative_deg="); Serial.print(leg.target, 3);
    Serial.println(" direction=UNCONFIRMED (4-pulse trials at 40 pulses/s)");
}
bool checkProgress(double error) {
    LegResult &leg = legs[legIndex];
    const double magnitude = fabs(error);
    if (leg.bursts && !leg.directionConfirmed) {
        const double toward = shortestDifference(measuredYaw, leg.startYaw) * (legIndex == 0 ? 1 : -1);
        if (toward <= -DIRECTION_RESPONSE_DEG) {
            directionGoodWindows = 0;
            if (++directionBadWindows >= 2) {
                abortTest("DIRECTION MISMATCH: measured yaw opposes target. Verify trial yaw sign and mechanics; no automatic retry.");
            } else reobserve();
            return false;
        }
        directionBadWindows = 0;
        if (toward >= DIRECTION_RESPONSE_DEG) {
            if (++directionGoodWindows < 2) { reobserve(); return false; }
            leg.directionConfirmed = true;
            Serial.println("DIRECTION CONFIRMED: two stopped BNO windows show yaw toward target");
        } else directionGoodWindows = 0;
    }
    if (magnitude > bestError + DIVERGENCE_DEG) {
        if (++divergenceWindows >= 2)
            abortTest("DIRECTION MISMATCH / RUNAWAY: measured yaw error repeatedly increased by over 0.6 deg");
        else reobserve();
        return false;
    }
    divergenceWindows = 0;
    bestError = fmin(bestError, magnitude);
    if (magnitude <= progressError - PROGRESS_DEG) {
        progressError = magnitude; lastProgressAt = millis();
    }
    if (magnitude > TOLERANCE_DEG && millis() - lastProgressAt >= PROGRESS_TIMEOUT_MS) {
        abortTest("No measured yaw progress for 15 seconds: inspect motor/load/BNO"); return false;
    }
    return true;
}
void commandCorrection(double error) {
    checkSafety();
    if (finalPrinted || !sensorFresh(millis()) || yaw->isRunning()) return;
    const int32_t delta = correctionSteps(error, legs[legIndex].directionConfirmed);
    if (!delta) return;
    if (yaw->setSpeedInHz(correctionSpeed(error, legs[legIndex].directionConfirmed)) != 0 ||
        yaw->move(delta) != MOVE_OK) { abortTest("FastAccelStepper rejected yaw correction"); return; }
    ++legs[legIndex].bursts;
    window.active = false;
    phase = Phase::MOVING;
    phaseStartedAt = millis();
}
void recordSettled() {
    LegResult &leg = legs[legIndex];
    const double toward = shortestDifference(measuredYaw, leg.startYaw) * (legIndex == 0 ? 1 : -1);
    if (!leg.bursts || !leg.bnoMotionSamples || !leg.directionConfirmed || toward < 1.0) {
        abortTest("Insufficient measured yaw response, direction confirmation, or BNO samples during motion"); return;
    }
    leg.settled = true;
    leg.finalYaw = measuredYaw;
    leg.error = shortestDifference(leg.target, measuredYaw);
    Serial.print("SETTLED "); Serial.print(legIndex == 0 ? "OUTBOUND" : "RETURN");
    Serial.print(" relative_deg="); Serial.print(measuredYaw, 3);
    Serial.print(" error_deg="); Serial.println(leg.error, 3);
    if (legIndex == 0) startLeg(1, measuredYaw);
    else finish(true, "Both measured relative yaw targets settled with confirmed direction and fresh BNO feedback");
}
void serviceState() {
    const uint32_t now = millis();
    if (phase == Phase::STARTUP || phase == Phase::BASELINE) {
        if (now - startedAt >= BASELINE_TIMEOUT_MS) { abortTest("Stable startup yaw baseline unavailable"); return; }
        if (phase == Phase::STARTUP) {
            if (now - startedAt < STARTUP_MS) return;
            phase = Phase::BASELINE;
            startWindow(SETTLE_WINDOW_MS);
        }
        if (now - window.started < window.duration) return;
        if (!usableWindow()) { startWindow(SETTLE_WINDOW_MS); return; }
        if (fabs(orientation.pitch) >= MAX_USABLE_PITCH_DEG) {
            abortTest("Startup pitch too near vertical; reposition for usable Euler yaw"); return;
        }
        referenceCoordinate = window.yaw.mean();
        referenceHeading = wrap360(heading.first + referenceCoordinate);
        baselineSd = window.yaw.sd(); baselineRange = window.yaw.range();
        referenceSet = true;
        Serial.print("REFERENCE heading_deg="); Serial.print(referenceHeading, 3);
        Serial.print(" sd/range_deg="); Serial.print(baselineSd, 3);
        Serial.print('/'); Serial.println(baselineRange, 3);
        startLeg(0, 0);
        return;
    }
    if (phase == Phase::MOVING) {
        if (now - phaseStartedAt >= MOVE_TIMEOUT_MS) { abortTest("Finite yaw burst timed out"); return; }
        if (yaw->isRunning()) return;
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
        startWindow(phase == Phase::SETTLING ? SETTLE_WINDOW_MS : OBSERVE_WINDOW_MS);
        return; // Motor stays stopped on unstable windows; leg deadline applies.
    }
    measuredYaw = window.yaw.mean() - referenceCoordinate;
    const double error = shortestDifference(legs[legIndex].target, measuredYaw);
    if (!checkProgress(error)) return;
    const bool allInTolerance =
        fabs(shortestDifference(legs[legIndex].target, window.yaw.minimum - referenceCoordinate)) <= TOLERANCE_DEG &&
        fabs(shortestDifference(legs[legIndex].target, window.yaw.maximum - referenceCoordinate)) <= TOLERANCE_DEG;
    if (phase == Phase::SETTLING && allInTolerance) { recordSettled(); return; }
    if (fabs(error) <= APPROACH_DEADBAND_DEG) {
        phase = Phase::SETTLING;
        startWindow(SETTLE_WINDOW_MS);
    } else commandCorrection(error);
}
void printTelemetry() {
    Serial.print("STATE "); Serial.print(phaseText());
    Serial.print(" leg="); Serial.print(legIndex == 0 ? "OUTBOUND" : "RETURN");
    Serial.print(" BNO_heading="); Serial.print(orientation.heading, 3);
    if (referenceSet) {
        Serial.print(" relative_yaw="); Serial.print(relativeYaw(), 3);
        Serial.print(" target="); Serial.print(legs[legIndex].target, 3);
        Serial.print(" error="); Serial.print(shortestDifference(legs[legIndex].target, relativeYaw()), 3);
    }
    Serial.print(" direction="); Serial.print(legs[legIndex].directionConfirmed ? "CONFIRMED" : "UNCONFIRMED");
    Serial.print(" age_ms="); Serial.print(millis() - lastBnoGood);
    Serial.print(" accuracy="); Serial.print(bnoAccuracy);
    Serial.print(" BNO_samples="); Serial.print(bnoHealth.freshSamples);
    Serial.print(" pulse_count_info="); Serial.println(yaw ? yaw->getCurrentPosition() : 0);
}
} // namespace

void setup() {
    Serial.begin(115200);
    delay(500);
    for (const uint8_t pin : {tmp_hardware::YAW_STEP_PIN, tmp_hardware::PITCH_STEP_PIN, tmp_hardware::CARRIAGE_STEP_PIN}) {
        pinMode(pin, OUTPUT); digitalWrite(pin, LOW);
    }
    Serial.println("TMP M06_closed_loop_yaw_control - RELATIVE YAW ONLY");
    Serial.println("Yaw DIR32 STEP33; FastAccelStepper. Pitch and carriage STEP held LOW.");
    Serial.println("BNO085: Bus B SDA4 SCL5 address 0x4A, rotation vector at 10 ms. No AS5600 required.");
    Serial.println("Sequence: stable yaw zero -> measured +3 deg -> settle -> measured zero. No north pointing.");
    Serial.println("Tolerance +/-0.4 deg for 1 s; 4-pulse direction trials then <=16 pulses; 40..120 pulses/s; accel 240.");
    Serial.print("UNVERIFIED trial positive STEP yaw sign="); Serial.println(TRIAL_POSITIVE_STEP_YAW_SIGN);
    Serial.println("Two stopped measured-response windows must confirm direction; mismatch aborts without auto-flipping.");
    Serial.println("Measured yaw guard +/-6 deg; 0/360 wrap handled. No homing, endstops, or drivetrain-ratio checks.");
    Serial.println("Clear yaw travel both ways. X/x aborts; power removal is the physical emergency stop.");
    Serial.println("At least 5 s without motion, then stable baseline. Reset starts a new sequence.");
    engine.init();
    yaw = engine.stepperConnectToPin(tmp_hardware::YAW_STEP_PIN);
    if (!yaw) { abortTest("Yaw FastAccelStepper initialization failed"); return; }
    yaw->setDirectionPin(tmp_hardware::YAW_DIR_PIN, true, 200);
    if (yaw->setSpeedInHz(MIN_SPEED_HZ) != 0 || yaw->setAcceleration(ACCELERATION) != 0) {
        abortTest("Yaw speed/acceleration configuration rejected"); return;
    }
    if (!Wire1.begin(tmp_hardware::I2C_BUS_B_SDA_PIN, tmp_hardware::I2C_BUS_B_SCL_PIN, 100000)) {
        abortTest("Required I2C Bus B initialization failed"); return;
    }
    Wire1.setTimeOut(50);
    Wire1.beginTransmission(0x4A);
    if (Wire1.endTransmission() != 0) { abortTest("Required BNO085 at Bus B 0x4A did not ACK"); return; }
    bnoInitialized = bno.begin_I2C(0x4A, &Wire1);
    if (!bnoInitialized || !enableReport()) { abortTest("BNO085 initialization/report enabling failed"); return; }
    startedAt = lastBnoGood = millis();
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
    serviceBno();
    checkSafety();
    if (finalPrinted) return;
    serviceState();
    if (!finalPrinted && millis() - lastDisplay >= 750) { lastDisplay = millis(); printTelemetry(); }
    delay(1);
}
