#include <Arduino.h>
#include <Wire.h>
#include <FastAccelStepper.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "hardware_config.h"
#include "sensor_support.h"
#include "motion_watchdog.h"

// Standalone experiment: no M08 sign multiplier or feedback corrections.
constexpr int32_t TEST_PULSES = 200;
constexpr uint32_t TEST_HZ = 40, TEST_ACCEL = 240;
constexpr uint32_t STALE_MS = 150, SETTLE_MS = 2000, SAMPLE_MS = 1000;
constexpr uint32_t MOVE_TIMEOUT_MS = 10000, WINDOW_TIMEOUT_MS = 30000;
constexpr double MIN_RESPONSE_DEG = 0.3, MAX_WINDOW_RANGE = 0.2;
constexpr double MAX_EXCURSION_DEG = 6.0, MAX_ABSOLUTE_PITCH = 75.0;
enum class Phase { IDLE, BASELINE, PREMOVE, MOVING, SETTLING, SAMPLE, DONE, FAULT };
Phase phase = Phase::IDLE;
FastAccelStepperEngine engine;
FastAccelStepper *motor = nullptr;
milestone4::DiagnosticBno085 bno;
milestone7::MotionWatchdog watchdog;
bool initialized = false, haveBno = false;
uint32_t lastFresh = 0, phaseAt = 0, windowAt = 0, lastPrint = 0;
double pitch = 0, initialPitch = 0;
uint8_t accuracy = 0;
unsigned leg = 0;
int32_t commandStart = 0;
char output[4096];
size_t outputUsed = 0, outputSent = 0;

void logText(const char *format, ...) {
    if (outputSent == outputUsed) outputSent = outputUsed = 0;
    va_list args; va_start(args, format);
    const int n = vsnprintf(output + outputUsed, sizeof(output) - outputUsed, format, args);
    va_end(args);
    if (n > 0) outputUsed += static_cast<size_t>(n) < sizeof(output) - outputUsed
        ? static_cast<size_t>(n) : sizeof(output) - outputUsed - 1;
}
void flushOutput() {
    const int room = Serial.availableForWrite();
    if (room > 0 && outputSent < outputUsed) {
        const size_t count = outputUsed - outputSent < static_cast<size_t>(room)
            ? outputUsed - outputSent : static_cast<size_t>(room);
        outputSent += Serial.write(reinterpret_cast<const uint8_t *>(output + outputSent), count);
    }
}
void fail(const char *reason) {
    if (phase == Phase::FAULT) return;
    if (motor) motor->forceStop();
    phase = Phase::FAULT; // Keep armed watchdog reasserting stop if already tripped.
    logText("FAULT: %s. No reverse/retry; reset required.\n", reason);
}
struct Window {
    unsigned n = 0;
    double sum = 0, low = 0, high = 0;
    void add(double value) {
        if (!n) low = high = value;
        low = fmin(low, value); high = fmax(high, value); sum += value; ++n;
    }
    double mean() const { return n ? sum / n : NAN; }
} window;
struct Encoder { bool read = false, good = false; uint8_t status = 0; double angle = NAN; };
struct Snapshot { double bnoMean = 0, range = 0; unsigned n = 0; Encoder encoder; } snapshots[3];
double responses[2] = {0, 0};
int directions[2] = {0, 0};

// Advisory reads ONLY when stopped and watchdog disarmed. No encoder writes.
Encoder readEncoder() {
    Encoder result;
    Wire1.beginTransmission(0x36); Wire1.write(static_cast<uint8_t>(0x0B));
    if (Wire1.endTransmission(false) != 0) return result;
    if (Wire1.requestFrom(static_cast<uint8_t>(0x36), static_cast<uint8_t>(3)) != 3) return result;
    const int status = Wire1.read(), high = Wire1.read(), low = Wire1.read();
    if (status < 0 || high < 0 || low < 0 || (high & 0xF0)) return result;
    result.read = true; result.status = static_cast<uint8_t>(status);
    result.good = (status & 0x38) == 0x20;
    result.angle = ((high << 8) | low) * (360.0 / 4096.0);
    return result;
}
const char *directionText(int sign) { return sign > 0 ? "INCREASES" : sign < 0 ? "DECREASES" : "INCONCLUSIVE"; }
void reportLeg() {
    const Snapshot &before = snapshots[leg], &after = snapshots[leg + 1];
    const double delta = after.bnoMean - before.bnoMean;
    const double threshold = fmax(MIN_RESPONSE_DEG, before.range + after.range);
    const int direction = fabs(delta) >= threshold ? (delta > 0 ? 1 : -1) : 0;
    responses[leg] = delta; directions[leg] = direction;
    logText("RESULT command=%+ld pulses generated=%ld BNO_before=%.3f BNO_after=%.3f delta=%+.3f deg -> %s (threshold=%.3f)\n",
        static_cast<long>(leg == 0 ? TEST_PULSES : -TEST_PULSES),
        static_cast<long>(motor->getCurrentPosition() - commandStart), before.bnoMean, after.bnoMean, delta,
        directionText(direction), threshold);
    if (before.encoder.read && after.encoder.read) {
        double change = after.encoder.angle - before.encoder.angle;
        if (change >= 180) change -= 360;
        if (change < -180) change += 360;
        const bool usable = before.encoder.good && after.encoder.good && fabs(change) >= 1 && fabs(change) < 120;
        logText("AS5600 wrapped_delta=%+.3f shaft_deg numeric_direction=%s BNO_numeric_agreement=%s; advisory endpoints only\n",
            change, usable ? directionText(change > 0 ? 1 : -1) : "INCONCLUSIVE",
            usable && direction ? ((change > 0) == (delta > 0) ? "SAME" : "OPPOSITE") : "INCONCLUSIVE");
    } else logText("AS5600 comparison=UNAVAILABLE; does not affect test\n");
}
void capture(unsigned index) {
    watchdog.disarm();
    Snapshot &s = snapshots[index];
    s.bnoMean = window.mean(); s.range = window.high - window.low; s.n = window.n;
    s.encoder = readEncoder();
    logText("SNAPSHOT %u BNO_mean=%.3f range=%.3f samples=%u AS5600_read=%s raw_deg=%.3f status=0x%02X MD=%u ML=%u MH=%u quality=%s\n",
        index, s.bnoMean, s.range, s.n, s.encoder.read ? "OK" : "FAILED", s.encoder.angle,
        s.encoder.status, (s.encoder.status >> 5) & 1, (s.encoder.status >> 4) & 1, (s.encoder.status >> 3) & 1,
        s.encoder.read && s.encoder.good ? "MAGNET_FLAGS_OK_NOT_PROOF" : "UNRELIABLE/UNAVAILABLE");
}
void pollBno() {
    if (bno.wasReset()) { fail("BNO reset"); return; }
    sh2_SensorValue_t value;
    const bool got = bno.getSensorEvent(&value);
    if (bno.wasReset()) { fail("BNO reset during read"); return; }
    watchdog.check(millis());
    if (watchdog.tripped()) { fail("BNO stale >=150 ms"); return; }
    if (!got || value.sensorId != SH2_ROTATION_VECTOR) return;
    milestone4::EulerAngles angles;
    if (!milestone4::quaternionToEuler(value.un.rotationVector, angles)) { fail("invalid BNO rotation vector"); return; }
    pitch = angles.pitch; accuracy = value.status & 3; haveBno = true;
   
    if (millis() - lastFresh >= STALE_MS) window = {};
    lastFresh = millis(); watchdog.recordFresh(lastFresh);
    if (fabs(pitch) >= MAX_ABSOLUTE_PITCH) { fail("absolute pitch >=75 deg"); return; }
    if ((phase == Phase::MOVING || phase == Phase::SETTLING || phase == Phase::SAMPLE) &&
        fabs(pitch - initialPitch) >= MAX_EXCURSION_DEG) { fail("pitch excursion >=6 deg from baseline"); return; }
    if (phase == Phase::BASELINE || phase == Phase::SAMPLE) {
        if (!window.n) windowAt = millis();
        window.add(pitch);
    }
}
void setup() {
    Serial.begin(115200);
    for (const uint8_t pin : {tmp_hardware::YAW_STEP_PIN, tmp_hardware::PITCH_STEP_PIN, tmp_hardware::CARRIAGE_STEP_PIN}) {
        pinMode(pin, OUTPUT); digitalWrite(pin, LOW);
    }
    engine.init(); motor = engine.stepperConnectToPin(tmp_hardware::PITCH_STEP_PIN);
    if (!motor) { fail("pitch motor initialization"); return; }
    motor->setDirectionPin(tmp_hardware::PITCH_DIR_PIN, true, 200);
    if (motor->setSpeedInHz(TEST_HZ) != 0 || motor->setAcceleration(TEST_ACCEL) != 0 ||
        !watchdog.begin(motor, STALE_MS)) { fail("motor/watchdog initialization"); return; }
    if (!Wire1.begin(tmp_hardware::I2C_BUS_B_SDA_PIN, tmp_hardware::I2C_BUS_B_SCL_PIN, 100000)) { fail("Bus B initialization"); return; }
    Wire1.setTimeOut(50);
    if (!bno.begin_I2C(0x4A, &Wire1)) { fail("BNO initialization"); return; }
    bno.wasReset(); // Consume the normal initialization reset before enabling reports.
    if (!bno.enableReport(SH2_ROTATION_VECTOR, 10000)) { fail("BNO report initialization"); return; }
    initialized = true;
    logText("PITCH DIRECTION CHECK: DIR26 STEP12. No yaw/carriage motion. BNO Bus B 0x4A; optional shaft AS5600 0x36.\n"
        "G starts ONE +200/-200 pulse pair at 40 Hz, acceleration 240. Positive=DIR HIGH, negative=DIR LOW.\n"
        "Nominal 1600 pulses/motor rev and 15:1 -> 3 deg/leg, conditional on unchanged microstepping.\n"
        "Clear travel in BOTH directions. X aborts; reset after completion/fault. No automatic startup motion.\n");
}
void loop() {
    flushOutput();
    for (unsigned n = 0; n < 16 && Serial.available(); ++n) {
        const int c = Serial.read();
        if (c == 'X' || c == 'x') fail("operator X");
        else if ((c == 'G' || c == 'g') && initialized && phase == Phase::IDLE) {
            phase = Phase::BASELINE; phaseAt = millis(); window = {};
            logText("Collecting stable BNO baseline before any motion...\n");
        }
    }
    if (!initialized || phase == Phase::FAULT || phase == Phase::DONE) { delay(1); return; }
    pollBno();
    if (phase == Phase::FAULT) return;
    const uint32_t now = millis();
    if (haveBno && now - lastPrint >= 500 && outputUsed == outputSent) {
        lastPrint = now;
        logText("LIVE BNO_pitch=%.3f accuracy=%u good_age_ms=%lu moving=%s\n", pitch, accuracy,
            static_cast<unsigned long>(now - lastFresh), motor->isRunning() ? "YES" : "NO");
    }
    if (phase == Phase::BASELINE || phase == Phase::SAMPLE) {
        if (now - phaseAt >= WINDOW_TIMEOUT_MS) { fail("no stable BNO measurement window"); return; }
        if (window.n >= 50 && now - windowAt >= SAMPLE_MS && now - lastFresh < STALE_MS) {
            if (window.high - window.low > MAX_WINDOW_RANGE) { window = {}; return; }
            if (phase == Phase::BASELINE) { capture(0); initialPitch = snapshots[0].bnoMean; }
            else {
                capture(leg + 1); reportLeg();
                if (leg == 1) {
                    phase = Phase::DONE;
                    logText("FINAL positive_steps BNO_%s; negative_steps BNO_%s; return_residual=%+.3f deg\n",
                        directionText(directions[0]), directionText(directions[1]), snapshots[2].bnoMean - snapshots[0].bnoMean);
                    if (directions[0] && directions[1] == -directions[0])
                        logText("Observed candidate POSITIVE_STEP_PITCH_SIGN=%+d; verify repeatability. Firmware constants NOT changed.\n", directions[0]);
                    else logText("Direction INCONCLUSIVE: no sign recommendation. Review backlash, BNO stability and physical motion.\n");
                    logText("DONE: no further movement until reset. Equal reverse pulses do not guarantee physical return.\n");
                    return;
                }
                ++leg;
            }
            phase = Phase::PREMOVE; phaseAt = millis();
            logText("NEXT command=%+ld raw motor pulses, DIR=%s; waiting for output drain and a new good BNO sample\n",
                static_cast<long>(leg == 0 ? TEST_PULSES : -TEST_PULSES), leg == 0 ? "HIGH" : "LOW");
        }
    } else if (phase == Phase::PREMOVE) {
        if (now - phaseAt >= WINDOW_TIMEOUT_MS) { fail("pre-move readiness timeout"); return; }
        if (outputSent == outputUsed && lastFresh > phaseAt && now - lastFresh < STALE_MS) {
            if (fabs(pitch - snapshots[leg].bnoMean) > MAX_WINDOW_RANGE) { fail("pitch changed after stopped measurement"); return; }
            commandStart = motor->getCurrentPosition();
            if (!watchdog.arm(lastFresh)) { fail("watchdog arm"); return; }
            if (motor->move(leg == 0 ? TEST_PULSES : -TEST_PULSES) != MOVE_OK) { fail("motor command rejected"); return; }
            watchdog.check(millis());
            if (watchdog.tripped()) { fail("watchdog during motor command"); return; }
            phase = Phase::MOVING; phaseAt = millis();
        }
    } else if (phase == Phase::MOVING) {
        if (now - phaseAt >= MOVE_TIMEOUT_MS) { fail("fixed pulse move timeout"); return; }
        if (!motor->isRunning()) {
            if (motor->getCurrentPosition() - commandStart != (leg == 0 ? TEST_PULSES : -TEST_PULSES)) {
                fail("generated pulse endpoint mismatch"); return;
            }
            phase = Phase::SETTLING; phaseAt = now;
        }
    } else if (phase == Phase::SETTLING && now - phaseAt >= SETTLE_MS) {
        phase = Phase::SAMPLE; phaseAt = now; window = {};
    }
    delay(1);
}
