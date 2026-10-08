#pragma once
// Deterministic three-axis decision fixture, not an ESP32 scheduler or validated
// motor/gearbox model. Integrate independent acceleration-limited motors at 1 ms.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <initializer_list>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>
using std::isfinite;
constexpr int HEX = 16, OUTPUT = 1, LOW = 0;
constexpr float RAD_TO_DEG = 57.29577951308232F;

namespace simulated {
enum class Drive { IDLE, FINITE, CONTINUOUS, BRAKING };
struct PlantMotor {
    Drive drive = Drive::IDLE;
    uint32_t runningVisibleAt = 0;
    double position = 0, velocity = 0, target = 0, degrees = 0;
    double degreesPerStep = 0.02;
    int physicalSign = -1, direction = 1;
    // Keep continuous run*() and finite move() response independently
    // configurable. Powered LEVEL evidence established these two production
    // command paths separately; tests must not infer one from the other.
    int continuousPhysicalMultiplier = 1, finitePhysicalMultiplier = 1;
    int activePhysicalMultiplier = 1;
    double speed = 40;
    uint32_t stoppedAt = 0;
    int32_t acceleration = 240;
    bool frozen = false, neverStops = false, needsStoppedSample = false, idleForceStopLatched = false;
    // Opt-in FAS sub-hertz fixture: a normal stop drains the current step's
    // pause, while forceStop drains only the already queued short commands.
    bool stepPauses = false, failContinuousStart = false, queueNeverStarts = false, forceStopFails = false;
    uint32_t nextStepAt = 0, appliedPeriodUs = 0, forceDrainUntil = 0;
    unsigned brakingSteps = 0;
};
struct MoveCommand {
    uint8_t stepPin;
    int32_t steps;
    double speed;
    int32_t acceleration;
    uint32_t at, sampleAt;
    double yawAt, pitchAt;
    bool continuous, wasBraking, beforeStoppedSample;
};
static uint32_t now = 0, lastSampleAt = 0;
static uint32_t continuousStartDelayMs = 0;
// Opt-in reproduction of the historical idle forceStop/start latch. Normal
// watchdog recovery deliberately repeats forceStop while draining its queue;
// that does not assert every real driver permanently drops its next run.
static bool latchIdleForceStop = false;
static uint32_t forceStopDrainMs = 0;
// Independent GPIO latch fixture: intentionally not derived from motor sign.
static uint32_t gpioOutput = 0;
static PlantMotor motors[3];
static bool axisInitFails = false, busBInitFails = false;
static bool speedFails = false, accelerationFails = false;
static bool moveRejected = false;
static uint8_t rejectedStepPin = 0;
static double baselineYaw = 70.0, baselinePitch = 24.0;
static double noiseAmplitude = 0, yawDisturbance = 0, pitchDisturbance = 0;
static bool encoderPlantFeedback = false;
static double encoderBaselineDegrees[2] = {37.0, 113.0};
// Synthetic encoder travel magnitudes, independent of controller direction.
// Encoder-backed fixtures explicitly set physicalSign from measured hardware.
static double encoderDegreesPerStep[2] = {0.18, 0.225};
static double encoderAxisDisturbance[2] = {};
static int pendingSerial = -1;
static std::string serialInput;
static int serialWriteSpace = 128;
static unsigned forceStops = 0, gentleStops = 0;
static uint32_t firstForceStopAt = 0;
static std::vector<MoveCommand> commands;
static std::vector<uint8_t> connectedPins, highPins, addressedSensors;
static void (*independentTick)(uint32_t) = nullptr;
static void (*sensorWait)(uint32_t) = nullptr;

inline double actualYaw() { return baselineYaw + motors[0].degrees + yawDisturbance; }
inline double actualPitch() { return baselinePitch + motors[1].degrees + pitchDisturbance; }
inline void integrate(PlantMotor &motor) {
    if (motor.drive == Drive::IDLE) return;
    if (motor.stepPauses && motor.drive != Drive::FINITE) {
        if (static_cast<int32_t>(now - motor.nextStepAt) < 0) return;
        if (motor.drive == Drive::BRAKING) {
            if (motor.brakingSteps) {
                --motor.brakingSteps;
                motor.nextStepAt = now + (motor.appliedPeriodUs + 999) / 1000;
                motor.position += motor.direction;
                if (!motor.frozen) motor.degrees += motor.direction * motor.degreesPerStep *
                    motor.physicalSign * motor.activePhysicalMultiplier;
                return;
            }
            if (!motor.neverStops) {
                motor.drive = Drive::IDLE; motor.velocity = 0;
                motor.stoppedAt = now; motor.needsStoppedSample = true;
            }
            return;
        }
        motor.position += motor.direction;
        if (!motor.frozen) motor.degrees += motor.direction * motor.degreesPerStep *
            motor.physicalSign * motor.activePhysicalMultiplier;
        motor.velocity = motor.direction * motor.speed;
        motor.appliedPeriodUs = static_cast<uint32_t>(1000000.0 / motor.speed);
        motor.nextStepAt = now + (motor.appliedPeriodUs + 999) / 1000;
        return;
    }
    const double remaining = motor.target - motor.position;
    double desired = motor.direction * static_cast<double>(motor.speed);
    if (motor.drive == Drive::BRAKING) desired = 0;
    if (motor.drive == Drive::FINITE) {
        const double stoppingSpeed = sqrt(2.0 * motor.acceleration * fabs(remaining));
        desired = (remaining >= 0 ? 1 : -1) * std::min<double>(motor.speed, stoppingSpeed);
    }
    const double dv = motor.acceleration * 0.001;
    const double previousSpeed = motor.velocity;
    motor.velocity += std::max(-dv, std::min(dv, desired - motor.velocity));
    double travel = 0.0005 * (previousSpeed + motor.velocity);
    bool stopped = motor.drive == Drive::BRAKING && motor.velocity == 0;
    if (motor.drive == Drive::FINITE && fabs(travel) >= fabs(remaining)) {
        travel = remaining; motor.velocity = 0; stopped = true;
    }
    motor.position += travel;
    if (!motor.frozen) motor.degrees += travel * motor.degreesPerStep *
        motor.physicalSign * motor.activePhysicalMultiplier;
    if (stopped && !motor.neverStops) {
        motor.drive = Drive::IDLE; motor.stoppedAt = now; motor.needsStoppedSample = true;
    }
}
inline void advance(uint32_t duration) {
    while (duration--) {
        ++now;
        integrate(motors[0]); integrate(motors[1]); integrate(motors[2]);
        if (independentTick) independentTick(now);
    }
}
}
inline uint32_t millis() { return simulated::now; }
inline uint32_t micros() { return simulated::now * 1000U; }
inline void delay(uint32_t duration) { simulated::advance(duration); }
namespace simulated {
inline void sensorDelay(uint32_t duration) {
    if (sensorWait) sensorWait(duration); else advance(duration);
}
}
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t pin, int value) { if (value != LOW) simulated::highPins.push_back(pin); }
template <typename T> T constrain(T value, T minimum, T maximum)
{ return value < minimum ? minimum : value > maximum ? maximum : value; }
class SerialCapture {
public:
    std::string output;
    void begin(uint32_t) {}
    int available() { return (simulated::pendingSerial >= 0 ? 1 : 0) + static_cast<int>(simulated::serialInput.size()); }
    int read() {
        if (simulated::pendingSerial >= 0) {
            const int value = simulated::pendingSerial; simulated::pendingSerial = -1; return value;
        }
        if (simulated::serialInput.empty()) return -1;
        const unsigned char value = simulated::serialInput.front();
        simulated::serialInput.erase(0, 1);
        return value;
    }
    int availableForWrite() { return simulated::serialWriteSpace; }
    size_t write(const uint8_t *bytes, size_t length) {
        output.append(reinterpret_cast<const char *>(bytes), length); return length;
    }
    void flush() {}
    void print(const char *value) { output += value; }
    void print(char value) { output += value; }
    template <typename T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
    void print(T value, int base = 10) {
        std::ostringstream formatted;
        if (base == HEX) formatted << std::hex << std::uppercase;
        formatted << +value; output += formatted.str();
    }
    void print(double value, int places = 2) {
        std::ostringstream formatted;
        formatted << std::fixed << std::setprecision(places) << value; output += formatted.str();
    }
    void println() { output += '\n'; }
    template <typename T> void println(T value) { print(value); println(); }
    void println(double value, int places) { print(value, places); println(); }
};
static SerialCapture Serial;
