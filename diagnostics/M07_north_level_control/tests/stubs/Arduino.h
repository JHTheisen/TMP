#pragma once
// Deterministic two-axis decision fixture, not an ESP32 scheduler or validated
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
    double position = 0, velocity = 0, target = 0, degrees = 0;
    double degreesPerStep = 0.02;
    int physicalSign = -1, direction = 1;
    uint32_t speed = 40, stoppedAt = 0;
    int32_t acceleration = 240;
    bool frozen = false, neverStops = false, needsStoppedSample = false;
};
struct MoveCommand {
    uint8_t stepPin;
    int32_t steps;
    uint32_t speed;
    int32_t acceleration;
    uint32_t at, sampleAt;
    double yawAt, pitchAt;
    bool continuous, wasBraking, beforeStoppedSample;
};
static uint32_t now = 0, lastSampleAt = 0;
static PlantMotor motors[2];
static bool axisInitFails = false, busBInitFails = false, bnoInitFails = false;
static bool reportInitFails = false, bnoAckFails = false, speedFails = false, accelerationFails = false;
static bool moveRejected = false;
static uint8_t rejectedStepPin = 0;
static uint32_t bnoPauseStart = 0, bnoPauseEnd = 0, bnoResetAt = 0, blockBnoMs = 0;
static bool bnoResetDelivered = false, resetDuringPoll = false;
static unsigned reenableFailuresRemaining = 0;
static bool invalidQuaternion = false, wrongReportType = false;
static uint8_t accuracy = 3;
static double baselineYaw = 70.0, baselinePitch = 24.0;
static double noiseAmplitude = 0, yawDisturbance = 0, pitchDisturbance = 0;
static bool frozenFeedback = false;
static double heldYaw = 0, heldPitch = 0;
static int pendingSerial = -1;
static int serialWriteSpace = 128;
static unsigned forceStops = 0, gentleStops = 0, bnoReads = 0;
static uint32_t firstForceStopAt = 0;
static std::vector<MoveCommand> commands;
static std::vector<uint8_t> connectedPins, highPins, addressedSensors;
static void (*independentTick)(uint32_t) = nullptr;

inline double actualYaw() { return baselineYaw + motors[0].degrees + yawDisturbance; }
inline double actualPitch() { return baselinePitch + motors[1].degrees + pitchDisturbance; }
inline void integrate(PlantMotor &motor) {
    if (motor.drive == Drive::IDLE) return;
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
    if (!motor.frozen) motor.degrees += travel * motor.degreesPerStep * motor.physicalSign;
    if (stopped && !motor.neverStops) {
        motor.drive = Drive::IDLE; motor.stoppedAt = now; motor.needsStoppedSample = true;
    }
}
inline void advance(uint32_t duration) {
    while (duration--) {
        ++now;
        integrate(motors[0]); integrate(motors[1]);
        if (independentTick) independentTick(now);
    }
}
}
inline uint32_t millis() { return simulated::now; }
inline void delay(uint32_t duration) { simulated::advance(duration); }
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t pin, int value) { if (value != LOW) simulated::highPins.push_back(pin); }
template <typename T> T constrain(T value, T minimum, T maximum)
{ return value < minimum ? minimum : value > maximum ? maximum : value; }
class SerialCapture {
public:
    std::string output;
    void begin(uint32_t) {}
    int available() { return simulated::pendingSerial >= 0 ? 1 : 0; }
    int read() { const int value = simulated::pendingSerial; simulated::pendingSerial = -1; return value; }
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
