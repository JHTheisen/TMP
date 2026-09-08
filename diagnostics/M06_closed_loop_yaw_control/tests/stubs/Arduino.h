#pragma once
// Deterministic fixtures adapted from M05. They exercise firmware decisions,
// not ESP32 timing, real motor torque, I2C electrical behavior, or BNO fusion.
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
constexpr int HEX = 16;
constexpr int OUTPUT = 1;
constexpr int LOW = 0;
constexpr float RAD_TO_DEG = 57.29577951308232F;

namespace simulated {
struct MoveCommand {
    uint8_t stepPin; int32_t steps; uint32_t speed; int32_t acceleration;
    uint32_t at; double yawAt;
};
static uint32_t now = 0;
static bool axisInitFails = false, busBInitFails = false, bnoInitFails = false;
static bool reportInitFails = false, bnoAckFails = false, speedFails = false, accelerationFails = false;
static bool moveRejected = false, moveNeverFinishes = false;
static uint32_t bnoPauseStart = 0, bnoPauseEnd = 0, bnoResetAt = 0, blockBnoMs = 0;
static bool bnoResetDelivered = false, resetDuringPoll = false;
static unsigned reenableFailuresRemaining = 0;
static bool invalidQuaternion = false, wrongReportType = false;
static std::vector<MoveCommand> commands;
static int32_t startSteps = 0, targetSteps = 0, lastPlantSteps = 0;
static uint32_t moveStart = 0, moveDuration = 0;
static bool moving = false;
static int yawSign = 1;
static double degreesPerPulse = 0.015; // Synthetic plant gain, independent of firmware.
static double baselineYaw = 123.0, baselinePitch = 5.0, cradleYaw = 0;
static double pulseEfficiency = 1.0, backlashSteps = 0, backlashRemaining = 0;
static int previousDirection = 0;
static double noiseAmplitude = 0, driftDegreesPerSecond = 0, disturbanceDegrees = 0;
static bool frozenMotor = false, frozenFeedback = false;
static int pendingSerial = -1;
static unsigned forceStops = 0, bnoReads = 0;
static std::vector<uint8_t> connectedPins, highPins, addressedSensors;
static double minimumYaw = 1e9, maximumYaw = -1e9;
inline int32_t position() {
    int32_t result = targetSteps;
    if (moving && now - moveStart < moveDuration)
        result = startSteps + static_cast<int32_t>(static_cast<int64_t>(targetSteps - startSteps) *
                                                   (now - moveStart) / moveDuration);
    else if (!moveNeverFinishes) moving = false;
    const int32_t delta = result - lastPlantSteps;
    lastPlantSteps = result;
    if (delta && !frozenMotor) {
        const int direction = delta > 0 ? 1 : -1;
        if (previousDirection && direction != previousDirection) backlashRemaining = backlashSteps;
        previousDirection = direction;
        const double physicalDelta = delta * pulseEfficiency;
        const double takenUp = std::min(backlashRemaining, fabs(physicalDelta));
        backlashRemaining -= takenUp;
        cradleYaw += yawSign * (physicalDelta - direction * takenUp) * degreesPerPulse;
    }
    return result;
}
inline double actualYaw() {
    position();
    const double result = baselineYaw + cradleYaw + disturbanceDegrees +
                          driftDegreesPerSecond * now / 1000.0;
    minimumYaw = std::min(minimumYaw, result); maximumYaw = std::max(maximumYaw, result);
    return result;
}
}
inline uint32_t millis() { return simulated::now; }
inline void delay(uint32_t duration) { simulated::now += duration; simulated::position(); }
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
