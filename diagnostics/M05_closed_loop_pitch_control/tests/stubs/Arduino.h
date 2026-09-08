#pragma once

// Deterministic host fixtures, adapted from M04. They exercise firmware decisions;
// they do not reproduce ESP32 scheduling, I2C latency, motor torque, or BNO fusion.
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
    uint8_t stepPin;
    int32_t steps;
    uint32_t speed;
    int32_t acceleration;
    uint32_t at;
    double pitchAt;
};
static uint32_t now = 0;
static bool axisInitFails = false;
static bool busBInitFails = false;
static bool encoderInitFails = false;
static bool bnoInitFails = false;
static bool reportInitFails = false;
static bool moveRejected = false;
static bool moveNeverFinishes = false;
static uint32_t bnoPauseStart = 0;
static uint32_t bnoPauseEnd = 0;
static uint32_t encoderPauseStart = 0;
static uint32_t encoderPauseEnd = 0;
static uint32_t bnoResetAt = 0;
static bool bnoResetDelivered = false;
static uint32_t blockBnoMs = 0;
static uint32_t blockEncoderMs = 0;
static bool invalidQuaternion = false;
static bool wrongReportType = false;
static bool encoderReadError = false;
static int encoderRawOverride = -1;
static std::vector<MoveCommand> commands;
static int32_t startSteps = 0;
static int32_t targetSteps = 0;
static int32_t lastPlantSteps = 0;
static uint32_t moveStart = 0;
static uint32_t moveDuration = 0;
static bool moving = false;
static int encoderSign = 1;
static int pitchSign = 1;
static double pulsesPerRev = 1600.0;
static double fixtureRatio = 15.0;
static double baselinePitch = 5.0;
static double shaftSteps = 0.0;
static double cradleSteps = 0.0;
static double pulseEfficiency = 1.0;
static double backlashSteps = 0.0;
static double backlashRemaining = 0.0;
static int previousDirection = 0;
static double noiseAmplitude = 0.0;
static double driftDegreesPerSecond = 0.0;
static double disturbanceDegrees = 0.0;
static bool frozenMotor = false;
static bool frozenCradle = false;
static bool frozenEncoder = false;
static bool missingPitchMagnet = false;
static bool encoderOutage = false;
static int pendingSerial = -1;
static unsigned forceStops = 0;
static unsigned bnoReads = 0;
static unsigned encoderReads = 0;
static std::vector<uint8_t> connectedPins;
static std::vector<uint8_t> highPins;
static double minimumPitch = 1e9;
static double maximumPitch = -1e9;

inline int32_t position()
{
    int32_t result = targetSteps;
    if (moving && now - moveStart < moveDuration)
        result = startSteps + static_cast<int32_t>(static_cast<int64_t>(targetSteps - startSteps) *
                                                   (now - moveStart) / moveDuration);
    else if (!moveNeverFinishes) moving = false;
    const int32_t delta = result - lastPlantSteps;
    lastPlantSteps = result;
    if (delta != 0 && !frozenMotor) {
        const int direction = delta > 0 ? 1 : -1;
        if (previousDirection && direction != previousDirection) backlashRemaining = backlashSteps;
        previousDirection = direction;
        const double physicalDelta = delta * pulseEfficiency;
        shaftSteps += physicalDelta;
        const double takenUp = std::min(backlashRemaining, fabs(physicalDelta));
        backlashRemaining -= takenUp;
        if (!frozenCradle) cradleSteps += physicalDelta - direction * takenUp;
    }
    return result;
}

inline double actualPitch()
{
    position();
    const double result = baselinePitch + pitchSign * cradleSteps * 360.0 / pulsesPerRev / fixtureRatio +
                          disturbanceDegrees + driftDegreesPerSecond * now / 1000.0;
    minimumPitch = std::min(minimumPitch, result);
    maximumPitch = std::max(maximumPitch, result);
    return result;
}
inline bool encoderUnavailable()
{
    return encoderOutage || (now >= encoderPauseStart && now < encoderPauseEnd);
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
    void print(T value, int base = 10)
    {
        std::ostringstream formatted;
        if (base == HEX) formatted << std::hex << std::uppercase;
        formatted << +value;
        output += formatted.str();
    }
    void print(double value, int places = 2)
    {
        std::ostringstream formatted;
        formatted << std::fixed << std::setprecision(places) << value;
        output += formatted.str();
    }
    void println() { output += '\n'; }
    template <typename T> void println(T value) { print(value); println(); }
    void println(double value, int places) { print(value, places); println(); }
};
static SerialCapture Serial;
