#pragma once

// Minimal deterministic host replacements. These verify firmware bookkeeping and
// reporting, not ESP32 timing, I2C behavior, or physical movement.
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>
#include <algorithm>
#include <initializer_list>

using std::isfinite;
constexpr int HEX = 16;
constexpr int OUTPUT = 1;
constexpr int LOW = 0;
constexpr float RAD_TO_DEG = 57.29577951308232F;

namespace simulated {
struct MoveCommand { uint8_t stepPin; int32_t steps; };
static uint32_t now = 0;
static bool axisInitFails = false;
static uint32_t bnoPauseStart = 0;
static uint32_t bnoPauseEnd = 0;
static uint32_t bnoResetAt = 0;
static bool bnoResetDelivered = false;
static unsigned reenableFailuresRemaining = 0;
static uint32_t encoderFailureAt = 0;
static bool encoderFailureDelivered = false;
static std::vector<MoveCommand> commands;
static int32_t startSteps = 0;
static int32_t targetSteps = 0;
static uint32_t moveStart = 0;
static uint32_t moveDuration = 0;
static bool moving = false;
static int encoderSign = 1;
static int pitchSign = 1;
static double fixtureRatio = 15.0;
static bool missingPitchMagnet = false;
static bool encoderOutage = false;
static bool noisyBno = false;
static int pendingSerial = -1;
static std::vector<uint8_t> connectedPins;
static std::vector<uint8_t> highPins;
inline int32_t position()
{
    if (!moving || now - moveStart >= moveDuration) { moving = false; return targetSteps; }
    return startSteps + static_cast<int32_t>(static_cast<int64_t>(targetSteps - startSteps) * (now - moveStart) / moveDuration);
}
}

inline uint32_t millis() { return simulated::now; }
inline void delay(uint32_t duration) { simulated::now += duration; }
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t pin, int value) { if (value != LOW) simulated::highPins.push_back(pin); }
template <typename T> T constrain(T value, T minimum, T maximum)
{
    return value < minimum ? minimum : value > maximum ? maximum : value;
}

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
