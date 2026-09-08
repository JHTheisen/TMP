#pragma once
#include <math.h>
#include <stdint.h>

namespace milestone6 {
// Trial polarity only. Each leg must confirm measured yaw response before
// using normal bursts. Change to -1 ONLY after an operator verifies a mismatch.
// No automatic sign flipping, gearing estimation, or calibration is performed.
#ifndef M06_TRIAL_POSITIVE_STEP_YAW_SIGN
#define M06_TRIAL_POSITIVE_STEP_YAW_SIGN -1
#endif
constexpr int TRIAL_POSITIVE_STEP_YAW_SIGN = M06_TRIAL_POSITIVE_STEP_YAW_SIGN;
constexpr double TARGET_DELTA_DEG = 3.0;
constexpr double TOLERANCE_DEG = 0.4;
constexpr double APPROACH_DEADBAND_DEG = 0.10;
constexpr double RELATIVE_LIMIT_DEG = 6.0;
constexpr double MAX_USABLE_PITCH_DEG = 75.0; // Euler heading becomes ill-conditioned near vertical.
constexpr int32_t MAX_BURST_STEPS = 16;
constexpr int32_t TRIAL_BURST_STEPS = 4;
constexpr uint32_t MIN_SPEED_HZ = 40;
constexpr uint32_t MAX_SPEED_HZ = 120;
constexpr int32_t ACCELERATION = 240;
// Conservative error-to-pulse gain carried over from M05, not a yaw drivetrain
// model. No pulse count or theoretical ratio enters position acceptance/safety.
constexpr double PULSES_PER_ERROR_DEG = 16.0;
constexpr double DIRECTION_RESPONSE_DEG = 0.3;
static_assert(TRIAL_POSITIVE_STEP_YAW_SIGN == 1 || TRIAL_POSITIVE_STEP_YAW_SIGN == -1,
              "Trial yaw polarity must be +1 or -1");

inline double wrap360(double angle) {
    if (!isfinite(angle)) return NAN;
    double result = fmod(angle, 360.0);
    if (result < 0) result += 360.0;
    return result;
}
// Signed target - current in [-180, 180). The exact half-turn is ambiguous
// and is rejected by HeadingTracker; it cannot be a valid +/-6-degree test.
inline double shortestDifference(double target, double current) {
    if (!isfinite(target) || !isfinite(current)) return NAN;
    double result = wrap360(target) - wrap360(current);
    if (result >= 180.0) result -= 360.0;
    if (result < -180.0) result += 360.0;
    return result;
}
struct HeadingTracker {
    bool initialized = false;
    double first = 0, previous = 0, continuous = 0;
    bool update(double heading) {
        if (!isfinite(heading)) return false;
        const double wrapped = wrap360(heading);
        if (!initialized) {
            initialized = true; first = previous = wrapped; continuous = 0;
            return true;
        }
        const double delta = shortestDifference(wrapped, previous);
        if (fabs(delta) >= 180.0) return false;
        continuous += delta;
        previous = wrapped;
        return true;
    }
};
inline int32_t correctionSteps(double error, bool confirmed) {
    if (!isfinite(error) || fabs(error) <= APPROACH_DEADBAND_DEG) return 0;
    const int32_t limit = confirmed ? MAX_BURST_STEPS : TRIAL_BURST_STEPS;
    int32_t steps = static_cast<int32_t>(fmin(static_cast<double>(limit),
        ceil(fabs(error) * PULSES_PER_ERROR_DEG)));
    if (steps < 2) steps = 2;
    return (error > 0 ? steps : -steps) * TRIAL_POSITIVE_STEP_YAW_SIGN;
}
inline uint32_t correctionSpeed(double error, bool confirmed) {
    if (!confirmed || !isfinite(error)) return MIN_SPEED_HZ;
    return static_cast<uint32_t>(fmin(static_cast<double>(MAX_SPEED_HZ),
        MIN_SPEED_HZ + 40.0 * fabs(error)));
}
struct Statistics {
    uint32_t n = 0;
    double sum = 0, sumSquares = 0, minimum = 0, maximum = 0;
    void add(double value) {
        if (!n) minimum = maximum = value;
        minimum = fmin(minimum, value); maximum = fmax(maximum, value);
        ++n; sum += value; sumSquares += value * value;
    }
    double mean() const { return n ? sum / n : 0; }
    double range() const { return n ? maximum - minimum : 0; }
    double sd() const {
        return n < 2 ? 0 : sqrt(fmax(0.0, (sumSquares - sum * sum / n) / (n - 1)));
    }
};
} // namespace milestone6
