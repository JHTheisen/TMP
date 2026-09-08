#pragma once
#include <math.h>
#include <stdint.h>

namespace milestone5 {
// User confirmed positive commands increase BNO pitch on this installation.
// Change ONLY this sign if positive FastAccelStepper commands lower BNO pitch.
// This is an installation setting, not learned/calibrated by the diagnostic.
constexpr int POSITIVE_STEP_PITCH_SIGN = 1;
constexpr double TARGET_DELTA_DEG = 3.0;
constexpr double TOLERANCE_DEG = 0.4;
// Approach further inside the acceptance band to leave room for sensor noise.
constexpr double APPROACH_DEADBAND_DEG = 0.10;
constexpr double RELATIVE_LIMIT_DEG = 6.0;
constexpr double ABSOLUTE_LIMIT_DEG = 75.0;
constexpr int32_t MAX_BURST_STEPS = 16;
constexpr uint32_t MIN_SPEED_HZ = 40;
constexpr uint32_t MAX_SPEED_HZ = 120;
constexpr int32_t ACCELERATION = 240;
// Approximate feed-forward sizing ONLY. Neither position nor safety uses this
// estimate, and returning to an old step count is never an objective.
constexpr double APPROX_PULSES_PER_REV = 1600.0;
constexpr double APPROX_REDUCTION = 15.0;
constexpr double INCREMENT_GAIN = 0.25;
static_assert(POSITIVE_STEP_PITCH_SIGN == 1 || POSITIVE_STEP_PITCH_SIGN == -1,
              "Pitch direction must be +1 or -1");

inline int32_t correctionSteps(double error) {
    if (!isfinite(error) || fabs(error) <= APPROACH_DEADBAND_DEG) return 0;
    int32_t steps = static_cast<int32_t>(fmin(static_cast<double>(MAX_BURST_STEPS),
        ceil(fabs(error) * APPROX_PULSES_PER_REV * APPROX_REDUCTION / 360.0 * INCREMENT_GAIN)));
    if (steps < 2) steps = 2;
    return (error > 0 ? steps : -steps) * POSITIVE_STEP_PITCH_SIGN;
}
inline uint32_t correctionSpeed(double error) {
    if (!isfinite(error)) return MIN_SPEED_HZ;
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
} // namespace milestone5
