#pragma once
#include <math.h>
#include <stdint.h>

namespace milestone7 {
// Physically verified mappings; yaw retains small per-run response probes.
constexpr int POSITIVE_STEP_PITCH_SIGN = -1;
#ifndef M07_TRIAL_POSITIVE_STEP_YAW_SIGN
#define M07_TRIAL_POSITIVE_STEP_YAW_SIGN -1
#endif
constexpr int TRIAL_POSITIVE_STEP_YAW_SIGN = M07_TRIAL_POSITIVE_STEP_YAW_SIGN;
constexpr double TOLERANCE_DEG = 0.4;
constexpr double APPROACH_DEADBAND_DEG = 0.10;
constexpr double MAX_BURST_STEPS = 200;
constexpr int32_t TRIAL_BURST_STEPS = 4;
constexpr uint32_t MIN_SPEED_HZ = 40;
constexpr uint32_t MAX_SPEED_HZ = 1000;
constexpr int32_t ACCELERATION = 240;
constexpr double PITCH_PULSES_PER_ERROR_DEG = 12.0;
constexpr double YAW_PULSES_PER_ERROR_DEG = 16.0;
constexpr double DIRECTION_RESPONSE_DEG = 0.3;
constexpr uint32_t YAW_SLEW_SPEED_HZ = 1000, PITCH_SLEW_SPEED_HZ = 600;
constexpr int32_t YAW_SLEW_ACCELERATION = 1000, PITCH_SLEW_ACCELERATION = 600;
constexpr double YAW_APPROACH_DEG = 4.0, PITCH_APPROACH_DEG = 2.0;
constexpr double SLEW_ENTRY_HYSTERESIS_DEG = 2.0;
constexpr double BRAKING_DISTANCE_FACTOR = 1.5, BRAKING_LATENCY_S = 0.25;
constexpr uint32_t VELOCITY_WINDOW_MS = 100, PRECISION_OBSERVE_MS = 100;
constexpr uint32_t SLEW_PROGRESS_TIMEOUT_MS = 2000, BURST_TIMEOUT_MS = 4000;
constexpr uint32_t BRAKING_TIMEOUT_MS = 3000;
constexpr uint8_t BNO_MIN_ACCURACY = 2;
constexpr uint32_t BNO_ACCURACY_GRACE_MS = 1000;

inline uint32_t slewSpeed(bool pitch) { return pitch ? PITCH_SLEW_SPEED_HZ : YAW_SLEW_SPEED_HZ; }
inline int32_t slewAcceleration(bool pitch) { return pitch ? PITCH_SLEW_ACCELERATION : YAW_SLEW_ACCELERATION; }
inline double approachThreshold(bool pitch) { return pitch ? PITCH_APPROACH_DEG : YAW_APPROACH_DEG; }
inline int stepDirection(double error, bool pitch) {
    return (error > 0 ? 1 : -1) * (pitch ? POSITIVE_STEP_PITCH_SIGN : TRIAL_POSITIVE_STEP_YAW_SIGN);
}
inline double brakingThreshold(bool pitch, double peakAngularSpeed) {
    // BNO-derived deg/s; no assumed shaft-to-cradle ratio. Use full configured
    // speed for stopping time, including while still accelerating. The extra
    // latency margin covers measurement/filter age and the FAS command queue.
    const double stoppingTime = static_cast<double>(slewSpeed(pitch)) / slewAcceleration(pitch);
    return approachThreshold(pitch) + fabs(peakAngularSpeed) *
        (0.5 * BRAKING_DISTANCE_FACTOR * stoppingTime + BRAKING_LATENCY_S);
}
struct AccuracyGrace {
    bool low = false;
    uint32_t since = 0, episodes = 0, recoveries = 0, longestMs = 0;
    uint32_t age(uint32_t now) const { return low ? now - since : 0; }
    void observe(uint8_t accuracy, uint32_t now) {
        if (accuracy < BNO_MIN_ACCURACY) {
            if (!low) { low = true; since = now; ++episodes; }
        } else if (low) {
            longestMs = age(now) > longestMs ? age(now) : longestMs;
            low = false; ++recoveries;
        }
    }
    bool expired(uint32_t now) const { return low && age(now) >= BNO_ACCURACY_GRACE_MS; }
};
static_assert(POSITIVE_STEP_PITCH_SIGN == 1 || POSITIVE_STEP_PITCH_SIGN == -1, "Pitch sign must be +1 or -1");
static_assert(TRIAL_POSITIVE_STEP_YAW_SIGN == 1 || TRIAL_POSITIVE_STEP_YAW_SIGN == -1, "Yaw sign must be +1 or -1");

inline double wrap360(double angle) {
    if (!isfinite(angle)) return NAN;
    double result = fmod(angle, 360.0);
    if (result < 0) result += 360.0;
    return result;
}
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
        if (!initialized) { initialized = true; first = previous = wrapped; continuous = 0; return true; }
        const double delta = shortestDifference(wrapped, previous);
        if (fabs(delta) >= 180.0) return false;
        continuous += delta; previous = wrapped; return true;
    }
};
inline int32_t correctionSteps(double error, bool confirmed, bool pitchAxis) {
    if (!isfinite(error) || fabs(error) <= APPROACH_DEADBAND_DEG) return 0;
    const int32_t limit = confirmed ? static_cast<int32_t>(MAX_BURST_STEPS) : TRIAL_BURST_STEPS;
    const double gain = pitchAxis ? PITCH_PULSES_PER_ERROR_DEG : YAW_PULSES_PER_ERROR_DEG;
    int32_t steps = static_cast<int32_t>(fmin(static_cast<double>(limit), ceil(fabs(error) * gain)));
    if (steps < 2) steps = 2;
    const int sign = pitchAxis ? POSITIVE_STEP_PITCH_SIGN : TRIAL_POSITIVE_STEP_YAW_SIGN;
    return (error > 0 ? steps : -steps) * sign;
}
inline uint32_t correctionSpeed(double error, bool confirmed) {
    if (!confirmed || !isfinite(error)) return MIN_SPEED_HZ;
    return static_cast<uint32_t>(fmin(static_cast<double>(MAX_SPEED_HZ), MIN_SPEED_HZ + 40.0 * fabs(error)));
}
struct Statistics {
    uint32_t n = 0; double sum = 0, sumSquares = 0, minimum = 0, maximum = 0;
    void add(double value) { if (!n) minimum = maximum = value; minimum = fmin(minimum, value); maximum = fmax(maximum, value); ++n; sum += value; sumSquares += value * value; }
    double mean() const { return n ? sum / n : 0; }
    double range() const { return n ? maximum - minimum : 0; }
    double sd() const { return n < 2 ? 0 : sqrt(fmax(0.0, (sumSquares - sum * sum / n) / (n - 1))); }
};
} // namespace milestone7
