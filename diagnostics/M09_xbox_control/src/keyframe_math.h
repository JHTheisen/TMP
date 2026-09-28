#pragma once
#include <stdint.h>
#include <limits.h>
#include <math.h>

namespace m09 { namespace keyframes {
constexpr uint32_t MIN_DURATION_MS = 1000, MAX_DURATION_MS = 60000;
struct AxisPlan {
    int32_t target = 0;
    uint32_t speedMilliHz = 0;
    int32_t acceleration = 0;
    double predictedSeconds = 0, rampSeconds = 0;
    bool moving = false;
};

inline double minimumSeconds(double distance, double speed, double acceleration) {
    return distance <= speed * speed / acceleration ? 2 * sqrt(distance / acceleration) :
        distance / speed + speed / acceleration;
}
inline const char *failureReason(int32_t current, int32_t target, uint32_t ms,
                                 uint32_t speed, int32_t acceleration) {
    const int64_t delta = static_cast<int64_t>(target) - current;
    if (delta < INT32_MIN || delta > INT32_MAX) return "signed displacement overflow";
    if (ms < MIN_DURATION_MS || ms > MAX_DURATION_MS) return "duration outside 1000..60000 ms";
    if (ms / 1000.0 < minimumSeconds(fabs(static_cast<double>(delta)), speed, acceleration))
        return "duration too short for speed/acceleration caps";
    return "step resolution or rounded profile timing; shorten duration for sparse moves";
}

// All axes target one wall-clock duration. Start with a 20% acceleration /
// 60% cruise / 20% deceleration profile. FastAccelStepper accepts integer
// acceleration; solve the cruise speed again after rounding that acceleration.
// Thus small axes can have shorter ramps, but never sequential start delays.
// This is a finite generated-step plan, not measured-angle interpolation.
inline bool planAxis(int32_t current, int32_t target, uint32_t durationMs,
                     uint32_t maxSpeedHz, int32_t maxAcceleration, AxisPlan &plan) {
    plan = {}; plan.target = target;
    if (durationMs < MIN_DURATION_MS || durationMs > MAX_DURATION_MS ||
        !maxSpeedHz || maxAcceleration <= 0) return false;
    const int64_t delta = static_cast<int64_t>(target) - current;
    if (delta < INT32_MIN || delta > INT32_MAX) return false;
    if (!delta) return true;
    const double distance = fabs(static_cast<double>(delta));
    const double seconds = durationMs / 1000.0;
    const double ramp = seconds * 0.2;
    const double acceleration = fmin(maxAcceleration, ceil(distance / (ramp * (seconds - ramp))));
    if (!isfinite(acceleration) || acceleration < 1 || acceleration > maxAcceleration) return false;
    const double discriminant = seconds * seconds - 4 * distance / acceleration;
    if (discriminant < 0) return false;
    // Stable form of the smaller quadratic root for long, slow moves.
    const double speed = 2 * distance / (seconds + sqrt(discriminant));
    const double milliHz = round(speed * 1000);
    if (!isfinite(milliHz) || milliHz < 1 || milliHz > maxSpeedHz * 1000.0) return false;
    const double actualSpeed = milliHz / 1000.0;
    const double predicted = distance / actualSpeed + actualSpeed / acceleration;
    // Very sparse moves cannot visibly progress together on this integer-step
    // mechanism. Reject them instead of silently waiting and jumping at the end.
    if (1 / actualSpeed > fmax(0.25, seconds * 0.05) ||
        fabs(predicted - seconds) > fmax(0.02, seconds * 0.005)) return false;
    plan.moving = true; plan.speedMilliHz = static_cast<uint32_t>(milliHz);
    plan.acceleration = static_cast<int32_t>(acceleration);
    plan.predictedSeconds = predicted; plan.rampSeconds = actualSpeed / acceleration;
    return true;
}
} } // namespace m09::keyframes
