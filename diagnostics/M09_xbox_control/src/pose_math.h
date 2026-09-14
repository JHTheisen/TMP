#pragma once
#include "control_math.h"

namespace milestone8 {
// Timing estimates use a response learned from BNO motion (pulses/degree),
// never the M07 error-to-correction gains as a mechanical gearing ratio.
// They exclude the shared final settling window and remain approximate:
// sensor latency, backlash and changing load are handled by closed-loop control.
constexpr double MAX_PLANNED_BURST_SECONDS = 3.9;
// Reserve sensor/control allowance inside M07's unchanged 15-second precision
// and 2-second slew progress deadlines. The controller measures cumulative
// 0.15-degree improvement while outside its 0.4-degree settling tolerance.
constexpr double PROGRESS_DEG = 0.15;
constexpr double MAX_PLANNED_PRECISION_PROGRESS_SECONDS = 14.5;
constexpr double MAX_PLANNED_SLEW_PROGRESS_SECONDS = milestone7::SLEW_PROGRESS_TIMEOUT_MS / 1000.0 - 0.25;
constexpr unsigned MAX_PREDICTED_CORRECTIONS = 2048;

inline double finiteSeconds(double pulses, double speed, double accel) {
    if (!isfinite(pulses) || pulses < 0 || !isfinite(speed) || speed <= 0 ||
        !isfinite(accel) || accel <= 0) return NAN;
    if (pulses == 0) return 0;
    const double rampDistance = speed * speed / accel;
    const double seconds = pulses <= rampDistance ? 2 * sqrt(pulses / accel) :
        pulses / speed + speed / accel;
    return isfinite(seconds) ? seconds : NAN;
}

// Inverts the trapezoidal profile; at the shortest possible duration this
// returns the peak speed of its triangular profile. Caller applies motor limits.
inline double syncSpeedForDuration(double pulses, double accel, double duration) {
    if (!isfinite(pulses) || pulses < 0 || !isfinite(accel) || accel <= 0 ||
        !isfinite(duration) || duration < 0) return NAN;
    if (pulses == 0) return 0;
    const double minimum = 2 * sqrt(pulses / accel);
    if (duration < minimum) return NAN;
    const double discriminant = fmax(0.0, duration * duration - minimum * minimum);
    const double result = 2 * pulses / (duration + sqrt(discriminant));
    return isfinite(result) && result > 0 ? result : NAN;
}

inline double slewProgressSeconds(bool pitch, double pulsesPerDegree, uint32_t cap) {
    if (!isfinite(pulsesPerDegree) || pulsesPerDegree <= 0 || cap < 1 || cap > milestone7::slewSpeed(pitch)) return NAN;
    const double accel = milestone7::slewAcceleration(pitch);
    const double pulses = PROGRESS_DEG * pulsesPerDegree;
    const double rampPulses = static_cast<double>(cap) * cap / (2 * accel);
    // Continuous stepping accelerates from rest, without a terminal deceleration.
    const double seconds = pulses <= rampPulses ? sqrt(2 * pulses / accel) :
        pulses / cap + cap / (2 * accel);
    return isfinite(seconds) ? seconds : NAN;
}

inline double estimateAxisSeconds(double error, bool pitch, double pulsesPerDegree,
                                  double peakAngularSpeed, uint32_t cap) {
    using namespace milestone7;
    if (!isfinite(error) || !isfinite(pulsesPerDegree) || pulsesPerDegree <= 0 ||
        !isfinite(peakAngularSpeed) || peakAngularSpeed < 0 || cap < 1 || cap > slewSpeed(pitch)) return NAN;
    double remaining = fabs(error), seconds = 0;
    double progressError = remaining, progressAge = 0;
    if (remaining <= APPROACH_DEADBAND_DEG) return 0;
    const double observe = PRECISION_OBSERVE_MS / 1000.0;
    if (remaining > brakingThreshold(pitch, peakAngularSpeed) + SLEW_ENTRY_HYSTERESIS_DEG) {
        const double progressSeconds = slewProgressSeconds(pitch, pulsesPerDegree, cap);
        if (!isfinite(progressSeconds) || progressSeconds >= MAX_PLANNED_SLEW_PROGRESS_SECONDS) return NAN;
        const double accel = slewAcceleration(pitch), angularAccel = accel / pulsesPerDegree;
        const double rampSeconds = cap / accel;
        const double brakeFactor = 0.5 * BRAKING_DISTANCE_FACTOR * slewSpeed(pitch) / accel + BRAKING_LATENCY_S;
        const double approach = approachThreshold(pitch);
        const double rampDegrees = 0.5 * angularAccel * rampSeconds * rampSeconds;
        const double fullSpeedBrake = brakingThreshold(pitch, fmax(peakAngularSpeed, cap / pulsesPerDegree));
        double triggerSeconds, triggerDegrees, triggerSpeed;
        if (remaining > rampDegrees + fullSpeedBrake) {
            triggerDegrees = remaining - fullSpeedBrake;
            triggerSeconds = rampSeconds + (triggerDegrees - rampDegrees) * pulsesPerDegree / cap;
            triggerSpeed = cap;
        } else {
            // The stop condition during acceleration is displacement + approach
            // + brakeFactor * max(retainedPeak, currentSpeed) >= initial error.
            const double retainedTime = sqrt(2 * (remaining - approach - brakeFactor * peakAngularSpeed) / angularAccel);
            const double linear = brakeFactor * angularAccel;
            const double growingTime = 2 * (remaining - approach) /
                (linear + sqrt(linear * linear + 2 * angularAccel * (remaining - approach)));
            triggerSeconds = fmin(rampSeconds, fmin(retainedTime, growingTime));
            triggerSpeed = accel * triggerSeconds;
            triggerDegrees = 0.5 * angularAccel * triggerSeconds * triggerSeconds;
        }
        remaining = fabs(remaining - triggerDegrees - triggerSpeed * triggerSpeed / (2 * accel * pulsesPerDegree));
        seconds = triggerSeconds + triggerSpeed / accel + observe;
        if (!isfinite(remaining) || !isfinite(seconds) || seconds < 0) return NAN;
        // The last measured progress may precede braking. Carry an upper bound
        // on that age into precision, and require a full new 0.15-degree gain
        // from the stopped position instead of assuming a reset at that point.
        progressError = remaining;
        progressAge = remaining <= TOLERANCE_DEG ? 0 :
            progressSeconds + triggerSpeed / accel + observe;
    }
    for (unsigned correction = 0; remaining > APPROACH_DEADBAND_DEG; ++correction) {
        if (correction >= MAX_PREDICTED_CORRECTIONS) return NAN;
        const double pulses = fabs(static_cast<double>(correctionSteps(remaining, true, pitch)));
        const double speed = fmin(static_cast<double>(cap), correctionSpeed(remaining, true));
        const double burst = finiteSeconds(pulses, speed, ACCELERATION);
        if (!isfinite(burst) || burst >= MAX_PLANNED_BURST_SECONDS) return NAN;
        seconds += burst + observe;
        progressAge += burst + observe;
        // Check before endpoint resets: even a burst that eventually reaches
        // tolerance or a new progress marker must not span the safety deadline.
        if (remaining > TOLERANCE_DEG && progressAge >= MAX_PLANNED_PRECISION_PROGRESS_SECONDS) return NAN;
        const double next = fabs(remaining - pulses / pulsesPerDegree);
        // A noncontracting correction cannot support a reliable timing plan.
        if (!isfinite(next) || next >= remaining || !isfinite(seconds)) return NAN;
        if (next <= progressError - PROGRESS_DEG) { progressError = next; progressAge = 0; }
        if (next <= TOLERANCE_DEG) progressAge = 0;
        remaining = next;
    }
    return seconds;
}

struct AxisTiming {
    bool achievable;
    uint32_t capHz;
    double seconds;
};

inline AxisTiming chooseAxisCap(double error, bool pitch, double pulsesPerDegree,
                                double peakAngularSpeed, double targetSeconds) {
    const uint32_t native = milestone7::slewSpeed(pitch);
    const double fastest = estimateAxisSeconds(error, pitch, pulsesPerDegree, peakAngularSpeed, native);
    AxisTiming result = {false, native, fastest};
    if (!isfinite(targetSeconds) || targetSeconds < 0 || !isfinite(fastest)) return result;
    if (fastest == 0) { result.achievable = true; return result; }
    if (targetSeconds < fastest - 1e-6) return result;
    // Find the lowest legal cap: slow motion must still meet M07's unchanged
    // burst and measured-progress deadlines.
    uint32_t low = 1, high = native;
    while (low < high) {
        const uint32_t middle = low + (high - low) / 2;
        if (isfinite(estimateAxisSeconds(error, pitch, pulsesPerDegree, peakAngularSpeed, middle))) high = middle;
        else low = middle + 1;
    }
    const double slowest = estimateAxisSeconds(error, pitch, pulsesPerDegree, peakAngularSpeed, low);
    if (targetSeconds > slowest + 1e-6) return {false, low, slowest};
    high = native;
    // Integer-Hz limits and correction-step rounding prevent exact equality.
    // Retain the closest valid duration encountered by the bounded search.
    result.achievable = true;
    while (low <= high) {
        const uint32_t middle = low + (high - low) / 2;
        const double predicted = estimateAxisSeconds(error, pitch, pulsesPerDegree, peakAngularSpeed, middle);
        if (!isfinite(predicted)) { low = middle + 1; continue; }
        if (fabs(predicted - targetSeconds) < fabs(result.seconds - targetSeconds)) result = {true, middle, predicted};
        if (predicted > targetSeconds) low = middle + 1;
        else { if (middle == 0) break; high = middle - 1; }
    }
    return result;
}
} // namespace milestone8
