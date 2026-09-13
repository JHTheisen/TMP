#include "../src/pose_math.h"
#include <assert.h>
#include <initializer_list>
#include <stdio.h>
#include <stdlib.h>

using namespace milestone8;
static unsigned checks = 0;
static void check(bool condition) {
    ++checks;
    if (!condition) { fprintf(stderr, "M08 pose math check %u failed\n", checks); exit(1); }
}
static bool near(double actual, double expected, double tolerance = 1e-8) {
    return isfinite(actual) && fabs(actual - expected) <= tolerance;
}

// Independent time-step integration checks the continuous-slew root calculation
// against an ideal frozen BNO response, including braking during acceleration.
// The 0.5-second comparison tolerance allows one additional small correction
// when sample-time rounding places the result across the 0.10-degree deadband.
static double sampledAxisSeconds(double error, bool pitch, double response, double retainedPeak, uint32_t cap) {
    using namespace milestone7;
    double remaining = fabs(error), elapsed = 0;
    if (remaining > brakingThreshold(pitch, retainedPeak) + SLEW_ENTRY_HYSTERESIS_DEG) {
        const double dt = 0.0001, accel = slewAcceleration(pitch);
        double speed = 0, peak = retainedPeak;
        while (remaining > brakingThreshold(pitch, peak)) {
            const double nextSpeed = fmin(static_cast<double>(cap), speed + accel * dt);
            remaining -= (speed + nextSpeed) * 0.5 * dt / response;
            speed = nextSpeed; peak = fmax(peak, speed / response); elapsed += dt;
            assert(elapsed < 1000);
        }
        remaining = fabs(remaining - speed * speed / (2 * accel * response));
        elapsed += speed / accel + 0.1;
    }
    for (unsigned n = 0; remaining > APPROACH_DEADBAND_DEG; ++n) {
        assert(n < MAX_PREDICTED_CORRECTIONS);
        const double pulses = fabs(static_cast<double>(correctionSteps(remaining, true, pitch)));
        const double speed = fmin(static_cast<double>(cap), correctionSpeed(remaining, true));
        elapsed += finiteSeconds(pulses, speed, ACCELERATION) + 0.1;
        remaining = fabs(remaining - pulses / response);
    }
    return elapsed;
}

int main() {
    check(near(finiteSeconds(25, 100, 100), 1)); // triangular: peak 50 Hz
    check(near(finiteSeconds(100, 100, 100), 2)); // triangular/trapezoidal boundary
    check(near(finiteSeconds(300, 100, 100), 4)); // acceleration, cruise, deceleration
    check(near(finiteSeconds(0, 100, 100), 0));
    check(isnan(finiteSeconds(-1, 100, 100)));
    check(isnan(finiteSeconds(1, 0, 100)));
    check(isnan(finiteSeconds(1, 100, -1)));
    check(isnan(finiteSeconds(INFINITY, 100, 100)));
    check(isnan(finiteSeconds(1, NAN, 100)));
    check(near(syncSpeedForDuration(25, 100, 1), 50));
    check(near(syncSpeedForDuration(300, 100, 4), 100));
    check(isnan(syncSpeedForDuration(25, 100, 0.9)));
    check(near(syncSpeedForDuration(0, 100, 0), 0));
    check(isnan(syncSpeedForDuration(10, 0, 1)));
    check(isnan(syncSpeedForDuration(10, 100, NAN)));
    for (double pulses : {1.0, 100.0, 1500.0, 10000.0}) {
        for (double accel : {100.0, 240.0, 1000.0}) {
            const double minimum = 2 * sqrt(pulses / accel);
            for (double ratio : {1.0, 1.2, 2.0, 10.0}) {
                const double duration = minimum * ratio;
                const double speed = syncSpeedForDuration(pulses, accel, duration);
                check(near(finiteSeconds(pulses, speed, accel), duration, duration * 1e-10));
            }
        }
    }
    // Exact stopped-burst case: 0.12 deg yaw -> two pulses -> 0.08 deg,
    // therefore a triangular two-pulse move plus one observation interval.
    check(near(estimateAxisSeconds(0.12, false, 50, 0, 1000), finiteSeconds(2, 44, 240) + 0.1));
    check(near(estimateAxisSeconds(0.05, false, 50, 0, 1000), 0));
    check(isnan(estimateAxisSeconds(20, false, 0, 0, 1000)));
    check(isnan(estimateAxisSeconds(NAN, false, 50, 0, 1000)));
    check(isnan(estimateAxisSeconds(20, false, 50, NAN, 1000)));
    check(isnan(estimateAxisSeconds(20, false, 50, -1, 1000)));
    check(isnan(estimateAxisSeconds(20, false, 50, 0, 1001)));
    check(isnan(estimateAxisSeconds(20, false, 50, 0, 0)));
    check(isnan(estimateAxisSeconds(5, false, 50, 0, 1))); // burst timeout
    check(isnan(estimateAxisSeconds(1, false, 0.5, 0, 1000))); // nonconvergent response
    // With the current 48-pulse pitch gain, cap 9 fits individual bursts but
    // misses cumulative progress at 1200 pulses/degree; cap 10 meets the guard.
    const double slowBurst = finiteSeconds(24, 9, 240);
    check(slowBurst < MAX_PLANNED_BURST_SECONDS);
    check(6 * (slowBurst + 0.1) > 15);
    check(isnan(estimateAxisSeconds(0.5, true, 1200, 0, 9)));
    check(isfinite(estimateAxisSeconds(0.5, true, 1200, 0, 10)));
    const AxisTiming progressLimited = chooseAxisCap(0.5, true, 1200, 0, 64.68);
    check(!progressLimited.achievable && progressLimited.capHz >= 10 && isfinite(progressLimited.seconds));
    check(progressLimited.seconds < 64.68);
    // Once already in tolerance, this unchanged watchdog does not impose a
    // progress deadline; slow legal bursts can finish the tighter hold approach.
    check(isfinite(estimateAxisSeconds(0.4, true, 1200, 0, 6)));
    check(near(slewProgressSeconds(false, 100, 1000), sqrt(30.0 / 1000)));
    check(near(slewProgressSeconds(false, 1150, 100), 1.775));
    check(PROGRESS_DEG * 1150 / 100 < MAX_PLANNED_SLEW_PROGRESS_SECONDS);
    check(slewProgressSeconds(false, 1150, 100) > MAX_PLANNED_SLEW_PROGRESS_SECONDS);
    check(isnan(estimateAxisSeconds(60, false, 1150, 0, 100)));
    check(isnan(slewProgressSeconds(false, 0, 100)));
    check(isnan(slewProgressSeconds(false, 50, 0)));
    for (bool pitch : {false, true}) {
        for (double degreesPerPulse : {0.02, pitch ? 0.01 : 0.025, 0.0075}) {
            const double response = 1 / degreesPerPulse;
            const uint32_t native = milestone7::slewSpeed(pitch);
            for (double error : {0.5, 3.0, 8.0, 20.0, 60.0}) {
                const double fastest = estimateAxisSeconds(error, pitch, response, 0, native);
                check(isfinite(fastest) && fastest > 0);
                check(near(fastest, sampledAxisSeconds(error, pitch, response, 0, native), 0.5));
                check(near(estimateAxisSeconds(-error, pitch, response, 0, native), fastest));
                const AxisTiming atNative = chooseAxisCap(error, pitch, response, 0, fastest);
                check(atNative.achievable && near(atNative.seconds, fastest));
                const AxisTiming slower = chooseAxisCap(error, pitch, response, 0, fastest * 1.25);
                check(slower.achievable && slower.capHz <= native && slower.capHz >= 1);
                check(fabs(slower.seconds - fastest * 1.25) < fmax(0.5, fastest * 0.05));
                check(near(slower.seconds, sampledAxisSeconds(error, pitch, response, 0, slower.capHz), 0.5));
                const AxisTiming tooFast = chooseAxisCap(error, pitch, response, 0, fastest * 0.5);
                check(!tooFast.achievable && tooFast.capHz == native);
                const AxisTiming tooSlow = chooseAxisCap(error, pitch, response, 0, 1e6);
                check(!tooSlow.achievable && isfinite(tooSlow.seconds));
            }
        }
    }
    for (double peak : {5.0, 30.0}) {
        check(near(estimateAxisSeconds(60, false, 50, peak, 1000),
                   sampledAxisSeconds(60, false, 50, peak, 1000), 0.5));
    }
    check(!chooseAxisCap(5, false, 0, 0, 10).achievable);
    check(!chooseAxisCap(5, false, 50, 0, NAN).achievable);
    check(chooseAxisCap(0, false, 50, 0, 10).achievable);
    printf("M08 pose math: %u checks passed\n", checks);
}
