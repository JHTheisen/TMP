#include <Arduino.h>
#include <FastAccelStepper.h>
#include "../src/control_math.h"
#include "../src/sensor_support.h"
#include <cstdio>
#include <limits>
namespace {
unsigned checks = 0;
void require(bool ok, const char *expression, int line) {
    ++checks;
    if (!ok) { std::fprintf(stderr, "FAIL line %d: %s\n", line, expression); std::exit(EXIT_FAILURE); }
}
#define CHECK(condition) require((condition), #condition, __LINE__)
}
int main() {
    using namespace milestone6;
    CHECK(shortestDifference(1, 359) == 2);
    CHECK(shortestDifference(359, 1) == -2);
    CHECK(shortestDifference(360, 0) == 0);
    CHECK(shortestDifference(-1, 1) == -2);
    CHECK(wrap360(-721) == 359);
    CHECK(shortestDifference(180, 0) == -180);
    for (int current = -720; current <= 720; current += 15) {
        for (int target = -720; target <= 720; target += 15) {
            const double difference = shortestDifference(target, current);
            CHECK(difference >= -180 && difference < 180);
            CHECK(fabs(shortestDifference(current + difference, target)) < 1e-10);
            if (fabs(difference) != 180)
                CHECK(difference == -shortestDifference(current, target));
        }
    }
    HeadingTracker tracker;
    CHECK(tracker.update(359.8)); CHECK(tracker.update(0.2));
    CHECK(fabs(tracker.continuous - 0.4) < 1e-9);
    CHECK(tracker.update(359.7)); CHECK(fabs(tracker.continuous + 0.1) < 1e-9);
    CHECK(!tracker.update(179.7));
    CHECK(fabs(tracker.continuous + 0.1) < 1e-9);
    tracker = {};
    for (int i = 0; i <= 800; ++i) CHECK(tracker.update(i % 360));
    CHECK(tracker.continuous == 800); // Continuous travel never aliases a full turn.
    Statistics window;
    tracker = {};
    for (int i = 0; i < 100; ++i) {
        CHECK(tracker.update(i % 2 ? 0.1 : 359.9));
        window.add(tracker.continuous);
    }
    CHECK(fabs(shortestDifference(wrap360(tracker.first + window.mean()), 0)) < 1e-9);
    CHECK(window.range() < 0.201 && window.sd() < 0.102);
    for (bool confirmed : {false, true}) {
        CHECK(correctionSteps(APPROACH_DEADBAND_DEG, confirmed) == 0);
        for (double error : {-1e300, -360.0, -6.0, -3.0, -0.5, 0.0, 0.5, 3.0, 6.0, 360.0, 1e300}) {
            CHECK(abs(correctionSteps(error, confirmed)) <= (confirmed ? MAX_BURST_STEPS : TRIAL_BURST_STEPS));
            CHECK(correctionSpeed(error, confirmed) >= MIN_SPEED_HZ && correctionSpeed(error, confirmed) <= MAX_SPEED_HZ);
            CHECK(correctionSteps(error, confirmed) == -correctionSteps(-error, confirmed));
        }
    }
    for (double invalid : {NAN, INFINITY, -INFINITY}) {
        CHECK(correctionSteps(invalid, true) == 0);
        CHECK(!isfinite(shortestDifference(invalid, 0)));
        CHECK(!tracker.update(invalid));
    }
    milestone4::EulerAngles euler = {0, 0, 0};
    sh2_RotationVectorWAcc_t quaternion = {0, 0, 0, 0};
    CHECK(!milestone4::quaternionToEuler(quaternion, euler));
    quaternion.real = std::numeric_limits<float>::quiet_NaN();
    CHECK(!milestone4::quaternionToEuler(quaternion, euler));
    for (double degrees : {0.0, 0.1, 90.0, 179.9, 180.0, 270.0, 359.9}) {
        const double halfAngle = degrees / RAD_TO_DEG / 2;
        quaternion = {static_cast<float>(2 * cos(halfAngle)), 0, 0, static_cast<float>(2 * sin(halfAngle))};
        CHECK(milestone4::quaternionToEuler(quaternion, euler));
        CHECK(fabs(shortestDifference(euler.heading, degrees)) < 0.001);
    }
    std::printf("PASS: wrap, continuity, command and quaternion math (%u checks)\n", checks);
}
