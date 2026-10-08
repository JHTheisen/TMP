#include "../src/control_math.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <initializer_list>
namespace { unsigned checks = 0; void check(bool ok, const char *text, int line) { ++checks; if (!ok) { std::fprintf(stderr, "FAIL line %d: %s\n", line, text); std::exit(EXIT_FAILURE); } } }
#define CHECK(value) check((value), #value, __LINE__)
int main() {
    using namespace milestone7;
    CHECK(shortestDifference(0, 359) == 1);
    CHECK(shortestDifference(0, 1) == -1);
    CHECK(shortestDifference(1, 359) == 2);
    CHECK(shortestDifference(359, 1) == -2);
    CHECK(wrap360(-1) == 359);
    HeadingTracker tracker;
    CHECK(tracker.update(359.8)); CHECK(tracker.update(0.2)); CHECK(fabs(tracker.continuous - 0.4) < 1e-9);
    CHECK(tracker.update(359.7)); CHECK(fabs(tracker.continuous + 0.1) < 1e-9);
    CHECK(!tracker.update(179.7));
    for (int current = 0; current < 360; current += 15) {
        const double error = shortestDifference(0, current);
        CHECK(error >= -180 && error < 180);
        CHECK(fabs(shortestDifference(0, wrap360(current + error))) < 1e-9);
    }
    for (bool pitch : {false, true}) {
        CHECK(correctionSteps(APPROACH_DEADBAND_DEG, true, pitch) == 0);
        CHECK(abs(correctionSteps(3, true, pitch)) <= MAX_BURST_STEPS);
        CHECK(correctionSteps(3, true, pitch) == -correctionSteps(-3, true, pitch));
        CHECK(abs(correctionSteps(3, false, pitch)) <= TRIAL_BURST_STEPS);
        CHECK(brakingThreshold(pitch, 0) == approachThreshold(pitch));
        CHECK(brakingThreshold(pitch, 20) > brakingThreshold(pitch, 10));
        CHECK(brakingThreshold(pitch, 10) == brakingThreshold(pitch, -10));
        CHECK(stepDirection(10, pitch) == -stepDirection(-10, pitch));
    }
    std::printf("PASS: north wrap, independent axis commands and heading continuity (%u checks)\n", checks);
}
