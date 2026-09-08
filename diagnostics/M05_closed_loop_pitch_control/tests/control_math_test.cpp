// Boundary checks for finite command limits and preserved quaternion validity.
#include <Arduino.h>
#include <AS5600.h>
#include <FastAccelStepper.h>
#include "../src/control_math.h"
#include "../src/sensor_support.h"
#include <cstdio>
#include <limits>

namespace {
unsigned checks = 0;
void require(bool condition, const char *expression, int line)
{
    ++checks;
    if (!condition) {
        std::fprintf(stderr, "FAIL line %d: %s\n", line, expression);
        std::exit(EXIT_FAILURE);
    }
}
#define CHECK(condition) require((condition), #condition, __LINE__)
}

int main()
{
    using namespace milestone5;
    CHECK(APPROACH_DEADBAND_DEG < TOLERANCE_DEG);
    CHECK(correctionSteps(APPROACH_DEADBAND_DEG) == 0);
    CHECK(correctionSteps(-APPROACH_DEADBAND_DEG) == 0);
    CHECK(correctionSteps(APPROACH_DEADBAND_DEG + 0.001) > 0);
    CHECK(correctionSteps(-APPROACH_DEADBAND_DEG - 0.001) < 0);
    for (double error : {-1e300, -360.0, -6.0, -3.0, -0.5, 0.0, 0.5, 3.0, 6.0, 360.0, 1e300}) {
        CHECK(abs(correctionSteps(error)) <= MAX_BURST_STEPS);
        CHECK(correctionSpeed(error) >= MIN_SPEED_HZ && correctionSpeed(error) <= MAX_SPEED_HZ);
        CHECK(correctionSteps(error) == -correctionSteps(-error));
    }
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
        CHECK(correctionSteps(invalid) == 0);
        CHECK(correctionSpeed(invalid) == MIN_SPEED_HZ);
    }
    milestone4::EulerAngles euler = {0, 0, 0};
    sh2_RotationVectorWAcc_t quaternion = {0, 0, 0, 0};
    CHECK(!milestone4::quaternionToEuler(quaternion, euler));
    quaternion.real = std::numeric_limits<float>::quiet_NaN();
    CHECK(!milestone4::quaternionToEuler(quaternion, euler));
    const double halfAngle = 30.0 / RAD_TO_DEG / 2;
    quaternion = {static_cast<float>(2 * cos(halfAngle)), 0,
                  static_cast<float>(2 * sin(halfAngle)), 0};
    CHECK(milestone4::quaternionToEuler(quaternion, euler));
    CHECK(fabs(euler.pitch - 30.0) < 0.001);
    std::printf("PASS: math and quaternion boundaries (%u checks)\n", checks);
}
