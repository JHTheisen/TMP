// Independent hardware-sign fixture: October 28 negative FAS yaw increases
// raw yaw; pitch retains October 8 positive-FAS/increasing-raw behavior.
// Never derive this plant from production control signs.
#include "../src/main.cpp"
#include <cassert>
#include <cstdio>
#include <string>

#undef assert
#define assert(ok) do { if (!(ok)) { std::fprintf(stderr, "FAIL %d: %s yaw=%f pitch=%f\n", __LINE__, #ok, heading.continuous, physicalPitch()); std::exit(1); } } while (0)

void advanceDirection(uint32_t ms) {
    const uint32_t start = millis();
    while (millis() - start < ms) {
        if (celestialActive() && millis() - celestialUpdatedAt >= 1000)
            updateCelestial(celestialId, celestialSequence + 1,
                            wrap360(yawAxis.target), pitchAxis.target);
        loop();
    }
}
int main(int argc, char **argv) {
    assert(argc == 2);
    const std::string scenario = argv[1];
    const bool mounted = scenario.find("mount_") == 0;
    const bool pitch = scenario.find("pitch") != std::string::npos;
    const bool negative = scenario.find("negative") != std::string::npos;
    const bool precision = scenario.find("precision") != std::string::npos;
    const bool inverted = scenario.find("inverted") != std::string::npos;
    const bool track = scenario.find("track") != std::string::npos;
    const bool reversed = scenario == "wrong_wiring";
    simulated::independentTick = [](uint32_t now) { motionWatchdog.check(now); commandWatchdog.check(now); };
    simulated::encoderPlantFeedback = true;
    Wire.encoder.present = Wire1.encoder.present = true;
    setup();
    // Fixture speed magnitudes only. Sensor geometry is independently 1:1 yaw
    // and 15 shaft turns per pitch cradle turn; yaw's motor reduction is absent.
    for (unsigned n = 0; n < 2; ++n) {
        auto &motor = simulated::motors[n];
        motor.physicalSign = (n == 0 || mounted) ? -1 : 1;
        motor.degreesPerStep = n ? 1.0/67.2 : 1.0/52.0;
        simulated::encoderDegreesPerStep[n] = motor.degreesPerStep * (n ? (mounted ? -15.0 : 15.0) : 1.0);
    }
    advanceDirection(200);
    encoderReferences[0].configure(inverted ? -360 : 360);
    encoderReferences[1].configure((inverted || mounted) ? -24 : 24);
    assert(encoderReferences[0].setZero(sensorSnapshot.positions[0], millis()));
    assert(encoderReferences[1].setZero(sensorSnapshot.positions[1], millis()));
    serviceEncoders();
    if (scenario == "geometry" || scenario == "mount_geometry") {
        // Independent physical displacements: direct yaw must not be divided
        // by the 9:1 MOTOR gearbox; pitch requires shaft unwrapping through 450°.
        for (unsigned n = 1; n <= 45; ++n) {
            simulated::encoderAxisDisturbance[0] = n;
            simulated::encoderAxisDisturbance[1] = n * (30.0 / 45.0);
            advanceDirection(25);
        }
        assert(fabs(heading.continuous - 45) < .1);
        assert(fabs(physicalPitch() - 30) < .01);
        std::puts("PASS encoder direction: geometry");
        return 0;
    }
    if (scenario == "mount_level") {
        for (unsigned n=1; n<=5; ++n) {
            simulated::encoderAxisDisturbance[1] = n;
            advanceDirection(30);
        }
        assert(fabs(physicalPitch() - 5) < .02);
        beginOrientation(true);
        advanceDirection(500);
        assert(physicalPitch() < 5 && pitchMotor->getCurrentPosition() > 0);
        for (unsigned n=0; n<600 && !commandIdle(); ++n) advanceDirection(100);
        assert(commandIdle() && fabs(physicalPitch()) <= TOLERANCE_DEG);
        std::puts("PASS encoder direction: mount_level");
        return 0;
    }
    if (scenario == "mount_reference_limits") {
        for (unsigned n=1; n<=45; ++n) {
            simulated::encoderAxisDisturbance[1] = n * 2;
            advanceDirection(25);
        }
        assert(fabs(physicalPitch() - 90) < .02);
        beginCelestial(91, 0, 10);
        assert(!celestialActive()); // Absolute physical pitch guard still applies.
        simulated::serialInput += "SET_LEVEL\n";
        advanceDirection(100);
        assert(fabs(physicalPitch()) < .02);
        simulated::encoderAxisDisturbance[1] = 89;
        advanceDirection(100);
        assert(fabs(physicalPitch() + 1) < .02);
        std::puts("PASS encoder direction: mount_reference_limits");
        return 0;
    }
    const int wanted = negative ? -1 : 1;
    const double target = wanted * (precision ? 1.5 : 25.0);
    const bool logGoto = scenario == "log_goto";
    beginCelestial(91, logGoto ? 126.831845 : (track ? 0 : (pitch ? 0 : wrap360(target))),
                   logGoto ? 24.489174 : (track ? 0 : (pitch ? target : 0)));
    assert(celestialActive());
    if (reversed) simulated::motors[0].physicalSign = 1;
    if (track) {
        advanceDirection(1300);
        assert(celestialTracking);
        // Fixed target, displaced measured axis: isolate residual feedback sign.
        simulated::encoderAxisDisturbance[pitch ? 1 : 0] = -wanted * (inverted ? -1 : 1);
        advanceDirection(500);
        const double initial = fabs(pitch ? pitchAxis.error : yawAxis.error);
        advanceDirection(30000);
        assert(celestialTracking && !poseStopping);
        assert(fabs(pitch ? pitchAxis.error : yawAxis.error) < initial - .1);
    } else if (reversed) {
        advanceDirection(6000);
        assert(!celestialActive() || poseStopping);
        assert(Serial.output.find("progress") != std::string::npos || Serial.output.find("runaway") != std::string::npos);
    } else if (logGoto) {
        advanceDirection(600);
        assert(heading.continuous > .1 && physicalPitch() > .1);
        assert(yawAxis.error < 126.831845 && pitchAxis.error < 24.489174);
        for (unsigned n = 0; n < 900 && !celestialTracking && !poseStopping; ++n) advanceDirection(100);
        assert(celestialTracking && !poseStopping);
        assert(fabs(yawAxis.error) <= TOLERANCE_DEG && fabs(pitchAxis.error) <= TOLERANCE_DEG);
    } else {
        advanceDirection(600);
        const double measured = pitch ? physicalPitch() : heading.continuous;
        assert(measured * wanted > .1);
        assert(fabs(target - measured) < fabs(target) - .1);
        assert(!poseStopping && celestialActive());
        const auto &first = simulated::commands.front();
        assert(first.stepPin == (pitch ? 12 : 33));
        assert((first.steps > 0 ? 1 : -1) == wanted * (pitch ? 1 : -1) *
               ((inverted || (mounted && pitch)) ? -1 : 1));
        assert(first.continuous == !precision);
        for (unsigned n = 0; n < 900 && !celestialTracking && !poseStopping; ++n) advanceDirection(100);
        assert(celestialTracking && !poseStopping);
        assert(fabs(yawAxis.error) <= TOLERANCE_DEG && fabs(pitchAxis.error) <= TOLERANCE_DEG);
    }
    if (mounted) {
        assert(fabs(physicalPitch() - (simulated::motors[1].degrees +
               simulated::encoderAxisDisturbance[1])) < .03);
        assert(axisFeedbackStepSign(pitchAxis) == -1);
        assert(axisFeedbackStepSign(yawAxis) == -1);
    }
    std::printf("PASS encoder direction: %s\n", scenario.c_str());
}
