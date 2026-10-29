// Current mount, independent of production signs: negative FAS yaw steps
// increase raw/physical yaw (2026-10-28 log and operator clockwise check).
#include "../src/main.cpp"
#include <cassert>
#include <cstdio>
#include <string>

#undef assert
#define assert(ok) do { if (!(ok)) { std::fprintf(stderr, "FAIL %d: %s yaw=%f error=%f\n", __LINE__, #ok, heading.continuous, yawAxis.error); std::exit(1); } } while (0)

void advanceYaw(uint32_t ms) {
    const uint32_t start = millis();
    while (millis() - start < ms) {
        if (celestialActive() && !poseStopping && millis() - celestialUpdatedAt >= 1000) {
            char update[120];
            snprintf(update, sizeof(update), "CELESTIAL_UPDATE %lu %lu %.8f %.8f\n",
                static_cast<unsigned long>(celestialId), static_cast<unsigned long>(celestialSequence + 1),
                wrap360(yawAxis.target), pitchAxis.target);
            simulated::serialInput += update;
        }
        loop();
    }
}
void sendYaw(const std::string &line) {
    simulated::serialInput += line + "\n";
    advanceYaw(20);
}
int main(int argc, char **argv) {
    assert(argc == 2);
    const std::string scenario = argv[1];
    simulated::independentTick = [](uint32_t now) { motionWatchdog.check(now); commandWatchdog.check(now); };
    simulated::encoderPlantFeedback = true;
    // Start near raw zero so positive and negative movements exercise wrapping.
    simulated::encoderBaselineDegrees[0] = 0;
    Wire.encoder.present = Wire1.encoder.present = true;
    setup();
    simulated::motors[0].physicalSign = -1;
    simulated::motors[0].degreesPerStep = 1.0 / 52.0;
    simulated::encoderDegreesPerStep[0] = 1.0 / 52.0;
    simulated::motors[1].physicalSign = -1;
    simulated::motors[1].degreesPerStep = 1.0 / 67.2;
    simulated::encoderDegreesPerStep[1] = -15.0 / 67.2;
    advanceYaw(200);
    sendYaw("ENCODER_CONFIG 360 -24");
    sendYaw("SET_NORTH"); sendYaw("SET_LEVEL");
    assert(fresh(millis()) && commandIdle());

    if (scenario == "manual_positive" || scenario == "manual_negative") {
        const int sign = scenario == "manual_positive" ? 1 : -1;
        sendYaw("JOG 0 0 0");
        for (unsigned n = 0; n < 30; ++n)
            sendYaw(sign > 0 ? "JOG 100 0 0" : "JOG -100 0 0");
        assert(heading.continuous * sign > .5);
        assert(yawMotor->getCurrentPosition() * sign < 0);
        assert(fabs(physicalPitch()) < .02);
        sendYaw("STOP"); advanceYaw(2000);
        assert(commandIdle() && poseMotorsStopped());
    } else {
        double start = 0, target = 90, delta = 90, pitchTarget = 0;
        if (scenario == "negative") { target = 270; delta = -90; }
        else if (scenario == "wrap_positive") { start = -2; target = 2; delta = 4; }
        else if (scenario == "wrap_negative") { start = 2; target = 358; delta = -4; }
        else if (scenario == "saturn") { target = 265.112481; delta = -94.887519; pitchTarget = 6.912045; }
        else assert(scenario == "positive" || scenario == "stall" || scenario == "wrong_direction" || scenario == "abort");
        simulated::encoderAxisDisturbance[0] = start;
        advanceYaw(100);
        const double initialYaw = heading.continuous;
        if (scenario == "stall") simulated::motors[0].frozen = true;
        if (scenario == "wrong_direction") simulated::motors[0].physicalSign = 1;
        char command[100];
        snprintf(command, sizeof(command), "CELESTIAL_GOTO 3 %.8f %.8f", target, pitchTarget);
        sendYaw(command);
        assert(celestialActive());
        assert(fabs(yawAxis.error - delta) < .15);
        const double initialError = fabs(yawAxis.error);
        if (scenario == "abort") {
            advanceYaw(200); sendYaw("X"); advanceYaw(100);
            assert(phase == Phase::ABORTED && poseMotorsStopped() && !celestialActive());
        } else if (scenario == "stall" || scenario == "wrong_direction") {
            advanceYaw(6000);
            assert(commandIdle() && poseMotorsStopped() && !celestialActive());
            assert(Serial.output.find("progress") != std::string::npos ||
                   Serial.output.find("wrong-direction/runaway") != std::string::npos);
            assert(Serial.output.find("POSE STOPPING:") != std::string::npos);
        } else {
            advanceYaw(600);
            assert((heading.continuous - initialYaw) * delta > 0);
            assert(fabs(yawAxis.error) < initialError - .1);
            assert(!simulated::commands.empty());
            assert(simulated::commands.front().steps * delta < 0);
            if (scenario == "saturn") assert(physicalPitch() > .1 && pitchAxis.error < pitchTarget);
            bool braked = false;
            for (unsigned n = 0; n < 900 && !celestialTracking && !poseStopping; ++n) {
                advanceYaw(100);
                braked = braked || yawAxis.motion == Motion::BRAKING;
            }
            assert(celestialTracking && !poseStopping && !celestialFailed);
            assert(fabs(yawAxis.error) <= TOLERANCE_DEG && fabs(pitchAxis.error) <= TOLERANCE_DEG);
            assert(fabs(heading.continuous - (initialYaw + delta)) < .5);
            assert(settleSamples > 0 && millis() - settleStarted >= SETTLE_MS);
            if (fabs(delta) > 6) assert(braked && simulated::gentleStops > 0);
            assert(poseMotorsStopped());
            sendYaw("STOP"); advanceYaw(2000);
            assert(commandIdle() && poseMotorsStopped());
        }
    }
    std::printf("PASS yaw direction: %s\n", scenario.c_str());
}
