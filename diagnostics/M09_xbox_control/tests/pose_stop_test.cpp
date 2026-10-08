// Exercise the real POSE/STOP dispatch while the motor fixture advances
// independently. Sensor stalls yield CPU time to the actual foreground loop.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
void require(bool ok, const char *expression, int lineNumber) {
    if (!ok) {
        std::fprintf(stderr, "FAIL POSE STOP line %d: %s\n%s", lineNumber, expression, Serial.output.c_str());
        std::exit(1);
    }
}
#define CHECK(value) require((value), #value, __LINE__)
uint32_t injectStopAt = 0, handledStopAt = 0;
bool stopInjected = false;
void tick(uint32_t now) {
    motionWatchdog.check(now); commandWatchdog.check(now);
    if (injectStopAt && !stopInjected && now >= injectStopAt) {
        stopInjected = true; simulated::serialInput += "STOP\n";
    }
    if (stopInjected && poseStopping && !handledStopAt) handledStopAt = now;
}
void advance(uint32_t duration) {
    const uint32_t start = millis();
    while (millis() - start < duration) loop();
}
void line(const std::string &value) { simulated::serialInput += value + "\n"; advance(1); }
void assertStoppedAndNoRestart(size_t commandCount) {
    advance(BRAKING_TIMEOUT_MS + 200);
    CHECK(commandIdle() && manualReady() && !poseActive && !poseStopping);
    CHECK(poseMotorsStopped());
    CHECK(simulated::commands.size() == commandCount);
    CHECK(Serial.output.find("POSE CANCELLED: operator STOP; target not completed") != std::string::npos);
    CHECK(Serial.output.find("FINAL RESULT: PASS") == std::string::npos);
    const int32_t yaw = yawMotor->getCurrentPosition(), pitch = pitchMotor->getCurrentPosition();
    const int32_t carriage = carriageMotor->getCurrentPosition();
    advance(300);
    CHECK(yawMotor->getCurrentPosition() == yaw && pitchMotor->getCurrentPosition() == pitch);
    CHECK(carriageMotor->getCurrentPosition() == carriage && simulated::commands.size() == commandCount);
    // Cancellation is recoverable through the same centered manual handshake.
    line("JOG 0 0 0"); CHECK(manualActive);
    line("JOG 100 100 100");
    CHECK(yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
    line("STOP"); advance(2000);
    CHECK(commandIdle() && manualReady() && poseMotorsStopped());
}
}

int main(int argc, char **argv) {
    CHECK(argc == 2);
    const std::string scenario = argv[1];
    simulated::independentTick = tick;
    simulated::baselineYaw = 20; simulated::baselinePitch = 8;
    Wire.encoder.present = Wire1.encoder.present = true;
    simulated::encoderPlantFeedback = true;
    setup(); advance(200);
    simulated::motors[0].physicalSign = simulated::motors[1].physicalSign = 1;
    encoderReferences[0].configure(360 * simulated::motors[0].degreesPerStep / .18);
    encoderReferences[1].configure(360 * simulated::motors[1].degreesPerStep / .225);
    CHECK(encoderReferences[0].setZero(sensorSnapshot.positions[0], millis()));
    CHECK(encoderReferences[1].setZero(sensorSnapshot.positions[1], millis()));
    serviceEncoders();
    CHECK(commandIdle() && manualReady());
    // STOP coverage is independent of first-run planner learning. These values
    // describe only the deterministic plant used by this offline fixture.
    yawPulsesPerDegree = 1 / simulated::motors[0].degreesPerStep;
    pitchPulsesPerDegree = 1 / simulated::motors[1].degreesPerStep;
    if (scenario == "pending") {
        beginPose(0, 0, 10000, true);
        CHECK(poseActive && carriagePending && simulated::commands.empty());
        line("STOP");
        CHECK(!carriagePending && simulated::commands.empty());
        assertStoppedAndNoRestart(0);
    } else {
        if (scenario == "carriage") line("MOVE 0 0 10000");
        else line("MOVE 3 2 10000");
        advance(200);
        CHECK(poseActive && carriageMotor->isRunning());
        if (scenario != "carriage") CHECK(yawMotor->isRunning() && pitchMotor->isRunning());
        const size_t commandCount = simulated::commands.size();
        const unsigned gentleStopsBefore = simulated::gentleStops;
        if (scenario == "stall") {
            injectStopAt = millis() + 50;
            Wire.encoder.requestDelayMs = 1000;
            advance(1100);
            CHECK(stopInjected && handledStopAt && handledStopAt - injectStopAt <= 20);
            Wire.encoder.requestDelayMs = 0;
        } else {
            if (scenario == "timeout") {
                for (auto &motor : simulated::motors) motor.neverStops = true;
            }
            line("STOP");
            CHECK(poseStopping && poseActive && !commandIdle() && !carriagePending);
            CHECK(!motionWatchdog.armed());
            CHECK(carriageMotor->plant().drive == simulated::Drive::BRAKING);
            if (scenario != "carriage") {
                CHECK(yawMotor->plant().drive == simulated::Drive::BRAKING);
                CHECK(pitchMotor->plant().drive == simulated::Drive::BRAKING);
            }
            if (scenario == "repeat" || scenario == "timeout") {
                const uint32_t originalStopAt = poseStopStartedAt;
                const unsigned stops = simulated::gentleStops;
                advance(50); line("STOP");
                CHECK(poseStopStartedAt == originalStopAt && simulated::gentleStops == stops);
                line("MOVE 0 0 100");
                CHECK(poseStopping && simulated::commands.size() == commandCount);
                if (scenario == "timeout") {
                    const uint32_t elapsed = millis() - originalStopAt;
                    CHECK(elapsed < BRAKING_TIMEOUT_MS);
                    advance(BRAKING_TIMEOUT_MS - elapsed - 1);
                    CHECK(poseStopping && !commandIdle());
                    advance(100);
                    CHECK(commandIdle() && poseMotorsStopped());
                    CHECK(Serial.output.find("braking timeout; motors forced stopped") != std::string::npos);
                    for (auto &motor : simulated::motors) motor.neverStops = false;
                }
            } else if (scenario == "abort") {
                line("X"); advance(100);
                CHECK(phase == Phase::ABORTED && !poseActive && !poseStopping && poseMotorsStopped());
                line("JOG 0 0 0"); CHECK(!manualActive && phase == Phase::ABORTED);
                CHECK(simulated::commands.size() == commandCount);
                std::printf("PASS POSE STOP: abort remains latched during braking\n");
                return 0;
            } else CHECK(scenario == "angular" || scenario == "carriage");
        }
        CHECK(simulated::gentleStops == gentleStopsBefore + (scenario == "carriage" ? 1U : 3U));
        assertStoppedAndNoRestart(commandCount);
    }
    std::printf("PASS POSE STOP: %s\n", scenario.c_str());
}
