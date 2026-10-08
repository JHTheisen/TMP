// Real M09 setup/loop decisions with synthetic devices and motor mechanics.
// These tests establish software isolation, not hardware calibration or timing.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
unsigned checks = 0;
void require(bool ok, const char *expression, int line) {
    ++checks;
    if (!ok) {
        std::fprintf(stderr, "FAIL lifecycle line %d: %s\n%s", line, expression, Serial.output.c_str());
        std::exit(1);
    }
}
#define CHECK(value) require((value), #value, __LINE__)
void independentTick(uint32_t now) {
    motionWatchdog.check(now);
    commandWatchdog.check(now);
}
void advance(uint32_t duration) {
    const uint32_t start = millis();
    while (static_cast<uint32_t>(millis() - start) < duration) loop();
}
void line(const std::string &value) {
    simulated::serialInput += value + "\n";
    advance(10);
    CHECK(simulated::serialInput.empty());
}
void checkStopped() {
    CHECK(!yawMotor->isRunning() && !pitchMotor->isRunning() && !carriageMotor->isRunning());
}
void reject(const std::string &value) {
    const auto count = simulated::commands.size();
    const auto output = Serial.output.size();
    line(value);
    advance(30);
    CHECK(commandIdle() && manualReady());
    CHECK(simulated::commands.size() == count);
    CHECK(Serial.output.find("POSE REJECTED", output) != std::string::npos);
    checkStopped();
}
void waitUntilIdle(uint32_t timeout) {
    const uint32_t start = millis();
    while (!commandIdle() && millis() - start < timeout) loop();
    CHECK(commandIdle() && manualReady());
    checkStopped();
}
void stream(int yaw, int pitch, int carriage, uint32_t duration) {
    const uint32_t start = millis();
    while (millis() - start < duration) {
        line("JOG " + std::to_string(yaw) + " " + std::to_string(pitch) + " " + std::to_string(carriage));
        advance(10);
        CHECK(manualActive && phase != Phase::ABORTED);
    }
}
}

int main(int argc, char **argv) {
    if (argc != 2) return 1;
    const std::string scenario = argv[1];
    simulated::baselineYaw = 70;
    simulated::baselinePitch = 8;
    simulated::physicalPitchUsesRoll = false;
    simulated::rawBnoRoll = -3;
    simulated::independentTick = independentTick;

    const bool missingBno = scenario == "startup_missing_bno" || scenario == "manual_without_sensors" ||
        scenario == "angular_requires_bno" || scenario == "carriage_without_bno" || scenario == "carriage_overflow";
    if (missingBno) simulated::bnoAckFails = true;
    if (scenario == "startup_init_failure") simulated::bnoInitFails = true;
    if (scenario == "startup_report_failure") simulated::reportInitFails = true;
    if (scenario == "startup_bus_a_failure") Wire.beginFails = true;
    if (scenario == "startup_bus_b_failure") simulated::busBInitFails = true;
    if (scenario == "startup_low_accuracy" || scenario == "north_qualification" || scenario == "pitch_mapping")
        simulated::accuracy = 1;
    if (scenario == "startup_accuracy_recovery") {
        simulated::accuracy = 0;
        simulated::reinitAccuracy = 3;
    }
    if (scenario == "startup_accuracy_recovery_failure") {
        simulated::accuracy = 0;
        simulated::reinitInitFails = true;
    }
    if (scenario == "startup_invalid") simulated::invalidQuaternion = true;
    if (scenario == "startup_stale") simulated::bnoPauseEnd = 100000;
    if (scenario == "encoders" || scenario == "encoder_failure") {
        Wire.encoder.present = Wire1.encoder.present = true;
        Wire.encoder.raw = 1024;
        Wire1.encoder.raw = 2048;
    }
    if (scenario == "pitch_mapping") simulated::rawBnoRollNoise = 1;

    const uint32_t setupAt = millis();
    setup();
    CHECK(millis() - setupAt <= 1000);
    CHECK(commandIdle() && manualReady());
    CHECK(!pitchReady && !referenceSet && !northUsable);
    CHECK(simulated::commands.empty());
    checkStopped();
    CHECK(yawMotor->directionPin == 32 && yawMotor->stepPin == 33);
    CHECK(pitchMotor->directionPin == 26 && pitchMotor->stepPin == 12);
    CHECK(carriageMotor->directionPin == 21 && carriageMotor->stepPin == 22);
    CHECK(POSITIVE_STEP_PITCH_SIGN == -1 && TRIAL_POSITIVE_STEP_YAW_SIGN == -1);

    // Recovery scenarios have their own timed assertions below; do not consume
    // them in the generic startup branch before those checks can run.
    if (scenario.compare(0, 8, "startup_") == 0 && scenario != "startup_accuracy_recovery" &&
        scenario != "startup_accuracy_recovery_failure") {
        advance(2200);
        CHECK(commandIdle() && manualReady() && simulated::commands.empty());
        CHECK(Serial.output.find("M09 READY") != std::string::npos);
        checkStopped();
        const bool validBno = scenario == "startup_valid" || scenario == "startup_bus_a_failure";
        if (validBno) {
            CHECK(pitchReady && referenceSet && northUsable);
            CHECK(simulated::bnoBeginCalls == 1);
            CHECK(Serial.output.find("BNO_TRACE kind=STARTUP_ACCURACY_STUCK") == std::string::npos);
        } else if (scenario == "startup_low_accuracy" || scenario == "startup_accuracy_recovery" ||
                   scenario == "startup_accuracy_recovery_failure") CHECK(pitchReady && !referenceSet && !northUsable);
        else CHECK(!pitchReady && !referenceSet && !northUsable);
        if (scenario == "startup_bus_a_failure") {
            CHECK(!encoders.state(0).busAvailable && encoders.state(1).busAvailable);
            CHECK(Wire.encoder.addressTransfers == 0);
        }
        if (scenario == "startup_bus_b_failure") {
            CHECK(encoders.state(0).busAvailable && !encoders.state(1).busAvailable);
            CHECK(Wire1.encoder.addressTransfers == 0);
        }
    } else if (scenario == "encoders" || scenario == "encoder_failure") {
        advance(2200);
        CHECK(encoders.state(0).valid && encoders.state(1).valid);
        CHECK(encoders.state(0).raw == 1024 && encoders.state(1).raw == 2048);
        CHECK(encoders.state(0).reads > 10 && encoders.state(1).reads > 10);
        CHECK(Serial.output.find("ENCODER_STATE bus=A") != std::string::npos);
        CHECK(Serial.output.find("ENCODER_STATE bus=B") != std::string::npos);
        CHECK(pitchReady && referenceSet && northUsable && commandIdle());
        if (scenario == "encoder_failure") {
            Wire.encoder.requestLength = 1;
            Wire1.encoder.status = 0x30; // Readable but too weak; not calibrated.
            advance(1200);
            CHECK(!encoders.state(0).valid && encoders.state(0).hasSample);
            CHECK(encoders.state(0).failures > 0 && encoders.state(0).ageMs(millis()) >= 1000);
            CHECK(encoders.state(1).valid && !encoders.state(1).magnetGood());
            CHECK(pitchReady && referenceSet && northUsable && manualReady());
        }
        CHECK(simulated::commands.empty());
    } else if (scenario == "idle_recovery" || scenario == "reset_recovery") {
        advance(2200);
        CHECK(pitchReady && referenceSet && northUsable);
        if (scenario == "idle_recovery") {
            simulated::bnoPauseStart = millis();
            simulated::bnoPauseEnd = millis() + 350;
            advance(200);
            CHECK(!bnoValid && !pitchReady && !referenceSet && !northUsable);
        } else {
            simulated::bnoResetAt = millis();
            advance(30);
            CHECK(simulated::bnoResetDelivered && !pitchReady && !referenceSet);
            // A fresh accepted post-reset sample can immediately restore direct
            // NORTH readiness; the stable POSE reference still needs requalification.
            CHECK(northUsable == northHeadingUsable(millis()));
        }
        CHECK(commandIdle() && manualReady() && simulated::commands.empty());
        advance(1800);
        CHECK(bnoValid && pitchReady && referenceSet && northUsable);
        CHECK(commandIdle() && manualReady() && simulated::commands.empty());
    } else if (scenario == "startup_accuracy_recovery") {
        advance(600);
        line("JOG 0 0 0"); // Established zero-JOG handshake before live commands.
        line("JOG 250 250 250");
        CHECK(manualActive && yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
        stream(250, 250, 250, 3200); // Maintain the existing 250 ms host lease.
        CHECK(simulated::bnoBeginCalls >= 2 && simulated::sh2CloseCalls == 1);
        CHECK(manualActive && !commandWatchdog.tripped());
        CHECK(Serial.output.find("BNO_TRACE kind=STARTUP_ACCURACY_STUCK") != std::string::npos);
        CHECK(Serial.output.find("BNO_TRACE kind=REINIT_SUCCESS") != std::string::npos);
        stream(250, 250, 250, 400);
        CHECK(bnoAccuracy == 3 && bnoValid && manualActive);
        CHECK(Serial.output.find("BNO_TRACE kind=RECOVERY_SUCCESS") != std::string::npos);
        line("STOP");
        waitUntilIdle(1000);
        CHECK(commandIdle() && manualReady());
    } else if (scenario == "startup_accuracy_recovery_failure") {
        advance(600);
        line("JOG 0 0 0");
        line("JOG 250 250 250");
        CHECK(manualActive && yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
        stream(250, 250, 250, 3200);
        CHECK(simulated::bnoBeginCalls == 2 && simulated::sh2CloseCalls == 2); // Old session and failed replacement.
        CHECK(manualActive && !commandWatchdog.tripped());
        CHECK(Serial.output.find("BNO_TRACE kind=STARTUP_ACCURACY_STUCK") != std::string::npos);
        CHECK(Serial.output.find("BNO_TRACE kind=REINIT_FAILURE") != std::string::npos);
        stream(250, 250, 250, 3500);
        CHECK(simulated::bnoBeginCalls == 2 && manualActive && !commandWatchdog.tripped());
        line("STOP");
        waitUntilIdle(1000);
        CHECK(commandIdle() && manualReady());
    } else if (scenario == "north_qualification") {
        advance(2200);
        CHECK(pitchReady && !referenceSet && !northUsable && bnoAccuracy == 1);
        simulated::accuracy = 2;
        advance(400);
        CHECK(!referenceSet && northUsable); // Direct heading is usable; POSE baseline remains unqualified.
        advance(2100);
        CHECK(referenceSet && northUsable && bnoAccuracy == 2);
        simulated::accuracy = 0;
        advance(50);
        CHECK(!northUsable && bnoAccuracy == 0 && manualReady());
        simulated::accuracy = 3;
        advance(50);
        CHECK(northUsable && bnoAccuracy == 3);
        CHECK(simulated::commands.empty());
    } else if (scenario == "manual_without_sensors") {
        advance(50);
        CHECK(!bnoInitialized && !encoders.state(0).valid && !encoders.state(1).valid);
        line("JOG 0 0 0");
        CHECK(manualActive);
        stream(250, 250, 1000, 2000);
        CHECK(yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
        CHECK(carriageMotor->getCurrentPosition() > 500);
        CHECK(!motionWatchdog.armed() && !motionWatchdog.tripped());
        line("STOP");
        waitUntilIdle(4000);
        CHECK(!manualActive && !commandWatchdog.tripped());
        CHECK(!pitchReady && !referenceSet);
    } else if (scenario == "angular_requires_bno") {
        reject("MOVE 0 -3 0");
        reject("MOVE 3 0 0");
        reject("POSE 70 0 0");
        CHECK(!pitchReady && !referenceSet);
        line("JOG 0 0 0");
        CHECK(manualActive);
        line("STOP");
        waitUntilIdle(1000);
    } else if (scenario == "carriage_without_bno") {
        line("MOVE 0 0 1500");
        CHECK(poseActive && !poseNeedsBno);
        waitUntilIdle(10000);
        CHECK(carriageMotor->getCurrentPosition() == 1500);
        CHECK(!motionWatchdog.tripped() && !pitchReady && !referenceSet);
        CHECK(simulated::motors[0].position == 0 && simulated::motors[1].position == 0);
        for (const auto &command : simulated::commands) CHECK(command.stepPin == 22);
        line("MOVE 0 0 -3000");
        waitUntilIdle(12000);
        CHECK(carriageMotor->getCurrentPosition() == -1500);
    } else if (scenario == "carriage_overflow") {
        simulated::motors[2].position = INT32_MAX - 5;
        reject("MOVE 0 0 10");
        reject("MOVE 0 0 2147483648");
        reject("POSE 70 8 -2147483648"); // Target fits, target minus current does not.
        simulated::motors[2].position = INT32_MIN + 5;
        reject("MOVE 0 0 -10");
        CHECK(simulated::commands.empty());
    } else if (scenario == "missing_timing") {
        advance(2200);
        CHECK(pitchReady && referenceSet && northUsable);
        CHECK(yawPulsesPerDegree == 0 && pitchPulsesPerDegree == 0);
        reject("MOVE 4 -2 0");
        CHECK(simulated::commands.empty());
        line("POSE 71 7 0");
        CHECK(poseActive && poseNeedsBno);
        waitUntilIdle(90000);
        CHECK(fabs(shortestDifference(71, orientation.heading)) <= TOLERANCE_DEG);
        CHECK(fabs(orientation.pitch - 7) <= TOLERANCE_DEG);
        CHECK(yawPulsesPerDegree > 0 && pitchPulsesPerDegree > 0);
        CHECK(!simulated::commands.empty());
        for (const auto &command : simulated::commands) {
            CHECK(command.stepPin != 22 && !command.continuous);
            CHECK(command.speed <= 80 && abs(command.steps) <= 16);
            CHECK(!command.wasBraking && !command.beforeStoppedSample);
        }
    } else if (scenario == "pitch_mapping") {
        advance(2200);
        CHECK(pitchReady && !referenceSet && !northUsable);
        CHECK(fabs(baselinePitch - 8) < 0.001);
        CHECK(fabs(orientation.pitch - 8) < 0.001);
        CHECK(orientation.roll > -4.01 && orientation.roll < -1.99);
        line("MOVE 0 -3 0");
        CHECK(poseActive && poseNeedsBno && !yawRequired);
        waitUntilIdle(90000);
        CHECK(fabs(orientation.pitch - 5) <= TOLERANCE_DEG);
        CHECK(fabs(simulated::actualPitch() - 5) <= TOLERANCE_DEG);
        CHECK(orientation.roll > -4.01 && orientation.roll < -1.99);
        CHECK(fabs(pitchAxis.current - physicalPitch()) < 0.01);
        CHECK(fabs(pitchStartAngle - 8) < 0.001);
        CHECK(pitchPulsesPerDegree > 0 && pitchAxis.peakAngularSpeed > 0);
        sensorTelemetry(); advance(100);
        CHECK(Serial.output.find("pitch_axis=PITCH") != std::string::npos);
        char expected[64];
        snprintf(expected, sizeof(expected), "physical_pitch=%.3f", physicalPitch());
        CHECK(Serial.output.find(expected) != std::string::npos);
        CHECK(simulated::motors[0].position == 0 && simulated::motors[2].position == 0);
        for (const auto &command : simulated::commands) CHECK(command.stepPin == 12);
    } else {
        std::fprintf(stderr, "Unknown lifecycle scenario: %s\n", scenario.c_str());
        return 1;
    }
    std::printf("PASS M09 lifecycle: %s (%u checks)\n", scenario.c_str(), checks);
}
