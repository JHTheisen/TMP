#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
void require(bool ok, const char *expression, int line) {
    if (!ok) { std::fprintf(stderr, "FAIL manual line %d: %s\n%s", line, expression, Serial.output.c_str()); std::exit(1); }
}
#define CHECK(value) require((value), #value, __LINE__)
void tick(uint32_t now) { motionWatchdog.check(now); commandWatchdog.check(now); }
void advance(uint32_t duration) { const uint32_t until = millis() + duration; while (millis() < until) loop(); }
void line(const std::string &value) { simulated::serialInput += value + "\n"; advance(10); }
void stream(int yaw, int pitch, uint32_t duration) {
    const uint32_t until = millis() + duration;
    while (millis() < until) {
        line("JOG " + std::to_string(yaw) + " " + std::to_string(pitch)); advance(10);
    }
}
void arm() { line("JOG 0 0"); CHECK(manualActive && phase == Phase::MANUAL); }
void readyStopped() {
    CHECK(commandIdle() && !manualActive && manualReady());
    CHECK(manualMotorsStopped() && !commandWatchdog.armed() && !commandWatchdog.tripped());
}
void stop() { line("STOP"); advance(2000); readyStopped(); }
void recover() {
    advance(100); readyStopped();
    const size_t commands = simulated::commands.size();
    line("JOG 500 500"); CHECK(!manualActive && simulated::commands.size() == commands);
    arm(); stream(100, 100, 300);
    CHECK(yawMotor->isRunning() && pitchMotor->isRunning()); stop();
}
void aborted() {
    advance(100);
    CHECK(phase == Phase::ABORTED && finalPrinted && !manualActive);
    const size_t commands = simulated::commands.size();
    for (auto &motor : simulated::motors) CHECK(motor.drive == simulated::Drive::IDLE);
    line("JOG 0 0"); line("JOG 100 100");
    CHECK(phase == Phase::ABORTED && simulated::commands.size() == commands);
}
}
int main(int argc, char **argv) {
    if (argc != 2) return 1;
    const std::string scenario = argv[1];
    simulated::physicalPitchUsesRoll = false;
    simulated::independentTick = tick;
    if (scenario == "missing_bno") simulated::bnoAckFails = true;
    if (scenario == "low_accuracy") simulated::accuracy = 0;
    if (scenario == "axis_init") simulated::axisInitFails = true;
    if (scenario == "speed_init") simulated::speedFails = true;
    setup();
    if (scenario == "axis_init" || scenario == "speed_init") {
        aborted(); std::printf("manual %s passed\n", scenario.c_str()); return 0;
    }
    advance(50);
    CHECK(controlReady && commandIdle() && manualReady());
    CHECK(simulated::commands.empty()); // Startup never moves or learns timing.
    CHECK(millis() < 3000);
    CHECK(yawMotor->stepPin == 33 && yawMotor->directionPin == 32);
    CHECK(pitchMotor->stepPin == 12 && pitchMotor->directionPin == 26);
    CHECK(carriageMotor->stepPin == 22 && carriageMotor->directionPin == 21);
    if (scenario == "startup") {
        advance(6000); readyStopped(); CHECK(simulated::commands.empty());
    } else if (scenario == "admission") {
        for (const auto *bad : {"JOG 1 0", "JOG nan 0", "JOG inf 0", "JOG 1001 0", "JOG 0.5 0",
                               "JOG 0", "JOG 0 0 1", "JOG 0 0 0 0"}) {
            line(bad); CHECK(!manualActive && simulated::commands.empty());
        }
        arm(); advance(6000);
        CHECK(manualActive && !commandWatchdog.armed() && simulated::commands.empty());
        stop();
    } else {
        arm();
        if (scenario == "axes" || scenario == "rates" || scenario == "reverse") {
            const double yaw = simulated::actualYaw(), pitch = simulated::actualPitch();
            stream(250, 300, 500);
            CHECK(simulated::actualYaw() > yaw && simulated::actualPitch() > pitch);
            CHECK(carriageMotor->getCurrentPosition() == 0 && !carriageMotor->isRunning());
            CHECK(manualYaw.rate == YAW_SLEW_SPEED_HZ / 4 && manualPitch.rate == PITCH_TRAVEL_SPEED_HZ * 3 / 10);
            const size_t commands = simulated::commands.size();
            stream(500, 500, 200); CHECK(simulated::commands.size() == commands);
            if (scenario == "rates") {
                stream(1000, 1000, 200);
                CHECK(manualYaw.rate == YAW_SLEW_SPEED_HZ && manualPitch.rate == PITCH_TRAVEL_SPEED_HZ);
                CHECK(simulated::motors[0].acceleration == YAW_SLEW_ACCELERATION);
                CHECK(simulated::motors[1].acceleration == PITCH_TRAVEL_ACCELERATION);
            }
            if (scenario == "reverse") {
                stream(-250, -250, 1300);
                CHECK(manualYaw.direction == -1 && manualPitch.direction == -1);
                for (size_t i = commands; i < simulated::commands.size(); ++i) {
                    const auto &command = simulated::commands[i];
                    CHECK(!command.wasBraking && command.continuous);
                    const auto &motor = simulated::motors[command.stepPin == 33 ? 0 : 1];
                    CHECK(command.at - motor.stoppedAt >= PRECISION_OBSERVE_MS);
                }
            }
            stream(0, 0, 1500);
            CHECK(manualActive && manualMotorsStopped() && !commandWatchdog.armed());
            const size_t centeredCommands = simulated::commands.size();
            advance(1000); CHECK(simulated::commands.size() == centeredCommands); stop();
        } else if (scenario == "missing_bno" || scenario == "low_accuracy" || scenario == "stale" ||
                   scenario == "invalid" || scenario == "wrong_report" || scenario == "reset" ||
                   scenario == "no_orientation_limits" || scenario == "frozen_feedback" || scenario == "stop_without_bno") {
            if (scenario == "stale" || scenario == "stop_without_bno") {
                simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 10000;
            }
            if (scenario == "invalid") simulated::invalidQuaternion = true;
            if (scenario == "wrong_report") simulated::wrongReportType = true;
            if (scenario == "reset") simulated::resetDuringPoll = true;
            if (scenario == "no_orientation_limits") {
                simulated::yawDisturbance = 250; simulated::pitchDisturbance = 100;
                simulated::motors[0].physicalSign *= -1; // No BNO direction/runaway gate for JOG.
            }
            if (scenario == "frozen_feedback") simulated::frozenFeedback = true;
            for (unsigned accuracy = 0; accuracy <= 3; ++accuracy) {
                simulated::accuracy = accuracy; stream(100, 100, 300);
                CHECK(manualActive && yawMotor->isRunning() && pitchMotor->isRunning());
                CHECK(!motionWatchdog.armed() && !motionWatchdog.tripped());
            }
            stop();
        } else if (scenario == "long_manual") {
            stream(1, 1, 92000);
            CHECK(manualActive && yawMotor->isRunning() && pitchMotor->isRunning()); stop();
        } else if (scenario == "axis_rejected") {
            simulated::moveRejected = true; simulated::rejectedStepPin = 33;
            stream(100, 100, 300);
            CHECK(manualActive && !yawMotor->isRunning() && pitchMotor->isRunning());
            CHECK(Serial.output.find("MANUAL AXIS ERROR: YAW") != std::string::npos);
            simulated::moveRejected = false; stream(100, 100, 300);
            CHECK(yawMotor->isRunning() && pitchMotor->isRunning()); stop();
        } else {
            stream(200, 200, 400);
            CHECK(yawMotor->isRunning() && pitchMotor->isRunning());
            simulated::forceStops = 0;
            if (scenario == "host_loss") {
                const uint32_t stoppedSendingAt = millis();
                advance(400);
                CHECK(simulated::forceStops >= 2);
                CHECK(simulated::firstForceStopAt - stoppedSendingAt <= MANUAL_COMMAND_TIMEOUT_MS + 10);
                recover();
            } else if (scenario == "malformed_lease") {
                for (unsigned i = 0; i != 25; ++i) { line("JOG nan 200"); advance(10); }
                recover();
            } else if (scenario == "late_packet") {
                simulated::independentTick = nullptr;
                simulated::advance(300); CHECK(yawMotor->isRunning());
                line("JOG 200 200");
                CHECK(!manualActive && !yawMotor->isRunning());
                simulated::independentTick = tick; recover();
            } else if (scenario == "blocked_main") {
                simulated::advance(400); // No loop()/sensor/serial service.
                CHECK(commandWatchdog.tripped());
                CHECK(!yawMotor->isRunning() && !pitchMotor->isRunning());
                loop(); recover();
            } else if (scenario == "slow_encoders") {
                Wire.encoder.present = Wire1.encoder.present = true;
                Wire.encoder.addressDelayMs = Wire1.encoder.addressDelayMs = 180;
                advance(300);
                CHECK(simulated::forceStops >= 2 && phase != Phase::ABORTED);
                Wire.encoder.addressDelayMs = Wire1.encoder.addressDelayMs = 0;
                recover();
            } else if (scenario == "braking_timeout") {
                simulated::motors[0].neverStops = true;
                stream(0, 200, 3600);
                CHECK(manualActive && !yawMotor->isRunning() && pitchMotor->isRunning());
                CHECK(Serial.output.find("YAW braking timeout") != std::string::npos);
                simulated::motors[0].neverStops = false; stop();
            } else if (scenario == "unexpected_stop") {
                yawMotor->forceStop(); advance(10);
                CHECK(manualActive && manualYaw.request == 0 && pitchMotor->isRunning());
                CHECK(Serial.output.find("YAW continuous motor stopped unexpectedly") != std::string::npos);
                stream(200, 200, 400); CHECK(yawMotor->isRunning()); stop();
            } else if (scenario == "abort") {
                line("X"); aborted();
            } else { std::fprintf(stderr, "Unknown manual scenario\n"); return 2; }
        }
    }
    std::printf("manual %s passed\n", scenario.c_str());
}
