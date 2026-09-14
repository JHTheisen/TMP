#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
unsigned checks = 0;
size_t baselineCommands = 0;
void require(bool ok, const char *expression, int line) {
    ++checks;
    if (!ok) { std::fprintf(stderr, "FAIL line %d: %s\n%s", line, expression, Serial.output.c_str()); std::exit(1); }
}
#define CHECK(value) require((value), #value, __LINE__)
void tick(uint32_t now) { motionWatchdog.check(now); commandWatchdog.check(now); }
void advance(uint32_t duration) { const uint32_t until = millis() + duration; while (millis() < until) loop(); }
void line(const std::string &value) { simulated::serialInput += value + "\n"; advance(10); }
void stream(int yaw, int pitch, uint32_t duration) {
    const uint32_t until = millis() + duration;
    while (millis() < until && phase != Phase::ABORTED) {
        line("JOG " + std::to_string(yaw) + " " + std::to_string(pitch)); advance(10);
    }
}
void startup(bool pitchOnly = false) {
    simulated::independentTick = tick;
    if (pitchOnly) simulated::accuracy = 0;
    setup();
    const uint32_t until = millis() + 100000;
    while (!finalPrinted && millis() < until) loop();
    advance(500);
    CHECK(commandIdle()); CHECK(referenceSet != pitchOnly);
    baselineCommands = simulated::commands.size();
}
void arm() { line("JOG 0 0"); CHECK(manualActive && phase == Phase::MANUAL); }
void stop() { line("STOP"); advance(2000); CHECK(commandIdle() && !manualActive); }
void aborted() {
    advance(200);
    CHECK(phase == Phase::ABORTED && finalPrinted && !manualActive);
    const size_t count = simulated::commands.size();
    for (auto &motor : simulated::motors) CHECK(motor.drive == simulated::Drive::IDLE);
    line("JOG 0 0"); line("JOG 100 100"); line("MOVE 2 2 10");
    CHECK(phase == Phase::ABORTED && count == simulated::commands.size());
}
}
int main(int argc, char **argv) {
    if (argc != 2) return 1;
    const std::string requestedScenario = argv[1];
    const bool lowAccuracy = requestedScenario.find("low_") == 0;
    const std::string scenario = lowAccuracy ? requestedScenario.substr(4) : requestedScenario;
    if (scenario == "startup_busy") {
        simulated::independentTick = tick; setup(); line("JOG 0 0"); line("JOG 100 100");
        CHECK(!manualActive && simulated::commands.empty()); line("X"); aborted();
    } else {
        startup(lowAccuracy || scenario == "pitch_only_ready");
        if (scenario == "admission" || scenario == "pitch_only_ready") {
            line("JOG 10 0"); line("JOG nan 0"); line("JOG 1001 0"); line("JOG 0.5 0");
            line("JOG 0"); line("JOG 0 0 1"); line(std::string(140, 'a'));
            CHECK(!manualActive && baselineCommands == simulated::commands.size());
            simulated::accuracy = 0; advance(20);
            line("STATUS"); CHECK(Serial.output.find("M09 READY") != std::string::npos);
            arm();
            for (unsigned accuracy = 0; accuracy <= 3; ++accuracy) {
                simulated::accuracy = accuracy; stream(100, 100, 300);
                CHECK(manualActive && bnoAccuracy == accuracy);
                CHECK(yawMotor->isRunning() && pitchMotor->isRunning());
                if (scenario == "pitch_only_ready") CHECK(!referenceSet && !northUsable);
                stop(); arm();
            }
            stop();
            if (scenario == "pitch_only_ready") {
                const size_t count = simulated::commands.size();
                line("MOVE 2 0 0"); line("MOVE 0 0 10");
                CHECK(!poseActive && simulated::commands.size() == count && !referenceSet && !northUsable);
            } else {
                simulated::accuracy = 0; advance(20);
                const size_t count = simulated::commands.size();
                line("MOVE 2 0 0"); CHECK(!poseActive && simulated::commands.size() == count && !northUsable);
            }
        } else {
            arm();
            if (scenario == "accuracy") {
                for (unsigned accuracy : {3u, 0u, 1u, 2u, 0u}) {
                    simulated::accuracy = accuracy; stream(100, 100, 400);
                    CHECK(manualActive && bnoAccuracy == accuracy);
                    if (accuracy < 2) CHECK(!northUsable);
                }
                stop();
            } else if (scenario == "center") {
                stream(0, 0, 2000); CHECK(simulated::commands.size() == baselineCommands); stop();
            } else if (scenario == "axes" || scenario == "reverse" || scenario == "pose_after") {
                const double y = orientation.heading, p = orientation.roll;
                stream(250, 0, 700); CHECK(!finalPrinted && simulated::actualYaw() > y + 0.5);
                CHECK(fabs(simulated::actualPitch() - p) < 0.01);
                stream(0, 0, 700); CHECK(!yawMotor->isRunning() && !pitchMotor->isRunning());
                const double stoppedYaw = simulated::actualYaw();
                stream(0, 250, 700); CHECK(simulated::actualPitch() > p + 0.5);
                CHECK(fabs(simulated::actualYaw() - stoppedYaw) < 0.01);
                if (scenario == "reverse") {
                    stream(0, -250, 1000); CHECK(manualPitch.direction == -1 && !finalPrinted);
                    CHECK(pitchMotor->getCurrentSpeedInMilliHz() > 0);
                }
                const double oldYawTiming = yawPulsesPerDegree, oldPitchTiming = pitchPulsesPerDegree;
                stop();
                CHECK(yawPulsesPerDegree == oldYawTiming && pitchPulsesPerDegree == oldPitchTiming);
                if (scenario == "pose_after") {
                    line("POSE 0 0 100"); const uint32_t until = millis() + 100000;
                    while (!finalPrinted && millis() < until) loop();
                    CHECK(commandIdle() && carriageMotor->getCurrentPosition() == 100);
                    CHECK(fabs(orientation.roll) <= TOLERANCE_DEG);
                    CHECK(fabs(shortestDifference(0, orientation.heading)) <= TOLERANCE_DEG);
                }
            } else if (scenario == "rate_update") {
                stream(100, 100, 400);
                const size_t count = simulated::commands.size();
                stream(500, 500, 700);
                CHECK(simulated::commands.size() == count); // Rate changes do not restart/reverse continuous motion.
                CHECK(manualYaw.rate == 500 && manualPitch.rate == 300);
                CHECK(simulated::motors[0].acceleration == 1000 && simulated::motors[1].acceleration == 600);
                stream(1000, 1000, 300); CHECK(manualYaw.rate == 1000 && manualPitch.rate == 600); stop();
            } else if (scenario == "busy") {
                stream(100, 100, 100); line("MOVE 2 2 100");
                CHECK(!poseActive && manualActive && carriageMotor->getCurrentPosition() == 0); stop();
            } else if (scenario == "blocked_uart") {
                simulated::serialWriteSpace = 0; stream(150, 150, 4000);
                CHECK(manualActive && fresh(millis()) && !motionWatchdog.tripped());
                stop(); simulated::serialWriteSpace = 128;
            } else {
                stream(250, 250, 300); CHECK(yawMotor->isRunning() && pitchMotor->isRunning());
                simulated::forceStops = 0;
                if (scenario == "host_loss" || scenario == "partial" || scenario == "malformed_lease") {
                    const uint32_t lastCommand = millis() - 20;
                    if (scenario == "partial") simulated::serialInput += "JOG 0";
                    if (scenario == "malformed_lease") line("JOG nan 0");
                    advance(300); CHECK(commandWatchdog.tripped());
                    CHECK(simulated::firstForceStopAt - lastCommand == MANUAL_COMMAND_TIMEOUT_MS);
                } else if (scenario == "blocked_host") {
                    // Independent callback still runs while loop is absent; feed
                    // only BNO timestamps to isolate the command lease watchdog.
                    for (unsigned i = 0; i < 30; ++i) { motionWatchdog.recordFresh(millis()); delay(10); }
                    CHECK(commandWatchdog.tripped()); advance(10);
                } else if (scenario == "stale" || scenario == "invalid" || scenario == "wrong_report" || scenario == "blocked_bno") {
                    const uint32_t lastFresh = lastBnoGood;
                    if (scenario == "stale") { simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 1000; }
                    if (scenario == "invalid") simulated::invalidQuaternion = true;
                    if (scenario == "wrong_report") simulated::wrongReportType = true;
                    if (scenario == "blocked_bno") simulated::blockBnoMs = 1000;
                    stream(250, 250, 400); CHECK(motionWatchdog.tripped());
                    CHECK(simulated::firstForceStopAt - lastFresh == BNO_STALE_MS);
                } else if (scenario == "reset") {
                    simulated::resetDuringPoll = true; stream(250, 250, 40);
                } else if (scenario == "abort") {
                    simulated::serialInput += "JOG 12x"; advance(20);
                } else if (scenario == "wrong_direction") {
                    simulated::motors[1].physicalSign = 1; stream(250, 250, 1000);
                } else if (scenario == "no_progress") {
                    simulated::motors[1].frozen = true; stream(250, 250, 2500);
                } else if (scenario == "yaw_guard") {
                    simulated::yawDisturbance += RELATIVE_LIMIT_DEG; stream(250, 250, 40);
                } else if (scenario == "pitch_guard") {
                    simulated::pitchDisturbance = MAX_USABLE_PITCH_DEG; stream(250, 250, 40);
                } else if (scenario == "braking_timeout") {
                    simulated::motors[1].neverStops = true; stream(0, 0, BRAKING_TIMEOUT_MS + 200);
                } else if (scenario == "timeout") {
                    controlStartedAt = millis() - LEG_TIMEOUT_MS; advance(10);
                } else if (scenario == "speed_rejected") {
                    simulated::speedFails = true; stream(500, 500, 40);
                } else if (scenario == "late_packet") {
                    simulated::independentTick = nullptr;
                    delay(260); line("JOG 0 0"); CHECK(commandWatchdog.tripped());
                } else { std::fprintf(stderr, "Unknown scenario %s\n", scenario.c_str()); return 1; }
                aborted();
            }
        }
    }
    if (scenario != "pose_after") {
        CHECK(carriageMotor->getCurrentPosition() == 0);
        for (size_t i = baselineCommands; i < simulated::commands.size(); ++i) {
            const auto &command = simulated::commands[i];
            CHECK(command.stepPin != 22);
            CHECK(command.speed <= slewSpeed(command.stepPin == 12));
            CHECK(!command.wasBraking && !command.beforeStoppedSample);
        }
    }
    std::printf("PASS M09: %s (%u checks)\n", scenario.c_str(), checks);
}
