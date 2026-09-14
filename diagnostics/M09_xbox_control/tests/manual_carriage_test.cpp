#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
void require(bool ok, int line) {
    if (!ok) { std::fprintf(stderr, "FAIL carriage line %d\n%s", line, Serial.output.c_str()); std::exit(1); }
}
#define CHECK(value) require((value), __LINE__)
bool checkEnvelope = true;
void tick(uint32_t now) {
    motionWatchdog.check(now); commandWatchdog.check(now);
    if (checkEnvelope) CHECK(fabs(simulated::motors[2].position) <= CARRIAGE_LIMIT_STEPS + 0.01);
}
void advance(uint32_t duration) { const auto end = millis() + duration; while (millis() < end) loop(); }
void line(const std::string &text) { simulated::serialInput += text + "\n"; advance(10); }
void stream(int carriage, uint32_t duration, int yaw = 0, int pitch = 0) {
    const auto end = millis() + duration;
    while (millis() < end && phase != Phase::ABORTED) {
        line("JOG " + std::to_string(yaw) + " " + std::to_string(pitch) + " " + std::to_string(carriage));
        advance(10);
    }
}
void arm() { line("JOG 0 0 0"); CHECK(manualActive); }
void stop() { line("STOP"); advance(2000); CHECK(commandIdle() && !manualActive && !carriageMotor->isRunning()); }
}
int main(int argc, char **argv) {
    if (argc != 2) return 1;
    const std::string scenario = argv[1];
    simulated::accuracy = 0; simulated::independentTick = tick;
    setup();
    while (!finalPrinted && millis() < 30000) loop();
    CHECK(commandIdle() && pitchReady && !referenceSet && !northUsable);
    if (scenario == "protocol") {
        for (const auto *bad : {"JOG 0 0 1", "JOG 0 0 nan", "JOG 0 0 inf", "JOG 0 0 1001", "JOG 0 0 0.5", "JOG 0 0 0 0"}) {
            line(bad); CHECK(!manualActive && simulated::commands.empty());
        }
        arm(); stream(250, 300); CHECK(carriageMotor->isRunning());
        line("JOG 0 0"); CHECK(manualCarriage.request == 0);
        stream(0, 700); CHECK(!carriageMotor->isRunning()); stop();
    } else {
        arm();
        if (scenario == "axes" || scenario == "reverse" || scenario == "rate" || scenario == "pose_after") {
            const double yaw = simulated::actualYaw(), pitch = simulated::actualPitch();
            stream(100, 300);
            CHECK(carriageMotor->getCurrentPosition() > 10);
            CHECK(simulated::actualYaw() == yaw && simulated::actualPitch() == pitch);
            const auto count = simulated::commands.size();
            stream(250, 300); CHECK(simulated::commands.size() == count && manualCarriage.rate == 250);
            CHECK(simulated::motors[2].acceleration == CARRIAGE_ACCELERATION);
            if (scenario == "rate") {
                stream(1000, 100); CHECK(manualCarriage.rate == CARRIAGE_MAX_SPEED_HZ);
            }
            if (scenario == "reverse") {
                stream(-250, 900); CHECK(manualCarriage.direction == -1 && carriageMotor->isRunning());
            }
            stream(0, 1200); CHECK(!carriageMotor->isRunning());
            const auto position = carriageMotor->getCurrentPosition();
            stream(0, 400); CHECK(carriageMotor->getCurrentPosition() == position);
            stream(100, 200, 100, 100);
            CHECK(yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
            stop(); CHECK(!yawMotor->isRunning() && !pitchMotor->isRunning());
            if (scenario == "pose_after") {
                const auto commands = simulated::commands.size();
                line("MOVE 0 0 10"); CHECK(!poseActive && simulated::commands.size() == commands);
                CHECK(!referenceSet && !northUsable);
            }
        } else if (scenario == "boundary_stop") {
            for (const int direction : {1, -1}) {
                for (const uint32_t duration : {100u, 300u, 350u, 400u, 500u, 600u}) {
                    simulated::motors[2].position = direction * 400;
                    stream(direction * 1000, duration); stop(); arm();
                }
            }
            stop();
        } else if (scenario == "limits" || scenario == "near_limit") {
            if (scenario == "near_limit") simulated::motors[2].position = 490;
            stream(1000, 2500); CHECK(manualActive && carriageMotor->getCurrentPosition() == 500 && !carriageMotor->isRunning());
            const auto count = simulated::commands.size();
            stream(1000, 500); CHECK(simulated::commands.size() == count);
            stream(-1000, 3000); CHECK(manualActive && carriageMotor->getCurrentPosition() == -500 && !carriageMotor->isRunning());
            stop(); arm(); stream(-1000, 300); CHECK(carriageMotor->getCurrentPosition() == -500);
            stream(250, 500); CHECK(carriageMotor->getCurrentPosition() > -500); stop();
        } else {
            stream(250, 300, 100, 100); CHECK(carriageMotor->isRunning());
            const auto lastFresh = lastBnoGood;
            simulated::forceStops = 0;
            if (scenario == "host_loss" || scenario == "malformed") {
                if (scenario == "malformed") line("JOG 0 0 nan");
                advance(300); CHECK(commandWatchdog.tripped());
            } else if (scenario == "stale" || scenario == "invalid" || scenario == "wrong_report" || scenario == "blocked_bno") {
                if (scenario == "stale") { simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 1000; }
                if (scenario == "invalid") simulated::invalidQuaternion = true;
                if (scenario == "wrong_report") simulated::wrongReportType = true;
                if (scenario == "blocked_bno") simulated::blockBnoMs = 1000;
                stream(250, 400, 100, 100);
                CHECK(motionWatchdog.tripped() && simulated::firstForceStopAt - lastFresh == BNO_STALE_MS);
            } else if (scenario == "reset") {
                simulated::resetDuringPoll = true; stream(250, 40);
            } else if (scenario == "abort") {
                line("X");
            } else if (scenario == "braking_timeout") {
                simulated::motors[2].neverStops = true; line("STOP"); advance(BRAKING_TIMEOUT_MS + 200);
            } else if (scenario == "stop_stale") {
                line("STOP"); simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 1000;
                advance(300); CHECK(motionWatchdog.tripped());
            } else if (scenario == "limit_guard") {
                checkEnvelope = false; simulated::motors[2].position = 501; advance(10);
            } else if (scenario == "timeout") {
                controlStartedAt = millis() - LEG_TIMEOUT_MS; advance(10);
            } else if (scenario == "speed_rejected") {
                simulated::speedFails = true; stream(500, 40);
            } else { std::fprintf(stderr, "Unknown carriage scenario %s\n", argv[1]); return 1; }
            advance(100);
            CHECK(phase == Phase::ABORTED && !manualActive);
            for (auto &motor : simulated::motors) CHECK(motor.drive == simulated::Drive::IDLE);
            const auto count = simulated::commands.size(); line("JOG 0 0 0"); line("JOG 0 0 250");
            CHECK(phase == Phase::ABORTED && simulated::commands.size() == count);
        }
    }
    for (const auto &command : simulated::commands) {
        CHECK(command.speed <= (command.stepPin == 22 ? CARRIAGE_MAX_SPEED_HZ : slewSpeed(command.stepPin == 12)));
        CHECK(!command.wasBraking && !command.beforeStoppedSample);
        if (command.stepPin == 22) CHECK(!command.continuous);
    }
    std::printf("PASS M09 carriage: %s\n", argv[1]);
}
