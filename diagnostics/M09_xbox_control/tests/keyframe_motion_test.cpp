// Real command dispatch/control with independent 1 ms motor and watchdog ticks.
// No hardware, homing, physical repeatability or ESP32 scheduler is simulated.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
void require(bool ok, const char *expression, int number) {
    if (!ok) { std::fprintf(stderr, "FAIL keyframe line %d: %s\n%s", number, expression, Serial.output.c_str()); std::exit(1); }
}
#define CHECK(value) require((value), #value, __LINE__)
uint32_t injectAt = 0, handledAt = 0;
bool injected = false;
void tick(uint32_t now) {
    motionWatchdog.check(now); commandWatchdog.check(now);
    if (injectAt && !injected && now >= injectAt) {
        injected = true; simulated::serialInput += "STOP\n";
    }
    if (injected && !handledAt && simulated::motors[0].drive == simulated::Drive::BRAKING) handledAt = now;
}
void advance(uint32_t duration) { const auto start = millis(); while (millis() - start < duration) loop(); }
void command(const std::string &value) { simulated::serialInput += value + "\n"; advance(20); }
uint32_t snapshot(unsigned id) {
    command("SNAP " + std::to_string(id));
    const auto start = Serial.output.rfind("KEYFRAME_SNAPSHOT id=" + std::to_string(id) + " ");
    CHECK(start != std::string::npos);
    const auto at = Serial.output.find("epoch=", start);
    CHECK(at != std::string::npos);
    return static_cast<uint32_t>(std::stoul(Serial.output.substr(at + 6)));
}
void keymove(uint32_t epoch, const std::string &targets = "160 -320 80", unsigned duration = 10000, unsigned id = 2) {
    command("KEYMOVE " + std::to_string(id) + " " + std::to_string(epoch) + " " + targets + " " + std::to_string(duration));
}
bool stopped() { return !yawMotor->isRunning() && !pitchMotor->isRunning() && !carriageMotor->isRunning(); }
void finishMove() {
    const auto start = millis();
    while ((!commandIdle() || !stopped()) && millis() - start < 18000) loop();
    advance(100);
    CHECK(commandIdle() && stopped());
}
void noRestart(size_t commands) {
    CHECK(stopped()); advance(300);
    CHECK(simulated::commands.size() == commands && stopped());
}
}

int main(int argc, char **argv) {
    CHECK(argc == 2);
    const std::string scenario = argv[1];
    simulated::physicalPitchUsesRoll = false;
    simulated::baselineYaw = 20; simulated::baselinePitch = 8;
    simulated::accuracy = 1; simulated::independentTick = tick;
    if (scenario == "missing_bno") simulated::bnoInitFails = true;
    setup(); advance(2200);
    CHECK(commandIdle() && (pitchReady || scenario == "missing_bno") && !northUsable);
    const uint32_t epoch = snapshot(1);
    CHECK(epoch != 0);
    CHECK(Serial.output.find("yaw_steps=0 pitch_steps=0 carriage_steps=0") != std::string::npos);
    CHECK(Serial.output.find("unhomed=YES") != std::string::npos);
    if (scenario == "snapshot") {
        command("JOG 0 0 0"); command("JOG 100 100 100");
        command("SNAP 4");
        CHECK(Serial.output.find("SNAP REJECTED id=4") != std::string::npos);
        command("STOP"); advance(2000);
        const auto after = snapshot(5);
        CHECK(after == epoch); // Normal gentle STOP preserves commanded coordinates.
        CHECK(yawMotor->getCurrentPosition() != 0 && pitchMotor->getCurrentPosition() != 0);
    } else if (scenario == "reject") {
        for (const auto &bad : {
             "KEYMOVE 2 " + std::to_string(epoch) + " 999999 1 1 1000",
             "KEYMOVE 20 " + std::to_string(epoch) + " 0 999999 0 1000",
             "KEYMOVE 21 " + std::to_string(epoch) + " 160 -320 999999 1000",
             "KEYMOVE 3 " + std::to_string(epoch) + " 1 1 1 0",
             "KEYMOVE 4 " + std::to_string(epoch) + " 1 1 1 60001",
             "KEYMOVE 5 " + std::to_string(epoch) + " 2147483648 1 1 10000",
             "KEYMOVE 6 " + std::to_string(epoch) + " 1 1 1.5 10000",
             "KEYMOVE 7 " + std::to_string(epoch) + " nan 1 1 10000",
             "KEYMOVE 8 " + std::to_string(epoch + 1) + " 160 -320 80 10000",
             std::string("KEYMOVE 9 1 1 1 1"), std::string("SNAP 0")}) {
            command(bad); CHECK(simulated::commands.empty() && commandIdle() && stopped());
        }
        CHECK(Serial.output.find("axis=yaw requested_ms=1000") != std::string::npos);
        CHECK(Serial.output.find("axis=pitch requested_ms=1000") != std::string::npos);
        CHECK(Serial.output.find("axis=carriage requested_ms=1000") != std::string::npos);
        CHECK(Serial.output.find("min_ms=") != std::string::npos);
    } else if (scenario == "admission") {
        pitchReady = referenceSet = bnoValid = northUsable = false;
        simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 20000;
        keymove(epoch); CHECK(keyframeActive); finishMove();
        CHECK(Serial.output.find("status=PASS") != std::string::npos);
        CHECK(snapshot(8) == epoch);
    } else if (scenario == "travel") {
        command("KEYRETURN 2 " + std::to_string(epoch) + " 4000 -4800 2000");
        CHECK(keyframeActive && keyframe.travel);
        CHECK(simulated::commands.size() == 3);
        CHECK(simulated::commands[1].speed == 2400 && simulated::commands[1].acceleration == 2400);
        finishMove(); CHECK(pitchMotor->getCurrentPosition() == -4800);
        CHECK(millis() < 8000 && snapshot(3) == epoch);
    } else if (scenario == "stepped") {
        for (unsigned segment = 1; segment <= 8; ++segment) {
            const int yaw = 20 * segment, pitch = -40 * segment, carriage = 10 * segment;
            keymove(epoch, std::to_string(yaw) + " " + std::to_string(pitch) + " " +
                std::to_string(carriage), 1250, segment + 1);
            finishMove();
            CHECK(Serial.output.find("KEYMOVE RESULT id=" + std::to_string(segment + 1) + " status=PASS") != std::string::npos);
            CHECK(yawMotor->getCurrentPosition() == yaw && pitchMotor->getCurrentPosition() == pitch && carriageMotor->getCurrentPosition() == carriage);
            const auto commands = simulated::commands.size();
            // Host settle/exposure/post delay sends no movement command.
            for (unsigned ms = 0; ms < 1500; ms += 10) {
                advance(10); CHECK(stopped());
                CHECK(yawMotor->getCurrentPosition() == yaw && pitchMotor->getCurrentPosition() == pitch && carriageMotor->getCurrentPosition() == carriage);
            }
            CHECK(simulated::commands.size() == commands);
        }
        CHECK(yawMotor->getCurrentPosition() == 160 && pitchMotor->getCurrentPosition() == -320 && carriageMotor->getCurrentPosition() == 80);
    } else if (scenario == "configuration") {
        simulated::accelerationFails = true; keymove(epoch);
        CHECK(simulated::commands.empty() && stopped());
    } else {
        if (scenario == "partial_start") {
            simulated::moveRejected = true; simulated::rejectedStepPin = tmp_hardware::PITCH_STEP_PIN;
        }
        keymove(epoch, scenario == "zero_axis" ? "160 0 80" : (scenario == "no_op" ? "0 0 0" : "160 -320 80"));
        if (scenario == "no_op") {
            CHECK(stopped() && simulated::commands.empty());
            CHECK(Serial.output.find("KEYMOVE RESULT id=2 status=PASS") != std::string::npos);
        } else if (scenario == "partial_start") {
            advance(3500); CHECK(stopped() && commandIdle());
            CHECK(simulated::commands.size() <= 1);
            CHECK(Serial.output.find("KEYFRAME_INVALIDATED") != std::string::npos);
            CHECK(Serial.output.find("status=PASS") == std::string::npos);
        } else {
            CHECK(!commandIdle() && !manualActive);
            CHECK(simulated::commands.size() == (scenario == "zero_axis" ? 2U : 3U));
            const auto begin = simulated::commands.front().at;
            CHECK(simulated::commands.back().at - begin <= 2);
            for (const auto &move : simulated::commands) CHECK(!move.continuous && move.speed <= 80 && move.acceleration <= 250);
            const auto count = simulated::commands.size();
            if (scenario == "complete" || scenario == "zero_axis" || scenario == "missing_bno") {
                command("JOG 0 0 0"); CHECK(!manualActive);
                command("SNAP 3");
                CHECK(Serial.output.find("SNAP REJECTED id=3") != std::string::npos);
                if (millis() < begin + 5000) advance(begin + 5000 - millis());
                CHECK(yawMotor->isRunning() && carriageMotor->isRunning());
                CHECK(fabs(simulated::motors[0].position / 160.0 - 0.5) < 0.03);
                CHECK(fabs(simulated::motors[2].position / 80.0 - 0.5) < 0.03);
                if (scenario == "complete") {
                    CHECK(pitchMotor->isRunning());
                    CHECK(fabs(simulated::motors[1].position / -320.0 - 0.5) < 0.03);
                }
                finishMove();
                CHECK(yawMotor->getCurrentPosition() == 160 && carriageMotor->getCurrentPosition() == 80);
                CHECK(pitchMotor->getCurrentPosition() == (scenario == "zero_axis" ? 0 : -320));
                for (const auto &motor : simulated::motors) {
                    if (motor.target != 0) CHECK(abs(static_cast<int32_t>(motor.stoppedAt - begin) - 10000) <= 100);
                }
                CHECK(Serial.output.find("KEYMOVE RESULT id=2 status=PASS") != std::string::npos);
                CHECK(Serial.output.find("PITCH settled=") == std::string::npos);
                CHECK(snapshot(6) == epoch);
                // A reverse leg returns all generated counts to their captured A.
                keymove(epoch, "0 0 0", 10000, 7); finishMove();
                CHECK(yawMotor->getCurrentPosition() == 0 && pitchMotor->getCurrentPosition() == 0 && carriageMotor->getCurrentPosition() == 0);
                command("MOVE 0 0 100");
                CHECK(poseActive && carriageMotor->getAcceleration() == CARRIAGE_ACCELERATION);
                finishMove();
                if (scenario == "missing_bno") { std::puts("PASS KEYMOVE with no BNO hardware"); return 0; }
                command("MOVE 0 1 100"); advance(50);
                CHECK(poseActive && carriageMotor->getAcceleration() == CARRIAGE_ACCELERATION);
                command("STOP"); finishMove();
            } else {
                advance(1200);
                if (scenario == "stop" || scenario == "brake_timeout") {
                    if (scenario == "brake_timeout") for (auto &motor : simulated::motors) motor.neverStops = true;
                    command("STOP");
                    CHECK(simulated::gentleStops >= 3 && !motionWatchdog.armed());
                    command("STOP"); // No restart or second motion.
                } else if (scenario == "abort") command("X");
                else if (scenario == "stale") {
                    simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 5000;
                } else if (scenario == "reset") simulated::resetDuringPoll = true;
                else if (scenario == "pitch_guard") simulated::pitchDisturbance = 70;
                else if (scenario == "stall_stop") {
                    injectAt = millis() + 30; simulated::blockBnoMs = 1000;
                    advance(1100);
                    CHECK(injected && handledAt && handledAt - injectAt <= 20);
                    CHECK(bnoTrace.acquireMaxUs >= 1000000);
                } else if (scenario == "stall_continue") { simulated::blockBnoMs = 1000; advance(1100); }
                else if (scenario == "unexpected_stop") yawMotor->forceStop();
                else { CHECK(false); }
                if (scenario == "stale" || scenario == "reset" || scenario == "pitch_guard" || scenario == "stall_continue") {
                    finishMove(); CHECK(Serial.output.find("KEYMOVE RESULT id=2 status=PASS") != std::string::npos);
                    CHECK(snapshot(8) == epoch);
                    std::printf("PASS sensor-independent keyframe: %s\n", scenario.c_str()); return 0;
                }
                advance(3500);
                CHECK(stopped()); noRestart(count);
                CHECK(Serial.output.find("KEYMOVE RESULT id=2 status=PASS") == std::string::npos);
                if (scenario == "abort") CHECK(phase == Phase::ABORTED);
                else CHECK(commandIdle());
                if (scenario != "stop" && scenario != "stall_stop")
                    CHECK(Serial.output.find("KEYFRAME_INVALIDATED") != std::string::npos);
                if (scenario == "stop") CHECK(snapshot(8) == epoch);
            }
        }
    }
    std::printf("PASS keyframe motion: %s\n", scenario.c_str());
}
