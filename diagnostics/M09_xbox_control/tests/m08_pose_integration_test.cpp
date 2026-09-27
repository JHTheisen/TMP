// Preserve the BNO controller after changing its startup/failure lifecycle.
// Explicit synthetic timing below is a fixture, never firmware calibration.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>
namespace {
void require(bool ok, const char *expression, int line) {
    if (!ok) { std::fprintf(stderr, "FAIL pose line %d: %s\n%s", line, expression, Serial.output.c_str()); std::exit(1); }
}
#define CHECK(value) require((value), #value, __LINE__)
void tick(uint32_t now) { motionWatchdog.check(now); commandWatchdog.check(now); }
void advance(uint32_t duration) { const uint32_t start = millis(); while (millis() - start < duration) loop(); }
void line(const std::string &value) { simulated::serialInput += value + "\n"; advance(10); }
void complete() { const uint32_t start = millis(); while (poseActive && millis() - start < 95000) loop(); advance(100); CHECK(commandIdle() && manualReady()); }
void assertManualRecovery() {
    CHECK(commandIdle() && manualReady() && !poseActive);
    line("JOG 0 0 0"); CHECK(manualActive);
    line("JOG 100 100 100"); CHECK(yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
    line("STOP"); advance(2000); CHECK(commandIdle());
}
}
int main(int argc, char **argv) {
    if (argc != 2) return 1;
    const std::string scenario = argv[1];
    simulated::physicalPitchUsesRoll = false;
    simulated::independentTick = tick;
    simulated::baselineYaw = 20; simulated::baselinePitch = 8;
    if (scenario == "pitch_low" || scenario == "pitch_carriage_low") simulated::accuracy = 0;
    setup(); advance(2200);
    CHECK(commandIdle() && pitchReady && simulated::commands.empty());
    // Known synthetic response exercises the learned-response planner. Separate
    // lifecycle coverage checks conservative first-boot precision admission.
    yawPulsesPerDegree = 1 / simulated::motors[0].degreesPerStep;
    pitchPulsesPerDegree = 1 / simulated::motors[1].degreesPerStep;
    if (scenario == "invalid") {
        for (const auto *bad : {"POSE nan 0 0", "POSE 10 75 0", "POSE 10 -75 0", "MOVE 180 0 0", "MOVE -181 0 0", "POSE 0 0 1.5", "POSE 0 0 0 junk"}) line(bad);
        CHECK(commandIdle() && simulated::commands.empty());
    } else if (scenario == "coordinated" || scenario == "relative" || scenario == "pitch_low" || scenario == "pitch_carriage_low" || scenario == "no_op") {
        if (scenario == "no_op") {
            line("POSE 20 8 0"); CHECK(commandIdle() && simulated::commands.empty());
        } else {
            if (scenario == "relative") line("MOVE 3 -2 1200");
            else if (scenario == "pitch_low") line("MOVE 0 -2 0");
            else if (scenario == "pitch_carriage_low") line("MOVE 0 -2 1200");
            else line("POSE 23 6 1200");
            CHECK(poseActive);
            line("JOG 0 0 0"); CHECK(poseActive && !manualActive);
            complete();
            CHECK(Serial.output.find("FINAL RESULT: PASS") != std::string::npos);
            CHECK(fabs(orientation.pitch - 6) <= TOLERANCE_DEG);
            CHECK(fabs(shortestDifference((scenario == "pitch_low" || scenario == "pitch_carriage_low") ? 20 : 23, orientation.heading)) <= TOLERANCE_DEG);
            CHECK(carriageMotor->getCurrentPosition() == (scenario == "pitch_low" ? 0 : 1200));
            CHECK(fabs(orientation.roll - simulated::rawBnoRoll) < 0.01);
        }
    } else {
        line("POSE 35 13 1200"); CHECK(poseActive);
        advance(100);
        if (scenario == "stale") { simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 1000; advance(300); }
        else if (scenario == "blocked_bno") { simulated::blockBnoMs = 500; advance(10); }
        else if (scenario == "invalid_feedback") { simulated::invalidQuaternion = true; advance(300); }
        else if (scenario == "wrong_report") { simulated::wrongReportType = true; advance(300); }
        else if (scenario == "accuracy") { simulated::accuracy = 1; advance(30); }
        else if (scenario == "reset") { simulated::resetDuringPoll = true; advance(30); }
        else if (scenario == "runaway") { simulated::yawDisturbance = -2; advance(30); }
        else if (scenario == "pitch_guard") { simulated::pitchDisturbance = 70; advance(30); }
        else if (scenario == "no_progress") { simulated::frozenFeedback = true; advance(16000); }
        else if (scenario == "timeout") { controlStartedAt = millis() - LEG_TIMEOUT_MS; advance(30); }
        else if (scenario == "abort") { line("X"); CHECK(phase == Phase::ABORTED && !poseActive); }
        else { std::fprintf(stderr, "Unknown pose scenario\n"); return 2; }
        if (scenario != "abort") {
            advance(100); // Nonblocking serial summary must have time to drain.
            CHECK(commandIdle() && Serial.output.find("OPERATION FAILED:") != std::string::npos);
            assertManualRecovery();
        }
        for (auto &motor : simulated::motors) CHECK(motor.drive == simulated::Drive::IDLE);
    }
    std::printf("PASS preserved BNO POSE: %s\n", scenario.c_str());
}
