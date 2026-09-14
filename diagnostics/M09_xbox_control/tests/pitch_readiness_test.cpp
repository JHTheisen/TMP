// Actual firmware decisions with synthetic BNO/motor feedback, including an
// independent watchdog tick during blocked foreground reads.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
unsigned checks = 0;
void require(bool ok, const char *expr, int line) {
    ++checks;
    if (!ok) { std::fprintf(stderr, "FAIL %d: %s\n%s", line, expr, Serial.output.c_str()); std::exit(1); }
}
#define CHECK(x) require((x), #x, __LINE__)
void tick(uint32_t now) { motionWatchdog.check(now); }
void advance(uint32_t ms) { const uint32_t end = millis() + ms; while (millis() < end) loop(); }
void drain() { while (txOffset < txLength) loop(); }
void send(const std::string &line) {
    simulated::serialInput = line + "\n";
    while (!simulated::serialInput.empty()) loop();
}
void rejected(const std::string &line) {
    const size_t count = simulated::commands.size();
    send(line); advance(100);
    CHECK(commandIdle() && simulated::commands.size() == count);
}
}

int main(int argc, char **argv) {
    if (argc != 2) return 1;
    const std::string scenario = argv[1];
    simulated::accuracy = scenario == "accuracy_one" ? 1 : 0;
    simulated::baselinePitch = 8;
    simulated::independentTick = tick;
    setup();
    if (scenario == "blocked_uart") simulated::serialWriteSpace = 0;
    if (scenario == "invalid_baseline") simulated::invalidQuaternion = true;
    if (scenario == "stale_baseline") { simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = 40000; }
    if (scenario == "unstable_heading") simulated::noiseAmplitude = 2;
    if (scenario == "roll_mapping") simulated::rawBnoPitchNoise = 1;
    while (!finalPrinted && millis() < 30000) {
        if (scenario == "unstable_pitch") simulated::pitchDisturbance = sin(millis() * 0.03);
        if (scenario == "calibration_recovers") simulated::accuracy = millis() > 10000 ? 2 : 0;
        loop();
        if (scenario == "calibration_recovers" && referenceSet) break;
    }
    if (scenario == "calibration_recovers") {
        CHECK(referenceSet && pitchReady && northUsable && yawRequired);
        CHECK(phase == Phase::MOVING && millis() < startedAt + BASELINE_TIMEOUT_MS);
        std::puts("PASS: calibrated startup recovers within original deadline"); return 0;
    }
    if (scenario == "blocked_uart") simulated::serialWriteSpace = 128;
    drain();
    if (scenario == "invalid_baseline" || scenario == "stale_baseline" || scenario == "unstable_pitch") {
        CHECK(phase == Phase::ABORTED && !pitchReady && !referenceSet);
        CHECK(simulated::commands.empty());
        std::printf("PASS: %s\n", argv[1]); return 0;
    }
    CHECK(commandIdle() && pitchReady && !referenceSet && !northUsable && !yawRequired);
    CHECK(simulated::commands.empty() && pitchPulsesPerDegree == 0);
    CHECK(Serial.output.find("PITCH-ONLY READY") != std::string::npos);
    CHECK(millis() - startedAt >= BASELINE_TIMEOUT_MS);
    if (scenario == "roll_mapping") {
        CHECK(fabs(baselinePitch - 8) < 0.001);
        CHECK(orientation.pitch >= -4.001 && orientation.pitch <= -1.999);
        CHECK(fabs(orientation.roll - 8) < 0.001);
    }
    rejected("MOVE 5 0 0"); rejected("POSE 0 0 0");
    CHECK(Serial.output.find("yaw disabled: calibrated north baseline") != std::string::npos);
    rejected("MOVE 0 0 1"); rejected("MOVE 0 100 0");
    if (scenario == "late_accuracy") {
        simulated::accuracy = 3; advance(1500);
        CHECK(!referenceSet && !northUsable && simulated::commands.empty());
        rejected("MOVE 5 0 0");
    }
    if (scenario == "idle_stale" || scenario == "idle_reset") {
        if (scenario == "idle_stale") { simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 1000; }
        else simulated::bnoResetAt = millis();
        advance(300); CHECK(phase == Phase::ABORTED); CHECK(simulated::commands.empty());
        std::printf("PASS: %s\n", argv[1]); return 0;
    }
    // Also test the individual pitch-baseline requirements against a genuinely
    // acquired complete window; restore the window after each fault injection.
    if (scenario == "baseline_criteria") {
        window.active = true; startBaseline();
        // READY does not collect windows: collect explicitly without motion.
        finalPrinted = false; phase = Phase::BASELINE;
        while (millis() - window.started < BASELINE_MS) { serviceBno(); delay(1); }
        CHECK(stablePitchBaseline() && !stableBaseline());
        const Window saved = window;
        window.pitch.n = 29; CHECK(!stablePitchBaseline()); window = saved;
        window.firstPitch.n = 0; CHECK(!stablePitchBaseline()); window = saved;
        window.secondPitch.n = 0; CHECK(!stablePitchBaseline()); window = saved;
        window.started = millis(); CHECK(!stablePitchBaseline()); window = saved;
        window.interrupted = true; CHECK(!stablePitchBaseline()); window = saved;
        window.lastBno = millis() - WINDOW_GAP_MS - 1; CHECK(!stablePitchBaseline()); window = saved;
        window.pitch.maximum = window.pitch.minimum + 0.51; CHECK(!stablePitchBaseline()); window = saved;
        window.pitch.sumSquares += window.pitch.n * 0.16 * 0.16; CHECK(!stablePitchBaseline()); window = saved;
        window.secondPitch.sum += window.secondPitch.n * 0.21; CHECK(!stablePitchBaseline()); window = saved;
        bnoValid = false; CHECK(!stablePitchBaseline()); bnoValid = true;
        finalPrinted = true; phase = Phase::COMPLETE; window.active = false;
    }
    send(scenario == "unstable_heading" ? "MOVE 0 -8 0" : "POSE 70 0 0");
    CHECK(!finalPrinted && poseActive && !yawRequired);
    while (!pitchMotor->isRunning() && !finalPrinted) loop();
    CHECK(pitchMotor->isRunning() && !yawMotor->isRunning());
    const uint32_t lastFresh = lastBnoGood, faultAt = millis();
    const size_t commandsAtFault = simulated::commands.size();
    // Startup READY issues no forceStop; reset the fixture counter to identify
    // the independent stop time even if later fixtures change that detail.
    simulated::forceStops = 0;
    if (scenario == "stale") { simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 1000; }
    if (scenario == "blocked_bno") simulated::blockBnoMs = 1000;
    if (scenario == "invalid_motion") simulated::invalidQuaternion = true;
    if (scenario == "wrong_report") simulated::wrongReportType = true;
    if (scenario == "reset") simulated::bnoResetAt = millis();
    if (scenario == "reset_in_poll") simulated::resetDuringPoll = true;
    if (scenario == "abort") simulated::pendingSerial = 'X';
    if (scenario == "wrong_direction") simulated::motors[1].physicalSign = 1;
    if (scenario == "no_progress") simulated::motors[1].frozen = true;
    if (scenario == "pitch_guard") orientation.roll = 75.1;
    if (scenario == "yaw_guard") heading.continuous = pitchReadyYaw + 185.1;
    if (scenario == "timeout") controlStartedAt = millis() - LEG_TIMEOUT_MS;
    while (!finalPrinted && millis() - faultAt < 100000) loop();
    CHECK(finalPrinted); drain();
    const bool fault = scenario == "stale" || scenario == "blocked_bno" || scenario == "invalid_motion" ||
        scenario == "wrong_report" || scenario == "reset" || scenario == "reset_in_poll" || scenario == "abort" ||
        scenario == "wrong_direction" || scenario == "no_progress" || scenario == "pitch_guard" ||
        scenario == "yaw_guard" || scenario == "timeout";
    CHECK(phase == (fault ? Phase::ABORTED : Phase::COMPLETE));
    CHECK(!yawMotor->isRunning() && !pitchMotor->isRunning() && !carriageMotor->isRunning());
    CHECK(simulated::motors[0].position == 0 && simulated::motors[2].position == 0);
    for (const auto &cmd : simulated::commands) {
        CHECK(cmd.stepPin == 12 && cmd.at - cmd.sampleAt < BNO_STALE_MS);
        CHECK(!cmd.wasBraking && !cmd.beforeStoppedSample);
        CHECK(cmd.speed <= PITCH_SLEW_SPEED_HZ);
    }
    if (scenario == "stale" || scenario == "blocked_bno" || scenario == "invalid_motion" || scenario == "wrong_report") {
        CHECK(motionWatchdog.tripped());
        CHECK(simulated::firstForceStopAt - lastFresh == BNO_STALE_MS);
        CHECK(lastBnoGood == lastFresh && simulated::commands.size() == commandsAtFault);
    }
    if (!fault) {
        CHECK(pitchAxis.settled && !yawAxis.settled && !referenceSet && !northUsable);
        CHECK(fabs(orientation.roll) <= TOLERANCE_DEG && settleSamples >= 30);
        CHECK(millis() - settleStarted >= SETTLE_MS && !motionWatchdog.tripped());
        CHECK(millis() - faultAt > BNO_ACCURACY_GRACE_MS);
        if (scenario == "roll_mapping") {
            CHECK(fabs(pitchAxis.current - orientation.roll) < 0.01);
            CHECK(fabs(pitchStartAngle - 8) < 0.001);
            CHECK(pitchPulsesPerDegree > 99 && pitchPulsesPerDegree < 101);
            CHECK(pitchAxis.peakAngularSpeed > 0);
            CHECK(orientation.pitch >= -4.001 && orientation.pitch <= -1.999);
            Serial.output.clear(); telemetry(); drain();
            char expected[40]; snprintf(expected, sizeof(expected), "pitch=%.3f ", orientation.roll);
            CHECK(Serial.output.find(expected) != std::string::npos);
        }
        send("MOVE 0 2 0"); CHECK(!finalPrinted);
        while (!finalPrinted && millis() - faultAt < 100000) loop();
        CHECK(commandIdle() && fabs(orientation.roll - 2) <= TOLERANCE_DEG);
        if (scenario == "roll_mapping") CHECK(orientation.pitch >= -4.001 && orientation.pitch <= -1.999);
    } else {
        const size_t count = simulated::commands.size();
        send("MOVE 0 2 0"); CHECK(phase == Phase::ABORTED && simulated::commands.size() == count);
    }
    std::printf("PASS: pitch readiness %s (%u checks)\n", argv[1], checks);
}
