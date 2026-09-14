// Exercise the real M08 command parser and controller using synthetic, independent
// yaw/pitch feedback and an open-loop carriage. The gains below are test fixtures,
// not mechanical calibration or a claim about real carriage travel in millimeters.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
unsigned checks = 0;
bool tracking = false, wasRunning[3] = {};
uint32_t firstMotionAt[3] = {}, lastArrivalAt[3] = {};
bool sawAllThree = false;
void require(bool ok, const char *expression, int line) {
    ++checks;
    if (!ok) {
        std::fprintf(stderr, "FAIL line %d: %s\n%s", line, expression, Serial.output.c_str());
        std::exit(EXIT_FAILURE);
    }
}
#define CHECK(condition) require((condition), #condition, __LINE__)
size_t occurrences(const char *text) {
    size_t count = 0, offset = 0;
    while ((offset = Serial.output.find(text, offset)) != std::string::npos) { ++count; offset += strlen(text); }
    return count;
}
bool printed(const char *text) { return Serial.output.find(text) != std::string::npos; }
void independentTick(uint32_t now) {
    motionWatchdog.check(now);
    if (!tracking) return;
    bool all = true;
    for (unsigned index = 0; index < 3; ++index) {
        const bool running = simulated::motors[index].drive != simulated::Drive::IDLE;
        all = all && running;
        if (running && !firstMotionAt[index]) firstMotionAt[index] = now;
        if (!running && wasRunning[index]) lastArrivalAt[index] = now;
        wasRunning[index] = running;
    }
    sawAllThree = sawAllThree || all;
}
void resetMotionCapture() {
    for (unsigned i = 0; i < 3; ++i) {
        firstMotionAt[i] = lastArrivalAt[i] = 0;
        wasRunning[i] = false;
    }
    sawAllThree = false;
    tracking = true;
}
void advanceLoops(uint32_t duration) {
    const uint32_t until = millis() + duration;
    while (millis() < until) loop();
}
void queueLine(const std::string &command) { simulated::serialInput += command + "\n"; }
void drainInput() {
    const uint32_t started = millis();
    while (!simulated::serialInput.empty() && millis() - started < 1000) loop();
    CHECK(simulated::serialInput.empty());
}
void waitForResult(size_t previousResults, uint32_t deadline = 100000) {
    const uint32_t started = millis();
    while (occurrences("FINAL RESULT: ") == previousResults && millis() - started < deadline) loop();
    CHECK(occurrences("FINAL RESULT: ") == previousResults + 1);
    while (txOffset < txLength && millis() - started < deadline + 5000) loop();
    CHECK(txOffset == txLength);
    CHECK(finalPrinted);
    for (auto &motor : simulated::motors) CHECK(motor.drive == simulated::Drive::IDLE);
}
void execute(const std::string &command) {
    const size_t results = occurrences("FINAL RESULT: ");
    resetMotionCapture();
    queueLine(command);
    waitForResult(results);
    CHECK(phase == Phase::COMPLETE);
    CHECK(!motionWatchdog.tripped());
}
void checkPose(double yaw, double pitch, int32_t carriageSteps) {
    CHECK(fabs(shortestDifference(yaw, simulated::actualYaw())) <= TOLERANCE_DEG);
    CHECK(fabs(pitch - simulated::actualPitch()) <= TOLERANCE_DEG);
    CHECK(static_cast<int32_t>(simulated::motors[2].position) == carriageSteps);
    CHECK(yawAxis.settled && pitchAxis.settled);
    CHECK(millis() - settleStarted >= SETTLE_MS);
}
void startup(bool injectBusy = false, bool lowerGain = false) {
    simulated::baselineYaw = 70;
    simulated::baselinePitch = 45;
    simulated::independentTick = independentTick;
    setup();
    if (lowerGain) simulated::motors[0].degreesPerStep = simulated::motors[1].degreesPerStep = 0.0075;
    if (injectBusy) queueLine("POSE 45 25 450");
    waitForResult(0);
    CHECK(phase == Phase::COMPLETE && referenceSet);
    checkPose(0, 0, 0);
    CHECK(yawAxis.confirmed);
    CHECK(simulated::connectedPins.size() == 3);
    CHECK(simulated::connectedPins[0] == 33 && simulated::connectedPins[1] == 12 && simulated::connectedPins[2] == 22);
    CHECK(yawMotor->directionPin == 32 && pitchMotor->directionPin == 26);
    CHECK(engine.motors[2].directionPin == 21);
    CHECK(POSITIVE_STEP_PITCH_SIGN == -1 && TRIAL_POSITIVE_STEP_YAW_SIGN == -1);
    CHECK(Wire.frequency == 0 && Wire1.sda == 4 && Wire1.scl == 5);
    CHECK(Wire1.frequency == 100000 && Wire1.timeout == 50);
    for (uint8_t address : simulated::addressedSensors) CHECK(address == 0x4A);
    for (const auto &command : simulated::commands) CHECK(command.stepPin != 22);
    const size_t commands = simulated::commands.size();
    const unsigned reads = simulated::bnoReads;
    advanceLoops(100);
    CHECK(simulated::commands.size() == commands && simulated::bnoReads > reads);
}
void assertRejected(const std::string &command) {
    const size_t commands = simulated::commands.size(), results = occurrences("FINAL RESULT: ");
    const double carriageBefore = simulated::motors[2].position;
    queueLine(command); drainInput(); advanceLoops(100);
    CHECK(phase == Phase::COMPLETE && finalPrinted);
    CHECK(simulated::commands.size() == commands);
    CHECK(occurrences("FINAL RESULT: ") == results);
    CHECK(simulated::motors[2].position == carriageBefore);
}
void waitForAllThree() {
    const uint32_t started = millis();
    while (!sawAllThree && !finalPrinted && millis() - started < 5000) loop();
    CHECK(sawAllThree && !finalPrinted);
}
} // namespace

int main(int argc, char **argv) {
    if (argc != 2) return EXIT_FAILURE;
    const std::string scenario = argv[1];
    startup(scenario == "startup_busy", scenario == "lower_gain");
    const size_t startupCommands = simulated::commands.size();

    if (scenario == "coordinated" || scenario == "lower_gain") {
        const uint32_t started = millis();
        execute("POSE 45 25 450");
        checkPose(45, 25, 450);
        CHECK(sawAllThree);
        const uint32_t first = *std::min_element(firstMotionAt, firstMotionAt + 3);
        const uint32_t last = *std::max_element(firstMotionAt, firstMotionAt + 3);
        CHECK(first > started && last - first <= 15);
        CHECK(*std::min_element(lastArrivalAt, lastArrivalAt + 3) > first);
        const uint32_t spread = *std::max_element(lastArrivalAt, lastArrivalAt + 3) -
                                *std::min_element(lastArrivalAt, lastArrivalAt + 3);
        CHECK(spread <= std::max<uint32_t>(1500, (millis() - started) / 4));
        bool scaled = false, sawCarriage = false;
        for (size_t i = startupCommands; i < simulated::commands.size(); ++i) {
            const auto &command = simulated::commands[i];
            CHECK(command.at - command.sampleAt < BNO_STALE_MS);
            CHECK(!command.wasBraking);
            if (command.stepPin == 22) {
                sawCarriage = true;
                CHECK(!command.continuous && command.steps == 450);
                CHECK(command.speed > 0 && command.speed <= 1000 && command.acceleration == 1000);
                scaled = scaled || command.speed < 1000;
            } else if (command.continuous) {
                CHECK(command.speed > 0 && command.speed <= slewSpeed(command.stepPin == 12));
                scaled = scaled || command.speed < slewSpeed(command.stepPin == 12);
            }
        }
        CHECK(sawCarriage && scaled);
        CHECK(plannedDurationSeconds > 0);
        std::printf("ARRIVALS yaw=%lu pitch=%lu carriage=%lu spread_ms=%lu elapsed_ms=%lu planned_s=%.3f\n",
            static_cast<unsigned long>(lastArrivalAt[0] - started),
            static_cast<unsigned long>(lastArrivalAt[1] - started),
            static_cast<unsigned long>(lastArrivalAt[2] - started),
            static_cast<unsigned long>(spread), static_cast<unsigned long>(millis() - started), plannedDurationSeconds);
    } else if (scenario == "no_op") {
        const size_t results = occurrences("FINAL RESULT: ");
        queueLine("POSE 0 0 0"); drainInput(); advanceLoops(100);
        CHECK(phase == Phase::COMPLETE && occurrences("FINAL RESULT: ") == results);
        CHECK(printed("POSE ALREADY AT TARGET")); checkPose(0, 0, 0);
        CHECK(simulated::commands.size() == startupCommands);
    } else if (scenario == "carriage_only") {
        execute("POSE 0 0 250"); checkPose(0, 0, 250);
        CHECK(!firstMotionAt[0] && !firstMotionAt[1] && firstMotionAt[2]);
        CHECK(simulated::commands.size() == startupCommands + 1);
    } else if (scenario == "yaw_only") {
        execute("POSE 30 0 0"); checkPose(30, 0, 0);
        CHECK(firstMotionAt[0] && !firstMotionAt[1] && !firstMotionAt[2]);
    } else if (scenario == "pitch_only") {
        execute("POSE 0 15 0"); checkPose(0, 15, 0);
        CHECK(!firstMotionAt[0] && firstMotionAt[1] && !firstMotionAt[2]);
    } else if (scenario == "relative") {
        execute("POSE 350 -10 -200"); checkPose(350, -10, -200);
        const double yawBefore = simulated::actualYaw(), pitchBefore = simulated::actualPitch();
        execute("MOVE 20 15 350");
        checkPose(wrap360(yawBefore + 20), pitchBefore + 15, 150);
        CHECK(sawAllThree);
    } else if (scenario == "small_relative") {
        const double yawBefore = simulated::actualYaw(), pitchBefore = simulated::actualPitch();
        execute("MOVE 2 -2 100"); checkPose(wrap360(yawBefore + 2), pitchBefore - 2, 100);
        CHECK(sawAllThree);
    } else if (scenario == "repeated_absolute") {
        execute("POSE 20 10 300"); checkPose(20, 10, 300);
        const size_t before = simulated::commands.size();
        execute("POSE 350 -5 -200"); checkPose(350, -5, -200);
        bool sawDelta = false;
        for (size_t i = before; i < simulated::commands.size(); ++i)
            if (simulated::commands[i].stepPin == 22) sawDelta = simulated::commands[i].steps == -500;
        CHECK(sawDelta && sawAllThree);
    } else if (scenario == "invalid") {
        for (const std::string &command : {"POSE nan 0 0", "POSE inf 0 0", "POSE 0 -inf 0", "POSE 1e999 0 0",
            "POSE 1junk 0 0", "POSE 1 2 3junk", "POSE 1 2 1.5", "POSE 1 2 1e2", "POSE 1 2",
            "POSE 1 2 3 4", "POSE 1 2 2147483648", "POSE 1 2 -2147483649",
            "POSE 1 2 999999999999999999999999999999", "UNKNOWN 1 2 3",
            "POSE 0 0 501", "POSE 0 0 -501", "POSE 0 75 0", "POSE 0 -75 0"}) assertRejected(command);
        assertRejected(std::string(300, 'P'));
        execute("POSE 10 5 100\r"); checkPose(10, 5, 100); // Parser recovers after a discarded oversized line.
    } else if (scenario == "relative_bounds") {
        execute("POSE 10 5 450"); checkPose(10, 5, 450);
        assertRejected("MOVE 0 0 51");
        assertRejected("MOVE 0 80 0");
        assertRejected("MOVE 0 0 2147483647");
        assertRejected("MOVE 270 0 0");
        assertRejected("MOVE 360 0 0");
        assertRejected("MOVE 180 0 0");
        assertRejected("MOVE -181 0 0");
        execute("MOVE 0 0 -900"); checkPose(10, 5, -450);
    } else if (scenario == "busy") {
        const size_t before = occurrences("FINAL RESULT: ");
        resetMotionCapture(); queueLine("POSE 45 25 450"); drainInput(); waitForAllThree();
        queueLine("POSE 350 -10 -300"); drainInput();
        waitForResult(before); CHECK(phase == Phase::COMPLETE); checkPose(45, 25, 450);
        const size_t commands = simulated::commands.size();
        advanceLoops(1500); CHECK(simulated::commands.size() == commands);
    } else if (scenario == "startup_busy") {
        CHECK(simulated::commands.size() == startupCommands);
        execute("POSE 10 5 100"); checkPose(10, 5, 100);
    } else if (scenario == "fragmented") {
        simulated::serialInput = "POSE 20 10 250"; drainInput(); advanceLoops(500);
        CHECK(phase == Phase::COMPLETE && simulated::commands.size() == startupCommands);
        const size_t before = occurrences("FINAL RESULT: ");
        simulated::serialInput = "\r\n";
        waitForResult(before); CHECK(phase == Phase::COMPLETE); checkPose(20, 10, 250);
    } else if (scenario == "idle_feedback") {
        const unsigned reads = simulated::bnoReads;
        simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 60;
        advanceLoops(100);
        CHECK(phase == Phase::COMPLETE && simulated::bnoReads > reads);
        CHECK(!motionWatchdog.tripped());
        simulated::accuracy = 1; advanceLoops(100);
        assertRejected("POSE 20 10 250");
        simulated::accuracy = 3; advanceLoops(100);
        execute("POSE 20 10 250"); checkPose(20, 10, 250);
    } else if (scenario == "pitch_low_entry" || scenario == "pitch_accuracy_drop") {
        if (scenario == "pitch_low_entry") { simulated::accuracy = 0; advanceLoops(100); }
        const size_t before = occurrences("FINAL RESULT: ");
        resetMotionCapture(); queueLine("MOVE 0 8 0"); drainInput();
        CHECK(!yawRequired && !accuracyRequired && !finalPrinted);
        while (!pitchMotor->isRunning() && !finalPrinted) loop();
        simulated::accuracy = 0;
        waitForResult(before); CHECK(phase == Phase::COMPLETE);
        CHECK(!firstMotionAt[0] && firstMotionAt[1] && !firstMotionAt[2]);
        CHECK(fabs(orientation.roll - 8) <= TOLERANCE_DEG && !motionWatchdog.tripped());
        assertRejected("MOVE 5 0 0");
    } else if (scenario == "carriage_accuracy_brief" || scenario == "carriage_accuracy_sustained") {
        const size_t before = occurrences("FINAL RESULT: ");
        resetMotionCapture(); queueLine("MOVE 0 0 450"); drainInput();
        CHECK(!yawRequired && accuracyRequired && !finalPrinted);
        simulated::accuracy = 0; advanceLoops(400);
        CHECK(!finalPrinted && accuracyGrace.low);
        if (scenario == "carriage_accuracy_brief") simulated::accuracy = 3;
        waitForResult(before);
        CHECK(phase == (scenario == "carriage_accuracy_brief" ? Phase::COMPLETE : Phase::ABORTED));
        if (scenario == "carriage_accuracy_sustained") CHECK(printed("accuracy below 2 continuously for 1000 ms"));
    } else if (scenario == "idle_stale" || scenario == "idle_reset") {
        const size_t before = occurrences("FINAL RESULT: ");
        if (scenario == "idle_stale") {
            simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 1000;
        } else simulated::bnoResetAt = millis();
        waitForResult(before);
        CHECK(phase == Phase::ABORTED && simulated::commands.size() == startupCommands);
        if (scenario == "idle_stale") CHECK(printed("stale while idle"));
        if (scenario == "idle_reset") CHECK(printed("reset after north reference"));
        simulated::bnoPauseEnd = millis(); queueLine("POSE 20 10 250"); advanceLoops(1000);
        CHECK(phase == Phase::ABORTED && simulated::commands.size() == startupCommands);
    } else if (scenario == "idle_motion") {
        simulated::yawDisturbance = 5; simulated::pitchDisturbance = -3;
        advanceLoops(100);
        CHECK(phase == Phase::COMPLETE && simulated::commands.size() == startupCommands);
        execute("POSE 0 0 100"); checkPose(0, 0, 100);
        CHECK(sawAllThree);
    } else if (scenario == "missing_timing") {
        yawPulsesPerDegree = 0;
        assertRejected("POSE 10 0 100");
        pitchPulsesPerDegree = 0;
        assertRejected("POSE 0 10 100");
        execute("POSE 0 0 100"); checkPose(0, 0, 100);
        CHECK(!firstMotionAt[0] && !firstMotionAt[1] && firstMotionAt[2]);
    } else if (scenario == "blocked_output") {
        const size_t before = occurrences("FINAL RESULT: ");
        resetMotionCapture(); queueLine("POSE 45 25 450"); drainInput(); waitForAllThree();
        simulated::serialWriteSpace = 0;
        const uint32_t started = millis();
        while (!finalPrinted && millis() - started < 100000) loop();
        CHECK(finalPrinted && phase == Phase::COMPLETE);
        CHECK(occurrences("FINAL RESULT: ") == before && txOffset < txLength);
        const unsigned reads = simulated::bnoReads;
        advanceLoops(500);
        CHECK(phase == Phase::COMPLETE && simulated::bnoReads > reads);
        CHECK(bnoMaxGap < BNO_STALE_MS && fresh(millis()));
        for (auto &motor : simulated::motors) CHECK(motor.drive == simulated::Drive::IDLE);
        simulated::serialWriteSpace = 128;
        waitForResult(before); checkPose(45, 25, 450);
        execute("POSE 0 0 0"); checkPose(0, 0, 0);
    } else if (scenario == "abort" || scenario == "stale" || scenario == "blocked_bno" || scenario == "reset") {
        const size_t before = occurrences("FINAL RESULT: ");
        resetMotionCapture(); queueLine("POSE 45 25 450"); drainInput(); waitForAllThree();
        const size_t commands = simulated::commands.size();
        const uint32_t freshAtInjection = lastBnoGood;
        const unsigned previousStops = simulated::forceStops;
        simulated::forceStops = 0;
        if (scenario == "abort") simulated::pendingSerial = 'x';
        if (scenario == "stale") { simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 1000; }
        if (scenario == "blocked_bno") simulated::blockBnoMs = 1000;
        if (scenario == "reset") simulated::bnoResetAt = millis();
        waitForResult(before);
        CHECK(phase == Phase::ABORTED);
        CHECK(simulated::forceStops >= 3);
        CHECK(simulated::commands.size() == commands);
        if (scenario == "stale" || scenario == "blocked_bno") {
            CHECK(motionWatchdog.tripped());
            CHECK(simulated::firstForceStopAt - freshAtInjection == BNO_STALE_MS);
            CHECK(lastBnoGood == freshAtInjection);
        }
        if (scenario == "abort") CHECK(printed("Operator X abort"));
        if (scenario == "reset") CHECK(printed("reset after north reference"));
        simulated::forceStops += previousStops;
        queueLine("POSE 0 0 0"); advanceLoops(2000);
        CHECK(phase == Phase::ABORTED && simulated::commands.size() == commands);
    } else if (scenario == "carriage_rejected") {
        simulated::moveRejected = true; simulated::rejectedStepPin = 22;
        const size_t before = occurrences("FINAL RESULT: ");
        queueLine("POSE 45 25 450"); waitForResult(before);
        CHECK(phase == Phase::ABORTED);
        CHECK(simulated::motors[2].position == 0);
    } else if (scenario == "carriage_timeout") {
        simulated::motors[2].neverStops = true;
        const size_t before = occurrences("FINAL RESULT: ");
        queueLine("POSE 0 0 200"); waitForResult(before);
        CHECK(phase == Phase::ABORTED);
    } else {
        std::fprintf(stderr, "Unknown M08 scenario: %s\n", scenario.c_str());
        return EXIT_FAILURE;
    }
    for (auto &motor : simulated::motors) CHECK(motor.drive == simulated::Drive::IDLE);
    std::printf("PASS M08: %s (%u checks, %zu commands)\n", scenario.c_str(), checks, simulated::commands.size());
}
