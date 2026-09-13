// Compile the actual firmware against two independent synthetic motors and BNO.
// This tests decisions and asynchronous sequencing, not physical stopping margins,
// torque, fusion lag, I2C electrical faults, or ESP32 task scheduling.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
unsigned checks = 0;
void require(bool ok, const char *expression, int line) {
    ++checks;
    if (!ok) {
        std::fprintf(stderr, "FAIL line %d: %s\n%s", line, expression, Serial.output.c_str());
        std::exit(EXIT_FAILURE);
    }
}
#define CHECK(condition) require((condition), #condition, __LINE__)
bool printed(const char *text) { return Serial.output.find(text) != std::string::npos; }
bool oneOf(const std::string &value, std::initializer_list<const char *> options) {
    for (const char *option : options) if (value == option) return true;
    return false;
}
size_t occurrences(const char *text) {
    size_t count = 0, offset = 0;
    while ((offset = Serial.output.find(text, offset)) != std::string::npos) { ++count; offset += strlen(text); }
    return count;
}
void independentWatchdog(uint32_t now) { motionWatchdog.check(now); }
}

int main(int argc, char **argv) {
    if (argc != 2) return EXIT_FAILURE;
    const std::string scenario = argv[1];
    const bool successful = oneOf(scenario, {"normal", "wrap_positive", "wrap_negative", "accuracy_brief",
        "accuracy_repeated", "brief_gap", "fresh_after_stop", "startup_reset", "blocked_uart", "low_gain", "first_test"});
    simulated::baselinePitch = 45;
    if (scenario == "wrap_positive") { simulated::baselineYaw = 359.2; simulated::baselinePitch = 5; }
    if (scenario == "wrap_negative") { simulated::baselineYaw = 0.8; simulated::baselinePitch = -5; }
    if (scenario == "first_test") { simulated::baselineYaw = 20; simulated::baselinePitch = 8; }
    if (scenario == "wrong_direction") simulated::motors[0].physicalSign = 1;
    if (scenario == "no_probe_progress") simulated::motors[0].frozen = true;
    if (scenario == "axis_init") simulated::axisInitFails = true;
    if (scenario == "report_init") simulated::reportInitFails = true;
    if (scenario == "startup_reset") simulated::bnoResetAt = 1500;
    if (scenario == "blocked_uart") simulated::serialWriteSpace = 0;
    simulated::independentTick = independentWatchdog;
    setup();
    if (scenario == "low_gain") simulated::motors[0].degreesPerStep = simulated::motors[1].degreesPerStep = 0.0075;
    if (scenario == "startup_abort") simulated::pendingSerial = 'x';

    bool injected = false, sawBothSlew = false, sawIndependentModes = false, sawBraking = false;
    bool sawAccuracyLow = false, survivedAccuracyLow = false, sawStoppedGap = false;
    bool sawUARTBacklog = false;
    uint32_t injectedAt = 0, freshnessAtInjection = 0, lastProbeSample = 0;
    size_t commandsAtInjection = 0, checkedCommands = 0;
    double previousProbeHeading = heading.continuous, largestProbeSample = 0;
    uint32_t probesAtConfirmation = 0;
    while (!finalPrinted && millis() < 120000) {
        if (scenario == "baseline_low" && phase == Phase::BASELINE) simulated::accuracy = 1;
        if (scenario == "baseline_intermittent" && phase == Phase::BASELINE)
            simulated::accuracy = (millis() - window.started) < 100 ? 1 : 3;

        // Immediate pre-command guards are seeded directly; dynamic scenarios
        // inject only after real simultaneous continuous motion has begun.
        const bool firstReference = referenceSet && simulated::commands.empty();
        const bool bothSlew = yawAxis.motion == Motion::SLEW && pitchAxis.motion == Motion::SLEW;
        const bool seedNow = firstReference && oneOf(scenario, {"yaw_guard_positive", "yaw_guard_negative",
            "pitch_guard_positive", "pitch_guard_negative", "overall_timeout", "move_rejected", "pitch_slew_rejected"});
        const bool movingNow = bothSlew && oneOf(scenario, {"accuracy_brief", "accuracy_repeated", "accuracy_sustained",
            "brief_gap", "stale_gap", "blocked_bno", "reset", "reset_in_poll", "invalid_vector", "wrong_report",
            "abort", "runaway", "no_slew_progress", "frozen_feedback", "settle_running"});
        const bool brakeNow = yawAxis.motion == Motion::BRAKING && scenario == "braking_timeout";
        const bool burstNow = yawAxis.finiteActive && scenario == "burst_timeout";
        const bool slewCommandNow = scenario == "yaw_slew_rejected" && yawAxis.confirmed && yawAxis.observing && !yawAxis.slewStarts;
        const bool stoppedNow = scenario == "fresh_after_stop" && yawAxis.motion == Motion::BRAKING &&
            !yawMotor->isRunning();
        if (!injected && (seedNow || movingNow || brakeNow || burstNow || stoppedNow || slewCommandNow)) {
            injected = true; injectedAt = millis(); freshnessAtInjection = lastBnoGood;
            commandsAtInjection = simulated::commands.size();
            if (oneOf(scenario, {"accuracy_brief", "accuracy_repeated", "accuracy_sustained"})) simulated::accuracy = 1;
            if (oneOf(scenario, {"brief_gap", "stale_gap", "fresh_after_stop"})) {
                simulated::bnoPauseStart = millis();
                simulated::bnoPauseEnd = millis() + (scenario == "stale_gap" ? 10000 : 60);
            }
            if (scenario == "blocked_bno") simulated::blockBnoMs = 1000;
            if (scenario == "reset") simulated::bnoResetAt = millis();
            if (scenario == "reset_in_poll") simulated::resetDuringPoll = true;
            if (scenario == "invalid_vector") simulated::invalidQuaternion = true;
            if (scenario == "wrong_report") simulated::wrongReportType = true;
            if (scenario == "abort") simulated::pendingSerial = 'X';
            if (scenario == "runaway") simulated::motors[0].physicalSign *= -1;
            if (scenario == "no_slew_progress") simulated::motors[0].frozen = true;
            if (scenario == "frozen_feedback") simulated::frozenFeedback = true;
            if (scenario == "settle_running") {
                // A target-valued fresh sample and an old settle timer are
                // insufficient while the asynchronous motors still run.
                heading.continuous = northTargetContinuous; orientation.pitch = 0;
                ++bnoHealth.freshSamples; phase = Phase::SETTLING;
                settleStarted = millis() - SETTLE_MS; settleSamples = 100;
                serviceAxes();
                CHECK(!finalPrinted && yawMotor->isRunning() && pitchMotor->isRunning());
                CHECK(yawAxis.motion == Motion::BRAKING && pitchAxis.motion == Motion::BRAKING);
                CHECK(settleSamples == 0);
                simulated::pendingSerial = 'X';
            }
            if (scenario == "yaw_guard_positive") heading.continuous = northTargetContinuous + 185.1;
            if (scenario == "yaw_guard_negative") heading.continuous = northTargetContinuous - 185.1;
            if (scenario == "pitch_guard_positive") orientation.pitch = 75.1;
            if (scenario == "pitch_guard_negative") orientation.pitch = -75.1;
            if (scenario == "overall_timeout") controlStartedAt = millis() - LEG_TIMEOUT_MS;
            if (scenario == "move_rejected") simulated::moveRejected = true;
            if (scenario == "yaw_slew_rejected" || scenario == "pitch_slew_rejected") {
                simulated::moveRejected = true;
                simulated::rejectedStepPin = scenario == "yaw_slew_rejected" ? 33 : 12;
            }
            if (scenario == "braking_timeout" || scenario == "burst_timeout") simulated::motors[0].neverStops = true;
        }
        if (injected && scenario == "accuracy_brief" && millis() - injectedAt >= 400) simulated::accuracy = 3;
        if (injected && scenario == "accuracy_repeated") {
            const uint32_t age = millis() - injectedAt;
            simulated::accuracy = age < 1400 && age % 700 < 400 ? 1 : 3;
        }

        loop();
        sawUARTBacklog = sawUARTBacklog || (!finalPrinted && txLength > txOffset && simulated::serialWriteSpace == 0);
        sawBothSlew = sawBothSlew || (yawAxis.motion == Motion::SLEW && pitchAxis.motion == Motion::SLEW);
        sawIndependentModes = sawIndependentModes || (referenceSet && yawAxis.motion != pitchAxis.motion);
        sawBraking = sawBraking || yawAxis.motion == Motion::BRAKING || pitchAxis.motion == Motion::BRAKING;
        sawAccuracyLow = sawAccuracyLow || accuracyGrace.low;
        if (injected && millis() - injectedAt >= 300 && accuracyGrace.low && !finalPrinted) survivedAccuracyLow = true;
        if (!yawAxis.confirmed && bnoHealth.freshSamples != lastProbeSample) {
            if (referenceSet) largestProbeSample = std::max(largestProbeSample, fabs(heading.continuous - previousProbeHeading));
            previousProbeHeading = heading.continuous; lastProbeSample = bnoHealth.freshSamples;
        }
        if (yawAxis.confirmed && !probesAtConfirmation) probesAtConfirmation = yawAxis.bursts;
        if (scenario == "fresh_after_stop" && injected && millis() < simulated::bnoPauseEnd) {
            CHECK(simulated::commands.size() == commandsAtInjection);
            CHECK(!yawMotor->isRunning()); sawStoppedGap = true;
        }
        while (checkedCommands < simulated::commands.size()) {
            const auto &command = simulated::commands[checkedCommands++];
            CHECK(command.stepPin == 33 || command.stepPin == 12);
            CHECK(!command.wasBraking && !command.beforeStoppedSample);
            CHECK(command.at >= startedAt + STARTUP_MS + BASELINE_MS);
            CHECK(command.at - command.sampleAt < BNO_STALE_MS);
            if (command.continuous) {
                CHECK(command.speed == slewSpeed(command.stepPin == 12));
                CHECK(command.acceleration == slewAcceleration(command.stepPin == 12));
                if (command.stepPin == 33) CHECK(yawAxis.confirmed);
            } else {
                CHECK(abs(command.steps) <= MAX_BURST_STEPS);
                if (command.stepPin == 33 && !yawAxis.confirmed) {
                    CHECK(abs(command.steps) <= TRIAL_BURST_STEPS);
                    CHECK(command.speed == MIN_SPEED_HZ);
                }
            }
        }
    }

    CHECK(finalPrinted);
    CHECK(occurrences("FINAL RESULT: ") == 1);
    CHECK(simulated::connectedPins.size() == 2);
    CHECK(simulated::connectedPins[0] == 33 && simulated::connectedPins[1] == 12);
    CHECK(simulated::highPins.empty() && Wire.frequency == 0);
    for (const uint8_t address : simulated::addressedSensors) CHECK(address == 0x4A);
    CHECK(!yawMotor || !yawMotor->isRunning());
    CHECK(!pitchMotor || !pitchMotor->isRunning());
    if (successful) {
        CHECK(phase == Phase::COMPLETE && printed("FINAL RESULT: PASS"));
        CHECK(millis() - settleStarted >= SETTLE_MS && settleSamples >= 30);
        CHECK(yawAxis.settled && pitchAxis.settled && concurrentMotion);
        CHECK(fabs(yawAxis.error) <= TOLERANCE_DEG && fabs(pitchAxis.error) <= TOLERANCE_DEG);
        CHECK(fabs(shortestDifference(0, simulated::actualYaw())) <= TOLERANCE_DEG);
        CHECK(fabs(simulated::actualPitch()) <= TOLERANCE_DEG);
        CHECK(yawAxis.confirmed && probesAtConfirmation >= 4 && largestProbeSample < DIRECTION_RESPONSE_DEG);
        CHECK(yawAxis.motionSamples && pitchAxis.motionSamples);
        CHECK(yawMotor != pitchMotor && yawMotor->directionPin == 32 && pitchMotor->directionPin == 26);
        CHECK(Wire1.sda == 4 && Wire1.scl == 5 && Wire1.frequency == 100000 && Wire1.timeout == 50);
        CHECK(bno.address == 0x4A && bno.wire == &Wire1);
        CHECK(!motionWatchdog.tripped());
        if (!oneOf(scenario, {"wrap_positive", "wrap_negative"})) {
            if (scenario != "first_test") CHECK(sawBothSlew);
            CHECK(sawIndependentModes && sawBraking);
            CHECK(yawAxis.slewStarts == 1 && pitchAxis.slewStarts == 1);
            CHECK(yawAxis.bursts > probesAtConfirmation && pitchAxis.bursts > 0);
            CHECK(simulated::gentleStops == 2);
            CHECK(yawAxis.peakAngularSpeed > 1 && pitchAxis.peakAngularSpeed > 1);
        }
        if (oneOf(scenario, {"wrap_positive", "wrap_negative"})) {
            CHECK(yawAxis.slewStarts == 0);
            CHECK(fabs(northTargetContinuous) < 1);
            const int requiredDirection = scenario == "wrap_positive" ? -1 : 1;
            for (const auto &command : simulated::commands)
                if (command.stepPin == 33) CHECK(command.steps * requiredDirection > 0);
        }
        if (scenario == "accuracy_brief" || scenario == "accuracy_repeated") {
            CHECK(injected && sawAccuracyLow && survivedAccuracyLow && !accuracyGrace.low);
            CHECK(accuracyGrace.episodes == (scenario == "accuracy_brief" ? 1U : 2U));
            CHECK(accuracyGrace.recoveries == accuracyGrace.episodes);
            CHECK(accuracyGrace.longestMs >= 390 && accuracyGrace.longestMs <= 410);
        }
        if (scenario == "fresh_after_stop") CHECK(injected && sawStoppedGap);
        if (scenario == "blocked_uart") CHECK(bnoMaxGap <= 10 && sawUARTBacklog);
    } else {
        CHECK(phase == Phase::ABORTED && printed("FINAL RESULT: FAIL"));
        if (scenario == "baseline_low" || scenario == "baseline_intermittent") {
            CHECK(!referenceSet && simulated::commands.empty());
            CHECK(printed("baseline unavailable"));
        }
        if (scenario == "accuracy_sustained") {
            CHECK(injected && survivedAccuracyLow && printed("accuracy below 2 continuously"));
            CHECK(millis() - injectedAt >= BNO_ACCURACY_GRACE_MS);
            CHECK(millis() - injectedAt < BNO_ACCURACY_GRACE_MS + 50);
            CHECK(!motionWatchdog.tripped());
        }
        if (oneOf(scenario, {"stale_gap", "blocked_bno", "invalid_vector", "wrong_report"})) {
            CHECK(injected && motionWatchdog.tripped() && printed("feedback stale"));
            CHECK(simulated::firstForceStopAt - freshnessAtInjection == BNO_STALE_MS);
            CHECK(simulated::commands.size() == commandsAtInjection);
            CHECK(lastBnoGood == freshnessAtInjection); // Recovery cannot erase the outage.
            if (scenario == "blocked_bno") CHECK(bnoMaxGap >= 1000);
        }
        if (scenario == "reset" || scenario == "reset_in_poll") CHECK(injected && printed("reset after north reference"));
        if (scenario == "abort" || scenario == "startup_abort" || scenario == "settle_running") CHECK(printed("Operator X abort"));
        if (scenario == "settle_running") CHECK(injected);
        if (scenario == "wrong_direction" || scenario == "runaway") CHECK(printed("wrong-direction/runaway"));
        if (scenario == "wrong_direction") CHECK(!yawAxis.confirmed && yawAxis.slewStarts == 0);
        if (scenario == "yaw_guard_positive" || scenario == "yaw_guard_negative") CHECK(printed("+/-185 degrees"));
        if (scenario == "pitch_guard_positive" || scenario == "pitch_guard_negative") CHECK(printed("+/-75 degree absolute guard"));
        if (scenario == "overall_timeout") CHECK(printed("Two-axis control timeout"));
        if (scenario == "no_probe_progress" || scenario == "no_slew_progress") CHECK(printed("No measured yaw progress"));
        // Both fresh orientation fields freeze together. Either axis's progress
        // deadline can expire first, depending on its last measured improvement.
        if (scenario == "frozen_feedback") CHECK(printed("No measured yaw progress") || printed("No measured pitch progress"));
        if (scenario == "no_slew_progress" || scenario == "frozen_feedback") CHECK(injected && millis() - injectedAt < SLEW_PROGRESS_TIMEOUT_MS + 100);
        if (scenario == "braking_timeout" || scenario == "burst_timeout") CHECK(injected && printed("braking or finite correction timed out"));
        if (scenario == "move_rejected") CHECK(injected && printed("correction rejected"));
        if (scenario == "yaw_slew_rejected") CHECK(injected && yawAxis.slewStarts == 0 && printed("yaw slew rejected"));
        if (scenario == "pitch_slew_rejected") CHECK(injected && yawAxis.bursts == 1 && printed("pitch slew rejected"));
        if (scenario == "report_init") CHECK(DiagnosticBno085::writeFailures == 1);
    }

    const auto savedOutput = Serial.output;
    const auto savedCommands = simulated::commands.size();
    const auto savedReads = simulated::bnoReads;
    for (unsigned i = 0; i < 100; ++i) loop();
    CHECK(Serial.output == savedOutput && simulated::commands.size() == savedCommands && simulated::bnoReads == savedReads);
    CHECK(!yawMotor || !yawMotor->isRunning());
    CHECK(!pitchMotor || !pitchMotor->isRunning());
    std::printf("PASS: %s (%u checks, %zu commands)\n", scenario.c_str(), checks, savedCommands);
}
