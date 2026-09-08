// Execute real M06 setup()/loop() against synthetic motor and BNO fixtures.
// No AS5600 library/stub is provided; a new encoder dependency fails compilation.
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
bool printed(const char *value) { return Serial.output.find(value) != std::string::npos; }
size_t occurrences(const std::string &value) {
    size_t count = 0, at = 0;
    while ((at = Serial.output.find(value, at)) != std::string::npos) { ++count; at += value.size(); }
    return count;
}
bool oneOf(const std::string &value, std::initializer_list<const char *> choices) {
    for (const char *choice : choices) if (value == choice) return true;
    return false;
}
}
int main(int argc, char **argv) {
    if (argc != 2) return EXIT_FAILURE;
    const std::string scenario = argv[1];
    const bool successful = oneOf(scenario, {"normal", "wrap_forward", "wrap_zero_noise", "wrap_noise",
        "high_gain", "low_gain", "noise", "allowed_noise", "slip_backlash", "disturbance", "drift",
        "transient", "startup_reset", "startup_reset_retry"});
    const bool startupFailure = oneOf(scenario, {"axis_init", "bus_init", "bno_init", "bno_ack",
        "report_init", "speed_init", "accel_init", "unstable_baseline", "startup_tilt", "startup_abort", "pre_motion_outage"});
    simulated::yawSign = TRIAL_POSITIVE_STEP_YAW_SIGN;
    if (scenario == "wrap_forward" || scenario == "guard_wrap") simulated::baselineYaw = 359.0;
    if (scenario == "wrap_zero_noise") { simulated::baselineYaw = 0.0; simulated::noiseAmplitude = 0.2; }
    if (scenario == "wrap_noise") { simulated::baselineYaw = 359.98; simulated::noiseAmplitude = 0.2; }
    if (scenario == "high_gain") simulated::degreesPerPulse = 360.0 / 1600 / 7;
    if (scenario == "low_gain") simulated::degreesPerPulse = 360.0 / 3200 / 15;
    if (scenario == "noise" || scenario == "wrong_way_noise") simulated::noiseAmplitude = 0.08;
    if (scenario == "allowed_noise") simulated::noiseAmplitude = 0.2;
    if (scenario == "slip_backlash") {
        simulated::pulseEfficiency = 0.65; simulated::backlashSteps = 12; simulated::noiseAmplitude = 0.03;
    }
    if (scenario == "drift") simulated::driftDegreesPerSecond = 0.008;
    if (scenario == "wrong_way" || scenario == "wrong_way_noise") simulated::yawSign *= -1;
    if (scenario == "frozen_motor") simulated::frozenMotor = true;
    if (scenario == "frozen_feedback") simulated::frozenFeedback = true;
    if (scenario == "axis_init") simulated::axisInitFails = true;
    if (scenario == "bus_init") simulated::busBInitFails = true;
    if (scenario == "bno_init") simulated::bnoInitFails = true;
    if (scenario == "bno_ack") simulated::bnoAckFails = true;
    if (scenario == "report_init") simulated::reportInitFails = true;
    if (scenario == "speed_init") simulated::speedFails = true;
    if (scenario == "accel_init") simulated::accelerationFails = true;
    if (scenario == "unstable_baseline") simulated::noiseAmplitude = 1.0;
    if (scenario == "startup_tilt") simulated::baselinePitch = 80;
    if (scenario == "startup_reset" || scenario == "startup_reset_retry") {
        simulated::bnoResetAt = 1000;
        if (scenario == "startup_reset_retry") simulated::reenableFailuresRemaining = 1;
    }
    setup();
    if (scenario == "startup_abort") simulated::pendingSerial = 'x';
    bool injected = false, sawReturn = false;
    uint32_t injectedAt = 0;
    size_t commandsAtInjection = 0, checkedCommands = 0;
    double outboundActual = 0;
    while (!finalPrinted && millis() < 250000) {
        if (!sawReturn && legIndex == 1) {
            sawReturn = true; outboundActual = simulated::actualYaw();
            if (scenario == "return_wrong_way") simulated::yawSign *= -1;
        }
        if (!injected && scenario == "pre_motion_outage" && referenceSet) {
            injected = true;
            simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 10000;
        }
        if (!injected && ((phase == Phase::MOVING && simulated::commands.size() >= 3 && scenario != "runaway") ||
            (scenario == "runaway" && legs[0].directionConfirmed && measuredYaw >= 1.0))) {
            injected = true; injectedAt = millis(); commandsAtInjection = simulated::commands.size();
            if (scenario == "transient" || scenario == "bno_outage") {
                simulated::bnoPauseStart = millis();
                simulated::bnoPauseEnd = millis() + (scenario == "transient" ? 60 : 10000);
            }
            if (scenario == "blocked_bno") simulated::blockBnoMs = 160;
            if (scenario == "reset") simulated::bnoResetAt = millis();
            if (scenario == "reset_in_poll") simulated::resetDuringPoll = true;
            if (scenario == "invalid_quaternion") simulated::invalidQuaternion = true;
            if (scenario == "wrong_report") simulated::wrongReportType = true;
            if (scenario == "guard_positive" || scenario == "guard_wrap") simulated::disturbanceDegrees = 6.5;
            if (scenario == "guard_negative") simulated::disturbanceDegrees = -6.5;
            if (scenario == "heading_jump") simulated::disturbanceDegrees = 90;
            if (scenario == "tilt_motion") simulated::baselinePitch = 80;
            if (scenario == "abort") simulated::pendingSerial = 'X';
            if (scenario == "motion_timeout") simulated::moveNeverFinishes = true;
            if (scenario == "move_rejected") simulated::moveRejected = true;
            if (scenario == "disturbance") simulated::disturbanceDegrees = 0.8;
            if (scenario == "noisy_motion") simulated::noiseAmplitude = 0.8;
            if (scenario == "runaway") simulated::yawSign *= -1;
        }
        loop();
        // Every actual command must obey the restricted trial envelope until
        // measured direction has been confirmed on that leg.
        while (checkedCommands < simulated::commands.size()) {
            const auto &command = simulated::commands[checkedCommands++];
            if (!legs[legIndex].directionConfirmed) {
                CHECK(abs(command.steps) <= TRIAL_BURST_STEPS);
                CHECK(command.speed == MIN_SPEED_HZ);
            }
        }
    }
    CHECK(finalPrinted);
    CHECK(occurrences("FINAL RESULT: ") == 1);
    CHECK(simulated::connectedPins.size() == 1 && simulated::connectedPins[0] == 33);
    CHECK(simulated::highPins.empty());
    CHECK(Wire.frequency == 0); // Bus A and its uninstalled encoder are unused.
    for (uint8_t address : simulated::addressedSensors) CHECK(address == 0x4A);
    int32_t netCommanded = 0;
    bool sawReturnCommands = false;
    for (const auto &command : simulated::commands) {
        CHECK(command.stepPin == 33);
        CHECK(abs(command.steps) >= 1 && abs(command.steps) <= MAX_BURST_STEPS);
        CHECK(command.speed >= MIN_SPEED_HZ && command.speed <= MAX_SPEED_HZ);
        CHECK(command.acceleration == ACCELERATION);
        CHECK(command.at >= 6000);
        netCommanded += command.steps;
        if (command.steps * TRIAL_POSITIVE_STEP_YAW_SIGN < 0) sawReturnCommands = true;
    }
    if (successful) {
        CHECK(phase == Phase::COMPLETE && printed("FINAL RESULT: PASS"));
        CHECK(occurrences("DIRECTION CONFIRMED:") == 2);
        CHECK(sawReturn && sawReturnCommands);
        for (unsigned i = 0; i < 2; ++i) {
            CHECK(legs[i].settled && legs[i].directionConfirmed && legs[i].bnoMotionSamples);
            CHECK(fabs(legs[i].error) <= TOLERANCE_DEG);
        }
        CHECK(fabs(shortestDifference(outboundActual, referenceHeading) - 3.0) <= TOLERANCE_DEG + 0.1);
        CHECK(fabs(shortestDifference(simulated::actualYaw(), referenceHeading)) <= TOLERANCE_DEG + 0.1);
        CHECK(fabs(shortestDifference(referenceHeading, simulated::baselineYaw)) < 0.15);
        CHECK(simulated::maximumYaw - simulated::baselineYaw < 4.0);
        CHECK(simulated::minimumYaw - simulated::baselineYaw > -0.5);
        CHECK(Wire1.sda == 4 && Wire1.scl == 5 && Wire1.frequency == 100000 && Wire1.timeout == 50);
        CHECK(bno.address == 0x4A && bno.wire == &Wire1 && yaw->directionPin == 32);
        if (scenario == "disturbance") CHECK(abs(netCommanded) > 20);
        if (scenario == "startup_reset_retry") CHECK(reportFailures == 1 && DiagnosticBno085::writeFailures == 1);
    } else {
        CHECK(phase == Phase::ABORTED && printed("FINAL RESULT: FAIL"));
        CHECK(!yaw || !yaw->isRunning());
        if (startupFailure) CHECK(simulated::commands.empty());
        if (oneOf(scenario, {"bno_outage", "blocked_bno", "reset", "reset_in_poll", "invalid_quaternion",
            "wrong_report", "guard_positive", "guard_negative", "guard_wrap", "heading_jump", "tilt_motion", "abort"})) {
            CHECK(injected && simulated::commands.size() == commandsAtInjection);
            CHECK(millis() - injectedAt < 500);
        }
        if (scenario == "wrong_way" || scenario == "wrong_way_noise") {
            CHECK(printed("DIRECTION MISMATCH"));
            CHECK(!legs[0].directionConfirmed && !sawReturn);
            CHECK(simulated::minimumYaw > simulated::baselineYaw - 0.6);
        }
        if (scenario == "return_wrong_way") {
            CHECK(legs[0].settled && !legs[1].directionConfirmed && printed("DIRECTION MISMATCH"));
        }
        if (scenario == "runaway") CHECK(printed("DIRECTION MISMATCH / RUNAWAY"));
        if (scenario == "frozen_feedback" || scenario == "frozen_motor") CHECK(millis() < 30000);
        if (scenario == "motion_timeout") CHECK(millis() - injectedAt < 4000);
        if (scenario == "noisy_motion") CHECK(printed("Leg timeout") && simulated::commands.size() == commandsAtInjection);
        if (scenario == "report_init") CHECK(DiagnosticBno085::writeFailures == 1);
    }
    const std::string saved = Serial.output;
    const size_t commandCount = simulated::commands.size();
    const unsigned samples = simulated::bnoReads;
    for (unsigned i = 0; i < 500; ++i) loop();
    CHECK(!yaw || !yaw->isRunning());
    CHECK(Serial.output == saved && simulated::commands.size() == commandCount && simulated::bnoReads == samples);
    std::printf("PASS: %s (trial sign %+d; %u checks; %zu bursts)\n",
                scenario.c_str(), TRIAL_POSITIVE_STEP_YAW_SIGN, checks, commandCount);
}
