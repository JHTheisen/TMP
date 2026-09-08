// Runs the actual M05 setup()/loop() against deterministic sensor/actuator fixtures.
// Fixture parameters are synthetic test inputs, not measured gimbal properties.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
unsigned checks = 0;
void require(bool ok, const char *expression, int line)
{
    ++checks;
    if (!ok) {
        std::fprintf(stderr, "FAIL line %d: %s\n%s", line, expression, Serial.output.c_str());
        std::exit(EXIT_FAILURE);
    }
}
#define CHECK(condition) require((condition), #condition, __LINE__)
bool printed(const char *value) { return Serial.output.find(value) != std::string::npos; }
size_t occurrences(const std::string &value)
{
    size_t count = 0, at = 0;
    while ((at = Serial.output.find(value, at)) != std::string::npos) { ++count; at += value.size(); }
    return count;
}
bool oneOf(const std::string &value, std::initializer_list<const char *> choices)
{
    for (const char *choice : choices) if (value == choice) return true;
    return false;
}
}

int main(int argc, char **argv)
{
    if (argc != 2) return EXIT_FAILURE;
    const std::string scenario = argv[1];
    const bool successful = oneOf(scenario, {"normal", "encoder_opposite", "ratio7", "ratio10", "ratio30",
        "pulses3200", "noise", "allowed_noise", "slip_backlash", "disturbance", "drift", "transient", "magnet", "startup_reset"});
    const bool startupFailure = oneOf(scenario, {"axis_init", "bus_init", "encoder_init", "bno_init",
        "report_init", "unstable_baseline", "startup_tilt", "startup_abort"});
    if (scenario == "encoder_opposite") simulated::encoderSign = -1;
    if (scenario == "ratio7") simulated::fixtureRatio = 7.0;
    if (scenario == "ratio10") simulated::fixtureRatio = 10.0;
    if (scenario == "ratio30") simulated::fixtureRatio = 30.0;
    if (scenario == "pulses3200") { simulated::pulsesPerRev = 3200.0; simulated::fixtureRatio = 10.0; }
    if (scenario == "noise") simulated::noiseAmplitude = 0.08;
    if (scenario == "allowed_noise") simulated::noiseAmplitude = 0.20;
    if (scenario == "slip_backlash") {
        simulated::pulseEfficiency = 0.65;
        simulated::backlashSteps = 12.0;
        simulated::noiseAmplitude = 0.03;
    }
    if (scenario == "drift") simulated::driftDegreesPerSecond = 0.008;
    if (scenario == "magnet") simulated::missingPitchMagnet = true;
    if (scenario == "wrong_way") simulated::pitchSign = -1;
    if (scenario == "frozen_cradle") simulated::frozenCradle = true;
    if (scenario == "frozen_motor") simulated::frozenMotor = true;
    if (scenario == "frozen_encoder") simulated::frozenEncoder = true;
    if (scenario == "axis_init") simulated::axisInitFails = true;
    if (scenario == "bus_init") simulated::busBInitFails = true;
    if (scenario == "encoder_init") simulated::encoderInitFails = true;
    if (scenario == "bno_init") simulated::bnoInitFails = true;
    if (scenario == "report_init") simulated::reportInitFails = true;
    if (scenario == "unstable_baseline") simulated::noiseAmplitude = 1.0;
    if (scenario == "startup_tilt") simulated::baselinePitch = 80.0;
    if (scenario == "startup_reset") simulated::bnoResetAt = 1000;
    setup();
    if (scenario == "startup_abort") simulated::pendingSerial = 'x';

    bool injected = false;
    size_t commandsAtInjection = 0;
    uint32_t injectedAt = 0;
    double outboundActual = 0;
    bool sawReturn = false;
    while (!finalPrinted && millis() < 250000) {
        if (!sawReturn && legIndex == 1) {
            sawReturn = true;
            outboundActual = simulated::actualPitch();
        }
        if (!injected && phase == Phase::MOVING && simulated::commands.size() >= 3) {
            injected = true;
            injectedAt = millis();
            commandsAtInjection = simulated::commands.size();
            if (scenario == "transient") {
                simulated::bnoPauseStart = millis();
                simulated::bnoPauseEnd = millis() + 60;
                simulated::encoderPauseStart = millis();
                simulated::encoderPauseEnd = millis() + 40;
            }
            if (scenario == "bno_outage") { simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 10000; }
            if (scenario == "encoder_outage") simulated::encoderOutage = true;
            if (scenario == "encoder_error") simulated::encoderReadError = true;
            if (scenario == "encoder_ambiguous") simulated::encoderRawOverride = (previousRaw + 2048) % 4096;
            if (scenario == "blocked_encoder") simulated::blockEncoderMs = 110;
            if (scenario == "blocked_bno") simulated::blockBnoMs = 160;
            if (scenario == "reset") simulated::bnoResetAt = millis();
            if (scenario == "invalid_quaternion") simulated::invalidQuaternion = true;
            if (scenario == "wrong_report") simulated::wrongReportType = true;
            if (scenario == "relative_guard") simulated::disturbanceDegrees = 6.5;
            if (scenario == "absolute_guard") simulated::disturbanceDegrees = 75.0;
            if (scenario == "abort") simulated::pendingSerial = 'X';
            if (scenario == "motion_timeout") simulated::moveNeverFinishes = true;
            if (scenario == "move_rejected") simulated::moveRejected = true;
            if (scenario == "disturbance") simulated::disturbanceDegrees = 0.8;
            if (scenario == "noisy_motion") simulated::noiseAmplitude = 0.8;
        }
        loop();
    }
    CHECK(finalPrinted);
    CHECK(occurrences("FINAL RESULT: ") == 1);
    CHECK(simulated::connectedPins.size() == 1 && simulated::connectedPins[0] == 12);
    CHECK(simulated::highPins.empty());
    int32_t netCommanded = 0;
    unsigned negativeCommands = 0;
    for (const simulated::MoveCommand &command : simulated::commands) {
        CHECK(command.stepPin == 12);
        CHECK(abs(command.steps) >= 1 && abs(command.steps) <= milestone5::MAX_BURST_STEPS);
        CHECK(command.speed >= milestone5::MIN_SPEED_HZ && command.speed <= milestone5::MAX_SPEED_HZ);
        CHECK(command.acceleration == milestone5::ACCELERATION);
        CHECK(command.at >= 6000);
        netCommanded += command.steps;
        if (command.steps < 0) ++negativeCommands;
    }
    if (successful) {
        CHECK(phase == Phase::COMPLETE && printed("FINAL RESULT: PASS"));
        CHECK(sawReturn && negativeCommands > 0);
        CHECK(legs[0].settled && legs[1].settled);
        CHECK(fabs(legs[0].error) <= milestone5::TOLERANCE_DEG);
        CHECK(fabs(legs[1].error) <= milestone5::TOLERANCE_DEG);
        CHECK(fabs(outboundActual - referencePitch - 3.0) <= milestone5::TOLERANCE_DEG + 0.1);
        CHECK(fabs(simulated::actualPitch() - referencePitch) <= milestone5::TOLERANCE_DEG + 0.1);
        CHECK(simulated::maximumPitch - referencePitch < 4.0);
        CHECK(simulated::minimumPitch - referencePitch > -0.5);
        CHECK(simulated::bnoReads > 100 && simulated::encoderReads > 100);
        CHECK(Wire1.sda == 4 && Wire1.scl == 5 && Wire1.frequency == 100000 && Wire1.timeout == 50);
        CHECK(bno.address == 0x4A && bno.wire == &Wire1);
        CHECK(pitch->directionPin == 26);
        CHECK(fabs(shaftDegrees - simulated::encoderSign * simulated::shaftSteps *
                   360.0 / simulated::pulsesPerRev) < 0.2);
        // External motion changes the software count required to reach sensor zero.
        if (scenario == "disturbance") CHECK(abs(netCommanded) > 5);
    } else {
        CHECK(phase == Phase::ABORTED && printed("FINAL RESULT: FAIL"));
        CHECK(!pitch || !pitch->isRunning());
        if (startupFailure) CHECK(simulated::commands.empty());
        if (oneOf(scenario, {"bno_outage", "encoder_outage", "encoder_error", "encoder_ambiguous", "blocked_encoder", "blocked_bno",
                            "reset", "invalid_quaternion", "wrong_report", "relative_guard", "absolute_guard", "abort"})) {
            CHECK(injected);
            CHECK(simulated::commands.size() == commandsAtInjection);
            CHECK(millis() - injectedAt < 500);
        }
        if (scenario == "wrong_way") CHECK(simulated::minimumPitch > simulated::baselinePitch - 1.0);
        if (scenario == "frozen_cradle" || scenario == "frozen_motor") CHECK(millis() < 30000);
        if (scenario == "motion_timeout") CHECK(millis() - injectedAt < 4000);
        if (scenario == "noisy_motion") {
            CHECK(simulated::commands.size() == commandsAtInjection);
            CHECK(printed("Leg timeout"));
        }
        if (scenario == "report_init") CHECK(milestone4::DiagnosticBno085::writeFailures == 1);
    }
    CHECK(!pitch || !pitch->isRunning());
    const std::string saved = Serial.output;
    const size_t commandCount = simulated::commands.size();
    const unsigned bnoSamples = simulated::bnoReads;
    const unsigned encoderSamples = simulated::encoderReads;
    for (unsigned i = 0; i < 500; ++i) loop();
    CHECK(Serial.output == saved && simulated::commands.size() == commandCount);
    CHECK(simulated::bnoReads == bnoSamples && simulated::encoderReads == encoderSamples);
    std::printf("PASS: %s (%u firmware integration checks, %zu bursts, synthetic elapsed %.1f s)\n",
                scenario.c_str(), checks, commandCount, (millis() - 10000) / 1000.0);
}
