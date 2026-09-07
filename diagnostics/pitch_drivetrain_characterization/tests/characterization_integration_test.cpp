// Deterministic fixtures exercise the REAL firmware setup/loop. These values
// are synthetic arithmetic inputs, never measurements of the physical gimbal.
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
}

int main(int argc, char **argv)
{
    if (argc != 2) return EXIT_FAILURE;
    const std::string scenario = argv[1];
    if (scenario == "opposite") simulated::encoderSign = -1;
    if (scenario == "ratio10") simulated::fixtureRatio = 10.0;
    if (scenario == "magnet") simulated::missingPitchMagnet = true;
    if (scenario == "noisy") simulated::noisyBno = true;
    if (scenario == "init") simulated::axisInitFails = true;
    setup();
    bool injected = false;
    while (!finalPrinted && millis() < 300000)
    {
        if (!injected && phase == Phase::MOVING && currentSegment == 1)
        {
            injected = true;
            if (scenario == "persistent" || scenario == "transient") {
                simulated::bnoPauseStart = millis() + 100;
                simulated::bnoPauseEnd = simulated::bnoPauseStart +
                    (scenario == "persistent" ? 2100 : 1060);
            }
            if (scenario == "encoder") simulated::encoderOutage = true;
            if (scenario == "blocked_encoder") simulated::now += 600;
            if (scenario == "reset") {
                simulated::bnoResetAt = millis();
                simulated::reenableFailuresRemaining = 1;
            }
            if (scenario == "abort") simulated::pendingSerial = 'X';
        }
        loop();
    }
    CHECK(finalPrinted);
    CHECK(occurrences("=== MILESTONE 4 PITCH CHARACTERIZATION SUMMARY ===") == 1);
    CHECK(occurrences("FINAL RESULT: ") == 1);
    CHECK(simulated::connectedPins.size() == 1 && simulated::connectedPins[0] == 12);
    CHECK(simulated::highPins.empty());
    int32_t position = 0;
    for (const simulated::MoveCommand &command : simulated::commands) {
        CHECK(command.stepPin == 12);
        CHECK(abs(command.steps) <= 250);
        position += command.steps;
        CHECK(position >= -600 && position <= 600);
    }
    const bool successful = scenario == "normal" || scenario == "opposite" || scenario == "ratio10" ||
                            scenario == "transient" || scenario == "reset" || scenario == "magnet";
    if (successful)
    {
        CHECK(phase == Phase::COMPLETE);
        CHECK(printed("FINAL RESULT: PASS"));
        CHECK(segmentCount == 36 && simulated::commands.size() == 36);
        CHECK(position == 0 && pitch->getCurrentPosition() == startingSteps);
        CHECK(pitch->speed == 1000 && pitch->acceleration == 1000 && pitch->directionPin == 26);
        CHECK(learnedEncoderSign == simulated::encoderSign && learnedPitchSign == simulated::pitchSign);
        CHECK(directionConsistent);
        unsigned ratioCount = 0;
        for (size_t i = 0; i < segmentCount; ++i) {
            CHECK(records[i].began && records[i].finished && records[i].settled);
            CHECK(records[i].encoderSamples && records[i].bnoSamples);
            if (segments[i].kind == Kind::MEASURE && records[i].ratio.valid) {
                ++ratioCount;
                CHECK(fabs(records[i].ratio.magnitude - simulated::fixtureRatio) < 0.05);
            }
        }
        CHECK(ratioCount >= 15);
        CHECK(encoderReadFailures == 0);
        CHECK(printed("Repeatability matched-leg pairs: 6"));
        if (scenario == "magnet") {
            CHECK(encoderMagnetFailures > 0);
            CHECK(printed("magnet_status=NOT-DETECTED"));
            CHECK(printed("non-ideal AS5600 magnet status is advisory"));
            CHECK(!printed("unreadable/bad magnet"));
        }
        if (scenario == "transient") {
            CHECK(bnoHealth.staleEvents == 1 && bnoHealth.recoveredStaleEvents == 1);
            CHECK(bnoHealth.maxGapMs >= 1060 && !bnoHealth.persistentTestFailure);
        }
        if (scenario == "reset") {
            CHECK(bnoTestResets == 1 && reportFailures == 1 && reportSuccesses == 1);
            CHECK(resetFreshRecoveries == 1 && !resetAwaitingFresh);
            CHECK(!records[1].ratio.valid);
        }
    }
    else
    {
        CHECK(phase == Phase::ABORTED && printed("FINAL RESULT: FAIL"));
        CHECK(!pitch || !pitch->isRunning());
        if (scenario == "persistent") CHECK(bnoHealth.persistentTestFailure);
        if (scenario == "encoder") CHECK(encoderReadFailures > 0);
        if (scenario == "blocked_encoder") CHECK(printed("acquisition gap reached 500 ms before recovery"));
        if (scenario == "magnet" || scenario == "noisy" || scenario == "init") CHECK(simulated::commands.empty());
        if (scenario == "abort") CHECK(printed("Operator X abort"));
    }
    const std::string saved = Serial.output;
    const size_t commandCount = simulated::commands.size();
    const uint32_t sampleCount = bnoHealth.freshSamples;
    for (unsigned i = 0; i < 5000; ++i) loop();
    CHECK(Serial.output == saved && simulated::commands.size() == commandCount);
    CHECK(bnoHealth.freshSamples == sampleCount);
    std::printf("PASS: %s (%u firmware integration checks)\n", scenario.c_str(), checks);
}
