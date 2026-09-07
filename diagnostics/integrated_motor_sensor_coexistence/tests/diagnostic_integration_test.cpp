// Each scenario runs in a fresh process. Include the real application so these
// checks exercise setup/loop, sensor counters, motion transitions and Serial output.
#include "../src/main.cpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
unsigned checks = 0;
void require(bool condition, const char *expression, int line)
{
    ++checks;
    if (!condition)
    {
        std::fprintf(stderr, "FAIL line %d: %s\n%s", line, expression, Serial.output.c_str());
        std::exit(EXIT_FAILURE);
    }
}
#define CHECK(condition) require((condition), #condition, __LINE__)

size_t occurrences(const std::string &needle)
{
    size_t count = 0;
    for (size_t at = 0; (at = Serial.output.find(needle, at)) != std::string::npos; at += needle.size())
        ++count;
    return count;
}
bool printed(const char *text) { return Serial.output.find(text) != std::string::npos; }

void verifyCompletedSequence()
{
    CHECK(motionPhase == MotionPhase::COMPLETE);
    CHECK(simulated::commands.size() == 6);
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis)
    {
        CHECK(axisEvidence[axis].motionPassed());
        CHECK(axisEvidence[axis].forward.sensorsPassed());
        CHECK(axisEvidence[axis].reverse.sensorsPassed());
        CHECK(axes[axis]->stepper->getCurrentPosition() == axisEvidence[axis].start);
        CHECK(axes[axis]->stepper->speed == 1000);
        CHECK(axes[axis]->stepper->acceleration == 1000);
        CHECK(axes[axis]->stepper->directionPin == axes[axis]->dirPin);
        CHECK(simulated::commands[2 * axis].stepPin == axes[axis]->stepPin);
        CHECK(simulated::commands[2 * axis].steps == 500);
        CHECK(simulated::commands[2 * axis + 1].stepPin == axes[axis]->stepPin);
        CHECK(simulated::commands[2 * axis + 1].steps == -500);
    }
    CHECK(occurrences("MOVE START axis=") == 6);
    CHECK(occurrences("MOVE END axis=") == 6);
    CHECK(printed("YAW motion: PASS"));
    CHECK(printed("PITCH motion: PASS"));
    CHECK(printed("CARRIAGE motion: PASS"));
    CHECK(printed("Sensor telemetry during motion: PASS"));
    CHECK(betweenMoveSensorChecks > 0);
    CHECK(bnoNoEventPolls > bnoHealth.freshSamples);
    CHECK(bnoInvalidSamples == 0);
    CHECK(bnoProbeFailures == 0);
    CHECK(printed("BNO085 observable read failures: 0"));
}

void verifySummaryRemainsVisible()
{
    CHECK(occurrences("=== MILESTONE 3 FINAL SUMMARY ===") == 1);
    CHECK(occurrences("FINAL RESULT: ") == 1);
    const std::string savedOutput = Serial.output;
    const size_t savedCommands = simulated::commands.size();
    const uint32_t savedFresh = bnoHealth.freshSamples;
    const uint32_t savedReads = encoderA.successfulReads;
    for (unsigned i = 0; i < 5000; ++i) loop(); // 100 seconds after final result.
    CHECK(Serial.output == savedOutput);
    CHECK(simulated::commands.size() == savedCommands);
    CHECK(bnoHealth.freshSamples == savedFresh);
    CHECK(encoderA.successfulReads == savedReads);
}
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::fprintf(stderr, "Usage: diagnostic_integration_test normal|transient|persistent|encoder_failure|reset_retry|init_abort\n");
        return EXIT_FAILURE;
    }
    const std::string scenario = argv[1];
    if (scenario == "transient" || scenario == "persistent")
    {
        simulated::bnoPauseStart = 6000;
        simulated::bnoPauseEnd = scenario == "transient" ? 7050 : 8000;
    }
    else if (scenario == "encoder_failure") simulated::encoderFailureAt = 6000;
    else if (scenario == "reset_retry")
    {
        simulated::bnoResetAt = 6000;
        simulated::reenableFailuresRemaining = 1;
    }
    else if (scenario == "init_abort") simulated::axisInitFails = true;
    else CHECK(scenario == "normal");

    setup();
    while (!finalSummaryPrinted && millis() < 60000) loop();
    CHECK(finalSummaryPrinted);

    if (scenario == "init_abort")
    {
        CHECK(motionPhase == MotionPhase::ABORTED);
        CHECK(simulated::commands.empty());
        CHECK(printed("Initialization requirements not met; no motion commanded"));
        CHECK(printed("YAW motion: FAIL"));
        CHECK(printed("FINAL RESULT: FAIL"));
    }
    else
    {
        verifyCompletedSequence();
        CHECK(printed("magnet=TOO-WEAK (informational only)"));
        CHECK(printed("magnet=NOT-DETECTED (informational only)"));
        CHECK(printed("AS5600 Bus B communication: PASS"));
        if (scenario == "encoder_failure")
        {
            CHECK(encoderA.testReadFailures == 1);
            CHECK(encoderA.readingValid); // Recovery cannot erase the failure.
            CHECK(printed("AS5600 Bus A communication: FAIL"));
            CHECK(printed("FINAL RESULT: FAIL"));
        }
        else
        {
            CHECK(printed("AS5600 Bus A communication: PASS"));
            CHECK(encoderA.testReadFailures == 0);
            CHECK(encoderB.testReadFailures == 0);
        }
        if (scenario == "transient" || scenario == "persistent")
        {
            CHECK(bnoHealth.staleEvents == 1);
            CHECK(bnoHealth.recoveredStaleEvents == 1);
            CHECK(bnoHealth.staleChecks > 0);
            CHECK(!bnoHealth.staleActive);
            CHECK(printed("Recorded sensor warnings: YES"));
            CHECK(bnoIsFresh(millis()));
            if (scenario == "transient")
            {
                CHECK(bnoHealth.maxTestGapMs == 1060);
                CHECK(!bnoHealth.persistentTestFailure);
                CHECK(printed("BNO085 communication: PASS"));
                CHECK(printed("FINAL RESULT: PASS"));
            }
            else
            {
                CHECK(bnoHealth.maxTestGapMs == 2010);
                CHECK(bnoHealth.persistentTestFailure);
                CHECK(printed("BNO085 communication: FAIL"));
                CHECK(printed("FINAL RESULT: FAIL"));
            }
        }
        else if (scenario == "reset_retry")
        {
            CHECK(bnoResetEvents == 1);
            CHECK(bnoTestResetEvents == 1);
            CHECK(bnoReenableAttempts == 2);
            CHECK(bnoReenableFailures == 1);
            CHECK(bnoReenableSuccesses == 1);
            CHECK(bnoResetFreshRecoveries == 1);
            CHECK(!bnoResetAwaitingFresh);
            CHECK(!bnoHealth.persistentTestFailure);
            CHECK(printed("Recorded sensor warnings: YES"));
            CHECK(printed("BNO085 communication: PASS"));
            CHECK(printed("FINAL RESULT: PASS"));
        }
        else if (scenario == "normal")
        {
            CHECK(bnoHealth.staleEvents == 0);
            CHECK(bnoResetEvents == 0);
            CHECK(printed("Recorded sensor warnings: NONE"));
            CHECK(printed("BNO085 communication: PASS"));
            CHECK(printed("FINAL RESULT: PASS"));
        }
    }
    verifySummaryRemainsVisible();
    std::printf("PASS: %s integration scenario (%u checks)\n", scenario.c_str(), checks);
    return EXIT_SUCCESS;
}
