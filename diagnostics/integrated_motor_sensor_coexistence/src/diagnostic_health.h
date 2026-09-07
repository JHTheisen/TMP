#pragma once

#include <stdint.h>

namespace milestone3 {

constexpr uint32_t BNO_REPORT_INTERVAL_US = 10000;
constexpr uint32_t BNO_REPORT_INTERVAL_MS = BNO_REPORT_INTERVAL_US / 1000;
// Allow shared-bus/foreground scheduling jitter, but retain the observed 1060 ms
// interruption as a warning. A 200-report outage is a persistent test failure.
constexpr uint32_t BNO_STALE_AFTER_MS = 50 * BNO_REPORT_INTERVAL_MS;
constexpr uint32_t BNO_PERSISTENT_AFTER_MS = 200 * BNO_REPORT_INTERVAL_MS;

struct MoveEvidence {
    bool began = false;
    bool completed = false;
    int32_t target = 0;
    int32_t end = 0;
    uint32_t encoderReads[2] = {};
    uint32_t bnoFreshSamples = 0;
    uint32_t telemetrySnapshots = 0;

    bool motionPassed() const { return began && completed && end == target; }
    bool sensorsPassed() const
    {
        return encoderReads[0] > 0 && encoderReads[1] > 0 &&
               bnoFreshSamples > 0 && telemetrySnapshots > 0;
    }
};

struct AxisEvidence {
    int32_t start = 0;
    int32_t end = 0;
    bool returned = false;
    MoveEvidence forward;
    MoveEvidence reverse;

    bool motionPassed() const
    {
        return forward.motionPassed() && reverse.motionPassed() && returned && end == start;
    }
};

struct BnoHealth {
    uint32_t lastFreshOrStartMs = 0;
    uint32_t freshSamples = 0;
    uint32_t staleChecks = 0;
    uint32_t staleEvents = 0;
    uint32_t recoveredStaleEvents = 0;
    uint32_t maxGapMs = 0;
    uint32_t maxTestGapMs = 0;
    bool staleActive = false;
    bool persistentTestFailure = false;

    void observeTime(uint32_t now, bool duringTest)
    {
        const uint32_t gap = now - lastFreshOrStartMs;
        if (gap > maxGapMs) maxGapMs = gap;
        if (duringTest && gap > maxTestGapMs) maxTestGapMs = gap;
        if (gap > BNO_STALE_AFTER_MS && !staleActive)
        {
            staleActive = true;
            ++staleEvents;
        }
        if (duringTest && gap >= BNO_PERSISTENT_AFTER_MS)
        {
            persistentTestFailure = true;
        }
    }

    void recordFresh(uint32_t now, bool duringTest)
    {
        // Inspect the old timestamp first: recovery must not erase an outage
        // that happened while a foreground library call was blocking.
        observeTime(now, duringTest);
        if (staleActive) ++recoveredStaleEvents;
        staleActive = false;
        lastFreshOrStartMs = now;
        ++freshSamples;
    }

    void check(uint32_t now, bool duringTest)
    {
        observeTime(now, duringTest);
        if (staleActive) ++staleChecks;
    }
};

}  // namespace milestone3
