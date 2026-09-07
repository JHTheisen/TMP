#include "../src/diagnostic_health.h"

#include <cstdio>
#include <cstdlib>

namespace {

unsigned checks = 0;

void require(bool condition, const char *expression, int line)
{
    ++checks;
    if (!condition)
    {
        std::fprintf(stderr, "FAIL line %d: %s\n", line, expression);
        std::exit(EXIT_FAILURE);
    }
}

#define CHECK(condition) require((condition), #condition, __LINE__)

void staleThresholdAndEpisodeAccounting()
{
    milestone3::BnoHealth health;
    health.lastFreshOrStartMs = 100;

    health.check(600, true);  // Exactly 500 ms remains fresh.
    CHECK(!health.staleActive);
    CHECK(health.staleEvents == 0);
    CHECK(health.staleChecks == 0);
    CHECK(!health.persistentTestFailure);

    health.check(601, true);
    CHECK(health.staleActive);
    CHECK(health.staleEvents == 1);
    CHECK(health.staleChecks == 1);
    health.check(800, true);
    health.check(1160, true);
    CHECK(health.staleEvents == 1);  // Repeated checks are one outage.
    CHECK(health.staleChecks == 3);

    health.recordFresh(1160, true);  // Reproduce the reported 1060 ms gap.
    CHECK(!health.staleActive);
    CHECK(health.recoveredStaleEvents == 1);
    CHECK(health.freshSamples == 1);
    CHECK(health.maxGapMs == 1060);
    CHECK(health.maxTestGapMs == 1060);
    CHECK(!health.persistentTestFailure);
    health.check(1161, true);
    CHECK(health.staleChecks == 3);
    health.recordFresh(1170, true);
    CHECK(health.recoveredStaleEvents == 1);
    CHECK(health.freshSamples == 2);

    health.check(1771, true);
    CHECK(health.staleEvents == 2);
    CHECK(health.staleChecks == 4);
    health.recordFresh(1780, true);
    CHECK(health.recoveredStaleEvents == 2);
    CHECK(health.maxGapMs == 1060);
}

void freshReportAfterBlockingCallRetainsGap()
{
    milestone3::BnoHealth health;
    health.lastFreshOrStartMs = 9000;
    // There was no check while a foreground call occupied the whole gap.
    health.recordFresh(10060, true);
    CHECK(health.staleEvents == 1);
    CHECK(health.recoveredStaleEvents == 1);
    CHECK(health.staleChecks == 0);
    CHECK(!health.staleActive);
    CHECK(health.lastFreshOrStartMs == 10060);
    CHECK(health.maxGapMs == 1060);
    CHECK(health.maxTestGapMs == 1060);
    CHECK(!health.persistentTestFailure);
}

void persistentFailureBoundaryAndRecovery()
{
    milestone3::BnoHealth health;
    health.lastFreshOrStartMs = 20;
    health.check(2019, true);
    CHECK(!health.persistentTestFailure);
    health.check(2020, true);  // Exactly 2000 ms latches failure.
    CHECK(health.persistentTestFailure);
    CHECK(health.staleEvents == 1);
    health.recordFresh(2050, true);
    CHECK(health.persistentTestFailure);
    CHECK(!health.staleActive);
    CHECK(health.recoveredStaleEvents == 1);
    CHECK(health.maxTestGapMs == 2030);
    health.recordFresh(2060, true);
    CHECK(health.persistentTestFailure);

    milestone3::BnoHealth blocked;
    blocked.recordFresh(2000, true);  // First observation is recovery.
    CHECK(blocked.persistentTestFailure);
    CHECK(blocked.staleEvents == 1);
    CHECK(blocked.recoveredStaleEvents == 1);
    CHECK(blocked.staleChecks == 0);
}

void startupOutageIsSeparateFromActiveTest()
{
    milestone3::BnoHealth health;
    health.lastFreshOrStartMs = 1000;
    health.check(4000, false);
    CHECK(health.staleActive);
    CHECK(health.staleEvents == 1);
    CHECK(!health.persistentTestFailure);
    CHECK(health.maxGapMs == 3000);
    CHECK(health.maxTestGapMs == 0);
    health.recordFresh(4100, false);
    CHECK(health.recoveredStaleEvents == 1);
    CHECK(!health.persistentTestFailure);
    health.check(4110, true);
    CHECK(health.maxTestGapMs == 10);
    CHECK(health.maxGapMs == 3100);
    CHECK(!health.persistentTestFailure);
    health.check(6100, true);
    CHECK(health.persistentTestFailure);
    CHECK(health.staleEvents == 2);

    milestone3::BnoHealth ongoing;
    ongoing.check(3000, false);
    CHECK(!ongoing.persistentTestFailure);
    ongoing.check(3001, true);
    CHECK(ongoing.persistentTestFailure);
    CHECK(ongoing.staleEvents == 1);
}

void millisWrapPreservesElapsedTime()
{
    milestone3::BnoHealth health;
    health.lastFreshOrStartMs = UINT32_MAX - 255U;
    health.check(244, true);  // 500 ms across unsigned wrap.
    CHECK(!health.staleActive);
    CHECK(health.maxGapMs == 500);
    health.check(245, true);
    CHECK(health.staleActive);
    CHECK(health.staleEvents == 1);
    health.recordFresh(804, true);  // 1060 ms across wrap.
    CHECK(health.maxGapMs == 1060);
    CHECK(health.recoveredStaleEvents == 1);
    CHECK(!health.persistentTestFailure);
    health.check(814, true);
    CHECK(!health.staleActive);

    milestone3::BnoHealth persistent;
    persistent.lastFreshOrStartMs = UINT32_MAX - 255U;
    persistent.recordFresh(1744, true);  // 2000 ms across wrap.
    CHECK(persistent.persistentTestFailure);
    CHECK(persistent.maxTestGapMs == 2000);
    CHECK(persistent.recoveredStaleEvents == 1);
}

milestone3::MoveEvidence completedMove(int32_t target)
{
    milestone3::MoveEvidence move;
    move.began = true;
    move.completed = true;
    move.target = target;
    move.end = target;
    return move;
}

void motionRequiresCompleteEvidence()
{
    milestone3::MoveEvidence move;
    CHECK(!move.motionPassed());  // Default equal zero counts cannot pass.
    move = completedMove(500);
    CHECK(move.motionPassed());
    move.began = false;
    CHECK(!move.motionPassed());
    move.began = true;
    move.completed = false;
    CHECK(!move.motionPassed());
    move.completed = true;
    move.end = 499;
    CHECK(!move.motionPassed());

    milestone3::AxisEvidence axis;
    CHECK(!axis.motionPassed());
    axis.start = -17;
    axis.end = -17;
    axis.forward = completedMove(483);
    axis.reverse = completedMove(-17);
    CHECK(!axis.motionPassed());
    axis.returned = true;
    CHECK(axis.motionPassed());
    axis.end = -16;
    CHECK(!axis.motionPassed());
    axis.end = -17;
    axis.forward.completed = false;
    CHECK(!axis.motionPassed());
    axis.forward.completed = true;
    axis.reverse.began = false;
    CHECK(!axis.motionPassed());
    axis.reverse.began = true;
    axis.reverse.end = -18;
    CHECK(!axis.motionPassed());
}

void continuityNeedsBothEncodersNewBnoAndDisplay()
{
    milestone3::MoveEvidence move = completedMove(500);
    CHECK(!move.sensorsPassed());
    move.encoderReads[0] = 20;
    move.encoderReads[1] = 20;
    move.bnoFreshSamples = 120;
    move.telemetrySnapshots = 2;
    CHECK(move.sensorsPassed());
    move.encoderReads[0] = 0;
    CHECK(!move.sensorsPassed());
    move.encoderReads[0] = 20;
    move.encoderReads[1] = 0;
    CHECK(!move.sensorsPassed());
    move.encoderReads[1] = 20;
    move.bnoFreshSamples = 0;  // An old cached orientation is insufficient.
    CHECK(!move.sensorsPassed());
    move.bnoFreshSamples = 120;
    move.telemetrySnapshots = 0;
    CHECK(!move.sensorsPassed());
    move.telemetrySnapshots = 1;
    CHECK(move.sensorsPassed());
}

}  // namespace

int main()
{
    staleThresholdAndEpisodeAccounting();
    freshReportAfterBlockingCallRetainsGap();
    persistentFailureBoundaryAndRecovery();
    startupOutageIsSeparateFromActiveTest();
    millisWrapPreservesElapsedTime();
    motionRequiresCompleteEvidence();
    continuityNeedsBothEncodersNewBnoAndDisplay();
    std::printf("PASS: %u diagnostic health checks (7 scenarios)\n", checks);
    return EXIT_SUCCESS;
}
