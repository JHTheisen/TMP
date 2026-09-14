// Identical synthetic accuracy timelines through the actual M08/M09 firmware.
// This validates deadline decisions; it does NOT measure BNO calibration speed.
#ifdef AUDIT_M08
#include "../../M08_go_to_pose/src/main.cpp"
#else
#include "../src/main.cpp"
#endif
#include <cstdio>
#include <cstdlib>

void auditTick(uint32_t now) {
    motionWatchdog.check(now);
#ifndef AUDIT_M08
    commandWatchdog.check(now);
#endif
}
int main(int argc, char **argv) {
    if (argc != 2) return 1;
    const uint32_t accuracyTwoAt = static_cast<uint32_t>(std::strtoul(argv[1], nullptr, 10));
    simulated::accuracy = 0;
    simulated::baselinePitch = 8;
    simulated::independentTick = auditTick;
    setup();
    uint32_t decisionAt = 0, nextPoll = 0;
    while (millis() - startedAt < 40000) {
        const uint32_t elapsed = millis() - startedAt;
        simulated::accuracy = elapsed < accuracyTwoAt / 2 ? 0 : (elapsed < accuracyTwoAt ? 1 : (elapsed < accuracyTwoAt + 1000 ? 2 : 3));
#ifndef AUDIT_M08
        if (elapsed >= nextPoll) { simulated::serialInput += "STATUS\n"; nextPoll = elapsed + 500; }
#else
        (void)nextPoll;
#endif
        loop();
        if (!decisionAt && (referenceSet || pitchReady)) decisionAt = millis() - startedAt;
    }
    std::printf("accuracy2_at_ms=%lu decision_at_ms=%lu north_reference=%d accuracy_at_40s=%u phase=%s pitch_ready=%d\n",
        static_cast<unsigned long>(accuracyTwoAt), static_cast<unsigned long>(decisionAt),
        referenceSet, bnoAccuracy, phaseText(), pitchReady);
    if (!decisionAt || bnoAccuracy != 3) return 1;
    if (accuracyTwoAt <= 18000 && !referenceSet) return 1;
    if (accuracyTwoAt >= 19500 && referenceSet) return 1;
}
