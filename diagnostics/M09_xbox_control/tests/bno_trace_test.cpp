// Exercise production diagnostic callbacks and scheduling without hardware.
#include "../src/main.cpp"
#include <cassert>
#include <cstdio>
#include <string>

void pump() { publishSensorContext(); sensorWorker.testDispatch(); consumeSensorStatus(); }
int main() {
    setup();
    while (millis() < 50) loop();
    bnoTrace = BnoLifecycleDiagnostics{};
    const auto commands = simulated::commands.size();
    // Preserve two callbacks even if the library's Boolean latch coalesces them.
    m09_bno_reset(1234); delay(1); m09_bno_reset(2345);
    manualActive = true; pump();
    assert(bnoTrace.resetEvents == 2 && bnoTrace.size >= 2);
    assert(bnoTrace.events[0].a == 1234 && bnoTrace.events[1].a == 2345);
    assert(bnoTrace.events[1].atUs - bnoTrace.events[0].atUs == 1000);
    // Queries cannot run while armed, in POSE, or while braking/moving.
    manualActive = true; pump(); assert(simulated::productQueries == 0);
    manualActive = false; poseActive = true; pump(); assert(simulated::productQueries == 0);
    poseActive = false;
    simulated::motors[0].drive = simulated::Drive::BRAKING;
    pump(); assert(simulated::productQueries == 0);
    simulated::motors[0].drive = simulated::Drive::IDLE;
    const uint32_t before = millis(); pump();
    assert(millis() == before && simulated::productQueries == 1 && bnoTrace.queryPending);
    m09_bno_product(3456, 3, 1000, 2000); pump();
    assert(bnoTrace.productGeneration == 2 && bnoTrace.queryResponses == 1);
    assert(std::string(resetCauseText(3)) == "WATCHDOG");
    assert(std::string(resetCauseText(1)) == "POWER_ON");
    assert(std::string(resetCauseText(2)) == "INTERNAL");
    assert(std::string(resetCauseText(4)) == "EXTERNAL");
    assert(std::string(resetCauseText(99)) == "UNKNOWN");
    delay(1000); pump(); assert(!bnoTrace.queryPending && simulated::productQueries == 1);
    m09_bno_reset(4567); pump(); assert(simulated::productQueries == 2);
    delay(1000); pump();
    bool unavailable=false;
    for (unsigned n=0; n<bnoTrace.size; ++n) { const auto &e=bnoTrace.events[(bnoTrace.head+n)%32];
        if (std::string(e.kind)=="PRODUCT_QUERY_WINDOW_END" && e.result==-6) unavailable=true; }
    assert(unavailable);
    m09_bno_io(0xfffffff0U, 1100000, 1, 4, 0); pump();
    assert(bnoTrace.ioFailures == 1 && bnoTrace.ioSlow == 1 && bnoTrace.ioMaxUs == 1100000);
    m09_bno_io(12, 400, 2, 20, 1); pump();
    assert(bnoTrace.ioCalls == 2 && bnoTrace.ioFailures == 1);
    m09_bno_init_response(5678); pump();
    assert(bnoTrace.resetEvents == 3); // Unsolicited INIT does not redefine resets.
    // Explicit quality label does not change existing quaternion acceptance.
    sh2_SensorValue_t rv = {}; rv.sensorId = SH2_ROTATION_VECTOR; rv.status = 3;
    rv.un.rotationVector = {-0.0000610351562f, -0.0133666992f, -0.0000610351562f, -0.0000610351562f, -0.000244140625f};
    bnoDiagnostics.observe(rv, millis());
    assert(!bnoDiagnostics.plausible && bnoDiagnostics.malformed == 1);
    EulerAngles e; assert(quaternionToEuler(rv.un.rotationVector, e));
    rv.un.rotationVector = {1, 0, 0, 0, 1.7f}; rv.status = 0;
    bnoDiagnostics.observe(rv, millis());
    assert(bnoDiagnostics.plausible && bnoDiagnostics.status == 0); // plausibility != calibration
    // Retain records during UART congestion, and disclose bounded-ring overflow.
    txLength = sizeof(txBuffer); txOffset = 0;
    const auto pending = bnoTrace.size; serviceTraceOutput(); assert(bnoTrace.size == pending);
    for (unsigned n=0; n<40; ++n) m09_bno_reset(n);
    pump();
    assert(bnoTrace.dropped > 0 && bnoTrace.resetEvents == 43);
    txLength = txOffset = 0; serviceTraceOutput();
    assert(std::string(txBuffer, txLength).find("kind=RESET_COMPLETE") != std::string::npos);
    assert(simulated::commands.size() == commands);
    puts("PASS BNO trace: callback timestamps, idle asynchronous query, unavailable cause, overflow, malformed labels; no motor commands");
}
