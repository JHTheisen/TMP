#include "../src/main.cpp"
#include <cassert>
#include <cstdio>

void advance(uint32_t ms) { const auto start=millis(); while (millis()-start<ms) loop(); }
int main() {
    simulated::physicalPitchUsesRoll=false;
    setup(); advance(250);
    simulated::serialInput="JOG 0 0 0\n"; advance(20);
    simulated::serialInput="JOG 100 100 100\n"; advance(30);
    assert(manualActive && yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
    m09::SensorSample sample, ignored;
    while (sensorWorker.samples.pop(ignored)) {}
    sample.event.sensorId=SH2_ROTATION_VECTOR; sample.event.status=3;
    sample.event.un.rotationVector={1,0,0,0,0.125f};
    sample.receivedMs=millis()-200;
    sample.epoch=sensorWorker.resetEpoch.load();
    const auto goodBefore=lastBnoGood, readsBefore=simulated::bnoReads;
    assert(sensorWorker.samples.push(sample));
    serviceBno();
    assert(lastBnoGood==goodBefore && std::string(bnoDiagnostics.reason)=="stale_handoff");
    assert(simulated::bnoReads==readsBefore); // Foreground consumption performs no I/O.
    sample.receivedMs=millis(); assert(sensorWorker.samples.push(sample));
    // Overflow diagnostic events deliberately. Reset invalidation is independent
    // of either queue and cannot be hidden by old, otherwise valid sample data.
    for (unsigned n=0;n<40;++n) sensorWorker.onReset(n);
    const auto epoch=sensorWorker.resetEpoch.load();
    serviceBno();
    assert(sensorResetEpoch==epoch && !bnoValid && manualActive && !commandWatchdog.tripped());
    assert(lastBnoGood==goodBefore && bnoDiagnostics.reason==std::string("reset"));
    assert(yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
    // A post-reset report keeps its actual receipt time, not consumption time.
    sample.epoch=epoch; sample.receivedMs=millis()-5;
    assert(sensorWorker.samples.push(sample)); serviceBno();
    assert(bnoValid && lastBnoGood==sample.receivedMs && bnoDiagnostics.rotationAt==sample.receivedMs);
    puts("PASS sensor results: old samples stay stale, reset survives queue overflow, no foreground I/O or manual disarm");
}
