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
    // An old but plausible newer receipt is retained for display, never made
    // fresh for closed-loop control. Poor quality is metadata, not no data.
    hasPlausibleOrientation = false;
    sample.receivedMs = millis() - 200; sample.event.status = 0;
    assert(sensorWorker.samples.push(sample)); serviceBno();
    assert(hasPlausibleOrientation && lastPlausibleAt == sample.receivedMs && lastPlausibleAccuracy == 0);
    const auto retainedAt = lastPlausibleAt;
    sample.receivedMs = millis(); sample.event.un.rotationVector.real = NAN;
    assert(sensorWorker.samples.push(sample)); serviceBno();
    assert(hasPlausibleOrientation && lastPlausibleAt == retainedAt);
    sample.event.un.rotationVector = {0,0,0,0,0};
    assert(sensorWorker.samples.push(sample)); serviceBno();
    assert(lastPlausibleAt == retainedAt);
    // Powered malformed sample: finite Euler math used to admit this norm.
    sample.event.un.rotationVector = {-0.0000610351562f,-0.0141601562f,
        -0.0000610351562f,-0.0000610351562f,-0.000244140625f};
    sample.receivedMs = millis();
    assert(sensorWorker.samples.push(sample)); serviceBno();
    assert(!bnoDiagnostics.accepted && !bnoDiagnostics.plausible);
    assert(lastPlausibleAt == retainedAt && manualActive);
    assert(yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
    simulated::serialInput="STOP\n"; advance(1500);
    assert(commandIdle() && poseMotorsStopped());
    puts("PASS sensor results: old samples stay stale, reset survives queue overflow, no foreground I/O or manual disarm");
}
