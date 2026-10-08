// Real worker/lifecycle with injected ACK/handshake outcomes, no hardware I/O.
#include "../src/main.cpp"
#include <cassert>
#include <cstdio>
#include <string>

namespace {
bool feed = false, stopDuringInit = false, loseHostDuringInit = false;
uint32_t nextHost = 0, switchAt = 0, handledAt = 0, lastHost = 0;
void tick(uint32_t now) {
    motionWatchdog.check(now); commandWatchdog.check(now);
    if (switchAt && now >= switchAt && feed) {
        feed = false;
        if (stopDuringInit) simulated::serialInput += "STOP\n";
    }
    if (feed && now >= nextHost) {
        simulated::serialInput += "JOG 200 200 200\n";
        nextHost = now + 20; lastHost = now;
    }
    if (stopDuringInit && now >= switchAt && !handledAt && manualEnding) handledAt = now;
}
void advance(uint32_t ms) { const auto start = millis(); while (millis()-start < ms) loop(); }
void command(const char *text) { simulated::serialInput += std::string(text) + "\n"; advance(20); }
}
int main(int argc, char **argv) {
    assert(argc == 2); const std::string mode = argv[1];
    simulated::physicalPitchUsesRoll = false; simulated::independentTick = tick;
    Wire.encoder.present = Wire1.encoder.present = true;
    if (mode == "address_b" || mode == "reset_b" || mode == "accuracy_b") Wire1.bnoAddress = 0x4B;
    if (mode == "late_ack" || mode == "no_ack") simulated::bnoAckFails = true;
    if (mode == "retry" || mode == "slow_stop" || mode == "slow_lease") simulated::bnoInitialFailuresRemaining = 1;
    if (mode == "failed" || mode == "abort_retry") simulated::bnoInitFails = true;
    if (mode == "early_failed") simulated::bnoEarlyInitFails = true;
    if (mode == "bus_failed") simulated::busBInitFails = true;
    if (mode == "report_failed") simulated::reportInitFails = true;
    if (mode == "low_accuracy") simulated::accuracy = 1;
    if (mode == "accuracy_b") { simulated::accuracy = 0; simulated::reinitAccuracy = 3; }
    setup(); advance(50);
    assert(Wire.sda == 18 && Wire.scl == 19 && Wire1.sda == 4 && Wire1.scl == 5);
    assert(Wire.frequency == 100000 && Wire1.frequency == 100000);
    assert(Wire.timeout == 50 && Wire1.timeout == 50);
    assert(commandIdle() && poseMotorsStopped() && simulated::commands.empty());
    if (mode == "late_ack") {
        assert(sensorSnapshot.bnoInitStage == m09::BnoInitStage::NO_ACK && !bnoInitialized);
        simulated::bnoAckFails = false;
    } else if (mode == "abort_retry") {
        command("X"); advance(5000);
        assert(phase == Phase::ABORTED && simulated::bnoBeginCalls == 1 && poseMotorsStopped());
        puts("PASS BNO startup: abort cancels pending retries"); return 0;
    } else if (mode == "slow_stop" || mode == "slow_lease") {
        command("JOG 0 0 0"); feed = true; advance(600);
        assert(manualActive && yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
        simulated::bnoInitDelayMs = 1500;
        stopDuringInit = mode == "slow_stop"; loseHostDuringInit = !stopDuringInit;
        switchAt = 1100;
        advance(3000);
        assert(simulated::bnoBeginCalls == 2 && bnoInitialized && simulated::bnoSessionLeaks == 0);
        assert(commandIdle() && poseMotorsStopped());
        if (stopDuringInit) assert(handledAt >= switchAt && handledAt-switchAt <= 20 && simulated::forceStops == 0);
        if (loseHostDuringInit) assert(simulated::firstForceStopAt-lastHost >= 250 && simulated::firstForceStopAt-lastHost <= 260);
        assert(Wire.beginCalls == 1 && Wire1.beginCalls == 1);
        std::printf("PASS BNO startup: %s during 1500 ms handshake\n", mode.c_str()); return 0;
    } else if (mode == "report_failed") {
        assert(sensorSnapshot.bnoInitStage == m09::BnoInitStage::REPORT_FAILED);
        assert(bnoInitialized && !reportEnabled && !sensorSnapshot.bnoRetryPending);
        simulated::reportInitFails = false;
    }
    advance(4000);
    assert(Wire.beginCalls == 1 && Wire1.beginCalls == 1); // Never reset encoder buses to retry BNO.
    assert(simulated::bnoSessionLeaks == 0);
    assert(encoders.state(0).valid);
    if (mode == "bus_failed") {
        assert(!encoders.state(1).busAvailable && simulated::bnoBeginCalls == 0);
        assert(sensorSnapshot.bnoInitStage == m09::BnoInitStage::BUS_UNAVAILABLE);
    } else {
        assert(encoders.state(1).valid && encoders.state(1).reads > 20);
        if (mode == "no_ack" || mode == "failed" || mode == "early_failed") {
            assert(!bnoInitialized && !reportEnabled && !sensorSnapshot.bnoRetryPending);
            assert(sensorSnapshot.bnoAttempts == 3);
            assert(Serial.output.find("kind=STARTUP_EXHAUSTED") != std::string::npos);
            assert(sensorSnapshot.bnoInitStage == (mode == "no_ack" ? m09::BnoInitStage::NO_ACK : m09::BnoInitStage::HANDSHAKE_FAILED));
            const auto calls = simulated::bnoBeginCalls;
            advance(6000); assert(simulated::bnoBeginCalls == calls); // Bounded, no reset storm.
            if (mode != "no_ack") assert(calls == 3 && simulated::sh2CloseCalls == 3);
        } else {
            assert(bnoInitialized && reportEnabled && bnoValid);
            assert(sensorSnapshot.bnoInitStage == m09::BnoInitStage::READY && !sensorSnapshot.bnoRetryPending);
            assert(simulated::lastBnoWire == &Wire1 && simulated::lastBnoAddress == Wire1.bnoAddress);
            if (mode == "retry") {
                assert(simulated::bnoBeginCalls == 2 && simulated::sh2CloseCalls == 1);
                assert(simulated::bnoBeginTimes[1]-simulated::bnoBeginTimes[0] >= 1000);
            } else if (mode == "accuracy_b") assert(simulated::bnoBeginCalls == 2 && bnoAccuracy == 3);
            else assert(simulated::bnoBeginCalls == 1);
            if (mode == "low_accuracy") assert(!northUsable && bnoAccuracy == 1);
            if (mode == "reset_b") {
                simulated::bnoResetAt = millis(); advance(1700);
                assert(bnoValid && reportEnabled && simulated::bnoBeginCalls == 1 && simulated::lastBnoAddress == 0x4B);
            }
        }
    }
    assert(simulated::commands.empty() && commandIdle() && poseMotorsStopped());
    assert(Serial.output.find("BNO_INIT bus=B") != std::string::npos);
    std::printf("PASS BNO startup: %s\n", mode.c_str());
}
