#include <cassert>
#include <iostream>
#include "../src/main.cpp"

int main(int argc, char **argv) {
    assert(argc == 2);
    const std::string scenario = argv[1];
    simulated::baselinePitch = -10;
    simulated::motors[1].degreesPerStep = 0.015; // Documented 1600 pulses/rev, 15:1.
    simulated::motors[1].physicalSign = scenario == "negative" ? -1 : 1;
    if (scenario == "no_response") simulated::motors[1].frozen = true;
    if (scenario == "encoder_good" || scenario == "encoder_weak" || scenario == "encoder_short") simulated::encoderPresent = true;
    if (scenario == "encoder_weak") simulated::encoderStatus = 0x30; // MD and ML set together.
    if (scenario == "encoder_short") simulated::encoderShortRead = true;
    setup();
    simulated::independentTick = [](uint32_t now) { watchdog.check(now); };
    for (unsigned i = 0; i < 2500; ++i) loop();
    assert(simulated::commands.empty());
    assert(simulated::connectedPins.size() == 1 && simulated::connectedPins[0] == 12);
    simulated::pendingSerial = 'G';
    bool injected = false;
    for (unsigned i = 0; i < 70000 && phase != Phase::FAULT && phase != Phase::DONE; ++i) {
        if (!injected && phase == Phase::MOVING && millis() - phaseAt > 500) {
            injected = true;
            if (scenario == "abort") simulated::pendingSerial = 'X';
            if (scenario == "stale") { simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 1000; }
            if (scenario == "blocked_bno") simulated::blockBnoMs = 1000;
            if (scenario == "reset") simulated::bnoResetAt = millis();
            if (scenario == "excursion") simulated::pitchDisturbance = 7;
            if (scenario == "timeout") simulated::motors[1].neverStops = true;
            if (scenario == "blocked_uart") simulated::serialWriteSpace = 0;
        }
        if (scenario == "blocked_uart" && injected && phase == Phase::PREMOVE) simulated::serialWriteSpace = 128;
        loop();
    }
    simulated::serialWriteSpace = 128;
    for (unsigned i = 0; i < 200; ++i) loop();
    const bool fault = scenario == "abort" || scenario == "stale" || scenario == "blocked_bno" ||
        scenario == "reset" || scenario == "excursion" || scenario == "timeout";
    if (fault) {
        assert(phase == Phase::FAULT && simulated::commands.size() == 1);
        assert(!motor->isRunning());
        if (scenario == "stale" || scenario == "blocked_bno") {
            assert(watchdog.tripped());
            assert(simulated::firstForceStopAt - lastFresh <= 150);
        }
    } else {
        assert(phase == Phase::DONE && simulated::commands.size() == 2);
        assert(simulated::commands[0].steps == 200 && simulated::commands[1].steps == -200);
        assert(simulated::commands[1].at - simulated::commands[0].at >= 8000);
        assert(motor->getCurrentPosition() == 0);
        if (scenario == "no_response") assert(directions[0] == 0 && directions[1] == 0);
        else {
            const int expected = scenario == "negative" ? -1 : 1;
            assert(directions[0] == expected && directions[1] == -expected);
            assert(fabs(responses[0] - 3.0 * expected) < 0.01);
        }
        if (scenario == "encoder_good") assert(Serial.output.find("BNO_numeric_agreement=SAME") != std::string::npos);
        if (scenario == "encoder_weak") assert(Serial.output.find("BNO_numeric_agreement=INCONCLUSIVE") != std::string::npos);
        if (scenario == "encoder_short") assert(Serial.output.find("AS5600 comparison=UNAVAILABLE") != std::string::npos);
    }
    for (const auto &command : simulated::commands) {
        assert(command.stepPin == 12 && command.speed == 40 && command.acceleration == 240 && !command.continuous);
    }
    const size_t count = simulated::commands.size();
    simulated::pendingSerial = 'G';
    for (unsigned i = 0; i < 1000; ++i) loop();
    assert(simulated::commands.size() == count);
    std::cout << scenario << " PASS\n";
}
