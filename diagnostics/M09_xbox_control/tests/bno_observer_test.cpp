#include "../tools/bno_observer/main.cpp"
#include <cassert>
#include <cstdio>
int main() {
    setup();
    simulated::serialInput = "POSE 20 10 100\nMOVE 5 5 100\nJOG 0 0\nJOG 500 500\n";
    while (millis() < 70000) loop();
    assert(bnoHealth.freshSamples > 6000 && bnoAccuracy == 3);
    assert(!pitchReady && !referenceSet);
    assert(simulated::commands.empty());
    for (auto &motor : simulated::motors) assert(motor.drive == simulated::Drive::IDLE);
    simulated::serialInput = "X\n";
    for (unsigned i = 0; i < 100; ++i) loop();
    assert(finalPrinted && phase == Phase::ABORTED && simulated::commands.empty());
    std::puts("PASS: 70 s sensor observer with accuracy 3 cannot issue motor commands, including injected POSE/MOVE/JOG; X latches abort.");
}
