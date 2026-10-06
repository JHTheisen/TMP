// Exercise the INSTALLED, unmodified FastAccelStepper 1.2.7 ramp implementation.
// Build with -DTEST and its src include path, without the firmware motor stub.
// Its PC backend uses the same 16 MHz tick scale as this ESP32 configuration.
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <initializer_list>
#define printf(...) ((void)0)
#define puts(...) ((void)0)
#include "log2/Log2Representation.cpp"
#include "fas_ramp/RampCalculator.cpp"
#include "fas_ramp/RampControl.cpp"
#include "fas_ramp/RampGenerator.cpp"
#undef printf
#undef puts
void inject_fill_interrupt(int) {}
namespace {
void configure(RampGenerator &ramp) {
    ramp.init();
    assert(ramp.setAcceleration(250) == 0);
    ramp.setSpeedInTicks(800000000UL); // 0.020 Hz: 50 seconds, not 50 ms.
}
uint32_t next(RampGenerator &ramp, queue_end_s &end) {
    NextCommand cmd = {};
    ramp.getNextCommand(&end, &cmd);
    ramp.afterCommandEnqueued(&cmd);
    end.pos += cmd.command.count_up ? cmd.command.steps : -cmd.command.steps;
    end.count_up = cmd.command.count_up;
    return cmd.command.ticks * (cmd.command.steps ? cmd.command.steps : 1U);
}
}
int main() {
    RampGenerator ramp = {};
    queue_end_s end = {};
    configure(ramp);
    assert(ramp.startRun(true) == MOVE_OK);
    next(ramp, end); // First pulse plus a long remaining pause.
    assert(ramp.getCurrentPeriodInUs() == 50000000UL);
    // A newer requested speed does not erase the already pending slow period.
    ramp.setSpeedInTicks(80000000UL); // 0.200 Hz
    ramp.applySpeedAcceleration();
    next(ramp, end);
    assert(ramp.getCurrentPeriodInUs() == 50000000UL);
    ramp.initiateStop();
    uint64_t ticks = 0;
    while (ramp.isRampGeneratorActive()) {
        ticks += next(ramp, end);
        assert(ticks < 103ULL * TICKS_PER_S);
    }
    assert(ticks > 99ULL * TICKS_PER_S); // Pending interval PLUS one ramp-down step.

    // forceStop bypasses the long unqueued pause; physical queue drain remains
    // separate (covered by the firmware fixture's asynchronous drain model).
    assert(ramp.startRun(true) == MOVE_OK);
    next(ramp, end);
    ramp.forceStop();
    assert(next(ramp, end) == 0 && !ramp.isRampGeneratorActive());
    // Reasserting forceStop after ramp idle leaves its flag pending.
    ramp.forceStop();
    assert(ramp.startRun(false) == MOVE_OK && ramp.isRampGeneratorActive());
    const int32_t before = end.pos;
    assert(next(ramp, end) == 0 && !ramp.isRampGeneratorActive());
    assert(end.pos == before); // Failed launch emitted no steps/queue commands.
    // The consumed flag is cleared; a bounded retry starts the requested rate.
    assert(ramp.startRun(false) == MOVE_OK);
    assert(next(ramp, end) > 0 && ramp.isRampGeneratorActive());
    assert(!end.count_up);
    // The hardware log's -0.117127 + 0.115652 Hz demand is below the 5 mHz
    // representable floor. Cancel its old slow schedule rather than sending a
    // zero speed to the library or waiting through ordinary ramp-down pauses.
    assert(fabs(-0.117127 + 0.115652) < 0.005);
    assert(ramp.getCurrentPeriodInUs() >= 3000000UL && ramp.stepsToStop() <= 1);
    ramp.forceStop();
    const int32_t zeroPosition = end.pos;
    assert(next(ramp, end) == 0 && !ramp.isRampGeneratorActive());
    assert(end.pos == zeroPosition);
    // Repeated idle service does not submit another forceStop, so each fresh
    // same-direction/opposite-direction run starts without consuming a latch.
    for (bool direction : {false, true, false}) {
        assert(ramp.startRun(direction) == MOVE_OK);
        assert(next(ramp, end) > 0 && ramp.isRampGeneratorActive());
        assert(end.count_up == direction);
        ramp.forceStop();
        assert(next(ramp, end) == 0 && !ramp.isRampGeneratorActive());
    }
    puts("PASS installed FAS: long pause, normal stop, forced stop, continuous restart");
}
