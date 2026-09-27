#include <cassert>
#include <cstdint>
#include <iostream>
#include "../src/motion_watchdog.h"

using milestone7::MotionWatchdog;

namespace {
struct Fixture {
    FastAccelStepperEngine engine;
    FastAccelStepper *yaw;
    FastAccelStepper *pitch;
    MotionWatchdog watchdog;
    Fixture() {
        engine.init();
        yaw = engine.stepperConnectToPin(33);
        pitch = engine.stepperConnectToPin(12);
        assert(yaw && pitch);
        assert(watchdog.begin(yaw, pitch, 150));
    }
};

void exactBoundaryAndLatch() {
    Fixture fixture;
    auto &watchdog = fixture.watchdog;
    const unsigned stopsBefore = simulated::forceStops;
    watchdog.check(100000);
    assert(!watchdog.tripped());
    assert(simulated::forceStops == stopsBefore);
    assert(watchdog.arm(1000));
    watchdog.check(1149);
    assert(!watchdog.tripped());
    assert(simulated::forceStops == stopsBefore);
    watchdog.check(1150);
    assert(watchdog.tripped());
    assert(simulated::forceStops == stopsBefore + 2);

    // A command racing with the first stop cannot escape the next check.
    watchdog.check(1160);
    assert(simulated::forceStops == stopsBefore + 4);
    watchdog.recordFresh(1161);
    assert(watchdog.tripped());
    assert(!watchdog.arm(1161));
    watchdog.disarm();
    const unsigned stoppedAtDisarm = simulated::forceStops;
    watchdog.check(1000000);
    watchdog.recordFresh(1000001);
    assert(watchdog.tripped());
    assert(!watchdog.arm(1000001));
    assert(simulated::forceStops == stoppedAtDisarm);
}

void timelyFreshSampleExtendsDeadline() {
    Fixture fixture;
    auto &watchdog = fixture.watchdog;
    assert(watchdog.arm(10000));
    watchdog.recordFresh(10100);
    watchdog.check(10249);
    assert(!watchdog.tripped());
    watchdog.check(10250);
    assert(watchdog.tripped());
}

void delayedReadCannotEraseOutage() {
    Fixture fixture;
    auto &watchdog = fixture.watchdog;
    assert(watchdog.arm(500));
    // Simulate the foreground recovering exactly at the stale threshold before
    // the independent timer callback has had a chance to execute.
    watchdog.recordFresh(650);
    assert(watchdog.tripped());
    watchdog.recordFresh(651);
    assert(watchdog.tripped());
}

void wraparoundAndConcurrentTimestamp() {
    Fixture fixture;
    auto &watchdog = fixture.watchdog;
    const uint32_t start = UINT32_MAX - 74;
    assert(watchdog.arm(start));
    watchdog.check(static_cast<uint32_t>(start + 149));
    assert(!watchdog.tripped());
    watchdog.check(static_cast<uint32_t>(start + 150));
    assert(watchdog.tripped());

    Fixture futureFixture;
    auto &futureWatchdog = futureFixture.watchdog;
    assert(futureWatchdog.arm(1000));
    futureWatchdog.recordFresh(1001);
    // A timer can capture now immediately before the foreground publishes a
    // sample from the next millisecond. This is not a 49-day sensor outage.
    futureWatchdog.check(1000);
    assert(!futureWatchdog.tripped());
    futureWatchdog.check(1150);
    assert(!futureWatchdog.tripped());
    futureWatchdog.check(1151);
    assert(futureWatchdog.tripped());
}

void disarmedAndInitializationFailures() {
    Fixture fixture;
    auto &watchdog = fixture.watchdog;
    assert(watchdog.arm(1000));
    watchdog.disarm();
    const unsigned stopsBefore = simulated::forceStops;
    watchdog.check(5000);
    assert(!watchdog.tripped());
    assert(simulated::forceStops == stopsBefore);

    MotionWatchdog uninitialized;
    assert(!uninitialized.arm(0));
    assert(!uninitialized.begin(nullptr, fixture.pitch, 150));
    assert(!uninitialized.begin(fixture.yaw, nullptr, 150));
    assert(!uninitialized.begin(fixture.yaw, fixture.pitch, 0));
    assert(!uninitialized.begin(fixture.yaw, fixture.pitch, 0x80000000UL));
    assert(uninitialized.begin(fixture.yaw, fixture.pitch, 150));
    assert(!uninitialized.begin(fixture.yaw, fixture.pitch, 150));
    // Arming must preserve the actual BNO timestamp even when already stale.
    assert(uninitialized.arm(1000));
    uninitialized.check(1200);
    assert(uninitialized.tripped());
}
void optionalCarriageStopsWithBothAngularAxes() {
    FastAccelStepperEngine engine;
    engine.init();
    FastAccelStepper *yaw = engine.stepperConnectToPin(33);
    FastAccelStepper *pitch = engine.stepperConnectToPin(12);
    FastAccelStepper *carriage = engine.stepperConnectToPin(22);
    MotionWatchdog watchdog;
    assert(yaw && pitch && carriage);
    assert(watchdog.begin(yaw, pitch, 150, carriage));
    assert(yaw->runForward() == MOVE_OK && pitch->runBackward() == MOVE_OK && carriage->move(450) == MOVE_OK);
    assert(watchdog.arm(1000));
    watchdog.check(1149);
    assert(yaw->isRunning() && pitch->isRunning() && carriage->isRunning());
    const unsigned before = simulated::forceStops;
    watchdog.check(1150);
    assert(watchdog.tripped());
    assert(!yaw->isRunning() && !pitch->isRunning() && !carriage->isRunning());
    assert(simulated::forceStops == before + 3);
    // A carriage command racing the first stop is stopped on the next tick.
    assert(carriage->move(-450) == MOVE_OK);
    watchdog.check(1160);
    assert(!carriage->isRunning() && simulated::forceStops == before + 6);
}
void recoveryRequiresStoppedAndDisarmed() {
    FastAccelStepperEngine engine;
    engine.init();
    FastAccelStepper *yaw = engine.stepperConnectToPin(33);
    FastAccelStepper *pitch = engine.stepperConnectToPin(12);
    FastAccelStepper *carriage = engine.stepperConnectToPin(22);
    MotionWatchdog watchdog;
    assert(!watchdog.configured());
    assert(!watchdog.clearTripWhenStopped());
    assert(watchdog.begin(yaw, pitch, 250, carriage));
    assert(watchdog.configured());
    assert(watchdog.arm(1000));
    assert(watchdog.armed());
    assert(!watchdog.clearTripWhenStopped());
    watchdog.check(1250);
    assert(watchdog.tripped());
    assert(!watchdog.clearTripWhenStopped());
    watchdog.disarm();
    assert(!watchdog.armed());

    // A single moving axis must prevent clearing the old stop, including the
    // optional carriage. Clearing a trip is never itself a movement command.
    for (FastAccelStepper *motor : {yaw, pitch, carriage}) {
        assert(motor->runForward() == MOVE_OK);
        assert(!watchdog.clearTripWhenStopped());
        assert(watchdog.tripped());
        motor->forceStop();
    }
    const size_t commandsBefore = simulated::commands.size();
    assert(watchdog.clearTripWhenStopped());
    assert(!watchdog.tripped() && !watchdog.armed());
    assert(simulated::commands.size() == commandsBefore);
    watchdog.check(100000);
    assert(!watchdog.tripped());

    // Rearming creates a new lease. An earlier captured callback timestamp
    // must not interpret the new timestamp as a huge unsigned stale interval.
    assert(watchdog.arm(200000));
    assert(yaw->runForward() == MOVE_OK);
    const unsigned stopsBefore = simulated::forceStops;
    watchdog.check(1251);
    watchdog.check(200249);
    assert(yaw->isRunning() && !watchdog.tripped());
    assert(simulated::forceStops == stopsBefore);
    watchdog.check(200250);
    assert(!yaw->isRunning() && watchdog.tripped());
}
} // namespace

int main() {
    exactBoundaryAndLatch();
    timelyFreshSampleExtendsDeadline();
    delayedReadCannotEraseOutage();
    wraparoundAndConcurrentTimestamp();
    disarmedAndInitializationFailures();
    optionalCarriageStopsWithBothAngularAxes();
    recoveryRequiresStoppedAndDisarmed();
    std::cout << "M09 watchdog tests passed (stopped recovery and two-/three-axis decisions, not ESP scheduling).\n";
}
