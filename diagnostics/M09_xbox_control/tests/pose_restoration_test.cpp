// Exercise the restored first-POSE path and asynchronous feedback ordering.
// Synthetic motor response checks control decisions, not physical calibration.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
void require(bool ok, const char *expression, int line) {
    if (!ok) {
        std::fprintf(stderr, "FAIL restored POSE line %d: %s\n%s", line, expression, Serial.output.c_str());
        std::exit(1);
    }
}
#define CHECK(value) require((value), #value, __LINE__)
void tick(uint32_t now) { motionWatchdog.check(now); commandWatchdog.check(now); }
void advance(uint32_t duration) {
    const uint32_t start = millis();
    while (millis() - start < duration) loop();
}
void line(const std::string &value) { simulated::serialInput += value + "\n"; advance(10); }
void finishPose() {
    const uint32_t start = millis();
    while (poseActive && millis() - start < 95000) loop();
    advance(100);
    CHECK(commandIdle() && !poseActive && manualMotorsStopped());
    CHECK(Serial.output.find("FINAL RESULT: PASS") != std::string::npos);
}
void checkPrecisionCommands() {
    CHECK(!simulated::commands.empty());
    for (const auto &command : simulated::commands) {
        CHECK(!command.continuous && command.speed <= 80);
        if (command.stepPin != tmp_hardware::CARRIAGE_STEP_PIN)
            CHECK(std::abs(command.steps) <= 16 && command.acceleration == ACCELERATION);
    }
}
void reject(double yaw, double pitch, int64_t carriage = 0) {
    const size_t commands = simulated::commands.size();
    beginPose(yaw, pitch, carriage, false);
    CHECK(commandIdle() && !poseActive && simulated::commands.size() == commands);
}

// Direct queue injection permits different acquisition/delivery times without
// asking the simulated worker to fabricate a synchronous sample at delivery.
void sample(uint32_t received, double yaw, double pitch) {
    m09::SensorSample value;
    value.receivedMs = received;
    value.epoch = sensorWorker.resetEpoch.load(std::memory_order_acquire);
    value.event.sensorId = SH2_ROTATION_VECTOR;
    value.event.status = 3;
    value.event.sequence = static_cast<uint8_t>(acceptedBnoSequence + 1);
    value.event.timestamp = static_cast<uint64_t>(received) * 1000;
    const double hy = yaw / RAD_TO_DEG / 2, hp = pitch / RAD_TO_DEG / 2;
    const double hr = simulated::rawBnoRoll / RAD_TO_DEG / 2;
    const double cy = cos(hy), sy = sin(hy), cp = cos(hp), sp = sin(hp), cr = cos(hr), sr = sin(hr);
    value.event.un.rotationVector = {
        static_cast<float>(cy * cp * cr + sy * sp * sr),
        static_cast<float>(cy * cp * sr - sy * sp * cr),
        static_cast<float>(cy * sp * cr + sy * cp * sr),
        static_cast<float>(sy * cp * cr - cy * sp * sr), 0.125f};
    CHECK(sensorWorker.samples.push(value));
    serviceBno();
}
void heldPose() {
    m09::SensorSample pending;
    while (sensorWorker.samples.pop(pending)) {}
    operation = Operation::POSE; poseActive = true; finalPrinted = false;
    phase = Phase::MOVING; poseNeedsBno = true; yawRequired = accuracyRequired = true;
    resetPoseAxis(yawAxis, heading.continuous, 0);
    resetPoseAxis(pitchAxis, physicalPitch(), 0);
    carriageTarget = carriageMotor->getCurrentPosition(); carriagePending = false;
    controlStartedAt = millis(); lastControlSample = bnoHealth.freshSamples;
    settleSamples = 0; window.active = false;
    CHECK(motionWatchdog.arm(lastBnoGood));
}
}

int main(int argc, char **argv) {
    if (argc != 2) return 1;
    const std::string scenario = argv[1];
    simulated::independentTick = tick;
    simulated::physicalPitchUsesRoll = false;
    simulated::rawBnoRoll = -13;
    simulated::baselineYaw = 20; simulated::baselinePitch = 8;
    if (scenario == "pitch_low" || scenario == "unqualified") simulated::accuracy = 0;
    // Low startup accuracy triggers one established recovery at 2000 ms;
    // allow the independent pitch reference window to qualify after that reset.
    setup(); advance(simulated::accuracy < BNO_MIN_ACCURACY ? 3500 : 2200);
    CHECK(commandIdle() && pitchReady && simulated::commands.empty());
    CHECK(yawPulsesPerDegree == 0 && pitchPulsesPerDegree == 0);
    CHECK(fabs(physicalPitch() - 8) < 0.001);
    CHECK(fabs(orientation.roll - simulated::rawBnoRoll) < 0.001);

    if (scenario == "first_pose" || scenario == "first_carriage" || scenario == "pitch_low") {
        line(scenario == "pitch_low" ? "MOVE 0 1 0" :
            (scenario == "first_carriage" ? "POSE 21 9 100" : "POSE 21 9 0"));
        CHECK(poseActive);
        finishPose(); checkPrecisionCommands();
        CHECK(fabs(physicalPitch() - 9) <= TOLERANCE_DEG);
        CHECK(fabs(shortestDifference(scenario == "pitch_low" ? 20 : 21, orientation.heading)) <= TOLERANCE_DEG);
        CHECK(carriageMotor->getCurrentPosition() == (scenario == "first_carriage" ? 100 : 0));
        CHECK(pitchPulsesPerDegree > 0);
        if (scenario != "pitch_low") CHECK(yawPulsesPerDegree > 0);
        else CHECK(yawPulsesPerDegree == 0 && !yawMotor->isRunning());
    } else if (scenario == "unqualified") {
        pitchReady = referenceSet = northUsable = false;
        beginPose(1, 1, 0, true);
        CHECK(poseActive && yawRequired && !accuracyRequired);
        finishPose(); checkPrecisionCommands();
        CHECK(fabs(shortestDifference(21, orientation.heading)) <= TOLERANCE_DEG);
        CHECK(fabs(physicalPitch() - 9) <= TOLERANCE_DEG);
    } else if (scenario == "fallback_bounds") {
        reject(23.1, 8); reject(20, 11.1); reject(16.9, 8); reject(20, 4.9);
        line("MOVE 3 3 0"); CHECK(poseActive);
        finishPose(); checkPrecisionCommands();
        CHECK(fabs(shortestDifference(23, orientation.heading)) <= TOLERANCE_DEG);
        CHECK(fabs(physicalPitch() - 11) <= TOLERANCE_DEG);
    } else if (scenario == "admission") {
        bnoValid = false; reject(21, 9); bnoValid = true;
        reportEnabled = false; reject(21, 9); reportEnabled = true;
        bnoWatchdogReady = false; reject(21, 9); bnoWatchdogReady = true;
        reject(20, 75); reject(20, -75); reject(186, 8);
        const uint32_t savedReceipt = lastBnoGood;
        lastBnoGood = millis() - BNO_STALE_MS; reject(21, 9); lastBnoGood = savedReceipt;
        CHECK(simulated::commands.empty());
    } else if (scenario == "yaw_hold") {
        line("POSE 20 9 0"); CHECK(poseActive && yawPulsesPerDegree == 0);
        advance(50); simulated::yawDisturbance = 0.5;
        finishPose(); checkPrecisionCommands();
        bool correctedYaw = false;
        for (const auto &command : simulated::commands)
            if (command.stepPin == tmp_hardware::YAW_STEP_PIN) correctedYaw = true;
        CHECK(correctedYaw && fabs(shortestDifference(20, orientation.heading)) <= TOLERANCE_DEG);
    } else if (scenario == "yaw_recovered") {
        simulated::accuracy = 1; advance(30);
        CHECK(referenceSet && bnoAccuracy == 1);
        const double requestedYaw = heading.continuous + shortestDifference(19.8, orientation.heading);
        line("POSE 19.8 9 0");
        CHECK(poseActive && yawRequired && fabs(yawAxis.target - requestedYaw) < 0.001);
        simulated::accuracy = 3; simulated::yawDisturbance = 0.5;
        finishPose(); checkPrecisionCommands();
        CHECK(fabs(shortestDifference(19.8, orientation.heading)) <= TOLERANCE_DEG);
    } else if (scenario == "velocity_receipt") {
        heldPose(); const uint32_t base = millis();
        simulated::now = base + 20; sample(base + 10, 20, 8); serviceAxes();
        yawAxis.target += 1;
        simulated::now = base + 130; sample(base + 110, 21, 8); serviceAxes();
        CHECK(poseActive && fabs(yawAxis.angularSpeed - 10) < 0.001);
        CHECK(yawAxis.velocityAt == base + 110 && simulated::commands.empty());
    } else if (scenario == "observe_receipt" || scenario == "receipt_rollover") {
        if (scenario == "receipt_rollover") {
            simulated::now = UINT32_MAX - 80;
            lastBnoGood = millis() - 10;
            bnoHealth.lastFreshOrStartMs = lastBnoGood;
        }
        heldPose(); const uint32_t base = millis();
        yawAxis.target += 1;
        yawAxis.initialError = yawAxis.bestError = yawAxis.progressError = 1;
        yawAxis.motion = Motion::PRECISION; yawAxis.observing = true; yawAxis.stoppedAt = base + 30;
        simulated::now = base + 130; sample(base + 20, 20, 8); serviceAxes();
        CHECK(poseActive && yawAxis.observing && simulated::commands.empty());
        simulated::now = base + 150; sample(base + 50, 20, 8); serviceAxes();
        CHECK(poseActive && yawAxis.observing && simulated::commands.empty());
        simulated::now = base + 180; sample(base + 130, 20, 8); serviceAxes();
        CHECK(poseActive && !yawAxis.observing && !simulated::commands.empty());
    } else if (scenario == "settle_receipt") {
        heldPose(); const uint32_t base = millis();
        simulated::now = base + 20; sample(base + 10, 20, 8); serviceAxes();
        for (unsigned n = 0; n <= 90; ++n) {
            const uint32_t receipt = base + 130 + n * 10;
            simulated::now = receipt + (n ? 130 : 10);
            sample(receipt, 20, 8); serviceAxes();
            CHECK(poseActive);
        }
        CHECK(settleSamples >= 30 && settleLastSample - settleStarted == 900);
        for (unsigned n = 91; n <= 100; ++n) {
            const uint32_t receipt = base + 130 + n * 10;
            simulated::now = receipt + 130;
            sample(receipt, 20, 8); serviceAxes();
            if (n < 100) CHECK(poseActive);
        }
        CHECK(commandIdle() && !poseActive && pitchAxis.settled && yawAxis.settled);
        CHECK(simulated::commands.empty());
    } else if (scenario == "baseline_receipt") {
        m09::SensorSample pending;
        while (sensorWorker.samples.pop(pending)) {}
        referenceSet = pitchReady = northUsable = false;
        startBaseline(); const uint32_t base = millis();
        for (unsigned n = 1; n <= 99; ++n) {
            const uint32_t receipt = base + n * 10;
            simulated::now = receipt + (n == 1 ? 10 : 100);
            sample(receipt, 20, 8);
        }
        CHECK(millis() - window.started >= BASELINE_MS && window.pitch.n >= 30);
        CHECK(!stablePitchBaseline() && !stableBaseline());
        simulated::now = base + 1100; sample(base + 1000, 20, 8);
        CHECK(stablePitchBaseline() && stableBaseline());
    } else if (scenario == "sample_order") {
        m09::SensorSample pending;
        while (sensorWorker.samples.pop(pending)) {}
        const uint32_t base = millis();
        simulated::now = base + 20; sample(base + 10, 20, 8);
        const uint32_t accepted = bnoHealth.freshSamples;
        CHECK(lastBnoGood == base + 10);
        simulated::now = base + 30; sample(base + 10, 21, 9);
        CHECK(bnoHealth.freshSamples == accepted && lastBnoGood == base + 10);
        CHECK(fabs(orientation.heading - 20) < 0.001 && fabs(physicalPitch() - 8) < 0.001);
        simulated::now = base + 40; sample(base + 9, 22, 10);
        CHECK(bnoHealth.freshSamples == accepted && lastBnoGood == base + 10);
        CHECK(fabs(orientation.heading - 20) < 0.001 && fabs(physicalPitch() - 8) < 0.001);
        simulated::now = base + 50; sample(base + 20, 20, 8);
        CHECK(bnoHealth.freshSamples == accepted + 1 && lastBnoGood == base + 20);
    } else {
        std::fprintf(stderr, "Unknown restored POSE scenario: %s\n", scenario.c_str()); return 2;
    }
    std::printf("PASS restored POSE: %s\n", scenario.c_str());
}
