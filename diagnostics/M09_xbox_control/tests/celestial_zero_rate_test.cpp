// Reuse the integration fixture; drive accepted sensor residuals explicitly to
// hold exact feed-forward/correction cancellation without physical sensor drift.
#define main low_rate_fixture_main
#include "celestial_low_rate_test.cpp"
#undef main
namespace {
double fixedResidual = 0;
void demand(double feedHz, double correctionHz) {
    const double conversion = YAW_TRACK_PULSES_PER_DEG * axisFeedbackStepSign(yawAxis);
    celestialTargetRate[0] = feedHz / conversion;
    auto &track = celestialTrackAxes[0];
    track.correction = correctionHz / conversion;
    fixedResidual = track.correction == 0 ? 0 : copysign(
        APPROACH_DEADBAND_DEG + fabs(track.correction) / CELESTIAL_RESIDUAL_GAIN, track.correction);
    track.filteredError = fixedResidual;
    celestialRateKnown = true;
}
void samples(uint32_t duration, bool renew = true) {
    const uint32_t start = millis();
    while (millis() - start < duration) {
        simulated::advance(10);
        if (renew) celestialUpdatedAt = millis();
        sensorWorker.testDispatch(); serviceEncoders();
        simulated::lastSampleAt = millis();
        yawAxis.error = fixedResidual - celestialTargetRate[0] *
            (millis() - celestialUpdatedAt) / 1000.0;
        celestialSafety();
        if (poseStopping) servicePoseStop();
        else if (celestialActive()) serviceCelestialTrackAxis(yawAxis, millis(), millis());
    }
}
void zero() {
    demand(-0.117, 0.117);
    samples(300); active();
    CHECK(yawAxis.motion == Motion::TRACK_ZERO && !celestialTrackAxes[0].zeroStopping);
    CHECK(!yawMotor->isRunning());
    CHECK(fabs(celestialTargetRate[0] + celestialTrackAxes[0].correction) < 1e-12);
}
}
int main(int argc, char **argv) {
    CHECK(argc == 2);
    const std::string scenario = argv[1];
    simulated::baselineYaw = 20; simulated::baselinePitch = 8;
    simulated::encoderPlantFeedback = true;
    simulated::latchIdleForceStop = true; simulated::forceStopDrainMs = 20;
    Wire.encoder.present = Wire1.encoder.present = true;
    setup(); advance(2200); begin();
    simulated::motors[0].physicalSign = simulated::motors[1].physicalSign = 1;
    simulated::motors[0].stepPauses = true;
    simulated::motors[0].degreesPerStep = 1.0 / YAW_TRACK_PULSES_PER_DEG;
    const bool positive = scenario == "positive_zero_negative";
    demand(positive ? 0.020 : -0.020, 0); samples(300); active();
    CHECK(yawAxis.motion == Motion::TRACK && celestialTrackAxes[0].runConfirmed);
    const auto starts = simulated::commands.size();
    if (scenario == "unexpected_stop") {
        simulated::motors[0].drive = simulated::Drive::IDLE;
        samples(200);
        CHECK(poseStopping || commandIdle());
        CHECK(std::string(poseStopReason) == "TRACK motor stopped unexpectedly");
        CHECK(simulated::commands.size() == starts);
    } else if (scenario == "failed_reversal") {
        // A reversal passing through zero cannot start the opposite schedule
        // if the old queue/ramp refuses its stop request.
        simulated::motors[0].forceStopFails = true;
        demand(-0.117, 0.117); samples(100);
        CHECK(celestialTrackAxes[0].zeroStopping);
        demand(0.020, 0); samples(3200);
        CHECK(poseStopping);
        CHECK(std::string(poseStopReason) == "TRACK zero-rate stop timed out");
        CHECK(simulated::commands.size() == starts);
    } else {
        zero();
        CHECK(simulated::forceStops == 1);
        if (scenario == "near_zero") {
            for (unsigned n = 0; n < 100; ++n) {
                // Both signs inside the unrepresentable band: no actual
                // direction reversal or repeated stop/start command is needed.
                demand(-0.117, 0.117 + (n % 2 ? 0.001 : -0.001));
                samples(100); active(); CHECK(yawAxis.motion == Motion::TRACK_ZERO);
            }
            CHECK(simulated::commands.size() == starts && simulated::forceStops == 1);
        } else if (scenario == "long_zero") {
            // Even an error above the progress threshold is not a motor stall
            // while net motion demand is exactly zero (correction at its cap).
            demand(-0.520, 0.520); samples(210000); active();
            CHECK(fabs(fixedResidual) > TOLERANCE_DEG);
            CHECK(yawAxis.motion == Motion::TRACK_ZERO && !yawMotor->isRunning());
            CHECK(simulated::commands.size() == starts && simulated::forceStops == 1);
        } else if (scenario == "same_direction" || scenario == "positive_zero_negative" ||
                   scenario == "negative_zero_positive") {
            const int sign = scenario == "negative_zero_positive" ? 1 : -1;
            demand(sign * 0.020, 0); samples(500); active();
            CHECK(yawAxis.motion == Motion::TRACK && celestialTrackAxes[0].runConfirmed);
            CHECK(yawAxis.slewDirection == sign);
            CHECK(simulated::commands.size() == starts + 1);
            const auto &cmd = simulated::commands.back();
            CHECK(!cmd.wasBraking && !cmd.beforeStoppedSample);
        } else if (scenario == "lease") {
            samples(2990, false); active();
            samples(20, false);
            CHECK(!celestialActive() || poseStopping);
            CHECK(std::string(poseStopReason) == "target update lease expired after 3000 ms");
        } else if (scenario == "stop" || scenario == "takeover") {
            stop(); CHECK(commandIdle() && !celestialActive());
            if (scenario == "takeover") {
                simulated::serialInput += "JOG 0 0 0\n"; advance(20); CHECK(manualActive);
                simulated::serialInput += "JOG 100 100 0\n"; advance(20);
                CHECK(yawMotor->isRunning() && pitchMotor->isRunning());
            }
        } else CHECK(scenario == "exact_zero");
    }
    std::printf("PASS zero-rate: %s\n", scenario.c_str());
}
