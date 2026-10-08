// Real firmware dispatcher with opt-in long step pauses and queue-drain latency.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>
namespace {
void check(bool ok, const char *expression, int line) {
    if (!ok) {
        std::fprintf(stderr, "FAIL low-rate line %d: %s\n%s", line, expression, Serial.output.c_str());
        std::exit(1);
    }
}
#define CHECK(x) check((x), #x, __LINE__)
uint32_t updateAt = 0, sequence = 0;
double yawTarget = 20, pitchTarget = 8, yawDps = 0, pitchDps = 0;
bool heartbeat = true;
void advance(uint32_t duration) {
    const uint32_t start = millis();
    while (millis() - start < duration) {
        if (heartbeat && celestialActive() && millis() - updateAt >= 1000) {
            const double seconds = (millis() - updateAt) / 1000.0;
            yawTarget += yawDps * seconds; pitchTarget += pitchDps * seconds;
            updateCelestial(71, ++sequence, yawTarget, pitchTarget);
            updateAt = millis();
        }
        loop();
    }
}
void begin() {
    // Fixture represents previously measured, stopped encoder/step geometry.
    // Separate primary-feedback tests exercise learning and missing scales.
    celestialLearnedEncoderScale[0] = (360.0/4096) / (0.18 * YAW_TRACK_PULSES_PER_DEG);
    celestialLearnedEncoderScale[1] = (360.0/4096) / (0.225 * PITCH_TRACK_PULSES_PER_DEG);
    yawTarget = orientation.heading; pitchTarget = physicalPitch();
    updateAt = millis(); sequence = 0;
    beginCelestial(71, yawTarget, pitchTarget);
    advance(1300);
    CHECK(celestialActive() && celestialTracking && !poseStopping);
}
void rate(double hz) {
    yawDps = hz / YAW_TRACK_PULSES_PER_DEG;
    pitchDps = hz / PITCH_TRACK_PULSES_PER_DEG;
    // Set the already-filtered trajectory rate to isolate the transition from
    // the unrelated two-second target velocity filter. Real updates continue.
    celestialTargetRate[0] = yawDps; celestialTargetRate[1] = pitchDps;
    celestialRateKnown = true;
    for (auto &axis : celestialTrackAxes) axis.filteredError = axis.correction = 0;
}
void active() {
    CHECK(celestialActive() && celestialTracking && !poseStopping && !finalPrinted);
}
void stop() {
    simulated::serialInput += "STOP\n";
    advance(3300);
    CHECK(commandIdle() && poseMotorsStopped());
}
}
int main(int argc, char **argv) {
    CHECK(argc == 2);
    const std::string scenario = argv[1];
    simulated::physicalPitchUsesRoll = false;
    simulated::baselineYaw = yawTarget; simulated::baselinePitch = pitchTarget;
    simulated::encoderPlantFeedback = true;
    simulated::latchIdleForceStop = true; simulated::forceStopDrainMs = 20;
    Wire.encoder.present = Wire1.encoder.present = true;
    setup(); advance(2200);
    for (unsigned i = 0; i < 2; ++i) {
        simulated::motors[i].stepPauses = true;
        simulated::motors[i].degreesPerStep = 1.0 /
            (i == 0 ? YAW_TRACK_PULSES_PER_DEG : PITCH_TRACK_PULSES_PER_DEG);
    }
    begin();
    if (scenario == "startup_latch" || scenario == "startup_fails") {
        // Exact FAS failure: a forceStop flag left after the ramp is idle.
        yawMotor->forceStop(); pitchMotor->forceStop();
        if (scenario == "startup_fails") simulated::motors[0].failContinuousStart = true;
    }
    if (scenario == "startup_timeout") simulated::motors[0].queueNeverStarts = true;
    rate(scenario == "pending_period" || scenario == "minimum_rate" ? 0.005 : 0.020);
    advance(500);
    if (scenario == "startup_fails") {
        CHECK(poseStopping || commandIdle());
        CHECK(Serial.output.find("TRACK motor failed to start") != std::string::npos);
        CHECK(simulated::commands.size() <= 4); // Never retry indefinitely.
        return 0;
    }
    if (scenario == "startup_timeout") {
        advance(3000);
        CHECK(poseStopping || commandIdle());
        CHECK(Serial.output.find("TRACK motor failed to start") != std::string::npos);
        return 0;
    }
    active();
    CHECK(celestialTrackAxes[0].runConfirmed && celestialTrackAxes[1].runConfirmed);
    CHECK(yawMotor->isQueueRunning() && pitchMotor->isQueueRunning());
    CHECK(yawMotor->getCurrentPosition() == 0 && pitchMotor->getCurrentPosition() == 0);
    const size_t starts = simulated::commands.size();
    if (scenario == "stable" || scenario == "minimum_rate" || scenario == "startup_latch") {
        advance(scenario == "minimum_rate" ? 210000 : 120000); active();
        CHECK(simulated::commands.size() == starts);
        CHECK(yawMotor->getCurrentPosition() < 0 && pitchMotor->getCurrentPosition() < 0);
        if (scenario == "startup_latch") CHECK(starts == 4);
    } else if (scenario == "unexpected_stop") {
        simulated::motors[0].drive = simulated::Drive::IDLE;
        advance(200);
        CHECK(poseStopping || commandIdle());
        CHECK(Serial.output.find("TRACK motor stopped unexpectedly") != std::string::npos);
        CHECK(simulated::commands.size() == starts); // Confirmed motion is never retried.
        return 0;
    } else if (scenario == "forced_restart" || scenario == "takeover" || scenario == "lease") {
        if (scenario == "lease") { heartbeat = false; advance(6500); }
        else stop();
        CHECK(commandIdle() && poseMotorsStopped());
        CHECK(simulated::forceStops == 2); // One request per axis, despite queue draining.
        CHECK(!simulated::motors[0].idleForceStopLatched && !simulated::motors[1].idleForceStopLatched);
        if (scenario == "lease") {
            CHECK(Serial.output.find("target update lease expired after 3000 ms") != std::string::npos);
            return 0;
        }
        if (scenario == "takeover") {
            simulated::serialInput += "JOG 0 0 0\n"; advance(20);
            CHECK(manualActive);
            simulated::serialInput += "JOG 100 100 0\n"; advance(20);
            CHECK(yawMotor->isRunning() && pitchMotor->isRunning());
            return 0;
        }
        yawDps = pitchDps = 0; begin(); rate(0.020); advance(1000); active();
        CHECK(celestialTrackAxes[0].runConfirmed && celestialTrackAxes[1].runConfirmed);
        CHECK(!celestialTrackAxes[0].startRetried && !celestialTrackAxes[1].startRetried);
        advance(60000); active();
    } else {
        if (scenario == "pending_period") {
            rate(0.2); advance(200);
            CHECK(celestialTrackAxes[0].rateMilliHz == 200);
            CHECK(yawMotor->getPeriodInUsAfterCommandsCompleted() == 200000000);
        }
        if (scenario == "brake_stuck") simulated::motors[0].neverStops = true;
        if (scenario == "sensor_pause") {
            Wire.encoder.requestLength = 2;
            advance(4000); active();
            CHECK(celestialEncoderPaused && yawAxis.motion == Motion::BRAKING);
            Wire.encoder.requestLength = 3;
            advance(100000); active(); CHECK(!celestialEncoderPaused);
        } else if (scenario == "zero" || scenario == "below_minimum") {
            rate(scenario == "below_minimum" ? 0.004 : 0);
            advance(4000); active();
            CHECK(yawAxis.motion == Motion::TRACK_ZERO && pitchAxis.motion == Motion::TRACK_ZERO);
            CHECK(!yawMotor->isRunning() && !pitchMotor->isRunning());
            CHECK(simulated::commands.size() == starts);
            advance(110000); active();
        } else {
            rate(scenario == "pending_period" ? -0.2 : -0.020);
            advance(4000); active();
            CHECK(yawAxis.motion == Motion::BRAKING && pitchAxis.motion == Motion::BRAKING);
            CHECK(simulated::commands.size() == starts);
            const uint32_t timeout = celestialTrackAxes[0].brakeTimeoutMs;
            CHECK(timeout >= (scenario == "pending_period" ? 403000U : 103000U));
            advance(timeout);
            if (scenario == "brake_stuck") {
                CHECK(!celestialActive() || poseStopping);
                CHECK(Serial.output.find("TRACK braking timed out") != std::string::npos);
                return 0;
            }
            active();
            CHECK(millis() - celestialTrackAxes[0].progressAt < CELESTIAL_TRACK_PROGRESS_MS);
            if (scenario == "reverse" || scenario == "pending_period") {
                CHECK(yawAxis.motion == Motion::TRACK && pitchAxis.motion == Motion::TRACK);
                CHECK(yawAxis.slewDirection == 1 && pitchAxis.slewDirection == 1);
                CHECK(simulated::commands.size() == starts + 2);
                for (const auto &cmd : simulated::commands) CHECK(!cmd.wasBraking && !cmd.beforeStoppedSample);
            } else {
                CHECK(!yawMotor->isRunning() && !pitchMotor->isRunning());
                CHECK(simulated::commands.size() == starts);
            }
        }
    }
    stop();
    std::printf("PASS low-rate: %s\n", scenario.c_str());
    return 0;
}
