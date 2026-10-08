// Both host entry paths send the same GOTO/UPDATE protocol. A HERE target is
// initially satisfied; RA/Dec normally needs acquisition before this engine.
#define main celestial_fixture_main
#include "celestial_motion_test.cpp"
#undef main
int main(int argc, char **argv) {
    CHECK(argc == 2);
    const std::string scenario = argv[1];
    simulated::physicalPitchUsesRoll = false;
    simulated::baselineYaw = 20; simulated::baselinePitch = 8;
    simulated::encoderPlantFeedback = true; simulated::independentTick = tick;
    Wire.encoder.present = Wire1.encoder.present = true;
    setup(); advance(2200);
    simulated::motors[0].degreesPerStep = 1.0 / YAW_TRACK_PULSES_PER_DEG;
    simulated::motors[1].degreesPerStep = 1.0 / PITCH_TRACK_PULSES_PER_DEG;
    if (scenario == "learn_manual" || scenario == "learn_gap") {
        if (scenario == "learn_gap") {
            command("JOG 0 0 0"); command("JOG 300 -300 0");
            Wire.encoder.requestLength = Wire1.encoder.requestLength = 2;
            advance(100); command("STOP"); advance(1000);
            Wire.encoder.requestLength = Wire1.encoder.requestLength = 3;
            advance(500);
            CHECK(celestialLearnedEncoderScale[0] == 0 && celestialLearnedEncoderScale[1] == 0);
        }
        command("JOG 0 0 0");
        for (unsigned n=0; n<15; ++n) { command("JOG 300 -300 0"); advance(50); }
        command("STOP"); advance(1000);
        CHECK(celestialLearnedEncoderScale[0] > 0 && celestialLearnedEncoderScale[1] > 0);
    } else if (scenario != "unscaled") {
        celestialLearnedEncoderScale[0] = (360.0/4096) / (0.18 * YAW_TRACK_PULSES_PER_DEG);
        celestialLearnedEncoderScale[1] = (360.0/4096) / (0.225 * PITCH_TRACK_PULSES_PER_DEG);
    }
    const bool acquisition = scenario.find("radec") == 0;
    targetYaw = acquisition ? 45 : orientation.heading;
    targetPitch = acquisition ? 20 : physicalPitch();
    begin(); track(); CHECK(celestialEncoderMode);
    if (scenario == "unscaled") {
        advance(6000); preserved();
        CHECK(celestialEncoderPaused && poseMotorsStopped());
        CHECK(Serial.output.find("encoder reference/scale unavailable") != std::string::npos);
        command("STOP"); recover();
        std::puts("PASS AS5600 primary: unscaled HOLD"); return 0;
    }
    CHECK(celestialControlFeedbackReady());
    const double yawReference = celestialEncoderAxes[0].referenceAngle;
    const double pitchReference = celestialEncoderAxes[1].referenceAngle;
    const double yawScale = celestialEncoderAxes[0].degreesPerTick;
    for (unsigned second=0; second<120; ++second) {
        targetYaw += .003; targetPitch -= .0024;
        // These are BNO-only disturbances. The AS5600 fixture measures physical
        // shaft response independently, including stalls and wrong direction.
        simulated::yawDisturbance = second % 2 ? 25 : -20;
        simulated::pitchDisturbance = second % 2 ? 30 : -25;
        if (scenario.find("degraded") != std::string::npos && second >= 30 && second < 60)
            simulated::wrongReportType = true;
        else simulated::wrongReportType = false;
        advance(1000); preserved();
        CHECK(celestialEncoderMode && !celestialEncoderPaused);
        CHECK(celestialEncoderAxes[0].referenceAngle == yawReference);
        CHECK(celestialEncoderAxes[1].referenceAngle == pitchReference);
        CHECK(celestialEncoderAxes[0].degreesPerTick == yawScale);
        CHECK(fabs(yawAxis.error) < .1 && fabs(pitchAxis.error) < .1);
        if (second > 20) {
            CHECK(yawMotor->isRunning() && pitchMotor->isRunning());
            CHECK(fabs(celestialTrackAxes[0].rateMilliHz - 156) <= 1);
            CHECK(fabs(celestialTrackAxes[1].rateMilliHz - 161.28) <= 1);
        }
    }
    if (scenario == "invalid_encoder") {
        Wire1.encoder.requestLength = 2; advance(1000); preserved();
        CHECK(celestialEncoderPaused && poseMotorsStopped());
        Wire1.encoder.requestLength = 3; advance(1000); preserved();
        CHECK(!celestialEncoderPaused && celestialEncoderMode);
        Wire.encoder.status = 0; advance(1000); preserved();
        CHECK(celestialEncoderPaused && poseMotorsStopped());
        Wire.encoder.status = 0x20; advance(1000); preserved();
        CHECK(!celestialEncoderPaused);
    }
    command("STOP"); recover();
    std::printf("PASS AS5600 primary: %s\n", scenario.c_str());
}
