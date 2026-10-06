// Real dispatcher/POSE controller and synthetic motors/BNO; no hardware I/O.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>
namespace {
void require(bool ok, const char *expression, int line) {
    if (!ok) {
        std::fprintf(stderr, "FAIL celestial line %d: %s\n%s", line, expression, Serial.output.c_str());
        std::exit(1);
    }
}
#define CHECK(x) require((x), #x, __LINE__)
uint32_t sequence = 0, updateAt = 0;
double targetYaw = 45, targetPitch = 20;
bool heartbeat = false;
void tick(uint32_t now) { motionWatchdog.check(now); commandWatchdog.check(now); }
void send(const std::string &line) { simulated::serialInput += line + "\n"; }
void update() {
    char line[160];
    snprintf(line, sizeof(line), "CELESTIAL_UPDATE 71 %lu %.6f %.6f",
        static_cast<unsigned long>(++sequence), wrap360(targetYaw), targetPitch);
    send(line); updateAt = millis();
}
void advance(uint32_t ms) {
    const uint32_t start = millis();
    while (millis() - start < ms) {
        if (heartbeat && celestialActive() && millis() - updateAt >= 1000) update();
        loop();
    }
}
void command(const std::string &line) { send(line); advance(25); }
void begin() {
    char line[120];
    snprintf(line, sizeof(line), "CELESTIAL_GOTO 71 %.6f %.6f", wrap360(targetYaw), targetPitch);
    updateAt = millis(); heartbeat = true; command(line);
    CHECK(celestialActive() && celestialId == 71 && !manualActive && !keyframeActive);
    CHECK(fabs(shortestDifference(wrap360(heading.first + yawAxis.target), targetYaw)) < 0.001);
    CHECK(fabs(pitchAxis.target - targetPitch) < 0.001);
}
void track() {
    const uint32_t start = millis();
    while (celestialActive() && !celestialTracking && millis() - start < 180000) advance(100);
    CHECK(celestialTracking && celestialActive() && !finalPrinted);
    CHECK(fabs(yawAxis.error) <= TOLERANCE_DEG && fabs(pitchAxis.error) <= TOLERANCE_DEG);
}
void preserved() {
    CHECK(celestialActive() && celestialId == 71 && !poseStopping && !finalPrinted);
    CHECK(fabs(shortestDifference(wrap360(heading.first + yawAxis.target), wrap360(targetYaw))) < 0.002);
    CHECK(fabs(pitchAxis.target - targetPitch) < 0.002);
    CHECK(Serial.output.find("CELESTIAL_RESULT id=71 status=FAILED") == std::string::npos);
}
void recover() {
    heartbeat = false; advance(4000);
    CHECK(!poseActive && !poseStopping && commandIdle() && manualReady());
    CHECK(Serial.output.find("CELESTIAL_RESULT id=71") < Serial.output.rfind("M09 READY"));
    command("JOG 0 0 0"); CHECK(manualActive);
    command("JOG 200 -200 100");
    CHECK(yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
    command("STOP"); advance(2000); CHECK(commandIdle());
}
}
int main(int argc, char **argv) {
    if (argc != 2) return 1;
    const std::string scenario = argv[1];
    simulated::physicalPitchUsesRoll = false; simulated::independentTick = tick;
    simulated::encoderPlantFeedback = true;
    Wire.encoder.present = Wire1.encoder.present = true;
    simulated::baselineYaw = scenario == "wrap" ? 359.6 : (scenario == "track_rate_wrap" ? 359.99 : 20);
    simulated::baselinePitch = 8;
    if (scenario == "missing") simulated::bnoInitFails = true;
    if (scenario == "low_admission") simulated::accuracy = 1;
    setup();
    simulated::motors[0].degreesPerStep = 1.0 / YAW_TRACK_PULSES_PER_DEG;
    simulated::motors[1].degreesPerStep = 1.0 / PITCH_TRACK_PULSES_PER_DEG;
    advance(scenario == "goto_unqualified" ? 50 : 2200);
    CHECK(commandIdle() && manualReady() && simulated::commands.empty());
    if (scenario == "missing" || scenario == "low_admission") {
        command("CELESTIAL_GOTO 71 45 20");
        CHECK(!poseActive && commandIdle() && simulated::commands.empty());
        CHECK(Serial.output.find("CELESTIAL_REJECTED id=71") != std::string::npos);
        command("JOG 0 0 0"); command("JOG 200 200 0"); CHECK(manualActive);
    } else if (scenario == "invalid") {
        for (const char *bad : {"CELESTIAL_GOTO 0 45 20", "CELESTIAL_GOTO 71 nan 20",
            "CELESTIAL_GOTO 71 45 inf", "CELESTIAL_GOTO 71 45 75", "CELESTIAL_GOTO 71 45 -75",
            "CELESTIAL_GOTO 71 45 20 junk", "CELESTIAL_GOTO 4294967296 45 20"}) command(bad);
        CHECK(commandIdle() && !poseActive && simulated::commands.empty());
    } else if (scenario == "busy") {
        command("JOG 0 0 0"); command("JOG 200 200 0");
        command("CELESTIAL_GOTO 71 45 20"); CHECK(manualActive && !poseActive);
    } else if (scenario == "goto_sun" || scenario == "goto_yaw_outside") {
        // Both shortest-path endpoints exceed the former +185 envelope;
        // the second case also starts outside it.
        simulated::yawDisturbance = scenario == "goto_sun" ? 140 : 170;
        advance(100);
        const double reference = northTargetContinuous;
        const double initialYaw = heading.continuous;
        targetYaw = 217.3; targetPitch = 33.6;
        begin();
        CHECK(fabs(yawAxis.target - initialYaw - (scenario == "goto_sun" ? 57.3 : 27.3)) < 0.001);
        CHECK(yawAxis.target - reference > 185);
        track(); preserved();
        CHECK(heading.continuous - reference > 185);
        command("STOP"); recover();
    } else if (scenario == "track_yaw_positive_unbounded" || scenario == "track_yaw_negative_unbounded") {
        const int sign = scenario == "track_yaw_positive_unbounded" ? 1 : -1;
        simulated::yawDisturbance = sign * 100; advance(100);
        simulated::yawDisturbance = sign * 184.99 - simulated::baselineYaw; advance(100);
        const double reference = northTargetContinuous;
        targetYaw = sign * 184.99; targetPitch = 8;
        begin(); track();
        targetYaw += sign * 0.009; advance(1000); preserved();
        heartbeat = false;
        advance(2000); preserved(); // Extrapolated target crosses +/-185 before the next update.
        CHECK(fabs(yawAxis.target + celestialTargetRate[0] * ((millis()-celestialUpdatedAt)/1000.0) - reference) > 185);
        targetYaw += sign * 0.15; update(); advance(25); preserved();
        CHECK(fabs(yawAxis.target-reference)>185);
        heartbeat = true;
        for (unsigned second=0; second<10; ++second) {
            targetYaw += sign * 0.1; advance(1000); preserved();
        }
        CHECK(fabs(yawAxis.current-reference)>185);
        CHECK(yawAxis.motion==Motion::TRACK && yawAxis.slewDirection==-sign);
        command("STOP"); recover();
    } else {
        const bool bnoOutageScenario = scenario == "track_stale" || scenario == "track_accuracy" ||
            scenario == "track_reset" || scenario == "bad_feedback" || scenario == "track_unavailable" ||
            scenario == "track_intermittent" || scenario == "track_encoder_feedback" ||
            scenario == "track_long_outage" || scenario == "track_recovery" || scenario == "track_manual" ||
            scenario == "blocked_foreground";
        const bool startsAtTarget = (scenario.find("track_") == 0 && !bnoOutageScenario) || scenario == "tracking";
        if (startsAtTarget) { targetYaw = simulated::baselineYaw; targetPitch = 8; }
        if (scenario == "wrap") { targetYaw = 0.2; targetPitch = 8; }
        if (scenario == "high_reduction") {
            simulated::motors[0].degreesPerStep = 0.003;
            simulated::motors[1].degreesPerStep = 0.001;
        }
        const bool rateScenario = scenario.find("track_rate_") == 0;
        if (rateScenario) {
            CHECK(yawPulsesPerDegree == 0 && pitchPulsesPerDegree == 0);
            if (scenario == "track_rate_weak") {
                yawPulsesPerDegree = 269.442; pitchPulsesPerDegree = 500;
            }
        }
        begin();
        if (startsAtTarget || bnoOutageScenario) track();
        if (bnoOutageScenario)
            CHECK(celestialEncoderAxes[0].scaleKnown && celestialEncoderAxes[1].scaleKnown);
        if (scenario.find("track_sign_") == 0) {
            // Independent powered convention: positive STEP lowers BNO pitch.
            // Do not derive the expected motor direction from production helpers.
            simulated::motors[1].physicalSign = -1;
            const bool residualOnly = scenario.find("residual") != std::string::npos;
            const int angularSign = scenario.find("positive") != std::string::npos ? 1 : -1;
            const double initialPitch = simulated::actualPitch();
            const double initialYaw = simulated::actualYaw();
            const size_t commandsBefore = simulated::commands.size();
            const size_t outputBefore = Serial.output.size();
            if (residualOnly) simulated::pitchDisturbance = -angularSign * 0.3;
            for (unsigned second = 0; second < 30; ++second) {
                targetYaw += 0.003;
                if (!residualOnly) targetPitch += angularSign * 0.003;
                advance(1000); preserved();
            }
            CHECK(yawAxis.motion == Motion::TRACK && yawAxis.slewDirection == -1);
            CHECK(yawMotor->getCurrentSpeedInMilliHz() < 0);
            CHECK(simulated::actualYaw() > initialYaw);
            CHECK(pitchAxis.motion == Motion::TRACK);
            CHECK(pitchAxis.slewDirection == -angularSign);
            CHECK(pitchMotor->getCurrentSpeedInMilliHz() * angularSign < 0);
            const double physicalTravel = simulated::actualPitch() - simulated::pitchDisturbance - initialPitch;
            CHECK(physicalTravel * angularSign > 0.02);
            CHECK(simulated::commands.size() == commandsBefore + 2);
            for (size_t n = commandsBefore; n < simulated::commands.size(); ++n) {
                const auto &move = simulated::commands[n];
                if (move.stepPin == pitchMotor->stepPin) {
                    CHECK(move.steps == -angularSign && move.continuous);
                }
            }
            if (residualOnly) {
                CHECK(fabs(celestialTargetRate[1]) < 0.000001);
                CHECK(celestialTrackAxes[1].correction * angularSign > 0);
                CHECK(fabs(celestialTrackAxes[1].correction) <= CELESTIAL_RESIDUAL_MAX_DPS);
            } else {
                CHECK(fabs(celestialTargetRate[1] - angularSign * 0.003) < 0.000001);
                CHECK(celestialTrackAxes[1].correction == 0);
                CHECK(fabs(celestialTrackAxes[1].rateMilliHz - 201.6) <= 1);
                CHECK(Serial.output.find(angularSign > 0 ? "nominal_hz=-0.20" :
                    "nominal_hz=0.20", outputBefore) != std::string::npos);
            }
            command("STOP"); recover();
        } else if (rateScenario) {
            CHECK(celestialTrackAxes[0].conversion == YAW_TRACK_PULSES_PER_DEG);
            CHECK(celestialTrackAxes[1].conversion == PITCH_TRACK_PULSES_PER_DEG);
            const size_t before = simulated::commands.size();
            const double yawStart = simulated::actualYaw(), pitchStart = simulated::actualPitch();
            const uint32_t stopCount = simulated::gentleStops;
            for (unsigned second = 0; second < 180; ++second) {
                targetYaw += 0.003; targetPitch -= 0.0024;
                if (scenario == "track_rate_noise") {
                    simulated::yawDisturbance = second % 2 ? 0.35 : -0.35;
                    simulated::pitchDisturbance = second % 2 ? -0.2 : 0.2;
                }
                advance(1000); preserved();
                if (second > 3) {
                    CHECK(yawAxis.motion == Motion::TRACK && pitchAxis.motion == Motion::TRACK);
                    CHECK(yawMotor->isRunning() && pitchMotor->isRunning());
                }
            }
            CHECK(simulated::commands.size() == before + 2); // two starts, no catch-up bursts
            CHECK(simulated::gentleStops == stopCount);
            CHECK(fabs(celestialTargetRate[0] - 0.003) < 0.00001);
            CHECK(fabs(celestialTargetRate[1] + 0.0024) < 0.00001);
            CHECK(Serial.output.find("ACQUIRING_RESPONSE") == std::string::npos);
            CHECK(Serial.output.find("source=CONFIGURED") != std::string::npos);
            CHECK(Serial.output.find("nominal_hz=") != std::string::npos);
            CHECK(celestialTrackAxes[0].conversion == YAW_TRACK_PULSES_PER_DEG);
            if (scenario != "track_rate_noise") {
                CHECK(fabs(simulated::actualYaw() - yawStart - 0.54) < 0.025);
                CHECK(fabs(simulated::actualPitch() - pitchStart + 0.432) < 0.025);
                CHECK(fabs(celestialTrackAxes[0].rateMilliHz - 156.0) <= 1);
                CHECK(fabs(celestialTrackAxes[1].rateMilliHz - 161.28) <= 1);
                CHECK(Serial.output.find("commanded_hz=-0.156000") != std::string::npos);
            }
            if (scenario == "track_rate_residual") {
                simulated::yawDisturbance = -0.3;
                for (unsigned second = 0; second < 15; ++second) {
                    targetYaw += 0.003; targetPitch -= 0.0024;
                    advance(1000); preserved();
                }
                CHECK(celestialTrackAxes[0].correction > 0);
                CHECK(celestialTrackAxes[0].correction <= CELESTIAL_RESIDUAL_MAX_DPS);
                CHECK(celestialTrackAxes[0].rateMilliHz > 0.003 * YAW_TRACK_PULSES_PER_DEG * 1000);
                CHECK(simulated::commands.size() == before + 2);
                command("STOP"); recover();
            } else if (scenario == "track_rate_jitter") {
                heartbeat = false;
                for (unsigned n = 0; n < 30; ++n) {
                    const uint32_t acceptedAt = celestialUpdatedAt;
                    advance(n % 2 ? 700 : 1300);
                    const double elapsed = (millis() - acceptedAt) / 1000.0;
                    targetYaw += 0.003 * elapsed; targetPitch -= 0.0024 * elapsed;
                    update(); advance(5); preserved();
                    CHECK(fabs(celestialTargetRate[0] - 0.003) < 0.00002);
                    CHECK(fabs(celestialTargetRate[1] + 0.0024) < 0.00002);
                }
                CHECK(simulated::commands.size() == before + 2);
                command("STOP"); recover();
            } else if (scenario == "track_rate_lease") {
                heartbeat = false; advance(3200);
                CHECK(poseStopping || commandIdle());
                CHECK(Serial.output.find("target update lease expired") != std::string::npos);
                recover();
            } else if (scenario == "track_rate_unscaled_recovery") {
                celestialEncoderAxes[1].scaleKnown = false;
                // Do not let a new BNO sample relearn it before the outage.
                simulated::wrongReportType = true; advance(500);
                CHECK(!celestialEncoderMode && celestialEncoderPaused && poseMotorsStopped());
                simulated::wrongReportType = false; advance(500);
                preserved(); CHECK(!celestialEncoderMode && !celestialEncoderPaused);
                CHECK(yawAxis.motion == Motion::TRACK && pitchAxis.motion == Motion::TRACK);
                command("STOP"); recover();
            } else if (scenario == "track_rate_fallback") {
                CHECK(celestialEncoderFeedbackAvailable(millis()));
                simulated::wrongReportType = true; advance(500);
                preserved(); CHECK(celestialEncoderMode && !celestialEncoderPaused);
                CHECK(yawMotor->isRunning() && pitchMotor->isRunning());
                command("STOP"); recover();
            } else if (scenario == "track_rate_reverse") {
                for (unsigned second = 0; second < 30; ++second) {
                    targetYaw -= 0.003; targetPitch += 0.0024;
                    advance(1000); preserved();
                }
                CHECK(yawAxis.slewDirection == bnoSlewStepDirection(-1, false));
                CHECK(pitchAxis.slewDirection == bnoSlewStepDirection(1, true));
                for (size_t n = before; n < simulated::commands.size(); ++n)
                    CHECK(!simulated::commands[n].wasBraking);
                command("STOP"); recover();
            } else if (scenario == "track_rate_stall") {
                simulated::motors[0].frozen = true;
                for (unsigned second = 0; second < 240 && celestialActive() && !poseStopping; ++second) {
                    targetYaw += 0.01; targetPitch -= 0.0024;
                    advance(1000);
                }
                CHECK(poseStopping || commandIdle());
                CHECK(Serial.output.find("TRACK filtered position error made no progress") != std::string::npos);
                simulated::motors[0].frozen = false; recover();
            } else if (scenario == "track_rate_rejected") {
                simulated::speedFails = true;
                targetYaw += 0.1; advance(1500);
                CHECK(poseStopping || commandIdle());
                CHECK(Serial.output.find("TRACK continuous rate command rejected") != std::string::npos);
                simulated::speedFails = false; recover();
            } else { command("STOP"); recover(); }
        } else if (scenario == "goto" || scenario == "goto_unqualified" || scenario == "high_reduction") {
            if (scenario == "goto_unqualified") CHECK(!referenceSet);
            CHECK(yawPulsesPerDegree == 0 && pitchPulsesPerDegree == 0);
            command("STATUS"); CHECK(celestialActive());
            command("JOG 0 0 0"); CHECK(celestialActive() && !manualActive);
            track(); CHECK(concurrentMotion && yawAxis.settled && pitchAxis.settled);
            if (scenario == "high_reduction") {
                CHECK(celestialTimingKnown && celestialPulsesPerDegree[0] > 300 && celestialPulsesPerDegree[1] > 900);
                CHECK(celestialDeadlineMs > LEG_TIMEOUT_MS);
            }
            command("STOP"); recover();
        } else if (scenario == "tracking") {
            const size_t startCommands = simulated::commands.size();
            // Five minutes of ordinary sky-rate drift on both axes. This must
            // cross the normal POSE 90-second deadline without restarting GOTO.
            for (unsigned second = 0; second < 300; ++second) {
                targetYaw += 0.0042; targetPitch += 0.002;
                advance(1000); CHECK(celestialActive() && celestialTracking);
            }
            CHECK(yawMotor->getCurrentPosition() != 0 && pitchMotor->getCurrentPosition() != 0);
            CHECK(fabs(yawAxis.error) <= TOLERANCE_DEG + 0.05 && fabs(pitchAxis.error) <= TOLERANCE_DEG + 0.05);
            CHECK(simulated::commands.size() - startCommands < 80 && sequence <= 305);
            CHECK(celestialTrackAxes[0].conversion > 0 && celestialTrackAxes[1].conversion > 0);
            CHECK(yawAxis.motion == Motion::TRACK && pitchAxis.motion == Motion::TRACK);
            CHECK(Serial.output.find("status=ACQUIRING_RESPONSE") == std::string::npos);
            command("STOP"); recover();
        } else if (scenario == "wrap") {
            CHECK(fabs(yawAxis.target - heading.continuous) < 1);
            track(); targetYaw = 359.9; advance(1000);
            CHECK(celestialActive() && fabs(yawAxis.error) < 1);
            targetYaw = 0.1; advance(1000); CHECK(celestialActive() && fabs(yawAxis.error) < 1);
            command("STOP"); recover();
        } else if (scenario == "stop" || scenario == "track_stop") {
            advance(200); command("STOP"); CHECK(poseStopping || commandIdle()); recover();
            CHECK(Serial.output.find("CELESTIAL_RESULT id=71 status=STOPPED") != std::string::npos);
        } else if (scenario == "abort" || scenario == "track_abort") {
            advance(200); heartbeat = false; command("X");
            CHECK(phase == Phase::ABORTED && !poseActive && !manualReady());
            CHECK(poseMotorsStopped());
            CHECK(Serial.output.find("CELESTIAL_RESULT id=71 status=FAILED") != std::string::npos);
        } else if (scenario == "stale") {
            simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 10000;
            advance(500); preserved(); CHECK(!celestialEncoderMode && celestialEncoderPaused);
            advance(11000); preserved(); CHECK(!celestialEncoderMode && !celestialEncoderPaused);
            command("STOP"); recover();
        } else if (scenario == "track_stale") {
            const auto yawSteps = yawMotor->getCurrentPosition(), pitchSteps = pitchMotor->getCurrentPosition();
            const auto originalSequence = sequence;
            simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 6000;
            for (unsigned n=0; n<8; ++n) { targetYaw += 0.08; targetPitch += 0.06; advance(1100); preserved(); }
            CHECK(celestialTracking && celestialEncoderMode && celestialControlFeedbackReady() &&
                sequence > originalSequence);
            CHECK(yawMotor->getCurrentPosition() != yawSteps && pitchMotor->getCurrentPosition() != pitchSteps);
            CHECK(Serial.output.find("status=DEGRADED") != std::string::npos);
            CHECK(Serial.output.find("status=RECOVERED") != std::string::npos);
            command("STOP"); recover();
        } else if (scenario == "accuracy") {
            simulated::accuracy = 1; advance(2000); preserved();
            CHECK(!celestialEncoderMode && celestialEncoderPaused);
            simulated::accuracy = 3; advance(500); preserved();
            CHECK(!celestialEncoderMode && !celestialEncoderPaused);
            command("STOP"); recover();
        } else if (scenario == "track_accuracy") {
            const auto originalSequence = sequence;
            simulated::accuracy = 1;
            for (unsigned n=0; n<6; ++n) { targetYaw += 0.08; targetPitch += 0.06; advance(1100); preserved(); }
            CHECK(celestialTracking && celestialEncoderMode && sequence > originalSequence);
            simulated::accuracy = 3; advance(500); preserved();
            CHECK(Serial.output.find("status=RECOVERED") != std::string::npos);
            command("STOP"); recover();
        } else if (scenario == "reset") {
            simulated::resetDuringPoll = true; advance(500); preserved();
            CHECK(!celestialEncoderMode && !celestialEncoderPaused);
            command("STOP"); recover();
        } else if (scenario == "track_reset") {
            const auto id = celestialId;
            simulated::resetDuringPoll = true; advance(1000); preserved();
            CHECK(celestialTracking && celestialEncoderMode && celestialId == id);
            CHECK(Serial.output.find("status=RECOVERED") != std::string::npos);
            command("STOP"); recover();
        } else if (scenario == "bad_feedback") {
            simulated::invalidQuaternion = true; advance(3000); preserved();
            CHECK(celestialTracking && celestialEncoderMode);
            simulated::invalidQuaternion = false; advance(500); preserved();
            CHECK(Serial.output.find("status=RECOVERED") != std::string::npos);
            command("STOP"); recover();
        } else if (scenario == "track_unavailable") {
            simulated::wrongReportType = true;
            const auto originalSequence = sequence;
            for (unsigned n=0; n<20; ++n) { targetYaw += 0.08; targetPitch += 0.06; advance(1100); preserved(); }
            CHECK(celestialTracking && celestialEncoderMode && sequence > originalSequence);
            CHECK(celestialControlFeedbackReady());
            command("STOP"); recover();
        } else if (scenario == "track_intermittent") {
            const auto id = celestialId;
            for (unsigned episode=0; episode<4; ++episode) {
                simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 500;
                targetYaw += 0.08; targetPitch += 0.06; advance(1100); preserved();
            }
            simulated::accuracy = 1; advance(500); preserved();
            simulated::accuracy = 3; simulated::invalidQuaternion = true; advance(500); preserved();
            simulated::invalidQuaternion = false; advance(500); preserved();
            CHECK(celestialId == id && celestialTracking && celestialEncoderMode);
            command("STOP"); recover();
        } else if (scenario == "track_encoder_feedback") {
            simulated::wrongReportType = true; advance(500); preserved();
            const double yawBefore = yawAxis.current, pitchBefore = pitchAxis.current;
            const size_t commandsBefore = simulated::commands.size();
            simulated::encoderBaselineDegrees[0] += 9.0;
            simulated::encoderBaselineDegrees[1] += 9.0;
            advance(3000); preserved();
            CHECK(fabs(yawAxis.current-yawBefore) > 0.5);
            CHECK(fabs(pitchAxis.current-pitchBefore) > 0.2);
            CHECK(simulated::commands.size() > commandsBefore);
            command("STOP"); recover();
        } else if (scenario == "track_long_outage") {
            simulated::wrongReportType = true;
            const auto id = celestialId;
            for (unsigned n=0; n<14; ++n) {
                simulated::now += 12UL * 60 * 60 * 1000;
                celestialUpdatedAt = millis() - 1000;
                updateCelestial(71, ++sequence, wrap360(targetYaw), targetPitch);
                updateAt = millis(); advance(50); preserved();
            }
            CHECK(celestialTracking && celestialEncoderMode && celestialId == id);
            CHECK(millis() - lastBnoGood > 6UL * 24 * 60 * 60 * 1000);
            command("STOP"); recover();
        } else if (scenario == "track_recovery") {
            simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 10000;
            for (unsigned n=0; n<5; ++n) { targetYaw += 0.08; targetPitch += 0.06; advance(1100); preserved(); }
            // Continuous TRACK no longer naturally stops between corrections.
            // Keep renewing the lease while checking report-only BNO recovery.
            const auto id = celestialId;
            const double yawBefore = yawAxis.current, pitchBefore = pitchAxis.current;
            simulated::yawDisturbance = 5; simulated::pitchDisturbance = -3;
            simulated::bnoPauseEnd = millis() + 100;
            advance(500); preserved();
            CHECK(celestialId == id && celestialTracking && celestialEncoderMode);
            CHECK(fabs(yawAxis.current-yawBefore) < 0.2 && fabs(pitchAxis.current-pitchBefore) < 0.2);
            CHECK(fabs(celestialRecoveryYawDiscrepancy-5.0) < 0.1);
            CHECK(fabs(celestialRecoveryPitchDiscrepancy+3.0) < 0.1);
            CHECK(Serial.output.find("action=REPORT_ONLY") != std::string::npos);
            command("STOP"); recover();
        } else if (scenario == "track_unscaled_recovery") {
            CHECK(!celestialEncoderAxes[0].scaleKnown || !celestialEncoderAxes[1].scaleKnown);
            simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 600;
            advance(300); preserved();
            CHECK(!celestialEncoderMode && celestialEncoderPaused && poseMotorsStopped());
            advance(600); preserved();
            CHECK(!celestialEncoderMode && !celestialEncoderPaused && celestialControlFeedbackReady());
            CHECK(Serial.output.find("status=RECOVERED action=RESUME feedback=BNO") != std::string::npos);
            const size_t outputBefore = Serial.output.size();
            const size_t commandsBefore = simulated::commands.size();
            targetYaw += 0.6;
            const uint32_t correctionDeadline = millis() + 3000;
            while (celestialActive() && simulated::commands.size() == commandsBefore &&
                    millis() < correctionDeadline) advance(50);
            preserved();
            CHECK(celestialTracking && !celestialEncoderMode);
            CHECK(simulated::commands.size() > commandsBefore);
            CHECK(Serial.output.find("AXIS YAW -> TRACK", outputBefore) != std::string::npos);
            CHECK(Serial.output.find("AXIS YAW -> PRECISION", outputBefore) == std::string::npos);
            command("STOP"); recover();
        } else if (scenario == "track_malformed") {
            const double beforeYaw = orientation.heading, beforePitch = physicalPitch();
            const auto beforeReceipt = lastBnoGood, beforeDisplay = lastPlausibleAt;
            m09::SensorSample sample, ignored;
            while (sensorWorker.samples.pop(ignored)) {}
            ++simulated::now;
            sample.epoch = sensorWorker.resetEpoch.load(); sample.receivedMs = millis();
            sample.event.sensorId = SH2_ROTATION_VECTOR; sample.event.status = 3;
            sample.event.un.rotationVector = {-0.0000610351562f,-0.0141601562f,
                -0.0000610351562f,-0.0000610351562f,-0.000244140625f};
            CHECK(sensorWorker.samples.push(sample)); serviceBno();
            CHECK(!bnoDiagnostics.accepted && alignmentSampleInvalid);
            CHECK(lastBnoGood == beforeReceipt && lastPlausibleAt == beforeDisplay);
            CHECK(orientation.heading == beforeYaw && physicalPitch() == beforePitch);
            CHECK(celestialEncoderPaused && !celestialEncoderMode && celestialActive());
            advance(500); preserved(); CHECK(!celestialEncoderPaused);
            command("STOP"); recover();
        } else if (scenario == "track_manual") {
            simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = UINT32_MAX;
            advance(1000); preserved(); CHECK(celestialEncoderMode);
            command("STOP"); advance(4000);
            CHECK(commandIdle() && manualReady() && !celestialActive());
            command("JOG 0 0 0"); CHECK(manualActive);
            command("JOG 200 -200 100");
            CHECK(yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
            command("STOP"); advance(2000); CHECK(commandIdle());
        } else if (scenario == "lease" || scenario == "track_lease") {
            heartbeat = false;
            for (unsigned n = 0; n < 32; ++n) { command("STATUS"); advance(75); }
            CHECK(poseStopping || commandIdle()); recover();
            CHECK(Serial.output.find("target update lease expired") != std::string::npos);
        } else if (scenario == "stale_update") {
            advance(1100); const auto accepted = celestialUpdatedAt;
            command("CELESTIAL_UPDATE 72 80 45 20");
            command("CELESTIAL_UPDATE 71 1 45 20");
            command("CELESTIAL_UPDATE 71 2 nan 20");
            CHECK(celestialUpdatedAt == accepted && celestialActive());
            command("STOP"); recover();
            command("CELESTIAL_UPDATE 71 99 45 20"); CHECK(!poseActive && commandIdle());
        } else if (scenario == "target_jump") {
            advance(500); command("CELESTIAL_UPDATE 71 1 100 20");
            CHECK(poseStopping || commandIdle()); recover();
        } else if (scenario == "updated_guard") {
            heartbeat = false; targetPitch = 74.8;
            pitchAxis.target = targetPitch;
            advance(1000); command("CELESTIAL_UPDATE 71 1 45 75.1");
            CHECK(poseStopping || commandIdle()); recover();
        } else if (scenario == "no_progress") {
            simulated::motors[0].frozen = true; advance(20000);
            CHECK(!poseActive && commandIdle()); recover();
        } else if (scenario == "wrong_direction") {
            simulated::motors[0].physicalSign *= -1; advance(4000);
            CHECK(!poseActive && commandIdle()); recover();
        } else if (scenario == "deadline") {
            controlStartedAt = millis() - celestialDeadlineMs; advance(20);
            CHECK(poseStopping || commandIdle()); recover();
        } else if (scenario == "blocked_foreground") {
            // BNO acquisition can disappear while foreground/encoder control continues.
            simulated::blockBnoMs = 1000; advance(2000); preserved();
            CHECK(celestialTracking && celestialEncoderMode && !motionWatchdog.tripped());
            command("STOP"); recover();
        } else { std::fprintf(stderr, "Unknown celestial scenario\n"); return 2; }
    }
    std::printf("PASS celestial: %s\n", scenario.c_str());
}
