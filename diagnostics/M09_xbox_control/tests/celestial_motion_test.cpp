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
    simulated::baselineYaw = scenario == "wrap" ? 359.6 : 20;
    simulated::baselinePitch = 8;
    if (scenario == "missing") simulated::bnoInitFails = true;
    if (scenario == "low_admission") simulated::accuracy = 1;
    setup(); advance(scenario == "goto_unqualified" ? 50 : 2200);
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
    } else if (scenario == "yaw_guard") {
        heading.continuous = northTargetContinuous + 184.5;
        orientation.heading = wrap360(heading.first + heading.continuous);
        beginCelestial(71, wrap360(orientation.heading + 1), 20);
        CHECK(!poseActive && commandIdle());
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
        begin();
        if (startsAtTarget || bnoOutageScenario) track();
        if (bnoOutageScenario)
            CHECK(celestialEncoderAxes[0].scaleKnown && celestialEncoderAxes[1].scaleKnown);
        if (scenario == "goto" || scenario == "goto_unqualified" || scenario == "high_reduction") {
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
            for (size_t i = startCommands; i < simulated::commands.size(); ++i) CHECK(!simulated::commands[i].continuous);
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
            advance(500); preserved(); CHECK(celestialEncoderMode && celestialEncoderPaused);
            advance(11000); preserved();
            command("STOP"); recover();
        } else if (scenario == "track_stale") {
            const auto yawSteps = yawMotor->getCurrentPosition(), pitchSteps = pitchMotor->getCurrentPosition();
            const auto originalSequence = sequence;
            simulated::bnoPauseStart = millis(); simulated::bnoPauseEnd = millis() + 6000;
            for (unsigned n=0; n<8; ++n) { targetYaw += 0.08; targetPitch += 0.06; advance(1100); preserved(); }
            CHECK(celestialTracking && celestialEncoderMode && sequence > originalSequence);
            CHECK(yawMotor->getCurrentPosition() != yawSteps && pitchMotor->getCurrentPosition() != pitchSteps);
            CHECK(Serial.output.find("status=DEGRADED") != std::string::npos);
            CHECK(Serial.output.find("status=RECOVERED") != std::string::npos);
            command("STOP"); recover();
        } else if (scenario == "accuracy") {
            simulated::accuracy = 1; advance(2000); preserved(); CHECK(celestialEncoderMode);
            simulated::accuracy = 3; advance(500); preserved();
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
            simulated::resetDuringPoll = true; advance(500); preserved(); CHECK(celestialEncoderMode);
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
            advance(200); preserved();
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
            heartbeat = false;
            const uint32_t waitStarted = millis();
            while ((yawMotor->isRunning() || pitchMotor->isRunning()) && millis()-waitStarted < 3000) advance(100);
            CHECK(!yawMotor->isRunning() && !pitchMotor->isRunning());
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
