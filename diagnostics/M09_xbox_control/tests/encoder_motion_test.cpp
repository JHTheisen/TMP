// Exercise real firmware commands against encoder registers and an independent
// motor plant. These are software decisions, not hardware accuracy measurements.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>
namespace {
void check(bool ok, const char *expression, int line) {
    if (!ok) { std::fprintf(stderr, "FAIL encoder motion %d: %s (yaw=%f pitch=%f)\n%s", line, expression, orientation.heading, physicalPitch(), Serial.output.c_str()); std::exit(1); }
}
#define CHECK(x) check((x), #x, __LINE__)
void tick(uint32_t now) { motionWatchdog.check(now); commandWatchdog.check(now); }
uint32_t updated = 0, sequence = 0;
double targetYaw = 0, targetPitch = 0, yawRate = 0, pitchRate = 0;
bool heartbeat = false;
void advance(uint32_t duration) {
    const uint32_t start = millis();
    while (millis() - start < duration) {
        if (heartbeat && celestialActive() && millis() - updated >= 1000) {
            targetYaw += yawRate; targetPitch += pitchRate;
            updateCelestial(7, ++sequence, targetYaw, targetPitch); updated = millis();
        }
        loop();
    }
}
void command(const std::string &text) { simulated::serialInput += text + "\n"; advance(50); }
void configure() {
    command("ENCODER_CONFIG 38.46153846153846 23.80952380952381");
    command("SET_NORTH"); command("SET_LEVEL");
    CHECK(fresh(millis()));
}
void begin(double yaw, double pitch) {
    targetYaw = yaw; targetPitch = pitch; updated = millis(); sequence = 0; heartbeat = true;
    command("CELESTIAL_GOTO 7 " + std::to_string(yaw) + " " + std::to_string(pitch));
    CHECK(celestialActive());
}
void acquire() {
    for (unsigned n=0; n<900 && !celestialTracking && !poseStopping; ++n) advance(100);
    CHECK(celestialActive() && celestialTracking && !poseStopping);
    CHECK(fabs(yawAxis.error) <= TOLERANCE_DEG && fabs(pitchAxis.error) <= TOLERANCE_DEG);
}
}
int main(int argc, char **argv) {
    CHECK(argc == 2); const std::string scenario = argv[1];
    simulated::independentTick = tick;
    simulated::encoderPlantFeedback = true;
    Wire.encoder.present = Wire1.encoder.present = true;
    if (scenario == "bus_recovery") Wire.beginFails = true;
    setup();
    simulated::motors[0].physicalSign = simulated::motors[1].physicalSign = 1;
    simulated::motors[0].degreesPerStep = 1.0/YAW_TRACK_PULSES_PER_DEG;
    simulated::motors[1].degreesPerStep = 1.0/PITCH_TRACK_PULSES_PER_DEG;
    advance(200);
    CHECK(commandIdle() && simulated::commands.empty());
    if (scenario == "bus_recovery") {
        CHECK(!sensorSnapshot.positions[0].fresh(millis()) && manualReady());
        Wire.beginFails = false; advance(1100);
        CHECK(sensorSnapshot.positions[0].fresh(millis()));
        configure();
    } else if (scenario == "north" || scenario == "level" || scenario == "pose" || scenario == "alignment_stop") {
        configure();
        for (unsigned n=1; n<=4; ++n) {
            simulated::encoderAxisDisturbance[0] = n;
            simulated::encoderAxisDisturbance[1] = n;
            advance(30);
        }
        if (scenario == "pose") command("MOVE 1 1 100");
        else command(scenario == "north" ? "NORTH" : "LEVEL");
        CHECK(poseActive);
        if (scenario == "alignment_stop") command("STOP");
        for (unsigned n=0; n<600 && !commandIdle(); ++n) advance(100);
        CHECK(commandIdle() && poseMotorsStopped());
        if (scenario == "pose") {
            CHECK(fabs(heading.continuous-5) < .4 && fabs(physicalPitch()-5) < .4);
            CHECK(carriageMotor->getCurrentPosition() == 100);
        } else if (scenario != "alignment_stop") {
            CHECK(fabs(scenario == "north" ? heading.continuous : physicalPitch()) < .4);
            CHECK((scenario == "north" ? pitchMotor : yawMotor)->getCurrentPosition() == 0);
        }
    } else if (scenario == "admission") {
        command("CELESTIAL_GOTO 7 10 10"); CHECK(!poseActive);
        command("SET_NORTH"); CHECK(!referenceSet);
        command("ENCODER_CONFIG 360 0"); CHECK(!encoderReferences[0].degreesPerRevolution);
        command("ENCODER_CONFIG nan 360"); CHECK(!encoderReferences[0].degreesPerRevolution);
        configure();
        command("ENCODER_CONFIG 38.46153846153846 23.80952380952381");
        CHECK(!referenceSet && !pitchReady);
        command("SET_NORTH"); command("CELESTIAL_GOTO 7 10 10"); CHECK(!poseActive);
        command("SET_LEVEL"); CHECK(fresh(millis()));
        command("JOG 0 0 0"); command("JOG 100 0 0");
        const auto zero = encoderReferences[0].zeroTicks;
        command("SET_NORTH"); CHECK(encoderReferences[0].zeroTicks == zero);
    } else if (scenario == "references" || scenario == "protocol") {
        configure();
        for (unsigned n=1; n<=5; ++n) {
            simulated::encoderAxisDisturbance[0] = -2.0*n;
            simulated::encoderAxisDisturbance[1] = 3.0*n;
            advance(30);
        }
        CHECK(fabs(orientation.heading-350) < .03 && fabs(physicalPitch()-15) < .03);
        CHECK(fabs(heading.continuous+10) < .03);
        command("SET_NORTH"); CHECK(fabs(heading.continuous) < .01);
        CHECK(fabs(physicalPitch()-15) < .03);
        command("SET_LEVEL"); CHECK(fabs(physicalPitch()) < .01);
        command("SNAP 9");
        CHECK(Serial.output.find("feedback=AS5600 orientation_valid=YES") != std::string::npos);
        if (scenario == "protocol") std::puts(Serial.output.c_str());
    } else if (scenario == "no_step_substitution") {
        configure();
        simulated::motors[0].position = 12345; simulated::motors[1].position = -1000;
        advance(100); CHECK(heading.continuous == 0 && physicalPitch() == 0);
        simulated::encoderAxisDisturbance[0] = 1.0; advance(100);
        CHECK(fabs(heading.continuous-1) < .03);
    } else if (scenario == "wrap") {
        configure();
        for (unsigned n=0; n<800; ++n) { simulated::encoderAxisDisturbance[0] += 1; advance(20); }
        CHECK(fabs(heading.continuous-800) < .03 && fabs(orientation.heading-80) < .03);
        begin(79, 0); CHECK(fabs(yawAxis.target-799) < .03); acquire();
    } else {
        configure();
        begin(scenario == "goto" ? 25 : 0, scenario == "goto" ? 12 : 0);
        acquire();
        if (scenario == "tracking" || scenario == "stall" || scenario == "wrong_direction") {
            yawRate = .02; pitchRate = .015;
            if (scenario == "stall") simulated::motors[0].frozen = true;
            if (scenario == "wrong_direction") simulated::motors[0].physicalSign = -1;
            advance(scenario == "tracking" ? 30000 : 180000);
            if (scenario != "tracking") { CHECK(!celestialActive() || poseStopping); return 0; }
            CHECK(celestialTracking && !poseStopping && fabs(yawAxis.error) < .2 && fabs(pitchAxis.error) < .2);
            CHECK(simulated::commands.size() <= 4); // Persistent motion, not burst chasing.
        } else if (scenario == "disconnect" || scenario == "bad_magnet" || scenario == "worker_stale") {
            if (scenario == "disconnect") Wire.encoder.present = false;
            if (scenario == "bad_magnet") Wire1.encoder.status = 0x30;
            if (scenario == "worker_stale") Wire.encoder.requestDelayMs = 200;
            advance(3500); CHECK(commandIdle() && !celestialActive() && poseMotorsStopped());
            Wire.encoder.present = true; Wire1.encoder.status = 0x20; Wire.encoder.requestDelayMs = 0;
            advance(200); command("CELESTIAL_GOTO 8 0 0"); CHECK(!celestialActive());
            configure(); begin(0,0); acquire();
        } else if (scenario == "blocked_foreground") {
            yawRate = .03; advance(2000);
            simulated::advance(200); CHECK(motionWatchdog.tripped());
            advance(3500); CHECK(commandIdle() && poseMotorsStopped());
        } else if (scenario == "lease") {
            heartbeat = false; advance(6500); CHECK(commandIdle() && !celestialActive());
            CHECK(Serial.output.find("lease expired") != std::string::npos);
        } else if (scenario == "abort") {
            command("X"); CHECK(phase == Phase::ABORTED && poseMotorsStopped()); return 0;
        } else if (scenario != "goto" && scenario != "stop" && scenario != "takeover") return 2;
        command("STOP"); advance(3500); CHECK(commandIdle() && poseMotorsStopped());
        const auto count = simulated::commands.size(); advance(500); CHECK(simulated::commands.size() == count);
        if (scenario == "takeover") {
            command("JOG 0 0 0"); command("JOG 100 100 100");
            CHECK(manualActive && yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
        }
    }
    std::printf("PASS encoder motion: %s\n", scenario.c_str());
}
