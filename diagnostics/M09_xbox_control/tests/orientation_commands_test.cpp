// Real command dispatcher/controller; synthetic plant, never hardware access.
#include "../src/main.cpp"
#include <cstdio>
#include <cstdlib>
#include <string>
namespace {
void require(bool ok, const char *what, int line) {
    if (!ok) { std::fprintf(stderr, "FAIL alignment line %d: %s\n%s", line, what, Serial.output.c_str()); std::exit(1); }
}
#define CHECK(x) require((x), #x, __LINE__)
uint32_t stopAt = 0, handledAt = 0;
void tick(uint32_t now) {
    motionWatchdog.check(now); commandWatchdog.check(now);
    if (stopAt && now >= stopAt) { simulated::serialInput += "STOP\n"; stopAt = 0; }
    if (poseStopping && !handledAt) handledAt = now;
}
void advance(uint32_t ms) { const auto start = millis(); while (millis() - start < ms) loop(); }
void command(const std::string &line) { simulated::serialInput += line + "\n"; advance(20); }
size_t verifyPitchResponse(bool continuous, size_t firstCommand = 0) {
    size_t selected = static_cast<size_t>(-1);
    const auto deadline = millis() + 15000;
    while (poseActive && millis() < deadline && selected == static_cast<size_t>(-1)) {
        for (size_t index = firstCommand; index < simulated::commands.size(); ++index) {
            const auto &move = simulated::commands[index];
            if (move.stepPin == tmp_hardware::PITCH_STEP_PIN && move.continuous == continuous) {
                selected = index; break;
            }
        }
        if (selected == static_cast<size_t>(-1)) loop();
    }
    CHECK(selected != static_cast<size_t>(-1));
    const double started = simulated::commands[selected].pitchAt;
    const auto responseDeadline = millis() + 3000;
    while (poseActive && fabs(physicalPitch() - started) < 0.05 && millis() < responseDeadline) loop();
    CHECK(fabs(physicalPitch() - started) >= 0.05);
    CHECK(fabs(physicalPitch()) < fabs(started));
    CHECK((started < 0 && physicalPitch() > started) || (started > 0 && physicalPitch() < started));
    return selected;
}
void finishAlignment(bool pass) {
    const auto start = millis();
    while (poseActive && millis() - start < 600000) loop();
    advance(150);
    CHECK(commandIdle() && poseMotorsStopped() && !poseActive && !alignmentPaused && !poseStopping);
    CHECK(manualReady());
    CHECK((Serial.output.find(" RESULT: PASS") != std::string::npos) == pass);
}
void manual() {
    command("JOG 0 0 0"); CHECK(manualActive);
    for (int n=0; n<5; ++n) command("JOG 200 -200 200");
    CHECK(yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
    command("STOP"); advance(1500); CHECK(manualReady());
}
void firstYawAfterLevel() {
    command("JOG 0 0 0"); CHECK(manualActive);
    simulated::continuousStartDelayMs = 50;
    Serial.output.clear(); command("JOG 200 0 0");
    CHECK(manualActive && manualYaw.direction == 1);
    CHECK(Serial.output.find("MANUAL AXIS ERROR: YAW continuous motor stopped unexpectedly") == std::string::npos);
    advance(100); CHECK(yawMotor->isRunning());
    simulated::continuousStartDelayMs = 0;
    command("STOP"); advance(1500); CHECK(manualReady());
}
void playKeyframe() {
    command("SNAP 81"); CHECK(Serial.output.find("KEYFRAME_SNAPSHOT id=81") != std::string::npos);
    const auto y=yawMotor->getCurrentPosition(), p=pitchMotor->getCurrentPosition(), c=carriageMotor->getCurrentPosition();
    command("KEYMOVE 82 " + std::to_string(keyframeEpoch) + " " + std::to_string(y+100) + " " +
        std::to_string(p-200) + " " + std::to_string(c+80) + " 5000");
    CHECK(keyframeActive);
    command("LEVEL"); command("NORTH"); CHECK(keyframeActive); // Busy presses do not preempt.
    advance(6000); CHECK(manualReady());
    CHECK(Serial.output.find("KEYMOVE RESULT id=82 status=PASS") != std::string::npos);
}
void align(const char *name) {
    Serial.output.clear(); command(name); CHECK(alignmentActive()); finishAlignment(true);
}
void manualToAlign(const char *name, uint8_t expectedPending) {
    command("JOG 0 0 0"); CHECK(manualActive);
    for (int n=0; n<5; ++n) command("JOG 500 -500 500");
    CHECK(yawMotor->isRunning() && pitchMotor->isRunning() && carriageMotor->isRunning());
    Serial.output.clear(); command(name);
    CHECK(manualActive && manualEnding && !alignmentActive());
    CHECK(pendingOrientation==expectedPending);
    CHECK(Serial.output.find(std::string(name)+" HANDOFF: braking manual motion")!=std::string::npos);
    CHECK(Serial.output.find("POSE STOPPING") == std::string::npos);
    const auto start=millis();
    while (!alignmentActive() && millis()-start<5000) loop();
    CHECK(alignmentActive() && !manualActive && pendingOrientation==0);
    advance(20); // Drain queued acknowledgement after the manual service starts it.
    CHECK(Serial.output.find(std::string(name)+" ACCEPTED")!=std::string::npos);
    finishAlignment(true);
}
}
int main(int argc, char **argv) {
    CHECK(argc==2); const std::string test=argv[1];
    const bool level=test.find("level")!=std::string::npos;
    simulated::physicalPitchUsesRoll=false; simulated::rawBnoRoll=23;
    simulated::baselinePitch=level ? 50 : 8;
    simulated::baselineYaw=level ? 40 : 70;
    simulated::independentTick=tick;
    if (test=="level_zero") simulated::baselinePitch=0;
    if (test=="level_manual_first_yaw") simulated::baselinePitch=0;
    if (test=="level_delayed_slew_start") simulated::baselinePitch=8;
    if (test=="level_tolerance") simulated::baselinePitch=.25;
    if (test=="level_negative") simulated::baselinePitch=-50;
    if (test=="level_precision_negative") simulated::baselinePitch=-1.24;
    if (test=="level_precision_positive") simulated::baselinePitch=1.24;
    if (test=="level_boundary_precision_negative") simulated::baselinePitch=-3.9;
    if (test=="level_boundary_precision_positive") simulated::baselinePitch=3.9;
    if (test=="level_boundary_slew_negative") simulated::baselinePitch=-4.1;
    if (test=="level_boundary_slew_positive") simulated::baselinePitch=4.1;
    if (test=="north_zero") simulated::baselineYaw=0;
    if (test=="north_tolerance") { simulated::baselineYaw=.2; simulated::noiseAmplitude=.05; }
    if (test=="north_negative") simulated::baselineYaw=290;
    if (test=="north_359") simulated::baselineYaw=359;
    if (test=="north_1") simulated::baselineYaw=1;
    if (test=="north_180") simulated::baselineYaw=180;
    if (test=="north_219") simulated::baselineYaw=219;
    if (test=="north_large_success") simulated::baselineYaw=160;
    if (test=="north_travel_accept") simulated::yawDisturbance=100;
    if (test.find("low_quality")!=std::string::npos) simulated::accuracy=0;
    if (test=="missing") simulated::bnoInitFails=true;
    if (test=="level_eighty") simulated::baselinePitch=-79;
    if (test=="level_guard") simulated::baselinePitch=89.2;
    setup();
    // The physically verified a8c85a1 baseline uses -1 for both continuous
    // and finite LEVEL commands. Keep the two responses independently
    // configurable while matching that baseline's actual powered sign.
    simulated::motors[1].physicalSign=-1;
    simulated::motors[1].finitePhysicalMultiplier=1;
    if (test=="level_wrong_direction") simulated::motors[1].physicalSign=1;
    if (test=="north_usability") {
        advance(50); command("JOG 0 0 0"); advance(2100);
        CHECK(manualActive && bnoValid && bnoAccuracy==3 && northUsable && !referenceSet);
        CHECK(std::string(northUnavailableReason(millis()))=="usable");
        command("STOP"); advance(1000); Serial.output.clear(); command("NORTH");
        CHECK(alignmentActive() && Serial.output.find("NORTH ACCEPTED")!=std::string::npos);
        command("STOP"); finishAlignment(false);
        std::printf("PASS explicit alignment: %s\n",test.c_str()); return 0;
    }
    advance(2200);
    // Nominal 200 full steps * 8 microsteps * 15:1 / 360 = 66.667 output pulses/deg.
    // The firmware is NOT told this ratio; it must derive response from feedback.
    simulated::motors[1].degreesPerStep = 360.0/(1600*15);
    if (test=="level_high_reduction") simulated::motors[1].degreesPerStep=1.0/2000;
    CHECK(commandIdle() && manualReady());
    if (test=="manual_handoff") {
        manualToAlign("LEVEL", 1); manual();
        manualToAlign("NORTH", 2); manual();
    } else if (test=="level_manual_first_yaw") {
        align("LEVEL"); firstYawAfterLevel();
    } else if (test=="level_delayed_slew_start") {
        simulated::continuousStartDelayMs = 50;
        Serial.output.clear(); command("LEVEL");
        CHECK(alignmentActive() && Serial.output.find("LEVEL ACCEPTED") != std::string::npos);
        advance(80);
        CHECK(alignmentActive() && Serial.output.find("Continuous slew stopped unexpectedly") == std::string::npos);
        simulated::continuousStartDelayMs = 0;
        command("STOP"); finishAlignment(false);
    } else if (test=="pending_stop") {
        command("JOG 0 0 0");
        for (int n=0; n<5; ++n) command("JOG 500 -500 500");
        Serial.output.clear(); command("LEVEL");
        CHECK(pendingOrientation==1 && manualEnding && !alignmentActive());
        command("STOP"); CHECK(pendingOrientation==0);
        advance(2000);
        CHECK(manualReady() && !alignmentActive());
        CHECK(Serial.output.find("LEVEL ACCEPTED")==std::string::npos);
    } else if (test=="missing") {
        command("LEVEL"); command("NORTH"); CHECK(simulated::commands.empty());
        CHECK(Serial.output.find("BNO unavailable")!=std::string::npos);
        manual(); command("MOVE 0 0 100"); advance(3000); CHECK(manualReady()); playKeyframe(); manual();
    } else if (test=="north_low_quality") {
        Serial.output.clear(); command("NORTH");
        CHECK(!poseActive && simulated::commands.empty());
        CHECK(Serial.output.find("NORTH REJECTED: magnetic heading unavailable: accuracy_below_2")!=std::string::npos);
        CHECK(std::string(northUnavailableReason(millis()))=="accuracy_below_2");
        manual(); playKeyframe();
    } else if (test=="north_219_guard") {
        // The startup reference was qualified at 70 deg. Moving the continuous
        // generated position to heading 219 puts both the present position and
        // the shortest-path north target outside the preserved +/-185 deg cable guard.
        simulated::yawDisturbance=149; advance(100);
        CHECK(fabs(orientation.heading-219)<0.2);
        Serial.output.clear(); command("NORTH");
        CHECK(!poseActive && simulated::commands.empty());
        CHECK(Serial.output.find("NORTH yaw travel guard exceeded")!=std::string::npos);
        CHECK(Serial.output.find("guard_basis=absolute_unwrapped_envelope")!=std::string::npos);
        CHECK(Serial.output.find("current_position_offset_deg=")!=std::string::npos);
        CHECK(Serial.output.find("projected_target_offset_deg=")!=std::string::npos);
        CHECK(Serial.output.find("shortest_move_deg=141.000")!=std::string::npos);
        CHECK(Serial.output.find("limit_abs_offset_deg=185.0")!=std::string::npos);
        CHECK(Serial.output.find("violation=current_position_and_projected_target_outside")!=std::string::npos);
    } else if (test=="north_projected_limit_guard") {
        // Startup north is centered at heading 70. At heading 184 the current
        // unwrapped position is still inside +185, but the shortest north path
        // ends at +360 and would cross the cable envelope.
        simulated::yawDisturbance=114; advance(100);
        CHECK(fabs(orientation.heading-184)<0.2);
        Serial.output.clear(); command("NORTH");
        CHECK(!poseActive && simulated::commands.empty());
        CHECK(Serial.output.find("current_position_offset_deg=184.000")!=std::string::npos);
        CHECK(Serial.output.find("projected_target_offset_deg=360.000")!=std::string::npos);
        CHECK(Serial.output.find("shortest_move_deg=176.000")!=std::string::npos);
        CHECK(Serial.output.find("violation=projected_target_outside")!=std::string::npos);
    } else if (test=="invalid_admission") {
        simulated::invalidQuaternion=true; advance(25); command("LEVEL"); command("NORTH");
        CHECK(!poseActive && simulated::commands.empty());
        CHECK(Serial.output.find("invalid orientation sample")!=std::string::npos);
        manual(); playKeyframe();
        simulated::invalidQuaternion=false; advance(200); align("LEVEL");
    } else if (test=="handoffs") {
        manual(); playKeyframe(); manual();
        align("LEVEL"); manual(); align("NORTH"); manual();
        align("LEVEL"); playKeyframe(); manual(); align("NORTH"); playKeyframe(); manual();
        playKeyframe(); align("LEVEL"); manual(); align("LEVEL"); align("NORTH"); manual();
        align("NORTH"); align("LEVEL"); manual();
        for (int n=0; n<3; ++n) { align("LEVEL"); align("NORTH"); }
    } else if (test=="admission" || test=="level_guard") {
        command("LEVEL 1"); command("NORTH 0"); CHECK(simulated::commands.empty());
        if (test=="level_guard") { command("LEVEL"); CHECK(!poseActive); }
        else {
            simulated::bnoPauseStart=millis(); simulated::bnoPauseEnd=millis()+10000;
            advance(200); command("LEVEL"); command("NORTH"); CHECK(!poseActive);
            manual(); playKeyframe();
        }
    } else {
        const auto epoch=keyframeEpoch;
        const double initial=level ? physicalPitch() : shortestDifference(0, orientation.heading);
        const auto y=yawMotor->getCurrentPosition(), p=pitchMotor->getCurrentPosition(), c=carriageMotor->getCurrentPosition();
        Serial.output.clear(); command(level ? "LEVEL" : "NORTH"); CHECK(alignmentActive());
        CHECK(yawRequired==!level && pitchControlRequired()==level);
        const bool precisionCase = test.find("level_precision_")==0 || test.find("level_boundary_precision_")==0;
        const bool boundarySlewCase = test.find("level_boundary_slew_")==0;
        if (test=="level_positive" || test=="level_negative") {
            const size_t slew = verifyPitchResponse(true);
            const size_t precision = verifyPitchResponse(false, slew + 1);
            CHECK(precision > slew);
        } else if (precisionCase) {
            const size_t precision = verifyPitchResponse(false);
            CHECK(!simulated::commands[precision].continuous);
        } else if (boundarySlewCase) {
            const size_t slew = verifyPitchResponse(true);
            CHECK(simulated::commands[slew].continuous);
        }
        if (test=="north_stop_only") {
            advance(100); command("STOP"); finishAlignment(false);
            CHECK(Serial.output.find("NORTH RESULT: STOPPED")!=std::string::npos);
        } else if (test.find("stop")!=std::string::npos) {
            advance(100); const auto moves=simulated::commands.size();
            if (test.find("stall_stop")!=std::string::npos || test.find("pause_stop")!=std::string::npos) {
                const auto at=millis()+(test.find("pause_stop")!=std::string::npos ? 500 : 30); stopAt=at; simulated::blockBnoMs=1000;
                advance(1100); CHECK(handledAt && handledAt-at<=20);
            } else command("STOP");
            finishAlignment(false); CHECK(simulated::commands.size()==moves);
            CHECK(keyframeEpoch==epoch); manual(); playKeyframe();
            simulated::baselinePitch=simulated::actualPitch()-simulated::motors[1].degrees;
            align(level ? "NORTH" : "LEVEL");
        } else if (test.find("transient")!=std::string::npos || test.find("reset")!=std::string::npos || test.find("forced")!=std::string::npos) {
            advance(100);
            if (test.find("forced")!=std::string::npos) simulated::advance(200); // Foreground stalled: independent watchdog still stops.
            else if (test.find("reset")!=std::string::npos) simulated::resetDuringPoll=true;
            else simulated::blockBnoMs=1000;
            advance(1100);
            CHECK(Serial.output.find(" PAUSED:")!=std::string::npos);
            CHECK(Serial.output.find(" RESUMED")!=std::string::npos);
            CHECK((keyframeEpoch==epoch)==(test.find("forced")==std::string::npos));
            // Foreground braking preserves coordinates; actual forced stops invalidate.
            finishAlignment(true); manual(); playKeyframe();
        } else if (test.find("stale")!=std::string::npos || test.find("invalid")!=std::string::npos) {
            advance(100);
            if (test.find("stale")!=std::string::npos) {
                simulated::bnoPauseStart=millis(); simulated::bnoPauseEnd=millis()+100000;
            } else simulated::invalidQuaternion=true;
            advance(1400); CHECK(poseActive && alignmentPaused && poseMotorsStopped());
            const auto moves=simulated::commands.size(); advance(400); finishAlignment(false);
            CHECK(simulated::commands.size()==moves && keyframeEpoch==epoch);
            manual(); playKeyframe(); manual(); // No BNO recovery required for ordinary functions.
            simulated::bnoPauseEnd=0; simulated::invalidQuaternion=false; advance(300);
            align(level ? "LEVEL" : "NORTH");
        } else if (test.find("abort")!=std::string::npos) {
            command("X"); CHECK(phase==Phase::ABORTED && poseMotorsStopped());
            command("JOG 0 0 0"); CHECK(!manualActive);
        } else if (test=="north_braking_rebound") {
            const auto deadline = millis() + 15000;
            while (poseActive && yawAxis.motion != Motion::BRAKING && millis() < deadline) loop();
            CHECK(yawAxis.motion == Motion::BRAKING);
            simulated::yawDisturbance = 1.0;
            const auto precisionDeadline = millis() + 5000;
            while (poseActive && yawAxis.motion != Motion::PRECISION && millis() < precisionDeadline) loop();
            CHECK(poseActive && yawAxis.motion == Motion::PRECISION);
            simulated::yawDisturbance = 0;
            advance(500);
            CHECK(poseActive && Serial.output.find("Yaw wrong-direction/runaway guard") == std::string::npos);
            finishAlignment(true);
        } else if (test=="north_precision_noise") {
            const auto deadline = millis() + 15000;
            while (poseActive && yawAxis.motion != Motion::PRECISION && millis() < deadline) loop();
            CHECK(yawAxis.motion == Motion::PRECISION);
            simulated::noiseAmplitude = 0.8;
            advance(500);
            CHECK(poseActive && Serial.output.find("Yaw wrong-direction/runaway guard") == std::string::npos);
            simulated::noiseAmplitude = 0;
            finishAlignment(true);
        } else if (test=="north_wrong_direction" || test=="level_wrong_direction" || test=="level_no_progress") {
            if (level) simulated::motors[1].frozen=true;
            else simulated::motors[0].physicalSign=1;
            if (test=="level_wrong_direction") simulated::motors[1].frozen=false;
            finishAlignment(false); manual();
            if (test=="north_wrong_direction")
                CHECK(Serial.output.find("Yaw wrong-direction/runaway guard")!=std::string::npos);
        } else {
            finishAlignment(true);
            CHECK(fabs(level ? physicalPitch() : shortestDifference(0,orientation.heading))<=TOLERANCE_DEG);
            CHECK((level ? yawMotor->getCurrentPosition()==y : pitchMotor->getCurrentPosition()==p));
            CHECK(carriageMotor->getCurrentPosition()==c && keyframeEpoch==epoch);
            if (fabs(initial)<=TOLERANCE_DEG) CHECK(simulated::commands.empty());
            else {
                CHECK(!simulated::commands.empty());
                if (!level) CHECK((simulated::commands[0].steps>0)==(initial<0));
                bool fullSpeed=false;
                for (const auto &move:simulated::commands) {
                    CHECK(move.stepPin==(level ? tmp_hardware::PITCH_STEP_PIN : tmp_hardware::YAW_STEP_PIN));
                    CHECK(!move.beforeStoppedSample && !move.wasBraking);
                    if (move.continuous) {
                        CHECK(move.speed==alignmentSpeed(level) && move.acceleration==alignmentAcceleration(level));
                        fullSpeed=true;
                    }
                }
                if (fabs(initial)>=40) CHECK(fullSpeed);
                if (test=="level_high_reduction") CHECK(millis()-controlStartedAt>LEG_TIMEOUT_MS && alignmentDeadlineMs>LEG_TIMEOUT_MS);
            }
            CHECK(Serial.output.find(level ? "YAW settled=NOT_REQUESTED" : "PITCH settled=NOT_REQUESTED")!=std::string::npos);
            CHECK(Serial.output.find(" RESULT: PASS")<Serial.output.find("M09 READY"));
            if (test=="level_precision_negative") {
                CHECK(Serial.output.find("mode=PRECISION requested_bno_dir=1 step_sign=-1")!=std::string::npos);
                CHECK(Serial.output.find("call=move rc=0")!=std::string::npos);
                CHECK(Serial.output.find("event=DIR_CHECK")!=std::string::npos);
            }
            if (test=="level_precision_positive") {
                CHECK(Serial.output.find("mode=PRECISION requested_bno_dir=-1 step_sign=1")!=std::string::npos);
            }
            if (test=="level_negative") {
                CHECK(Serial.output.find("mode=SLEW requested_bno_dir=1 step_sign=-1")!=std::string::npos);
                CHECK(Serial.output.find("call=runBackward rc=0")!=std::string::npos);
                CHECK(Serial.output.find("mode=PRECISION requested_bno_dir=1 step_sign=-1")!=std::string::npos);
            }
            if (test=="level_positive") {
                CHECK(Serial.output.find("mode=SLEW requested_bno_dir=-1 step_sign=1")!=std::string::npos);
                CHECK(Serial.output.find("call=runForward rc=0")!=std::string::npos);
                CHECK(Serial.output.find("mode=PRECISION requested_bno_dir=-1 step_sign=1")!=std::string::npos);
            }
        }
    }
    std::printf("PASS explicit alignment: %s\n",test.c_str());
}
