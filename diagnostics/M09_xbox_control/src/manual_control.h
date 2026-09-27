// Included inside main.cpp's private namespace. Manual rates retain the motor
// engine, signs and acceleration; optional orientation is not a motion gate.
#include "pitch_direction_diagnostics.h"
void rejectManual(const char *reason) {
    queueText("MANUAL REJECTED: "); queueText(reason); queueText("\n");
}
bool manualMotorsStopped() {
    return yawMotor && pitchMotor && carriageMotor && !yawMotor->isRunning() &&
        !pitchMotor->isRunning() && !carriageMotor->isRunning();
}
bool manualReady() {
    if (!controlReady || !commandIdle() || !commandWatchdog.configured() || !manualMotorsStopped()) return false;
    // Drain any prior orientation-watchdog callback before this independent
    // manual session. A missing BNO watchdog never prevents manual readiness.
    if (motionWatchdog.configured()) {
        motionWatchdog.disarm();
        if (!motionWatchdog.clearTripWhenStopped()) return false;
    }
    return !commandWatchdog.tripped() || commandWatchdog.clearTripWhenStopped();
}
void completeManualSession(const char *reason) {
    pitchDirectionDiagnostics.checkPending = false;
    commandWatchdog.disarm();
    commandWatchdog.clearTripWhenStopped();
    manualYaw = {}; manualPitch = {}; manualCarriage = {};
    manualActive = manualEnding = poseActive = carriagePending = false;
    finalPrinted = true; phase = Phase::COMPLETE;
    queueText("MANUAL STOPPED: "); queueText(reason); queueText("; rearm centered\n");
    queueText(manualReady() ? "M09 READY\n" : "M09 BUSY\n");
}
void stopManualSession(const char *reason) {
    // The independent command timer already stops lost commands. Reassert in
    // the foreground and drain the existing motor queue before allowing rearm.
    // The independent callback may already have forced every motor idle, so
    // isRunning() alone cannot tell us that coordinate confidence was lost.
    invalidateKeyframes(reason);
    stopMotors();
    completeManualSession(reason);
}
void beginManual() {
    if (!manualReady()) {
        rejectManual("motor control, stopped motors and command watchdog required"); return;
    }
    manualYaw = {}; manualPitch = {}; manualCarriage = {};
    pitchDirectionDiagnostics.checkPending = false;
    motionWatchdog.disarm();
    manualActive = true; manualEnding = false; finalPrinted = false;
    operation = Operation::MANUAL; phase = Phase::MANUAL;
    yawRequired = accuracyRequired = false; accuracyGrace = {};
    // A centered session has no continuous motion to abandon. Its command
    // lease starts before the first nonzero request can reach a motor.
    queueText("MANUAL READY: JOG yaw pitch carriage [-1000,1000] every 20 ms; STOP exits; X aborts\n");
}
bool executeManualCommand(char **tokens, unsigned count) {
    if (strcmp(tokens[0], "STATUS") == 0 && count == 1) {
        queueText(phase == Phase::ABORTED ? "M09 ABORTED\n" :
            (manualActive ? "M09 MANUAL\n" : (manualReady() ? "M09 READY\n" : "M09 BUSY\n")));
        return true;
    }
    if (strcmp(tokens[0], "STOP") == 0 && count == 1) {
        if (manualActive) {
            safety(); if (finalPrinted) return true;
            manualEnding = true;
            manualYaw.request = manualPitch.request = manualCarriage.request = 0;
            // Keep the lease until serviceManual actually issues braking to
            // every moving axis. Receiving zero alone has not stopped pulses.
        } else if (keyframeActive) stopKeyframe("operator STOP");
        else beginPoseStop();
        return true;
    }
    if (strcmp(tokens[0], "JOG") != 0) return false;
    double yaw = 0, pitch = 0, carriage = 0;
    // Older two-axis hosts explicitly request zero carriage velocity.
    if ((count != 3 && count != 4) || !parseAngle(tokens[1], yaw) || !parseAngle(tokens[2], pitch) ||
        (count == 4 && !parseAngle(tokens[3], carriage)) ||
        fabs(yaw) > 1000 || fabs(pitch) > 1000 || fabs(carriage) > 1000 ||
        trunc(yaw) != yaw || trunc(pitch) != pitch || trunc(carriage) != carriage) {
        rejectManual("use JOG integer_yaw integer_pitch [integer_carriage], each in [-1000,1000]"); return true;
    }
    if (!manualActive) {
        if (yaw != 0 || pitch != 0 || carriage != 0) rejectManual("first send JOG 0 0 0 from READY to arm");
        else beginManual();
        return true;
    }
    if (manualEnding) { rejectManual("stopping; wait for READY and rearm centered"); return true; }
    const uint32_t now = millis();
    // Check the old lease before publishing new requests or accepting a late
    // packet; timeout recovery always returns to a centered arming handshake.
    commandWatchdog.check(now); safety(); if (finalPrinted) return true;
    if (commandWatchdog.armed()) {
        commandWatchdog.recordFresh(now); safety(); if (finalPrinted) return true;
    } else if ((yaw != 0 || pitch != 0 || carriage != 0) && !commandWatchdog.arm(now)) {
        rejectManual("command watchdog unavailable; retry from stopped state"); return true;
    }
    manualYaw.request = static_cast<int>(yaw);
    manualPitch.request = static_cast<int>(pitch);
    manualCarriage.request = static_cast<int>(carriage);
    return true;
}
void failManualAxis(FastAccelStepper *motor, ManualAxis &manual, const char *name, const char *reason) {
    invalidateKeyframes(reason);
    motor->forceStop();
    if (motor == pitchMotor) {
        pitchDirectionDiagnostics.checkPending = false;
        tracePitchDirection("FORCE_STOP");
    }
    manual.request = 0; manual.braking = true; manual.observing = false; manual.brakeAt = millis();
    queueText("MANUAL AXIS ERROR: "); queueText(name); queueText(" "); queueText(reason); queueText("\n");
    // This is not a fault latch. A later valid JOG may retry after the motor
    // has actually stopped and the existing reversal observation has elapsed.
}
bool serviceManualBraking(FastAccelStepper *motor, ManualAxis &manual, int requestedDirection,
                          const char *name, uint32_t now) {
    if (manual.braking) {
        if (motor->isRunning()) {
            if (now - manual.brakeAt >= BRAKING_TIMEOUT_MS)
                failManualAxis(motor, manual, name, "braking timeout; axis stopped");
            return true;
        }
        manual.braking = false; manual.direction = 0; manual.rate = 0;
        manual.observing = true; manual.stoppedAt = now;
        if (motor == pitchMotor) tracePitchDirection("STOPPED");
        return true;
    }
    if (manual.observing) {
        if (now - manual.stoppedAt < PRECISION_OBSERVE_MS) return true;
        manual.observing = false;
    }
    if (manual.direction && (!requestedDirection || requestedDirection != manual.direction)) {
        motor->stopMove(); manual.braking = true; manual.brakeAt = now;
        if (motor == pitchMotor) {
            pitchDirectionDiagnostics.checkPending = false;
            tracePitchDirection(requestedDirection ? "BRAKE_REVERSE" : "BRAKE_ZERO");
        }
        return true;
    }
    if (manual.direction && !motor->isRunning()) {
        failManualAxis(motor, manual, name, "continuous motor stopped unexpectedly"); return true;
    }
    return false;
}
void serviceManualAxis(Axis &axis, ManualAxis &manual, uint32_t now) {
    const int requestedDirection = manual.request == 0 ? 0 : (manual.request > 0 ? 1 : -1);
    const char *name = axis.pitch ? "PITCH" : "YAW";
    if (serviceManualBraking(axis.motor, manual, requestedDirection, name, now) || !requestedDirection) return;
    const uint32_t rate = std::max<uint32_t>(1, static_cast<uint32_t>(
        lround(fabs(static_cast<double>(manual.request)) * slewSpeed(axis.pitch) / 1000.0)));
    if (!manual.direction || rate != manual.rate) {
        safety(); if (finalPrinted) return;
        if (axis.motor->setAcceleration(slewAcceleration(axis.pitch)) != 0 || axis.motor->setSpeedInHz(rate) != 0) {
            failManualAxis(axis.motor, manual, name, "FastAccelStepper configuration rejected"); return;
        }
        if (!manual.direction) {
            const int direction = stepDirection(manual.request, axis.pitch);
            const auto result = direction > 0 ? axis.motor->runForward() : axis.motor->runBackward();
            if (axis.pitch) tracePitchDirection("RUN", direction > 0 ? "runForward" : "runBackward", static_cast<int>(result));
            if (result != MOVE_OK) {
                failManualAxis(axis.motor, manual, name, "FastAccelStepper motion rejected"); return;
            }
            manual.direction = requestedDirection;
            if (axis.pitch) {
                pitchDirectionDiagnostics.checkAt = millis();
                pitchDirectionDiagnostics.checkPending = true;
            }
        } else {
            axis.motor->applySpeedAcceleration();
        }
        manual.rate = rate;
    }
}
void serviceManualCarriage(uint32_t now) {
    auto &manual = manualCarriage;
    const int direction = manual.request == 0 ? 0 : (manual.request > 0 ? 1 : -1);
    if (serviceManualBraking(carriageMotor, manual, direction, "CARRIAGE", now) || !direction) return;
    const uint32_t rate = std::max<uint32_t>(1, static_cast<uint32_t>(
        lround(fabs(static_cast<double>(manual.request)) * CARRIAGE_MAX_SPEED_HZ / 1000.0)));
    if (!manual.direction || rate != manual.rate) {
        safety(); if (finalPrinted) return;
        if (carriageMotor->setAcceleration(CARRIAGE_ACCELERATION) != 0 || carriageMotor->setSpeedInHz(rate) != 0) {
            failManualAxis(carriageMotor, manual, "CARRIAGE", "FastAccelStepper configuration rejected"); return;
        }
        if (!manual.direction) {
            if ((direction > 0 ? carriageMotor->runForward() : carriageMotor->runBackward()) != MOVE_OK) {
                failManualAxis(carriageMotor, manual, "CARRIAGE", "FastAccelStepper motion rejected"); return;
            }
            manual.direction = direction;
        } else {
            carriageMotor->applySpeedAcceleration();
        }
        manual.rate = rate;
    }
}
void serviceManual() {
    safety(); if (finalPrinted) return;
    const uint32_t now = millis();
    serviceManualAxis(yawAxis, manualYaw, now); safety(); if (finalPrinted) return;
    serviceManualAxis(pitchAxis, manualPitch, now); safety(); if (finalPrinted) return;
    serviceManualCarriage(now); safety(); if (finalPrinted) return;
    servicePitchDirectionCheck();
    const bool centered = !manualYaw.request && !manualPitch.request && !manualCarriage.request;
    if (centered && (!yawMotor->isRunning() || manualYaw.braking) &&
        (!pitchMotor->isRunning() || manualPitch.braking) && (!carriageMotor->isRunning() || manualCarriage.braking)) {
        // All moving axes have now received stopMove/forceStop. Their motor
        // queues brake independently; no more host commands are needed to stop.
        commandWatchdog.disarm();
    }
    if (manualEnding && centered && manualMotorsStopped() &&
        !manualYaw.braking && !manualPitch.braking && !manualCarriage.braking &&
        !manualYaw.observing && !manualPitch.observing && !manualCarriage.observing) {
        completeManualSession("motors stopped"); return;
    }
    if (now - lastDisplay >= 250) {
        lastDisplay = now;
        char line[260];
        snprintf(line, sizeof(line), "MANUAL_STATE yaw_cmd=%d pitch_cmd=%d carriage_cmd=%d heading=%.3f pitch_roll=%.3f yaw_hz=%.1f pitch_hz=%.1f carriage_hz=%.1f carriage_steps=%ld\n",
            manualYaw.request, manualPitch.request, manualCarriage.request, orientation.heading, orientation.roll,
            yawMotor->getCurrentSpeedInMilliHz() / 1000.0, pitchMotor->getCurrentSpeedInMilliHz() / 1000.0,
            carriageMotor->getCurrentSpeedInMilliHz() / 1000.0,
            static_cast<long>(carriageMotor->getCurrentPosition()));
        queueText(line);
    }
}
