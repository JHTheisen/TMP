// Included inside main.cpp's private namespace after the preserved M08 helpers.
// Manual velocity uses the same hardware pulse engine, signs, acceleration,
// accepted BNO feedback and latched safety path. Carriage retains its step limits.
void rejectManual(const char *reason) {
    queueText("MANUAL REJECTED: "); queueText(reason); queueText("\n");
}
bool manualReady() {
    return commandIdle() && pitchReady && fresh(millis()) &&
        !motionWatchdog.tripped() && !commandWatchdog.tripped() &&
        !yawMotor->isRunning() && !pitchMotor->isRunning() && !carriageMotor->isRunning();
}
void beginManual() {
    if (!manualReady()) {
        rejectManual("completed startup, stopped motors and fresh valid BNO required"); return;
    }
    manualYaw = {}; manualPitch = {}; manualCarriage = {};
    resetPoseAxis(yawAxis, heading.continuous, 0);
    resetPoseAxis(pitchAxis, orientation.roll, 0);
    manualActive = true; manualEnding = false; finalPrinted = false;
    operation = Operation::MANUAL; phase = Phase::MANUAL;
    // JOG needs relative feedback, not calibrated north. POSE sets its own requirements.
    yawRequired = accuracyRequired = false; accuracyGrace = {};
    controlStartedAt = millis(); lastControlSample = bnoHealth.freshSamples;
    if (!motionWatchdog.arm(lastBnoGood) || !commandWatchdog.arm(millis())) {
        abortTest("Manual watchdog could not arm"); return;
    }
    safety();
    if (!finalPrinted) queueText("MANUAL READY: JOG yaw pitch carriage [-1000,1000] every 20 ms; STOP exits; X aborts\n");
}
bool executeManualCommand(char **tokens, unsigned count) {
    if (strcmp(tokens[0], "STATUS") == 0 && count == 1) {
        queueText(phase == Phase::ABORTED ? "M09 ABORTED\n" :
            (manualActive ? "M09 MANUAL\n" :
            (manualReady() ?
                "M09 READY\n" : "M09 BUSY\n")));
        return true;
    }
    if (strcmp(tokens[0], "STOP") == 0 && count == 1) {
        if (manualActive) {
            safety(); if (finalPrinted) return true;
            manualEnding = true;
            manualYaw.request = manualPitch.request = manualCarriage.request = 0;
            // Explicit STOP hands off to bounded, BNO-guarded braking. A lost
            // stream without STOP still trips independently and latches abort.
            commandWatchdog.disarm();
        }
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
    // Late data cannot renew an expired lease, even before the timer task runs.
    commandWatchdog.recordFresh(millis()); safety();
    if (!finalPrinted) {
        manualYaw.request = static_cast<int>(yaw);
        manualPitch.request = static_cast<int>(pitch);
        manualCarriage.request = static_cast<int>(carriage);
    }
    return true;
}
void serviceManualAxis(Axis &axis, ManualAxis &manual, uint32_t now) {
    axis.current = axis.pitch ? orientation.roll : heading.continuous;
    updateVelocity(axis, now);
    const int requestedDirection = manual.request == 0 ? 0 : (manual.request > 0 ? 1 : -1);
    if (manual.direction) {
        const double displacement = (axis.current - manual.start) * manual.direction;
        if (displacement < manual.best - 0.6) {
            abortTest(axis.pitch ? "Manual pitch wrong-direction/runaway guard" : "Manual yaw wrong-direction/runaway guard"); return;
        }
        manual.best = fmax(manual.best, displacement);
        if (displacement >= manual.progress + 0.15) { manual.progress = displacement; manual.progressAt = now; }
        const uint32_t timeout = manual.rate >= MIN_SPEED_HZ ? SLEW_PROGRESS_TIMEOUT_MS : PROGRESS_TIMEOUT_MS;
        if (!manual.braking && now - manual.progressAt >= timeout) {
            abortTest(axis.pitch ? "No measured manual pitch progress" : "No measured manual yaw progress"); return;
        }
    }
    if (manual.braking) {
        if (axis.motor->isRunning()) {
            if (now - manual.brakeAt >= BRAKING_TIMEOUT_MS) abortTest("Manual braking timeout");
            return;
        }
        manual.braking = false; manual.direction = 0; manual.rate = 0;
        manual.observing = true; manual.stoppedAt = now;
        return;
    }
    if (manual.observing) {
        if (now - manual.stoppedAt < PRECISION_OBSERVE_MS) return;
        manual.observing = false;
    }
    if (manual.direction && (!requestedDirection || requestedDirection != manual.direction)) {
        axis.motor->stopMove(); manual.braking = true; manual.brakeAt = now;
        return;
    }
    if (!requestedDirection) return;
    const double position = axis.pitch ? orientation.roll :
        heading.continuous - (referenceSet ? northTargetContinuous : pitchReadyYaw);
    const double limit = axis.pitch ? MAX_USABLE_PITCH_DEG : RELATIVE_LIMIT_DEG;
    if (limit - requestedDirection * position <= axis.brakeAtDeg) {
        abortTest(axis.pitch ? "Manual pitch stopping margin reached" : "Manual yaw stopping margin reached"); return;
    }
    const uint32_t rate = std::max<uint32_t>(1, static_cast<uint32_t>(
        lround(fabs(static_cast<double>(manual.request)) * slewSpeed(axis.pitch) / 1000.0)));
    if (manual.direction && !axis.motor->isRunning()) { abortTest("Manual continuous motor stopped unexpectedly"); return; }
    if (!manual.direction || rate != manual.rate) {
        safety(); if (finalPrinted) return;
        if (axis.motor->setAcceleration(slewAcceleration(axis.pitch)) != 0 || axis.motor->setSpeedInHz(rate) != 0) {
            abortTest("Manual FastAccelStepper configuration rejected"); return;
        }
        if (!manual.direction) {
            const int direction = stepDirection(manual.request, axis.pitch);
            if ((direction > 0 ? axis.motor->runForward() : axis.motor->runBackward()) != MOVE_OK) {
                abortTest("Manual FastAccelStepper motion rejected"); return;
            }
            manual.direction = requestedDirection; manual.start = axis.current;
            manual.best = manual.progress = 0; manual.progressAt = now;
        } else {
            axis.motor->applySpeedAcceleration();
        }
        manual.rate = rate;
        safety();
    }
}
void serviceManualCarriage(uint32_t now) {
    auto &manual = manualCarriage;
    const int direction = manual.request == 0 ? 0 : (manual.request > 0 ? 1 : -1);
    if (manual.braking || (manual.direction && !carriageMotor->isRunning())) {
        if (carriageMotor->isRunning()) {
            if (now - manual.brakeAt >= BRAKING_TIMEOUT_MS) abortTest("Manual carriage braking timeout");
            return;
        }
        if (!manual.braking && carriageMotor->getCurrentPosition() != manual.direction * CARRIAGE_LIMIT_STEPS) {
            abortTest("Manual carriage stopped before its boundary"); return;
        }
        manual.braking = false; manual.direction = 0; manual.rate = 0;
        manual.observing = true; manual.stoppedAt = now;
        return;
    }
    if (manual.observing) {
        if (now - manual.stoppedAt < PRECISION_OBSERVE_MS) return;
        manual.observing = false;
    }
    if (manual.direction && (!direction || direction != manual.direction)) {
        const double speed = fabs(carriageMotor->getCurrentSpeedInMilliHz() / 1000.0);
        const double stopSteps = speed * speed / (2.0 * CARRIAGE_ACCELERATION) + speed * 0.05 + 2;
        const double remaining = CARRIAGE_LIMIT_STEPS - manual.direction * carriageMotor->getCurrentPosition();
        // stopMove rewrites the finite target. Retain the endpoint ramp when
        // already near it; include queue/update allowance and step rounding.
        if (remaining > stopSteps) carriageMotor->stopMove();
        manual.braking = true; manual.brakeAt = now;
        return;
    }
    if (!direction || direction * carriageMotor->getCurrentPosition() >= CARRIAGE_LIMIT_STEPS) return;
    const uint32_t rate = std::max<uint32_t>(1, static_cast<uint32_t>(
        lround(fabs(static_cast<double>(manual.request)) * CARRIAGE_MAX_SPEED_HZ / 1000.0)));
    if (!manual.direction || rate != manual.rate) {
        safety(); if (finalPrinted) return;
        if (carriageMotor->setAcceleration(CARRIAGE_ACCELERATION) != 0 || carriageMotor->setSpeedInHz(rate) != 0) {
            abortTest("Manual carriage FastAccelStepper configuration rejected"); return;
        }
        if (!manual.direction) {
            // Velocity-controlled travel bounded by the existing +/-500-step envelope.
            // A finite target lets the motor's own ramp brake at the boundary.
            if (carriageMotor->moveTo(direction * CARRIAGE_LIMIT_STEPS) != MOVE_OK) {
                abortTest("Manual carriage FastAccelStepper motion rejected"); return;
            }
            manual.direction = direction;
        } else {
            carriageMotor->applySpeedAcceleration();
        }
        manual.rate = rate;
        safety();
    }
}
void serviceManual() {
    safety(); if (finalPrinted || !fresh(millis())) return;
    if (bnoHealth.freshSamples == lastControlSample) return;
    lastControlSample = bnoHealth.freshSamples;
    const uint32_t now = millis();
    serviceManualAxis(yawAxis, manualYaw, now); if (finalPrinted) return;
    serviceManualAxis(pitchAxis, manualPitch, now); if (finalPrinted) return;
    serviceManualCarriage(now); if (finalPrinted) return;
    if (!manualYaw.request && !manualPitch.request && !manualCarriage.request &&
        !yawMotor->isRunning() && !pitchMotor->isRunning() && !carriageMotor->isRunning() &&
        !manualYaw.braking && !manualPitch.braking && !manualCarriage.braking &&
        !manualYaw.observing && !manualPitch.observing && !manualCarriage.observing) {
        controlStartedAt = now; // The inherited 90 s deadline bounds continuous manual motion, not centered idle time.
        if (manualEnding) {
            motionWatchdog.disarm(); commandWatchdog.disarm();
            manualActive = manualEnding = false; finalPrinted = true; phase = Phase::COMPLETE;
            queueText("MANUAL STOPPED: motors stopped; rearm centered for manual or use POSE/MOVE\nM09 READY\n");
        }
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
