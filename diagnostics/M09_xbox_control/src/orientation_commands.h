// Admission and recovery for two explicit commands using the existing POSE
// controller. No sensor I/O belongs here; only the worker's accepted feedback.
void resetPoseAxis(Axis &axis, double target, double timingPeak);
bool checkSensorReset();
void safety();
constexpr uint32_t ALIGNMENT_RECOVERY_MS = 1500;
// Euler pitch is mathematically bounded to +/-90 degrees. Keep margin from its
// singular endpoint while allowing LEVEL to recover the physically observed 79-80 degrees.
constexpr double LEVEL_SENSOR_DOMAIN_LIMIT_DEG = 89.0;
// Begin foreground braking two watchdog ticks before its independent hard stop.
// A blocked foreground still gets the unchanged 150 ms watchdog protection.
constexpr uint32_t ALIGNMENT_PAUSE_AGE_MS = BNO_STALE_MS - 2 * MotionWatchdog::CHECK_INTERVAL_US / 1000;
uint32_t alignmentPauseAt = 0, alignmentStoppedAt = 0, alignmentPauseTotal = 0;
uint32_t alignmentDeadlineMs = LEG_TIMEOUT_MS;
bool alignmentStopped = false, alignmentTimingKnown = false;
double alignmentInitialError = 0;
double alignmentTimingAngle = 0, alignmentHeadingFirst = 0, alignmentYawReference = 0;
double alignmentPauseHeading = 0, alignmentPauseContinuous = 0;
int32_t alignmentTimingSteps = 0;
uint8_t pendingOrientation = 0; // 1=LEVEL, 2=NORTH; only during normal manual braking.

Axis &alignmentAxis() { return operation == Operation::LEVEL ? pitchAxis : yawAxis; }
uint32_t alignmentSpeed(bool pitch) { return pitch ? PITCH_TRAVEL_SPEED_HZ : YAW_SLEW_SPEED_HZ; }
int32_t alignmentAcceleration(bool pitch) { return pitch ? PITCH_TRAVEL_ACCELERATION : YAW_SLEW_ACCELERATION; }

// M09 has no configured mechanical steps/degree. These values come from the
// existing learned BNO response, not the 24/16 correction gains. Gear reduction
// and driver microstepping are both included in measured motor pulses/output deg.
double alignmentSeconds(bool pitch, double error, double pulsesPerDegree) {
    const double speed = alignmentSpeed(pitch), acceleration = alignmentAcceleration(pitch);
    double seconds = milestone8::finiteSeconds(error * pulsesPerDegree, speed, acceleration);
    // Conservative precision tail: braking margin at full angular speed, using
    // the same finite burst profile and observation delay as the real controller.
    double remaining = fmin(error, brakingThreshold(pitch, speed / pulsesPerDegree) + SLEW_ENTRY_HYSTERESIS_DEG);
    for (unsigned n = 0; remaining > TOLERANCE_DEG; ++n) {
        if (n == milestone8::MAX_PREDICTED_CORRECTIONS) return NAN;
        const double pulses = fabs(static_cast<double>(correctionSteps(remaining, true, pitch)));
        seconds += milestone8::finiteSeconds(pulses, correctionSpeed(remaining, true), ACCELERATION) +
            PRECISION_OBSERVE_MS / 1000.0;
        const double next = fabs(remaining - pulses / pulsesPerDegree);
        if (!isfinite(next) || next >= remaining) return NAN;
        remaining = next;
    }
    return seconds + SETTLE_MS / 1000.0;
}
void alignmentDeadline(bool pitch) {
    if (!isfinite(alignmentPulsesPerDegree) || alignmentPulsesPerDegree <= 0) return;
    const double seconds = alignmentSeconds(pitch, alignmentInitialError, alignmentPulsesPerDegree);
    if (!isfinite(seconds)) return; // Real progress checks still bound uncertain estimates.
    const double duration = ceil(2000 * seconds + 10000) + alignmentPauseTotal;
    if (duration >= 0x7fffffffUL) { abortAlignment("alignment timing estimate out of range"); return; }
    const uint32_t estimate = static_cast<uint32_t>(duration);
    if (!alignmentTimingKnown || estimate > alignmentDeadlineMs) {
        alignmentTimingKnown = true;
        alignmentDeadlineMs = std::max<uint32_t>(estimate, millis() - controlStartedAt + SETTLE_MS + 1000);
        char line[180];
        snprintf(line, sizeof(line), "%s DEADLINE deadline_ms=%lu pulses_per_deg=%.3f\n",
            alignmentName(), static_cast<unsigned long>(alignmentDeadlineMs), alignmentPulsesPerDegree);
        queueText(line);
    }
}
void updateAlignmentTiming(Axis &axis) {
    const double angle = axis.current - alignmentTimingAngle;
    const double pulses = static_cast<double>(axis.motor->getCurrentPosition()) - alignmentTimingSteps;
    const int sign = bnoFeedbackStepSign(axis);
    if (fabs(angle) < DIRECTION_RESPONSE_DEG || fabs(pulses) < 8 || pulses * angle * sign <= 0) return;
    const double estimate = fabs(pulses / angle);
    // Replan only when necessary; do not integrate thousands of hypothetical
    // bursts on each sensor sample or continually move a no-progress deadline.
    if (alignmentPulsesPerDegree <= 0 || estimate > alignmentPulsesPerDegree * 1.1) {
        alignmentPulsesPerDegree = estimate;
        alignmentDeadline(axis.pitch);
    }
}
uint32_t alignmentProgressTimeout(const Axis &axis) {
    const uint32_t existing = axis.motion == Motion::SLEW ? SLEW_PROGRESS_TIMEOUT_MS : PROGRESS_TIMEOUT_MS;
    if (alignmentPulsesPerDegree <= 0) return existing;
    const double pulses = 0.15 * alignmentPulsesPerDegree;
    double seconds;
    if (axis.motion == Motion::SLEW) {
        seconds = milestone8::finiteSeconds(pulses, alignmentSpeed(axis.pitch), alignmentAcceleration(axis.pitch));
    } else {
        const double burst = fmax(2.0, fabs(static_cast<double>(correctionSteps(axis.error, true, axis.pitch))));
        seconds = ceil(pulses / burst) * (milestone8::finiteSeconds(burst,
            correctionSpeed(axis.error, true), ACCELERATION) + PRECISION_OBSERVE_MS / 1000.0);
    }
    return isfinite(seconds) && seconds < 700000 ?
        std::max(existing, static_cast<uint32_t>(ceil(3000 * seconds + 1000))) : existing;
}
void abortAlignment(const char *reason) {
    if (poseStopping) return;
    queueText(alignmentName()); queueText(" FAILED: "); queueText(reason); queueText("\n");
    beginPoseStop(reason);
}
void pauseAlignment(const char *reason) {
    if (!alignmentActive() || poseStopping || alignmentPaused) return;
    alignmentPaused = true; alignmentPauseAt = millis(); alignmentStopped = false;
    alignmentPauseHeading = orientation.heading;
    // current/first were last updated before invalidateOrientation cleared the tracker.
    alignmentPauseContinuous = yawAxis.current;
    if (motionWatchdog.tripped()) invalidateKeyframes("alignment feedback watchdog forced stop");
    Axis &axis = alignmentAxis();
    axis.finiteActive = axis.observing = false;
    if (axis.motor->isRunning()) axis.motor->stopMove();
    motionWatchdog.disarm();
    settleSamples = 0; poseStoppedObserved = false;
    queueText(alignmentName()); queueText(" PAUSED: "); queueText(reason);
    queueText("; braking; feedback recovery grace_ms=1500\n");
}
bool alignmentSafety() {
    if (!alignmentPaused && (!fresh(millis()) || motionWatchdog.tripped() ||
        millis() - lastBnoGood >= ALIGNMENT_PAUSE_AGE_MS)) pauseAlignment("BNO feedback delayed or unavailable");
    if (alignmentPaused) {
        const uint32_t now = millis();
        if (now - alignmentPauseAt >= ALIGNMENT_RECOVERY_MS) {
            abortAlignment("BNO feedback did not recover within 1500 ms"); return false;
        }
        if (!poseMotorsStopped()) return false;
        if (!alignmentStopped) { alignmentStopped = true; alignmentStoppedAt = now; }
        if (!fresh(now) || !sampleAtOrAfter(lastBnoGood, alignmentStoppedAt + PRECISION_OBSERVE_MS) ||
            !sampleAtOrAfter(lastBnoGood, alignmentPauseAt + 1)) return false;
        if (!motionWatchdog.clearTripWhenStopped()) return false;
        // Keep the travel reference through a sensor tracker reset, but target
        // magnetic heading zero again in its newly received reported solution.
        heading.first = alignmentHeadingFirst;
        heading.continuous = alignmentPauseContinuous + shortestDifference(orientation.heading, alignmentPauseHeading);
        heading.previous = orientation.heading; heading.initialized = true;
        if (!referenceSet) pitchReadyYaw = alignmentYawReference;
        resetPoseAxis(yawAxis, operation == Operation::NORTH ?
            heading.continuous + shortestDifference(0, orientation.heading) : heading.continuous, 0);
        resetPoseAxis(pitchAxis, operation == Operation::LEVEL ? 0 : physicalPitch(), 0);
        if (operation == Operation::LEVEL) yawAxis.motion = Motion::HOLD;
        else pitchAxis.motion = Motion::HOLD;
        alignmentTimingAngle = alignmentAxis().current;
        alignmentTimingSteps = alignmentAxis().motor->getCurrentPosition();
        alignmentPauseTotal += now - alignmentPauseAt;
        alignmentDeadlineMs += now - alignmentPauseAt;
        if (!motionWatchdog.arm(lastBnoGood)) return false;
        alignmentPaused = false; phase = Phase::MOVING;
        lastControlSample = bnoHealth.freshSamples; recordTimingStarts();
        char line[140];
        snprintf(line, sizeof(line), "%s RESUMED deadline_ms=%lu\n", alignmentName(),
            static_cast<unsigned long>(alignmentDeadlineMs)); queueText(line);
    }
    if (millis() - controlStartedAt >= alignmentDeadlineMs) {
        abortAlignment("alignment displacement/response deadline expired"); return false;
    }
    if (operation == Operation::LEVEL && bnoValid && fabs(physicalPitch()) >= LEVEL_SENSOR_DOMAIN_LIMIT_DEG) {
        abortAlignment("physical pitch reached BNO Euler limit margin (+/-89 degrees)"); return false;
    }
    return !poseStopping;
}
void rejectOrientation(const char *command, const char *reason) {
    queueText(command); queueText(" REJECTED: "); queueText(reason); queueText("\n");
}
void beginOrientation(bool level) {
    const char *name = level ? "LEVEL" : "NORTH";
    checkSensorReset();
    if (!commandIdle() || !controlReady || !poseMotorsStopped()) {
        rejectOrientation(name, "stopped READY required; busy or abort latched"); return;
    }
    if (!bnoInitialized || !reportEnabled) { rejectOrientation(name, "BNO unavailable or rotation report disabled"); return; }
    if (alignmentSampleInvalid) { rejectOrientation(name, "invalid orientation sample; wait for fresh valid BNO data"); return; }
    if (!fresh(millis())) { rejectOrientation(name, "BNO orientation unavailable or stale"); return; }
    if (!isfinite(physicalPitch())) {
        rejectOrientation(name, "physical pitch orientation invalid"); return;
    }
    if (!level) {
        const char *reason = northUnavailableReason(millis());
        if (strcmp(reason, "usable") != 0) {
            char detail[150]; snprintf(detail, sizeof(detail), "magnetic heading unavailable: %s", reason);
            rejectOrientation(name, detail); return;
        }
    }
    if (level && fabs(physicalPitch()) >= LEVEL_SENSOR_DOMAIN_LIMIT_DEG) {
        rejectOrientation(name, "physical pitch at BNO Euler limit margin (+/-89 degrees)"); return;
    }
    if (!bnoWatchdogReady || !motionWatchdog.clearTripWhenStopped()) {
        rejectOrientation(name, "orientation watchdog unavailable"); return;
    }
    // Preserve the shortest heading move without a fixed yaw travel envelope.
    const double target = level ? heading.continuous : heading.continuous + shortestDifference(0, orientation.heading);
    const double reference = referenceSet ? northTargetContinuous : pitchReadyYaw;
    const double currentOffset = heading.continuous - reference;
    const double targetOffset = target - reference;
    operation = level ? Operation::LEVEL : Operation::NORTH;
    poseActive = poseNeedsBno = true; poseStopping = alignmentPaused = false;
    finalPrinted = false; phase = Phase::MOVING; window.active = false;
    posePrecisionOnly = false; poseStoppedObserved = false;
    resetPoseAxis(yawAxis, level ? heading.continuous : target, 0);
    resetPoseAxis(pitchAxis, level ? 0 : physicalPitch(), 0);
    yawRequired = !level; accuracyRequired = false; accuracyGrace = {};
    if (level) yawAxis.motion = Motion::HOLD; else pitchAxis.motion = Motion::HOLD;
    carriageTarget = carriageMotor->getCurrentPosition(); carriagePending = false;
    yawRateCap = YAW_SLEW_SPEED_HZ; pitchRateCap = PITCH_TRAVEL_SPEED_HZ;
    settleSamples = 0; sawYawMotion = sawPitchMotion = concurrentMotion = concurrentThreeMotion = false;
    plannedDurationSeconds = 0; timingLimited = false;
    controlStartedAt = millis(); lastControlSample = bnoHealth.freshSamples;
    alignmentPauseTotal = 0; alignmentDeadlineMs = LEG_TIMEOUT_MS; alignmentTimingKnown = false;
    alignmentPulsesPerDegree = level ? pitchPulsesPerDegree : yawPulsesPerDegree;
    alignmentInitialError = fabs(alignmentAxis().error);
    alignmentTimingAngle = alignmentAxis().current; alignmentTimingSteps = alignmentAxis().motor->getCurrentPosition();
    alignmentHeadingFirst = heading.first; alignmentYawReference = reference;
    recordTimingStarts(); alignmentDeadline(level);
    if (poseStopping) return;
    if (!motionWatchdog.arm(lastBnoGood)) { abortAlignment("orientation watchdog arm failed"); return; }
    char line[360];
    snprintf(line, sizeof(line), "%s ACCEPTED target_deg=0 reference=%s error_deg=%.3f current_offset_deg=%.3f target_offset_deg=%.3f tolerance_deg=%.3f cap_hz=%lu accel=%ld deadline_ms=%lu pulses_per_deg=%.3f\n",
        name, level ? "GRAVITY_EULER_PITCH" : "MAGNETIC_NORTH", alignmentAxis().error,
        currentOffset, targetOffset, TOLERANCE_DEG,
        static_cast<unsigned long>(alignmentSpeed(level)), static_cast<long>(alignmentAcceleration(level)),
        static_cast<unsigned long>(alignmentDeadlineMs), alignmentPulsesPerDegree);
    queueText(line); safety();
}
bool executeOrientationCommand(char **tokens, unsigned count) {
    const bool level = strcmp(tokens[0], "LEVEL") == 0;
    if (!level && strcmp(tokens[0], "NORTH") != 0) return false;
    if (count != 1) rejectOrientation(tokens[0], "use LEVEL or NORTH with no arguments");
    else if (manualActive) {
        if (manualEnding || pendingOrientation) rejectOrientation(tokens[0], "manual handoff already stopping");
        else {
            pendingOrientation = level ? 1 : 2;
            manualEnding = true;
            manualYaw.request = manualPitch.request = manualCarriage.request = 0;
            queueText(level ? "LEVEL HANDOFF: braking manual motion\n" :
                              "NORTH HANDOFF: braking manual motion\n");
        }
    }
    else beginOrientation(level);
    return true;
}
void cancelPendingOrientation() { pendingOrientation = 0; }
bool startPendingOrientation() {
    const uint8_t requested = pendingOrientation;
    pendingOrientation = 0;
    if (!requested) return false;
    beginOrientation(requested == 1);
    return poseActive;
}
