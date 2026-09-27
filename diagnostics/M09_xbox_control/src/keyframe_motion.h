// Included inside main.cpp's private namespace. This independent finite-step
// operation never calls the POSE angle controller or sensor objects.
struct KeyframeOperation {
    bool stopping = false, failed = false, latched = false, invalidated = false;
    uint32_t id = 0, started = 0, durationMs = 0, stopStarted = 0, lastDisplay = 0;
    uint32_t endedAt[3] = {};
    bool ended[3] = {};
    unsigned concurrentAxes = 0;
    m09::keyframes::AxisPlan axes[3];
    const char *reason = "generated-step targets reached";
} keyframe;
uint32_t keyframeEpoch = 1;

FastAccelStepper *keyframeMotor(unsigned axis) {
    return axis == 0 ? yawMotor : (axis == 1 ? pitchMotor : carriageMotor);
}
int32_t keyframeAcceleration(unsigned axis) {
    return axis == 0 ? YAW_SLEW_ACCELERATION :
        (axis == 1 ? PITCH_SLEW_ACCELERATION : CARRIAGE_ACCELERATION);
}
uint32_t keyframeSpeed(unsigned axis) {
    return axis == 0 ? YAW_SLEW_SPEED_HZ :
        (axis == 1 ? PITCH_SLEW_SPEED_HZ : CARRIAGE_MAX_SPEED_HZ);
}
void invalidateKeyframes(const char *reason) {
    if (++keyframeEpoch == 0) ++keyframeEpoch;
    char line[250];
    snprintf(line, sizeof(line), "KEYFRAME_INVALIDATED epoch=%lu reason=%s\n",
        static_cast<unsigned long>(keyframeEpoch), reason);
    queueText(line);
}
void rejectKeyframe(const char *command, uint32_t id, const char *reason) {
    char line[280];
    snprintf(line, sizeof(line), "%s REJECTED id=%lu reason=%s\n", command,
        static_cast<unsigned long>(id), reason);
    queueText(line);
}
void snapshotKeyframe(uint32_t id) {
    checkSensorReset();
    if (!controlReady || !commandIdle() || !poseMotorsStopped()) {
        rejectKeyframe("SNAP", id, "READY and all motors stopped required"); return;
    }
    const uint32_t now = millis();
    char line[450];
    snprintf(line, sizeof(line),
        "KEYFRAME_SNAPSHOT id=%lu epoch=%lu yaw_steps=%ld pitch_steps=%ld carriage_steps=%ld "
        "heading=%.3f physical_pitch=%.3f bno_valid=%s bno_age_ms=%lu accuracy=%u north_usable=%s unhomed=YES\n",
        static_cast<unsigned long>(id), static_cast<unsigned long>(keyframeEpoch),
        static_cast<long>(yawMotor->getCurrentPosition()), static_cast<long>(pitchMotor->getCurrentPosition()),
        static_cast<long>(carriageMotor->getCurrentPosition()), orientation.heading, physicalPitch(),
        fresh(now) ? "YES" : "NO", static_cast<unsigned long>(now - lastBnoGood), bnoAccuracy,
        fresh(now) && northUsable ? "YES" : "NO");
    queueText(line);
}

void stopKeyframe(const char *reason, bool failure, bool latch) {
    if (!keyframeActive) return;
    if (keyframe.stopping && !failure && !latch) return;
    keyframe.stopping = true; keyframe.failed |= failure; keyframe.latched |= latch;
    keyframe.reason = reason; keyframe.stopStarted = millis();
    bool forced = failure || latch;
    // Playback can use very low acceleration for its selected duration. STOP
    // uses the existing native braking acceleration, never a ten-second ramp.
    if (!forced) {
        for (unsigned axis = 0; axis < 3; ++axis) {
            auto *motor = keyframeMotor(axis);
            if (motor->isRunning()) {
                if (motor->setAcceleration(keyframeAcceleration(axis)) != 0) { forced = true; break; }
                motor->applySpeedAcceleration(); motor->stopMove();
            }
        }
    }
    if (forced) {
        keyframe.failed = true;
        for (unsigned axis = 0; axis < 3; ++axis) keyframeMotor(axis)->forceStop();
        if (!keyframe.invalidated) { invalidateKeyframes(reason); keyframe.invalidated = true; }
    }
    // Independent stale watchdog is no longer needed once every axis has
    // received braking/forceStop. Cancellation itself needs no sensor progress.
    motionWatchdog.disarm();
    char line[280];
    snprintf(line, sizeof(line), "KEYMOVE STOPPING id=%lu reason=%s\n",
        static_cast<unsigned long>(keyframe.id), reason);
    queueText(line);
}

void keyframeSafety() {
    if (!keyframeActive || keyframe.stopping) return;
    const uint32_t now = millis();
    motionWatchdog.check(now);
    if (motionWatchdog.tripped() || !fresh(now))
        stopKeyframe("BNO feedback stale or unavailable", true, false);
    else if (fabs(physicalPitch()) >= MAX_USABLE_PITCH_DEG)
        stopKeyframe("measured pitch reached +/-75 degree guard", true, false);
    else if (fabs(heading.continuous - (referenceSet ? northTargetContinuous : pitchReadyYaw)) >= RELATIVE_LIMIT_DEG)
        stopKeyframe("measured yaw travel guard exceeded", true, false);
    else if (now - keyframe.started > keyframe.durationMs + std::max<uint32_t>(2000, keyframe.durationMs / 5))
        stopKeyframe("requested duration deadline exceeded", true, false);
}

void serviceKeyframe() {
    if (!keyframeActive) return;
    keyframeSafety();
    const uint32_t now = millis();
    unsigned running = 0;
    bool unexpectedStop = false, earlyArrival = false;
    for (unsigned axis = 0; axis < 3; ++axis) {
        const bool moving = keyframeMotor(axis)->isRunning();
        if (moving) ++running;
        else if (keyframe.axes[axis].moving && !keyframe.ended[axis]) {
            keyframe.ended[axis] = true; keyframe.endedAt[axis] = now - keyframe.started;
            unexpectedStop |= keyframeMotor(axis)->getCurrentPosition() != keyframe.axes[axis].target;
            earlyArrival |= now - keyframe.started + std::max<uint32_t>(500, keyframe.durationMs / 10) < keyframe.durationMs;
        }
    }
    keyframe.concurrentAxes = std::max(keyframe.concurrentAxes, running);
    if (!keyframe.stopping && (unexpectedStop || earlyArrival)) {
        stopKeyframe(unexpectedStop ? "motor stopped short of generated-step target" :
            "motor arrived substantially before requested duration", true, false); return;
    }
    if (keyframe.stopping && running && now - keyframe.stopStarted >= BRAKING_TIMEOUT_MS) {
        for (unsigned axis = 0; axis < 3; ++axis) keyframeMotor(axis)->forceStop();
        keyframe.failed = true; keyframe.reason = "braking timeout; forced stop";
        if (!keyframe.invalidated) { invalidateKeyframes(keyframe.reason); keyframe.invalidated = true; }
        return;
    }
    if (!running) {
        bool targets = true;
        for (unsigned axis = 0; axis < 3; ++axis)
            targets &= keyframeMotor(axis)->getCurrentPosition() == keyframe.axes[axis].target;
        if (!keyframe.stopping && !targets) {
            stopKeyframe("motor stopped short of generated-step target", true, false); return;
        }
        if (!keyframe.stopping && now - keyframe.started > keyframe.durationMs + std::max<uint32_t>(500, keyframe.durationMs / 10)) {
            stopKeyframe("actual elapsed time exceeded requested duration tolerance", true, false); return;
        }
        motionWatchdog.disarm();
        if (!motionWatchdog.clearTripWhenStopped()) return;
        keyframeActive = false; finalPrinted = true;
        phase = keyframe.latched ? Phase::ABORTED : Phase::COMPLETE;
        char line[650];
        snprintf(line, sizeof(line),
            "KEYMOVE RESULT id=%lu status=%s epoch=%lu yaw_steps=%ld pitch_steps=%ld carriage_steps=%ld "
            "duration_ms=%lu elapsed_ms=%lu yaw_end_ms=%lu pitch_end_ms=%lu carriage_end_ms=%lu concurrent_axes=%u "
            "basis=GENERATED_STEPS angle_settling=NOT_CHECKED reason=%s\n%s\n",
            static_cast<unsigned long>(keyframe.id), keyframe.failed ? "FAILED" : (keyframe.stopping ? "STOPPED" : "PASS"),
            static_cast<unsigned long>(keyframeEpoch), static_cast<long>(yawMotor->getCurrentPosition()),
            static_cast<long>(pitchMotor->getCurrentPosition()), static_cast<long>(carriageMotor->getCurrentPosition()),
            static_cast<unsigned long>(keyframe.durationMs), static_cast<unsigned long>(now - keyframe.started),
            static_cast<unsigned long>(keyframe.endedAt[0]), static_cast<unsigned long>(keyframe.endedAt[1]),
            static_cast<unsigned long>(keyframe.endedAt[2]), keyframe.concurrentAxes, keyframe.reason,
            keyframe.latched ? "Latched abort; reset required. X/x aborts without automatic retry." : "M09 READY");
        queueText(line); return;
    }
    if (now - keyframe.lastDisplay >= 250) {
        keyframe.lastDisplay = now;
        char line[300];
        snprintf(line, sizeof(line), "KEYMOVE_STATE id=%lu phase=%s elapsed_ms=%lu yaw_steps=%ld pitch_steps=%ld carriage_steps=%ld running_axes=%u\n",
            static_cast<unsigned long>(keyframe.id), keyframe.stopping ? "STOPPING" : "MOVING",
            static_cast<unsigned long>(now - keyframe.started), static_cast<long>(yawMotor->getCurrentPosition()),
            static_cast<long>(pitchMotor->getCurrentPosition()), static_cast<long>(carriageMotor->getCurrentPosition()), running);
        queueText(line);
    }
}

void beginKeyframe(uint32_t id, uint32_t epoch, const int32_t targets[3], uint32_t durationMs) {
    checkSensorReset();
    if (!commandIdle() || !controlReady || !poseMotorsStopped()) {
        rejectKeyframe("KEYMOVE", id, "READY and all motors stopped required"); return;
    }
    if (epoch != keyframeEpoch) { rejectKeyframe("KEYMOVE", id, "capture epoch invalid; recapture A and B"); return; }
    if (!pitchReady || !fresh(millis()) || !bnoWatchdogReady || !motionWatchdog.clearTripWhenStopped()) {
        rejectKeyframe("KEYMOVE", id, "fresh BNO orientation and intact pitch baseline required"); return;
    }
    if (fabs(physicalPitch()) >= MAX_USABLE_PITCH_DEG ||
        fabs(heading.continuous - (referenceSet ? northTargetContinuous : pitchReadyYaw)) >= RELATIVE_LIMIT_DEG) {
        rejectKeyframe("KEYMOVE", id, "current orientation outside existing travel guards"); return;
    }
    m09::keyframes::AxisPlan plans[3];
    for (unsigned axis = 0; axis < 3; ++axis) {
        if (!m09::keyframes::planAxis(keyframeMotor(axis)->getCurrentPosition(), targets[axis], durationMs,
                keyframeSpeed(axis), keyframeAcceleration(axis), plans[axis])) {
            rejectKeyframe("KEYMOVE", id, "duration/displacement infeasible for finite speed, acceleration or step resolution"); return;
        }
    }
    // Validate all three profiles before configuring any motor, then configure
    // every participant before the first nonblocking start.
    for (unsigned axis = 0; axis < 3; ++axis) {
        if (!plans[axis].moving) continue;
        auto *motor = keyframeMotor(axis);
        if (motor->setAcceleration(plans[axis].acceleration) != 0 || motor->setSpeedInMilliHz(plans[axis].speedMilliHz) != 0) {
            invalidateKeyframes("keyframe motor configuration failed");
            rejectKeyframe("KEYMOVE", id, "motor configuration rejected before start"); return;
        }
    }
    if (!motionWatchdog.arm(lastBnoGood)) { rejectKeyframe("KEYMOVE", id, "BNO watchdog could not arm"); return; }
    keyframe = {}; keyframe.id = id; keyframe.durationMs = durationMs;
    for (unsigned axis = 0; axis < 3; ++axis) keyframe.axes[axis] = plans[axis];
    keyframe.started = millis(); keyframeActive = true;
    finalPrinted = false; phase = Phase::MOVING; window.active = false;
    yawRequired = accuracyRequired = false;
    char line[420];
    snprintf(line, sizeof(line), "KEYMOVE ACCEPTED id=%lu epoch=%lu duration_ms=%lu basis=GENERATED_STEPS unhomed=YES speed_millihz=%lu/%lu/%lu acceleration=%ld/%ld/%ld\n",
        static_cast<unsigned long>(id), static_cast<unsigned long>(keyframeEpoch), static_cast<unsigned long>(durationMs),
        static_cast<unsigned long>(plans[0].speedMilliHz), static_cast<unsigned long>(plans[1].speedMilliHz),
        static_cast<unsigned long>(plans[2].speedMilliHz), static_cast<long>(plans[0].acceleration),
        static_cast<long>(plans[1].acceleration), static_cast<long>(plans[2].acceleration));
    queueText(line);
    for (unsigned axis = 0; axis < 3; ++axis) {
        if (plans[axis].moving && keyframeMotor(axis)->moveTo(plans[axis].target) != MOVE_OK) {
            stopKeyframe("motor start rejected; all axes stopped", true, false); return;
        }
    }
    serviceKeyframe();
}

bool parseKeyframeInteger(const char *token, int64_t minimum, int64_t maximum, int64_t &value) {
    if (!token || !*token) return false;
    char *end = nullptr; errno = 0; value = strtoll(token, &end, 10);
    return end != token && !*end && errno != ERANGE && value >= minimum && value <= maximum;
}
bool executeKeyframeCommand(char **tokens, unsigned count) {
    const bool snapshot = strcmp(tokens[0], "SNAP") == 0;
    if (!snapshot && strcmp(tokens[0], "KEYMOVE") != 0) return false;
    int64_t id = 0;
    if (count < 2 || !parseKeyframeInteger(tokens[1], 1, UINT32_MAX, id)) {
        rejectKeyframe(tokens[0], 0, "positive uint32 request id required"); return true;
    }
    if (snapshot) {
        if (count != 2) rejectKeyframe("SNAP", id, "use SNAP request_id");
        else snapshotKeyframe(static_cast<uint32_t>(id));
        return true;
    }
    int64_t epoch = 0, duration = 0, positions[3] = {};
    bool valid = count == 7;
    if (valid) valid = parseKeyframeInteger(tokens[2], 1, UINT32_MAX, epoch) &&
        parseKeyframeInteger(tokens[6], m09::keyframes::MIN_DURATION_MS, m09::keyframes::MAX_DURATION_MS, duration);
    for (unsigned axis = 0; valid && axis < 3; ++axis)
        valid = parseKeyframeInteger(tokens[3 + axis], INT32_MIN, INT32_MAX, positions[axis]);
    if (!valid) { rejectKeyframe("KEYMOVE", id, "use KEYMOVE id epoch yaw_steps pitch_steps carriage_steps duration_ms (1000..60000)"); return true; }
    const int32_t targets[3] = {static_cast<int32_t>(positions[0]), static_cast<int32_t>(positions[1]), static_cast<int32_t>(positions[2])};
    beginKeyframe(static_cast<uint32_t>(id), static_cast<uint32_t>(epoch), targets, static_cast<uint32_t>(duration));
    return true;
}
