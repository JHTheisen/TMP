// Host-computed BNO-frame targets. GOTO uses the existing POSE controller;
// TRACK uses configured continuous rates through the same pulse service.
// No astronomy, sensor I/O or motor sign changes.
constexpr uint32_t CELESTIAL_LEASE_MS = 3000;
constexpr uint32_t CELESTIAL_MIN_UPDATE_MS = 100;
constexpr double CELESTIAL_MAX_TARGET_RATE_DPS = 1.0;
uint32_t celestialSequence = 0, celestialUpdatedAt = 0, celestialDeadlineMs = LEG_TIMEOUT_MS;
double celestialInitialErrors[2] = {}, celestialTimingAngles[2] = {};
double celestialPulsesPerDegree[2] = {};
int32_t celestialTimingSteps[2] = {};
bool celestialTimingKnown = false;
char celestialStopReason[240] = {};

// TRACK alone uses trajectory velocity plus a slow residual servo. Mechanical
// Provisional powered calibration; see CELESTIAL_TRACKING.md for evidence.
// These are STEP pulses/cradle degree, never positional gains or encoder scales.
constexpr double YAW_TRACK_PULSES_PER_DEG = 52.0;
constexpr double PITCH_TRACK_PULSES_PER_DEG = 67.2;
constexpr double CELESTIAL_RATE_FILTER_S = 2.0;
constexpr double CELESTIAL_RESIDUAL_FILTER_S = 10.0;
constexpr double CELESTIAL_RESIDUAL_GAIN = 0.02; // 1/s, outside the existing 0.1 deg noise band
constexpr double CELESTIAL_RESIDUAL_MAX_DPS = 0.01;
constexpr double CELESTIAL_RESIDUAL_RAMP_DPS2 = 0.002;
constexpr uint32_t CELESTIAL_RATE_SERVICE_MS = 100;
constexpr uint32_t CELESTIAL_TRACK_PROGRESS_MS = 60000;
constexpr uint32_t CELESTIAL_MIN_RATE_MILLIHZ = 5; // ESP32 FAS 16 MHz / uint32 tick interval
struct CelestialTrackAxis {
    double conversion = 0, filteredError = 0, correction = 0;
    double progressPosition = 0, commandedTravel = 0;
    uint32_t servicedAt = 0, progressAt = 0, rateMilliHz = 0;
    uint32_t brakeTimeoutMs = BRAKING_TIMEOUT_MS, launchAt = 0;
    bool started = false;
    bool runConfirmed = false, startRetried = false;
    bool zeroStopping = false;
};
CelestialTrackAxis celestialTrackAxes[2];
double celestialTargetRate[2] = {};
bool celestialRateKnown = false;
uint32_t celestialRateLoggedAt = 0;
void celestialRateTelemetry();

// A celestial session starts in the established BNO frame.  While that sensor
// is trustworthy, movement teaches the signed conversion from each wrapped
// AS5600 to its corresponding cradle axis (Bus A yaw, Bus B pitch). Acquisition
// can use BNO; TRACK always latches the encoder frame, never BNO fast feedback.
constexpr uint32_t CELESTIAL_ENCODER_STALE_MS = 150;
constexpr double CELESTIAL_ENCODER_TICK_DEG = 360.0 / 4096.0;
struct CelestialEncoderAxis {
    bool initialized = false, scaleKnown = false, calibrationAnchored = false;
    uint16_t raw = 0;
    int64_t ticks = 0, referenceTicks = 0, calibrationTicks = 0;
    int32_t calibrationSteps = 0;
    uint32_t lastGoodMs = 0;
    double referenceAngle = 0, calibrationAngle = 0;
    double degreesPerTick = 0, bestCalibrationTravel = 0;
};
CelestialEncoderAxis celestialEncoderAxes[2];
double celestialLearnedEncoderScale[2] = {};
uint32_t celestialCalibrationStoppedAt = 0;
bool celestialEncoderMode = false, celestialEncoderPaused = false;
bool celestialBnoHealthy = true, celestialBnoRecoveryReported = false;
uint32_t celestialEncoderGeneration = 0, celestialEncoderServicedGeneration = 0;
uint32_t celestialEncoderPauseAt = 0;
double celestialRecoveryYawDiscrepancy = 0, celestialRecoveryPitchDiscrepancy = 0;

int celestialEncoderDelta(uint16_t current, uint16_t previous) {
    int delta = static_cast<int>(current) - static_cast<int>(previous);
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    return delta;
}
bool observeCelestialEncoder(unsigned index) {
    const auto &sample = encoders.state(index);
    auto &axis = celestialEncoderAxes[index];
    if (!sample.hasSample || !sample.valid || !sample.magnetGood()) return false;
    if (!axis.initialized) {
        axis.initialized = true; axis.raw = sample.raw; axis.lastGoodMs = sample.lastGoodMs;
        axis.calibrationSteps = (index ? pitchMotor : yawMotor)->getCurrentPosition();
        axis.calibrationTicks = axis.ticks;
        ++celestialEncoderGeneration;
        return true;
    }
    if (sample.lastGoodMs == axis.lastGoodMs) return true;
    axis.ticks += celestialEncoderDelta(sample.raw, axis.raw);
    axis.raw = sample.raw; axis.lastGoodMs = sample.lastGoodMs;
    ++celestialEncoderGeneration;
    return true;
}
bool celestialEncoderAxisReady(unsigned index, uint32_t now) {
    const auto &state = encoders.state(index);
    const auto &axis = celestialEncoderAxes[index];
    return axis.initialized && axis.scaleKnown && state.valid && state.hasSample && state.magnetGood() &&
        state.ageMs(now) < CELESTIAL_ENCODER_STALE_MS;
}
bool celestialEncoderFeedbackAvailable(uint32_t now) {
    return celestialEncoderAxisReady(0, now) && celestialEncoderAxisReady(1, now);
}
void learnStoppedCelestialEncoderScales() {
    if (!yawMotor || !pitchMotor) return;
    const uint32_t now = millis();
    for (unsigned index = 0; index < 2; ++index) {
        const auto &sample = encoders.state(index);
        auto &axis = celestialEncoderAxes[index];
        if (!sample.valid || !sample.magnetGood() ||
            now - axis.lastGoodMs >= CELESTIAL_ENCODER_STALE_MS) axis.calibrationAnchored = false;
    }
    observeCelestialEncoder(0); observeCelestialEncoder(1);
    if (celestialActive()) return; // Never change a tracking frame or scale.
    if (!poseMotorsStopped()) { celestialCalibrationStoppedAt = now; return; }
    if (now - celestialCalibrationStoppedAt < CELESTIAL_ENCODER_STALE_MS) return;
    // Learn shaft-to-axis geometry from ordinary, completed motor movement.
    // The existing powered STEP/cradle-degree calibration and signs stay fixed.
    // Two stopped endpoints avoid pairing a delayed encoder sample with a
    // moving motor count. No calibration move is commanded here.
    for (unsigned index = 0; index < 2; ++index) {
        auto &encoder = celestialEncoderAxes[index];
        const auto &sample = encoders.state(index);
        if (!encoder.initialized || !sample.valid || !sample.magnetGood() ||
                sample.ageMs(now) >= CELESTIAL_ENCODER_STALE_MS) continue;
        Axis &axis = index ? pitchAxis : yawAxis;
        if (!encoder.calibrationAnchored) {
            encoder.calibrationSteps = axis.motor->getCurrentPosition();
            encoder.calibrationTicks = encoder.ticks;
            encoder.calibrationAnchored = true;
            continue;
        }
        if (celestialLearnedEncoderScale[index] != 0) continue;
        const double steps = static_cast<double>(axis.motor->getCurrentPosition()) - encoder.calibrationSteps;
        const double ticks = static_cast<double>(encoder.ticks - encoder.calibrationTicks);
        if (fabs(steps) < 32 || fabs(ticks) < 16 || fabs(ticks) <= encoder.bestCalibrationTravel) continue;
        const double scale = steps * bnoFeedbackStepSign(axis) /
            ((index ? PITCH_TRACK_PULSES_PER_DEG : YAW_TRACK_PULSES_PER_DEG) * ticks);
        if (!isfinite(scale) || fabs(scale) < 1e-6 || fabs(scale) > 1.0) continue;
        celestialLearnedEncoderScale[index] = scale;
        encoder.bestCalibrationTravel = fabs(ticks);
    }
}
void initializeCelestialEncoders() {
    celestialEncoderAxes[0] = {}; celestialEncoderAxes[1] = {};
    celestialEncoderGeneration = celestialEncoderServicedGeneration = 0;
    observeCelestialEncoder(0); observeCelestialEncoder(1);
    const double angles[2] = {heading.continuous, physicalPitch()};
    FastAccelStepper *motors[2] = {yawMotor, pitchMotor};
    for (unsigned index = 0; index < 2; ++index) {
        auto &axis = celestialEncoderAxes[index];
        axis.referenceTicks = axis.calibrationTicks = axis.ticks;
        axis.referenceAngle = axis.calibrationAngle = angles[index];
        axis.calibrationSteps = motors[index]->getCurrentPosition();
        axis.calibrationAnchored = true; // GOTO admission requires stopped motors.
        axis.degreesPerTick = celestialLearnedEncoderScale[index];
        axis.scaleKnown = axis.degreesPerTick != 0;
    }
    celestialEncoderMode = celestialEncoderPaused = false;
    celestialBnoHealthy = true; celestialBnoRecoveryReported = false;
    celestialEncoderPauseAt = 0;
    celestialRecoveryYawDiscrepancy = celestialRecoveryPitchDiscrepancy = 0;
}
void updateCelestialEncoderCalibration(unsigned index, double bnoAngle) {
    auto &encoder = celestialEncoderAxes[index];
    if (!encoder.initialized || celestialLearnedEncoderScale[index] != 0) return;
    const double tickTravel = static_cast<double>(encoder.ticks - encoder.calibrationTicks);
    const double encoderTravel = fabs(tickTravel * CELESTIAL_ENCODER_TICK_DEG);
    const double bnoTravel = bnoAngle - encoder.calibrationAngle;
    if (encoderTravel < 0.5 || fabs(bnoTravel) < DIRECTION_RESPONSE_DEG ||
        encoderTravel <= encoder.bestCalibrationTravel) return;
    const double estimate = bnoTravel / tickTravel;
    if (!isfinite(estimate) || fabs(estimate) < 1e-6 || fabs(estimate) > 1.0) return;
    encoder.degreesPerTick = estimate;
    encoder.scaleKnown = true;
    encoder.bestCalibrationTravel = encoderTravel;
}
void celestialRecordTrustedBno() {
    if (!celestialActive() || celestialEncoderMode || bnoAccuracy < BNO_MIN_ACCURACY) return;
    observeCelestialEncoder(0); observeCelestialEncoder(1);
    const double angles[2] = {heading.continuous, physicalPitch()};
    for (unsigned index = 0; index < 2; ++index) {
        updateCelestialEncoderCalibration(index, angles[index]);
        auto &encoder = celestialEncoderAxes[index];
        if (encoder.initialized) {
            encoder.referenceTicks = encoder.ticks;
            encoder.referenceAngle = angles[index];
        }
    }
}
void brakeCelestialTrackAxis(Axis &axis, uint32_t now, bool zeroRate = false) {
    auto &track = celestialTrackAxes[axis.pitch ? 1 : 0];
    // FAS 1.2.7 finishes pause_ticks_left before normal deceleration. The
    // applied period can still be longer than the latest requested period.
    // Capture both BEFORE stopMove(); never extend this deadline while waiting.
    const uint32_t appliedUs = track.runConfirmed ? axis.motor->getPeriodInUsAfterCommandsCompleted() : 0;
    const uint32_t requestedMs = track.rateMilliHz ?
        (1000000UL + track.rateMilliHz - 1) / track.rateMilliHz : 0;
    // Include FAS's remaining ramp steps as well as the in-flight interval:
    // even at sub-hertz speed its integer ramp state can retain one stop step.
    const uint32_t stopSteps = axis.motor->stepsToStop();
    // Zero demand must cancel a pending ultra-slow schedule, not preserve it
    // for another step period. Only use queue-draining forceStop when the
    // applied period exceeds the normal brake budget and <=1 ramp step remains.
    // Faster motion retains normal deceleration. Never reset motor position.
    const bool cancelPause = zeroRate && appliedUs >= BRAKING_TIMEOUT_MS * 1000UL && stopSteps <= 1;
    track.brakeTimeoutMs = BRAKING_TIMEOUT_MS + (cancelPause ? 0 :
        (1 + stopSteps) * std::max((appliedUs + 999) / 1000, requestedMs));
    if (cancelPause) {
        // Queue-only draining needs no new ramp stop flag (which could poison
        // a later start). Issue at most one request for this zero transition.
        if (axis.motor->isRampGeneratorActive()) axis.motor->forceStop();
    } else axis.motor->stopMove();
    track.zeroStopping = zeroRate;
    axis.commandAt = now;
    setMotion(axis, zeroRate ? Motion::TRACK_ZERO : Motion::BRAKING);
    char line[300];
    snprintf(line, sizeof(line),
        "CELESTIAL_AXIS_STOP id=%lu axis=%s reason=%s method=%s last_mHz=%lu applied_period_us=%lu stop_steps=%lu start_ms=%lu timeout_ms=%lu running=%s\n",
        static_cast<unsigned long>(celestialId), axis.pitch ? "PITCH" : "YAW",
        zeroRate ? "ZERO_RATE" : "REVERSAL_OR_FEEDBACK_HOLD", cancelPause ? "CANCEL_PAUSE" : "DECELERATE",
        static_cast<unsigned long>(track.rateMilliHz), static_cast<unsigned long>(appliedUs),
        static_cast<unsigned long>(stopSteps), static_cast<unsigned long>(now),
        static_cast<unsigned long>(track.brakeTimeoutMs), axis.motor->isRunning() ? "YES" : "NO");
    queueText(line);
}
void holdCelestialForEncoder(uint32_t now) {
    Axis *axes[2] = {&yawAxis, &pitchAxis};
    for (Axis *axis : axes) {
        if (!axis->motor->isRunning()) continue;
        if (axis->motion == Motion::TRACK) {
            brakeCelestialTrackAxis(*axis, now);
            continue;
        }
        axis->motor->stopMove();
        if (axis->motion == Motion::SLEW) {
            axis->motion = Motion::BRAKING;
            axis->commandAt = now;
        }
    }
    if (!celestialEncoderPaused) {
        for (auto &track : celestialTrackAxes) {
            track.started = false; track.correction = track.filteredError = 0;
        }
        celestialEncoderPaused = true; celestialEncoderPauseAt = now;
        char line[220];
        snprintf(line, sizeof(line),
            "CELESTIAL_DEGRADED id=%lu encoder_feedback=UNAVAILABLE action=HOLD session_preserved=YES\n",
            static_cast<unsigned long>(celestialId));
        queueText(line);
    }
}
void resumeCelestialEncoderFeedback(uint32_t now) {
    if (!celestialEncoderPaused) return;
    const uint32_t paused = now - celestialEncoderPauseAt;
    controlStartedAt += paused;
    yawAxis.lastProgress += paused; pitchAxis.lastProgress += paused;
    celestialEncoderPaused = false;
    char line[220];
    snprintf(line, sizeof(line),
        "CELESTIAL_DEGRADED id=%lu encoder_feedback=AVAILABLE action=RESUME paused_ms=%lu\n",
        static_cast<unsigned long>(celestialId), static_cast<unsigned long>(paused));
    queueText(line);
}
void resumeCelestialBnoFeedback(uint32_t now) {
    if (!celestialActive() || celestialEncoderMode || !celestialEncoderPaused ||
        !fresh(now) || bnoAccuracy < BNO_MIN_ACCURACY || !poseMotorsStopped()) return;
    const uint32_t paused = now - celestialEncoderPauseAt;
    controlStartedAt += paused;
    yawAxis.lastProgress = pitchAxis.lastProgress = now;
    celestialEncoderPaused = false;
    celestialBnoHealthy = true; celestialBnoRecoveryReported = false;
    char line[240];
    snprintf(line, sizeof(line),
        "CELESTIAL_BNO id=%lu status=RECOVERED action=RESUME feedback=BNO paused_ms=%lu\n",
        static_cast<unsigned long>(celestialId), static_cast<unsigned long>(paused));
    queueText(line);
}
void celestialBnoUnavailable(const char *reason) {
    if (!celestialActive() || poseStopping) return;
    motionWatchdog.disarm();
    observeCelestialEncoder(0); observeCelestialEncoder(1);
    const uint32_t now = millis();
    if (!celestialEncoderMode && celestialEncoderFeedbackAvailable(now)) {
        celestialEncoderMode = true;
        celestialEncoderServicedGeneration = celestialEncoderGeneration - 1;
    }
    if (celestialBnoHealthy) {
        char line[300];
        snprintf(line, sizeof(line),
            "CELESTIAL_BNO id=%lu status=DEGRADED reason=%s feedback=%s session_preserved=YES\n",
            static_cast<unsigned long>(celestialId), reason,
            celestialEncoderMode ? "AS5600" : "BNO_PAUSED");
        queueText(line);
    }
    celestialBnoHealthy = false; celestialBnoRecoveryReported = false;
    if (!celestialEncoderMode || !celestialEncoderFeedbackAvailable(now)) holdCelestialForEncoder(now);
}
bool celestialUsingEncoderFeedback() { return celestialActive() && celestialEncoderMode; }
bool celestialControlFeedbackReady() {
    if (celestialEncoderPaused) return false;
    if (!celestialUsingEncoderFeedback()) return fresh(millis());
    return celestialEncoderFeedbackAvailable(millis());
}
bool serviceCelestialEncoderFeedback(uint32_t now, uint32_t &sampleAt, bool &newSample) {
    observeCelestialEncoder(0); observeCelestialEncoder(1);
    if (!celestialEncoderFeedbackAvailable(now)) {
        holdCelestialForEncoder(now); newSample = false; return false;
    }
    resumeCelestialEncoderFeedback(now);
    const auto &yawEncoder = celestialEncoderAxes[0];
    const auto &pitchEncoder = celestialEncoderAxes[1];
    yawAxis.current = yawEncoder.referenceAngle +
        static_cast<double>(yawEncoder.ticks - yawEncoder.referenceTicks) * yawEncoder.degreesPerTick;
    pitchAxis.current = pitchEncoder.referenceAngle +
        static_cast<double>(pitchEncoder.ticks - pitchEncoder.referenceTicks) * pitchEncoder.degreesPerTick;
    yawAxis.error = yawAxis.target - yawAxis.current;
    pitchAxis.error = pitchAxis.target - pitchAxis.current;
    sampleAt = now - std::max(encoders.state(0).ageMs(now), encoders.state(1).ageMs(now));
    newSample = celestialEncoderGeneration != celestialEncoderServicedGeneration;
    celestialEncoderServicedGeneration = celestialEncoderGeneration;
    return true;
}
void celestialRecordBnoRecovery(const EulerAngles &sample, uint8_t accuracy, uint32_t sampleAt) {
    bnoHealth.recordFresh(sampleAt, false);
    orientation = sample; bnoAccuracy = accuracy; bnoValid = true; lastBnoGood = sampleAt;
    alignmentSampleInvalid = false;
    // Reference diagnostics only: do not consume the encoder control generation
    // here, or healthy BNO reports can starve the encoder-driven servo.
    celestialRecoveryYawDiscrepancy = shortestDifference(sample.heading,
        wrap360(heading.first + yawAxis.current));
    celestialRecoveryPitchDiscrepancy = physicalPitch(sample) - pitchAxis.current;
    heading.previous = wrap360(sample.heading);
    heading.continuous = yawAxis.current + celestialRecoveryYawDiscrepancy;
    if (!celestialBnoHealthy || !celestialBnoRecoveryReported) {
        char line[320];
        snprintf(line, sizeof(line),
            "CELESTIAL_BNO id=%lu status=RECOVERED yaw_discrepancy_deg=%.5f pitch_discrepancy_deg=%.5f action=REPORT_ONLY feedback=AS5600\n",
            static_cast<unsigned long>(celestialId), celestialRecoveryYawDiscrepancy,
            celestialRecoveryPitchDiscrepancy);
        queueText(line);
    }
    celestialBnoHealthy = true; celestialBnoRecoveryReported = true;
}

void rejectCelestial(uint32_t id, const char *reason) {
    char line[320];
    snprintf(line, sizeof(line), "CELESTIAL_REJECTED id=%lu reason=%s\n",
        static_cast<unsigned long>(id), reason);
    queueText(line);
}
void stopCelestial(const char *reason, bool failed) {
    if (!celestialActive() || poseStopping) return;
    celestialFailed = failed;
    snprintf(celestialStopReason, sizeof(celestialStopReason), "%s", reason);
    beginPoseStop(celestialStopReason);
}
void celestialResult(const char *reason, bool latched) {
    char line[360];
    snprintf(line, sizeof(line), "CELESTIAL_RESULT id=%lu status=%s reason=%s\n",
        static_cast<unsigned long>(celestialId),
        celestialFailed || latched ? "FAILED" : "STOPPED", reason);
    queueText(line);
    celestialTracking = celestialFailed = false;
    celestialEncoderMode = celestialEncoderPaused = false;
    celestialBnoHealthy = true; celestialBnoRecoveryReported = false;
    celestialId = 0;
}

// Same displacement/profile/precision-tail model used by LEVEL/NORTH, applied
// independently to both original POSE profiles. No assumed gearbox conversion.
double celestialAxisSeconds(bool pitch, double error, double pulsesPerDegree) {
    const double speed = slewSpeed(pitch), acceleration = slewAcceleration(pitch);
    double seconds = milestone8::finiteSeconds(error * pulsesPerDegree, speed, acceleration);
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
void celestialDeadline() {
    if (celestialTracking) return;
    double seconds = 0;
    for (unsigned n = 0; n < 2; ++n) {
        if (celestialInitialErrors[n] <= TOLERANCE_DEG) continue;
        if (celestialPulsesPerDegree[n] <= 0) return;
        const double axisSeconds = celestialAxisSeconds(n == 1,
            celestialInitialErrors[n], celestialPulsesPerDegree[n]);
        if (!isfinite(axisSeconds)) return;
        seconds = fmax(seconds, axisSeconds);
    }
    const double duration = ceil(2000 * seconds + 10000);
    if (!isfinite(duration) || duration >= 0x7fffffffUL) {
        stopCelestial("celestial timing estimate out of range"); return;
    }
    const uint32_t estimate = static_cast<uint32_t>(duration);
    if (!celestialTimingKnown || estimate > celestialDeadlineMs) {
        celestialTimingKnown = true;
        celestialDeadlineMs = std::max<uint32_t>(estimate, millis() - controlStartedAt + SETTLE_MS + 1000);
        char line[120];
        snprintf(line, sizeof(line), "CELESTIAL_DEADLINE id=%lu deadline_ms=%lu\n",
            static_cast<unsigned long>(celestialId), static_cast<unsigned long>(celestialDeadlineMs));
        queueText(line);
    }
}
void updateCelestialTiming(Axis &axis) {
    const unsigned index = axis.pitch ? 1 : 0;
    const double angle = axis.current - celestialTimingAngles[index];
    const double pulses = static_cast<double>(axis.motor->getCurrentPosition()) - celestialTimingSteps[index];
    if (fabs(angle) < DIRECTION_RESPONSE_DEG || fabs(pulses) < 8 ||
        pulses * angle * bnoFeedbackStepSign(axis) <= 0) return;
    const double estimate = fabs(pulses / angle);
    if (celestialPulsesPerDegree[index] <= 0 || estimate > celestialPulsesPerDegree[index] * 1.1) {
        celestialPulsesPerDegree[index] = estimate;
        celestialDeadline();
    }
}
uint32_t celestialProgressTimeout(const Axis &axis) {
    const uint32_t existing = axis.motion == Motion::SLEW ? SLEW_PROGRESS_TIMEOUT_MS : PROGRESS_TIMEOUT_MS;
    const double conversion = celestialPulsesPerDegree[axis.pitch ? 1 : 0];
    if (conversion <= 0) return existing;
    const double pulses = 0.15 * conversion;
    double seconds;
    if (axis.motion == Motion::SLEW) {
        seconds = milestone8::finiteSeconds(pulses, slewSpeed(axis.pitch), slewAcceleration(axis.pitch));
    } else {
        const double burst = fmax(2.0, fabs(static_cast<double>(correctionSteps(axis.error, true, axis.pitch))));
        seconds = ceil(pulses / burst) * (milestone8::finiteSeconds(burst,
            correctionSpeed(axis.error, true), ACCELERATION) + PRECISION_OBSERVE_MS / 1000.0);
    }
    return isfinite(seconds) && seconds < 700000 ?
        std::max(existing, static_cast<uint32_t>(ceil(3000 * seconds + 1000))) : existing;
}
void celestialSafety() {
    if (!celestialActive() || poseStopping) return;
    const uint32_t now = millis();
    if (now - celestialUpdatedAt >= CELESTIAL_LEASE_MS) {
        stopCelestial("target update lease expired after 3000 ms"); return;
    }
    if (!fresh(now) || now - lastBnoGood >= ALIGNMENT_PAUSE_AGE_MS || alignmentSampleInvalid)
        celestialBnoUnavailable("BNO orientation stale or invalid");
    if (bnoAccuracy < BNO_MIN_ACCURACY)
        celestialBnoUnavailable("BNO accuracy below 2");
    const bool feedbackAvailable = celestialControlFeedbackReady();
    if (!celestialTracking && feedbackAvailable && now - controlStartedAt >= celestialDeadlineMs) {
        stopCelestial("celestial GOTO displacement/response deadline expired"); return;
    }
    const double currentPitch = celestialEncoderMode ? pitchAxis.current : physicalPitch();
    if (fabs(currentPitch) >= MAX_USABLE_PITCH_DEG || fabs(pitchAxis.target) >= MAX_USABLE_PITCH_DEG) {
        stopCelestial("pitch reached +/-75 degree guard"); return;
    }
}
void enterCelestialTracking() {
    if (!celestialActive() || poseStopping || celestialTracking) return;
    learnTimingResponse();
    // Acquisition timing estimates remain separate and cannot alter TRACK scale.
    celestialTrackAxes[0].conversion = YAW_TRACK_PULSES_PER_DEG;
    celestialTrackAxes[1].conversion = PITCH_TRACK_PULSES_PER_DEG;
    celestialTracking = true;
    // Freeze the acquired BNO reference and scale; all subsequent yaw/pitch
    // position, residual, direction and response checks use AS5600 samples.
    celestialEncoderMode = true;
    celestialEncoderServicedGeneration = celestialEncoderGeneration - 1;
    for (unsigned index = 0; index < 2; ++index) {
        if (celestialEncoderAxes[index].scaleKnown)
            celestialLearnedEncoderScale[index] = celestialEncoderAxes[index].degreesPerTick;
    }
    if (!celestialEncoderFeedbackAvailable(millis())) {
        queueText("CELESTIAL_TRACK encoder reference/scale unavailable; HOLD until qualified AS5600 feedback; if uncalibrated, STOP and move both axes manually before retrying\n");
        holdCelestialForEncoder(millis());
    }
    // Acquisition settled at a finite target; continuous TRACK never completes.
    yawAxis.settled = pitchAxis.settled = false;
    yawAxis.slewFinished = pitchAxis.slewFinished = true;
    phase = Phase::MOVING;
    char line[140];
    snprintf(line, sizeof(line), "CELESTIAL_TRACK id=%lu tolerance_deg=%.3f\n",
        static_cast<unsigned long>(celestialId), TOLERANCE_DEG);
    queueText(line);
    celestialRateLoggedAt = millis();
    celestialRateTelemetry();
}
void beginCelestial(uint32_t id, double yaw, double pitch) {
    checkSensorReset();
    if (!commandIdle() || !controlReady || !poseMotorsStopped()) {
        rejectCelestial(id, "stopped READY required; busy or abort latched"); return;
    }
    const char *reason = northUnavailableReason(millis());
    if (strcmp(reason, "usable") != 0) { rejectCelestial(id, reason); return; }
    if (!fresh(millis()) || !isfinite(physicalPitch()) || !isfinite(yaw) || !isfinite(pitch)) {
        rejectCelestial(id, "fresh finite BNO orientation and targets required"); return;
    }
    if (fabs(pitch) >= MAX_USABLE_PITCH_DEG || fabs(physicalPitch()) >= MAX_USABLE_PITCH_DEG) {
        rejectCelestial(id, "current and target pitch must be inside +/-75 degrees"); return;
    }
    // Preserve the shortest heading move without a fixed yaw travel envelope.
    const double targetYaw = heading.continuous + shortestDifference(yaw, orientation.heading);
    if (!bnoWatchdogReady || !motionWatchdog.clearTripWhenStopped()) {
        rejectCelestial(id, "orientation watchdog unavailable"); return;
    }
    operation = Operation::CELESTIAL;
    poseActive = poseNeedsBno = true; poseStopping = alignmentPaused = false;
    finalPrinted = false; phase = Phase::MOVING; window.active = false;
    posePrecisionOnly = false; poseStoppedObserved = false;
    resetPoseAxis(yawAxis, targetYaw, 0); resetPoseAxis(pitchAxis, pitch, 0);
    yawRequired = true; accuracyRequired = false; accuracyGrace = {};
    carriageTarget = carriageMotor->getCurrentPosition(); carriagePending = false;
    yawRateCap = YAW_SLEW_SPEED_HZ; pitchRateCap = PITCH_SLEW_SPEED_HZ;
    plannedDurationSeconds = 0; timingLimited = false;
    settleSamples = 0; sawYawMotion = sawPitchMotion = concurrentMotion = concurrentThreeMotion = false;
    controlStartedAt = celestialUpdatedAt = millis(); lastControlSample = bnoHealth.freshSamples;
    celestialId = id; celestialSequence = 0; celestialTracking = celestialFailed = false;
    celestialTrackAxes[0] = {}; celestialTrackAxes[1] = {};
    celestialTargetRate[0] = celestialTargetRate[1] = 0; celestialRateKnown = false;
    celestialTimingKnown = false; celestialDeadlineMs = LEG_TIMEOUT_MS;
    celestialInitialErrors[0] = fabs(yawAxis.error); celestialInitialErrors[1] = fabs(pitchAxis.error);
    celestialTimingAngles[0] = heading.continuous; celestialTimingAngles[1] = physicalPitch();
    celestialTimingSteps[0] = yawMotor->getCurrentPosition(); celestialTimingSteps[1] = pitchMotor->getCurrentPosition();
    celestialPulsesPerDegree[0] = yawPulsesPerDegree; celestialPulsesPerDegree[1] = pitchPulsesPerDegree;
    initializeCelestialEncoders();
    recordTimingStarts();
    // Once admitted, BNO health is optional for this celestial session.  Its
    // independent stale watchdog must therefore never turn an outage into a
    // forced stop; encoder freshness is handled by a preserving HOLD instead.
    motionWatchdog.disarm();
    char line[220];
    snprintf(line, sizeof(line), "CELESTIAL_ACCEPTED id=%lu deadline_ms=%lu yaw_target=%.5f pitch_target=%.5f lease_ms=%lu\n",
        static_cast<unsigned long>(id), static_cast<unsigned long>(celestialDeadlineMs), wrap360(yaw), pitch,
        static_cast<unsigned long>(CELESTIAL_LEASE_MS));
    queueText(line);
    celestialDeadline();
    celestialSafety();
}
void shiftCelestialTarget(Axis &axis, double target) {
    // Compensate the error history only for the externally commanded change.
    // Retain measured progress, direction checks, braking and correction state.
    const double oldError = axis.target - axis.current;
    const double newError = target - axis.current;
    const double magnitudeChange = fabs(newError) - fabs(oldError);
    axis.bestError = fmax(0.0, axis.bestError + magnitudeChange);
    axis.progressError = fmax(0.0, axis.progressError + magnitudeChange);
    axis.previousErrorMagnitude = fmax(0.0, axis.previousErrorMagnitude + magnitudeChange);
    axis.target = target; axis.error = newError;
}
void updateCelestial(uint32_t id, uint32_t sequence, double yaw, double pitch) {
    if (!celestialActive() || poseStopping || id != celestialId) {
        rejectCelestial(id, "no matching active celestial session"); return;
    }
    celestialSafety();
    if (poseStopping) return;
    if (!sequence || sequence <= celestialSequence) {
        rejectCelestial(id, "update sequence must increase"); return;
    }
    const uint32_t elapsed = millis() - celestialUpdatedAt;
    if (elapsed < CELESTIAL_MIN_UPDATE_MS) {
        rejectCelestial(id, "update interval below 100 ms"); return;
    }
    const double yawDelta = shortestDifference(yaw, wrap360(heading.first + yawAxis.target));
    const double pitchDelta = pitch - pitchAxis.target;
    const double allowed = elapsed * (CELESTIAL_MAX_TARGET_RATE_DPS / 1000.0) + 0.002;
    if (!isfinite(yaw) || !isfinite(pitch) || fabs(yawDelta) > allowed || fabs(pitchDelta) > allowed) {
        stopCelestial("target update exceeds 1 degree/s angular rate bound"); return;
    }
    const double targetYaw = yawAxis.target + yawDelta;
    if (fabs(pitch) >= MAX_USABLE_PITCH_DEG) {
        stopCelestial("updated target exceeds pitch travel guard"); return;
    }
    shiftCelestialTarget(yawAxis, targetYaw); shiftCelestialTarget(pitchAxis, pitch);
    const double seconds = elapsed / 1000.0;
    const double alpha = seconds / (CELESTIAL_RATE_FILTER_S + seconds);
    const double deltas[2] = {yawDelta, pitchDelta};
    for (unsigned index = 0; index < 2; ++index) {
        const double rate = deltas[index] / seconds;
        celestialTargetRate[index] = celestialRateKnown ?
            celestialTargetRate[index] + alpha * (rate - celestialTargetRate[index]) : rate;
    }
    celestialRateKnown = true;
    celestialSequence = sequence; celestialUpdatedAt = millis();
}

void serviceCelestialTrackAxis(Axis &axis, uint32_t now, uint32_t sampleAt) {
    const unsigned index = axis.pitch ? 1 : 0;
    auto &track = celestialTrackAxes[index];
    if (axis.motion == Motion::TRACK_ZERO && track.zeroStopping) {
        if (axis.motor->isRunning()) {
            if (now - axis.commandAt >= track.brakeTimeoutMs)
                stopCelestial("TRACK zero-rate stop timed out");
            return;
        }
        if (!axis.observing) { axis.observing = true; axis.stoppedAt = now; return; }
        if (!sampleAtOrAfter(sampleAt, axis.stoppedAt + PRECISION_OBSERVE_MS)) return;
        axis.observing = false; track.zeroStopping = false;
        track.runConfirmed = false;
    }
    // A sensor pause or direction reversal must finish braking and observe a
    // fresh stopped sample before continuous motion is allowed again.
    if (axis.motion == Motion::BRAKING || axis.finiteActive || axis.observing) {
        if (axis.motor->isRunning()) {
            if (now - axis.commandAt >= track.brakeTimeoutMs)
                stopCelestial("TRACK braking timed out");
            return;
        }
        if (!axis.observing) {
            axis.finiteActive = false; axis.observing = true; axis.stoppedAt = now;
            return;
        }
        if (!sampleAtOrAfter(sampleAt, axis.stoppedAt + PRECISION_OBSERVE_MS)) return;
        axis.observing = false;
        // No motion response is expected during an intentional long brake.
        track.progressAt = now; track.progressPosition = axis.current; track.commandedTravel = 0;
        setMotion(axis, Motion::HOLD);
        return;
    }
    if (!celestialRateKnown) return;
    if (axis.motion == Motion::TRACK && !track.runConfirmed) {
        // runForward/Backward activates the ramp synchronously, not the queue.
        // A pending forceStop flag can consume that first run in FAS 1.2.7.
        // Queue execution (including zero-step pauses) confirms launch; a STEP
        // pulse is NOT required. Retry once only before that acknowledgment,
        // only when fully idle, and within the original bounded start window.
        if (axis.motor->isQueueRunning()) track.runConfirmed = true;
        else {
            if (now - axis.commandAt >= BRAKING_TIMEOUT_MS) {
                stopCelestial("TRACK motor failed to start"); return;
            }
            if (!axis.motor->isRunning() && now - track.launchAt >= PRECISION_OBSERVE_MS) {
                if (track.startRetried) { stopCelestial("TRACK motor failed to start"); return; }
                track.startRetried = true;
                track.launchAt = now;
                const int result = static_cast<int>(axis.slewDirection > 0 ?
                    axis.motor->runForward() : axis.motor->runBackward());
                if (result != static_cast<int>(MOVE_OK)) stopCelestial("TRACK continuous rate command rejected");
            }
            return;
        }
    }
    if (track.started && now - track.servicedAt < CELESTIAL_RATE_SERVICE_MS) return;
    const double seconds = track.started ? fmin(0.25, (now - track.servicedAt) / 1000.0) : 0.1;
    track.servicedAt = now;
    // Compare the measured position to the trajectory at sensor receipt time,
    // not UART delivery time. Do not mutate the last accepted target/lease.
    const double age = static_cast<int32_t>(sampleAt - celestialUpdatedAt) / 1000.0;
    const double residual = axis.error + celestialTargetRate[index] * age;
    track.filteredError += seconds / (CELESTIAL_RESIDUAL_FILTER_S + seconds) *
        (residual - track.filteredError);
    const double outsideNoise = fmax(0.0, fabs(track.filteredError) - APPROACH_DEADBAND_DEG);
    const double desiredCorrection = copysign(fmin(CELESTIAL_RESIDUAL_MAX_DPS,
        outsideNoise * CELESTIAL_RESIDUAL_GAIN), track.filteredError);
    const double ramp = CELESTIAL_RESIDUAL_RAMP_DPS2 * seconds;
    track.correction += fmax(-ramp, fmin(ramp, desiredCorrection - track.correction));
    const double rate = fmax(-CELESTIAL_MAX_TARGET_RATE_DPS,
        fmin(CELESTIAL_MAX_TARGET_RATE_DPS, celestialTargetRate[index] + track.correction));
    const int direction = bnoSlewStepDirection(rate, axis.pitch);
    const double requested = fmin(limitedSpeed(axis, slewSpeed(axis.pitch)) * 1000.0,
        fabs(rate) * track.conversion * 1000.0);
    // FAS's 32-bit tick interval cannot represent arbitrarily low rates. Treat
    // an unrepresentable rate as zero; never clamp UP and overdrive an axis.
    const uint32_t milliHz = requested >= CELESTIAL_MIN_RATE_MILLIHZ ?
        static_cast<uint32_t>(lround(requested)) : 0;
    // A moving target need not have shrinking residual error. Detect absent
    // measured motion instead, after enough commanded travel to distinguish it
    // from sensor resolution and legitimate sub-hertz step pauses. Keep the
    // residual filter/correction and telemetry independent of this stall check.
    if (!milliHz || axis.motion != Motion::TRACK || !track.started ||
            (axis.current - track.progressPosition) * bnoFeedbackStepSign(axis) * axis.slewDirection >= 0.1) {
        track.progressAt = now; track.progressPosition = axis.current; track.commandedTravel = 0;
    } else {
        track.commandedTravel += (track.rateMilliHz / 1000.0 / track.conversion) * seconds;
        if (track.commandedTravel >= 0.2 && now - track.progressAt >= CELESTIAL_TRACK_PROGRESS_MS) {
            stopCelestial("TRACK commanded motion has no measured response for 60000 ms"); return;
        }
    }
    track.started = true;
    if (!milliHz) {
        if (axis.motion == Motion::TRACK) brakeCelestialTrackAxis(axis, now, true);
        else if (axis.motion != Motion::TRACK_ZERO) setMotion(axis, Motion::TRACK_ZERO);
        return;
    }
    if (axis.motion == Motion::TRACK && !axis.motor->isRunning()) {
        if (now - axis.commandAt < PRECISION_OBSERVE_MS) return;
        stopCelestial("TRACK motor stopped unexpectedly"); return;
    }
    if (axis.motion == Motion::TRACK && direction != axis.slewDirection) {
        brakeCelestialTrackAxis(axis, now); return;
    }
    if (axis.motion == Motion::TRACK && track.rateMilliHz == milliHz) return;
    int result = axis.motor->setAcceleration(ACCELERATION);
    if (!result) result = axis.motor->setSpeedInMilliHz(milliHz);
    if (!result) {
        if (axis.motion == Motion::TRACK) axis.motor->applySpeedAcceleration();
        else result = static_cast<int>(direction > 0 ? axis.motor->runForward() : axis.motor->runBackward());
    }
    if (result != static_cast<int>(MOVE_OK)) { stopCelestial("TRACK continuous rate command rejected"); return; }
    track.rateMilliHz = milliHz;
    if (axis.motion != Motion::TRACK) {
        track.runConfirmed = track.startRetried = false;
        track.launchAt = now;
        axis.slewDirection = direction; axis.commandAt = now;
        setMotion(axis, Motion::TRACK);
    }
}
void serviceCelestialTracking(uint32_t now, uint32_t sampleAt) {
    celestialSafety();
    if (poseStopping || !celestialControlFeedbackReady()) return;
    const double seconds = (now - celestialUpdatedAt) / 1000.0;
    if (fabs(pitchAxis.target + celestialTargetRate[1] * seconds) >= MAX_USABLE_PITCH_DEG) {
        stopCelestial("extrapolated TRACK target exceeds pitch travel guard"); return;
    }
    serviceCelestialTrackAxis(yawAxis, now, sampleAt);
    if (!poseStopping) serviceCelestialTrackAxis(pitchAxis, now, sampleAt);
}
bool executeCelestialCommand(char **tokens, unsigned count) {
    const bool start = strcmp(tokens[0], "CELESTIAL_GOTO") == 0;
    if (!start && strcmp(tokens[0], "CELESTIAL_UPDATE") != 0) return false;
    int64_t parsedId = 0, parsedSequence = 0;
    if (count < 2 || !parseKeyframeInteger(tokens[1], 1, UINT32_MAX, parsedId)) {
        rejectCelestial(0, "positive uint32 session id required"); return true;
    }
    const uint32_t id = static_cast<uint32_t>(parsedId);
    double yaw = 0, pitch = 0;
    if (count != (start ? 4U : 5U) ||
        (!start && !parseKeyframeInteger(tokens[2], 1, UINT32_MAX, parsedSequence)) ||
        !parseAngle(tokens[start ? 2 : 3], yaw) || !parseAngle(tokens[start ? 3 : 4], pitch)) {
        rejectCelestial(id, "use CELESTIAL_GOTO id yaw pitch or CELESTIAL_UPDATE id sequence yaw pitch"); return true;
    }
    if (start) beginCelestial(id, yaw, pitch);
    else updateCelestial(id, static_cast<uint32_t>(parsedSequence), yaw, pitch);
    return true;
}
void celestialRateTelemetry() {
    Axis *axes[2] = {&yawAxis, &pitchAxis};
    for (unsigned n = 0; n < 2; ++n) {
        const auto &track = celestialTrackAxes[n];
        const Axis &axis = *axes[n];
        // Signed STEP Hz; correction_dps uses the feedback angle convention.
        const double nominal = celestialTargetRate[n] * track.conversion * bnoFeedbackStepSign(axis);
        const double correction = track.correction * track.conversion * bnoFeedbackStepSign(axis);
        const double commanded = axis.motion == Motion::TRACK && !poseStopping && !celestialEncoderPaused ?
            static_cast<double>(axis.slewDirection) * track.rateMilliHz / 1000.0 : 0;
        char line[420];
        snprintf(line, sizeof(line),
            "CELESTIAL_RATE id=%lu axis=%s source=CONFIGURED pulses_per_degree=%.3f rate_known=%s feedback=%s target_dps=%.6f nominal_hz=%.6f correction_dps=%.6f correction_hz=%.6f commanded_hz=%.6f error_deg=%.5f\n",
            static_cast<unsigned long>(celestialId), n ? "PITCH" : "YAW", track.conversion,
            celestialRateKnown ? "YES" : "NO", celestialEncoderMode ? "AS5600" : "BNO",
            celestialTargetRate[n], nominal, track.correction, correction, commanded, axis.error);
        queueText(line);
    }
}
void celestialTelemetry() {
    if (!celestialActive()) return;
    if (celestialTracking && millis() - celestialRateLoggedAt >= 2000) {
        celestialRateLoggedAt = millis();
        celestialRateTelemetry();
    }
    char line[620];
    snprintf(line, sizeof(line),
        "CELESTIAL_STATE id=%lu mode=%s yaw_target=%.5f pitch_target=%.5f yaw_error=%.5f pitch_error=%.5f lease_age_ms=%lu feedback=%s bno=%s encoder=%s bno_yaw_discrepancy_deg=%.5f bno_pitch_discrepancy_deg=%.5f yaw_target_dps=%.6f pitch_target_dps=%.6f yaw_rate_ready=%s pitch_rate_ready=%s\n",
        static_cast<unsigned long>(celestialId), poseStopping ? "STOPPING" : (celestialTracking ? "TRACK" : "GOTO"),
        wrap360(heading.first + yawAxis.target), pitchAxis.target, yawAxis.error, pitchAxis.error,
        static_cast<unsigned long>(millis() - celestialUpdatedAt), celestialEncoderMode ? "AS5600" : "BNO",
        celestialBnoHealthy ? "AVAILABLE" : "DEGRADED",
        celestialEncoderFeedbackAvailable(millis()) ? "AVAILABLE" : "UNAVAILABLE",
        celestialRecoveryYawDiscrepancy, celestialRecoveryPitchDiscrepancy,
        celestialTargetRate[0], celestialTargetRate[1],
        celestialTrackAxes[0].conversion > 0 ? "YES" : "NO",
        celestialTrackAxes[1].conversion > 0 ? "YES" : "NO");
    queueText(line);
}
