#pragma once
// Temporary foreground-only observation. Never writes GPIO, motor or lease state.
// Included inside main.cpp's namespace, before manual_control's functions.
struct PitchDirectionDiagnostics {
    uint32_t windowAt = 0, suppressed = 0, checkAt = 0;
    unsigned emitted = 0;
    bool checkPending = false;
    uint32_t levelCheckAt = 0;
    int levelRequestedAxisDirection = 0, levelStepSign = 0;
    bool levelCheckPending = false;
} pitchDirectionDiagnostics;

bool allowPitchDirectionTrace() {
    auto &d = pitchDirectionDiagnostics;
    const uint32_t now = millis();
    if (now - d.windowAt >= 1000) { d.windowAt = now; d.emitted = 0; }
    if (d.emitted >= 8) { ++d.suppressed; return false; }
    ++d.emitted; return true;
}

unsigned pitchDirOutputReadback() {
#ifdef M07_HOST_TEST
    return (simulated::gpioOutput >> tmp_hardware::PITCH_DIR_PIN) & 1U;
#else
    // OUTPUT-only pins need not have their input buffer enabled. Read the
    // output latch, not digitalRead(); this is NOT a measurement at the driver.
    return (REG_READ(GPIO_OUT_REG) >> tmp_hardware::PITCH_DIR_PIN) & 1U;
#endif
}
void tracePitchDirection(const char *event, const char *call = "none", int rc = 0) {
    auto &d = pitchDirectionDiagnostics;
    const uint32_t now = millis();
    // Bound chatter/rejected-start diagnostics independently of control. No wait.
    if (!allowPitchDirectionTrace()) return;
    const auto &m = manualPitch;
    const int step = m.request ? stepDirection(m.request, true) : 0;
    const unsigned dir = pitchDirOutputReadback();
    char result[12];
    if (strcmp(call, "none") == 0) strcpy(result, "NA");
    else snprintf(result, sizeof(result), "%d", rc);
    char line[256];
    snprintf(line, sizeof(line),
        "PITCH_DIR at_ms=%lu event=%s cmd=%d step_dir=%d call=%s rc=%s active_sign=%d braking=%u observing=%u ending=%u dir26_out=%u motor_mHz=%ld suppressed=%lu\n",
        (unsigned long)now, event, m.request, step, call, result, m.direction,
        unsigned(m.braking), unsigned(m.observing), unsigned(manualEnding), dir,
        (long)pitchMotor->getCurrentSpeedInMilliHz(), (unsigned long)d.suppressed);
    queueText(line); // Existing bounded, nonblocking TX buffer; same drop counter.
    d.suppressed = 0;
}
void traceLevelPitchDirection(const char *event, const Axis &axis, const char *mode,
                              int requestedAxisDirection, int32_t steps, const char *call, int rc) {
    if (!alignmentActive() || operation != Operation::LEVEL || !axis.pitch) return;
    auto &d = pitchDirectionDiagnostics;
    if (!allowPitchDirectionTrace()) return;
    const int stepSign = steps > 0 ? 1 : (steps < 0 ? -1 : 0);
    const unsigned dir = pitchDirOutputReadback();
    char line[384];
    snprintf(line, sizeof(line),
        "LEVEL_DIR at_ms=%lu event=%s pitch=%.3f target=%.3f error=%.3f mode=%s requested_encoder_dir=%d step_sign=%d steps=%ld call=%s rc=%d dir26_out=%u motor_mHz=%ld suppressed=%lu\n",
        (unsigned long)millis(), event, physicalPitch(), axis.target, axis.error,
        mode, requestedAxisDirection, stepSign, (long)steps, call, rc, dir,
        (long)pitchMotor->getCurrentSpeedInMilliHz(), (unsigned long)d.suppressed);
    queueText(line);
    d.suppressed = 0;
}
void armLevelPitchDirectionCheck(int requestedAxisDirection, int stepSign) {
    auto &d = pitchDirectionDiagnostics;
    if (!alignmentActive() || operation != Operation::LEVEL) return;
    d.levelCheckPending = true; d.levelCheckAt = millis();
    d.levelRequestedAxisDirection = requestedAxisDirection;
    d.levelStepSign = stepSign;
}
void serviceLevelPitchDirectionCheck() {
    auto &d = pitchDirectionDiagnostics;
    if (!d.levelCheckPending) return;
    if (!alignmentActive() || operation != Operation::LEVEL) {
        d.levelCheckPending = false; return;
    }
    if (millis() - d.levelCheckAt < 100) return;
    d.levelCheckPending = false;
    traceLevelPitchDirection("DIR_CHECK", pitchAxis, motionText(pitchAxis),
        d.levelRequestedAxisDirection, d.levelStepSign, "latch", static_cast<int>(MOVE_OK));
}
void servicePitchDirectionCheck() {
    auto &d = pitchDirectionDiagnostics;
    if (!d.checkPending) return;
    if (!manualActive || manualPitch.braking || manualPitch.observing || !manualPitch.direction) {
        d.checkPending = false; return;
    }
    if (millis() - d.checkAt < 100) return;
    d.checkPending = false;
    tracePitchDirection("DIR_CHECK"); // One later latch sample, never a delay.
}
