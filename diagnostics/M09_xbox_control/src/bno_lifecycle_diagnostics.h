#pragma once
// Included inside main.cpp's namespace, after its state/accessors. Observation
// only: callbacks never print, access I2C, or change a motor/reference/watchdog.
struct BnoLifecycleDiagnostics : BnoIoStatistics {
    BnoTraceEvent events[32];
    unsigned head = 0, size = 0;
} bnoTrace;

void publishSensorContext() {
    m09::SensorControlContext context;
    auto &e = context.trace;
    e.atMs = millis(); e.atUs = micros();
    e.bnoAge = bnoHealth.freshSamples ? e.atMs-lastBnoGood : UINT32_MAX;
    e.isFresh = fresh(e.atMs); e.phase = phaseText(); e.manual = manualActive; e.pose = poseActive;
    e.yaw = manualYaw.request; e.pitch = manualPitch.request; e.carriage = manualCarriage.request;
    e.yawMilliHz = yawMotor ? yawMotor->getCurrentSpeedInMilliHz() : 0;
    e.pitchMilliHz = pitchMotor ? pitchMotor->getCurrentSpeedInMilliHz() : 0;
    e.carriageMilliHz = carriageMotor ? carriageMotor->getCurrentSpeedInMilliHz() : 0;
    context.halted = phase == Phase::ABORTED;
    context.queryIdle = commandIdle() && !manualActive && !poseActive && yawMotor && pitchMotor &&
        carriageMotor && !yawMotor->isRunning() && !pitchMotor->isRunning() && !carriageMotor->isRunning();
    sensorWorker.control.publish(context);
}
const char *resetCauseText(uint32_t cause) {
    switch (cause) {
    case 0: return "UNSPECIFIED"; case 1: return "POWER_ON"; case 2: return "INTERNAL";
    case 3: return "WATCHDOG"; case 4: return "EXTERNAL"; case 5: return "OTHER";
    default: return "UNKNOWN";
    }
}
void serviceTraceOutput() {
    // One bounded line per loop, retain it if the existing UART queue is full.
    if (!bnoTrace.size) return;
    const auto &e = bnoTrace.events[bnoTrace.head];
    char line[1000];
    if (txOffset == txLength) txOffset = txLength = 0;
    if (sizeof(line) > sizeof(txBuffer) - txLength) return; // Don't format repeatedly into a full queue.
    snprintf(line, sizeof(line),
        "BNO_TRACE kind=%s at_ms=%lu at_us=%lu start_us=%lu duration_us=%lu reset_event=%lu a=%lu b=%lu c=%lu result=%d cause=%s bno_age_ms=%lu fresh=%s phase=%s manual=%u pose=%u jog=%d/%d/%d motor_mHz=%ld/%ld/%ld encB_attempt_ms=%lu encB_age_ms=%lu encB_failures=%lu last_io_start_us=%lu last_io_duration_us=%lu last_io_stage=%u last_io_ok=%d raw_status=%u seq=%u norm_sq=%.9g heading_accuracy_rad=%.9g context_at_ms=%lu\n",
        e.kind, (unsigned long)e.atMs, (unsigned long)e.atUs, (unsigned long)e.startUs,
        (unsigned long)e.durationUs, (unsigned long)e.resetEvent, (unsigned long)e.a,
        (unsigned long)e.b, (unsigned long)e.c, e.result,
        strcmp(e.kind, "PRODUCT_ID") == 0 ? resetCauseText(e.a) : "NA",
        (unsigned long)e.bnoAge, e.isFresh ? "YES" : "NO", e.phase, e.manual, e.pose,
        e.yaw, e.pitch, e.carriage, e.yawMilliHz, e.pitchMilliHz, e.carriageMilliHz,
        (unsigned long)e.encoderAttempt, (unsigned long)e.encoderAge, (unsigned long)e.encoderFailures,
        (unsigned long)e.lastIoStart, (unsigned long)e.lastIoDuration, e.lastIoStage, e.lastIoResult,
        e.rawStatus, e.sequence, e.normSquared, e.headingAccuracy, (unsigned long)e.contextAtMs);
    if (strlen(line) > sizeof(txBuffer) - txLength) return;
    queueText(line);
    bnoTrace.head = (bnoTrace.head + 1) % 32; --bnoTrace.size;
}
