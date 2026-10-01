// Included after finish()/abortTest() inside main.cpp's private namespace.
// Cancellation depends only on motor state and wall time, never on the sensor
// worker making progress. Keep POSE active until every motor has stopped so a
// new command cannot replace an in-flight braking command.
uint32_t poseStopStartedAt = 0;
bool poseStopForced = false;
const char *poseStopReason = "operator STOP";

bool poseMotorsStopped() {
    return yawMotor && pitchMotor && carriageMotor && !yawMotor->isRunning() &&
        !pitchMotor->isRunning() && !carriageMotor->isRunning();
}

void beginPoseStop(const char *reason = "operator STOP") {
    if (!poseActive || poseStopping) return;
    poseStopping = true;
    poseStopStartedAt = millis(); poseStopForced = false;
    poseStopReason = reason;
    carriagePending = false;
    // Publish cancellation before touching any motor: serviceAxes must never
    // issue another correction or a deferred carriage command after STOP.
    yawAxis.finiteActive = pitchAxis.finiteActive = false;
    yawAxis.observing = pitchAxis.observing = false;
    if (yawMotor && yawMotor->isRunning()) yawMotor->stopMove();
    if (pitchMotor && pitchMotor->isRunning()) pitchMotor->stopMove();
    if (carriageMotor && carriageMotor->isRunning()) carriageMotor->stopMove();
    // Every participating axis has received braking. BNO loss must not replace
    // normal braking with a feedback-dependent operation or block cancellation.
    motionWatchdog.disarm();
    queueText("POSE STOPPING: "); queueText(reason); queueText("; braking all moving axes\n");
}

void servicePoseStop() {
    if (!poseStopping || !poseActive) return;
    if (!poseMotorsStopped() && millis() - poseStopStartedAt >= BRAKING_TIMEOUT_MS) {
        if (yawMotor && yawMotor->isRunning()) yawMotor->forceStop();
        if (pitchMotor && pitchMotor->isRunning()) pitchMotor->forceStop();
        if (carriageMotor && carriageMotor->isRunning()) carriageMotor->forceStop();
        if (!poseStopForced) invalidateKeyframes("POSE braking timeout; forced stop");
        poseStopForced = true;
    }
    if (!poseMotorsStopped()) return;
    finish(false, poseStopForced ? "operator STOP; braking timeout; motors forced stopped" :
        (strcmp(poseStopReason, "operator STOP") == 0 ? "operator STOP; all motors stopped" : poseStopReason));
    queueText("POSE CANCELLED: "); queueText(poseStopReason); queueText("; target not completed\n");
}
