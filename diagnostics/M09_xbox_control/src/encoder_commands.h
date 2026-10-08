// Calibration is session-local. Reboot/configuration changes and loss of encoder
// continuity require explicit references; no motor move is issued here.
void serviceEncoders();
void sensorTelemetry();
bool executeCalibrationCommand(char **tokens, unsigned count) {
    const bool configure = strcmp(tokens[0], "ENCODER_CONFIG") == 0;
    const bool north = strcmp(tokens[0], "SET_NORTH") == 0;
    const bool level = strcmp(tokens[0], "SET_LEVEL") == 0;
    if (!configure && !north && !level) return false;
    serviceEncoders();
    if (!commandIdle() || !controlReady || !poseMotorsStopped()) {
        queueText("CALIBRATION REJECTED: stopped READY required; STOP first\n"); return true;
    }
    if (configure) {
        double yaw = 0, pitch = 0;
        m09::EncoderReference proposed[2];
        if (count != 3 || !parseAngle(tokens[1], yaw) || !parseAngle(tokens[2], pitch) ||
            !proposed[0].configure(yaw) || !proposed[1].configure(pitch)) {
            queueText("CALIBRATION REJECTED: ENCODER_CONFIG signed_yaw_deg_per_rev signed_pitch_deg_per_rev; nonzero magnitude <=360\n");
            return true;
        }
        encoderReferences[0] = proposed[0]; encoderReferences[1] = proposed[1];
        yawPulsesPerDegree = pitchPulsesPerDegree = 0;
        queueText("CALIBRATION CONFIGURED: session only; SET_NORTH and SET_LEVEL required\n");
    } else {
        const unsigned n = level ? 1 : 0;
        if (count != 1 || !encoderReferences[n].setZero(sensorSnapshot.positions[n], millis())) {
            queueText("CALIBRATION REJECTED: configure geometry and wait for fresh valid encoder\n"); return true;
        }
        queueText(level ? "CALIBRATION SET_LEVEL: horizontal reference stored for this session\n" :
                          "CALIBRATION SET_NORTH: north reference stored for this session\n");
    }
    serviceEncoders(); sensorTelemetry(); queueText("M09 READY\n");
    return true;
}
