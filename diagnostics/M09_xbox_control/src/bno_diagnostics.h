#pragma once
#include "sensor_support.h"

// Observation only: never feeds a motor, reference or watchdog decision.
// Counts events returned to the application, not every report inside SH-2.
struct BnoDiagnostics {
    uint32_t events = 0, rotations = 0, otherReports = 0, statusChanges = 0;
    uint32_t statusCounts[4] = {};
    uint8_t lastReport = 0, lastStatus = 0, lastSequence = 0;
    uint64_t lastTimestamp = 0;
    uint32_t lastEventAt = 0, rotationAt = 0;
    uint8_t status = 0, sequence = 0;
    uint64_t timestamp = 0;
    sh2_RotationVectorWAcc_t rotation = {0, 0, 0, 0, 0};
    milestone4::EulerAngles euler = {NAN, NAN, NAN};
    bool eulerValid = false, accepted = false;
    bool plausible = false;
    uint32_t malformed = 0;
    double normSquared = NAN;
    const char *reason = "no_event";

    void observe(const sh2_SensorValue_t &event, uint32_t now) {
        ++events;
        lastReport = event.sensorId; lastStatus = event.status; lastSequence = event.sequence;
        lastTimestamp = event.timestamp; lastEventAt = now;
        if (event.sensorId != SH2_ROTATION_VECTOR) { ++otherReports; return; }
        if (rotations && status != event.status) ++statusChanges;
        ++rotations;
        if (event.status < 4) ++statusCounts[event.status];
        status = event.status; sequence = event.sequence; timestamp = event.timestamp;
        rotation = event.un.rotationVector; rotationAt = now;
        normSquared = double(rotation.real)*rotation.real + double(rotation.i)*rotation.i +
            double(rotation.j)*rotation.j + double(rotation.k)*rotation.k;
        // Diagnostic label only. Do not alter acceptance, feedback or watchdogs.
        plausible = isfinite(normSquared) && fabs(normSquared - 1.0) <= 0.05 &&
            isfinite(rotation.accuracy) && rotation.accuracy >= 0;
        if (!plausible) ++malformed;
        euler = {NAN, NAN, NAN}; eulerValid = accepted = false; reason = "not_processed";
    }
};
