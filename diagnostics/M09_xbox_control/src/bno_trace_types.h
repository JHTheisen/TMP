#pragma once
#include <stdint.h>
#include <cmath>
struct BnoTraceEvent {
    const char *kind = "";
    uint32_t atUs = 0, atMs = 0, startUs = 0, durationUs = 0, contextAtMs = 0;
    uint32_t resetEvent = 0, a = 0, b = 0, c = 0;
    int result = 0;
    uint32_t bnoAge = 0, encoderAge = 0, encoderAttempt = 0, encoderFailures = 0;
    uint32_t lastIoStart = 0, lastIoDuration = 0;
    int lastIoResult = 0;
    unsigned lastIoStage = 0;
    bool isFresh = false, manual = false, pose = false;
    const char *phase = "";
    int yaw = 0, pitch = 0, carriage = 0;
    long yawMilliHz = 0, pitchMilliHz = 0, carriageMilliHz = 0;
    double normSquared = NAN, headingAccuracy = NAN;
    unsigned rawStatus = 0, sequence = 0;
};
struct BnoIoStatistics {
    uint32_t dropped = 0, resetEvents = 0, productGeneration = 0, queriedGeneration = 0;
    uint32_t ioCalls = 0, ioFailures = 0, ioSlow = 0, ioMaxUs = 0;
    uint32_t lastIoStart = 0, lastIoDuration = 0;
    unsigned lastIoStage = 0;
    int lastIoResult = 0;
    uint32_t acquireMaxUs = 0, encoderMaxUs = 0;
    uint32_t queryAt = 0, queryGeneration = 0, queryResponses = 0, encoderTraceAt = 0;
    bool queryPending = false;
};

