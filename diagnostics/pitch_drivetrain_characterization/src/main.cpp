#include <Arduino.h>
#include <AS5600.h>
#include <FastAccelStepper.h>
#include <Wire.h>
#include "hardware_config.h"
#include "sensor_support.h"
#include "characterization_math.h"

namespace {
using namespace milestone4;
constexpr int32_t PULSES_PER_MOTOR_REV = 1600; // User: 200 full steps, 8x microsteps.
constexpr int32_t LIMIT_STEPS = 600;          // Positions relative to startup, not move deltas.
constexpr int32_t PRELOAD_STEPS = 50;
constexpr int32_t MEASURE_STEPS = 250;
constexpr int32_t REVERSAL_PROBE_STEPS = 25;
constexpr uint32_t SPEED_HZ = 1000;
constexpr int32_t ACCELERATION = 1000;
constexpr uint32_t ENCODER_INTERVAL_MS = 10;
constexpr uint32_t ENCODER_FAILURE_MS = 500;
constexpr uint32_t DISPLAY_INTERVAL_MS = 750;
constexpr uint32_t STARTUP_MS = 5000;
constexpr uint32_t SETTLE_MIN_MS = 2000;
constexpr uint32_t WINDOW_MS = 1000;
constexpr uint32_t SETTLE_TIMEOUT_MS = 8000;
constexpr uint32_t MOVE_TIMEOUT_MS = 10000;
constexpr uint32_t MAX_WINDOW_SAMPLE_GAP_MS = 100;
constexpr size_t MAX_SEGMENTS = 40;
constexpr size_t REVERSALS = 4;

enum class Phase { STARTUP, BASELINE, MOVING, SETTLING, COMPLETE, ABORTED };
enum class Kind { PRELOAD, MEASURE, REVERSAL_PROBE };
struct Segment { int32_t delta; Kind kind; int8_t reversal; };
struct Snapshot {
    Endpoint measurement;
    int32_t steps = 0;
    uint16_t wrappedRaw = 0;
    uint32_t resetEpoch = 0;
};
struct Record {
    Snapshot before;
    Snapshot after;
    bool began = false;
    bool finished = false;
    bool settled = false;
    int32_t target = 0;
    int32_t endSteps = 0;
    uint32_t encoderSamples = 0;
    uint32_t bnoSamples = 0;
    bool bnoInterrupted = false;
    RatioResult ratio;
};
struct Statistics {
    uint32_t n = 0;
    double sum = 0;
    double sumSquares = 0;
    double minimum = 0;
    double maximum = 0;
    void add(double value)
    {
        if (n == 0) minimum = maximum = value;
        if (value < minimum) minimum = value;
        if (value > maximum) maximum = value;
        ++n; sum += value; sumSquares += value * value;
    }
    double mean() const { return n ? sum / n : 0; }
    double range() const { return n ? maximum - minimum : 0; }
    double sd() const
    {
        if (n < 2) return 0;
        const double variance = (sumSquares - sum * sum / n) / (n - 1);
        return sqrt(variance > 0 ? variance : 0);
    }
};

FastAccelStepperEngine engine;
FastAccelStepper *pitch = nullptr;
AS5600 pitchEncoder(&Wire1); // CONFIRMED motor shaft: Bus B SDA4/SCL5, 0x36.
AS5600 auxiliaryEncoder(&Wire); // Bus A: communication-only, not used for angles/ratios.
DiagnosticBno085 bno;
sh2_SensorValue_t bnoEvent = {};
EulerAngles orientation = {0, 0, 0};
AngleUnwrapper shaft;
BnoHealth bnoHealth;

Segment segments[MAX_SEGMENTS];
Record records[MAX_SEGMENTS];
size_t segmentCount = 0;
size_t currentSegment = 0;
Snapshot initialSnapshot;
Snapshot lastSnapshot;
ReversalTracker reversalTrackers[REVERSALS];
bool reversalStarted[REVERSALS] = {};
bool reversalInterrupted[REVERSALS] = {};
Phase phase = Phase::ABORTED;
bool testStarted = false;
bool finalPrinted = false;
bool pitchEncoderInitialized = false;
bool auxiliaryInitialized = false;
bool bnoInitialized = false;
bool reportEnabled = false;
bool bnoValid = false;
bool pitchEncoderValid = false;
bool motionSinceEncoderSample = false;
bool unwrapValid = true;
bool resetAwaitingFresh = false;
int32_t startingSteps = 0;
int32_t lastEncoderSteps = 0;
uint16_t encoderRaw = 0;
uint8_t encoderMagnet = 0;
uint8_t bnoAccuracy = 0;
uint32_t lastEncoderService = 0;
uint32_t lastEncoderGood = 0;
uint32_t lastAuxiliaryService = 0;
uint32_t lastBnoGood = 0;
uint32_t lastReportAttempt = 0;
uint32_t lastProbe = 0;
uint32_t lastDisplay = 0;
uint32_t deadline = 0;
uint32_t stoppedAt = 0;
uint32_t encoderReads = 0;
uint32_t encoderReadFailures = 0;
uint32_t encoderMagnetFailures = 0;
uint32_t encoderMaxGap = 0;
uint32_t auxiliaryReads = 0;
uint32_t auxiliaryFailures = 0;
uint32_t noEventPolls = 0;
uint32_t invalidBnoVectors = 0;
uint32_t bnoNoAck = 0;
uint32_t bnoResets = 0;
uint32_t bnoTestResets = 0;
uint32_t reportAttempts = 0;
uint32_t reportSuccesses = 0;
uint32_t reportFailures = 0;
uint32_t resetFreshRecoveries = 0;
uint32_t testBnoSamples = 0;
uint32_t testEncoderSamples = 0;
uint32_t displayDuringMotion = 0;
uint32_t settleRetries = 0;
int learnedEncoderSign = 0;
int learnedPitchSign = 0;
bool directionConsistent = true;

bool windowActive = false;
bool windowInterrupted = false;
uint32_t windowStartedAt = 0;
uint32_t windowLastBnoAt = 0;
uint32_t windowLastEncoderAt = 0;
uint32_t windowResetEpoch = 0;
Statistics pitchWindow;
Statistics firstHalf;
Statistics secondHalf;
Statistics shaftWindow;

void abortTest(const char *reason);
void printFinalSummary(bool completed, const char *reason);

const char *passFail(bool ok) { return ok ? "PASS" : "FAIL"; }
const char *signText(int sign) { return sign > 0 ? "+" : sign < 0 ? "-" : "INCONCLUSIVE"; }
const char *phaseText()
{
    switch (phase) {
    case Phase::STARTUP: return "STARTUP";
    case Phase::BASELINE: return "BASELINE";
    case Phase::MOVING: return "MOVING";
    case Phase::SETTLING: return "SETTLING";
    case Phase::COMPLETE: return "COMPLETE";
    case Phase::ABORTED: return "ABORTED";
    }
    return "UNKNOWN";
}
bool reached(uint32_t now, uint32_t when) { return static_cast<int32_t>(now - when) >= 0; }
bool actuallyMoving() { return phase == Phase::MOVING && pitch && pitch->isRunning(); }
bool bnoFresh(uint32_t now)
{
    return bnoInitialized && reportEnabled && bnoValid && now - lastBnoGood <= BNO_STALE_AFTER_MS;
}
bool encoderFresh(uint32_t now)
{
    return pitchEncoderInitialized && pitchEncoderValid && now - lastEncoderGood <= 100;
}
const char *magnetStatusText(uint8_t status)
{
    if (status & 0x10U) return "TOO-WEAK";
    if (status & 0x08U) return "TOO-STRONG";
    if (status & 0x20U) return "DETECTED";
    return "NOT-DETECTED";
}

void stopPitch()
{
    if (pitch) pitch->forceStop();
    delay(25); // Verified queue-drain behavior, preserves the reported stop count.
    digitalWrite(tmp_hardware::PITCH_STEP_PIN, LOW);
    digitalWrite(tmp_hardware::YAW_STEP_PIN, LOW);
    digitalWrite(tmp_hardware::CARRIAGE_STEP_PIN, LOW);
}

void addSegment(int32_t delta, Kind kind, int8_t reversal = -1)
{
    if (segmentCount < MAX_SEGMENTS) segments[segmentCount++] = {delta, kind, reversal};
}
void addReversal(int sign, int8_t group)
{
    for (int i = 0; i < 4; ++i) addSegment(sign * REVERSAL_PROBE_STEPS, Kind::REVERSAL_PROBE, group);
}
void addTraverse(int sign, int8_t group)
{
    for (int i = 0; i < 4; ++i) addSegment(sign * MEASURE_STEPS, Kind::MEASURE, group);
    addSegment(sign * PRELOAD_STEPS, Kind::PRELOAD, group);
}
void createPlan()
{
    addSegment(PRELOAD_STEPS, Kind::PRELOAD);
    addSegment(MEASURE_STEPS, Kind::MEASURE);
    addSegment(MEASURE_STEPS, Kind::MEASURE);
    addReversal(-1, 0); addTraverse(-1, 0);
    addReversal(1, 1); addTraverse(1, 1);
    addReversal(-1, 2); addTraverse(-1, 2);
    addReversal(1, 3);
    addSegment(MEASURE_STEPS, Kind::MEASURE, 3);
    addSegment(MEASURE_STEPS, Kind::MEASURE, 3);
}

void clearWindow()
{
    pitchWindow = {}; firstHalf = {}; secondHalf = {}; shaftWindow = {};
    windowStartedAt = millis();
    windowLastBnoAt = windowLastEncoderAt = windowStartedAt;
    windowResetEpoch = bnoResets;
    windowInterrupted = false;
    windowActive = true;
}

bool enableReport(bool initial)
{
    lastReportAttempt = millis();
    reportEnabled = bno.enableReport(SH2_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);
    if (!initial)
    {
        ++reportAttempts;
        if (reportEnabled) ++reportSuccesses;
        else ++reportFailures;
    }
    return reportEnabled;
}
bool handleReset()
{
    if (!bno.wasReset()) return false;
    ++bnoResets;
    if (testStarted) ++bnoTestResets;
    bnoValid = false;
    resetAwaitingFresh = true;
    windowInterrupted = true;
    if (testStarted && currentSegment < segmentCount) records[currentSegment].bnoInterrupted = true;
    enableReport(false);
    return true;
}
void serviceBno()
{
    if (!bnoInitialized) return;
    bnoHealth.observeTime(millis(), testStarted);
    handleReset();
    if (!reportEnabled && millis() - lastReportAttempt >= 500) enableReport(false);
    bnoEvent = {};
    const bool gotEvent = bno.getSensorEvent(&bnoEvent);
    const bool resetDuringPoll = handleReset();
    const uint32_t now = millis();
    bnoHealth.observeTime(now, testStarted); // Preserve a gap before accepting its recovery.
    if (!gotEvent) { ++noEventPolls; return; }
    if (resetDuringPoll || !reportEnabled || bnoEvent.sensorId != SH2_ROTATION_VECTOR) return;
    EulerAngles result;
    if (!quaternionToEuler(bnoEvent.un.rotationVector, result)) { ++invalidBnoVectors; return; }
    bnoHealth.recordFresh(now, testStarted);
    orientation = result;
    bnoAccuracy = bnoEvent.status;
    bnoValid = true;
    lastBnoGood = now;
    if (resetAwaitingFresh) { ++resetFreshRecoveries; resetAwaitingFresh = false; }
    if (testStarted) ++testBnoSamples;
    if (actuallyMoving()) ++records[currentSegment].bnoSamples;
    if (windowActive)
    {
        if (now - windowLastBnoAt > MAX_WINDOW_SAMPLE_GAP_MS) windowInterrupted = true;
        windowLastBnoAt = now;
        pitchWindow.add(result.pitch);
        if (now - windowStartedAt < WINDOW_MS / 2) firstHalf.add(result.pitch);
        else secondHalf.add(result.pitch);
    }
}

void servicePitchEncoder()
{
    const uint32_t serviceAt = millis();
    if (serviceAt - lastEncoderService < ENCODER_INTERVAL_MS) return;
    lastEncoderService = serviceAt;
    bool ok = pitchEncoderInitialized && pitchEncoder.isConnected();
    uint16_t raw = encoderRaw;
    uint8_t status = encoderMagnet;
    if (ok) { raw = pitchEncoder.rawAngle(); ok = pitchEncoder.lastError() == AS5600_OK; }
    if (ok) { status = pitchEncoder.readStatus(); ok = pitchEncoder.lastError() == AS5600_OK; }
    if (!ok) { ++encoderReadFailures; pitchEncoderValid = false; return; }
    encoderRaw = raw;
    encoderMagnet = status;
    ++encoderReads;
    if (!(status & 0x20U) || (status & 0x18U)) ++encoderMagnetFailures;
    const uint32_t now = millis();
    const int32_t steps = pitch ? pitch->getCurrentPosition() : 0;
    const uint32_t gap = now - lastEncoderGood;
    if (shaft.initialized && gap > encoderMaxGap) encoderMaxGap = gap;
    if (testStarted && gap >= ENCODER_FAILURE_MS)
    {
        abortTest("Required pitch AS5600 acquisition gap reached 500 ms before recovery");
        return;
    }
    if (!shaft.initialized) shaft.begin(raw);
    else
    {
        // >half a motor turn between samples has no unique modulo solution.
        // The elapsed check covers a reversal that could hide in net step delta.
        if (motionSinceEncoderSample &&
            (gap >= 1600 || abs(steps - lastEncoderSteps) >= PULSES_PER_MOTOR_REV / 2))
        {
            unwrapValid = false;
            abortTest("Ambiguous AS5600 unwrap: acquisition gap could contain half a turn");
            return;
        }
        if (!shaft.update(raw))
        {
            unwrapValid = false;
            abortTest("Ambiguous AS5600 half-turn or invalid raw sample");
            return;
        }
    }
    pitchEncoderValid = true;
    lastEncoderGood = now;
    lastEncoderSteps = steps;
    motionSinceEncoderSample = actuallyMoving();
    if (testStarted) ++testEncoderSamples;
    if (actuallyMoving()) ++records[currentSegment].encoderSamples;
    if (windowActive)
    {
        if (now - windowLastEncoderAt > MAX_WINDOW_SAMPLE_GAP_MS) windowInterrupted = true;
        windowLastEncoderAt = now;
        shaftWindow.add(shaft.accumulatedDegrees());
    }
}

void serviceAuxiliary()
{
    if (millis() - lastAuxiliaryService < 100) return;
    lastAuxiliaryService = millis();
    bool ok = auxiliaryInitialized && auxiliaryEncoder.isConnected();
    if (ok) { auxiliaryEncoder.rawAngle(); ok = auxiliaryEncoder.lastError() == AS5600_OK; }
    if (ok) { auxiliaryEncoder.readStatus(); ok = auxiliaryEncoder.lastError() == AS5600_OK; }
    if (ok) ++auxiliaryReads;
    else ++auxiliaryFailures;
}

void checkSafety()
{
    if (!testStarted || finalPrinted) return;
    const uint32_t now = millis();
    bnoHealth.observeTime(now, true);
    if (bnoHealth.persistentTestFailure)
        abortTest("Persistent required BNO085 outage: gap reached 2000 ms");
    else if (now - lastEncoderGood >= ENCODER_FAILURE_MS)
        abortTest("Required pitch AS5600 acquisition gap reached 500 ms before recovery");
    else if (bnoValid && (fabs(orientation.pitch) >= 80.0 ||
             fabs(orientation.pitch - initialSnapshot.measurement.pitchMean) > 12.0))
        abortTest("Pitch diagnostic guard exceeded: absolute 80 deg or relative 12 deg");
}
void serviceSensors()
{
    servicePitchEncoder();
    if (finalPrinted) return;
    checkSafety();
    if (finalPrinted) return;
    serviceBno();
    checkSafety();
    if (finalPrinted) return;
    servicePitchEncoder();
    if (finalPrinted) return;
    serviceAuxiliary();
    if (bnoInitialized && millis() - lastProbe >= 500)
    {
        lastProbe = millis();
        Wire1.beginTransmission(0x4A);
        if (Wire1.endTransmission() != 0) ++bnoNoAck;
        bnoHealth.check(millis(), testStarted);
    }
    checkSafety();
}

bool captureSnapshot(Snapshot &snapshot)
{
    const uint32_t now = millis();
    const double drift = secondHalf.mean() - firstHalf.mean();
    if (windowInterrupted || windowResetEpoch != bnoResets ||
        pitchWindow.n < 30 || shaftWindow.n < 30 || !firstHalf.n || !secondHalf.n ||
        pitchWindow.sd() > 0.15 || pitchWindow.range() > 0.5 || fabs(drift) > 0.2 ||
        shaftWindow.range() > 1.0 || !encoderFresh(now) || !bnoFresh(now) ||
        now - lastBnoGood > MAX_WINDOW_SAMPLE_GAP_MS ||
        now - windowLastEncoderAt > MAX_WINDOW_SAMPLE_GAP_MS) return false;
    snapshot.steps = pitch->getCurrentPosition();
    snapshot.wrappedRaw = encoderRaw;
    snapshot.resetEpoch = bnoResets;
    snapshot.measurement.motorDeg = shaftWindow.mean();
    snapshot.measurement.pitchMean = pitchWindow.mean();
    snapshot.measurement.pitchStddev = pitchWindow.sd();
    snapshot.measurement.pitchRange = pitchWindow.range();
    snapshot.measurement.pitchDrift = drift;
    snapshot.measurement.count = pitchWindow.n;
    snapshot.measurement.valid = true;
    return true;
}

void printTelemetry()
{
    if (actuallyMoving()) ++displayDuringMotion;
    Serial.print("STATE "); Serial.print(phaseText());
    Serial.print(" segment="); Serial.print(currentSegment + 1); Serial.print('/'); Serial.print(segmentCount);
    Serial.print(" pitch_steps="); Serial.print(pitch ? pitch->getCurrentPosition() : 0);
    Serial.print(" AS_raw="); Serial.print(encoderRaw);
    Serial.print(" wrapped_deg="); Serial.print(encoderRaw * AS5600_RAW_TO_DEGREES, 2);
    Serial.print(" unwrapped_deg="); Serial.print(shaft.accumulatedDegrees(), 2);
    Serial.print(" reads="); Serial.println(encoderReads);
    Serial.print("  BNO pitch="); Serial.print(orientation.pitch, 2);
    Serial.print(" age_ms="); Serial.print(millis() - bnoHealth.lastFreshOrStartMs);
    Serial.print(" fresh="); Serial.print(bnoHealth.freshSamples);
    Serial.print(" stale_events="); Serial.print(bnoHealth.staleEvents);
    Serial.print(" resets="); Serial.print(bnoResets);
    Serial.print(" accuracy="); Serial.println(bnoAccuracy);
}

void startSegment()
{
    if (!encoderFresh(millis()) || !bnoFresh(millis()) || resetAwaitingFresh)
    { abortTest("Required sensors not ready for next segment"); return; }
    Segment &segment = segments[currentSegment];
    Record &record = records[currentSegment];
    record.before = lastSnapshot;
    record.target = pitch->getCurrentPosition() + segment.delta;
    if (abs(record.target - startingSteps) > LIMIT_STEPS)
    { abortTest("Plan exceeded the user-authorized +/-1000 step envelope"); return; }
    if (segment.kind == Kind::REVERSAL_PROBE && !reversalStarted[segment.reversal])
    {
        reversalStarted[segment.reversal] = true;
        const int commandSign = directionSign(segment.delta, 0);
        reversalTrackers[segment.reversal].begin(lastSnapshot.measurement,
            learnedEncoderSign * commandSign, learnedPitchSign * commandSign);
    }
    windowActive = false;
    Serial.print("MOVE START segment="); Serial.print(currentSegment + 1);
    Serial.print(" delta="); Serial.print(segment.delta);
    Serial.print(" target="); Serial.println(record.target);
    if (pitch->move(segment.delta) != MOVE_OK)
    { abortTest("FastAccelStepper rejected pitch move"); return; }
    record.began = true;
    motionSinceEncoderSample = true;
    phase = Phase::MOVING;
    deadline = millis() + MOVE_TIMEOUT_MS;
}

void recordSettledSegment(const Snapshot &after)
{
    Record &record = records[currentSegment];
    const Segment &segment = segments[currentSegment];
    record.after = after;
    record.settled = true;
    if (record.before.resetEpoch != after.resetEpoch) record.bnoInterrupted = true;
    if (!record.bnoInterrupted) record.ratio = calculateRatio(record.before.measurement, after.measurement);
    if (segment.kind == Kind::MEASURE && record.ratio.valid)
    {
        const int commandSign = directionSign(segment.delta, 0);
        const int encoderSign = directionSign(record.ratio.motorDelta, 1.0) * commandSign;
        const int cradleSign = directionSign(record.ratio.pitchDelta, record.ratio.noiseGateDeg) * commandSign;
        if (learnedEncoderSign == 0) learnedEncoderSign = encoderSign;
        if (learnedPitchSign == 0) learnedPitchSign = cradleSign;
        if (encoderSign != learnedEncoderSign || cradleSign != learnedPitchSign) directionConsistent = false;
    }
    if (segment.reversal >= 0)
    {
        const size_t group = segment.reversal;
        if (record.bnoInterrupted) reversalInterrupted[group] = true;
        // The first large move may CONFIRM a detection from a small probe.
        // If it is the first detection itself, the wide bracket is labeled coarse.
        if (!reversalInterrupted[group] && !reversalTrackers[group].result().confirmed)
            reversalTrackers[group].observe(after.measurement);
    }
    Serial.print("SEGMENT RESULT "); Serial.print(currentSegment + 1);
    Serial.print(" cmd/actual_steps="); Serial.print(segment.delta); Serial.print('/');
    Serial.print(record.endSteps - record.before.steps);
    Serial.print(" motor_deg/rev=");
    const double motor = after.measurement.motorDeg - record.before.measurement.motorDeg;
    Serial.print(motor, 2); Serial.print('/'); Serial.print(motor / 360.0, 4);
    Serial.print(" cradle_deg="); Serial.print(after.measurement.pitchMean - record.before.measurement.pitchMean, 2);
    Serial.print(" ratio=");
    if (record.ratio.valid) Serial.print(record.ratio.magnitude, 1); else Serial.print("INCONCLUSIVE");
    if (record.ratio.valid)
    {
        Serial.print(" sensitivity="); Serial.print(record.ratio.lower, 1);
        Serial.print(".."); Serial.print(record.ratio.upper, 1);
    }
    Serial.print(" motor/cradle_sign="); Serial.print(signText(directionSign(motor, 1.0))); Serial.print('/');
    Serial.print(signText(directionSign(after.measurement.pitchMean - record.before.measurement.pitchMean,
                                      record.ratio.noiseGateDeg > 0 ? record.ratio.noiseGateDeg : 1.0)));
    Serial.print(" samples_AS/BNO="); Serial.print(record.encoderSamples); Serial.print('/'); Serial.print(record.bnoSamples);
    Serial.print(" settled_sd/range/drift="); Serial.print(after.measurement.pitchStddev, 2); Serial.print('/');
    Serial.print(after.measurement.pitchRange, 2); Serial.print('/'); Serial.println(after.measurement.pitchDrift, 2);
    lastSnapshot = after;
}

void serviceState()
{
    const uint32_t now = millis();
    if (phase == Phase::STARTUP)
    {
        if (!reached(now, deadline)) return;
        if (!encoderFresh(now) || !bnoFresh(now))
        { abortTest("Startup required sensor/magnet/freshness gate failed; no motion"); return; }
        phase = Phase::BASELINE;
        stoppedAt = now;
        clearWindow();
        return;
    }
    if (phase == Phase::MOVING)
    {
        if (pitch->isRunning())
        {
            if (reached(now, deadline)) abortTest("Pitch move exceeded 10000 ms watchdog");
            return;
        }
        Record &record = records[currentSegment];
        record.endSteps = pitch->getCurrentPosition();
        record.finished = record.endSteps == record.target;
        if (!record.finished) { abortTest("Pitch did not reach commanded firmware count"); return; }
        Serial.print("MOVE END segment="); Serial.print(currentSegment + 1);
        Serial.print(" steps="); Serial.println(record.endSteps);
        phase = Phase::SETTLING;
        stoppedAt = now;
        windowActive = false;
        return;
    }
    if (phase != Phase::BASELINE && phase != Phase::SETTLING) return;
    if (now - stoppedAt >= SETTLE_TIMEOUT_MS)
    { abortTest("No stable fresh endpoint within 8000 ms settling deadline"); return; }
    if (!windowActive)
    {
        if (now - stoppedAt >= SETTLE_MIN_MS) clearWindow();
        return;
    }
    if (now - windowStartedAt < WINDOW_MS) return;
    Snapshot snapshot;
    if (!captureSnapshot(snapshot)) { ++settleRetries; clearWindow(); return; }
    windowActive = false;
    if (phase == Phase::BASELINE)
    {
        if (fabs(snapshot.measurement.pitchMean) > 65.0)
        { abortTest("Start nearer level: baseline pitch exceeds +/-65 deg preflight limit"); return; }
        initialSnapshot = lastSnapshot = snapshot;
        startingSteps = snapshot.steps;
        Serial.print("BASELINE steps="); Serial.print(startingSteps);
        Serial.print(" AS_raw="); Serial.print(snapshot.wrappedRaw);
        Serial.print(" motor_deg="); Serial.print(snapshot.measurement.motorDeg, 2);
        Serial.print(" BNO_pitch="); Serial.println(snapshot.measurement.pitchMean, 2);
        testStarted = true;
        startSegment();
        return;
    }
    recordSettledSegment(snapshot);
    ++currentSegment;
    if (currentSegment < segmentCount) startSegment();
    else
    {
        stopPitch();
        bnoHealth.observeTime(millis(), true);
        phase = Phase::COMPLETE;
        printFinalSummary(true, "Fixed characterization sequence completed");
    }
}

void printRatioGroup(const char *label, const Statistics &ratios, double motorSum, double cradleSum, double gateSum)
{
    Serial.print("  "); Serial.print(label); Serial.print(": ");
    if (ratios.n == 0 || cradleSum <= 0) { Serial.println("INCONCLUSIVE"); return; }
    Serial.print(motorSum / cradleSum, 1);
    Serial.print(":1 eligible_legs="); Serial.print(ratios.n);
    Serial.print(" leg_range="); Serial.print(ratios.minimum, 1); Serial.print(".."); Serial.print(ratios.maximum, 1);
    Serial.print(" leg_sd="); Serial.print(ratios.sd(), 1);
    Serial.print(" sensitivity="); Serial.print(motorSum / (cradleSum + gateSum), 1); Serial.print("..");
    if (cradleSum > gateSum) Serial.println(motorSum / (cradleSum - gateSum), 1);
    else Serial.println("UNBOUNDED");
}

void printFinalSummary(bool completed, const char *reason)
{
    if (finalPrinted) return;
    finalPrinted = true;
    windowActive = false;
    const uint32_t now = millis();
    bool motionOk = completed && pitch && pitch->getCurrentPosition() == startingSteps;
    bool coexistenceOk = completed && testEncoderSamples > 0 && testBnoSamples > 0;
    Statistics forwardRatios, reverseRatios;
    double forwardMotor = 0, forwardCradle = 0, reverseMotor = 0, reverseCradle = 0;
    double forwardGate = 0, reverseGate = 0;
    Statistics repeatedMotorDifference, repeatedCradleDifference;
    int32_t commandedNet = 0;
    uint32_t commandedTravel = 0;
    unsigned finishedMoves = 0;
    unsigned ratioExcluded = 0;
    for (size_t i = 0; i < segmentCount; ++i)
    {
        const Record &record = records[i];
        motionOk = motionOk && record.began && record.finished && record.settled;
        coexistenceOk = coexistenceOk && record.encoderSamples > 0 && record.bnoSamples > 0;
        if (record.began) { commandedNet += segments[i].delta; commandedTravel += abs(segments[i].delta); }
        if (record.finished) ++finishedMoves;
        if (segments[i].kind != Kind::MEASURE) continue;
        if (!record.ratio.valid || !record.settled) { ++ratioExcluded; continue; }
        if (segments[i].delta > 0)
        {
            forwardRatios.add(record.ratio.magnitude);
            forwardMotor += fabs(record.ratio.motorDelta); forwardCradle += fabs(record.ratio.pitchDelta);
            forwardGate += record.ratio.noiseGateDeg;
        }
        else
        {
            reverseRatios.add(record.ratio.magnitude);
            reverseMotor += fabs(record.ratio.motorDelta); reverseCradle += fabs(record.ratio.pitchDelta);
            reverseGate += record.ratio.noiseGateDeg;
        }
        // Compare identical commanded start/end positions, not just all legs.
        for (size_t j = 0; j < i; ++j)
        {
            const Record &prior = records[j];
            if (segments[j].kind == Kind::MEASURE && prior.settled && prior.ratio.valid &&
                prior.before.steps == record.before.steps && prior.target == record.target)
            {
                repeatedMotorDifference.add(fabs(prior.ratio.motorDelta - record.ratio.motorDelta));
                repeatedCradleDifference.add(fabs(prior.ratio.pitchDelta - record.ratio.pitchDelta));
            }
        }
    }
    const bool encoderOk = unwrapValid && encoderFresh(now) && testEncoderSamples > 0;
    const bool bnoOk = bnoFresh(now) && !resetAwaitingFresh && !bnoHealth.persistentTestFailure && testBnoSamples > 0;
    const bool reductionOk = forwardRatios.n >= 2 && reverseRatios.n >= 2 &&
                             learnedEncoderSign && learnedPitchSign && directionConsistent;
    const bool passed = motionOk && coexistenceOk && encoderOk && bnoOk && reductionOk;
    Serial.println();
    Serial.println("=== MILESTONE 4 PITCH CHARACTERIZATION SUMMARY ===");
    Serial.print("Pitch motor motion: "); Serial.println(passFail(motionOk));
    Serial.print("Stepper accepted_net/travel/reported_delta: "); Serial.print(commandedNet); Serial.print('/');
    Serial.print(commandedTravel); Serial.print('/'); Serial.println(pitch ? pitch->getCurrentPosition() - startingSteps : 0);
    Serial.print("  segments_completed="); Serial.print(finishedMoves); Serial.print('/'); Serial.print(segmentCount);
    Serial.print(" final_steps="); Serial.println(pitch ? pitch->getCurrentPosition() : 0);
    Serial.println("  Yaw and carriage: no stepper objects or move commands; STEP outputs held LOW.");
    Serial.print("AS5600 motor encoder Bus B 0x36: "); Serial.println(passFail(encoderOk));
    Serial.print("  raw/wrapped_deg="); Serial.print(encoderRaw); Serial.print('/'); Serial.println(encoderRaw * AS5600_RAW_TO_DEGREES, 2);
    Serial.print("  unwrapped_net_deg/rev=");
    const double motorNet = shaft.accumulatedDegrees() - initialSnapshot.measurement.motorDeg;
    Serial.print(motorNet, 2); Serial.print('/'); Serial.println(motorNet / 360.0, 4);
    Serial.print("  sampled_absolute_travel_deg/rev="); Serial.print(shaft.travelDegrees(), 2); Serial.print('/');
    Serial.println(shaft.travelDegrees() / 360.0, 4);
    Serial.print("  reads/read_failures/magnet_failures/max_gap_ms="); Serial.print(encoderReads); Serial.print('/');
    Serial.print(encoderReadFailures); Serial.print('/'); Serial.print(encoderMagnetFailures); Serial.print('/'); Serial.println(encoderMaxGap);
    Serial.print("  magnet_status="); Serial.println(magnetStatusText(encoderMagnet));
    if (encoderMagnetFailures)
        Serial.println("  WARNING: non-ideal AS5600 magnet status is advisory; raw-angle integrity remains mandatory.");
    Serial.print("AS5600 Bus A optional communication reads/failures: "); Serial.print(auxiliaryReads); Serial.print('/'); Serial.println(auxiliaryFailures);
    Serial.print("BNO085 cradle health: "); Serial.println(passFail(bnoOk));
    Serial.print("  settled_final_pitch/net_displacement_deg=");
    if (completed) { Serial.print(lastSnapshot.measurement.pitchMean, 2); Serial.print('/');
        Serial.println(lastSnapshot.measurement.pitchMean - initialSnapshot.measurement.pitchMean, 2); }
    else Serial.println("UNAVAILABLE (sequence aborted)");
    Serial.print("  fresh/stale_events/recovered/stale_checks/max_gap_ms="); Serial.print(bnoHealth.freshSamples); Serial.print('/');
    Serial.print(bnoHealth.staleEvents); Serial.print('/'); Serial.print(bnoHealth.recoveredStaleEvents); Serial.print('/');
    Serial.print(bnoHealth.staleChecks); Serial.print('/'); Serial.println(bnoHealth.maxGapMs);
    Serial.print("  resets_total/test/reenable_attempts/ok/fail/fresh_recoveries=");
    Serial.print(bnoResets); Serial.print('/'); Serial.print(bnoTestResets); Serial.print('/'); Serial.print(reportAttempts); Serial.print('/');
    Serial.print(reportSuccesses); Serial.print('/'); Serial.print(reportFailures); Serial.print('/'); Serial.println(resetFreshRecoveries);
    Serial.print("  invalid_vectors/no_ACK/write_failures/no_event_polls="); Serial.print(invalidBnoVectors); Serial.print('/');
    Serial.print(bnoNoAck); Serial.print('/'); Serial.print(DiagnosticBno085::writeFailures); Serial.print('/'); Serial.println(noEventPolls);
    Serial.println("  No-event polls are not an exact low-level read failure count.");
    Serial.print("Direction positive command -> motor AS5600: "); Serial.println(signText(learnedEncoderSign));
    Serial.print("Direction positive command -> cradle BNO pitch: "); Serial.println(signText(learnedPitchSign));
    Serial.print("  consistent across qualified legs: "); Serial.println(directionConsistent ? "YES" : "NO");
    Serial.print("Measured reduction: "); Serial.println(reductionOk ? "AVAILABLE (approximate)" : "INCONCLUSIVE");
    Serial.println("  Nominal expectation only: approximately 15:1");
    printRatioGroup("forward", forwardRatios, forwardMotor, forwardCradle, forwardGate);
    printRatioGroup("reverse", reverseRatios, reverseMotor, reverseCradle, reverseGate);
    Serial.print("  combined: ");
    if (forwardCradle + reverseCradle > 0)
    {
        const double motor = forwardMotor + reverseMotor;
        const double cradle = forwardCradle + reverseCradle;
        const double gate = forwardGate + reverseGate;
        Serial.print(motor / cradle, 1); Serial.print(":1 sensitivity=");
        Serial.print(motor / (cradle + gate), 1); Serial.print("..");
        if (cradle > gate) Serial.println(motor / (cradle - gate), 1); else Serial.println("UNBOUNDED");
    }
    else Serial.println("INCONCLUSIVE");
    Serial.print("  excluded main legs="); Serial.print(ratioExcluded); Serial.print(" settle_retries="); Serial.println(settleRetries);
    Serial.println("  Ratios include possible residual take-up; window noise is not absolute angular accuracy.");
    Serial.println("  Sensitivity bounds use +/- summed noise gates; they are NOT confidence intervals.");
    Serial.print("Repeatability matched-leg pairs: "); Serial.println(repeatedMotorDifference.n);
    if (repeatedMotorDifference.n)
    {
        Serial.print("  max motor/cradle displacement difference_deg=");
        Serial.print(repeatedMotorDifference.maximum, 2); Serial.print('/'); Serial.println(repeatedCradleDifference.maximum, 2);
    }
    else Serial.println("  INCONCLUSIVE: no qualified repeated start/end pair");
    Serial.println("Reversal/backlash: detection evidence only; true mechanical backlash INCONCLUSIVE.");
    for (size_t i = 0; i < REVERSALS; ++i)
    {
        const ReversalResult &result = reversalTrackers[i].result();
        Serial.print("  reversal "); Serial.print(i + 1); Serial.print(": ");
        if (!reversalStarted[i] || reversalInterrupted[i] || !result.confirmed)
        { Serial.println("INCONCLUSIVE (noise, unconfirmed direction/response, or reset)"); continue; }
        Serial.print("response confirmed; threshold-crossing motor_deg="); Serial.print(result.lowerMotorDeg, 1);
        Serial.print(".."); Serial.print(result.upperMotorDeg, 1);
        Serial.print(" BNO threshold_deg="); Serial.print(result.thresholdDeg, 2);
        Serial.println(" (includes detection resolution/compliance; not a lash estimate)");
    }
    Serial.print("Sensor coexistence during motion: "); Serial.println(passFail(coexistenceOk));
    Serial.print("  motion_display_snapshots="); Serial.println(displayDuringMotion);
    Serial.print("Sensor warnings recorded: ");
    Serial.println((encoderReadFailures || encoderMagnetFailures || bnoHealth.staleEvents || bnoTestResets ||
        invalidBnoVectors || bnoNoAck || reportFailures || DiagnosticBno085::writeFailures) ? "YES - review counters" : "NONE");
    Serial.print("Reason: "); Serial.println(reason);
    Serial.println("No closed-loop control, homing, or automatic restart. Telemetry now stops.");
    Serial.println("Physical verification and measured drivetrain results remain pending user review.");
    Serial.print("FINAL RESULT: "); Serial.println(passFail(passed));
    Serial.println("=================================================");
    Serial.flush();
}

void abortTest(const char *reason)
{
    if (finalPrinted) return;
    stopPitch();
    phase = Phase::ABORTED;
    printFinalSummary(false, reason);
}

} // namespace

void setup()
{
    Serial.begin(115200);
    delay(500);
    for (const uint8_t pin : {tmp_hardware::YAW_STEP_PIN, tmp_hardware::PITCH_STEP_PIN, tmp_hardware::CARRIAGE_STEP_PIN})
    { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
    Serial.println("TMP MILESTONE 4 - PITCH DRIVETRAIN CHARACTERIZATION");
    Serial.println("Pitch DIR26 STEP12. Pitch AS5600=Bus B SDA4 SCL5 0x36; BNO085 same bus 0x4A.");
    Serial.println("AS5600 measures MOTOR SHAFT; BNO085 measures CRADLE. Bus A is communication-only.");
    Serial.println("1600 STEP pulses/motor rev (200 full steps, 8x microsteps), nominal reduction about 15:1.");
    Serial.println("Conservative excursion: 0 -> +550 -> -600 -> +550 -> -600 -> 0; <=250 pulses/measurement segment.");
    Serial.println("Nominal cradle travel +/-7.5 deg, 15 deg end-to-end. This is not a calibrated travel limit.");
    Serial.println("Clear both directions; keep clear. X aborts. Power removal is the physical emergency stop.");
    Serial.println("WARNING: no motion for at least 5 seconds, then a stable baseline is required.");
    createPlan();
    engine.init();
    pitch = engine.stepperConnectToPin(tmp_hardware::PITCH_STEP_PIN);
    if (!pitch) { abortTest("Pitch FastAccelStepper initialization failed"); return; }
    pitch->setDirectionPin(tmp_hardware::PITCH_DIR_PIN, true, 200);
    if (pitch->setSpeedInHz(SPEED_HZ) != 0 || pitch->setAcceleration(ACCELERATION) != 0)
    { abortTest("Pitch speed/acceleration configuration rejected"); return; }
    const bool busAOk = Wire.begin(tmp_hardware::I2C_BUS_A_SDA_PIN, tmp_hardware::I2C_BUS_A_SCL_PIN, 100000);
    if (busAOk) { Wire.setTimeOut(50); auxiliaryInitialized = auxiliaryEncoder.begin() && auxiliaryEncoder.isConnected(); }
    if (!Wire1.begin(tmp_hardware::I2C_BUS_B_SDA_PIN, tmp_hardware::I2C_BUS_B_SCL_PIN, 100000))
    { abortTest("Required I2C Bus B initialization failed"); return; }
    Wire1.setTimeOut(50);
    pitchEncoderInitialized = pitchEncoder.begin() && pitchEncoder.isConnected();
    if (!pitchEncoderInitialized) { abortTest("Pitch AS5600 0x36 initialization failed"); return; }
    Wire1.beginTransmission(0x4A);
    if (Wire1.endTransmission() != 0) { abortTest("Required BNO085 at Bus B 0x4A did not ACK"); return; }
    bnoInitialized = bno.begin_I2C(0x4A, &Wire1);
    if (!bnoInitialized || !enableReport(true)) { abortTest("BNO085 initialization/report enabling failed"); return; }
    bnoHealth.lastFreshOrStartMs = millis();
    lastEncoderGood = millis();
    phase = Phase::STARTUP;
    deadline = millis() + STARTUP_MS;
}

void loop()
{
    if (finalPrinted) { delay(20); return; }
    while (Serial.available())
    {
        const int input = Serial.read();
        if (input == 'X' || input == 'x') { abortTest("Operator X abort"); return; }
    }
    serviceSensors();
    if (finalPrinted) return;
    serviceState();
    if (!finalPrinted && millis() - lastDisplay >= DISPLAY_INTERVAL_MS)
    { lastDisplay = millis(); printTelemetry(); }
    delay(1);
}
