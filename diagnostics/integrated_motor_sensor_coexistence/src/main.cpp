#include <Arduino.h>
#include <AS5600.h>
#include <Adafruit_BNO08x.h>
#include <FastAccelStepper.h>
#include <Wire.h>

#include "hardware_config.h"
#include "diagnostic_health.h"

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;

constexpr int32_t TEST_STEPS = 500;
constexpr uint32_t TEST_SPEED_HZ = 1000;
constexpr int32_t TEST_ACCELERATION_STEPS_PER_SECOND_SQUARED = 1000;
constexpr uint16_t DIRECTION_CHANGE_DELAY_US = 200;
constexpr uint32_t STARTUP_DELAY_MS = 5000;
constexpr uint32_t DIRECTION_PAUSE_MS = 2000;
constexpr uint32_t AXIS_PAUSE_MS = 3000;
constexpr uint32_t MOVE_TIMEOUT_MS = 10000;

constexpr uint32_t I2C_FREQUENCY_HZ = 100000;
constexpr uint16_t I2C_TIMEOUT_MS = 50;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 750;
constexpr uint32_t SENSOR_SERVICE_INTERVAL_MS = 50;
constexpr uint32_t BNO_PROBE_INTERVAL_MS = 500;
constexpr uint32_t BNO_REPORT_RETRY_MS = 500;
constexpr uint32_t SENSOR_READY_MAX_AGE_MS = 500;
using namespace milestone3;

constexpr uint8_t AS5600_ADDRESS = 0x36;
constexpr uint8_t BNO085_PRIMARY_ADDRESS = 0x4A;
constexpr uint8_t BNO085_ALTERNATE_ADDRESS = 0x4B;

struct Axis {
    const char *name;
    uint8_t stepPin;
    uint8_t dirPin;
    FastAccelStepper *stepper;
    bool initialized;
};

struct I2cBus {
    const char *name;
    TwoWire *wire;
    uint8_t sdaPin;
    uint8_t sclPin;
    bool initialized;
};

struct EncoderDevice {
    EncoderDevice(I2cBus *deviceBus, AS5600 *deviceSensor, bool ready)
        : bus(deviceBus), sensor(deviceSensor), initialized(ready) {}
    I2cBus *bus;
    AS5600 *sensor;
    bool initialized;
    bool readingValid = false;
    uint16_t rawAngle = 0;
    uint8_t magnetStatus = 0;
    uint32_t lastReadMs = 0;
    uint32_t successfulReads = 0;
    uint32_t readFailures = 0;
    uint32_t testSuccessfulReads = 0;
    uint32_t testReadFailures = 0;
};

struct EulerAngles {
    float heading;
    float pitch;
    float roll;
};

enum class MotionPhase : uint8_t {
    STARTUP_WARNING,
    MOVING_FORWARD,
    DIRECTION_PAUSE,
    MOVING_REVERSE,
    AXIS_PAUSE,
    COMPLETE,
    ABORTED,
};

FastAccelStepperEngine engine = FastAccelStepperEngine();

Axis yaw = {
    "YAW",
    tmp_hardware::YAW_STEP_PIN,
    tmp_hardware::YAW_DIR_PIN,
    nullptr,
    false,
};

Axis pitch = {
    "PITCH",
    tmp_hardware::PITCH_STEP_PIN,
    tmp_hardware::PITCH_DIR_PIN,
    nullptr,
    false,
};

Axis carriage = {
    "LINEAR_CARRIAGE",
    tmp_hardware::CARRIAGE_STEP_PIN,
    tmp_hardware::CARRIAGE_DIR_PIN,
    nullptr,
    false,
};

Axis *const axes[] = {&yaw, &pitch, &carriage};
constexpr size_t AXIS_COUNT = sizeof(axes) / sizeof(axes[0]);

I2cBus busA = {
    "A",
    &Wire,
    tmp_hardware::I2C_BUS_A_SDA_PIN,
    tmp_hardware::I2C_BUS_A_SCL_PIN,
    false,
};

I2cBus busB = {
    "B",
    &Wire1,
    tmp_hardware::I2C_BUS_B_SDA_PIN,
    tmp_hardware::I2C_BUS_B_SCL_PIN,
    false,
};

I2cBus *const buses[] = {&busA, &busB};

AS5600 as5600A(&Wire);
AS5600 as5600B(&Wire1);
EncoderDevice encoderA = {&busA, &as5600A, false};
EncoderDevice encoderB = {&busB, &as5600B, false};
EncoderDevice *const encoders[] = {&encoderA, &encoderB};

// In BNO08x 1.2.7 the I2C HAL returns 0 for a failed write, but SHTP treats
// 0 as "retry forever". Turn that transport failure into SH2_ERR_IO so report
// enabling returns to our recovery/state machine. Successful writes are unchanged.
class DiagnosticBno085 : public Adafruit_BNO08x {
public:
    DiagnosticBno085() : Adafruit_BNO08x(-1) {}
    static uint32_t writeFailures;

protected:
    bool _init(int32_t sensorId) override
    {
        originalWrite = _HAL.write;
        _HAL.write = checkedWrite;
        return Adafruit_BNO08x::_init(sensorId);
    }

private:
    static int (*originalWrite)(sh2_Hal_t *, uint8_t *, unsigned);
    static int checkedWrite(sh2_Hal_t *hal, uint8_t *buffer, unsigned length)
    {
        const int result = originalWrite(hal, buffer, length);
        if (result <= 0)
        {
            ++writeFailures;
            return result == 0 ? SH2_ERR_IO : result;
        }
        return result;
    }
};

uint32_t DiagnosticBno085::writeFailures = 0;
int (*DiagnosticBno085::originalWrite)(sh2_Hal_t *, uint8_t *, unsigned) = nullptr;
DiagnosticBno085 bno085;
// Adafruit retains this pointer for callbacks during other SH2 operations too.
sh2_SensorValue_t bnoSensorValue = {};
I2cBus *bnoBus = nullptr;
uint8_t bnoAddress = 0;
bool bnoInitialized = false;
bool bnoReportEnabled = false;
bool bnoReadingValid = false;
EulerAngles latestOrientation = {0.0F, 0.0F, 0.0F};
uint8_t latestBnoAccuracy = 0;
uint32_t lastBnoReadingMs = 0;
BnoHealth bnoHealth;
uint32_t bnoNoEventPolls = 0;
uint32_t bnoInvalidSamples = 0;
uint32_t bnoProbeSuccesses = 0;
uint32_t bnoProbeFailures = 0;
uint32_t bnoResetEvents = 0;
uint32_t bnoTestResetEvents = 0;
uint32_t bnoReenableAttempts = 0;
uint32_t bnoReenableSuccesses = 0;
uint32_t bnoReenableFailures = 0;
uint32_t bnoResetFreshRecoveries = 0;
uint32_t bnoTestFreshSamples = 0;
uint32_t lastBnoProbeMs = 0;
uint32_t lastBnoReportAttemptMs = 0;
bool bnoResetAwaitingFresh = false;

MotionPhase motionPhase = MotionPhase::ABORTED;
size_t currentAxisIndex = 0;
AxisEvidence axisEvidence[AXIS_COUNT];
uint32_t phaseDeadlineMs = 0;
uint32_t moveDeadlineMs = 0;
bool testActive = false;
bool finalSummaryPrinted = false;

uint32_t lastTelemetryMs = 0;
uint32_t lastSensorServiceMs = 0;
uint32_t testTelemetrySnapshots = 0;
uint32_t betweenMoveSensorChecks = 0;

bool deadlineReached(uint32_t now, uint32_t deadline)
{
    return static_cast<int32_t>(now - deadline) >= 0;
}

void printHexAddress(uint8_t address)
{
    Serial.print("0x");
    if (address < 0x10)
    {
        Serial.print('0');
    }
    Serial.print(address, HEX);
}

const char *phaseText()
{
    switch (motionPhase)
    {
        case MotionPhase::STARTUP_WARNING:
            return "STARTUP_WARNING";
        case MotionPhase::MOVING_FORWARD:
            return "MOVING_FORWARD";
        case MotionPhase::DIRECTION_PAUSE:
            return "DIRECTION_PAUSE";
        case MotionPhase::MOVING_REVERSE:
            return "MOVING_REVERSE";
        case MotionPhase::AXIS_PAUSE:
            return "AXIS_PAUSE";
        case MotionPhase::COMPLETE:
            return "COMPLETE";
        case MotionPhase::ABORTED:
            return "ABORTED";
    }
    return "UNKNOWN";
}

bool motionIsRunning()
{
    return motionPhase == MotionPhase::MOVING_FORWARD ||
           motionPhase == MotionPhase::MOVING_REVERSE;
}

bool phaseIsBetweenMoves()
{
    return motionPhase == MotionPhase::DIRECTION_PAUSE ||
           motionPhase == MotionPhase::AXIS_PAUSE;
}

void stopAllSteppers()
{
    for (Axis *axis : axes)
    {
        if (axis->stepper != nullptr)
        {
            axis->stepper->forceStop();
        }
    }

    delay(25);  // FastAccelStepper may need about 20 ms to drain its queue.

    for (Axis *axis : axes)
    {
        pinMode(axis->stepPin, OUTPUT);
        digitalWrite(axis->stepPin, LOW);
    }
}

void printPinMap()
{
    Serial.println("PINMAP verified current TMP hardware:");
    for (const Axis *axis : axes)
    {
        Serial.print("PINMAP axis=");
        Serial.print(axis->name);
        Serial.print(" DIR_GPIO=");
        Serial.print(axis->dirPin);
        Serial.print(" STEP_GPIO=");
        Serial.println(axis->stepPin);
    }

    for (const I2cBus *bus : buses)
    {
        Serial.print("PINMAP I2C_bus=");
        Serial.print(bus->name);
        Serial.print(" SDA_GPIO=");
        Serial.print(bus->sdaPin);
        Serial.print(" SCL_GPIO=");
        Serial.println(bus->sclPin);
    }
}

bool initializeAxis(Axis &axis)
{
    Serial.print("INIT device=STEPPER axis=");
    Serial.print(axis.name);
    Serial.print(" STEP_GPIO=");
    Serial.print(axis.stepPin);
    Serial.print(" DIR_GPIO=");
    Serial.print(axis.dirPin);
    Serial.print(" status=");

    axis.stepper = engine.stepperConnectToPin(axis.stepPin);
    if (axis.stepper == nullptr)
    {
        Serial.println("FAILED reason=step-pin-unavailable");
        return false;
    }

    axis.stepper->setDirectionPin(axis.dirPin, true, DIRECTION_CHANGE_DELAY_US);

    const int8_t speedResult = axis.stepper->setSpeedInHz(TEST_SPEED_HZ);
    if (speedResult != 0)
    {
        Serial.print("FAILED reason=speed-rejected code=");
        Serial.println(speedResult);
        return false;
    }

    const int8_t accelerationResult =
        axis.stepper->setAcceleration(TEST_ACCELERATION_STEPS_PER_SECOND_SQUARED);
    if (accelerationResult != 0)
    {
        Serial.print("FAILED reason=acceleration-rejected code=");
        Serial.println(accelerationResult);
        return false;
    }

    axis.initialized = true;
    Serial.print("OK speed_hz=");
    Serial.print(TEST_SPEED_HZ);
    Serial.print(" acceleration_steps_per_s2=");
    Serial.println(TEST_ACCELERATION_STEPS_PER_SECOND_SQUARED);
    return true;
}

bool initializeBus(I2cBus &bus)
{
    Serial.print("INIT I2C bus=");
    Serial.print(bus.name);
    Serial.print(" SDA=");
    Serial.print(bus.sdaPin);
    Serial.print(" SCL=");
    Serial.print(bus.sclPin);
    Serial.print(" frequency_hz=");
    Serial.print(I2C_FREQUENCY_HZ);
    Serial.print(" status=");

    bus.initialized = bus.wire->begin(bus.sdaPin, bus.sclPin, I2C_FREQUENCY_HZ);
    if (!bus.initialized)
    {
        Serial.println("FAILED");
        return false;
    }

    bus.wire->setTimeOut(I2C_TIMEOUT_MS);
    Serial.println("OK");
    return true;
}

bool addressResponds(const I2cBus &bus, uint8_t address)
{
    if (!bus.initialized)
    {
        return false;
    }

    bus.wire->beginTransmission(address);
    return bus.wire->endTransmission() == 0;
}

void initializeEncoder(EncoderDevice &device)
{
    Serial.print("INIT device=AS5600 bus=");
    Serial.print(device.bus->name);
    Serial.print(" address=");
    printHexAddress(AS5600_ADDRESS);
    Serial.print(" status=");

    if (!device.bus->initialized)
    {
        Serial.println("FAILED reason=bus-not-initialized");
        return;
    }

    device.initialized = device.sensor->begin() && device.sensor->isConnected();
    if (!device.initialized)
    {
        Serial.println("MISSING communication_check_will_fail=yes");
        return;
    }

    Serial.println("OK");
}

void printBnoProbe(const I2cBus &bus, uint8_t address, bool found)
{
    Serial.print("PROBE device=BNO085 bus=");
    Serial.print(bus.name);
    Serial.print(" address=");
    printHexAddress(address);
    Serial.print(" status=");
    Serial.println(found ? "ACK" : "NO-ACK");
}

bool enableBnoReport(bool printStatus = true)
{
    lastBnoReportAttemptMs = millis();
    bnoReportEnabled = bno085.enableReport(SH2_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);
    if (!printStatus) return bnoReportEnabled;
    Serial.print("INIT device=BNO085 report=SH2_ROTATION_VECTOR interval_us=");
    Serial.print(BNO_REPORT_INTERVAL_US);
    Serial.print(" status=");
    Serial.println(bnoReportEnabled ? "OK" : "FAILED");
    return bnoReportEnabled;
}

void initializeBno()
{
    size_t matchCount = 0;
    bool expectedLocationFound = false;
    const uint8_t addresses[] = {BNO085_PRIMARY_ADDRESS, BNO085_ALTERNATE_ADDRESS};

    for (I2cBus *bus : buses)
    {
        for (uint8_t address : addresses)
        {
            const bool found = addressResponds(*bus, address);
            printBnoProbe(*bus, address, found);
            if (found)
            {
                ++matchCount;
                expectedLocationFound = expectedLocationFound ||
                                        (bus == &busB && address == BNO085_PRIMARY_ADDRESS);
            }
        }
    }

    if (!expectedLocationFound)
    {
        Serial.println("INIT device=BNO085 status=MISSING expected_bus=B expected_address=0x4A");
        return;
    }

    if (matchCount != 1)
    {
        Serial.println("INIT device=BNO085 status=FAILED reason=ambiguous-multiple-addresses");
        return;
    }

    bnoBus = &busB;
    bnoAddress = BNO085_PRIMARY_ADDRESS;

    Serial.print("INIT device=BNO085 bus=");
    Serial.print(bnoBus->name);
    Serial.print(" address=");
    printHexAddress(bnoAddress);
    Serial.print(" status=");

    bnoInitialized = bno085.begin_I2C(bnoAddress, bnoBus->wire);
    if (!bnoInitialized)
    {
        Serial.println("FAILED reason=library-initialization");
        bnoBus = nullptr;
        bnoAddress = 0;
        return;
    }

    Serial.println("OK");
    enableBnoReport();
}

bool quaternionToEuler(const sh2_RotationVectorWAcc_t &rotation, EulerAngles &angles)
{
    const float realSquared = rotation.real * rotation.real;
    const float iSquared = rotation.i * rotation.i;
    const float jSquared = rotation.j * rotation.j;
    const float kSquared = rotation.k * rotation.k;
    const float magnitudeSquared = realSquared + iSquared + jSquared + kSquared;

    if (!isfinite(magnitudeSquared) || magnitudeSquared <= 0.0F)
    {
        return false;
    }

    angles.heading = atan2f(
                         2.0F * (rotation.i * rotation.j + rotation.k * rotation.real),
                         iSquared - jSquared - kSquared + realSquared) *
                     RAD_TO_DEG;

    float pitchArgument =
        -2.0F * (rotation.i * rotation.k - rotation.j * rotation.real) / magnitudeSquared;
    pitchArgument = constrain(pitchArgument, -1.0F, 1.0F);
    angles.pitch = asinf(pitchArgument) * RAD_TO_DEG;

    angles.roll = atan2f(
                      2.0F * (rotation.j * rotation.k + rotation.i * rotation.real),
                      -iSquared - jSquared + kSquared + realSquared) *
                  RAD_TO_DEG;

    if (angles.heading < 0.0F)
    {
        angles.heading += 360.0F;
    }

    return isfinite(angles.heading) && isfinite(angles.pitch) && isfinite(angles.roll);
}

MoveEvidence *runningMoveEvidence()
{
    if (!testActive || !motionIsRunning() || currentAxisIndex >= AXIS_COUNT ||
        !axes[currentAxisIndex]->stepper->isRunning())
    {
        return nullptr;
    }
    AxisEvidence &evidence = axisEvidence[currentAxisIndex];
    return motionPhase == MotionPhase::MOVING_FORWARD ? &evidence.forward : &evidence.reverse;
}

bool bnoIsFresh(uint32_t now)
{
    return bnoInitialized && bnoReportEnabled && bnoReadingValid &&
           now - lastBnoReadingMs <= BNO_STALE_AFTER_MS;
}

void retryBnoReport()
{
    ++bnoReenableAttempts;
    if (enableBnoReport(false)) ++bnoReenableSuccesses;
    else ++bnoReenableFailures;
}

bool handleBnoReset()
{
    if (!bno085.wasReset()) return false;
    ++bnoResetEvents;
    if (testActive) ++bnoTestResetEvents;
    bnoReadingValid = false;
    bnoResetAwaitingFresh = true;
    retryBnoReport();
    return true;
}

void serviceBno()
{
    if (!bnoInitialized) return;
    bnoHealth.observeTime(millis(), testActive);
    handleBnoReset();
    if (!bnoReportEnabled && millis() - lastBnoReportAttemptMs >= BNO_REPORT_RETRY_MS)
    {
        retryBnoReport();
    }

    // A false result means no decoded event, not a proven I2C read failure.
    // Keep polling even while disabled so reset notifications can be received.
    sh2_SensorValue_t &sensorValue = bnoSensorValue;
    sensorValue = {};
    const bool gotEvent = bno085.getSensorEvent(&sensorValue);
    const bool resetDuringPoll = handleBnoReset();
    const uint32_t now = millis();
    bnoHealth.observeTime(now, testActive);
    if (!gotEvent)
    {
        ++bnoNoEventPolls;
        return;
    }
    if (resetDuringPoll || !bnoReportEnabled || sensorValue.sensorId != SH2_ROTATION_VECTOR)
    {
        return;
    }

    EulerAngles angles;
    if (!quaternionToEuler(sensorValue.un.rotationVector, angles))
    {
        ++bnoInvalidSamples;
        return;
    }

    bnoHealth.recordFresh(now, testActive);
    latestOrientation = angles;
    latestBnoAccuracy = sensorValue.status;
    lastBnoReadingMs = now;
    bnoReadingValid = true;
    if (bnoResetAwaitingFresh)
    {
        ++bnoResetFreshRecoveries;
        bnoResetAwaitingFresh = false;
    }
    if (testActive) ++bnoTestFreshSamples;
    MoveEvidence *move = runningMoveEvidence();
    if (move != nullptr) ++move->bnoFreshSamples;
}

const char *magnetStatusText(uint8_t status)
{
    if ((status & 0x10U) != 0U) return "TOO-WEAK";
    if ((status & 0x08U) != 0U) return "TOO-STRONG";
    if ((status & 0x20U) != 0U) return "DETECTED";
    return "NOT-DETECTED";
}

void serviceEncoder(EncoderDevice &device, size_t index)
{
    bool ok = device.initialized && device.sensor->isConnected();
    if (ok)
    {
        device.rawAngle = device.sensor->readAngle();
        ok = device.sensor->lastError() == AS5600_OK;
    }
    if (ok)
    {
        device.magnetStatus = device.sensor->readStatus();
        ok = device.sensor->lastError() == AS5600_OK;
    }
    device.readingValid = ok;
    if (!ok)
    {
        ++device.readFailures;
        if (testActive) ++device.testReadFailures;
        return;
    }

    device.lastReadMs = millis();
    ++device.successfulReads;
    if (testActive) ++device.testSuccessfulReads;
    MoveEvidence *move = runningMoveEvidence();
    if (move != nullptr) ++move->encoderReads[index];
}

void serviceSensors()
{
    serviceBno();
    uint32_t now = millis();
    if (now - lastSensorServiceMs >= SENSOR_SERVICE_INTERVAL_MS)
    {
        lastSensorServiceMs = now;
        serviceEncoder(encoderA, 0);
        serviceBno();  // Drain reports between shared-bus register transactions.
        serviceEncoder(encoderB, 1);
        serviceBno();
        now = millis();
        bnoHealth.check(now, testActive);
        if (testActive && phaseIsBetweenMoves()) ++betweenMoveSensorChecks;
    }
    if (bnoInitialized && now - lastBnoProbeMs >= BNO_PROBE_INTERVAL_MS)
    {
        lastBnoProbeMs = now;
        if (addressResponds(*bnoBus, bnoAddress)) ++bnoProbeSuccesses;
        else ++bnoProbeFailures;
        bnoHealth.observeTime(millis(), testActive);
    }
}

void printAxisPosition(const Axis &axis)
{
    if (axis.stepper == nullptr) Serial.print("NA");
    else Serial.print(axis.stepper->getCurrentPosition());
}

void printEncoderReading(const EncoderDevice &device)
{
    Serial.print("  AS5600 ");
    Serial.print(device.bus->name);
    Serial.print(": ");
    Serial.print(device.readingValid ? "OK" : "READ-FAILED");
    if (device.readingValid)
    {
        Serial.print(" angle_deg=");
        Serial.print(device.rawAngle * AS5600_RAW_TO_DEGREES, 2);
        Serial.print(" magnet=");
        Serial.print(magnetStatusText(device.magnetStatus));
    }
    Serial.print(" reads=");
    Serial.print(device.successfulReads);
    Serial.print(" failures=");
    Serial.println(device.readFailures);
}

void printTelemetry()
{
    // Display cached data only; acquisition and all health counters run separately.
    MoveEvidence *move = runningMoveEvidence();
    if (move != nullptr) ++move->telemetrySnapshots;
    if (testActive) ++testTelemetrySnapshots;
    Serial.print("STATE ");
    Serial.print(phaseText());
    Serial.print(" axis=");
    Serial.print(currentAxisIndex < AXIS_COUNT ? axes[currentAxisIndex]->name : "NONE");
    Serial.print(" steps Y/P/C=");
    printAxisPosition(yaw);
    Serial.print('/');
    printAxisPosition(pitch);
    Serial.print('/');
    printAxisPosition(carriage);
    Serial.println();
    printEncoderReading(encoderA);
    printEncoderReading(encoderB);
    Serial.print("  BNO085: ");
    Serial.print(bnoIsFresh(millis()) ? "FRESH" : "WAITING/STALE");
    if (bnoReadingValid)
    {
        Serial.print(" H/P/R=");
        Serial.print(latestOrientation.heading, 1);
        Serial.print('/');
        Serial.print(latestOrientation.pitch, 1);
        Serial.print('/');
        Serial.print(latestOrientation.roll, 1);
        Serial.print(" accuracy=");
        Serial.print(latestBnoAccuracy);
    }
    Serial.print(" age_or_wait_ms=");
    Serial.print(millis() - bnoHealth.lastFreshOrStartMs);
    Serial.print(" fresh=");
    Serial.print(bnoHealth.freshSamples);
    Serial.print(" stale_events=");
    Serial.print(bnoHealth.staleEvents);
    Serial.print(" resets=");
    Serial.println(bnoResetEvents);
}

uint8_t initializedAxisCount()
{
    uint8_t count = 0;
    for (const Axis *axis : axes) count += axis->initialized ? 1U : 0U;
    return count;
}

uint8_t initializedEncoderCount()
{
    uint8_t count = 0;
    for (const EncoderDevice *encoder : encoders) count += encoder->initialized ? 1U : 0U;
    return count;
}

bool initializationAllowsMotion()
{
    // Preserve the original motion gate allowing one available AS5600. The final
    // communication result still requires both buses/devices to be verified.
    return initializedAxisCount() == AXIS_COUNT && busA.initialized && busB.initialized &&
           initializedEncoderCount() >= 1U && bnoInitialized && bnoReportEnabled;
}

bool encoderReady(const EncoderDevice &device, uint32_t now)
{
    return device.readingValid && now - device.lastReadMs <= SENSOR_READY_MAX_AGE_MS;
}

bool sensorsReady(uint32_t now)
{
    return initializedEncoderCount() >= 1U &&
           (!encoderA.initialized || encoderReady(encoderA, now)) &&
           (!encoderB.initialized || encoderReady(encoderB, now)) && bnoIsFresh(now);
}

const char *passFail(bool passed) { return passed ? "PASS" : "FAIL"; }

void printMoveEvidence(const char *direction, const MoveEvidence &move)
{
    Serial.print("  ");
    Serial.print(direction);
    Serial.print(" began=");
    Serial.print(move.began ? "YES" : "NO");
    Serial.print(" completed=");
    Serial.print(move.completed ? "YES" : "NO");
    Serial.print(" target/end=");
    Serial.print(move.target);
    Serial.print('/');
    Serial.print(move.end);
    Serial.print(" during_motion AS_A/AS_B/BNO/display=");
    Serial.print(move.encoderReads[0]);
    Serial.print('/');
    Serial.print(move.encoderReads[1]);
    Serial.print('/');
    Serial.print(move.bnoFreshSamples);
    Serial.print('/');
    Serial.println(move.telemetrySnapshots);
}

bool encoderCommunicationPassed(const EncoderDevice &device, uint32_t now)
{
    return device.initialized && encoderReady(device, now) &&
           device.testSuccessfulReads > 0 && device.testReadFailures == 0;
}

void printEncoderSummary(const EncoderDevice &device, bool passed)
{
    Serial.print("AS5600 Bus ");
    Serial.print(device.bus->name);
    Serial.print(" communication: ");
    Serial.println(passFail(passed));
    Serial.print("  successful_reads total/test=");
    Serial.print(device.successfulReads);
    Serial.print('/');
    Serial.print(device.testSuccessfulReads);
    Serial.print(" read_failures total/test=");
    Serial.print(device.readFailures);
    Serial.print('/');
    Serial.print(device.testReadFailures);
    Serial.print(" magnet=");
    Serial.print(device.readingValid ? magnetStatusText(device.magnetStatus) : "UNAVAILABLE");
    Serial.println(" (informational only)");
}

void printFinalSummary(bool sequenceCompleted, const char *reason)
{
    if (finalSummaryPrinted) return;
    finalSummaryPrinted = true;
    const uint32_t now = millis();
    bool motionPassed = sequenceCompleted;
    bool continuityPassed = sequenceCompleted && betweenMoveSensorChecks > 0;
    const bool encoderAPassed = encoderCommunicationPassed(encoderA, now);
    const bool encoderBPassed = encoderCommunicationPassed(encoderB, now);
    const bool bnoPassed = bnoIsFresh(now) && bnoTestFreshSamples > 0 &&
                           !bnoHealth.persistentTestFailure && !bnoResetAwaitingFresh;

    Serial.println();
    Serial.println("=== MILESTONE 3 FINAL SUMMARY ===");
    for (size_t i = 0; i < AXIS_COUNT; ++i)
    {
        AxisEvidence &evidence = axisEvidence[i];
        if (axes[i]->stepper != nullptr)
        {
            evidence.end = axes[i]->stepper->getCurrentPosition();
            evidence.returned = evidence.returned && evidence.end == evidence.start;
        }
        motionPassed = motionPassed && evidence.motionPassed();
        continuityPassed = continuityPassed && evidence.forward.sensorsPassed() &&
                           evidence.reverse.sensorsPassed();
        Serial.print(i == 2 ? "CARRIAGE" : axes[i]->name);
        Serial.print(" motion: ");
        Serial.println(passFail(evidence.motionPassed()));
        printMoveEvidence("FORWARD", evidence.forward);
        printMoveEvidence("REVERSE", evidence.reverse);
        Serial.print("  returned_to_start=");
        Serial.print(evidence.returned ? "YES" : "NO");
        Serial.print(" start/final_steps=");
        Serial.print(evidence.start);
        Serial.print('/');
        printAxisPosition(*axes[i]);
        Serial.println();
    }
    printEncoderSummary(encoderA, encoderAPassed);
    printEncoderSummary(encoderB, encoderBPassed);
    Serial.print("BNO085 communication: ");
    Serial.println(passFail(bnoPassed));
    Serial.print("BNO085 fresh samples total/test: ");
    Serial.print(bnoHealth.freshSamples);
    Serial.print('/');
    Serial.println(bnoTestFreshSamples);
    Serial.print("BNO085 stale events: ");
    Serial.println(bnoHealth.staleEvents);
    Serial.print("  recovered_events=");
    Serial.print(bnoHealth.recoveredStaleEvents);
    Serial.print(" stale_checks_50ms=");
    Serial.print(bnoHealth.staleChecks);
    Serial.print(" unresolved_stale=");
    Serial.println(bnoHealth.staleActive ? "YES" : "NO");
    Serial.print("  max_gap_ms total/test=");
    Serial.print(bnoHealth.maxGapMs);
    Serial.print('/');
    Serial.print(bnoHealth.maxTestGapMs);
    Serial.print(" persistent_test_failure=");
    Serial.println(bnoHealth.persistentTestFailure ? "YES" : "NO");
    Serial.print("BNO085 reset events: ");
    Serial.print(bnoResetEvents);
    Serial.print(" (during_test=");
    Serial.print(bnoTestResetEvents);
    Serial.println(')');
    Serial.print("  report_reenable attempts/successes/failures=");
    Serial.print(bnoReenableAttempts);
    Serial.print('/');
    Serial.print(bnoReenableSuccesses);
    Serial.print('/');
    Serial.print(bnoReenableFailures);
    Serial.print(" fresh_recoveries=");
    Serial.print(bnoResetFreshRecoveries);
    Serial.print(" awaiting_fresh=");
    Serial.println(bnoResetAwaitingFresh ? "YES" : "NO");
    Serial.print("BNO085 observable read failures: ");
    Serial.print(bnoInvalidSamples + bnoProbeFailures);
    Serial.print(" (invalid_vectors=");
    Serial.print(bnoInvalidSamples);
    Serial.print(" no_ack_probes=");
    Serial.print(bnoProbeFailures);
    Serial.println(')');
    Serial.print("  ACK_probes_ok=");
    Serial.print(bnoProbeSuccesses);
    Serial.print(" no_event_polls=");
    Serial.print(bnoNoEventPolls);
    Serial.println(" (not classified as read failures by this API)");
    Serial.print("BNO085 transport write failures: ");
    Serial.println(DiagnosticBno085::writeFailures);
    Serial.print("Sensor telemetry during motion: ");
    Serial.println(passFail(continuityPassed));
    Serial.print("  test_display_snapshots=");
    Serial.print(testTelemetrySnapshots);
    Serial.print(" between_move_sensor_checks=");
    Serial.println(betweenMoveSensorChecks);
    const bool warnings = bnoHealth.staleEvents > 0 || bnoTestResetEvents > 0 ||
                          bnoInvalidSamples > 0 || bnoProbeFailures > 0 ||
                          bnoReenableFailures > 0 || DiagnosticBno085::writeFailures > 0 ||
                          encoderA.readFailures > 0 ||
                          encoderB.readFailures > 0;
    Serial.print("Recorded sensor warnings: ");
    Serial.println(warnings ? "YES - review counters above, including recovered events" : "NONE");
    Serial.print("Sequence status: ");
    Serial.println(reason);
    Serial.println("Firmware step counts do not prove mechanical return or positioning accuracy.");
    Serial.println("Second AS5600: communication only; magnet/mechanical verification pending.");
    Serial.println("Test stopped. No automatic restart; telemetry is now silent until board reset.");
    Serial.print("FINAL RESULT: ");
    Serial.println(passFail(motionPassed && encoderAPassed && encoderBPassed &&
                            bnoPassed && continuityPassed));
    Serial.println("=================================");
    Serial.flush();
}

void abortMotionTest(const char *reason)
{
    stopAllSteppers();
    motionPhase = MotionPhase::ABORTED;
    testActive = false;
    printFinalSummary(false, reason);
}

void startMove(Axis &axis, int32_t relativeSteps, bool forward)
{
    AxisEvidence &axisResult = axisEvidence[currentAxisIndex];
    MoveEvidence &move = forward ? axisResult.forward : axisResult.reverse;
    move.target = axis.stepper->getCurrentPosition() + relativeSteps;
    Serial.print("MOVE START axis=");
    Serial.print(axis.name);
    Serial.print(" direction=");
    Serial.print(forward ? "FORWARD" : "REVERSE");
    Serial.print(" delta_steps=");
    Serial.print(relativeSteps);
    Serial.print(" start_position=");
    Serial.print(axis.stepper->getCurrentPosition());
    Serial.print(" target_position=");
    Serial.println(move.target);

    const auto moveResult = axis.stepper->move(relativeSteps);
    if (moveResult != MOVE_OK)
    {
        abortMotionTest("FAIL: FastAccelStepper rejected a move command");
        return;
    }
    move.began = true;  // Accepted firmware command; physical travel needs observation.
    moveDeadlineMs = millis() + MOVE_TIMEOUT_MS;
    motionPhase = forward ? MotionPhase::MOVING_FORWARD : MotionPhase::MOVING_REVERSE;
}

void finishCurrentMove(uint32_t now)
{
    Axis &axis = *axes[currentAxisIndex];
    AxisEvidence &axisResult = axisEvidence[currentAxisIndex];
    const bool forward = motionPhase == MotionPhase::MOVING_FORWARD;
    MoveEvidence &move = forward ? axisResult.forward : axisResult.reverse;
    move.end = axis.stepper->getCurrentPosition();
    move.completed = true;
    axisResult.end = move.end;
    Serial.print("MOVE END axis=");
    Serial.print(axis.name);
    Serial.print(" direction=");
    Serial.print(forward ? "FORWARD" : "REVERSE");
    Serial.print(" position_check=");
    Serial.print(passFail(move.motionPassed()));
    Serial.print(" sensor_evidence=");
    Serial.println(passFail(move.sensorsPassed()));

    if (!move.motionPassed())
    {
        abortMotionTest("FAIL: A motor did not reach its commanded step count");
        return;
    }
    if (forward)
    {
        motionPhase = MotionPhase::DIRECTION_PAUSE;
        phaseDeadlineMs = now + DIRECTION_PAUSE_MS;
        return;
    }
    axisResult.returned = move.end == axisResult.start;
    if (!axisResult.returned)
    {
        abortMotionTest("FAIL: An axis did not return to its starting step count");
        return;
    }
    motionPhase = MotionPhase::AXIS_PAUSE;
    phaseDeadlineMs = now + AXIS_PAUSE_MS;
}

void completeMotionTest()
{
    stopAllSteppers();
    bnoHealth.observeTime(millis(), testActive);
    motionPhase = MotionPhase::COMPLETE;
    testActive = false;
    printFinalSummary(true, "All six motor moves finished; see individual checks above");
}

void serviceMotionState(uint32_t now)
{
    switch (motionPhase)
    {
        case MotionPhase::STARTUP_WARNING:
            if (!deadlineReached(now, phaseDeadlineMs)) return;
            if (!sensorsReady(now))
            {
                abortMotionTest("FAIL: Required sensors were not healthy/fresh at motion start");
                return;
            }
            Serial.println("GATE motion_authorized=YES reason=initialization-and-fresh-sensors-ready");
            currentAxisIndex = 0;
            for (size_t i = 0; i < AXIS_COUNT; ++i)
            {
                axisEvidence[i].start = axes[i]->stepper->getCurrentPosition();
                axisEvidence[i].end = axisEvidence[i].start;
            }
            testActive = true;
            startMove(*axes[currentAxisIndex], TEST_STEPS, true);
            return;

        case MotionPhase::MOVING_FORWARD:
        case MotionPhase::MOVING_REVERSE:
            if (!axes[currentAxisIndex]->stepper->isRunning()) finishCurrentMove(now);
            else if (deadlineReached(now, moveDeadlineMs))
            {
                abortMotionTest("FAIL: A motor move exceeded its 10000 ms safety timeout");
            }
            return;

        case MotionPhase::DIRECTION_PAUSE:
            if (deadlineReached(now, phaseDeadlineMs))
                startMove(*axes[currentAxisIndex], -TEST_STEPS, false);
            return;

        case MotionPhase::AXIS_PAUSE:
            if (!deadlineReached(now, phaseDeadlineMs)) return;
            ++currentAxisIndex;
            if (currentAxisIndex >= AXIS_COUNT) completeMotionTest();
            else startMove(*axes[currentAxisIndex], TEST_STEPS, true);
            return;

        case MotionPhase::COMPLETE:
        case MotionPhase::ABORTED:
            return;
    }
}

}  // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(500);
    Serial.println();
    Serial.println("TMP Milestone 3: integrated motor and sensor coexistence test (recorded results)");
    Serial.println("Open-loop motion only; sensor readings do not control any motor.");
    Serial.println("Display: 750 ms; AS5600/health service: 50 ms; BNO polling: every loop.");
    Serial.print("BNO policy: report_us=");
    Serial.print(BNO_REPORT_INTERVAL_US);
    Serial.print(" stale_after_ms=");
    Serial.print(BNO_STALE_AFTER_MS);
    Serial.print(" persistent_at_ms=");
    Serial.println(BNO_PERSISTENT_AFTER_MS);
    printPinMap();

    engine.init();
    for (Axis *axis : axes) initializeAxis(*axis);
    initializeBus(busA);
    initializeBus(busB);
    initializeEncoder(encoderA);
    initializeEncoder(encoderB);
    initializeBno();
    bnoHealth.lastFreshOrStartMs = millis();

    Serial.print("INIT SUMMARY steppers_ready=");
    Serial.print(initializedAxisCount());
    Serial.print("/3 i2c_buses_ready=");
    Serial.print((busA.initialized ? 1 : 0) + (busB.initialized ? 1 : 0));
    Serial.print("/2 as5600_ready=");
    Serial.print(initializedEncoderCount());
    Serial.print("/2 bno085_ready=");
    Serial.println(bnoInitialized && bnoReportEnabled ? "YES" : "NO");
    if (!initializationAllowsMotion())
    {
        abortMotionTest("FAIL: Initialization requirements not met; no motion commanded");
        return;
    }
    if (initializedEncoderCount() < 2U)
    {
        Serial.println("WARN one AS5600 missing: motion gate allows test, but final result will FAIL.");
    }
    Serial.println("NOTE AS5600 magnet status is informational; both devices must communicate to PASS.");
    motionPhase = MotionPhase::STARTUP_WARNING;
    phaseDeadlineMs = millis() + STARTUP_DELAY_MS;
    Serial.print("WARNING: Motion begins in ");
    Serial.print(STARTUP_DELAY_MS / 1000);
    Serial.println(" seconds if required sensors are healthy. Keep clear of the machine.");
}

void loop()
{
    if (finalSummaryPrinted)
    {
        delay(20);
        return;  // Freeze the test evidence and leave the final summary on screen.
    }
    serviceSensors();
    serviceMotionState(millis());
    if (!finalSummaryPrinted && millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)
    {
        lastTelemetryMs = millis();
        printTelemetry();
    }
    delay(1);
}
