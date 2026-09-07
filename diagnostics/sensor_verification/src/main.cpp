#include <Arduino.h>
#include <AS5600.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>

#include "hardware_config.h"

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t I2C_FREQUENCY_HZ = 100000;
constexpr uint16_t I2C_TIMEOUT_MS = 50;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 250;
constexpr uint32_t BNO_REPORT_INTERVAL_US = 10000;
constexpr uint32_t BNO_STALE_AFTER_MS = 1000;

constexpr uint8_t AS5600_ADDRESS = 0x36;
constexpr uint8_t BNO085_PRIMARY_ADDRESS = 0x4A;
constexpr uint8_t BNO085_ALTERNATE_ADDRESS = 0x4B;

struct I2cBus {
    const char *name;
    TwoWire *wire;
    uint8_t sdaPin;
    uint8_t sclPin;
    bool initialized;
};

struct EncoderDevice {
    I2cBus *bus;
    AS5600 *sensor;
    bool initialized;
};

struct EulerAngles {
    float heading;
    float pitch;
    float roll;
};

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

Adafruit_BNO08x bno085(-1);
I2cBus *bnoBus = nullptr;
uint8_t bnoAddress = 0;
bool bnoInitialized = false;
bool bnoReportEnabled = false;
bool bnoReadingValid = false;
EulerAngles latestOrientation = {0.0F, 0.0F, 0.0F};
uint8_t latestBnoAccuracy = 0;
uint32_t lastBnoReadingMs = 0;
uint32_t lastTelemetryMs = 0;

void printHexAddress(uint8_t address)
{
    Serial.print("0x");
    if (address < 0x10)
    {
        Serial.print('0');
    }
    Serial.print(address, HEX);
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
        Serial.println("MISSING");
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

bool enableBnoReport()
{
    bnoReportEnabled = bno085.enableReport(SH2_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);
    Serial.print("INIT device=BNO085 report=SH2_ROTATION_VECTOR interval_us=");
    Serial.print(BNO_REPORT_INTERVAL_US);
    Serial.print(" status=");
    Serial.println(bnoReportEnabled ? "OK" : "FAILED");
    return bnoReportEnabled;
}

void initializeBno()
{
    struct ProbeMatch {
        I2cBus *bus;
        uint8_t address;
    };

    ProbeMatch matches[4];
    size_t matchCount = 0;
    const uint8_t addresses[] = {BNO085_PRIMARY_ADDRESS, BNO085_ALTERNATE_ADDRESS};

    for (I2cBus *bus : buses)
    {
        for (uint8_t address : addresses)
        {
            const bool found = addressResponds(*bus, address);
            printBnoProbe(*bus, address, found);
            if (found)
            {
                matches[matchCount++] = {bus, address};
            }
        }
    }

    if (matchCount == 0)
    {
        Serial.println("INIT device=BNO085 status=MISSING expected_address=0x4A-or-0x4B");
        return;
    }

    if (matchCount > 1)
    {
        Serial.println("INIT device=BNO085 status=FAILED reason=ambiguous-multiple-addresses");
        return;
    }

    bnoBus = matches[0].bus;
    bnoAddress = matches[0].address;

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

    if (magnitudeSquared <= 0.0F)
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

    return true;
}

void serviceBno()
{
    if (!bnoInitialized || !bnoReportEnabled)
    {
        return;
    }

    if (bno085.wasReset())
    {
        Serial.println("WARN device=BNO085 event=reset-detected action=re-enable-report");
        bnoReadingValid = false;
        enableBnoReport();
    }

    sh2_SensorValue_t sensorValue;
    if (!bno085.getSensorEvent(&sensorValue))
    {
        return;
    }

    if (sensorValue.sensorId != SH2_ROTATION_VECTOR)
    {
        return;
    }

    EulerAngles angles;
    if (!quaternionToEuler(sensorValue.un.rotationVector, angles))
    {
        return;
    }

    latestOrientation = angles;
    latestBnoAccuracy = sensorValue.status;
    lastBnoReadingMs = millis();
    bnoReadingValid = true;
}

const char *magnetStatusText(uint8_t status)
{
    if ((status & 0x20U) != 0U)
    {
        return "DETECTED";
    }
    if ((status & 0x10U) != 0U)
    {
        return "TOO-WEAK";
    }
    if ((status & 0x08U) != 0U)
    {
        return "TOO-STRONG";
    }
    return "NOT-DETECTED";
}

void printEncoderReading(EncoderDevice &device)
{
    Serial.print("DATA device=AS5600 bus=");
    Serial.print(device.bus->name);
    Serial.print(" address=");
    printHexAddress(AS5600_ADDRESS);

    if (!device.initialized)
    {
        Serial.println(" status=UNAVAILABLE");
        return;
    }

    if (!device.sensor->isConnected())
    {
        Serial.println(" status=READ-FAILED reason=no-ack");
        return;
    }

    const uint16_t rawAngle = device.sensor->readAngle();
    const int angleError = device.sensor->lastError();
    if (angleError != AS5600_OK)
    {
        Serial.print(" status=READ-FAILED error=");
        Serial.println(angleError);
        return;
    }

    const uint8_t sensorStatus = device.sensor->readStatus();
    const int statusError = device.sensor->lastError();
    if (statusError != AS5600_OK)
    {
        Serial.print(" status=READ-FAILED error=");
        Serial.println(statusError);
        return;
    }

    Serial.print(" status=OK raw=");
    Serial.print(rawAngle);
    Serial.print(" angle_deg=");
    Serial.print(rawAngle * AS5600_RAW_TO_DEGREES, 2);
    Serial.print(" magnet=");
    Serial.println(magnetStatusText(sensorStatus));
}

void printBnoReading()
{
    Serial.print("DATA device=BNO085");

    if (!bnoInitialized)
    {
        Serial.println(" status=UNAVAILABLE");
        return;
    }

    Serial.print(" bus=");
    Serial.print(bnoBus->name);
    Serial.print(" address=");
    printHexAddress(bnoAddress);

    if (!bnoReportEnabled)
    {
        Serial.println(" status=UNAVAILABLE reason=report-not-enabled");
        return;
    }

    if (!bnoReadingValid)
    {
        Serial.println(" status=WAITING-FOR-ROTATION-VECTOR");
        return;
    }

    const uint32_t readingAgeMs = millis() - lastBnoReadingMs;
    Serial.print(" status=");
    Serial.print(readingAgeMs > BNO_STALE_AFTER_MS ? "STALE" : "OK");
    Serial.print(" heading_deg=");
    Serial.print(latestOrientation.heading, 2);
    Serial.print(" pitch_deg=");
    Serial.print(latestOrientation.pitch, 2);
    Serial.print(" roll_deg=");
    Serial.print(latestOrientation.roll, 2);
    Serial.print(" accuracy=");
    Serial.print(latestBnoAccuracy);
    Serial.print(" age_ms=");
    Serial.println(readingAgeMs);
}

void printTelemetry()
{
    printEncoderReading(encoderA);
    printEncoderReading(encoderB);
    printBnoReading();
    Serial.println();
}

}  // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(500);

    Serial.println();
    Serial.println("TMP Milestone 2: dual-I2C sensor verification");
    Serial.println("No motor motion is configured in this diagnostic.");

    initializeBus(busA);
    initializeBus(busB);
    initializeEncoder(encoderA);
    initializeEncoder(encoderB);
    initializeBno();

    Serial.println();
    Serial.println("Initialization complete. Live readings follow every 250 ms.");
}

void loop()
{
    serviceBno();

    const uint32_t now = millis();
    if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)
    {
        lastTelemetryMs = now;
        printTelemetry();
    }

    delay(1);
}
