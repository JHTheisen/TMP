#include <Arduino.h>
#include <Wire.h>
#include <cstdio>
#include "sensor_support.h"

// Sensor only: no motor library, motor GPIO setup, STEP pulses or commands.
namespace {
constexpr int SDA_PIN = 4, SCL_PIN = 5;
constexpr uint8_t BNO_ADDRESS = 0x4A;
constexpr uint32_t PRINT_MS = 100, STALE_MS = 150, REPORT_RETRY_MS = 1000;
milestone4::DiagnosticBno085 bno;
milestone4::EulerAngles angles = {0, 0, 0};
bool initialized = false, reportsEnabled = false, haveSample = false, invalidSample = false;
uint8_t accuracy = 0;
uint32_t lastGoodMs = 0, lastPrintMs = 0, lastReportAttemptMs = 0;

void enableReports() {
    lastReportAttemptMs = millis();
    reportsEnabled = bno.enableReport(SH2_ROTATION_VECTOR, milestone4::BNO_REPORT_INTERVAL_US);
    if (!reportsEnabled) Serial.println("BNO report unavailable; retrying.");
}
bool handleReset() {
    if (!bno.wasReset()) return false;
    haveSample = false; invalidSample = false; reportsEnabled = false;
    Serial.println("BNO reset; waiting for new orientation.");
    enableReports();
    return true;
}
void pollBno() {
    if (!initialized) return;
    if (handleReset()) return;
    if (!reportsEnabled) {
        if (millis() - lastReportAttemptMs >= REPORT_RETRY_MS) enableReports();
        return;
    }
    sh2_SensorValue_t event = {};
    const bool received = bno.getSensorEvent(&event);
    if (handleReset() || !received || event.sensorId != SH2_ROTATION_VECTOR) return;
    milestone4::EulerAngles result;
    if (!milestone4::quaternionToEuler(event.un.rotationVector, result)) {
        invalidSample = true; // Do not refresh the last valid sample's age.
        return;
    }
    angles = result; accuracy = event.status & 3;
    haveSample = true; invalidSample = false; lastGoodMs = millis();
}
void printOrientation(uint32_t now) {
    if (!haveSample) {
        Serial.println("Yaw=     --  Pitch=     --  Roll=     --  Acc=--  NO DATA  Age=-- ms");
        return;
    }
    const uint32_t age = now - lastGoodMs;
    const char *state = invalidSample ? "INVALID" :
        (reportsEnabled && age < STALE_MS ? "FRESH" : "STALE");
    char line[128];
    snprintf(line, sizeof(line), "Yaw=%7.2f  Pitch=%7.2f  Roll=%7.2f  Acc=%u  %-7s Age=%lu ms\n",
        angles.heading, angles.pitch, angles.roll, accuracy, state, static_cast<unsigned long>(age));
    Serial.print(line);
}
} // namespace

void setup() {
    Serial.begin(115200); delay(500);
    Serial.println("BNO AXIS CHECK | Bus B SDA4/SCL5, 0x4A | degrees | 10 Hz | no motor control");
    Serial.println("Move the cradle by hand; compare Yaw (heading), Pitch and Roll. Accuracy 0/1 is displayed normally.");
    if (!Wire1.begin(SDA_PIN, SCL_PIN, 100000)) {
        Serial.println("ERROR: Bus B initialization failed; reset to retry."); return;
    }
    Wire1.setTimeOut(50);
    Wire1.beginTransmission(BNO_ADDRESS);
    if (Wire1.endTransmission() != 0) {
        Serial.println("ERROR: BNO at 0x4A did not ACK; check wiring and reset."); return;
    }
    if (!bno.begin_I2C(BNO_ADDRESS, &Wire1)) {
        Serial.println("ERROR: BNO initialization failed; reset to retry."); return;
    }
    initialized = true;
    bno.wasReset(); // Consume the normal startup reset before enabling reports.
    enableReports();
}
void loop() {
    pollBno();
    const uint32_t now = millis();
    if (now - lastPrintMs >= PRINT_MS) {
        lastPrintMs = now; printOrientation(now);
    }
    delay(1);
}