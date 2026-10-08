#pragma once
#include <Arduino.h>
class TwoWire {
public:
    struct EncoderFixture {
        bool present = false, writeAccepted = true;
        uint16_t raw = 0;
        uint8_t status = 0x20, addressResult = 0, requestLength = 3;
        int failedReadIndex = -1, highOverride = -1;
        uint32_t addressDelayMs = 0, requestDelayMs = 0;
        unsigned addressTransfers = 0, requests = 0, writes = 0;
        uint8_t lastRegister = 0, requestedBytes = 0;
    } encoder;
    explicit TwoWire(unsigned bus) : bus(bus) {}
    unsigned bus;
    bool beginFails = false;
    unsigned beginCalls = 0;
    uint8_t sda = 0, scl = 0;
    uint32_t frequency = 0;
    uint16_t timeout = 0;
    bool begin(uint8_t data, uint8_t clock, uint32_t hz) {
        ++beginCalls;
        sda = data; scl = clock; frequency = hz;
        return !beginFails && !(bus == 1 && simulated::busBInitFails);
    }
    void setTimeOut(uint16_t value) { timeout = value; }
    void beginTransmission(uint8_t address) {
        address_ = address; simulated::addressedSensors.push_back(address);
    }
    size_t write(uint8_t value) {
        if (address_ != 0x36) return 1;
        ++encoder.writes;
        encoder.lastRegister = value;
        return encoder.writeAccepted ? 1 : 0;
    }
    // Encoders are absent by default.
    uint8_t endTransmission(bool = true) {
        if (address_ == 0x36) {
            ++encoder.addressTransfers;
            simulated::sensorDelay(encoder.addressDelayMs);
            return encoder.present ? encoder.addressResult : 2;
        }
        return 2;
    }
    size_t requestFrom(uint8_t address, uint8_t count) {
        readIndex_ = 0;
        ++encoder.requests;
        encoder.requestedBytes = count;
        simulated::sensorDelay(encoder.requestDelayMs);
        return address == 0x36 && encoder.present ? encoder.requestLength : 0;
    }
    int read() {
        simulated::lastSampleAt = millis();
        const int index = readIndex_++;
        if (index == encoder.failedReadIndex || index >= encoder.requestLength) return -1;
        if (index == 0) return encoder.status;
        uint16_t raw = encoder.raw;
        if (simulated::encoderPlantFeedback && bus < 2) {
            double degrees = simulated::encoderBaselineDegrees[bus] +
                (simulated::motors[bus].degrees + simulated::encoderAxisDisturbance[bus]) /
                simulated::motors[bus].degreesPerStep * simulated::encoderDegreesPerStep[bus];
            degrees = fmod(degrees, 360.0); if (degrees < 0) degrees += 360.0;
            raw = static_cast<uint16_t>(degrees * 4096.0 / 360.0) & 0x0fffU;
        }
        if (index == 1) return encoder.highOverride >= 0 ? encoder.highOverride : raw >> 8;
        if (index == 2) return raw & 0xFF;
        return -1;
    }
private:
    uint8_t address_ = 0;
    int readIndex_ = 0;
};
static TwoWire Wire(0);
static TwoWire Wire1(1);
