#pragma once
#include <Arduino.h>
namespace simulated {
static bool encoderPresent = false;
static uint8_t encoderStatus = 0x20;
static bool encoderShortRead = false;
}
class TwoWire {
public:
    explicit TwoWire(unsigned bus) : bus(bus) {}
    unsigned bus;
    uint8_t sda = 0, scl = 0;
    uint32_t frequency = 0;
    uint16_t timeout = 0;
    bool begin(uint8_t data, uint8_t clock, uint32_t hz) {
        sda = data; scl = clock; frequency = hz;
        return !(bus == 1 && simulated::busBInitFails);
    }
    void setTimeOut(uint16_t value) { timeout = value; }
    void beginTransmission(uint8_t address) {
        address_ = address; simulated::addressedSensors.push_back(address);
    }
    // No AS5600 exists in any fixture. Only the required BNO can acknowledge.
    void write(uint8_t) {}
    uint8_t requestFrom(uint8_t, uint8_t) { readIndex_ = 0; return simulated::encoderShortRead ? 0 : 3; }
    int read() {
        const int raw = static_cast<int>(fmod(360.0 + simulated::motors[1].position * 360.0 / 1600.0, 360.0) * 4096.0 / 360.0);
        if (readIndex_++ == 0) return simulated::encoderStatus;
        return readIndex_ == 2 ? raw >> 8 : raw & 255;
    }
    uint8_t endTransmission(bool = true) {
        if (address_ == 0x36) return simulated::encoderPresent ? 0 : 2;
        return bus == 1 && address_ == 0x4A && !simulated::bnoAckFails ? 0 : 2;
    }
private:
    uint8_t address_ = 0;
    unsigned readIndex_ = 0;
};
static TwoWire Wire(0);
static TwoWire Wire1(1);
