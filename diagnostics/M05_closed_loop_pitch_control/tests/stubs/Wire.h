#pragma once
#include <Arduino.h>
class TwoWire {
public:
    explicit TwoWire(unsigned bus) : bus(bus) {}
    unsigned bus;
    uint8_t sda = 0;
    uint8_t scl = 0;
    uint32_t frequency = 0;
    uint16_t timeout = 0;
    bool begin(uint8_t data, uint8_t clock, uint32_t hz)
    { sda = data; scl = clock; frequency = hz; return !(bus == 1 && simulated::busBInitFails); }
    void setTimeOut(uint16_t value) { timeout = value; }
    void beginTransmission(uint8_t address) { address_ = address; }
    uint8_t endTransmission()
    { return address_ == 0x36 || (bus == 1 && address_ == 0x4A) ? 0 : 2; }
private:
    uint8_t address_ = 0;
};
static TwoWire Wire(0);
static TwoWire Wire1(1);
