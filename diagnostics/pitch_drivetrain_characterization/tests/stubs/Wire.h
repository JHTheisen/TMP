#pragma once
#include <Arduino.h>

class TwoWire {
public:
    explicit TwoWire(unsigned bus) : bus(bus) {}
    unsigned bus;
    bool begin(uint8_t, uint8_t, uint32_t) { return true; }
    void setTimeOut(uint16_t) {}
    void beginTransmission(uint8_t address) { address_ = address; }
    uint8_t endTransmission()
    {
        return address_ == 0x36 || (bus == 1 && address_ == 0x4A) ? 0 : 2;
    }
private:
    uint8_t address_ = 0;
};
static TwoWire Wire(0);
static TwoWire Wire1(1);
