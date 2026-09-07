#pragma once
#include <Wire.h>

constexpr int AS5600_OK = 0;
constexpr float AS5600_RAW_TO_DEGREES = 360.0F / 4096.0F;
class AS5600 {
public:
    explicit AS5600(TwoWire *wire) : wire_(wire) {}
    bool begin() { return true; }
    bool isConnected() { return true; }
    uint16_t readAngle()
    {
        error_ = AS5600_OK;
        if (wire_->bus == 0 && simulated::encoderFailureAt != 0 &&
            millis() >= simulated::encoderFailureAt && !simulated::encoderFailureDelivered)
        {
            simulated::encoderFailureDelivered = true;
            error_ = 1;
        }
        return 1234;
    }
    uint8_t readStatus()
    {
        error_ = AS5600_OK;
        // Both informational magnet cases must preserve communication PASS.
        return wire_->bus == 0 ? 0x10 : 0;
    }
    int lastError() { return error_; }
private:
    TwoWire *wire_;
    int error_ = AS5600_OK;
};
