#pragma once
#include <Wire.h>

constexpr int AS5600_OK = 0;
constexpr float AS5600_RAW_TO_DEGREES = 360.0F / 4096.0F;
class AS5600 {
public:
    explicit AS5600(TwoWire *wire) : wire_(wire) {}
    bool begin() { return true; }
    bool isConnected() { return !(wire_->bus == 1 && simulated::encoderOutage); }
    uint16_t rawAngle()
    {
        error_ = AS5600_OK;
        if (wire_->bus == 0 && simulated::encoderFailureAt != 0 &&
            millis() >= simulated::encoderFailureAt && !simulated::encoderFailureDelivered)
        {
            simulated::encoderFailureDelivered = true;
            error_ = 1;
        }
        const double angle = 350.0 + simulated::encoderSign * simulated::position() * 360.0 / 3200.0;
        const int32_t ticks = static_cast<int32_t>(lround(angle * 4096.0 / 360.0));
        return static_cast<uint16_t>((ticks % 4096 + 4096) % 4096);
    }
    uint8_t readStatus()
    {
        error_ = AS5600_OK;
        // Both informational magnet cases must preserve communication PASS.
        return wire_->bus == 0 ? 0 : simulated::missingPitchMagnet ? 0 : 0x20;
    }
    int lastError() { return error_; }
private:
    TwoWire *wire_;
    int error_ = AS5600_OK;
};
