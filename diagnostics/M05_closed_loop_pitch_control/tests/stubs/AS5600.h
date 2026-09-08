#pragma once
#include <Wire.h>
constexpr int AS5600_OK = 0;
constexpr float AS5600_RAW_TO_DEGREES = 360.0F / 4096.0F;
class AS5600 {
public:
    explicit AS5600(TwoWire *wire) : wire_(wire) {}
    bool begin() { return !(wire_->bus == 1 && simulated::encoderInitFails); }
    bool isConnected() { return !(wire_->bus == 1 && simulated::encoderUnavailable()); }
    uint16_t rawAngle()
    {
        error_ = simulated::encoderReadError && wire_->bus == 1 ? 1 : AS5600_OK;
        if (wire_->bus == 1 && simulated::blockEncoderMs) {
            delay(simulated::blockEncoderMs);
            simulated::blockEncoderMs = 0;
        }
        simulated::position();
        if (wire_->bus == 1) ++simulated::encoderReads;
        if (wire_->bus == 1 && simulated::encoderRawOverride >= 0)
            return static_cast<uint16_t>(simulated::encoderRawOverride);
        const double shaft = simulated::frozenEncoder ? 0.0 : simulated::shaftSteps;
        const double startAngle = simulated::encoderSign > 0 ? 350.0 : 10.0;
        const double angle = startAngle + simulated::encoderSign * shaft * 360.0 / simulated::pulsesPerRev;
        const int32_t ticks = static_cast<int32_t>(lround(angle * 4096.0 / 360.0));
        return static_cast<uint16_t>((ticks % 4096 + 4096) % 4096);
    }
    uint8_t readStatus()
    {
        error_ = AS5600_OK;
        return wire_->bus == 0 ? 0 : simulated::missingPitchMagnet ? 0 : 0x20;
    }
    int lastError() { return error_; }
private:
    TwoWire *wire_;
    int error_ = AS5600_OK;
};
