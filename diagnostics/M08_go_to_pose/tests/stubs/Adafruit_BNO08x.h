#pragma once
#include <Wire.h>
constexpr uint8_t SH2_ROTATION_VECTOR = 5;
constexpr int SH2_ERR_IO = -4;
struct sh2_Hal_t { int (*write)(sh2_Hal_t *, uint8_t *, unsigned); };
struct sh2_RotationVectorWAcc_t { float real, i, j, k; };
struct sh2_SensorValue_t {
    uint8_t sensorId = 0;
    uint8_t status = 0;
    struct { sh2_RotationVectorWAcc_t rotationVector; } un;
};
class Adafruit_BNO08x {
public:
    explicit Adafruit_BNO08x(int) {}
    uint8_t address = 0;
    TwoWire *wire = nullptr;
    bool begin_I2C(uint8_t selectedAddress, TwoWire *selectedWire)
    {
        address = selectedAddress;
        wire = selectedWire;
        _HAL.write = simulatedWrite;
        return !simulated::bnoInitFails && _init(0);
    }
    bool enableReport(uint8_t, uint32_t intervalUs)
    {
        reportIntervalMs_ = intervalUs / 1000;
        uint8_t command = 0;
        // Exercise the preserved DiagnosticBno085 HAL write-failure wrapper.
        const int result = _HAL.write(&_HAL, &command, 1);
        if (result == 0) std::abort();
        enabled_ = result > 0;
        return enabled_;
    }
    bool wasReset()
    {
        if (simulated::bnoResetAt == 0 || simulated::bnoResetDelivered ||
            millis() < simulated::bnoResetAt) return false;
        simulated::bnoResetDelivered = true;
        enabled_ = false;
        return true;
    }
    bool getSensorEvent(sh2_SensorValue_t *event)
    {
        if (simulated::blockBnoMs) {
            delay(simulated::blockBnoMs);
            simulated::blockBnoMs = 0;
        }
        if (!enabled_ || millis() - lastSampleMs_ < reportIntervalMs_ ||
            (millis() >= simulated::bnoPauseStart && millis() < simulated::bnoPauseEnd)) return false;
        lastSampleMs_ = millis();
        ++simulated::bnoReads;
        event->sensorId = simulated::wrongReportType ? 8 : SH2_ROTATION_VECTOR;
        event->status = simulated::accuracy;
        simulated::lastSampleAt = millis();
        if (simulated::resetDuringPoll) {
            simulated::bnoResetAt = millis();
            simulated::resetDuringPoll = false;
        }
        if (simulated::invalidQuaternion) {
            event->un.rotationVector = {0, 0, 0, 0};
            return true;
        }
        const double yawDegrees = (simulated::frozenFeedback ? simulated::heldYaw : simulated::actualYaw()) +
                                    simulated::noiseAmplitude * sin(millis() * 0.027);
        const double halfAngle = yawDegrees / RAD_TO_DEG / 2.0;
        const double pitchDegrees = simulated::frozenFeedback ? simulated::heldPitch : simulated::actualPitch();
        if (!simulated::frozenFeedback) { simulated::heldYaw = yawDegrees; simulated::heldPitch = pitchDegrees; }
        const double halfRoll = pitchDegrees / RAD_TO_DEG / 2.0;
        const double halfPitch = (simulated::rawBnoPitch + simulated::rawBnoPitchNoise * sin(millis() * 0.03)) / RAD_TO_DEG / 2.0;
        const double cy = cos(halfAngle), sy = sin(halfAngle);
        const double cp = cos(halfPitch), sp = sin(halfPitch);
        const double cr = cos(halfRoll), sr = sin(halfRoll);
        // Z(yaw) * Y(raw BNO pitch) * X(physical cradle pitch = BNO roll).
        // Keep raw pitch nonzero so using it accidentally cannot pass leveling.
        event->un.rotationVector = {
            static_cast<float>(cy * cp * cr + sy * sp * sr),
            static_cast<float>(cy * cp * sr - sy * sp * cr),
            static_cast<float>(cy * sp * cr + sy * cp * sr),
            static_cast<float>(sy * cp * cr - cy * sp * sr)};
        return true;
    }
protected:
    virtual bool _init(int32_t) { return true; }
    sh2_Hal_t _HAL = {nullptr};
private:
    static int simulatedWrite(sh2_Hal_t *, uint8_t *, unsigned length)
    {
        if (simulated::reportInitFails) return 0;
        if (simulated::bnoResetDelivered && simulated::reenableFailuresRemaining) {
            --simulated::reenableFailuresRemaining;
            return 0;
        }
        return static_cast<int>(length);
    }
    bool enabled_ = false;
    uint32_t reportIntervalMs_ = 10;
    uint32_t lastSampleMs_ = 0;
};
