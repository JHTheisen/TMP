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
    bool begin_I2C(uint8_t, TwoWire *)
    {
        _HAL.write = simulatedWrite;
        return _init(0);
    }
    bool enableReport(uint8_t, uint32_t intervalUs)
    {
        reportIntervalMs_ = intervalUs / 1000;
        uint8_t command = 0;
        // Invoke the real diagnostic HAL wrapper. A raw zero would make the
        // pinned SHTP sender retry indefinitely; the wrapper must return < 0.
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
        if (!enabled_ || millis() - lastSampleMs_ < reportIntervalMs_ ||
            (millis() >= simulated::bnoPauseStart && millis() < simulated::bnoPauseEnd))
            return false;
        lastSampleMs_ = millis();
        event->sensorId = SH2_ROTATION_VECTOR;
        event->status = 3;
        event->un.rotationVector = {1.0F, 0.0F, 0.0F, 0.0F};
        return true;
    }
protected:
    virtual bool _init(int32_t) { return true; }
    sh2_Hal_t _HAL = {nullptr};
private:
    static int simulatedWrite(sh2_Hal_t *, uint8_t *, unsigned length)
    {
        if (simulated::bnoResetDelivered && simulated::reenableFailuresRemaining > 0)
        {
            --simulated::reenableFailuresRemaining;
            return 0;
        }
        return static_cast<int>(length);
    }
    bool enabled_ = false;
    uint32_t reportIntervalMs_ = 10;
    uint32_t lastSampleMs_ = 0;
};
