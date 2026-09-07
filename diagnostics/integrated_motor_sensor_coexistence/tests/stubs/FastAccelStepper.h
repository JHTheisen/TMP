#pragma once
#include <Arduino.h>

constexpr int8_t MOVE_OK = 0;
class FastAccelStepper {
public:
    uint8_t stepPin = 0;
    uint8_t directionPin = 0;
    uint32_t speed = 0;
    int32_t acceleration = 0;
    void setDirectionPin(uint8_t pin, bool, uint16_t) { directionPin = pin; }
    int8_t setSpeedInHz(uint32_t value) { speed = value; return 0; }
    int8_t setAcceleration(int32_t value) { acceleration = value; return 0; }
    int8_t move(int32_t steps)
    {
        simulated::commands.push_back({stepPin, steps});
        start_ = getCurrentPosition();
        target_ = start_ + steps;
        startedAt_ = millis();
        running_ = true;
        return MOVE_OK;
    }
    bool isRunning()
    {
        if (running_ && millis() - startedAt_ >= durationMs_) running_ = false;
        return running_;
    }
    int32_t getCurrentPosition()
    {
        if (!isRunning()) return target_;
        return start_ + static_cast<int32_t>((target_ - start_) *
                       static_cast<int64_t>(millis() - startedAt_) / durationMs_);
    }
    void forceStop()
    {
        target_ = getCurrentPosition();
        running_ = false;
    }
private:
    // Approximately the triangular-profile duration of the real command.
    // Position interpolation is only a reporting fixture, not a motion model.
    static constexpr uint32_t durationMs_ = 1415;
    bool running_ = false;
    uint32_t startedAt_ = 0;
    int32_t start_ = 0;
    int32_t target_ = 0;
};
class FastAccelStepperEngine {
public:
    void init() {}
    FastAccelStepper *stepperConnectToPin(uint8_t pin)
    {
        if (simulated::axisInitFails && connected_ == 0) { ++connected_; return nullptr; }
        auto *stepper = &steppers_[connected_++];
        stepper->stepPin = pin;
        return stepper;
    }
private:
    FastAccelStepper steppers_[3];
    unsigned connected_ = 0;
};
