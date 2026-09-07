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
        simulated::startSteps = simulated::position();
        simulated::targetSteps = simulated::startSteps + steps;
        simulated::moveStart = millis();
        // Synthetic duration, not a physical motor model.
        simulated::moveDuration = static_cast<uint32_t>(ceil(2000.0 * sqrt(abs(steps) / 1000.0)));
        simulated::moving = true;
        simulated::commands.push_back({stepPin, steps});
        return MOVE_OK;
    }
    bool isRunning() { simulated::position(); return simulated::moving; }
    int32_t getCurrentPosition() { return simulated::position(); }
    void forceStop() { simulated::targetSteps = simulated::position(); simulated::moving = false; }
};
class FastAccelStepperEngine {
public:
    FastAccelStepper motor;
    void init() {}
    FastAccelStepper *stepperConnectToPin(uint8_t pin)
    {
        simulated::connectedPins.push_back(pin);
        if (simulated::axisInitFails) return nullptr;
        motor.stepPin = pin;
        return &motor;
    }
};
