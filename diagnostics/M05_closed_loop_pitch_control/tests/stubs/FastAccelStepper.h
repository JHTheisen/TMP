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
        if (simulated::moveRejected) return -1;
        simulated::startSteps = simulated::position();
        const double pitchAt = simulated::actualPitch();
        simulated::targetSteps = simulated::startSteps + steps;
        simulated::moveStart = millis();
        const double distance = abs(steps);
        // Approximate acceleration-limited duration, with linear discrete pulses.
        // This is only a deterministic fixture, never a validated drivetrain model.
        const double rampDistance = static_cast<double>(speed) * speed / acceleration;
        const double seconds = distance <= rampDistance ? 2 * sqrt(distance / acceleration) :
                               2.0 * speed / acceleration + (distance - rampDistance) / speed;
        simulated::moveDuration = static_cast<uint32_t>(ceil(1000 * seconds));
        simulated::moving = true;
        simulated::commands.push_back({stepPin, steps, speed, acceleration, millis(), pitchAt});
        return MOVE_OK;
    }
    bool isRunning() { simulated::position(); return simulated::moving; }
    int32_t getCurrentPosition() { return simulated::position(); }
    void forceStop()
    {
        simulated::targetSteps = simulated::position();
        simulated::moving = false;
        ++simulated::forceStops;
    }
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
