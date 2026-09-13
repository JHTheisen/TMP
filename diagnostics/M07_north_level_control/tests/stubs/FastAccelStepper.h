#pragma once
#include <Arduino.h>
constexpr int8_t MOVE_OK = 0;
class FastAccelStepper {
public:
    uint8_t stepPin = 0, directionPin = 0;
    unsigned index = 0;
    simulated::PlantMotor &plant() { return simulated::motors[index]; }
    void setDirectionPin(uint8_t pin, bool, uint16_t) { directionPin = pin; }
    int8_t setSpeedInHz(uint32_t value) { plant().speed = value; return simulated::speedFails ? -1 : 0; }
    int8_t setAcceleration(int32_t value) { plant().acceleration = value; return simulated::accelerationFails ? -1 : 0; }
    int8_t move(int32_t steps) { return command(steps, false); }
    int8_t runForward() { return command(1, true); }
    int8_t runBackward() { return command(-1, true); }
    bool isRunning() { return plant().drive != simulated::Drive::IDLE; }
    int32_t getCurrentPosition() { return static_cast<int32_t>(plant().position); }
    int32_t getCurrentSpeedInMilliHz() { return static_cast<int32_t>(plant().velocity * 1000); }
    void stopMove() {
        if (isRunning()) plant().drive = simulated::Drive::BRAKING;
        ++simulated::gentleStops;
    }
    void forceStop() {
        if (!simulated::forceStops) simulated::firstForceStopAt = millis();
        plant().drive = simulated::Drive::IDLE; plant().velocity = 0;
        plant().stoppedAt = millis(); plant().needsStoppedSample = true;
        ++simulated::forceStops;
    }
private:
    int8_t command(int32_t amount, bool continuous) {
        if (simulated::moveRejected && (!simulated::rejectedStepPin || simulated::rejectedStepPin == stepPin)) return -1;
        auto &motor = plant();
        simulated::commands.push_back({stepPin, amount, motor.speed, motor.acceleration,
            millis(), simulated::lastSampleAt, simulated::actualYaw(), simulated::actualPitch(), continuous,
            motor.drive == simulated::Drive::BRAKING,
            motor.needsStoppedSample && simulated::lastSampleAt <= motor.stoppedAt});
        motor.target = motor.position + amount; motor.direction = amount > 0 ? 1 : -1;
        motor.drive = continuous ? simulated::Drive::CONTINUOUS : simulated::Drive::FINITE;
        motor.needsStoppedSample = false;
        return MOVE_OK;
    }
};
class FastAccelStepperEngine {
public:
    FastAccelStepper motors[2];
    unsigned count = 0;
    void init() {}
    FastAccelStepper *stepperConnectToPin(uint8_t pin) {
        simulated::connectedPins.push_back(pin);
        if (simulated::axisInitFails || count >= 2) return nullptr;
        auto &motor = motors[count]; motor.stepPin = pin; motor.index = count++;
        if (motor.index == 1) { motor.plant().physicalSign = 1; motor.plant().degreesPerStep = 0.025; }
        return &motor;
    }
};
