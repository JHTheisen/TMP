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
    int8_t setSpeedInMilliHz(uint32_t value) { plant().speed = value / 1000.0; return simulated::speedFails ? -1 : 0; }
    uint32_t getSpeedInMilliHz() { return static_cast<uint32_t>(plant().speed * 1000); }
    uint32_t getAcceleration() { return plant().acceleration; }
    int8_t setAcceleration(int32_t value) { plant().acceleration = value; return simulated::accelerationFails ? -1 : 0; }
    int8_t move(int32_t steps) { return command(steps, false); }
    int8_t moveTo(int32_t position) {
        const auto result = command(position - getCurrentPosition(), false);
        // The synthetic continuous plant may brake between integer steps, but
        // an absolute generated-step endpoint is still the requested integer.
        if (result == MOVE_OK) plant().target = position;
        return result;
    }
    int8_t runForward() { return command(1, true); }
    int8_t runBackward() { return command(-1, true); }
    void applySpeedAcceleration() {} // The fixture integrates the configured rate on its next independent tick.
    bool isRunning() {
        auto &motor = plant();
        if (motor.idleForceStopLatched && motor.drive == simulated::Drive::CONTINUOUS && millis() >= motor.runningVisibleAt) {
            motor.drive = simulated::Drive::IDLE; motor.velocity = 0; motor.idleForceStopLatched = false;
            return false;
        }
        return motor.drive != simulated::Drive::IDLE && millis() >= motor.runningVisibleAt;
    }
    int32_t getCurrentPosition() { return static_cast<int32_t>(plant().position); }
    int32_t getCurrentSpeedInMilliHz() { return static_cast<int32_t>(plant().velocity * 1000); }
    void stopMove() {
        if (isRunning()) plant().drive = simulated::Drive::BRAKING;
        ++simulated::gentleStops;
    }
    void forceStop() {
        if (!simulated::forceStops) simulated::firstForceStopAt = millis();
        if (simulated::latchIdleForceStop && plant().drive == simulated::Drive::IDLE)
            plant().idleForceStopLatched = true;
        plant().drive = simulated::Drive::IDLE; plant().velocity = 0; plant().runningVisibleAt = 0;
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
        motor.activePhysicalMultiplier = continuous ?
            motor.continuousPhysicalMultiplier : motor.finitePhysicalMultiplier;
        motor.drive = continuous ? simulated::Drive::CONTINUOUS : simulated::Drive::FINITE;
        motor.runningVisibleAt = continuous ? millis() + simulated::continuousStartDelayMs : 0;
        motor.needsStoppedSample = false;
        return MOVE_OK;
    }
};
class FastAccelStepperEngine {
public:
    FastAccelStepper motors[3];
    unsigned count = 0;
    void init() {}
    FastAccelStepper *stepperConnectToPin(uint8_t pin) {
        simulated::connectedPins.push_back(pin);
        if (simulated::axisInitFails || count >= 3) return nullptr;
        auto &motor = motors[count]; motor.stepPin = pin; motor.index = count++;
        // Synthetic response for the current -1 sign / 48-pulse correction gain;
        // this is a test plant, not a measurement of the physical gearing.
        if (motor.index == 1) { motor.plant().physicalSign = -1; motor.plant().degreesPerStep = 0.01; }
        if (motor.index == 2) { motor.plant().physicalSign = 1; motor.plant().degreesPerStep = 0; }
        return &motor;
    }
};
