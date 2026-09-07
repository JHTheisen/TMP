#include <Arduino.h>
#include <FastAccelStepper.h>

#include "hardware_config.h"

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr int32_t TEST_STEPS = 500;
constexpr uint32_t TEST_SPEED_HZ = 1000;
constexpr int32_t TEST_ACCELERATION_STEPS_PER_SECOND_SQUARED = 1000;
constexpr uint16_t DIRECTION_CHANGE_DELAY_US = 200;
constexpr uint32_t STARTUP_DELAY_MS = 5000;
constexpr uint32_t DIRECTION_PAUSE_MS = 2000;
constexpr uint32_t AXIS_PAUSE_MS = 3000;

struct Axis {
    const char *name;
    uint8_t stepPin;
    uint8_t dirPin;
    FastAccelStepper *stepper;
};

FastAccelStepperEngine engine = FastAccelStepperEngine();

Axis yaw = {
    "YAW",
    tmp_hardware::YAW_STEP_PIN,
    tmp_hardware::YAW_DIR_PIN,
    nullptr,
};

Axis pitch = {
    "PITCH",
    tmp_hardware::PITCH_STEP_PIN,
    tmp_hardware::PITCH_DIR_PIN,
    nullptr,
};

Axis carriage = {
    "LINEAR CARRIAGE",
    tmp_hardware::CARRIAGE_STEP_PIN,
    tmp_hardware::CARRIAGE_DIR_PIN,
    nullptr,
};

Axis *const axes[] = {&yaw, &pitch, &carriage};

void stopAllSteppers()
{
    for (Axis *axis : axes)
    {
        if (axis->stepper != nullptr)
        {
            axis->stepper->forceStop();
        }
    }

    delay(25);  // FastAccelStepper may need about 20 ms to drain its queue.

    for (Axis *axis : axes)
    {
        pinMode(axis->stepPin, OUTPUT);
        digitalWrite(axis->stepPin, LOW);
    }
}

[[noreturn]] void haltWithError(const Axis &axis, const char *message, int result = 0)
{
    stopAllSteppers();
    Serial.println();
    Serial.print("FATAL: ");
    Serial.print(axis.name);
    Serial.print(": ");
    Serial.print(message);
    if (result != 0)
    {
        Serial.print(" (code ");
        Serial.print(result);
        Serial.print(')');
    }
    Serial.println();
    Serial.println("All STEP outputs are stopped. Reset the ESP32 after correcting the problem.");

    while (true)
    {
        delay(1000);
    }
}

void initializeAxis(Axis &axis)
{
    Serial.print("Initializing ");
    Serial.print(axis.name);
    Serial.print(" (STEP GPIO ");
    Serial.print(axis.stepPin);
    Serial.print(", DIR GPIO ");
    Serial.print(axis.dirPin);
    Serial.println(")...");

    axis.stepper = engine.stepperConnectToPin(axis.stepPin);
    if (axis.stepper == nullptr)
    {
        haltWithError(axis, "FastAccelStepper could not assign the STEP pin");
    }

    axis.stepper->setDirectionPin(axis.dirPin, true, DIRECTION_CHANGE_DELAY_US);

    const int8_t speedResult = axis.stepper->setSpeedInHz(TEST_SPEED_HZ);
    if (speedResult != 0)
    {
        haltWithError(axis, "test speed was rejected", speedResult);
    }

    const int8_t accelerationResult =
        axis.stepper->setAcceleration(TEST_ACCELERATION_STEPS_PER_SECOND_SQUARED);
    if (accelerationResult != 0)
    {
        haltWithError(axis, "test acceleration was rejected", accelerationResult);
    }
}

void waitForMoveToFinish(const Axis &axis)
{
    while (axis.stepper->isRunning())
    {
        delay(1);
    }
}

void commandRelativeMove(Axis &axis, int32_t steps, const char *directionName)
{
    Serial.print(axis.name);
    Serial.print(": moving ");
    Serial.print(directionName);
    Serial.print(" (");
    Serial.print(steps);
    Serial.println(" steps)");

    const auto moveResult = axis.stepper->move(steps);
    if (moveResult != MOVE_OK)
    {
        haltWithError(axis, "move command was rejected", static_cast<int>(moveResult));
    }

    waitForMoveToFinish(axis);

    Serial.print(axis.name);
    Serial.print(": move complete; step position = ");
    Serial.println(axis.stepper->getCurrentPosition());
}

void testAxis(Axis &axis)
{
    const int32_t startingPosition = axis.stepper->getCurrentPosition();

    Serial.println();
    Serial.print("=== Testing ");
    Serial.print(axis.name);
    Serial.println(" only ===");

    commandRelativeMove(axis, TEST_STEPS, "FORWARD / positive");

    Serial.print(axis.name);
    Serial.println(": stopped; pausing before reverse...");
    delay(DIRECTION_PAUSE_MS);

    commandRelativeMove(axis, -TEST_STEPS, "BACKWARD / negative");

    const int32_t finalPosition = axis.stepper->getCurrentPosition();
    if (finalPosition != startingPosition)
    {
        haltWithError(axis, "did not return to its starting step count");
    }

    Serial.print(axis.name);
    Serial.println(": returned to its starting step count");
    Serial.println("Axis test complete; pausing before the next axis...");
    delay(AXIS_PAUSE_MS);
}

void printPinMap()
{
    Serial.println("Verified current TMP pin map:");
    for (const Axis *axis : axes)
    {
        Serial.print("  ");
        Serial.print(axis->name);
        Serial.print(": DIR GPIO ");
        Serial.print(axis->dirPin);
        Serial.print(", STEP GPIO ");
        Serial.println(axis->stepPin);
    }
}

}  // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(500);

    Serial.println();
    Serial.println("TMP FastAccelStepper three-axis motion test");
    Serial.println("Milestone 1: open-loop STEP/DIR checkout only");
    printPinMap();

    engine.init();
    initializeAxis(yaw);
    initializeAxis(pitch);
    initializeAxis(carriage);

    Serial.println("All three FastAccelStepper objects initialized successfully.");
    Serial.print("WARNING: Motion begins in ");
    Serial.print(STARTUP_DELAY_MS / 1000);
    Serial.println(" seconds. Keep clear of the machine.");
    delay(STARTUP_DELAY_MS);

    testAxis(yaw);
    testAxis(pitch);
    testAxis(carriage);

    stopAllSteppers();
    Serial.println();
    Serial.println("PASS: Commanded motion test complete.");
    Serial.println("All axes returned to their starting step counts; no further motion will be commanded.");
    Serial.println("This confirms firmware command completion only. Verify physical motion separately.");
}

void loop()
{
    // This isolated diagnostic intentionally runs once after reset.
}
