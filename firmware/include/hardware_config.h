#pragma once

#include <stdint.h>

// Authoritative TMP pin map, verified by physical testing on the current machine.
// Do not change these assignments without revalidating the wiring and hardware.
namespace tmp_hardware {

constexpr uint8_t YAW_DIR_PIN = 32;
constexpr uint8_t YAW_STEP_PIN = 33;

constexpr uint8_t PITCH_DIR_PIN = 26;
constexpr uint8_t PITCH_STEP_PIN = 12;

constexpr uint8_t CARRIAGE_DIR_PIN = 21;
constexpr uint8_t CARRIAGE_STEP_PIN = 22;

// Reserved for later sensor milestones. They are intentionally unused here.
constexpr uint8_t I2C_BUS_A_SDA_PIN = 18;
constexpr uint8_t I2C_BUS_A_SCL_PIN = 19;
constexpr uint8_t I2C_BUS_B_SDA_PIN = 4;
constexpr uint8_t I2C_BUS_B_SCL_PIN = 5;

}  // namespace tmp_hardware
