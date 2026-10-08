#pragma once
#include <Arduino.h>
#include "hardware_config.h"
#include "encoder_position.h"
#include "sensor_handoff.h"
#ifndef M07_HOST_TEST
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif
namespace m09 {
struct EncoderSnapshot {
    EncoderState samples[2];
    const EncoderState &state(unsigned n) const { return samples[n]; }
};
struct SensorSnapshot {
    EncoderSnapshot encoders;
    EncoderPosition positions[2];
    bool setupDone = false;
};
class SensorWorker {
public:
    SnapshotMailbox<SensorSnapshot> status;
    bool start() {
#ifdef M07_HOST_TEST
        started_ = true; return true;
#else
        return xTaskCreatePinnedToCore(taskEntry, "m09_encoders", 4096, this, 1, nullptr, 0) == pdPASS;
#endif
    }
#ifdef M07_HOST_TEST
    void testDispatch() {
        if (!started_ || inFlight_) return;
        inFlight_ = true; step(); inFlight_ = false;
    }
#endif
private:
    EncoderAcquisition acquisition_{Wire, Wire1};
    SensorSnapshot state_;
    uint32_t busRetryAt_ = 0;
#ifdef M07_HOST_TEST
    bool started_ = false, inFlight_ = false;
#else
    static void taskEntry(void *context) {
        auto &worker = *static_cast<SensorWorker *>(context);
        for (;;) { worker.step(); vTaskDelay(pdMS_TO_TICKS(1)); }
    }
#endif
    void step() {
        if (!state_.setupDone) {
            const bool a = Wire.begin(tmp_hardware::I2C_BUS_A_SDA_PIN, tmp_hardware::I2C_BUS_A_SCL_PIN, 100000);
            const bool b = Wire1.begin(tmp_hardware::I2C_BUS_B_SDA_PIN, tmp_hardware::I2C_BUS_B_SCL_PIN, 100000);
            Wire.setTimeOut(20); Wire1.setTimeOut(20);
            acquisition_.begin(a, b, millis()); state_.setupDone = true;
        }
        // Failed bus initialization is recoverable without resetting the other
        // encoder's continuity. Transactions themselves are retried by service().
        if (millis() - busRetryAt_ >= 1000) {
            busRetryAt_ = millis();
            if (!acquisition_.state(0).busAvailable && Wire.begin(
                    tmp_hardware::I2C_BUS_A_SDA_PIN, tmp_hardware::I2C_BUS_A_SCL_PIN, 100000)) {
                Wire.setTimeOut(20); acquisition_.busRecovered(0);
            }
            if (!acquisition_.state(1).busAvailable && Wire1.begin(
                    tmp_hardware::I2C_BUS_B_SDA_PIN, tmp_hardware::I2C_BUS_B_SCL_PIN, 100000)) {
                Wire1.setTimeOut(20); acquisition_.busRecovered(1);
            }
        }
        acquisition_.service(millis());
        for (unsigned n = 0; n < 2; ++n) {
            state_.encoders.samples[n] = acquisition_.state(n);
            state_.positions[n].observe(state_.encoders.samples[n], millis());
        }
        status.publish(state_);
    }
};
} // namespace m09
