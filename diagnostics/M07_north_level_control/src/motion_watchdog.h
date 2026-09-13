#pragma once
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <atomic>
#include <stdint.h>
#ifndef M07_HOST_TEST
#include <esp_timer.h>
#endif

namespace milestone7 {

// Lives for the firmware lifetime. The ESP timer task checks independently of
// loop(), so an I2C/library call that blocks loop() cannot postpone the stop.
// begin/arm/recordFresh/disarm have one foreground caller; check also runs in
// the timer task. No sensor or Serial operation belongs in that callback.
class MotionWatchdog {
public:
    static constexpr uint32_t CHECK_INTERVAL_US = 10000;

    bool begin(FastAccelStepper *yaw, FastAccelStepper *pitch, uint32_t timeoutMs) {
        if (configured_ || !yaw || !pitch || !timeoutMs || timeoutMs >= 0x80000000UL)
            return false;
        yaw_ = yaw;
        pitch_ = pitch;
        timeoutMs_ = timeoutMs;
#ifndef M07_HOST_TEST
        esp_timer_create_args_t args = {};
        args.callback = timerCallback;
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "m07_bno_stop";
        if (esp_timer_create(&args, &timer_) != ESP_OK) return false;
        if (esp_timer_start_periodic(timer_, CHECK_INTERVAL_US) != ESP_OK) {
            esp_timer_delete(timer_);
            timer_ = nullptr;
            return false;
        }
#endif
        configured_ = true;
        return true;
    }

    // Pass the last accepted BNO sample time, not a replacement current time.
    // A trip requires a board reset; neither disarm nor a later sample clears it.
    bool arm(uint32_t lastFreshMs) {
        if (!configured_ || state_.load(std::memory_order_acquire) != 0) return false;
        lastFreshMs_.store(lastFreshMs, std::memory_order_relaxed);
        uint32_t expected = 0;
        return state_.compare_exchange_strong(expected, ARMED,
                                               std::memory_order_release,
                                               std::memory_order_relaxed);
    }

    void recordFresh(uint32_t now) {
        // Check the old timestamp first. A late getSensorEvent return must not
        // hide a stale interval, even if the timer task has not run yet.
        check(now);
        if (state_.load(std::memory_order_acquire) == ARMED)
            lastFreshMs_.store(now, std::memory_order_release);
        // The timer may trip between that load and store; only the timestamp
        // changes, so this race cannot clear the independently latched trip.
    }

    void disarm() { state_.fetch_and(~ARMED, std::memory_order_acq_rel); }

    bool tripped() const {
        return (state_.load(std::memory_order_acquire) & TRIPPED) != 0;
    }

    // Public to let host fixtures advance the watchdog during a blocked BNO
    // call. Production invokes this from an independent periodic ESP timer.
    void check(uint32_t now) {
        uint32_t state = state_.load(std::memory_order_acquire);
        if (!(state & ARMED)) return;
        if (!(state & TRIPPED)) {
            const uint32_t age = now - lastFreshMs_.load(std::memory_order_acquire);
            // A fresh write can occur just after this callback captured now.
            // Treat that small "future" timestamp as fresh, not unsigned-wrap
            // staleness. Ordinary millis() rollover still subtracts correctly.
            if (age < timeoutMs_ || age >= 0x80000000UL) return;
            if (!state_.compare_exchange_strong(state, ARMED | TRIPPED,
                                                std::memory_order_acq_rel,
                                                std::memory_order_acquire)) {
                if (!(state & ARMED) || !(state & TRIPPED)) return;
            }
        }
        // FastAccelStepper 1.2.7 documents forceStop as interrupt-safe. It
        // drains already queued commands (normally about 20 ms); this is not
        // a zero-latency physical stop. Reassert while armed to cover a command
        // already in flight when the watchdog trips. Foreground code must also
        // reject commands once tripped and disarm only after latching idle.
        yaw_->forceStop();
        pitch_->forceStop();
    }

private:
    static constexpr uint32_t ARMED = 1;
    static constexpr uint32_t TRIPPED = 2;
    std::atomic<uint32_t> state_{0};
    std::atomic<uint32_t> lastFreshMs_{0};
    FastAccelStepper *yaw_ = nullptr;
    FastAccelStepper *pitch_ = nullptr;
    uint32_t timeoutMs_ = 0;
    bool configured_ = false;
#ifndef M07_HOST_TEST
    esp_timer_handle_t timer_ = nullptr;
    static void timerCallback(void *argument) {
        static_cast<MotionWatchdog *>(argument)->check(millis());
    }
#endif
};

} // namespace milestone7
