#pragma once
#include <Arduino.h>
#include <algorithm>
#include "hardware_config.h"
#include "sensor_support.h"
#include "encoder_acquisition.h"
#include "bno_diagnostics.h"
#include "bno_trace_types.h"
#include "sensor_handoff.h"
#ifndef M07_HOST_TEST
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif
extern "C" int m09_sh2_request_product_id(void);

namespace m09 {
struct SensorControlContext {
    BnoTraceEvent trace;
    bool queryIdle = false, halted = false;
};
struct EncoderSnapshot {
    EncoderState samples[2];
    const EncoderState &state(unsigned n) const { return samples[n]; }
};
struct SensorSnapshot {
    bool setupDone = false, bnoInitialized = false, reportEnabled = false;
    uint32_t consumedResets = 0, reportFailures = 0, writeFailures = 0, sampleDrops = 0;
    EncoderSnapshot encoders;
    BnoIoStatistics io;
};
struct SensorSample {
    sh2_SensorValue_t event{};
    uint32_t receivedMs = 0, epoch = 0;
};

class SensorWorker {
public:
    SnapshotMailbox<SensorControlContext> control;
    SnapshotMailbox<SensorSnapshot> status;
    SensorQueue<SensorSample, 32> samples;
    SensorQueue<BnoTraceEvent, 32> traces;
    // Reset invalidation is sticky even if a bounded sample/trace queue fills.
    std::atomic<uint32_t> resetEpoch{0};

    bool start() {
#ifdef M07_HOST_TEST
        started_ = true;
        return true;
#else
        return xTaskCreatePinnedToCore(taskEntry, "m09_sensors", 8192, this, 1,
                                      nullptr, 0) == pdPASS;
#endif
    }
#ifdef M07_HOST_TEST
    // Deterministic fixture dispatches the same worker body. Sensor waits yield
    // to real foreground loop() calls; inFlight_ prevents recursive sensor I/O.
    void testDispatch() {
        if (!started_ || inFlight_) return;
        inFlight_ = true; step(); inFlight_ = false;
    }
#endif
    void onIo(uint32_t start, uint32_t duration, unsigned stage, unsigned bytes, int ok) {
        ++state_.io.ioCalls;
        if (!ok) ++state_.io.ioFailures;
        if (duration >= 10000) ++state_.io.ioSlow;
        state_.io.ioMaxUs = std::max(state_.io.ioMaxUs, duration);
        state_.io.lastIoStart = start; state_.io.lastIoDuration = duration;
        state_.io.lastIoStage = stage; state_.io.lastIoResult = ok;
        if (!ok || duration >= 10000) trace("I2C", start, duration, stage, bytes, 0, ok);
    }
    void onReset(uint32_t transportUs) {
        ++state_.io.resetEvents;
        resetEpoch.fetch_add(1, std::memory_order_release);
        trace("RESET_COMPLETE", micros(), 0, transportUs);
    }
    void onProduct(uint32_t transportUs, uint8_t cause, uint32_t part, uint32_t build) {
        state_.io.productGeneration = state_.io.resetEvents;
        if (state_.io.queryPending) ++state_.io.queryResponses;
        trace("PRODUCT_ID", transportUs, 0, cause, part, build);
    }
    void onInitResponse(uint32_t transportUs) { trace("UNSOLICITED_INIT", transportUs, 0); }

private:
    // These devices and every SH-2 call have exactly one owner, including setup,
    // report recovery, product queries, and both AS5600 bus transactions.
    milestone4::DiagnosticBno085 bno_;
    EncoderAcquisition encoders_{Wire, Wire1};
    SensorSnapshot state_;
    SensorControlContext context_;
    BnoDiagnostics raw_;
    sh2_SensorValue_t event_{}; // library callback pointer remains valid between calls
    uint32_t lastReportAttempt_ = 0, consumedEpoch_ = 0;
#ifdef M07_HOST_TEST
    bool started_ = false, inFlight_ = false;
#endif
    void publish() {
        state_.encoders.samples[0] = encoders_.state(0);
        state_.encoders.samples[1] = encoders_.state(1);
        state_.writeFailures = milestone4::DiagnosticBno085::writeFailures;
        status.publish(state_); // contended snapshot retries next worker iteration
    }
    void trace(const char *kind, uint32_t start, uint32_t duration,
               uint32_t a=0, uint32_t b=0, uint32_t c=0, int result=0) {
        control.read(context_);
        BnoTraceEvent e = context_.trace;
        e.contextAtMs = e.atMs;
        e.kind = kind; e.atMs = millis(); e.atUs = micros(); e.startUs = start; e.durationUs = duration;
        if (e.bnoAge != UINT32_MAX) e.bnoAge += e.atMs - e.contextAtMs;
        e.isFresh = e.isFresh && e.bnoAge < 150;
        e.resetEvent = state_.io.resetEvents; e.a=a; e.b=b; e.c=c; e.result=result;
        const auto &encoder = encoders_.state(1);
        e.encoderAge = encoder.ageMs(e.atMs); e.encoderAttempt = encoder.lastAttemptMs;
        e.encoderFailures = encoder.failures;
        e.lastIoStart = state_.io.lastIoStart; e.lastIoDuration = state_.io.lastIoDuration;
        e.lastIoStage = state_.io.lastIoStage; e.lastIoResult = state_.io.lastIoResult;
        e.rawStatus = raw_.status; e.sequence = raw_.sequence;
        e.normSquared = raw_.normSquared; e.headingAccuracy = raw_.rotation.accuracy;
        if (!traces.push(e)) ++state_.io.dropped;
    }
    void enableReport() {
        lastReportAttempt_ = millis();
        state_.reportEnabled = bno_.enableReport(SH2_ROTATION_VECTOR, milestone4::BNO_REPORT_INTERVAL_US);
        if (!state_.reportEnabled) ++state_.reportFailures;
        trace("REPORT_ENABLE", micros(), 0, SH2_ROTATION_VECTOR, milestone4::BNO_REPORT_INTERVAL_US, 0, state_.reportEnabled);
    }
    bool handleReset() {
        if (!bno_.wasReset()) return false;
        ++state_.consumedResets;
        // Fixture/legacy flag without the instrumented callback still invalidates.
        if (resetEpoch.load(std::memory_order_acquire) == consumedEpoch_)
            resetEpoch.fetch_add(1, std::memory_order_release);
        consumedEpoch_ = resetEpoch.load(std::memory_order_acquire);
        trace("RESET_CONSUMED", micros(), 0, state_.consumedResets);
        state_.reportEnabled = false;
        publish();
        enableReport();
        return true;
    }
    void initialize() {
        const bool a = Wire.begin(tmp_hardware::I2C_BUS_A_SDA_PIN, tmp_hardware::I2C_BUS_A_SCL_PIN, 100000);
        const bool b = Wire1.begin(tmp_hardware::I2C_BUS_B_SDA_PIN, tmp_hardware::I2C_BUS_B_SCL_PIN, 100000);
        Wire.setTimeOut(50); Wire1.setTimeOut(50);
        encoders_.begin(a, b, millis());
        if (b) {
            Wire1.beginTransmission(0x4A);
            if (Wire1.endTransmission() == 0) {
                state_.bnoInitialized = bno_.begin_I2C(0x4A, &Wire1);
                if (state_.io.productGeneration == state_.io.resetEvents)
                    state_.io.queriedGeneration = state_.io.resetEvents;
                if (state_.bnoInitialized) enableReport();
            }
        }
        state_.setupDone = true;
        publish();
    }
    void serviceResetCause() {
        auto &io = state_.io;
        if (io.queryPending && (millis()-io.queryAt >= 1000 || io.queryGeneration != io.resetEvents)) {
            trace("PRODUCT_QUERY_WINDOW_END", micros(), (millis()-io.queryAt)*1000U,
                  io.queryGeneration, io.resetEvents, io.queryResponses, io.queryResponses ? 0 : -6);
            io.queryPending = false;
        }
        control.read(context_);
        if (!state_.bnoInitialized || !state_.reportEnabled || !context_.queryIdle || context_.halted ||
            io.queryPending || io.resetEvents <= io.queriedGeneration) return;
        io.queriedGeneration = io.resetEvents; io.queryGeneration = io.resetEvents;
        io.queryAt = millis(); io.queryResponses = 0;
        const uint32_t start = micros();
        const int result = m09_sh2_request_product_id();
        io.queryPending = result == 0;
        trace("PRODUCT_QUERY_SENT", start, uint32_t(micros()-start), io.queryGeneration, 0, 0, result);
    }
    void step() {
        control.read(context_);
        if (context_.halted) { publish(); return; }
        if (!state_.setupDone) initialize();
        control.read(context_);
        if (context_.halted) { publish(); return; }
        if (state_.bnoInitialized) {
            handleReset();
            if (!state_.reportEnabled && millis()-lastReportAttempt_ >= 500) enableReport();
            event_ = {};
            const uint32_t start = micros();
            const bool got = bno_.getSensorEvent(&event_);
            const uint32_t receivedMs = millis(), elapsed = micros()-start;
            state_.io.acquireMaxUs = std::max(state_.io.acquireMaxUs, elapsed);
            if (elapsed >= 10000) trace("ACQUIRE_SLOW", start, elapsed, 0, 0, 0, got);
            if (got) {
                raw_.observe(event_, receivedMs);
                if (event_.sensorId == SH2_ROTATION_VECTOR && !raw_.plausible)
                    trace("MALFORMED_RV", start, elapsed, event_.sequence, event_.status);
            }
            const bool reset = handleReset();
            if (got && !reset && state_.reportEnabled) {
                SensorSample sample; sample.event=event_; sample.receivedMs=receivedMs;
                sample.epoch=resetEpoch.load(std::memory_order_acquire);
                if (!samples.push(sample)) ++state_.sampleDrops;
            }
        }
        control.read(context_);
        if (!context_.halted) {
            const uint32_t start = micros(), attempts = encoders_.attempts(1);
            encoders_.service(millis());
            if (encoders_.attempts(1) != attempts) {
                const uint32_t elapsed = micros()-start;
                state_.io.encoderMaxUs = std::max(state_.io.encoderMaxUs, elapsed);
                if (elapsed >= 10000 || (!encoders_.state(1).valid && millis()-state_.io.encoderTraceAt >= 500)) {
                    const auto &e = encoders_.state(1);
                    trace("ENCODER_B", start, elapsed, e.status, e.raw, e.failures, e.valid);
                    state_.io.encoderTraceAt = millis();
                }
            }
            serviceResetCause();
        }
        publish();
    }
#ifndef M07_HOST_TEST
    static void taskEntry(void *arg) {
        auto &worker = *static_cast<SensorWorker *>(arg);
        for (;;) { worker.step(); vTaskDelay(pdMS_TO_TICKS(1)); }
    }
#endif
};
} // namespace m09
