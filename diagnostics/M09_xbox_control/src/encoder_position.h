#pragma once
#include <cmath>
#include <cstdint>
#include "encoder_acquisition.h"

namespace m09 {
constexpr uint32_t ENCODER_STALE_MS = 150;

// Unwrap in the acquisition task so skipped foreground snapshots cannot lose
// revolutions. A failed read, bad magnet, half-turn or stale gap breaks continuity.
struct EncoderPosition {
    bool valid = false, initialized = false;
    uint16_t raw = 0;
    int64_t ticks = 0;
    uint32_t at = 0, reads = 0, epoch = 0;
    void invalidate() {
        if (initialized) ++epoch;
        valid = initialized = false;
    }
    void observe(const EncoderState &sample, uint32_t now) {
        if (!sample.valid || !sample.magnetGood() || sample.ageMs(now) >= ENCODER_STALE_MS) {
            invalidate(); return;
        }
        if (initialized && sample.reads == reads) return;
        if (initialized && sample.lastGoodMs - at >= ENCODER_STALE_MS) invalidate();
        if (!initialized) { ticks = sample.raw; initialized = true; }
        else {
            int delta = int(sample.raw) - int(raw);
            if (delta > 2048) delta -= 4096;
            if (delta < -2048) delta += 4096;
            if (std::abs(delta) == 2048) { invalidate(); return; }
            ticks += delta;
        }
        raw = sample.raw; at = sample.lastGoodMs; reads = sample.reads; valid = true;
    }
    bool fresh(uint32_t now) const { return valid && now - at < ENCODER_STALE_MS; }
};

struct EncoderReference {
    // Signed output-axis degrees per encoder revolution. Zero means unknown.
    // No default mounting or reduction assumption.
    double degreesPerRevolution = 0;
    bool calibrated = false;
    int64_t zeroTicks = 0;
    uint32_t epoch = 0;
    bool configure(double scale) {
        if (!std::isfinite(scale) || scale == 0 || std::abs(scale) > 360.0) return false;
        degreesPerRevolution = scale; calibrated = false; return true;
    }
    void observe(const EncoderPosition &position, uint32_t now) {
        if (!position.fresh(now) || position.epoch != epoch) calibrated = false;
    }
    bool setZero(const EncoderPosition &position, uint32_t now) {
        if (!degreesPerRevolution || !position.fresh(now)) return false;
        zeroTicks = position.ticks; epoch = position.epoch; calibrated = true; return true;
    }
    bool ready(const EncoderPosition &position, uint32_t now) const {
        return calibrated && degreesPerRevolution != 0 && position.fresh(now) && epoch == position.epoch;
    }
    double angle(const EncoderPosition &position, uint32_t now) const {
        return ready(position, now) ? double(position.ticks - zeroTicks) * degreesPerRevolution / 4096.0 : NAN;
    }
};
} // namespace m09
