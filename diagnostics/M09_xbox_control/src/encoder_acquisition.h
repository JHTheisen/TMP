#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace m09 {

// These are wrapped magnet/shaft readings, not calibrated cradle coordinates.
// A complete sample remains readable when magnet flags are non-ideal; callers
// must report those flags separately instead of claiming calibrated feedback.
struct EncoderState {
    bool busAvailable = false;
    bool available = false; // The most recent register-address transfer ACKed.
    bool valid = false;     // The most recent attempt returned all three bytes.
    bool hasSample = false;
    uint16_t raw = 0;
    uint8_t status = 0;
    uint32_t lastAttemptMs = 0, lastGoodMs = 0;
    uint32_t reads = 0, failures = 0;

    uint32_t ageMs(uint32_t now) const {
        return hasSample ? static_cast<uint32_t>(now - lastGoodMs) : UINT32_MAX;
    }
    double angleDegrees() const { return raw * (360.0 / 4096.0); }
    bool magnetGood() const { return hasSample && (status & 0x38U) == 0x20U; }
};

class EncoderAcquisition {
public:
    EncoderAcquisition(TwoWire &busA, TwoWire &busB) : buses_{&busA, &busB} {}

    // The caller initializes the buses and their existing bounded Wire timeout.
    // Begin performs no I2C, scan, retry, calibration or device configuration.
    void begin(bool busAAvailable, bool busBAvailable, uint32_t now) {
        states_[0] = EncoderState{};
        states_[1] = EncoderState{};
        states_[0].busAvailable = busAAvailable;
        states_[1].busAvailable = busBAvailable;
        next_ = 0;
        lastServiceMs_ = now - SERVICE_INTERVAL_MS;
    }

    // Alternate buses at a 10 ms minimum cadence (~20 ms per device). One call
    // performs at most one read; there are no catch-up reads after a slow call.
    bool service(uint32_t now) {
        if (static_cast<uint32_t>(now - lastServiceMs_) < SERVICE_INTERVAL_MS) return false;
        lastServiceMs_ = now;
        const unsigned index = next_;
        next_ ^= 1U;
        EncoderState &sample = states_[index];
        if (!sample.busAvailable) return false;
        sample.lastAttemptMs = now;
        sample.available = sample.valid = false;
        TwoWire &bus = *buses_[index];

        // Reuse pitch_direction_check's read-only STATUS/RAW_ANGLE transaction.
        // The single write selects register 0x0B; no configuration/OTP is written.
        bus.beginTransmission(ADDRESS);
        if (bus.write(static_cast<uint8_t>(0x0B)) != 1 || bus.endTransmission(false) != 0) {
            ++sample.failures;
            return true;
        }
        sample.available = true;
        if (bus.requestFrom(ADDRESS, static_cast<uint8_t>(3)) != 3) {
            ++sample.failures;
            return true;
        }
        const int status = bus.read(), high = bus.read(), low = bus.read();
        if (status < 0 || high < 0 || low < 0 || (high & 0xF0)) {
            ++sample.failures;
            return true;
        }
        sample.status = static_cast<uint8_t>(status);
        sample.raw = static_cast<uint16_t>((high << 8) | low);
        sample.valid = sample.hasSample = true;
        sample.lastGoodMs = millis();
        ++sample.reads;
        return true;
    }

    // Index 0 is Bus A; index 1 is Bus B. No axis mapping is inferred here.
    const EncoderState &state(unsigned index) const { return states_[index]; }
    void busRecovered(unsigned index) { states_[index].busAvailable = true; }
    uint32_t attempts(unsigned index) const { return states_[index].reads + states_[index].failures; }

private:
    static constexpr uint8_t ADDRESS = 0x36;
    static constexpr uint32_t SERVICE_INTERVAL_MS = 10;
    TwoWire *buses_[2];
    EncoderState states_[2];
    uint32_t lastServiceMs_ = 0;
    unsigned next_ = 0;
};

} // namespace m09
