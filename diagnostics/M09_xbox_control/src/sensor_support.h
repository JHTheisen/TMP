#pragma once
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// Copied from physically verified Milestone 3; no edits to that project.
namespace milestone4 {
constexpr uint32_t BNO_REPORT_INTERVAL_US = 10000;
constexpr uint32_t BNO_STALE_AFTER_MS = 50 * (BNO_REPORT_INTERVAL_US / 1000);
constexpr uint32_t BNO_PERSISTENT_AFTER_MS = 200 * (BNO_REPORT_INTERVAL_US / 1000);
struct EulerAngles { float heading; float pitch; float roll; };
// In BNO08x 1.2.7 the I2C HAL returns 0 for a failed write, but SHTP treats
// 0 as "retry forever". Turn that transport failure into SH2_ERR_IO so report
// enabling returns to our recovery/state machine. Successful writes are unchanged.
class DiagnosticBno085 : public Adafruit_BNO08x {
public:
    DiagnosticBno085() : Adafruit_BNO08x(-1) {}
    static uint32_t writeFailures;

protected:
    bool _init(int32_t sensorId) override
    {
        originalWrite = _HAL.write;
        _HAL.write = checkedWrite;
        return Adafruit_BNO08x::_init(sensorId);
    }

private:
    static int (*originalWrite)(sh2_Hal_t *, uint8_t *, unsigned);
    static int checkedWrite(sh2_Hal_t *hal, uint8_t *buffer, unsigned length)
    {
        const int result = originalWrite(hal, buffer, length);
        if (result <= 0)
        {
            ++writeFailures;
            return result == 0 ? SH2_ERR_IO : result;
        }
        return result;
    }
};

uint32_t DiagnosticBno085::writeFailures = 0;
int (*DiagnosticBno085::originalWrite)(sh2_Hal_t *, uint8_t *, unsigned) = nullptr;
bool quaternionToEuler(const sh2_RotationVectorWAcc_t &rotation, EulerAngles &angles)
{
    const float realSquared = rotation.real * rotation.real;
    const float iSquared = rotation.i * rotation.i;
    const float jSquared = rotation.j * rotation.j;
    const float kSquared = rotation.k * rotation.k;
    const float magnitudeSquared = realSquared + iSquared + jSquared + kSquared;

    if (!isfinite(magnitudeSquared) || magnitudeSquared <= 0.0F)
    {
        return false;
    }

    angles.heading = atan2f(
                         2.0F * (rotation.i * rotation.j + rotation.k * rotation.real),
                         iSquared - jSquared - kSquared + realSquared) *
                     RAD_TO_DEG;

    float pitchArgument =
        -2.0F * (rotation.i * rotation.k - rotation.j * rotation.real) / magnitudeSquared;
    pitchArgument = constrain(pitchArgument, -1.0F, 1.0F);
    angles.pitch = asinf(pitchArgument) * RAD_TO_DEG;

    angles.roll = atan2f(
                      2.0F * (rotation.j * rotation.k + rotation.i * rotation.real),
                      -iSquared - jSquared + kSquared + realSquared) *
                  RAD_TO_DEG;

    if (angles.heading < 0.0F)
    {
        angles.heading += 360.0F;
    }

    return isfinite(angles.heading) && isfinite(angles.pitch) && isfinite(angles.roll);
}

struct BnoHealth {
    uint32_t lastFreshOrStartMs = 0;
    uint32_t freshSamples = 0;
    uint32_t staleChecks = 0;
    uint32_t staleEvents = 0;
    uint32_t recoveredStaleEvents = 0;
    uint32_t maxGapMs = 0;
    uint32_t maxTestGapMs = 0;
    bool staleActive = false;
    bool persistentTestFailure = false;

    void observeTime(uint32_t now, bool duringTest)
    {
        const uint32_t gap = now - lastFreshOrStartMs;
        if (gap > maxGapMs) maxGapMs = gap;
        if (duringTest && gap > maxTestGapMs) maxTestGapMs = gap;
        if (gap > BNO_STALE_AFTER_MS && !staleActive)
        {
            staleActive = true;
            ++staleEvents;
        }
        if (duringTest && gap >= BNO_PERSISTENT_AFTER_MS)
        {
            persistentTestFailure = true;
        }
    }

    void recordFresh(uint32_t now, bool duringTest)
    {
        // Inspect the old timestamp first: recovery must not erase an outage
        // that happened while a foreground library call was blocking.
        observeTime(now, duringTest);
        if (staleActive) ++recoveredStaleEvents;
        staleActive = false;
        lastFreshOrStartMs = now;
        ++freshSamples;
    }

    void check(uint32_t now, bool duringTest)
    {
        observeTime(now, duringTest);
        if (staleActive) ++staleChecks;
    }
};


} // namespace milestone4
