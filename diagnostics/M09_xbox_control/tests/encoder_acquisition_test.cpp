#include <cassert>
#include <cmath>
#include <iostream>
#include "../src/encoder_acquisition.h"

namespace {
struct Fixture {
    TwoWire a{0}, b{1};
    m09::EncoderAcquisition encoders{a, b};
    Fixture(bool busA = true, bool busB = true, uint32_t now = 0) {
        simulated::now = now;
        a.encoder.present = b.encoder.present = true;
        a.encoder.raw = 1024;
        b.encoder.raw = 4095;
        encoders.begin(busA, busB, now);
    }
    bool service(uint32_t now) {
        simulated::now = now;
        return encoders.service(now);
    }
};

void bothDevicesAndCadence() {
    Fixture f;
    assert(!f.encoders.state(0).hasSample);
    assert(f.encoders.state(0).ageMs(0) == UINT32_MAX);
    assert(f.service(0));
    assert(f.encoders.state(0).raw == 1024);
    assert(f.encoders.state(0).angleDegrees() == 90.0);
    assert(f.encoders.state(0).valid && f.encoders.state(0).available);
    assert(f.encoders.state(0).magnetGood());
    assert(f.b.encoder.requests == 0);
    assert(!f.service(0) && !f.service(9));
    assert(f.service(10));
    assert(f.encoders.state(1).raw == 4095);
    assert(f.encoders.state(0).ageMs(10) == 10);
    assert(f.encoders.state(1).ageMs(10) == 0);
    assert(!f.service(19));
    assert(f.service(20));
    assert(f.encoders.state(0).reads == 2 && f.encoders.state(1).reads == 1);
    assert(f.a.encoder.lastRegister == 0x0B && f.b.encoder.lastRegister == 0x0B);
    assert(f.a.encoder.requestedBytes == 3 && f.b.encoder.requestedBytes == 3);
    assert(f.a.encoder.writes == f.a.encoder.requests);
    assert(f.service(1000));
    assert(!f.service(1000)); // No catch-up after an arbitrarily late loop.
    assert(f.encoders.state(1).reads == 2);
}

void missingAndRecovery() {
    Fixture f;
    f.a.encoder.present = false;
    assert(f.service(0));
    assert(!f.encoders.state(0).available && !f.encoders.state(0).valid);
    assert(!f.encoders.state(0).hasSample && f.encoders.state(0).failures == 1);
    assert(f.a.encoder.requests == 0);
    assert(f.service(10) && f.encoders.state(1).valid);
    f.a.encoder.present = true;
    assert(f.service(20) && f.encoders.state(0).valid);
    f.b.encoder.present = false;
    assert(f.service(30));
    assert(!f.encoders.state(1).valid && f.encoders.state(1).hasSample);
    assert(f.encoders.state(1).raw == 4095 && f.encoders.state(1).ageMs(30) == 20);
    assert(f.encoders.state(0).valid);
}

void failedBusDoesNotBlockOtherBus() {
    Fixture f(false, true);
    assert(!f.service(0));
    assert(!f.encoders.state(0).busAvailable && !f.encoders.state(0).hasSample);
    assert(f.a.encoder.addressTransfers == 0 && f.a.encoder.writes == 0);
    assert(f.service(10) && f.encoders.state(1).valid);
    Fixture g(true, false);
    assert(g.service(0) && g.encoders.state(0).valid);
    assert(!g.service(10) && g.b.encoder.addressTransfers == 0);
}

void incompleteOrInvalidData() {
    for (int fault = 0; fault < 8; ++fault) {
        Fixture f;
        assert(f.service(0));
        if (fault == 0) f.a.encoder.requestLength = 0;
        if (fault == 1) f.a.encoder.requestLength = 2;
        if (fault == 2) f.a.encoder.failedReadIndex = 0;
        if (fault == 3) f.a.encoder.failedReadIndex = 1;
        if (fault == 4) f.a.encoder.failedReadIndex = 2;
        if (fault == 5) f.a.encoder.highOverride = 0x10;
        if (fault == 6) f.a.encoder.addressResult = 4;
        if (fault == 7) f.a.encoder.writeAccepted = false;
        assert(f.service(10) && f.encoders.state(1).valid);
        assert(f.service(20));
        const auto &sample = f.encoders.state(0);
        assert(!sample.valid && sample.hasSample && sample.failures == 1);
        assert(sample.raw == 1024 && sample.reads == 1 && sample.ageMs(20) == 20);
        assert(sample.available == (fault < 6));
        assert(f.encoders.state(1).valid);
    }
}

void magnetFlagsAreSeparateFromReadValidity() {
    for (uint8_t status : {0x00, 0x20, 0x30, 0x28, 0x38}) {
        Fixture f;
        f.a.encoder.status = status;
        assert(f.service(0));
        const auto &sample = f.encoders.state(0);
        assert(sample.valid && sample.status == status && sample.failures == 0);
        assert(sample.magnetGood() == (status == 0x20));
    }
}

void timestampsAndCadenceSurviveRolloverAndSlowRead() {
    Fixture f(true, true, UINT32_MAX - 5);
    assert(f.service(UINT32_MAX - 5));
    assert(!f.service(3));
    assert(f.service(4));
    assert(f.encoders.state(0).ageMs(4) == 10);
    assert(f.service(14));
    assert(f.encoders.state(0).ageMs(14) == 0);

    Fixture g;
    g.a.encoder.addressDelayMs = 50;
    g.a.encoder.requestDelayMs = 50;
    assert(g.service(0));
    assert(simulated::now == 100 && g.encoders.state(0).lastGoodMs == 100);
    assert(g.encoders.state(0).ageMs(100) == 0);
    assert(g.b.encoder.requests == 0); // Never read both buses in one call.
    assert(g.service(100));
    assert(!g.service(100));
}
} // namespace

int main() {
    bothDevicesAndCadence();
    missingAndRecovery();
    failedBusDoesNotBlockOtherBus();
    incompleteOrInvalidData();
    magnetFlagsAreSeparateFromReadValidity();
    timestampsAndCadenceSurviveRolloverAndSlowRead();
    std::cout << "Encoder acquisition checks passed\n";
}
