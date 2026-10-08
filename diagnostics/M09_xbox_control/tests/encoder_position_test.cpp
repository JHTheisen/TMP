#include "../src/encoder_position.h"
#include "../src/control_math.h"
#include <cassert>
#include <cstdio>
using namespace m09;
EncoderState sample(uint16_t raw, uint32_t at, uint32_t reads) {
    EncoderState s;
    s.valid = s.hasSample = true; s.status = 0x20;
    s.raw = raw; s.lastGoodMs = at; s.reads = reads; return s;
}
int main() {
    assert(sample(0, 0, 1).angleDegrees() == 0);
    assert(sample(1024, 0, 1).angleDegrees() == 90);
    assert(sample(4095, 0, 1).angleDegrees() == 359.912109375);
    EncoderPosition position; EncoderReference reference;
    position.observe(sample(4090, 10, 1), 10);
    assert(!reference.setZero(position, 10)); // No assumed mounting.
    assert(!reference.configure(0) && !reference.configure(NAN));
    assert(reference.configure(-36)); // Explicit synthetic 10:1 reversed shaft.
    assert(reference.setZero(position, 10));
    position.observe(sample(10, 30, 2), 30);
    assert(position.ticks == 4106);
    assert(fabs(reference.angle(position, 30) + 16.0 * 36 / 4096) < 1e-10);
    position.observe(sample(4090, 50, 3), 50);
    assert(reference.angle(position, 50) == 0);
    for (unsigned i = 1; i <= 12; ++i)
        position.observe(sample((4090 + i * 1024) % 4096, 50 + 20*i, 3+i), 50+20*i);
    assert(position.ticks == 4090 + 3*4096);
    assert(reference.angle(position, 290) == -108);
    assert(milestone7::wrap360(reference.angle(position, 290)) == 252);
    assert(reference.setZero(position, 290));
    assert(reference.angle(position, 290) == 0); // Repeat calibration.
    auto bad = sample(4090, 300, 16); bad.status = 0x30;
    const auto epoch = position.epoch;
    position.observe(bad, 300);
    position.observe(sample(4090, 310, 17), 310);
    assert(position.epoch != epoch); // Invalidity survives an unconsumed snapshot.
    reference.observe(position, 310);
    assert(!reference.calibrated && std::isnan(reference.angle(position, 310)));
    assert(reference.setZero(position, 310));
    reference.observe(position, 460);
    assert(!reference.calibrated); // Cached data never becomes fresh.
    position.observe(sample(100, 470, 18), 470);
    assert(reference.setZero(position, 470));
    position.observe(sample(2148, 490, 19), 490); // Ambiguous exact half revolution.
    assert(!position.valid);
    position = {};
    position.observe(sample(4090, UINT32_MAX - 9, 1), UINT32_MAX - 9);
    position.observe(sample(10, 10, 2), 10);
    assert(position.ticks == 4106 && position.fresh(11));
    std::puts("PASS encoder conversion, direction, wrapping, revolutions, references and recovery");
}
