#include "../src/sensor_support.h"
#include <cassert>
#include <cstdio>

unsigned failures = 0, lastStage = 0;
extern "C" void m09_bno_io(uint32_t, uint32_t, unsigned stage, unsigned, int ok) {
    if (!ok) { ++failures; lastStage = stage; }
}
void packet(TwoWire &wire, unsigned length, bool continuation = false) {
    wire.packet = TwoWire::PacketFixture{};
    wire.packet.data.resize(length > 4 ? length : 4);
    for (unsigned i = 0; i < wire.packet.data.size(); ++i) wire.packet.data[i] = uint8_t(i);
    wire.packet.data[0] = length & 255;
    wire.packet.data[1] = (length >> 8) | (continuation ? 0x80 : 0);
    wire.packet.data[2] = 2;
    wire.packet.data[3] = 42;
}
class ConnectedBno : public milestone4::DiagnosticBno085 {
public:
    int read(uint8_t *data, uint32_t *timestamp) {
        assert(_HAL.read);
        return _HAL.read(&_HAL, data, 384, timestamp);
    }
};
int main() {
    milestone4::BnoPacketReader reader;
    uint8_t data[386]; uint32_t timestamp = 0;
    assert(!reader.begin(nullptr, 0x4A));
    Wire1.bufferFails = true;
    assert(!reader.begin(&Wire1, 0x4A));
    Wire1.bufferFails = false;
    for (uint8_t address : {0x4A, 0x4B}) {
        Wire1.bnoAddress = address;
        assert(reader.begin(&Wire1, address));
        assert(Wire1.bufferSize == 384 && Wire.bufferSize == 128);
        for (unsigned length : {5U, 20U, 128U, 152U, 256U, 384U}) {
            for (bool continuation : {false, true}) {
                packet(Wire1, length, continuation);
                memset(data, 0xa5, sizeof(data));
                const uint32_t start = micros();
                assert(reader.read(data+1, 384, &timestamp) == int(length));
                assert(timestamp == start+1000); // header receipt, not payload completion
                assert((Wire1.packet.requests == std::vector<size_t>{4, length}));
                assert(memcmp(data+1, Wire1.packet.data.data(), length) == 0);
                assert(data[0] == 0xa5 && data[length+1] == 0xa5);
            }
        }
    }
    for (unsigned length : {0U, 1U, 3U, 385U, 65535U}) {
        packet(Wire1, length);
        assert(reader.read(data, 384, &timestamp) == 0);
        assert(Wire1.packet.requests.size() == 1);
    }
    packet(Wire1, 4);
    assert(reader.read(data, 384, &timestamp) == 4);
    assert(Wire1.packet.requests.size() == 1);
    packet(Wire1, 152);
    assert(reader.read(data, 128, &timestamp) == 0);
    assert(Wire1.packet.requests.size() == 1 && lastStage == 5);
    for (unsigned failAt : {1U, 2U}) {
        packet(Wire1, 152); Wire1.packet.failRequest = failAt;
        timestamp = 99;
        assert(reader.read(data, 384, &timestamp) == 0);
        assert(lastStage == failAt && Wire1.packet.requests.size() == failAt);
        if (failAt == 1) assert(timestamp == 99);
    }
    packet(Wire1, 152); Wire1.packet.data.resize(151);
    assert(reader.read(data, 384, &timestamp) == 0 && lastStage == 2);
    packet(Wire1, 152); Wire1.packet.failByte = 15;
    assert(reader.read(data, 384, &timestamp) == 0 && lastStage == 2);
    packet(Wire1, 152); Wire1.packet.changedHeader = true;
    assert(reader.read(data, 384, &timestamp) == 0 && lastStage == 5);
    assert(failures >= 10);
    // Verify the firmware subclass installs this reader before SH-2 startup.
    ConnectedBno bno;
    Wire1.bufferFails = true;
    const unsigned begins = simulated::bnoBeginCalls;
    assert(!bno.begin_I2C(0x4B, &Wire1) && simulated::bnoBeginCalls == begins);
    Wire1.bufferFails = false;
    assert(bno.begin_I2C(0x4B, &Wire1));
    packet(Wire1, 152);
    assert(bno.read(data, &timestamp) == 152);
    assert((Wire1.packet.requests == std::vector<size_t>{4, 152}));
    puts("BNO complete-packet transport: PASS");
}
