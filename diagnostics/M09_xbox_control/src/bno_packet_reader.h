#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <sh2_hal.h>
#include <cstring>

extern "C" void m09_bno_io(uint32_t, uint32_t, unsigned, unsigned, int) __attribute__((weak));

namespace milestone4 {
// The pinned BusIO limits ESP32 reads to 128 bytes (and casts lengths to
// uint8_t). Keep BNO packets whole instead: Wire supports size_t lengths and
// a configurable buffer. Only the sensor worker owns this bus/reader.
class BnoPacketReader {
public:
    bool begin(TwoWire *wire, uint8_t address) {
        wire_ = wire;
        address_ = address;
        return wire_ && wire_->setBufferSize(SH2_HAL_MAX_TRANSFER_IN) >= SH2_HAL_MAX_TRANSFER_IN;
    }

    int read(uint8_t *buffer, unsigned capacity, uint32_t *timestamp) {
        uint8_t header[4];
        if (!wire_ || capacity < sizeof(header) || !receive(header, sizeof(header), 1)) return 0;
        *timestamp = micros(); // Preserve polling-HAL header receipt time.
        const uint16_t rawLength = uint16_t(header[0]) | (uint16_t(header[1]) << 8);
        const unsigned length = rawLength & 0x7fffU;
        if (!length) return 0; // Empty transfer, not an orientation sample.
        if (rawLength == 0xffffU || length < sizeof(header) ||
            length > capacity || length > SH2_HAL_MAX_TRANSFER_IN) {
            invalidPacket(length);
            return 0;
        }
        if (length == sizeof(header)) {
            memcpy(buffer, header, sizeof(header));
            return length;
        }
        // STOP after the header peek, then read the repeated header and all
        // payload bytes together. Never split at BusIO's 128-byte boundary.
        if (!receive(buffer, length, 2)) return 0;
        if (memcmp(buffer, header, sizeof(header)) != 0) {
            invalidPacket(length);
            return 0; // Do not hand an inconsistent/truncated packet to SH-2.
        }
        return length;
    }

private:
    TwoWire *wire_ = nullptr;
    uint8_t address_ = 0;
    bool receive(uint8_t *buffer, size_t length, unsigned stage) {
        const uint32_t start = micros();
        // Explicit types select ESP32's non-truncating requestFrom overload.
        bool ok = wire_->requestFrom(uint16_t(address_), length, true) == length;
        if (ok) {
            for (size_t i = 0; i < length; ++i) {
                const int value = wire_->read();
                if (value < 0) { ok = false; break; }
                buffer[i] = uint8_t(value);
            }
        }
        if (m09_bno_io) m09_bno_io(start, uint32_t(micros()-start), stage, length, ok);
        return ok;
    }
    static void invalidPacket(unsigned length) {
        if (m09_bno_io) m09_bno_io(micros(), 0, 5, length, 0);
    }
};
} // namespace milestone4
