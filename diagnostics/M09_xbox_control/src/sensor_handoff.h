#pragma once
#include <atomic>
#include <cstddef>

namespace m09 {
// No waiting, spinning, allocation, I/O or callbacks inside these handoffs.
// Unlike an optimistic seqlock, the mailbox never races on non-atomic payloads.
template<class T> class SnapshotMailbox {
    std::atomic_flag busy_ = ATOMIC_FLAG_INIT;
    T value_{};
public:
    bool publish(const T &value) {
        if (busy_.test_and_set(std::memory_order_acquire)) return false;
        value_ = value;
        busy_.clear(std::memory_order_release);
        return true;
    }
    bool read(T &value) {
        if (busy_.test_and_set(std::memory_order_acquire)) return false;
        value = value_;
        busy_.clear(std::memory_order_release);
        return true;
    }
};

// Exactly one producer (sensor worker) and one consumer (foreground).
template<class T, unsigned Capacity> class SensorQueue {
    T values_[Capacity + 1]{};
    std::atomic<unsigned> write_{0}, read_{0};
public:
    bool push(const T &value) {
        const unsigned write = write_.load(std::memory_order_relaxed);
        const unsigned next = (write + 1) % (Capacity + 1);
        if (next == read_.load(std::memory_order_acquire)) return false;
        values_[write] = value;
        write_.store(next, std::memory_order_release);
        return true;
    }
    bool pop(T &value) {
        const unsigned read = read_.load(std::memory_order_relaxed);
        if (read == write_.load(std::memory_order_acquire)) return false;
        value = values_[read];
        read_.store((read + 1) % (Capacity + 1), std::memory_order_release);
        return true;
    }
};
} // namespace m09
