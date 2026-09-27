// Actual two-thread stress of the exact primitives used on the ESP32.
#include "../src/sensor_handoff.h"
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <thread>

struct Packet { uint32_t sequence=0, words[24]{}; };
int main() {
    m09::SensorQueue<Packet, 4> queue;
    m09::SnapshotMailbox<Packet> mailbox;
    constexpr uint32_t count=100000;
    std::atomic<bool> done{false};
    std::thread producer([&] {
        for (uint32_t n=1; n<=count; ++n) {
            Packet p; p.sequence=n;
            for (auto &word:p.words) word=n^0xa55a1234U;
            while (!queue.push(p)) std::this_thread::yield();
            mailbox.publish(p);
        }
        done.store(true,std::memory_order_release);
    });
    uint32_t expected=1;
    while (expected<=count) {
        Packet p;
        if (queue.pop(p)) {
            assert(p.sequence==expected++);
            for (auto word:p.words) assert(word==(p.sequence^0xa55a1234U));
        }
        if (mailbox.read(p) && p.sequence)
            for (auto word:p.words) assert(word==(p.sequence^0xa55a1234U));
    }
    producer.join(); assert(done.load());
    Packet p; assert(!queue.pop(p));
    for (unsigned i=0;i<4;++i) assert(queue.push(p));
    assert(!queue.push(p));
    puts("PASS sensor handoff: 100000 ordered records, coherent concurrent snapshots, bounded overflow");
}
