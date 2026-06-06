// test_ring_buffer.cpp — multithreaded stress tests for SpscRingBuffer
// and LockedRingBuffer.
//
// Three tests, each proves a specific property:
//
//   1. test_no_drop_in_order:
//      Producer yields when full, so no drops. Consumer must receive
//      EVERY value EXACTLY ONCE, IN ORDER. Catches any race that would
//      duplicate, lose, or reorder data on the happy path.
//
//   2. test_drops_preserve_order:
//      Producer never yields, capacity is tiny. Drops are expected.
//      Consumer must receive a strictly-increasing SUBSEQUENCE of pushed
//      values. Catches races that would tear a value mid-write or yield
//      stale data from a previously-dropped slot.
//
//   3. test_locked_buffer_same_interface:
//      Same flow as test 1 but against LockedRingBuffer. Proves the two
//      implementations are drop-in compatible at the API level — the
//      benchmark switches between them via a template parameter.
//
// Tests use uint64_t so we can use UINT64_MAX as a "never seen anything
// yet" sentinel without ambiguity.

#include "ring_buffer.hpp"

#include <atomic>
#include <cstdint>
#include <iostream>
#include <thread>

namespace {

bool test_no_drop_in_order() {
    constexpr std::size_t kItems    = 1'000'000;
    constexpr std::size_t kCapacity = 1024;

    sfp::SpscRingBuffer<std::uint64_t, kCapacity> rb;
    std::atomic<bool> producer_done{false};

    std::thread producer([&] {
        for (std::uint64_t i = 0; i < kItems; ++i) {
            while (!rb.push(i)) {
                std::this_thread::yield();
            }
        }
        producer_done.store(true, std::memory_order_release);
    });

    std::uint64_t expected = 0;
    std::uint64_t value{};
    std::size_t   received = 0;
    while (true) {
        if (rb.pop(value)) {
            if (value != expected) {
                std::cerr << "  FAIL: expected " << expected
                          << " got " << value << " at index " << received
                          << '\n';
                producer.join();
                return false;
            }
            ++expected;
            ++received;
        } else if (producer_done.load(std::memory_order_acquire)) {
            // Producer is done but the buffer may still have items.
            while (rb.pop(value)) {
                if (value != expected) {
                    std::cerr << "  FAIL on drain: expected " << expected
                              << " got " << value << '\n';
                    producer.join();
                    return false;
                }
                ++expected;
                ++received;
            }
            break;
        }
    }
    producer.join();

    if (received != kItems) {
        std::cerr << "  FAIL: received " << received
                  << ", expected " << kItems << '\n';
        return false;
    }
    // Drops here count "push attempts that hit a full buffer" — they
    // don't mean lost data, because the producer retried. They're a
    // measure of how often the consumer fell behind. Just report.
    std::cout << "  PASS: " << kItems << " items round-tripped in order ("
              << rb.dropped() << " transient back-pressure events).\n";
    return true;
}

bool test_drops_preserve_order() {
    constexpr std::size_t kItems    = 100'000;
    constexpr std::size_t kCapacity = 16;

    sfp::SpscRingBuffer<std::uint64_t, kCapacity> rb;
    std::atomic<bool> producer_done{false};

    std::thread producer([&] {
        for (std::uint64_t i = 0; i < kItems; ++i) {
            rb.push(i);  // No retry — drops are expected.
        }
        producer_done.store(true, std::memory_order_release);
    });

    constexpr std::uint64_t kSentinel = UINT64_MAX;
    std::uint64_t last_seen = kSentinel;
    std::uint64_t value{};
    std::size_t   received = 0;

    // Check ordering and accept the value. Returns false on violation.
    auto accept = [&](const char* where) -> bool {
        if (last_seen != kSentinel && value <= last_seen) {
            std::cerr << "  FAIL " << where << ": out-of-order " << value
                      << " after " << last_seen << '\n';
            return false;
        }
        last_seen = value;
        ++received;
        return true;
    };

    while (true) {
        if (rb.pop(value)) {
            if (!accept("main loop")) { producer.join(); return false; }
        } else if (producer_done.load(std::memory_order_acquire)) {
            while (rb.pop(value)) {
                if (!accept("drain")) { producer.join(); return false; }
            }
            break;
        }
    }
    producer.join();

    const auto dropped = rb.dropped();
    if (received + dropped != kItems) {
        std::cerr << "  FAIL: received(" << received << ") + dropped("
                  << dropped << ") != pushed(" << kItems << ")\n";
        return false;
    }
    std::cout << "  PASS: " << received << " received, " << dropped
              << " dropped, strictly monotonic.\n";
    return true;
}

bool test_locked_buffer_same_interface() {
    constexpr std::size_t kItems    = 100'000;
    constexpr std::size_t kCapacity = 64;

    sfp::LockedRingBuffer<std::uint64_t, kCapacity> rb;
    std::atomic<bool> producer_done{false};

    std::thread producer([&] {
        for (std::uint64_t i = 0; i < kItems; ++i) {
            while (!rb.push(i)) {
                std::this_thread::yield();
            }
        }
        producer_done.store(true, std::memory_order_release);
    });

    std::uint64_t expected = 0;
    std::uint64_t value{};
    while (true) {
        if (rb.pop(value)) {
            if (value != expected) {
                std::cerr << "  FAIL: locked buffer ordering broken at "
                          << expected << ", got " << value << '\n';
                producer.join();
                return false;
            }
            ++expected;
        } else if (producer_done.load(std::memory_order_acquire)
                   && rb.size_approx() == 0) {
            break;
        }
    }
    producer.join();

    if (expected != kItems) {
        std::cerr << "  FAIL: received " << expected
                  << ", expected " << kItems << '\n';
        return false;
    }
    std::cout << "  PASS: locked variant API matches, " << expected
              << " items.\n";
    return true;
}

} // namespace

int main() {
    std::cout << "Test 1: no-drop in-order streaming (1M items, cap 1024)\n";
    if (!test_no_drop_in_order()) return 1;

    std::cout << "Test 2: drops preserve ordering (100k items, cap 16)\n";
    if (!test_drops_preserve_order()) return 1;

    std::cout << "Test 3: locked variant interface (100k items, cap 64)\n";
    if (!test_locked_buffer_same_interface()) return 1;

    std::cout << "\nAll ring buffer tests passed.\n";
    return 0;
}
