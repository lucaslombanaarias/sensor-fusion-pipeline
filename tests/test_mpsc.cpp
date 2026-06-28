// test_mpsc.cpp — verify the lock-free multi-producer / single-consumer
// ring buffer: FIFO and drop-on-full single-threaded, then correctness
// under many concurrent producers — no loss, no duplication, per-producer
// order preserved. The concurrent case is also what CI exercises under
// ThreadSanitizer, which is where a wrong memory ordering would surface.

#include "ring_buffer.hpp"

#include <atomic>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

namespace {

bool test_fifo_and_drop() {
    sfp::MpscRingBuffer<int, 8> buf;

    for (int i = 0; i < 8; ++i) {
        if (!buf.push(i)) {
            std::cerr << "  FAIL: push " << i << " rejected before full\n";
            return false;
        }
    }
    if (buf.push(999)) {
        std::cerr << "  FAIL: push into a full buffer succeeded\n";
        return false;
    }
    if (buf.dropped() != 1) {
        std::cerr << "  FAIL: dropped()=" << buf.dropped() << ", expected 1\n";
        return false;
    }

    int out = -1;
    for (int i = 0; i < 8; ++i) {
        if (!buf.pop(out) || out != i) {
            std::cerr << "  FAIL: pop got " << out << ", expected " << i << '\n';
            return false;
        }
    }
    if (buf.pop(out)) {
        std::cerr << "  FAIL: pop from an empty buffer succeeded\n";
        return false;
    }
    std::cout << "  PASS: FIFO order, drop-on-full, empty handling.\n";
    return true;
}

bool test_concurrent() {
    constexpr int P = 4;
    constexpr int M = 50000;
    constexpr std::size_t total = static_cast<std::size_t>(P) * M;
    sfp::MpscRingBuffer<int, 1024> buf;

    // Producer p pushes p*M .. p*M+M-1 in order, retrying on full so
    // nothing is ever dropped — that makes "received exactly `total`" an
    // exact assertion.
    std::atomic<bool> go{false};
    std::vector<std::thread> producers;
    for (int p = 0; p < P; ++p) {
        producers.emplace_back([&, p] {
            while (!go.load(std::memory_order_acquire)) { /* spin to start */ }
            for (int k = 0; k < M; ++k) {
                const int v = p * M + k;
                while (!buf.push(v)) std::this_thread::yield();
            }
        });
    }

    std::vector<std::uint8_t> seen(total, 0);
    std::vector<int> last_k(P, -1);
    bool dup = false, order_ok = true;

    std::thread consumer([&] {
        std::size_t got = 0;
        int v = 0;
        while (got < total) {
            if (!buf.pop(v)) { std::this_thread::yield(); continue; }
            ++got;
            if (v < 0 || static_cast<std::size_t>(v) >= total) { dup = true; continue; }
            if (seen[static_cast<std::size_t>(v)]) dup = true;
            else seen[static_cast<std::size_t>(v)] = 1;
            const int p = v / M, k = v % M;     // p in [0,P) since v < total
            if (k <= last_k[p]) order_ok = false;  // per-producer FIFO
            last_k[p] = k;
        }
    });

    go.store(true, std::memory_order_release);
    for (auto& t : producers) t.join();
    consumer.join();

    if (dup) {
        std::cerr << "  FAIL: a value arrived twice or out of range\n";
        return false;
    }
    for (std::size_t i = 0; i < total; ++i) {
        if (!seen[i]) {
            std::cerr << "  FAIL: value " << i << " never received\n";
            return false;
        }
    }
    if (!order_ok) {
        std::cerr << "  FAIL: per-producer FIFO order violated\n";
        return false;
    }

    // dropped() counts transient full-buffer rejections, which the
    // producers simply retried — not lost items. "No loss" is what the
    // exactly-once check above proves; the retry count is informational.
    std::cout << "  PASS: " << P << " producers x " << M << " = " << total
              << " items received exactly once, in per-producer order ("
              << buf.dropped() << " transient full-buffer retries).\n";
    return true;
}

} // namespace

int main() {
    std::cout << "Test 1: FIFO order and drop-on-full (single thread)\n";
    if (!test_fifo_and_drop()) return 1;

    std::cout << "Test 2: concurrent producers — no loss, no dup, ordered\n";
    if (!test_concurrent()) return 1;

    std::cout << "\nAll MPSC ring buffer tests passed.\n";
    return 0;
}
