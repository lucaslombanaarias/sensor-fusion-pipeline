// ring_buffer.hpp — SPSC ring buffers for the sensor-fusion pipeline.
//
// Two implementations with the same interface:
//
//   SpscRingBuffer<T, N>     — lock-free, single-producer/single-consumer.
//                               Two atomic counters, acquire/release pairing,
//                               cache-line padding to prevent false sharing.
//
//   LockedRingBuffer<T, N>   — same interface, std::mutex backed.
//                               Used to A/B against the lock-free version
//                               in the benchmark.
//
// Drop policy for both: when the buffer is full, push() returns false and
// increments a dropped counter. The newest sample is discarded; the oldest
// is kept. This preserves SPSC purity (only the producer writes head_,
// only the consumer writes tail_) and gives us a clean diagnostic of when
// the consumer can't keep up.
//
// Capacity must be a power of two for the lock-free variant so wrap-around
// is a bitmask rather than a modulo. The locked variant doesn't require it.

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <mutex>

namespace sfp {

template <typename T, std::size_t Capacity>
class SpscRingBuffer {
    static_assert(Capacity > 0, "Capacity must be > 0");
    static_assert((Capacity & (Capacity - 1)) == 0,
                  "Capacity must be a power of 2");

public:
    SpscRingBuffer() = default;

    SpscRingBuffer(const SpscRingBuffer&)            = delete;
    SpscRingBuffer& operator=(const SpscRingBuffer&) = delete;
    SpscRingBuffer(SpscRingBuffer&&)                 = delete;
    SpscRingBuffer& operator=(SpscRingBuffer&&)      = delete;

    // Producer-side. Returns false if the buffer is full; in that case
    // the item is dropped and dropped_count is incremented.
    //
    // Memory ordering:
    //   - head_ load is relaxed because only this thread writes it.
    //   - tail_ load is acquire to synchronize with the consumer's
    //     release-store of tail_ when it freed a slot.
    //   - The buffer write happens between the two atomics; it is
    //     ordinary (non-atomic) but ordered by the surrounding atomics.
    //   - head_ store is release so the consumer's acquire-load of head_
    //     also sees this buffer write.
    bool push(const T& item) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        if (head - tail >= Capacity) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        buffer_[head & kMask] = item;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    // Consumer-side. Returns false if the buffer is empty.
    //
    // Mirror of push(): relaxed on the index this thread owns (tail_),
    // acquire on the index the other thread publishes (head_), and a
    // release store of tail_ when we're done so the producer's
    // acquire-load sees the freed slot.
    bool pop(T& out) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t head = head_.load(std::memory_order_acquire);
        if (head == tail) {
            return false;
        }
        out = buffer_[tail & kMask];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    // Approximate item count. Stale by the time the caller reads it
    // since both threads may have moved on. Useful for diagnostics
    // and for drain loops at shutdown.
    std::size_t size_approx() const noexcept {
        const std::size_t head = head_.load(std::memory_order_acquire);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        return head - tail;
    }

    // Total push() calls rejected because the buffer was full.
    std::size_t dropped() const noexcept {
        return dropped_.load(std::memory_order_relaxed);
    }

    static constexpr std::size_t capacity() noexcept { return Capacity; }

private:
    // Wrap-around mask. Power-of-2 capacity makes this an AND instead
    // of a modulo divide — typically 3-5x faster on the hot path.
    static constexpr std::size_t kMask = Capacity - 1;

    // Common cache line size on x86_64 and ARM64. C++17 provides
    // std::hardware_destructive_interference_size for this, but it's
    // unreliable across toolchains (libstdc++ warns when used, libc++
    // doesn't always define it). 64 is the right answer for every
    // mainstream CPU this project targets.
    static constexpr std::size_t kCacheLine = 64;

    // Three atomics, each on its own cache line. Without the padding,
    // head_ and tail_ would share a cache line, and the producer's
    // store to head_ would invalidate the consumer's L1 copy of the
    // line (and vice versa) on every operation — "false sharing".
    // The padding turns a hot cache-line ping-pong into independent
    // local accesses. Measurably worth the few bytes.
    alignas(kCacheLine) std::atomic<std::size_t> head_{0};      // producer-write
    alignas(kCacheLine) std::atomic<std::size_t> tail_{0};      // consumer-write
    alignas(kCacheLine) std::atomic<std::size_t> dropped_{0};   // producer-write, stats only

    // The data itself. Default-constructed; T must be default-
    // constructible. Aligned to its own line to avoid sharing with
    // the dropped_ counter.
    alignas(kCacheLine) std::array<T, Capacity> buffer_{};
};

// Locked baseline. Same interface, mutex-protected. Strictly worse
// than the SPSC version under contention, but it's the apples-to-
// apples comparison the benchmark needs.
template <typename T, std::size_t Capacity>
class LockedRingBuffer {
    static_assert(Capacity > 0, "Capacity must be > 0");

public:
    LockedRingBuffer() = default;

    LockedRingBuffer(const LockedRingBuffer&)            = delete;
    LockedRingBuffer& operator=(const LockedRingBuffer&) = delete;

    bool push(const T& item) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (size_ == Capacity) {
            ++dropped_;
            return false;
        }
        buffer_[head_] = item;
        head_ = (head_ + 1) % Capacity;
        ++size_;
        return true;
    }

    bool pop(T& out) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (size_ == 0) {
            return false;
        }
        out = buffer_[tail_];
        tail_ = (tail_ + 1) % Capacity;
        --size_;
        return true;
    }

    std::size_t size_approx() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return size_;
    }

    std::size_t dropped() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return dropped_;
    }

    static constexpr std::size_t capacity() noexcept { return Capacity; }

private:
    mutable std::mutex mtx_;
    std::array<T, Capacity> buffer_{};
    std::size_t head_{0};
    std::size_t tail_{0};
    std::size_t size_{0};
    std::size_t dropped_{0};
};

} // namespace sfp
