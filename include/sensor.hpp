// sensor.hpp — producer thread that simulates one sensor stream.
//
// Each SensorProducer owns one std::thread and publishes a single
// SensorReading per period (1 / publish_hz). The reading value is
//
//     value(t) = true_value + drift_rate * t + N(0, noise_stddev)
//
// so two sensors of the same type and the same {true_value, drift_rate}
// agree on ground truth and differ only in noise — exactly the model
// the inverse-variance fusion math assumes.
//
// SensorProducer is templated on the buffer type, so the same class
// drives the lock-free and locked variants in the A/B benchmark
// without any virtual-call overhead.
//
// Lifecycle:
//   ctor()    -- prepare, do not start
//   start()   -- spawn the thread
//   ... main thread sets *running_ = false ...
//   join()   -- wait for the thread to exit
//
// The producer never retries on a full buffer. A failed push is a
// lost sample, and the buffer's dropped() counter directly reflects
// the sensor's loss rate.

#pragma once

#include "config.hpp"
#include "messages.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <random>
#include <thread>

namespace sfp {

template <typename Buffer>
class SensorProducer {
public:
    SensorProducer(const SensorConfig& cfg,
                   Buffer* buffer,
                   const std::atomic<bool>* running)
        : cfg_(cfg)
        , buffer_(buffer)
        , running_(running)
        // Seed each sensor's RNG deterministically from its id, but spread
        // adjacent ids across the seed space so id 0 and 1 don't yield
        // highly-correlated noise streams. The constant is the integer
        // part of (2^32 / golden ratio) -- a stock hash multiplier.
        , rng_(static_cast<std::uint32_t>(cfg.sensor_id) * 0x9E3779B9u
               + 1u)
        , noise_(0.0, cfg.noise_stddev)
        , period_(period_from_hz(cfg.publish_hz))
    {}

    SensorProducer(const SensorProducer&)            = delete;
    SensorProducer& operator=(const SensorProducer&) = delete;
    SensorProducer(SensorProducer&&)                 = delete;
    SensorProducer& operator=(SensorProducer&&)      = delete;

    void start() {
        thread_ = std::thread(&SensorProducer::run, this);
    }

    void join() {
        if (thread_.joinable()) thread_.join();
    }

    // Stats. Safe from any thread; counters are atomic.
    std::uint64_t samples_published() const noexcept {
        return samples_published_.load(std::memory_order_relaxed);
    }
    std::uint64_t samples_dropped() const noexcept {
        // The buffer counts push attempts that hit a full buffer.
        // Since this producer never retries, that's also the lost-
        // sample count.
        return buffer_->dropped();
    }
    const SensorConfig& config() const noexcept { return cfg_; }

private:
    static Clock::duration period_from_hz(double hz) noexcept {
        if (hz <= 0.0) return Clock::duration::max();
        const auto ns = static_cast<long long>(1.0e9 / hz);
        return std::chrono::nanoseconds(ns);
    }

    void run() {
        const auto start_time = Clock::now();
        auto next_tick = start_time + period_;

        while (running_->load(std::memory_order_acquire)) {
            std::this_thread::sleep_until(next_tick);

            const auto now = Clock::now();

            // Cheap shutdown re-check after the sleep — saves one wasted
            // sample's worth of work if we were asked to stop while we
            // were asleep.
            if (!running_->load(std::memory_order_acquire)) break;

            const double t =
                std::chrono::duration<double>(now - start_time).count();
            const double truth = cfg_.true_value + cfg_.drift_rate * t;
            const double reading = truth + noise_(rng_);

            const SensorReading r{cfg_.sensor_id, cfg_.type, reading, now};
            buffer_->push(r);

            samples_published_.fetch_add(1, std::memory_order_relaxed);

            next_tick += period_;
            // If the OS preempted us long enough that we're already past
            // the next scheduled tick, don't try to catch up by firing a
            // burst of readings. Resynchronize to "now" and continue at
            // the configured rate. This is the standard pattern for
            // real-time loops on a non-RT kernel.
            if (next_tick < now) {
                next_tick = now + period_;
            }
        }
    }

    SensorConfig             cfg_;
    Buffer*                  buffer_;
    const std::atomic<bool>* running_;
    std::thread              thread_;

    std::mt19937                     rng_;
    std::normal_distribution<double> noise_;
    Clock::duration                  period_;

    std::atomic<std::uint64_t> samples_published_{0};
};

} // namespace sfp
