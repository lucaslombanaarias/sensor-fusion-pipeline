// test_sensor.cpp — verify the producer publishes at the configured
// rate, produces noise with the configured statistics, tracks ground-
// truth drift, and works against both buffer variants.
//
// Each test runs a sensor concurrently with a "collector" thread that
// drains the ring buffer into a vector. This mirrors what the estimator
// will do in the real pipeline — the buffer is a hand-off, not a
// storage area — and means tests don't have to pre-size buffers to
// hold every reading.
//
// All tolerances are deliberately loose to avoid flakiness on a busy
// machine: 20% on rate, ~0.2 units on mean, [0.3, 0.7] on stddev for a
// configured sigma of 0.5. Tight enough to catch real bugs, loose
// enough not to false-fail under scheduler noise.

#include "config.hpp"
#include "messages.hpp"
#include "platform.hpp"
#include "ring_buffer.hpp"
#include "sensor.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

namespace {

constexpr std::size_t kTestBufCapacity = 256;

// Run a sensor for `ms` milliseconds against `Buffer`, with a collector
// thread draining the buffer into a vector. Returns the collected
// readings; writes the buffer's drop count to *dropped_out if provided.
template <typename Buffer>
std::vector<sfp::SensorReading> run_sensor_for(
    const sfp::SensorConfig& cfg,
    int                      ms,
    std::size_t*             dropped_out = nullptr)
{
    Buffer buffer;
    std::atomic<bool> running{true};
    sfp::SensorProducer<Buffer> sensor(cfg, &buffer, &running);

    std::vector<sfp::SensorReading> collected;
    const auto expected = static_cast<std::size_t>(
        cfg.publish_hz * ms / 1000.0 * 1.5);
    collected.reserve(expected);

    std::thread collector([&] {
        sfp::SensorReading r;
        while (running.load(std::memory_order_acquire)
               || buffer.size_approx() > 0) {
            if (!buffer.pop(r)) {
                std::this_thread::yield();
                continue;
            }
            collected.push_back(r);
        }
    });

    sensor.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    running.store(false, std::memory_order_release);
    sensor.join();
    collector.join();

    if (dropped_out) *dropped_out = buffer.dropped();
    return collected;
}

using LockfreeBuf = sfp::SpscRingBuffer<sfp::SensorReading, kTestBufCapacity>;
using LockedBuf   = sfp::LockedRingBuffer<sfp::SensorReading, kTestBufCapacity>;

bool test_sensor_publishes_at_rate_and_noise() {
    const sfp::SensorConfig cfg = sfp::make_sensor(
        /*id=*/0, sfp::SensorType::Temperature,
        /*hz=*/1000.0,
        /*truth=*/25.0, /*drift=*/0.0,
        /*sigma=*/0.5);

    constexpr int kRunMs = 500;
    std::size_t   dropped = 0;
    auto readings = run_sensor_for<LockfreeBuf>(cfg, kRunMs, &dropped);

    const std::size_t count = readings.size();
    if (count == 0) {
        std::cerr << "  FAIL: no readings collected\n";
        return false;
    }

    double sum = 0.0, sum_sq = 0.0;
    for (const auto& r : readings) {
        sum    += r.value;
        sum_sq += r.value * r.value;
    }
    const double mean   = sum / static_cast<double>(count);
    const double var    = (sum_sq / static_cast<double>(count)) - mean * mean;
    const double stddev = std::sqrt(var > 0.0 ? var : 0.0);

    // 1 kHz over 500 ms is ~500 readings where the OS sleep granularity
    // is well under a millisecond (Linux). On Windows the timer floor is
    // ~1 ms even after timeBeginPeriod(1), so a 1 ms-period sleep_until
    // loop tops out around ~500 Hz — half the nominal rate, by the OS,
    // not by any pipeline bug. Use a looser lower bound there. The upper
    // bound is platform-independent (no scheduler lets the loop run
    // *faster* than configured) and still catches a runaway producer.
    const std::size_t expected_count = 500;
#if defined(_WIN32)
    const std::size_t min_count = expected_count * 3 / 10;  // ~300 Hz floor
#else
    const std::size_t min_count = expected_count * 4 / 5;
#endif
    const std::size_t max_count = expected_count * 6 / 5;
    if (count < min_count || count > max_count) {
        std::cerr << "  FAIL: rate off — expected ~" << expected_count
                  << " readings (min " << min_count << "), got " << count
                  << " (dropped " << dropped << ")\n";
        return false;
    }
    if (std::abs(mean - 25.0) > 0.2) {
        std::cerr << "  FAIL: mean " << mean
                  << " too far from truth 25.0\n";
        return false;
    }
    if (stddev < 0.3 || stddev > 0.7) {
        std::cerr << "  FAIL: stddev " << stddev
                  << " outside [0.3, 0.7]\n";
        return false;
    }

    std::cout << "  PASS: " << count << " readings in " << kRunMs
              << " ms, mean=" << mean << " (truth=25.0), stddev="
              << stddev << " (config=0.5), drops=" << dropped << '\n';
    return true;
}

bool test_sensor_tracks_drift() {
    const sfp::SensorConfig cfg = sfp::make_sensor(
        /*id=*/1, sfp::SensorType::Temperature,
        /*hz=*/500.0,
        /*truth=*/20.0, /*drift=*/4.0,
        /*sigma=*/0.1);

    std::size_t dropped = 0;
    auto readings = run_sensor_for<LockfreeBuf>(cfg, 1000, &dropped);

    if (readings.size() < 100) {
        std::cerr << "  FAIL: not enough readings ("
                  << readings.size() << ")\n";
        return false;
    }

    double early = 0.0, late = 0.0;
    for (int i = 0; i < 50; ++i) {
        early += readings[static_cast<std::size_t>(i)].value;
    }
    for (std::size_t i = readings.size() - 50; i < readings.size(); ++i) {
        late += readings[i].value;
    }
    early /= 50.0;
    late  /= 50.0;

    const double dt = std::chrono::duration<double>(
        readings.back().timestamp - readings.front().timestamp).count();
    const double observed_drift = (late - early) / dt;

    if (std::abs(observed_drift - 4.0) > 1.0) {
        std::cerr << "  FAIL: observed drift " << observed_drift
                  << " too far from configured 4.0\n";
        return false;
    }

    std::cout << "  PASS: drift " << observed_drift
              << " units/sec (config 4.0), early=" << early
              << " late=" << late << '\n';
    return true;
}

bool test_sensor_with_locked_buffer() {
    const sfp::SensorConfig cfg = sfp::make_sensor(
        /*id=*/2, sfp::SensorType::Voltage,
        /*hz=*/500.0,
        /*truth=*/12.0, /*drift=*/0.0,
        /*sigma=*/0.05);

    auto readings = run_sensor_for<LockedBuf>(cfg, 300);

    if (readings.size() < 100) {
        std::cerr << "  FAIL: only " << readings.size()
                  << " readings via locked buffer\n";
        return false;
    }
    for (const auto& r : readings) {
        if (r.type != sfp::SensorType::Voltage || r.sensor_id != 2) {
            std::cerr << "  FAIL: reading mis-tagged\n";
            return false;
        }
    }

    std::cout << "  PASS: locked variant produced " << readings.size()
              << " readings, all tagged correctly.\n";
    return true;
}

bool test_two_sensors_independent_noise() {
    const sfp::SensorConfig cfgA = sfp::make_sensor(
        /*id=*/10, sfp::SensorType::Temperature, /*hz=*/1000.0,
        /*truth=*/30.0, /*drift=*/0.0, /*sigma=*/1.0);
    const sfp::SensorConfig cfgB = sfp::make_sensor(
        /*id=*/11, sfp::SensorType::Temperature, /*hz=*/1000.0,
        /*truth=*/30.0, /*drift=*/0.0, /*sigma=*/1.0);

    LockfreeBuf bufA, bufB;
    std::atomic<bool> running{true};
    sfp::SensorProducer<LockfreeBuf> a(cfgA, &bufA, &running);
    sfp::SensorProducer<LockfreeBuf> b(cfgB, &bufB, &running);

    std::vector<double> A, B;
    A.reserve(300);
    B.reserve(300);

    std::thread collA([&] {
        sfp::SensorReading r;
        while (running.load(std::memory_order_acquire)
               || bufA.size_approx() > 0) {
            if (bufA.pop(r)) A.push_back(r.value);
            else std::this_thread::yield();
        }
    });
    std::thread collB([&] {
        sfp::SensorReading r;
        while (running.load(std::memory_order_acquire)
               || bufB.size_approx() > 0) {
            if (bufB.pop(r)) B.push_back(r.value);
            else std::this_thread::yield();
        }
    });

    a.start();
    b.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    running.store(false, std::memory_order_release);
    a.join();
    b.join();
    collA.join();
    collB.join();

    const std::size_t n = std::min(A.size(), B.size());
    if (n < 50) {
        std::cerr << "  FAIL: not enough samples (A=" << A.size()
                  << " B=" << B.size() << ")\n";
        return false;
    }

    double mA = 0.0, mB = 0.0;
    for (std::size_t i = 0; i < n; ++i) { mA += A[i]; mB += B[i]; }
    mA /= static_cast<double>(n);
    mB /= static_cast<double>(n);

    double num = 0.0, dA = 0.0, dB = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double da = A[i] - mA;
        const double db = B[i] - mB;
        num += da * db;
        dA  += da * da;
        dB  += db * db;
    }
    const double corr = num / std::sqrt(dA * dB);

    if (std::abs(corr) > 0.3) {
        std::cerr << "  FAIL: noise streams too correlated, r="
                  << corr << '\n';
        return false;
    }

    std::cout << "  PASS: two sensors, " << n
              << " paired samples, correlation r=" << corr << '\n';
    return true;
}

} // namespace

int main() {
    sfp::ScopedHighResTimer hires_timer;  // 1 ms timer on Windows; no-op on POSIX

    std::cout << "Test 1: rate and noise statistics (1 kHz, 0.5 s, sigma=0.5)\n";
    if (!test_sensor_publishes_at_rate_and_noise()) return 1;

    std::cout << "Test 2: drift tracking (4 units/sec over 1 s)\n";
    if (!test_sensor_tracks_drift()) return 1;

    std::cout << "Test 3: locked buffer compatibility (500 Hz, 0.3 s)\n";
    if (!test_sensor_with_locked_buffer()) return 1;

    std::cout << "Test 4: independent RNG streams across sensors\n";
    if (!test_two_sensors_independent_noise()) return 1;

    std::cout << "\nAll sensor tests passed.\n";
    return 0;
}
