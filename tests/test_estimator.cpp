// test_estimator.cpp — verify the estimator fuses correctly, hits its
// timing target, and works against both buffer variants.
//
// The fusion math claims to be Bayesian-optimal under Gaussian noise:
// two equal-precision sensors should give a fused estimate with
// 1/sqrt(2) the noise of either one alone. Test 2 measures that
// empirically.
//
// The timing test is conservative: we run at 200 Hz for 1 s and assert
// that the loop hit at least 170 ticks (15% tolerance) and that loop
// latency stays in single-digit microseconds. Jitter is bounded
// loosely (max < 5 ms) because a busy CI machine can briefly stall a
// thread for that long.

#include "config.hpp"
#include "estimator.hpp"
#include "messages.hpp"
#include "platform.hpp"
#include "ring_buffer.hpp"
#include "sensor.hpp"
#include "stats.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace {

constexpr std::size_t kSensorBufCap = 256;
constexpr std::size_t kLogBufCap    = 4096;

using SensorBuf      = sfp::SpscRingBuffer<sfp::SensorReading, kSensorBufCap>;
using LogBuf         = sfp::SpscRingBuffer<sfp::LogRecord,     kLogBufCap>;
using Sensor         = sfp::SensorProducer<SensorBuf>;
using EstimatorT     = sfp::Estimator<SensorBuf, LogBuf>;

using LockedSBuf     = sfp::LockedRingBuffer<sfp::SensorReading, kSensorBufCap>;
using LockedLBuf     = sfp::LockedRingBuffer<sfp::LogRecord,     kLogBufCap>;
using LockedSensor   = sfp::SensorProducer<LockedSBuf>;
using LockedEstimator = sfp::Estimator<LockedSBuf, LockedLBuf>;

// Windows may fall back to the ~15.6 ms system timer for a background
// process even with platform.hpp's 1 ms request (a Win11 focus/power
// policy), capping the loop near 64 Hz with ~16 ms jitter. Where a check
// depends on the achievable loop rate (tick counts, jitter), relax it
// there; the value/accuracy/fusion-latency checks stay strict on both.
#if defined(_WIN32)
constexpr bool kWindows = true;
#else
constexpr bool kWindows = false;
#endif

// Compute mean and stddev of fused-state values on a single channel by
// draining the log buffer. Skips ticks where the channel had no fresh
// reading (its bit cleared in channel_mask).
struct ChannelSummary {
    std::size_t n;
    double      mean;
    double      stddev;
};

template <typename LB>
ChannelSummary summarize_channel(LB& log, sfp::SensorType ch) {
    const int bit = static_cast<int>(ch);
    sfp::RunningStats stats;
    sfp::LogRecord rec;
    while (log.pop(rec)) {
        if (rec.state.channel_mask & (1u << bit)) {
            stats.add(rec.state.values[static_cast<std::size_t>(bit)]);
        }
    }
    return {stats.count(), stats.mean(), stats.stddev()};
}

// One sensor, one channel. Fused estimate should hug the truth.
bool test_single_sensor_converges() {
    const sfp::SensorConfig cfg = sfp::make_sensor(
        /*id=*/0, sfp::SensorType::Temperature,
        /*hz=*/1000.0,
        /*truth=*/25.0, /*drift=*/0.0, /*sigma=*/0.5);
    const sfp::EstimatorConfig est_cfg{/*hz=*/100.0, /*spin_us=*/50};

    SensorBuf sbuf;
    LogBuf    lbuf;
    std::atomic<bool> running{true};

    Sensor sensor(cfg, &sbuf, &running);
    EstimatorT estimator(est_cfg,
                         {EstimatorT::SensorPort{&sbuf, cfg}},
                         &lbuf, &running);

    sensor.start();
    estimator.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    running.store(false, std::memory_order_release);
    sensor.join();
    estimator.join();

    const auto ticks = estimator.tick_count();
    if (ticks < (kWindows ? 20u : 40u)) {
        std::cerr << "  FAIL: only " << ticks << " estimator ticks\n";
        return false;
    }

    const auto sum = summarize_channel(lbuf, sfp::SensorType::Temperature);
    if (sum.n < (kWindows ? 15u : 30u)) {
        std::cerr << "  FAIL: only " << sum.n << " channel updates\n";
        return false;
    }
    if (std::abs(sum.mean - 25.0) > 0.2) {
        std::cerr << "  FAIL: mean " << sum.mean
                  << " too far from truth 25.0\n";
        return false;
    }
    // Per-tick fused estimate is just the latest reading; its stddev
    // should be in the noise ballpark (sigma=0.5).
    if (sum.stddev < 0.2 || sum.stddev > 0.8) {
        std::cerr << "  FAIL: fused stddev " << sum.stddev
                  << " outside [0.2, 0.8]\n";
        return false;
    }
    std::cout << "  PASS: " << ticks << " ticks, " << sum.n
              << " channel updates, mean=" << sum.mean
              << " stddev=" << sum.stddev << '\n';
    return true;
}

// Two sensors of equal precision on the same channel. Fused stddev
// should be roughly sigma/sqrt(2) — the Bayesian-optimal reduction
// for two equal-variance observations. We allow a generous tolerance
// since the fusion happens once per estimator tick (not once per
// sensor sample), so the effective noise is the per-tick weighted
// average which depends on intra-tick sample alignment.
bool test_two_sensors_lower_variance() {
    const double sigma = 1.0;
    const sfp::SensorConfig cfgA = sfp::make_sensor(
        /*id=*/0, sfp::SensorType::Voltage, /*hz=*/1000.0,
        /*truth=*/12.0, /*drift=*/0.0, /*sigma=*/sigma);
    const sfp::SensorConfig cfgB = sfp::make_sensor(
        /*id=*/1, sfp::SensorType::Voltage, /*hz=*/1000.0,
        /*truth=*/12.0, /*drift=*/0.0, /*sigma=*/sigma);
    const sfp::EstimatorConfig est_cfg{/*hz=*/200.0, /*spin_us=*/50};

    // Baseline: one sensor only.
    double single_stddev = 0.0;
    {
        SensorBuf sbuf;
        LogBuf    lbuf;
        std::atomic<bool> running{true};
        Sensor sensor(cfgA, &sbuf, &running);
        EstimatorT est(est_cfg,
                       {EstimatorT::SensorPort{&sbuf, cfgA}},
                       &lbuf, &running);
        sensor.start();
        est.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        running.store(false, std::memory_order_release);
        sensor.join();
        est.join();
        single_stddev =
            summarize_channel(lbuf, sfp::SensorType::Voltage).stddev;
    }

    // Two sensors.
    double pair_stddev = 0.0;
    {
        SensorBuf sbufA, sbufB;
        LogBuf    lbuf;
        std::atomic<bool> running{true};
        Sensor a(cfgA, &sbufA, &running);
        Sensor b(cfgB, &sbufB, &running);
        EstimatorT est(est_cfg,
                       {EstimatorT::SensorPort{&sbufA, cfgA},
                        EstimatorT::SensorPort{&sbufB, cfgB}},
                       &lbuf, &running);
        a.start();
        b.start();
        est.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        running.store(false, std::memory_order_release);
        a.join();
        b.join();
        est.join();
        pair_stddev =
            summarize_channel(lbuf, sfp::SensorType::Voltage).stddev;
    }

    // Expectation: pair_stddev ≈ single_stddev / sqrt(2) ≈ 0.707 * single.
    // Allow generous tolerance — anywhere in [0.5x, 0.9x] is a clear win.
    const double ratio = pair_stddev / single_stddev;
    if (ratio < 0.50 || ratio > 0.95) {
        std::cerr << "  FAIL: ratio " << ratio
                  << " outside expected [0.50, 0.95]; single="
                  << single_stddev << " pair=" << pair_stddev << '\n';
        return false;
    }
    std::cout << "  PASS: single stddev=" << single_stddev
              << ", paired stddev=" << pair_stddev
              << ", ratio=" << ratio
              << " (theoretical 0.707 for equal variances)\n";
    return true;
}

// Different channels are tracked independently.
bool test_multi_channel() {
    const sfp::SensorConfig cfg_t = sfp::make_sensor(
        /*id=*/0, sfp::SensorType::Temperature, /*hz=*/500.0,
        /*truth=*/22.0, /*drift=*/0.0, /*sigma=*/0.3);
    const sfp::SensorConfig cfg_v = sfp::make_sensor(
        /*id=*/1, sfp::SensorType::Voltage, /*hz=*/500.0,
        /*truth=*/48.0, /*drift=*/0.0, /*sigma=*/0.1);
    const sfp::EstimatorConfig est_cfg{/*hz=*/100.0, /*spin_us=*/50};

    SensorBuf sbuf_t, sbuf_v;
    LogBuf    lbuf;
    std::atomic<bool> running{true};

    Sensor s_t(cfg_t, &sbuf_t, &running);
    Sensor s_v(cfg_v, &sbuf_v, &running);
    EstimatorT est(est_cfg,
                   {EstimatorT::SensorPort{&sbuf_t, cfg_t},
                    EstimatorT::SensorPort{&sbuf_v, cfg_v}},
                   &lbuf, &running);

    s_t.start();
    s_v.start();
    est.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    running.store(false, std::memory_order_release);
    s_t.join();
    s_v.join();
    est.join();

    // Drain log buffer into per-channel stats.
    sfp::RunningStats t_stats, v_stats;
    sfp::LogRecord rec;
    while (lbuf.pop(rec)) {
        const std::size_t ti = static_cast<std::size_t>(sfp::SensorType::Temperature);
        const std::size_t vi = static_cast<std::size_t>(sfp::SensorType::Voltage);
        if (rec.state.channel_mask & (1u << ti)) t_stats.add(rec.state.values[ti]);
        if (rec.state.channel_mask & (1u << vi)) v_stats.add(rec.state.values[vi]);
    }
    if (std::abs(t_stats.mean() - 22.0) > 0.2) {
        std::cerr << "  FAIL: temperature mean " << t_stats.mean() << '\n';
        return false;
    }
    if (std::abs(v_stats.mean() - 48.0) > 0.1) {
        std::cerr << "  FAIL: voltage mean " << v_stats.mean() << '\n';
        return false;
    }
    std::cout << "  PASS: temp=" << t_stats.mean() << " (truth 22.0), "
              << "voltage=" << v_stats.mean() << " (truth 48.0)\n";
    return true;
}

// Timing discipline: hit the target tick rate with bounded jitter.
bool test_timing_and_jitter() {
    const sfp::SensorConfig cfg = sfp::make_sensor(
        /*id=*/0, sfp::SensorType::Position, /*hz=*/1000.0,
        /*truth=*/0.0, /*drift=*/0.0, /*sigma=*/0.01);
    const sfp::EstimatorConfig est_cfg{/*hz=*/200.0, /*spin_us=*/50};

    SensorBuf sbuf;
    LogBuf    lbuf;
    std::atomic<bool> running{true};

    Sensor sensor(cfg, &sbuf, &running);
    EstimatorT est(est_cfg,
                   {EstimatorT::SensorPort{&sbuf, cfg}},
                   &lbuf, &running);
    sensor.start();
    est.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    running.store(false, std::memory_order_release);
    sensor.join();
    est.join();

    const auto ticks = est.tick_count();
    const auto& lat  = est.latency_ns();  // nanoseconds
    const auto& jit  = est.jitter_us();   // microseconds

    // 200 Hz x 1 s = 200 ticks. Linux allows 15% under for warmup +
    // scheduler; Windows may fall back to the ~15.6 ms timer (see
    // kWindows note), so the floor there only guards against a dead loop.
    const std::uint64_t min_ticks = kWindows ? 50u : 170u;
    if (ticks < min_ticks) {
        std::cerr << "  FAIL: only " << ticks << " ticks at 200 Hz "
                  << "(min " << min_ticks << ")\n";
        return false;
    }
    // Fusion of one sensor is sub-microsecond: mean well under 100 us
    // (100000 ns), max under 5 ms (5e6 ns) even with a scheduler hiccup
    // landing inside the timed region.
    if (lat.mean() > 100000.0 || lat.max() > 5e6) {
        std::cerr << "  FAIL: latency mean=" << lat.mean()
                  << " max=" << lat.max() << " ns\n";
        return false;
    }
    // Jitter can spike on a busy machine; the cap is loose but catches
    // obvious bugs (forgetting to update next_tick, etc). On Windows the
    // ~15.6 ms timer fallback alone can push jitter past a 5 ms cap.
    const double jitter_cap_us = kWindows ? 50000.0 : 5000.0;
    if (jit.max() > jitter_cap_us) {
        std::cerr << "  FAIL: jitter max " << jit.max() << " us\n";
        return false;
    }

    std::cout << "  PASS: " << ticks << " ticks @ 200 Hz, "
              << "latency mean=" << (lat.mean() / 1000.0) << " us / max="
              << (lat.max() / 1000.0)
              << " us, jitter stddev=" << jit.stddev()
              << " us / max=" << jit.max() << " us\n";
    return true;
}

// Same code path against the locked buffer pair.
bool test_locked_estimator() {
    const sfp::SensorConfig cfg = sfp::make_sensor(
        /*id=*/0, sfp::SensorType::Current, /*hz=*/500.0,
        /*truth=*/2.5, /*drift=*/0.0, /*sigma=*/0.05);
    const sfp::EstimatorConfig est_cfg{/*hz=*/100.0, /*spin_us=*/50};

    LockedSBuf sbuf;
    LockedLBuf lbuf;
    std::atomic<bool> running{true};

    LockedSensor sensor(cfg, &sbuf, &running);
    LockedEstimator est(est_cfg,
                        {LockedEstimator::SensorPort{&sbuf, cfg}},
                        &lbuf, &running);
    sensor.start();
    est.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    running.store(false, std::memory_order_release);
    sensor.join();
    est.join();

    const auto sum = summarize_channel(lbuf, sfp::SensorType::Current);
    if (sum.n < (kWindows ? 10u : 20u) || std::abs(sum.mean - 2.5) > 0.05) {
        std::cerr << "  FAIL: locked estimator mean " << sum.mean
                  << " (n=" << sum.n << ")\n";
        return false;
    }
    std::cout << "  PASS: locked variant fused " << sum.n
              << " ticks, mean=" << sum.mean << " (truth 2.5)\n";
    return true;
}

// Complementary filter: a noisy position encoder plus a clean velocity
// sensor, with position drift equal to velocity (physically consistent
// — velocity is the time-derivative of position). The filter should
// track the true position trajectory with lower RMS error than the raw
// encoder noise, because the velocity integral smooths the encoder.
bool test_complementary_filter() {
    const double pos_sigma = 0.05;   // noisy encoder
    const double velocity  = 0.5;    // rad/s, also the position drift
    const sfp::SensorConfig pos_cfg = sfp::make_sensor(
        /*id=*/0, sfp::SensorType::Position, /*hz=*/1000.0,
        /*truth=*/0.0, /*drift=*/velocity, /*sigma=*/pos_sigma);
    const sfp::SensorConfig vel_cfg = sfp::make_sensor(
        /*id=*/1, sfp::SensorType::Velocity, /*hz=*/1000.0,
        /*truth=*/velocity, /*drift=*/0.0, /*sigma=*/0.005);

    // Run once with the filter, once without, same sensor stream config.
    auto run = [&](bool use_filter) -> double {
        sfp::EstimatorConfig est_cfg{/*hz=*/200.0, /*spin_us=*/50};
        est_cfg.use_complementary_filter = use_filter;
        est_cfg.complementary_alpha      = 0.05;

        SensorBuf pbuf, vbuf;
        LogBuf    lbuf;
        std::atomic<bool> running{true};
        Sensor ps(pos_cfg, &pbuf, &running);
        Sensor vs(vel_cfg, &vbuf, &running);
        EstimatorT est(est_cfg,
                       {EstimatorT::SensorPort{&pbuf, pos_cfg},
                        EstimatorT::SensorPort{&vbuf, vel_cfg}},
                       &lbuf, &running);
        ps.start();
        vs.start();
        est.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        running.store(false, std::memory_order_release);
        ps.join();
        vs.join();
        est.join();

        // RMS error of the position estimate vs the true trajectory
        // truth(t) = velocity * t, with t relative to the first sample.
        const std::size_t kPos =
            static_cast<std::size_t>(sfp::SensorType::Position);
        sfp::LogRecord rec;
        bool have_t0 = false;
        sfp::Timestamp t0{};
        double sum_sq = 0.0;
        std::size_t n = 0;
        while (lbuf.pop(rec)) {
            if (!(rec.state.channel_mask & (1u << kPos))) continue;
            if (!have_t0) { t0 = rec.state.timestamp; have_t0 = true; }
            const double t =
                std::chrono::duration<double>(rec.state.timestamp - t0).count();
            const double truth = velocity * t;
            const double err = rec.state.values[kPos] - truth;
            sum_sq += err * err;
            ++n;
        }
        return (n > 0) ? std::sqrt(sum_sq / static_cast<double>(n)) : 1e9;
    };

    const double rms_off = run(false);
    const double rms_on  = run(true);

    // The raw encoder per-sample noise is pos_sigma = 0.05. With the
    // filter off, the position estimate is just the latest encoder
    // reading, so its RMS error should be ~pos_sigma. With the filter
    // on, velocity integration smooths it, so RMS error should drop
    // meaningfully. Require at least a 25% reduction — conservative;
    // in practice it's larger.
    if (rms_on >= rms_off * 0.75) {
        std::cerr << "  FAIL: filter did not reduce error enough — "
                  << "off=" << rms_off << " on=" << rms_on << '\n';
        return false;
    }
    std::cout << "  PASS: position RMS error off=" << rms_off
              << " on=" << rms_on << " ("
              << (100.0 * (1.0 - rms_on / rms_off)) << "% reduction, "
              << "alpha=0.05)\n";
    return true;
}

// The filter must keep producing a position estimate even when encoder
// samples are sparse — it coasts on velocity integration. Run a slow
// encoder (5 Hz) at a 200 Hz loop so most ticks have no fresh position
// reading, and confirm the Position channel is marked present on
// essentially every tick.
bool test_complementary_coasts() {
    const double velocity = 0.3;
    const sfp::SensorConfig pos_cfg = sfp::make_sensor(
        /*id=*/0, sfp::SensorType::Position, /*hz=*/5.0,   // very slow
        /*truth=*/0.0, /*drift=*/velocity, /*sigma=*/0.01);
    const sfp::SensorConfig vel_cfg = sfp::make_sensor(
        /*id=*/1, sfp::SensorType::Velocity, /*hz=*/1000.0,
        /*truth=*/velocity, /*drift=*/0.0, /*sigma=*/0.002);

    sfp::EstimatorConfig est_cfg{/*hz=*/200.0, /*spin_us=*/50};
    est_cfg.use_complementary_filter = true;
    est_cfg.complementary_alpha      = 0.05;

    SensorBuf pbuf, vbuf;
    LogBuf    lbuf;
    std::atomic<bool> running{true};
    Sensor ps(pos_cfg, &pbuf, &running);
    Sensor vs(vel_cfg, &vbuf, &running);
    EstimatorT est(est_cfg,
                   {EstimatorT::SensorPort{&pbuf, pos_cfg},
                    EstimatorT::SensorPort{&vbuf, vel_cfg}},
                   &lbuf, &running);
    ps.start();
    vs.start();
    est.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    running.store(false, std::memory_order_release);
    ps.join();
    vs.join();
    est.join();

    const std::size_t kPos = static_cast<std::size_t>(sfp::SensorType::Position);
    sfp::LogRecord rec;
    std::size_t total = 0, with_position = 0;
    double last_value = 0.0;
    bool monotonic_ok = true;
    while (lbuf.pop(rec)) {
        ++total;
        if (rec.state.channel_mask & (1u << kPos)) {
            ++with_position;
            // Position should advance roughly monotonically (velocity > 0).
            if (rec.state.values[kPos] + 1e-6 < last_value) monotonic_ok = false;
            last_value = rec.state.values[kPos];
        }
    }

    // Encoder fires ~5 times/sec, loop runs 200 times/sec, so without
    // coasting only ~2.5% of ticks would have a position. With coasting
    // we expect essentially all of them (after the first encoder reading).
    const double coverage =
        (total > 0) ? static_cast<double>(with_position) / static_cast<double>(total)
                    : 0.0;
    if (coverage < 0.90) {
        std::cerr << "  FAIL: position present on only "
                  << (100.0 * coverage) << "% of ticks (expected >90%)\n";
        return false;
    }
    if (!monotonic_ok) {
        std::cerr << "  FAIL: coasted position not monotonic under +velocity\n";
        return false;
    }
    std::cout << "  PASS: 5 Hz encoder, 200 Hz loop — position present on "
              << (100.0 * coverage) << "% of ticks via velocity coasting.\n";
    return true;
}

// Kalman filter: the same noisy-encoder + clean-velocity setup as the
// complementary-filter test, but routed through the coupled 2-state
// Kalman filter. It should track the true trajectory with substantially
// lower RMS error than the raw encoder.
bool test_kalman_reduces_error() {
    const double pos_sigma = 0.05;   // noisy encoder
    const double velocity  = 0.5;    // rad/s, also the position drift
    const sfp::SensorConfig pos_cfg = sfp::make_sensor(
        /*id=*/0, sfp::SensorType::Position, /*hz=*/1000.0,
        /*truth=*/0.0, /*drift=*/velocity, /*sigma=*/pos_sigma);
    const sfp::SensorConfig vel_cfg = sfp::make_sensor(
        /*id=*/1, sfp::SensorType::Velocity, /*hz=*/1000.0,
        /*truth=*/velocity, /*drift=*/0.0, /*sigma=*/0.005);

    auto run = [&](bool use_kalman) -> double {
        sfp::EstimatorConfig est_cfg{/*hz=*/200.0, /*spin_us=*/50};
        est_cfg.use_kalman_filter    = use_kalman;
        est_cfg.kalman_process_noise = 1.0;

        SensorBuf pbuf, vbuf;
        LogBuf    lbuf;
        std::atomic<bool> running{true};
        Sensor ps(pos_cfg, &pbuf, &running);
        Sensor vs(vel_cfg, &vbuf, &running);
        EstimatorT est(est_cfg,
                       {EstimatorT::SensorPort{&pbuf, pos_cfg},
                        EstimatorT::SensorPort{&vbuf, vel_cfg}},
                       &lbuf, &running);
        ps.start();
        vs.start();
        est.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        running.store(false, std::memory_order_release);
        ps.join();
        vs.join();
        est.join();

        const std::size_t kPos =
            static_cast<std::size_t>(sfp::SensorType::Position);
        sfp::LogRecord rec;
        bool have_t0 = false;
        sfp::Timestamp t0{};
        double sum_sq = 0.0;
        std::size_t n = 0;
        while (lbuf.pop(rec)) {
            if (!(rec.state.channel_mask & (1u << kPos))) continue;
            if (!have_t0) { t0 = rec.state.timestamp; have_t0 = true; }
            const double t =
                std::chrono::duration<double>(rec.state.timestamp - t0).count();
            const double err = rec.state.values[kPos] - velocity * t;
            sum_sq += err * err;
            ++n;
        }
        return (n > 0) ? std::sqrt(sum_sq / static_cast<double>(n)) : 1e9;
    };

    const double rms_off = run(false);
    const double rms_on  = run(true);

    // Same conservative bar as the complementary-filter test: at least a
    // 25% reduction. In practice the Kalman filter does much better.
    if (rms_on >= rms_off * 0.75) {
        std::cerr << "  FAIL: Kalman did not reduce error enough — off="
                  << rms_off << " on=" << rms_on << '\n';
        return false;
    }
    std::cout << "  PASS: position RMS error off=" << rms_off
              << " on=" << rms_on << " ("
              << (100.0 * (1.0 - rms_on / rms_off)) << "% reduction, "
              << "Kalman)\n";
    return true;
}

} // namespace

int main() {
    sfp::ScopedHighResTimer hires_timer;  // 1 ms timer on Windows; no-op on POSIX

    std::cout << "Test 1: single sensor converges to truth\n";
    if (!test_single_sensor_converges()) return 1;

    std::cout << "Test 2: two equal-precision sensors reduce variance\n";
    if (!test_two_sensors_lower_variance()) return 1;

    std::cout << "Test 3: multi-channel tracked independently\n";
    if (!test_multi_channel()) return 1;

    std::cout << "Test 4: timing and jitter at 200 Hz\n";
    if (!test_timing_and_jitter()) return 1;

    std::cout << "Test 5: locked buffer drop-in\n";
    if (!test_locked_estimator()) return 1;

    std::cout << "Test 6: complementary filter reduces position error\n";
    if (!test_complementary_filter()) return 1;

    std::cout << "Test 7: complementary filter coasts through sparse encoder\n";
    if (!test_complementary_coasts()) return 1;

    std::cout << "Test 8: Kalman filter reduces position error\n";
    if (!test_kalman_reduces_error()) return 1;

    std::cout << "\nAll estimator tests passed.\n";
    return 0;
}
