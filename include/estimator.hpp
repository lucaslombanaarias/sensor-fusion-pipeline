// estimator.hpp — fixed-rate consumer thread that drains all sensor
// buffers, fuses readings into a state estimate, and (optionally)
// pushes a LogRecord to a log buffer for the logger thread.
//
// The Estimator class is templated on the SensorBuffer and LogBuffer
// types so the A/B benchmark can swap lock-free for locked variants
// without touching this file. Both buffer types only need push/pop/
// dropped/size_approx — duck-typed, no inheritance.
//
// Timing model per tick:
//
//   1. sleep_until(next_tick - spin_window)        ← coarse wait
//   2. while (now() < next_tick) { /* spin */ }    ← fine wait
//   3. wake_time = now();   jitter = wake_time - next_tick
//   4. fuse_tick();          latency = end - work_start
//   5. log_buffer->push(LogRecord{state, latency, jitter})
//   6. next_tick += period
//
// Jitter is "how late did we wake compared to the deadline" — always
// >= 0 on a non-RT kernel; the spin window pulls it down to single-
// digit microseconds when configured. Latency is "how long did the
// fusion work take" — this is the bare CPU cost of the fusion step,
// not the end-to-end loop time.
//
// Fusion algorithm (this pass): inverse-variance weighted average per
// channel. For each tick, drain every sensor buffer keeping only the
// most-recent reading per sensor, then for each channel that received
// any reading compute
//
//     fused[c] = sum(w_i * z_i) / sum(w_i)    for sensors i with type c
//
// Channels with no new reading this tick keep their previous value
// and have their bit cleared in channel_mask. Per-channel exponential
// smoothing and the position-velocity complementary filter for the
// robotics demo land in a later pass on top of this.

#pragma once

#include "config.hpp"
#include "kalman.hpp"
#include "messages.hpp"
#include "stats.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <utility>
#include <vector>

namespace sfp {

template <typename SensorBuffer, typename LogBuffer>
class Estimator {
public:
    struct SensorPort {
        SensorBuffer* buffer;
        SensorConfig  config;
    };

    Estimator(const EstimatorConfig&   cfg,
              std::vector<SensorPort>  sensors,
              LogBuffer*               log_buffer,   // may be nullptr
              const std::atomic<bool>* running)
        : cfg_(cfg)
        , sensors_(std::move(sensors))
        , log_buffer_(log_buffer)
        , running_(running)
        , period_(period_from_hz(cfg.loop_hz))
        , latest_per_sensor_(sensors_.size())
        , samples_consumed_per_sensor_(sensors_.size(), 0)
        , kf_(cfg.kalman_process_noise)
    {
        last_state_.tick         = 0;
        last_state_.channel_mask = 0;
        last_state_.values.fill(0.0);

        // The Kalman filter only makes sense if there is a Position or
        // Velocity channel to estimate; otherwise leave it disengaged
        // regardless of the config flag.
        for (const auto& port : sensors_) {
            if (port.config.type == SensorType::Position ||
                port.config.type == SensorType::Velocity) {
                has_pos_vel_ = true;
                break;
            }
        }
    }

    Estimator(const Estimator&)            = delete;
    Estimator& operator=(const Estimator&) = delete;
    Estimator(Estimator&&)                 = delete;
    Estimator& operator=(Estimator&&)      = delete;

    void start() {
        thread_ = std::thread(&Estimator::run, this);
    }

    void join() {
        if (thread_.joinable()) thread_.join();
    }

    // Live counter — safe from any thread, useful for "are we still
    // running" checks while the estimator is active.
    std::uint64_t tick_count() const noexcept {
        return tick_count_.load(std::memory_order_relaxed);
    }

    // The following accessors read fields written only by the estimator
    // thread. They are safe to call from another thread only after
    // join() — no internal synchronization beyond the thread join's
    // happens-before edge.
    FusedState                       last_state() const noexcept { return last_state_; }
    // Latency stats are in NANOSECONDS (fusion work is often sub-us).
    const RunningStats&              latency_ns() const noexcept { return latency_stats_; }
    // Jitter stats are in MICROSECONDS (scheduler-scale).
    const RunningStats&              jitter_us()  const noexcept { return jitter_stats_; }
    const std::vector<std::uint64_t>& samples_consumed() const noexcept {
        return samples_consumed_per_sensor_;
    }
    std::uint64_t log_drops() const noexcept {
        return log_buffer_ ? log_buffer_->dropped() : 0;
    }

private:
    static Clock::duration period_from_hz(double hz) noexcept {
        if (hz <= 0.0) return Clock::duration::max();
        return std::chrono::nanoseconds(static_cast<long long>(1.0e9 / hz));
    }

    void run() {
        const auto start_time = Clock::now();
        auto next_tick = start_time + period_;
        const auto spin_window = std::chrono::microseconds(cfg_.spin_wait_us);

        while (running_->load(std::memory_order_acquire)) {
            // Two-stage wait: coarse sleep_until gets us close, busy
            // spin closes the gap to single-digit microseconds. The
            // spin window is bounded by spin_wait_us so the CPU cost
            // is bounded too: at 200 Hz with a 100us window the spin
            // wastes ~2% of one core.
            if (cfg_.spin_wait_us > 0) {
                const auto coarse_target = next_tick - spin_window;
                if (Clock::now() < coarse_target) {
                    std::this_thread::sleep_until(coarse_target);
                }
                while (Clock::now() < next_tick) {
                    // spin
                }
            } else {
                std::this_thread::sleep_until(next_tick);
            }

            // Re-check shutdown after the wait; saves one wasted
            // fusion if we were told to stop while sleeping.
            if (!running_->load(std::memory_order_acquire)) break;

            const auto wake_time = Clock::now();
            const auto jitter_us =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    wake_time - next_tick).count();
            jitter_stats_.add(static_cast<double>(jitter_us));

            const auto work_start = Clock::now();
            fuse_tick(wake_time);
            const auto work_end = Clock::now();
            // Measure in nanoseconds — the fusion work is often under a
            // microsecond, so microsecond resolution would round most
            // ticks to zero. Stats are kept in nanoseconds; the
            // benchmark divides by 1000 to report microseconds with
            // sub-us precision.
            const auto latency_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    work_end - work_start).count();
            latency_stats_.add(static_cast<double>(latency_ns));

            if (log_buffer_) {
                const LogRecord record{
                    last_state_,
                    static_cast<std::int64_t>(latency_ns),
                    static_cast<std::int64_t>(jitter_us)};
                log_buffer_->push(record);
            }

            tick_count_.fetch_add(1, std::memory_order_relaxed);

            // Advance deadline; if we're behind, resync to "now" rather
            // than catching up by firing a burst of ticks.
            next_tick += period_;
            const auto now_after = Clock::now();
            if (next_tick < now_after) {
                next_tick = now_after + period_;
            }
        }
    }

    void fuse_tick(const Timestamp& now) {
        constexpr int kNumChannels = static_cast<int>(SensorType::Count_);

        // Reset the "has a fresh reading this tick" flags. The reading
        // values are overwritten on a fresh push so we don't need to
        // wipe them.
        for (auto& p : latest_per_sensor_) {
            p.second = false;
        }

        // Drain every buffer. Keep only the most recent reading per
        // sensor — older intra-tick readings are intentionally
        // discarded. Increment the per-sensor consumed count by the
        // full pop count so the stats reflect throughput, not just
        // the number we kept.
        SensorReading r;
        for (std::size_t i = 0; i < sensors_.size(); ++i) {
            std::uint64_t popped = 0;
            while (sensors_[i].buffer->pop(r)) {
                latest_per_sensor_[i] = {r, true};
                ++popped;
            }
            samples_consumed_per_sensor_[i] += popped;
        }

        // Inverse-variance weighted accumulator, one per channel.
        struct Accum {
            double weighted_sum = 0.0;
            double weight_sum   = 0.0;
            int    count        = 0;
        };
        std::array<Accum, kNumChannels> accum{};

        for (std::size_t i = 0; i < sensors_.size(); ++i) {
            if (!latest_per_sensor_[i].second) continue;
            const SensorReading& reading = latest_per_sensor_[i].first;
            const std::size_t ch = static_cast<std::size_t>(reading.type);
            const double w = sensors_[i].config.fusion_weight;
            accum[ch].weighted_sum += w * reading.value;
            accum[ch].weight_sum   += w;
            ++accum[ch].count;
        }

        // Update channels that received readings; hold previous value
        // for those that didn't.
        std::uint32_t mask = 0;
        for (std::size_t ch = 0;
             ch < static_cast<std::size_t>(kNumChannels); ++ch) {
            if (accum[ch].count > 0 && accum[ch].weight_sum > 0.0) {
                last_state_.values[ch] =
                    accum[ch].weighted_sum / accum[ch].weight_sum;
                mask |= (1u << ch);
            }
        }

        // Optional Kalman filter on the Position + Velocity channels.
        // Runs a constant-velocity predict to "now", then folds in
        // whichever position/velocity measurements arrived this tick,
        // each weighted by its own noise variance. It always yields an
        // estimate for both channels — coasting on the prediction when no
        // measurement arrived — so their bits are always set. Takes
        // precedence over the complementary filter.
        if (cfg_.use_kalman_filter && has_pos_vel_) {
            constexpr std::size_t kPos = static_cast<std::size_t>(SensorType::Position);
            constexpr std::size_t kVel = static_cast<std::size_t>(SensorType::Velocity);

            kf_.predict(tick_dt_seconds(now));
            for (std::size_t i = 0; i < sensors_.size(); ++i) {
                if (!latest_per_sensor_[i].second) continue;
                const SensorReading& reading = latest_per_sensor_[i].first;
                const double sigma = sensors_[i].config.noise_stddev;
                const double R = (sigma > 0.0) ? sigma * sigma : 1e-12;
                if (reading.type == SensorType::Position) {
                    kf_.update_position(reading.value, R);
                } else if (reading.type == SensorType::Velocity) {
                    kf_.update_velocity(reading.value, R);
                }
            }
            last_state_.values[kPos] = kf_.position();
            last_state_.values[kVel] = kf_.velocity();
            mask |= (1u << kPos) | (1u << kVel);
        }

        // Optional complementary filter on the Position channel. Blends
        // the encoder-measured position with the velocity integral so we
        // get encoder accuracy at low frequency, velocity smoothness at
        // high frequency, and — importantly — a usable position estimate
        // even on ticks where no fresh encoder sample arrived.
        else if (cfg_.use_complementary_filter) {
            constexpr std::size_t kPos = static_cast<std::size_t>(SensorType::Position);
            constexpr std::size_t kVel = static_cast<std::size_t>(SensorType::Velocity);
            const std::uint32_t pos_bit = (1u << kPos);
            const std::uint32_t vel_bit = (1u << kVel);

            // Need a velocity estimate (this tick or held over) to predict.
            const bool have_velocity =
                (mask & vel_bit) || (last_state_.channel_mask & vel_bit);

            if (have_velocity) {
                const double dt = tick_dt_seconds(now);
                const double velocity = last_state_.values[kVel];
                // Predict from the *previous* fused position plus the
                // velocity integral over this interval.
                const double predicted = comp_prev_position_ + velocity * dt;

                double fused_position;
                if (mask & pos_bit) {
                    // Encoder measured this tick: standard blend.
                    const double measured = last_state_.values[kPos];
                    const double a = cfg_.complementary_alpha;
                    fused_position = a * measured + (1.0 - a) * predicted;
                } else {
                    // No encoder this tick: coast on pure integration.
                    fused_position = predicted;
                }

                last_state_.values[kPos] = fused_position;
                comp_prev_position_      = fused_position;
                mask |= pos_bit;  // we always have a position estimate now
            }
        }

        last_state_.channel_mask = mask;
        last_state_.timestamp    = now;
        last_state_.tick         = tick_count_.load(std::memory_order_relaxed);
    }

    // Seconds since the previous fuse_tick, used by the complementary
    // filter's velocity integration and the Kalman filter's predict step.
    // First call returns the nominal loop period so the very first
    // prediction isn't a huge dt.
    double tick_dt_seconds(const Timestamp& now) noexcept {
        double dt;
        if (have_last_tick_time_) {
            dt = std::chrono::duration<double>(now - last_tick_time_).count();
        } else {
            dt = std::chrono::duration<double>(period_).count();
            have_last_tick_time_ = true;
        }
        last_tick_time_ = now;
        return dt;
    }

    // Config and ports.
    EstimatorConfig          cfg_;
    std::vector<SensorPort>  sensors_;
    LogBuffer*               log_buffer_;
    const std::atomic<bool>* running_;
    std::thread              thread_;
    Clock::duration          period_;

    // Hot-path scratch storage. Reused every tick to avoid per-tick
    // allocations that would inflate jitter.
    std::vector<std::pair<SensorReading, bool>> latest_per_sensor_;

    // State, owned by the estimator thread. Read-after-join only.
    FusedState                 last_state_{};
    RunningStats               latency_stats_;
    RunningStats               jitter_stats_;
    std::vector<std::uint64_t> samples_consumed_per_sensor_;
    std::atomic<std::uint64_t> tick_count_{0};

    // Complementary filter state (used only when enabled in config).
    double    comp_prev_position_  = 0.0;

    // Per-tick dt tracking, shared by the complementary and Kalman filters.
    Timestamp last_tick_time_{};
    bool      have_last_tick_time_ = false;

    // Kalman filter state (used only when enabled in config).
    KalmanFilter2 kf_;
    bool          has_pos_vel_ = false;
};

} // namespace sfp
