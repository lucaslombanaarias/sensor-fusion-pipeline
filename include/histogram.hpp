// histogram.hpp — fixed-memory logarithmic histogram for latency
// percentiles (p50 / p99 / p99.9).
//
// RunningStats (stats.hpp) gives mean/stddev/min/max in O(1), but it
// cannot answer a percentile query — you can't recover a p99 from a
// running mean. For tail-latency work the percentiles are the whole
// point: the mean hides the scheduler-induced tail, p99.9 exposes it.
//
// This is an HdrHistogram-style log-bucketed histogram: kSubBuckets
// buckets per power-of-two octave, so it keeps constant *relative*
// precision (~1/kSubBuckets) across many orders of magnitude with a
// fixed, small array and zero allocation. record() is O(1) — one log2
// and an increment — so it stays on the estimator's hot path without
// inflating jitter.
//
// Values are treated as nonnegative (nanoseconds in this project); a
// value <= 1 lands in bucket 0.

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace sfp {

class LatencyHistogram {
public:
    static constexpr int kSubBuckets = 8;    // buckets per octave (~12.5% res)
    static constexpr int kOctaves    = 28;   // up to 2^28 ns ≈ 0.27 s
    static constexpr int kNumBuckets = kSubBuckets * kOctaves;

    void record(double value_ns) noexcept {
        const std::int64_t v =
            (value_ns > 0.0) ? static_cast<std::int64_t>(value_ns) : 0;
        ++count_;
        if (v < min_) min_ = v;
        if (v > max_) max_ = v;
        ++buckets_[static_cast<std::size_t>(bucket_index(v))];
    }

    std::uint64_t count() const noexcept { return count_; }
    std::int64_t  min()   const noexcept { return (count_ > 0) ? min_ : 0; }
    std::int64_t  max()   const noexcept { return (count_ > 0) ? max_ : 0; }

    // Percentile p in (0, 100]; returns an estimate in the same units as
    // record() (nanoseconds). Resolution is the bucket width (~1/kSubBuckets
    // in relative terms).
    double percentile(double p) const noexcept {
        if (count_ == 0) return 0.0;
        const std::uint64_t target = static_cast<std::uint64_t>(
            std::ceil(p / 100.0 * static_cast<double>(count_)));
        std::uint64_t cum = 0;
        for (int i = 0; i < kNumBuckets; ++i) {
            cum += buckets_[static_cast<std::size_t>(i)];
            if (cum >= target) return bucket_value(i);
        }
        return static_cast<double>(max_);
    }

private:
    static int bucket_index(std::int64_t v) noexcept {
        if (v <= 1) return 0;
        int idx = static_cast<int>(
            std::log2(static_cast<double>(v)) * kSubBuckets);
        if (idx < 0) idx = 0;
        if (idx >= kNumBuckets) idx = kNumBuckets - 1;
        return idx;
    }

    // Geometric center of bucket idx — the inverse of bucket_index.
    static double bucket_value(int idx) noexcept {
        return std::pow(2.0,
            (static_cast<double>(idx) + 0.5) / kSubBuckets);
    }

    std::array<std::uint64_t, kNumBuckets> buckets_{};
    std::uint64_t count_ = 0;
    std::int64_t  min_   = std::numeric_limits<std::int64_t>::max();
    std::int64_t  max_   = std::numeric_limits<std::int64_t>::min();
};

} // namespace sfp
