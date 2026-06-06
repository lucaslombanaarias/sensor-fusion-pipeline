// stats.hpp — running statistics via Welford's online algorithm.
//
// Adds samples one at a time in O(1); reports mean, variance, stddev,
// min, max without storing the samples. This is the right primitive
// for the estimator's per-tick latency and jitter accumulation: at
// 200 Hz over 30 seconds that's 6000 samples — storing them is fine,
// but the online approach lets us also use the same class for
// arbitrarily long runs without a memory bound.
//
// Not thread-safe. The estimator thread updates its own instances;
// other threads only read after join().

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace sfp {

class RunningStats {
public:
    void add(double x) noexcept {
        ++n_;
        // Welford recurrence: numerically stable for streaming data.
        // delta * delta2 == (x - old_mean) * (x - new_mean), which is
        // the per-sample contribution to the sum of squared deviations.
        const double delta = x - mean_;
        mean_ += delta / static_cast<double>(n_);
        const double delta2 = x - mean_;
        m2_ += delta * delta2;

        if (x < min_) min_ = x;
        if (x > max_) max_ = x;
    }

    std::uint64_t count() const noexcept { return n_; }

    double mean() const noexcept {
        return (n_ > 0) ? mean_ : 0.0;
    }

    // Sample variance (n-1 denominator), 0 when there's not enough data.
    double variance() const noexcept {
        return (n_ > 1) ? m2_ / static_cast<double>(n_ - 1) : 0.0;
    }

    double stddev() const noexcept {
        return std::sqrt(variance());
    }

    double min() const noexcept {
        return (n_ > 0) ? min_ : 0.0;
    }

    double max() const noexcept {
        return (n_ > 0) ? max_ : 0.0;
    }

private:
    std::uint64_t n_    = 0;
    double        mean_ = 0.0;
    double        m2_   = 0.0;
    double        min_  =  std::numeric_limits<double>::infinity();
    double        max_  = -std::numeric_limits<double>::infinity();
};

} // namespace sfp
