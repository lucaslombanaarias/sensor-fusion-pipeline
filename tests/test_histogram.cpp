// test_histogram.cpp — unit tests for the log-bucketed latency histogram
// (include/histogram.hpp).
//
// The histogram trades exactness for fixed memory: percentiles are
// accurate to the bucket width (~1/kSubBuckets relative, ~12.5%). The
// tolerances below reflect that — tight enough to catch a real bug,
// loose enough not to fail on quantization.

#include "histogram.hpp"

#include <cmath>
#include <iostream>

namespace {

// A constant stream should report that constant at every percentile,
// within bucket resolution, and exact min/max.
bool test_constant() {
    sfp::LatencyHistogram h;
    const double v = 376.0;  // the project's headline mean, in ns
    for (int i = 0; i < 10000; ++i) h.record(v);

    const double p50 = h.percentile(50);
    const double p999 = h.percentile(99.9);
    if (std::abs(p50 - v) > v * 0.15 || std::abs(p999 - v) > v * 0.15) {
        std::cerr << "  FAIL: constant " << v << " -> p50=" << p50
                  << " p99.9=" << p999 << '\n';
        return false;
    }
    if (h.min() != 376 || h.max() != 376) {
        std::cerr << "  FAIL: min/max " << h.min() << "/" << h.max() << '\n';
        return false;
    }
    std::cout << "  PASS: constant " << v << " -> p50=" << p50
              << ", p99.9=" << p999 << ", min/max exact.\n";
    return true;
}

// A uniform ramp 1..N: p-th percentile should land near p% of N.
bool test_uniform_percentiles() {
    sfp::LatencyHistogram h;
    const int N = 100000;
    for (int i = 1; i <= N; ++i) h.record(static_cast<double>(i));

    struct Case { double p; double expected; };
    const Case cases[] = {{50, 50000}, {90, 90000}, {99, 99000}, {99.9, 99900}};
    for (const auto& c : cases) {
        const double got = h.percentile(c.p);
        // Allow 15% relative (bucket resolution) plus a small floor.
        if (std::abs(got - c.expected) > c.expected * 0.15) {
            std::cerr << "  FAIL: p" << c.p << " = " << got
                      << " expected ~" << c.expected << '\n';
            return false;
        }
    }
    std::cout << "  PASS: uniform 1.." << N << " — p50/p90/p99/p99.9 ="
              << h.percentile(50) << "/" << h.percentile(90) << "/"
              << h.percentile(99) << "/" << h.percentile(99.9) << '\n';
    return true;
}

// Percentiles must be monotonic, and a heavy tail must lift p99.9 well
// above p50 — the property that makes the histogram worth having.
bool test_tail_is_visible() {
    sfp::LatencyHistogram h;
    for (int i = 0; i < 9990; ++i) h.record(300.0);     // body ~300 ns
    for (int i = 0; i < 10; ++i)   h.record(500000.0);  // 0.1% tail at 0.5 ms

    const double p50  = h.percentile(50);
    const double p99  = h.percentile(99);
    const double p999 = h.percentile(99.9);
    if (!(p50 <= p99 && p99 <= p999)) {
        std::cerr << "  FAIL: not monotonic: " << p50 << " " << p99
                  << " " << p999 << '\n';
        return false;
    }
    if (p50 > 1000.0 || p999 < 100000.0) {
        std::cerr << "  FAIL: tail not separated — p50=" << p50
                  << " p99.9=" << p999 << '\n';
        return false;
    }
    std::cout << "  PASS: body p50=" << p50 << " ns, tail p99.9=" << p999
              << " ns (tail visible).\n";
    return true;
}

} // namespace

int main() {
    std::cout << "Test 1: constant stream\n";
    if (!test_constant()) return 1;

    std::cout << "Test 2: uniform-ramp percentiles\n";
    if (!test_uniform_percentiles()) return 1;

    std::cout << "Test 3: heavy tail lifts p99.9 above p50\n";
    if (!test_tail_is_visible()) return 1;

    std::cout << "\nAll histogram tests passed.\n";
    return 0;
}
