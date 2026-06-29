// benchmark.hpp — assemble and run the full pipeline for a given buffer
// type, returning the measured metrics.
//
// This is where everything connects: N sensor producers, N sensor
// buffers, one estimator, one log buffer, one logger. The function is
// templated on the buffer template so we can instantiate the entire
// pipeline with lock-free buffers and again with locked buffers, and
// compare. That comparison is the headline result of the project.
//
// The sensor buffers are held in a std::deque, not a std::vector,
// because the estimator stores raw pointers to them and a vector could
// reallocate and invalidate those pointers when it grows. A deque
// gives stable element addresses.

#pragma once

#include "config.hpp"
#include "estimator.hpp"
#include "logger.hpp"
#include "messages.hpp"
#include "pipeline_config.hpp"
#include "sensor.hpp"
#include "stats.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace sfp {

struct BenchmarkResult {
    std::string   label;            // "lock-free" or "locked"
    std::uint64_t ticks            = 0;
    double        duration_s       = 0.0;
    double        throughput_hz    = 0.0;   // fused estimates per second
    double        latency_mean_us  = 0.0;
    double        latency_p50_us   = 0.0;
    double        latency_p99_us   = 0.0;
    double        latency_p999_us  = 0.0;
    double        latency_max_us   = 0.0;
    double        jitter_mean_us   = 0.0;
    double        jitter_stddev_us = 0.0;
    double        jitter_max_us    = 0.0;
    std::uint64_t total_consumed   = 0;     // sensor samples fused
    std::uint64_t total_sensor_drops = 0;
    std::uint64_t log_records      = 0;
    std::uint64_t log_drops        = 0;
};

// SensorBufT and LogBufT are the concrete buffer types (already
// parameterized on capacity). csv_path may be empty to skip logging.
template <typename SensorBufT, typename LogBufT>
BenchmarkResult run_pipeline(const PipelineConfig& cfg,
                             const std::string&    label,
                             const std::string&    csv_path,
                             bool                  stream_json = false) {
    using Sensor    = SensorProducer<SensorBufT>;
    using Estim     = Estimator<SensorBufT, LogBufT>;

    std::atomic<bool> running{true};

    // Stable-address storage for the per-sensor buffers.
    std::deque<SensorBufT> sensor_buffers;
    std::vector<std::unique_ptr<Sensor>> sensors;
    std::vector<typename Estim::SensorPort> ports;

    for (const auto& scfg : cfg.sensors) {
        sensor_buffers.emplace_back();
        SensorBufT* buf = &sensor_buffers.back();
        sensors.push_back(
            std::make_unique<Sensor>(scfg, buf, &running));
        ports.push_back(typename Estim::SensorPort{buf, scfg});
    }

    const bool want_logger = stream_json || !csv_path.empty();

    LogBufT log_buffer;
    Estim estimator(cfg.estimator, ports,
                    want_logger ? &log_buffer : nullptr,
                    &running);

    std::unique_ptr<Logger<LogBufT>> logger;
    if (want_logger) {
        logger = std::make_unique<Logger<LogBufT>>(
            csv_path, cfg.log_channels, &log_buffer, &running, stream_json);
    }

    // Start consumers before producers so no samples pile up before
    // anyone is draining.
    if (logger) logger->start();
    estimator.start();
    for (auto& s : sensors) s->start();

    const auto t_start = Clock::now();
    std::this_thread::sleep_for(
        std::chrono::duration<double>(cfg.duration_s));
    running.store(false, std::memory_order_release);

    for (auto& s : sensors) s->join();
    estimator.join();
    if (logger) logger->join();
    const auto t_end = Clock::now();

    const double elapsed =
        std::chrono::duration<double>(t_end - t_start).count();

    BenchmarkResult r;
    r.label            = label;
    r.ticks            = estimator.tick_count();
    r.duration_s       = elapsed;
    r.throughput_hz    = static_cast<double>(r.ticks) / elapsed;
    r.latency_mean_us  = estimator.latency_ns().mean() / 1000.0;
    r.latency_p50_us   = estimator.latency_hist().percentile(50.0)  / 1000.0;
    r.latency_p99_us   = estimator.latency_hist().percentile(99.0)  / 1000.0;
    r.latency_p999_us  = estimator.latency_hist().percentile(99.9)  / 1000.0;
    r.latency_max_us   = estimator.latency_ns().max() / 1000.0;
    r.jitter_mean_us   = estimator.jitter_us().mean();
    r.jitter_stddev_us = estimator.jitter_us().stddev();
    r.jitter_max_us    = estimator.jitter_us().max();

    for (auto c : estimator.samples_consumed()) r.total_consumed += c;
    for (auto& s : sensors) r.total_sensor_drops += s->samples_dropped();

    if (logger) {
        r.log_records = logger->records_written();
        r.log_drops   = estimator.log_drops();
    }
    return r;
}

} // namespace sfp
