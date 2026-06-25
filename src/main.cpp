// main.cpp — command-line entry point for the fusion pipeline.
//
// Usage:
//   sfp [--config battery|robotics] [--duration SECONDS]
//       [--spin-us N] [--csv PATH] [--compare]
//
// Default: battery config, 30 s, 50 us spin, CSV to fusion_log.csv,
// lock-free buffers only.
//
// With --compare, runs the pipeline twice — once lock-free, once
// locked — and prints a side-by-side table. The lock-free run writes
// the CSV; the locked run is timing-only (no CSV) to keep the disk out
// of the comparison.

#include "benchmark.hpp"
#include "pipeline_config.hpp"
#include "platform.hpp"
#include "ring_buffer.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

// Buffer capacities. Sensor buffers are small — at the configured
// rates the estimator drains them every 5 ms, so 256 is generous.
// The log buffer is larger because file I/O is bursty.
constexpr std::size_t kSensorCap = 256;
constexpr std::size_t kLogCap    = 8192;

using LFSensorBuf = sfp::SpscRingBuffer<sfp::SensorReading, kSensorCap>;
using LFLogBuf    = sfp::SpscRingBuffer<sfp::LogRecord,     kLogCap>;
using LKSensorBuf = sfp::LockedRingBuffer<sfp::SensorReading, kSensorCap>;
using LKLogBuf    = sfp::LockedRingBuffer<sfp::LogRecord,     kLogCap>;

void print_result(const sfp::BenchmarkResult& r) {
    std::printf("  %-16s : %s\n", "buffer", r.label.c_str());
    std::printf("  %-16s : %llu\n", "ticks",
                static_cast<unsigned long long>(r.ticks));
    std::printf("  %-16s : %.3f s\n", "duration", r.duration_s);
    std::printf("  %-16s : %.1f Hz\n", "throughput", r.throughput_hz);
    std::printf("  %-16s : %.2f us\n", "latency mean", r.latency_mean_us);
    std::printf("  %-16s : %.2f us\n", "latency max", r.latency_max_us);
    std::printf("  %-16s : %.2f us\n", "jitter mean", r.jitter_mean_us);
    std::printf("  %-16s : %.2f us\n", "jitter stddev", r.jitter_stddev_us);
    std::printf("  %-16s : %.2f us\n", "jitter max", r.jitter_max_us);
    std::printf("  %-16s : %llu\n", "samples fused",
                static_cast<unsigned long long>(r.total_consumed));
    std::printf("  %-16s : %llu\n", "sensor drops",
                static_cast<unsigned long long>(r.total_sensor_drops));
    std::printf("  %-16s : %llu\n", "log records",
                static_cast<unsigned long long>(r.log_records));
    std::printf("  %-16s : %llu\n", "log drops",
                static_cast<unsigned long long>(r.log_drops));
}

void print_comparison(const sfp::BenchmarkResult& lf,
                      const sfp::BenchmarkResult& lk) {
    std::printf("\n");
    std::printf("%-20s %15s %15s\n", "metric", "lock-free", "locked");
    std::printf("%-20s %15s %15s\n", "------", "---------", "------");
    std::printf("%-20s %15.1f %15.1f\n", "throughput (Hz)",
                lf.throughput_hz, lk.throughput_hz);
    std::printf("%-20s %15.2f %15.2f\n", "latency mean (us)",
                lf.latency_mean_us, lk.latency_mean_us);
    std::printf("%-20s %15.2f %15.2f\n", "latency max (us)",
                lf.latency_max_us, lk.latency_max_us);
    std::printf("%-20s %15.2f %15.2f\n", "jitter mean (us)",
                lf.jitter_mean_us, lk.jitter_mean_us);
    std::printf("%-20s %15.2f %15.2f\n", "jitter stddev (us)",
                lf.jitter_stddev_us, lk.jitter_stddev_us);
    std::printf("%-20s %15.2f %15.2f\n", "jitter max (us)",
                lf.jitter_max_us, lk.jitter_max_us);
    std::printf("%-20s %15llu %15llu\n", "samples fused",
                static_cast<unsigned long long>(lf.total_consumed),
                static_cast<unsigned long long>(lk.total_consumed));
    if (lf.latency_mean_us > 0.0) {
        const double speedup = lk.latency_mean_us / lf.latency_mean_us;
        std::printf("\n  lock-free is %.2fx %s on mean fusion latency\n",
                    speedup >= 1.0 ? speedup : 1.0 / speedup,
                    speedup >= 1.0 ? "faster" : "slower");
    }
}

} // namespace

int main(int argc, char** argv) {
    // On Windows, pull the system timer down to 1 ms for the process
    // lifetime so the fixed-rate loops can hit their deadlines. No-op
    // on POSIX.
    sfp::ScopedHighResTimer hires_timer;

    std::string config_name = "battery";
    double      duration_s  = 30.0;
    int         spin_us     = 50;
    std::string csv_path    = "fusion_log.csv";
    bool        compare     = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "error: %s needs a value\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--config")        config_name = next("--config");
        else if (arg == "--duration") duration_s = std::atof(next("--duration"));
        else if (arg == "--spin-us")  spin_us = std::atoi(next("--spin-us"));
        else if (arg == "--csv")      csv_path = next("--csv");
        else if (arg == "--compare")  compare = true;
        else if (arg == "--help" || arg == "-h") {
            std::printf(
                "usage: %s [--config battery|robotics] "
                "[--duration S] [--spin-us N] [--csv PATH] [--compare]\n",
                argv[0]);
            return 0;
        } else {
            std::fprintf(stderr, "unknown argument: %s\n", arg.c_str());
            return 2;
        }
    }

    sfp::PipelineConfig cfg =
        (config_name == "robotics")
            ? sfp::robotics_config(duration_s, spin_us)
            : sfp::battery_config(duration_s, spin_us);

    std::printf("=== Sensor fusion pipeline: %s config ===\n",
                cfg.name.c_str());
    std::printf("  %zu sensors, %.0f Hz estimator, %.0f s run, "
                "%d us spin window\n\n",
                cfg.sensors.size(), cfg.estimator.loop_hz,
                cfg.duration_s, spin_us);

    if (compare) {
        std::printf("Running lock-free pipeline (with CSV)...\n");
        auto lf = sfp::run_pipeline<LFSensorBuf, LFLogBuf>(
            cfg, "lock-free", csv_path);
        std::printf("Running locked pipeline (timing only)...\n");
        auto lk = sfp::run_pipeline<LKSensorBuf, LKLogBuf>(
            cfg, "locked", "");
        print_comparison(lf, lk);
        std::printf("\nCSV written to %s\n", csv_path.c_str());
    } else {
        auto r = sfp::run_pipeline<LFSensorBuf, LFLogBuf>(
            cfg, "lock-free", csv_path);
        print_result(r);
        std::printf("\nCSV written to %s\n", csv_path.c_str());
    }

    return 0;
}
