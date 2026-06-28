// logger.hpp — logger thread that drains the estimator's LogRecord
// buffer to a CSV file.
//
// The logger runs on its own thread and is the only thread that touches
// the file. The estimator pushes LogRecords; the logger pops them in
// batches and writes them. Because file I/O is bursty (an fwrite can
// block on a page-cache flush, an fsync can block on the disk), the
// estimator must never wait on it — that's the whole reason the
// LogRecord buffer exists. A slow disk causes log drops, never a
// missed estimator deadline.
//
// Templated on LogBuffer so the lock-free / locked benchmark swap
// reaches here too.
//
// CSV columns:
//   tick, t_seconds, latency_us, jitter_us, <one column per active
//   channel>, channel_mask
//
// The active channels are passed in at construction so the header and
// the per-row column set match the configured sensors. t_seconds is
// relative to the logger's start so the plot script gets a clean
// zero-based time axis.

#pragma once

#include "config.hpp"
#include "messages.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

namespace sfp {

template <typename LogBuffer>
class Logger {
public:
    // stream_json: instead of writing CSV to `path`, emit one JSON object
    // per tick to stdout (flushed live). This is what feeds the web
    // dashboard's Server-Sent-Events stream; `path` is ignored.
    Logger(const std::string&             path,
           std::vector<SensorType>        channels,  // columns to emit
           LogBuffer*                      buffer,
           const std::atomic<bool>*        running,
           bool                            stream_json = false)
        : path_(path)
        , channels_(std::move(channels))
        , buffer_(buffer)
        , running_(running)
        , stream_json_(stream_json)
    {}

    Logger(const Logger&)            = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&)                 = delete;
    Logger& operator=(Logger&&)      = delete;

    bool start() {
        if (stream_json_) {
            file_ = stdout;          // stream JSON to stdout; no header
        } else {
            file_ = std::fopen(path_.c_str(), "w");
            if (!file_) return false;
            write_header();
        }
        thread_ = std::thread(&Logger::run, this);
        return true;
    }

    void join() {
        if (thread_.joinable()) thread_.join();
        // Don't fclose stdout in stream mode — we don't own it.
        if (file_ && !stream_json_) {
            std::fclose(file_);
        }
        file_ = nullptr;
    }

    std::uint64_t records_written() const noexcept {
        return records_written_.load(std::memory_order_relaxed);
    }

private:
    void write_header() {
        std::fputs("tick,t_seconds,latency_ns,jitter_us", file_);
        for (SensorType ch : channels_) {
            std::fputc(',', file_);
            std::fputs(sensor_type_name(ch), file_);
        }
        std::fputs(",channel_mask\n", file_);
    }

    void write_record(const LogRecord& rec, double t_seconds) {
        if (stream_json_) { write_json(rec, t_seconds); return; }
        // Fixed-width-ish CSV. fprintf is fine here — this is the logger
        // thread, decoupled from the estimator, so its cost doesn't
        // affect the measured loop latency.
        std::fprintf(file_, "%llu,%.6f,%lld,%lld",
                     static_cast<unsigned long long>(rec.state.tick),
                     t_seconds,
                     static_cast<long long>(rec.loop_latency_ns),
                     static_cast<long long>(rec.loop_jitter_us));
        for (SensorType ch : channels_) {
            const std::size_t idx = static_cast<std::size_t>(ch);
            std::fprintf(file_, ",%.6f", rec.state.values[idx]);
        }
        std::fprintf(file_, ",%u\n",
                     static_cast<unsigned>(rec.state.channel_mask));
    }

    // One compact JSON object per line, e.g.
    //   {"tick":42,"t":0.21,"latency_ns":380,"jitter_us":35,
    //    "temperature":40.1,"voltage":399.8,"current":120.3,"mask":7}
    void write_json(const LogRecord& rec, double t_seconds) {
        std::fprintf(file_,
                     "{\"tick\":%llu,\"t\":%.6f,\"latency_ns\":%lld,"
                     "\"jitter_us\":%lld",
                     static_cast<unsigned long long>(rec.state.tick),
                     t_seconds,
                     static_cast<long long>(rec.loop_latency_ns),
                     static_cast<long long>(rec.loop_jitter_us));
        for (SensorType ch : channels_) {
            const std::size_t idx = static_cast<std::size_t>(ch);
            std::fprintf(file_, ",\"%s\":%.6f",
                         sensor_type_name(ch), rec.state.values[idx]);
        }
        std::fprintf(file_, ",\"mask\":%u}\n",
                     static_cast<unsigned>(rec.state.channel_mask));
    }

    void run() {
        const auto t0 = Clock::now();
        LogRecord rec;

        // Drain loop. We pop in a tight inner loop to keep the buffer
        // from filling, then yield when it's empty so we don't burn a
        // core spinning. When the estimator stops, drain whatever is
        // left before exiting.
        while (running_->load(std::memory_order_acquire)) {
            bool any = false;
            while (buffer_->pop(rec)) {
                const double t =
                    std::chrono::duration<double>(rec.state.timestamp - t0)
                        .count();
                write_record(rec, t);
                records_written_.fetch_add(1, std::memory_order_relaxed);
                any = true;
            }
            if (any && stream_json_) {
                std::fflush(file_);  // push this batch to the SSE reader now
            }
            if (!any) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        // Final drain after shutdown.
        while (buffer_->pop(rec)) {
            const double t =
                std::chrono::duration<double>(rec.state.timestamp - t0)
                    .count();
            write_record(rec, t);
            records_written_.fetch_add(1, std::memory_order_relaxed);
        }

        std::fflush(file_);
    }

    std::string              path_;
    std::vector<SensorType>  channels_;
    LogBuffer*               buffer_;
    const std::atomic<bool>* running_;
    std::thread              thread_;
    std::FILE*               file_ = nullptr;
    bool                     stream_json_ = false;

    std::atomic<std::uint64_t> records_written_{0};
};

} // namespace sfp
