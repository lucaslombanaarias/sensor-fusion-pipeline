// test_logger.cpp — verify the logger writes well-formed CSV, emits the
// right columns for the configured channels, and loses no records under
// a normal load.
//
// The logger is fed by hand here (we push LogRecords directly rather
// than running a real estimator) so the tests are deterministic: we
// know exactly how many records went in and can check exactly that
// many came out, plus that the values round-trip through the CSV.

#include "config.hpp"
#include "logger.hpp"
#include "messages.hpp"
#include "ring_buffer.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr std::size_t kLogCap = 4096;
using LogBuf  = sfp::SpscRingBuffer<sfp::LogRecord, kLogCap>;
using LogBufL = sfp::LockedRingBuffer<sfp::LogRecord, kLogCap>;

std::vector<std::string> read_lines(const std::string& path) {
    std::ifstream f(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(f, line)) lines.push_back(line);
    return lines;
}

std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) out.push_back(item);
    return out;
}

sfp::LogRecord make_record(std::uint64_t tick, double temp, double volt,
                           std::int64_t lat, std::int64_t jit) {
    sfp::LogRecord rec{};
    rec.state.tick = tick;
    rec.state.timestamp = sfp::Clock::now();
    rec.state.values.fill(0.0);
    rec.state.values[static_cast<std::size_t>(sfp::SensorType::Temperature)] = temp;
    rec.state.values[static_cast<std::size_t>(sfp::SensorType::Voltage)]     = volt;
    rec.state.channel_mask =
        (1u << static_cast<unsigned>(sfp::SensorType::Temperature)) |
        (1u << static_cast<unsigned>(sfp::SensorType::Voltage));
    rec.loop_latency_ns = lat;
    rec.loop_jitter_us  = jit;
    return rec;
}

bool test_csv_header_and_columns() {
    const std::string path = "/tmp/sfp_test_logger_1.csv";
    std::remove(path.c_str());

    LogBuf buf;
    std::atomic<bool> running{true};
    std::vector<sfp::SensorType> channels{
        sfp::SensorType::Temperature, sfp::SensorType::Voltage};

    sfp::Logger<LogBuf> logger(path, channels, &buf, &running);
    if (!logger.start()) {
        std::cerr << "  FAIL: could not open " << path << '\n';
        return false;
    }

    for (std::uint64_t i = 0; i < 100; ++i) {
        while (!buf.push(make_record(i, 25.0 + 0.1 * static_cast<double>(i),
                                     48.0, 5, 12))) {
            std::this_thread::yield();
        }
    }
    // Give the logger time to drain.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    running.store(false, std::memory_order_release);
    logger.join();

    auto lines = read_lines(path);
    if (lines.empty()) {
        std::cerr << "  FAIL: empty CSV\n";
        return false;
    }

    const std::string expected_header =
        "tick,t_seconds,latency_ns,jitter_us,temperature,voltage,channel_mask";
    if (lines[0] != expected_header) {
        std::cerr << "  FAIL: header mismatch\n    got: " << lines[0]
                  << "\n    exp: " << expected_header << '\n';
        return false;
    }

    // 1 header + 100 data rows.
    if (lines.size() != 101) {
        std::cerr << "  FAIL: expected 101 lines, got "
                  << lines.size() << '\n';
        return false;
    }

    // Spot-check the columns of the first data row.
    auto cols = split(lines[1], ',');
    if (cols.size() != 7) {
        std::cerr << "  FAIL: row has " << cols.size()
                  << " columns, expected 7\n";
        return false;
    }

    std::cout << "  PASS: header + " << (lines.size() - 1)
              << " rows, 7 columns each.\n";
    return true;
}

bool test_values_round_trip() {
    const std::string path = "/tmp/sfp_test_logger_2.csv";
    std::remove(path.c_str());

    LogBuf buf;
    std::atomic<bool> running{true};
    std::vector<sfp::SensorType> channels{
        sfp::SensorType::Temperature, sfp::SensorType::Voltage};

    sfp::Logger<LogBuf> logger(path, channels, &buf, &running);
    logger.start();

    // Push a few records with known values.
    const double temps[] = {25.5, 26.0, 24.75};
    const double volts[] = {48.1, 47.9, 48.0};
    for (int i = 0; i < 3; ++i) {
        while (!buf.push(make_record(static_cast<std::uint64_t>(i),
                                     temps[i], volts[i], 7, 15))) {
            std::this_thread::yield();
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    running.store(false, std::memory_order_release);
    logger.join();

    auto lines = read_lines(path);
    if (lines.size() != 4) {  // header + 3
        std::cerr << "  FAIL: expected 4 lines, got " << lines.size() << '\n';
        return false;
    }

    // Column order: tick,t_seconds,latency_ns,jitter_us,temperature,voltage,channel_mask
    for (int i = 0; i < 3; ++i) {
        auto cols = split(lines[static_cast<std::size_t>(i + 1)], ',');
        const double temp = std::stod(cols[4]);
        const double volt = std::stod(cols[5]);
        const long   lat  = std::stol(cols[2]);
        const long   jit  = std::stol(cols[3]);
        if (std::abs(temp - temps[i]) > 1e-4 ||
            std::abs(volt - volts[i]) > 1e-4 ||
            lat != 7 || jit != 15) {
            std::cerr << "  FAIL: row " << i << " value mismatch — temp="
                      << temp << " volt=" << volt << " lat=" << lat
                      << " jit=" << jit << '\n';
            return false;
        }
    }

    std::cout << "  PASS: 3 records round-tripped through CSV exactly.\n";
    return true;
}

bool test_no_loss_under_load() {
    const std::string path = "/tmp/sfp_test_logger_3.csv";
    std::remove(path.c_str());

    LogBuf buf;
    std::atomic<bool> running{true};
    std::vector<sfp::SensorType> channels{sfp::SensorType::Temperature,
                                          sfp::SensorType::Voltage};

    sfp::Logger<LogBuf> logger(path, channels, &buf, &running);
    logger.start();

    // Feeder thread pushes 50k records, retrying on full so nothing is
    // lost at the producer side. The logger must write all of them.
    constexpr std::uint64_t kN = 50'000;
    std::thread feeder([&] {
        for (std::uint64_t i = 0; i < kN; ++i) {
            while (!buf.push(make_record(i, 25.0, 48.0, 5, 10))) {
                std::this_thread::yield();
            }
        }
    });
    feeder.join();

    // Wait for the logger to catch up, then stop.
    while (buf.size_approx() > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    running.store(false, std::memory_order_release);
    logger.join();

    const auto written = logger.records_written();
    if (written != kN) {
        std::cerr << "  FAIL: wrote " << written << " of " << kN << '\n';
        return false;
    }
    // Also confirm the file has the right line count.
    auto lines = read_lines(path);
    if (lines.size() != kN + 1) {
        std::cerr << "  FAIL: file has " << lines.size()
                  << " lines, expected " << (kN + 1) << '\n';
        return false;
    }

    std::cout << "  PASS: " << written
              << " records written, zero loss, file line count matches.\n";
    return true;
}

bool test_locked_logger() {
    const std::string path = "/tmp/sfp_test_logger_4.csv";
    std::remove(path.c_str());

    LogBufL buf;
    std::atomic<bool> running{true};
    std::vector<sfp::SensorType> channels{sfp::SensorType::Temperature,
                                          sfp::SensorType::Voltage};

    sfp::Logger<LogBufL> logger(path, channels, &buf, &running);
    logger.start();

    for (std::uint64_t i = 0; i < 500; ++i) {
        while (!buf.push(make_record(i, 25.0, 48.0, 5, 10))) {
            std::this_thread::yield();
        }
    }
    while (buf.size_approx() > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    running.store(false, std::memory_order_release);
    logger.join();

    if (logger.records_written() != 500) {
        std::cerr << "  FAIL: locked logger wrote "
                  << logger.records_written() << " of 500\n";
        return false;
    }
    std::cout << "  PASS: locked variant wrote 500 records.\n";
    return true;
}

} // namespace

int main() {
    std::cout << "Test 1: CSV header and column count\n";
    if (!test_csv_header_and_columns()) return 1;

    std::cout << "Test 2: values round-trip through CSV\n";
    if (!test_values_round_trip()) return 1;

    std::cout << "Test 3: no record loss under load (50k records)\n";
    if (!test_no_loss_under_load()) return 1;

    std::cout << "Test 4: locked buffer variant\n";
    if (!test_locked_logger()) return 1;

    std::cout << "\nAll logger tests passed.\n";
    return 0;
}
