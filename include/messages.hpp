// messages.hpp — types that flow between pipeline threads.
//
// This file will grow: SensorReading is here now; FusedState and
// LogRecord land when the estimator and logger arrive.
//
// All message types are trivially-copyable structs so they round-trip
// through the SPSC ring buffer without allocation or copy semantics.

#pragma once

#include <array>
#include <chrono>
#include <cstdint>

namespace sfp {

// All timestamps in the pipeline use steady_clock. Monotonic, not
// affected by wall-clock adjustments — the right choice for any
// measurement of duration or jitter.
using Clock     = std::chrono::steady_clock;
using Timestamp = Clock::time_point;

enum class SensorType : int {
    Temperature,
    Voltage,
    Current,
    Position,
    Velocity,
    Force,
    Count_  // sentinel; keep last
};

constexpr const char* sensor_type_name(SensorType t) noexcept {
    switch (t) {
        case SensorType::Temperature: return "temperature";
        case SensorType::Voltage:     return "voltage";
        case SensorType::Current:     return "current";
        case SensorType::Position:    return "position";
        case SensorType::Velocity:    return "velocity";
        case SensorType::Force:       return "force";
        case SensorType::Count_:      return "?";
    }
    return "?";
}

struct SensorReading {
    int        sensor_id;
    SensorType type;
    double     value;
    Timestamp  timestamp;
};

// Estimator output. One value per channel; channels with no fresh
// reading this tick keep their previous value and have their bit
// cleared in channel_mask. Plain struct, trivially copyable.
struct FusedState {
    std::uint64_t                                              tick;
    Timestamp                                                  timestamp;
    std::array<double, static_cast<std::size_t>(SensorType::Count_)> values;
    std::uint32_t channel_mask;
};

// What the estimator pushes to the logger: the fused state plus the
// per-tick diagnostic numbers the benchmark reports. Keeping these
// together means the CSV is self-contained — no separate latency file
// needed. loop_latency_ns is the fusion work time in nanoseconds;
// loop_jitter_us is the wake-vs-deadline error in microseconds.
struct LogRecord {
    FusedState   state;
    std::int64_t loop_latency_ns;
    std::int64_t loop_jitter_us;
};

} // namespace sfp
