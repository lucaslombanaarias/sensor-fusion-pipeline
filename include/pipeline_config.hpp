// pipeline_config.hpp — full pipeline configurations for the two demo
// contexts. These are the "swap a struct, change the whole demo"
// definitions the project is built around.
//
// A PipelineConfig bundles the estimator settings, the list of sensors,
// and the run parameters. main.cpp picks one based on a command-line
// argument.

#pragma once

#include "config.hpp"
#include "messages.hpp"

#include <string>
#include <vector>

namespace sfp {

struct PipelineConfig {
    std::string               name;
    EstimatorConfig           estimator;
    std::vector<SensorConfig> sensors;
    std::vector<SensorType>   log_channels;  // CSV columns, in order
    double                    duration_s;
};

// Battery / EV context: a pack monitor fusing two redundant temperature
// probes, a voltage sensor, and a current sensor. The two temperature
// probes are the case where inverse-variance fusion does real work —
// they measure the same physical quantity with different noise, and the
// estimator combines them optimally.
inline PipelineConfig battery_config(double duration_s = 30.0,
                                     int spin_us = 50) {
    PipelineConfig cfg;
    cfg.name      = "battery";
    cfg.estimator = EstimatorConfig{/*loop_hz=*/200.0, /*spin_us=*/spin_us};
    cfg.sensors = {
        // Two temperature probes on the same cell group: 40 C, slowly
        // warming, different noise characteristics.
        make_sensor(0, SensorType::Temperature, /*hz=*/100.0,
                    /*truth=*/40.0, /*drift=*/0.05, /*sigma=*/0.8),
        make_sensor(1, SensorType::Temperature, /*hz=*/100.0,
                    /*truth=*/40.0, /*drift=*/0.05, /*sigma=*/0.4),
        // Pack voltage: ~400 V nominal, low noise, slight sag under load.
        make_sensor(2, SensorType::Voltage, /*hz=*/500.0,
                    /*truth=*/400.0, /*drift=*/-0.2, /*sigma=*/0.5),
        // Pack current: ~120 A draw, noisier (switching ripple).
        make_sensor(3, SensorType::Current, /*hz=*/1000.0,
                    /*truth=*/120.0, /*drift=*/0.0, /*sigma=*/2.0),
    };
    cfg.log_channels = {SensorType::Temperature, SensorType::Voltage,
                        SensorType::Current};
    cfg.duration_s = duration_s;
    return cfg;
}

// Robotics context: one joint of a robot arm. A high-rate encoder
// reports position, a tachometer reports velocity, and a load cell
// reports force at the end effector. Position and velocity are the
// pair a complementary filter would blend (a later module); for now
// they're fused per-channel.
inline PipelineConfig robotics_config(double duration_s = 30.0,
                                      int spin_us = 50) {
    PipelineConfig cfg;
    cfg.name      = "robotics";
    cfg.estimator = EstimatorConfig{/*loop_hz=*/200.0, /*spin_us=*/spin_us};
    // Turn on the complementary filter: blend encoder position with the
    // velocity integral. alpha = 0.05 → time constant ~95 ms at 5 ms dt.
    cfg.estimator.use_complementary_filter = true;
    cfg.estimator.complementary_alpha      = 0.05;
    // Process noise for the optional Kalman filter (--kalman swaps it in
    // for the complementary filter). Low q: the joint is near
    // constant-velocity, so trust the model and smooth hard.
    cfg.estimator.kalman_process_noise     = 1.0;
    cfg.sensors = {
        // Joint encoder: position in radians, sweeping, low noise.
        make_sensor(0, SensorType::Position, /*hz=*/1000.0,
                    /*truth=*/0.0, /*drift=*/0.5, /*sigma=*/0.002),
        // Redundant secondary encoder, noisier.
        make_sensor(1, SensorType::Position, /*hz=*/500.0,
                    /*truth=*/0.0, /*drift=*/0.5, /*sigma=*/0.01),
        // Tachometer: velocity rad/s, roughly constant during the sweep.
        make_sensor(2, SensorType::Velocity, /*hz=*/1000.0,
                    /*truth=*/0.5, /*drift=*/0.0, /*sigma=*/0.02),
        // End-effector load cell: force in newtons.
        make_sensor(3, SensorType::Force, /*hz=*/500.0,
                    /*truth=*/12.0, /*drift=*/0.0, /*sigma=*/0.3),
    };
    cfg.log_channels = {SensorType::Position, SensorType::Velocity,
                        SensorType::Force};
    cfg.duration_s = duration_s;
    return cfg;
}

} // namespace sfp
