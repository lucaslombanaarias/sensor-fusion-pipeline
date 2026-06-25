// config.hpp — pipeline configuration structures.
//
// Plain aggregates; brace-initialize them in configs/battery.cpp,
// configs/robotics.cpp, etc.

#pragma once

#include "messages.hpp"

#include <string>
#include <vector>

namespace sfp {

struct SensorConfig {
    int        sensor_id;       // unique within the pipeline
    SensorType type;            // which physical channel this sensor reports

    // --- Simulation parameters ------------------------------------------
    // Ground truth value at t=0. Drift moves it linearly over time so the
    // fusion has something non-trivial to track. With drift_rate = 0 the
    // truth is constant and the estimator just smooths noise.
    double     publish_hz;
    double     true_value;
    double     drift_rate;      // units per second
    double     noise_stddev;    // gaussian sigma on each reading

    // --- Fusion parameter -----------------------------------------------
    // Used by the estimator. For Bayesian-optimal inverse-variance
    // weighting this should be 1.0 / (noise_stddev * noise_stddev), but
    // it's configurable so we can experiment with deliberately-wrong
    // weights and see the effect on the estimate.
    double     fusion_weight;
};

// Convenience: build a SensorConfig with the optimal inverse-variance
// weight pre-computed, so callers don't have to spell out the math.
inline SensorConfig make_sensor(int id, SensorType type, double hz,
                                double truth, double drift, double sigma) {
    const double w = (sigma > 0.0) ? 1.0 / (sigma * sigma) : 1.0;
    return SensorConfig{id, type, hz, truth, drift, sigma, w};
}

struct EstimatorConfig {
    double loop_hz;          // e.g. 200.0
    int    spin_wait_us;     // sub-deadline busy-wait window in
                             // microseconds. 0 disables; 50-100 gives
                             // sub-10us deadline accuracy at the cost
                             // of a few percent CPU on the estimator
                             // core. The right answer for real-time
                             // loops on a non-RT kernel.

    // --- Complementary filter (robotics context) -----------------------
    // When enabled, the Position channel is blended with the integral of
    // the Velocity channel:
    //
    //   predicted = prev_position + velocity * dt
    //   position  = alpha * measured_position + (1 - alpha) * predicted
    //
    // This is the textbook complementary filter: trust the encoder
    // (alpha) for low-frequency accuracy, trust integrated velocity
    // (1 - alpha) for high-frequency smoothness, and — crucially — keep
    // producing a position estimate via pure integration on ticks where
    // no fresh encoder reading arrived. A small alpha (~0.02) gives a
    // time constant of dt*(1-alpha)/alpha; at dt=5ms, alpha=0.02 that's
    // ~245 ms. Ignored unless both a Position and a Velocity channel
    // exist. Defaults below leave it off so the battery config is
    // unaffected.
    bool   use_complementary_filter = false;
    double complementary_alpha      = 0.02;

    // --- Kalman filter (robotics context) -------------------------------
    // When enabled, the Position and Velocity channels are estimated
    // jointly by a 2-state constant-velocity Kalman filter (see
    // kalman.hpp) instead of the per-channel average / complementary
    // filter. The filter carries a full covariance, so the two channels
    // are statistically coupled and each measurement is weighted by its
    // own noise. kalman_process_noise is the acceleration spectral
    // density q: larger tracks faster, smaller smooths more. Takes
    // precedence over the complementary filter when both are set.
    // Ignored unless a Position or Velocity channel exists.
    bool   use_kalman_filter   = false;
    double kalman_process_noise = 1.0;
};

} // namespace sfp
