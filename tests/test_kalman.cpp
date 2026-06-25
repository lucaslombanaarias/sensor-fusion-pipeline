// test_kalman.cpp — unit tests for the 2-state constant-velocity Kalman
// filter (include/kalman.hpp), in isolation from the threaded pipeline.
//
// These verify the four properties that matter:
//   1. Predict-only coasting advances position by velocity*dt.
//   2. Repeated measurements shrink the state covariance (information
//      accumulates) while the estimate tracks truth.
//   3. The coupled covariance lets the filter *infer velocity from a
//      position ramp* — something the per-channel average and the
//      complementary filter cannot do.
//   4. The scalar update equations keep the covariance a valid
//      (symmetric, positive-semidefinite) matrix.

#include "kalman.hpp"

#include <cmath>
#include <iostream>

namespace {

// Coasting: with no measurements, predict() integrates velocity.
bool test_coasting() {
    sfp::KalmanFilter2 kf(1.0);
    kf.reset(/*p0=*/2.0, /*v0=*/0.5);

    const double dt = 0.01;
    for (int i = 0; i < 100; ++i) kf.predict(dt);  // 1.0 s total

    const double expected = 2.0 + 0.5 * 1.0;  // p0 + v*t
    if (std::abs(kf.position() - expected) > 1e-9) {
        std::cerr << "  FAIL: coasted position " << kf.position()
                  << " != expected " << expected << '\n';
        return false;
    }
    if (std::abs(kf.velocity() - 0.5) > 1e-9) {
        std::cerr << "  FAIL: velocity drifted to " << kf.velocity() << '\n';
        return false;
    }
    std::cout << "  PASS: coasted to position " << kf.position()
              << " (expected " << expected << "), velocity held.\n";
    return true;
}

// Repeated position measurements shrink var(position) and the estimate
// hugs the (constant) truth.
bool test_information_accumulates() {
    sfp::KalmanFilter2 kf(/*process_noise=*/0.01);
    kf.reset(0.0, 0.0, /*p0_var=*/1.0e3);

    const double truth = 7.5;
    const double R     = 0.04;   // sigma = 0.2
    const double dt    = 0.005;

    const double var_before = kf.var_position();
    for (int i = 0; i < 500; ++i) {
        kf.predict(dt);
        kf.update_position(truth, R);
    }
    const double var_after = kf.var_position();

    if (var_after >= var_before || var_after >= R) {
        std::cerr << "  FAIL: var(position) did not shrink below R: before="
                  << var_before << " after=" << var_after << " R=" << R << '\n';
        return false;
    }
    if (std::abs(kf.position() - truth) > 0.05) {
        std::cerr << "  FAIL: estimate " << kf.position()
                  << " too far from truth " << truth << '\n';
        return false;
    }
    std::cout << "  PASS: var(position) " << var_before << " -> " << var_after
              << " (< R=" << R << "), estimate=" << kf.position() << '\n';
    return true;
}

// Coupled state: feed only POSITION measurements on a constant-velocity
// ramp; the filter should infer the VELOCITY through the off-diagonal
// covariance term, with no velocity measurement at all.
bool test_velocity_inferred_from_position() {
    sfp::KalmanFilter2 kf(/*process_noise=*/0.1);
    kf.reset(0.0, 0.0, 1.0e3);

    const double v_true = 2.0;
    const double dt     = 0.005;
    const double R      = 1.0e-4;  // clean encoder
    double t = 0.0;
    for (int i = 0; i < 3000; ++i) {  // 15 s
        t += dt;
        kf.predict(dt);
        kf.update_position(v_true * t, R);  // exact ramp, position only
    }

    if (std::abs(kf.velocity() - v_true) > 0.05) {
        std::cerr << "  FAIL: inferred velocity " << kf.velocity()
                  << " != true " << v_true << '\n';
        return false;
    }
    std::cout << "  PASS: velocity inferred from position-only ramp = "
              << kf.velocity() << " (true " << v_true << ").\n";
    return true;
}

// The update equations must keep P symmetric and positive-semidefinite.
bool test_covariance_valid() {
    sfp::KalmanFilter2 kf(0.5);
    kf.reset(0.0, 0.0, 1.0e3);

    const double dt = 0.005;
    bool ok = true;
    for (int i = 0; i < 1000; ++i) {
        kf.predict(dt);
        // Alternate the two measurement types to exercise both updates.
        if (i % 2 == 0) kf.update_position(0.5 * static_cast<double>(i) * dt, 1e-4);
        else            kf.update_velocity(0.5, 4e-4);

        const double p00 = kf.var_position();
        const double p11 = kf.var_velocity();
        const double p01 = kf.covariance();
        const double det = p00 * p11 - p01 * p01;
        // Variances strictly positive; determinant non-negative within
        // a small floating-point tolerance => positive-semidefinite.
        if (p00 <= 0.0 || p11 <= 0.0 || det < -1e-9) {
            std::cerr << "  FAIL: invalid covariance at step " << i
                      << " p00=" << p00 << " p11=" << p11
                      << " p01=" << p01 << " det=" << det << '\n';
            ok = false;
            break;
        }
    }
    if (ok) {
        std::cout << "  PASS: covariance stayed symmetric and "
                     "positive-semidefinite over 1000 steps.\n";
    }
    return ok;
}

} // namespace

int main() {
    std::cout << "Test 1: predict-only coasting integrates velocity\n";
    if (!test_coasting()) return 1;

    std::cout << "Test 2: repeated measurements accumulate information\n";
    if (!test_information_accumulates()) return 1;

    std::cout << "Test 3: velocity inferred from position-only ramp\n";
    if (!test_velocity_inferred_from_position()) return 1;

    std::cout << "Test 4: covariance stays valid (symmetric, PSD)\n";
    if (!test_covariance_valid()) return 1;

    std::cout << "\nAll Kalman filter tests passed.\n";
    return 0;
}
