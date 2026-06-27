// test_ekf.cpp — unit tests for the 2-D IMU/GPS Extended Kalman Filter
// (include/ekf.hpp) and the small matrix library it rests on
// (include/matrix.hpp). Runs on a synthetic trajectory, so it needs no
// dataset and is safe for CI; the same filter is exercised end to end on
// real KITTI data by apps/ekf_localization.

#include "ekf.hpp"
#include "matrix.hpp"

#include <cmath>
#include <iostream>
#include <random>

namespace {

// --- matrix.hpp sanity ---------------------------------------------------
bool test_matrix() {
    sfp::Mat<2, 3> a{};
    a(0, 0) = 1; a(0, 1) = 2; a(0, 2) = 3;
    a(1, 0) = 4; a(1, 1) = 5; a(1, 2) = 6;
    sfp::Mat<3, 2> b{};
    b(0, 0) = 7;  b(0, 1) = 8;
    b(1, 0) = 9;  b(1, 1) = 10;
    b(2, 0) = 11; b(2, 1) = 12;
    const sfp::Mat<2, 2> c = a * b;   // [[58,64],[139,154]]
    if (c(0, 0) != 58 || c(0, 1) != 64 || c(1, 0) != 139 || c(1, 1) != 154) {
        std::cerr << "  FAIL: matmul wrong\n";
        return false;
    }
    // Inverse round-trips to identity.
    sfp::Mat<2, 2> m{};
    m(0, 0) = 4; m(0, 1) = 7; m(1, 0) = 2; m(1, 1) = 6;
    const sfp::Mat<2, 2> id = m * sfp::inverse2(m);
    if (std::abs(id(0, 0) - 1) > 1e-9 || std::abs(id(1, 1) - 1) > 1e-9 ||
        std::abs(id(0, 1)) > 1e-9 || std::abs(id(1, 0)) > 1e-9) {
        std::cerr << "  FAIL: 2x2 inverse wrong\n";
        return false;
    }
    std::cout << "  PASS: matmul and 2x2 inverse correct.\n";
    return true;
}

// --- prediction: straight line, no noise --------------------------------
bool test_predict_straight() {
    sfp::Ekf2D ekf;
    ekf.init(0.0, 0.0, 0.0);          // heading east
    const double v = 5.0, dt = 0.1;
    for (int i = 0; i < 100; ++i) ekf.predict(v, 0.0, dt);  // 10 s
    if (std::abs(ekf.x() - 50.0) > 1e-6 || std::abs(ekf.y()) > 1e-6) {
        std::cerr << "  FAIL: straight-line predict x=" << ekf.x()
                  << " y=" << ekf.y() << " (expected 50, 0)\n";
        return false;
    }
    std::cout << "  PASS: straight-line dead-reckoning x=" << ekf.x() << ".\n";
    return true;
}

// --- update: a GPS fix shrinks covariance and pulls the estimate --------
bool test_update_corrects() {
    sfp::Ekf2D ekf(0.3, 0.02, /*p0_var=*/100.0);
    ekf.init(0.0, 0.0, 0.0);
    const double var_before = ekf.var_x();
    ekf.update_gps(10.0, 0.0, /*sigma=*/1.0);
    if (ekf.var_x() >= var_before) {
        std::cerr << "  FAIL: covariance did not shrink\n";
        return false;
    }
    if (ekf.x() < 9.0) {  // large prior var -> snaps most of the way to 10
        std::cerr << "  FAIL: estimate " << ekf.x() << " not pulled to fix\n";
        return false;
    }
    std::cout << "  PASS: GPS fix pulled estimate to " << ekf.x()
              << ", var " << var_before << " -> " << ekf.var_x() << ".\n";
    return true;
}

// --- the headline test: on a noisy circular drive, the fused EKF beats
//     both raw GPS and dead-reckoning-only ----------------------------
bool test_fusion_beats_baselines() {
    const double v = 8.0;          // m/s
    const double omega = 0.10;     // rad/s -> ~80 m radius circle
    const double dt = 0.05;        // 20 Hz
    const int    steps = 3000;     // 150 s, ~2.4 laps
    const int    gps_every = 20;   // 1 Hz
    const double gps_sigma = 1.5;

    std::mt19937 rng(7);
    std::normal_distribution<double> gps_n(0.0, gps_sigma);
    std::normal_distribution<double> spd_n(0.0, 0.2);
    std::normal_distribution<double> gyr_n(0.0, 0.01);
    const double gyro_bias = 0.005;

    sfp::Ekf2D ekf(0.3, 0.02);
    double tx = 0.0, ty = 0.0, tpsi = 0.0;     // truth
    double dx = 0.0, dy = 0.0, dpsi = 0.0;     // dead-reckoning only
    ekf.init(0.0, 0.0, 0.0);

    double ss_ekf = 0.0, ss_gps = 0.0, ss_dr = 0.0;
    int n = 0, ng = 0;

    for (int i = 1; i <= steps; ++i) {
        // Advance ground truth exactly.
        tx += v * std::cos(tpsi) * dt;
        ty += v * std::sin(tpsi) * dt;
        tpsi += omega * dt;

        // Noisy dead-reckoning inputs (shared by EKF predict and DR-only).
        const double vm = v + spd_n(rng);
        const double wm = omega + gyro_bias + gyr_n(rng);
        ekf.predict(vm, wm, dt);
        dx += vm * std::cos(dpsi) * dt;
        dy += vm * std::sin(dpsi) * dt;
        dpsi += wm * dt;

        if (i % gps_every == 0) {
            const double gx = tx + gps_n(rng), gy = ty + gps_n(rng);
            ekf.update_gps(gx, gy, gps_sigma);
            ss_gps += (gx - tx) * (gx - tx) + (gy - ty) * (gy - ty);
            ++ng;
        }
        ss_ekf += (ekf.x() - tx) * (ekf.x() - tx) + (ekf.y() - ty) * (ekf.y() - ty);
        ss_dr  += (dx - tx) * (dx - tx) + (dy - ty) * (dy - ty);
        ++n;
    }

    const double rmse_ekf = std::sqrt(ss_ekf / n);
    const double rmse_gps = std::sqrt(ss_gps / ng);
    const double rmse_dr  = std::sqrt(ss_dr / n);

    if (!(rmse_ekf < rmse_gps && rmse_ekf < rmse_dr)) {
        std::cerr << "  FAIL: fusion not best — ekf=" << rmse_ekf
                  << " gps=" << rmse_gps << " dr=" << rmse_dr << '\n';
        return false;
    }
    std::cout << "  PASS: RMSE ekf=" << rmse_ekf << " m < gps=" << rmse_gps
              << " m, dead-reckoning=" << rmse_dr << " m (drifts).\n";
    return true;
}

} // namespace

int main() {
    std::cout << "Test 1: fixed-size matrix algebra\n";
    if (!test_matrix()) return 1;
    std::cout << "Test 2: straight-line prediction\n";
    if (!test_predict_straight()) return 1;
    std::cout << "Test 3: GPS update corrects and shrinks covariance\n";
    if (!test_update_corrects()) return 1;
    std::cout << "Test 4: fused EKF beats GPS-only and dead-reckoning\n";
    if (!test_fusion_beats_baselines()) return 1;
    std::cout << "\nAll EKF tests passed.\n";
    return 0;
}
