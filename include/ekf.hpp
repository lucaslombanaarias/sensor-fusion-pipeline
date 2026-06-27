// ekf.hpp — 2-D Extended Kalman Filter for vehicle localization, fusing
// dead-reckoning (forward speed + gyro yaw-rate) with GPS position fixes.
//
// This is the genuinely-nonlinear, genuinely-coupled estimator the
// project builds toward. Where kalman.hpp is a *linear* constant-velocity
// filter, this one carries heading in the state and propagates motion
// through a unicycle model — the position update rotates the forward
// velocity by the heading, so cos/sin of a state variable appear in the
// dynamics. The Jacobian of that nonlinear model, recomputed every step,
// is what makes it an *Extended* KF.
//
// This is the textbook vehicle-localization EKF (Thrun/Burgard/Fox,
// "Probabilistic Robotics"): high-rate dead-reckoning that drifts without
// correction, anchored by lower-rate, noisier GPS that has no drift. The
// filter fuses them into a trajectory more accurate than either alone.
//
// State (3):   x = [ px, py, psi ]   position (m, local), heading (rad)
//
// Prediction, inputs v = forward speed, w = yaw rate (gyro), over dt:
//   px'  = px + v*cos(psi)*dt
//   py'  = py + v*sin(psi)*dt
//   psi' = psi + w*dt
//   P'   = F P F^T + Q,   Q = G diag(sig_v^2, sig_w^2) G^T
// with F = d f / d x and G = d f / d u formed analytically below.
//
// Update (GPS x/y fix, measurement noise sigma):
//   z = [px,py]+noise,  H = [[1,0,0],[0,1,0]]
//   y = z - H x;  S = H P H^T + R;  K = P H^T S^-1
//   x += K y;     P = (I - K H) P
//
// Standard library only; fixed-size linear algebra (matrix.hpp), no
// allocation. Validated on a synthetic trajectory in test_ekf and, end to
// end, on a real KITTI drive (apps/ekf_localization.cpp).

#pragma once

#include "matrix.hpp"

#include <cmath>

namespace sfp {

class Ekf2D {
public:
    // sigma_speed / sigma_yawrate are the dead-reckoning input noise
    // densities driving the process-noise matrix Q; p0_var is the initial
    // state uncertainty.
    Ekf2D(double sigma_speed = 0.3, double sigma_yawrate = 0.02,
          double p0_var = 1.0) noexcept
        : sigma_speed_(sigma_speed)
        , sigma_yawrate_(sigma_yawrate)
    {
        for (std::size_t i = 0; i < 3; ++i) P_(i, i) = p0_var;
    }

    void init(double px, double py, double psi) noexcept {
        x_(0, 0) = px; x_(1, 0) = py; x_(2, 0) = psi;
    }

    // Dead-reckoning prediction from forward speed and yaw rate.
    void predict(double speed, double yaw_rate, double dt) noexcept {
        if (dt <= 0.0) return;
        const double psi = x_(2, 0);
        const double c = std::cos(psi), s = std::sin(psi);

        x_(0, 0) += speed * c * dt;
        x_(1, 0) += speed * s * dt;
        x_(2, 0)  = psi + yaw_rate * dt;

        // Jacobian F = d f / d x at the previous state.
        Mat<3, 3> F = Mat<3, 3>::identity();
        F(0, 2) = -speed * s * dt;
        F(1, 2) =  speed * c * dt;

        // Control Jacobian G = d f / d u, u = [speed, yaw_rate].
        Mat<3, 2> G{};
        G(0, 0) = c * dt;
        G(1, 0) = s * dt;
        G(2, 1) = dt;

        Mat<2, 2> Qu{};
        Qu(0, 0) = sigma_speed_   * sigma_speed_;
        Qu(1, 1) = sigma_yawrate_ * sigma_yawrate_;

        P_ = F * P_ * F.transpose() + G * Qu * G.transpose();
    }

    // GPS position update (x, y) with isotropic measurement stddev sigma.
    void update_gps(double zx, double zy, double sigma) noexcept {
        Mat<2, 3> H{};
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;

        Mat<2, 1> z;
        z(0, 0) = zx; z(1, 0) = zy;
        const Mat<2, 1> y = z - H * x_;            // innovation

        Mat<2, 2> R{};
        R(0, 0) = sigma * sigma;
        R(1, 1) = sigma * sigma;

        const Mat<2, 2> S = H * P_ * H.transpose() + R;
        const Mat<3, 2> K = P_ * H.transpose() * inverse2(S);

        x_ = x_ + K * y;
        const Mat<3, 3> I = Mat<3, 3>::identity();
        P_ = (I - K * H) * P_;
    }

    double x()   const noexcept { return x_(0, 0); }
    double y()   const noexcept { return x_(1, 0); }
    double psi() const noexcept { return x_(2, 0); }
    double var_x() const noexcept { return P_(0, 0); }
    double var_y() const noexcept { return P_(1, 1); }

private:
    double    sigma_speed_;
    double    sigma_yawrate_;
    Mat<3, 1> x_{};
    Mat<3, 3> P_{};
};

} // namespace sfp
