// kalman.hpp — a minimal 2-state (position, velocity) Kalman filter with
// a constant-velocity motion model.
//
// This is the "coupled state" estimator the README flags as the natural
// step beyond the per-channel inverse-variance average and the fixed-gain
// complementary filter. Unlike those, it carries a full 2x2 covariance,
// so the position and velocity estimates are statistically coupled: a
// velocity measurement sharpens the position estimate (and vice versa)
// through the off-diagonal covariance term, and every measurement is
// weighted by its own uncertainty automatically rather than by a hand-set
// blend constant.
//
// Standard library only — no Eigen. The state is 2-D and every
// measurement is scalar (a position reading or a velocity reading), so
// the innovation covariance S is a scalar and the Kalman gain is a 2x1
// vector computed with a single division: no matrix inversion, no
// allocation, just a handful of explicit floating-point expressions on
// the estimator's hot path.
//
// Model:
//   state    x = [p, v]
//   predict  x <- F x,   F = [[1, dt], [0, 1]]        (constant velocity)
//            P <- F P F^T + Q
//   Q is the standard continuous-white-noise-acceleration discretization
//   with spectral density q:
//            Q = q * [[dt^3/3, dt^2/2], [dt^2/2, dt]]
//   update   scalar z with variance R; H = [1,0] for a position
//            measurement, [0,1] for a velocity measurement:
//            y = z - H x;   S = H P H^T + R;   K = P H^T / S
//            x <- x + K y;  P <- (I - K H) P
//
// P is symmetric, so only p00_, p01_, p11_ are stored. The scalar
// updates preserve symmetry exactly (checked in test_kalman).

#pragma once

namespace sfp {

class KalmanFilter2 {
public:
    // process_noise is the spectral density q of the white-noise
    // acceleration driving the constant-velocity model: larger q lets the
    // filter trust measurements more and adapt faster, smaller q makes it
    // smoother and more sluggish. p0_var is the (deliberately large)
    // initial variance on both states — "we don't know the state yet", so
    // the first measurements dominate.
    explicit KalmanFilter2(double process_noise = 1.0,
                           double p0_var = 1.0e3) noexcept
        : q_(process_noise)
        , p00_(p0_var)
        , p11_(p0_var)
    {}

    void reset(double p0, double v0, double p0_var = 1.0e3) noexcept {
        p_   = p0;
        v_   = v0;
        p00_ = p0_var;
        p01_ = 0.0;
        p11_ = p0_var;
    }

    // Advance state and covariance by dt seconds. Non-positive dt is a
    // no-op (nothing has elapsed).
    void predict(double dt) noexcept {
        if (dt <= 0.0) return;

        // State: p += v*dt, v unchanged.
        p_ += v_ * dt;

        // Covariance: P <- F P F^T + Q, expanded for F = [[1,dt],[0,1]]
        // and symmetric P (p10 == p01).
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double new_p00 =
            p00_ + 2.0 * dt * p01_ + dt2 * p11_ + q_ * dt3 / 3.0;
        const double new_p01 = p01_ + dt * p11_ + q_ * dt2 / 2.0;
        const double new_p11 = p11_ + q_ * dt;
        p00_ = new_p00;
        p01_ = new_p01;
        p11_ = new_p11;
    }

    // Fuse a scalar position measurement with variance R (H = [1, 0]).
    void update_position(double z, double R) noexcept {
        const double S = p00_ + R;
        if (S <= 0.0) return;
        const double k0 = p00_ / S;   // gain, position row
        const double k1 = p01_ / S;   // gain, velocity row
        const double y  = z - p_;     // innovation
        p_ += k0 * y;
        v_ += k1 * y;
        // P <- (I - K H) P with H = [1, 0], using the pre-update entries.
        const double old_p00 = p00_;
        const double old_p01 = p01_;
        p00_ = old_p00 - k0 * old_p00;
        p01_ = old_p01 - k0 * old_p01;
        p11_ = p11_    - k1 * old_p01;
    }

    // Fuse a scalar velocity measurement with variance R (H = [0, 1]).
    void update_velocity(double z, double R) noexcept {
        const double S = p11_ + R;
        if (S <= 0.0) return;
        const double k0 = p01_ / S;   // gain, position row
        const double k1 = p11_ / S;   // gain, velocity row
        const double y  = z - v_;
        p_ += k0 * y;
        v_ += k1 * y;
        // P <- (I - K H) P with H = [0, 1], using the pre-update entries.
        const double old_p01 = p01_;
        const double old_p11 = p11_;
        p00_ = p00_    - k0 * old_p01;
        p01_ = old_p01 - k0 * old_p11;
        p11_ = old_p11 - k1 * old_p11;
    }

    double position() const noexcept { return p_; }
    double velocity() const noexcept { return v_; }

    // Covariance accessors, for diagnostics and tests.
    double var_position() const noexcept { return p00_; }
    double var_velocity() const noexcept { return p11_; }
    double covariance()   const noexcept { return p01_; }

private:
    double q_   = 1.0;     // process-noise spectral density
    double p_   = 0.0;     // position estimate
    double v_   = 0.0;     // velocity estimate
    double p00_ = 1.0e3;   // var(position)
    double p01_ = 0.0;     // cov(position, velocity)
    double p11_ = 1.0e3;   // var(velocity)
};

} // namespace sfp
