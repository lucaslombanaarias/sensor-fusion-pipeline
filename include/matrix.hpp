// matrix.hpp — tiny fixed-size dense matrix for the EKF.
//
// The 2-state Kalman filter (kalman.hpp) hand-codes its 2x2 algebra. The
// 5-state EKF (ekf.hpp) needs 5x5 / 5x2 / 2x5 / 2x2 products, transposes,
// and one 2x2 inverse — too much to spell out by hand without mistakes,
// but far too little to justify pulling in Eigen. This is a header-only
// Mat<R,C> with compile-time dimensions: dimension mismatches are
// caught by the type system, every size is on the stack, and there is no
// allocation. Standard library only, in keeping with the rest of the
// project.

#pragma once

#include <array>
#include <cstddef>

namespace sfp {

template <std::size_t R, std::size_t C>
struct Mat {
    std::array<double, R * C> d{};

    double& operator()(std::size_t i, std::size_t j) noexcept { return d[i * C + j]; }
    double  operator()(std::size_t i, std::size_t j) const noexcept { return d[i * C + j]; }

    // Matrix product: (R x C) * (C x K) = (R x K).
    template <std::size_t K>
    Mat<R, K> operator*(const Mat<C, K>& o) const noexcept {
        Mat<R, K> out;
        for (std::size_t i = 0; i < R; ++i)
            for (std::size_t k = 0; k < K; ++k) {
                double s = 0.0;
                for (std::size_t j = 0; j < C; ++j) s += (*this)(i, j) * o(j, k);
                out(i, k) = s;
            }
        return out;
    }

    Mat operator+(const Mat& o) const noexcept {
        Mat out;
        for (std::size_t i = 0; i < R * C; ++i) out.d[i] = d[i] + o.d[i];
        return out;
    }
    Mat operator-(const Mat& o) const noexcept {
        Mat out;
        for (std::size_t i = 0; i < R * C; ++i) out.d[i] = d[i] - o.d[i];
        return out;
    }

    Mat<C, R> transpose() const noexcept {
        Mat<C, R> out;
        for (std::size_t i = 0; i < R; ++i)
            for (std::size_t j = 0; j < C; ++j) out(j, i) = (*this)(i, j);
        return out;
    }

    static Mat identity() noexcept {
        static_assert(R == C, "identity() requires a square matrix");
        Mat out;
        for (std::size_t i = 0; i < R; ++i) out(i, i) = 1.0;
        return out;
    }
};

// Closed-form 2x2 inverse — the only inverse the EKF needs, since every
// measurement update here is 2-dimensional (a GPS x/y fix), so the
// innovation covariance S is 2x2.
inline Mat<2, 2> inverse2(const Mat<2, 2>& m) noexcept {
    const double det = m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
    const double inv = (det != 0.0) ? 1.0 / det : 0.0;
    Mat<2, 2> out;
    out(0, 0) =  m(1, 1) * inv;
    out(0, 1) = -m(0, 1) * inv;
    out(1, 0) = -m(1, 0) * inv;
    out(1, 1) =  m(0, 0) * inv;
    return out;
}

} // namespace sfp
