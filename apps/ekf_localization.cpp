// ekf_localization.cpp — run the IMU+GPS EKF (include/ekf.hpp) over a
// real KITTI raw "OXTS" sequence and report how much the fusion beats raw
// GPS.
//
// KITTI's OXTS stream is an RTK-grade INS solution, so we treat it as
// ground truth: the IMU channels (forward acceleration, yaw rate) drive
// the EKF's prediction, and a *simulated* consumer-grade GPS — the true
// position corrupted with Gaussian noise and downsampled to ~1 Hz —
// drives the updates. That is the standard way to evaluate a
// loosely-coupled INS/GPS filter on this dataset: real trajectory, real
// inertial data, a GPS whose noise we know so the baseline is honest.
//
// Usage:
//   ekf_localization <oxts_dir> [out.csv] [gps_period] [gps_sigma_m]
//
//   <oxts_dir>     directory containing oxts/data/*.txt and
//                  oxts/timestamps.txt (see scripts/fetch_kitti_oxts.py)
//   gps_period     emit a GPS fix every Nth frame (default 10 ≈ 1 Hz)
//   gps_sigma_m    GPS noise stddev in metres (default 1.5)
//
// Prints GPS-only vs EKF position RMSE and writes a CSV (truth / gps /
// ekf x,y) for scripts/plot_trajectory.py.

#include "ekf.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

constexpr double kPi   = 3.14159265358979323846;
constexpr double kEarth = 6378137.0;  // WGS-84 equatorial radius (m)

struct Oxts {
    double lat, lon, yaw, vf, af, yaw_rate;
};

// KITTI Mercator projection: lat/lon (deg) -> local metres, with the
// scale fixed at the first latitude (matches the KITTI devkit).
struct Mercator {
    double scale = 1.0, x0 = 0.0, y0 = 0.0;
    bool   init = false;
    void project(double lat, double lon, double& x, double& y) {
        if (!init) { scale = std::cos(lat * kPi / 180.0); init = true; }
        const double mx = scale * lon * kPi / 180.0 * kEarth;
        const double my = scale * kEarth *
            std::log(std::tan((90.0 + lat) * kPi / 360.0));
        if (x0 == 0.0 && y0 == 0.0) { x0 = mx; y0 = my; }
        x = mx - x0;
        y = my - y0;
    }
};

double seconds_of_day(const std::string& ts) {
    // "2011-09-30 13:12:10.525111255" -> seconds since midnight.
    const auto sp = ts.find(' ');
    if (sp == std::string::npos) return 0.0;
    int h = 0, m = 0; double s = 0.0;
    std::sscanf(ts.c_str() + sp + 1, "%d:%d:%lf", &h, &m, &s);
    return h * 3600.0 + m * 60.0 + s;
}

bool load_sequence(const std::string& dir,
                   std::vector<Oxts>& frames,
                   std::vector<double>& times) {
    const fs::path data = fs::path(dir) / "oxts" / "data";
    const fs::path tsf  = fs::path(dir) / "oxts" / "timestamps.txt";
    if (!fs::exists(data) || !fs::exists(tsf)) {
        std::fprintf(stderr, "error: %s missing oxts/data or timestamps\n",
                     dir.c_str());
        return false;
    }

    std::vector<fs::path> files;
    for (const auto& e : fs::directory_iterator(data))
        if (e.path().extension() == ".txt") files.push_back(e.path());
    std::sort(files.begin(), files.end());

    for (const auto& p : files) {
        std::ifstream f(p);
        std::vector<double> v;
        double x;
        while (f >> x) v.push_back(x);
        if (v.size() < 23) continue;
        // Field indices per KITTI oxts/dataformat.txt.
        Oxts o;
        o.lat = v[0]; o.lon = v[1]; o.yaw = v[5];
        o.vf = v[8]; o.af = v[14]; o.yaw_rate = v[22];  // wu: yaw rate (up axis)
        frames.push_back(o);
    }

    std::ifstream tf(tsf);
    std::string line;
    while (std::getline(tf, line))
        if (!line.empty()) times.push_back(seconds_of_day(line));

    const std::size_t n = std::min(frames.size(), times.size());
    frames.resize(n);
    times.resize(n);
    return n > 1;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
            "usage: %s <oxts_dir> [out.csv] [gps_period] [gps_sigma_m]\n",
            argv[0]);
        return 2;
    }
    const std::string dir = argv[1];
    const std::string csv = (argc > 2) ? argv[2] : "kitti_ekf.csv";
    const int    gps_period = (argc > 3) ? std::atoi(argv[3]) : 10;
    const double gps_sigma  = (argc > 4) ? std::atof(argv[4]) : 1.5;

    std::vector<Oxts> frames;
    std::vector<double> times;
    if (!load_sequence(dir, frames, times)) return 1;

    Mercator merc;
    sfp::Ekf2D ekf(/*sigma_speed=*/0.3, /*sigma_yawrate=*/0.02);

    // Simulated sensor errors on the real trajectory: a consumer GPS with
    // Gaussian noise, plus dead-reckoning inputs corrupted by gyro bias +
    // noise and speed noise. (KITTI's own INS is too clean to need fusing,
    // so we inject realistic error and let the EKF earn its keep.)
    std::mt19937 rng(42);
    std::normal_distribution<double> gps_noise(0.0, gps_sigma);
    std::normal_distribution<double> speed_noise(0.0, 0.2);
    std::normal_distribution<double> gyro_noise(0.0, 0.01);
    const double gyro_bias = 0.006;  // rad/s — a steady gyro drift

    std::ofstream out(csv);
    out << "t,truth_x,truth_y,gps_x,gps_y,ekf_x,ekf_y,dr_x,dr_y\n";

    // Dead-reckoning-only pose (same noisy inputs, never sees GPS).
    double dr_x = 0.0, dr_y = 0.0, dr_psi = 0.0;

    double sum_sq_ekf = 0.0, sum_sq_gps = 0.0, sum_sq_dr = 0.0;
    std::size_t n_ekf = 0, n_gps = 0;
    const double t0 = times.front();

    for (std::size_t i = 0; i < frames.size(); ++i) {
        const Oxts& o = frames[i];
        double tx, ty;
        merc.project(o.lat, o.lon, tx, ty);

        if (i == 0) {
            ekf.init(tx, ty, o.yaw);
            dr_x = tx; dr_y = ty; dr_psi = o.yaw;
        } else {
            const double dt = times[i] - times[i - 1];
            const double v = o.vf + speed_noise(rng);
            const double w = o.yaw_rate + gyro_bias + gyro_noise(rng);
            ekf.predict(v, w, dt);
            // Same inputs, no correction — shows the drift GPS fixes cure.
            dr_x += v * std::cos(dr_psi) * dt;
            dr_y += v * std::sin(dr_psi) * dt;
            dr_psi += w * dt;
        }

        // Simulated GPS fix every gps_period frames.
        const bool has_gps = (i % static_cast<std::size_t>(gps_period) == 0);
        double gx = 0.0, gy = 0.0;
        if (has_gps) {
            gx = tx + gps_noise(rng);
            gy = ty + gps_noise(rng);
            ekf.update_gps(gx, gy, gps_sigma);
            sum_sq_gps += (gx - tx) * (gx - tx) + (gy - ty) * (gy - ty);
            ++n_gps;
        }

        const double ex = ekf.x() - tx, ey = ekf.y() - ty;
        sum_sq_ekf += ex * ex + ey * ey;
        sum_sq_dr  += (dr_x - tx) * (dr_x - tx) + (dr_y - ty) * (dr_y - ty);
        ++n_ekf;

        out << (times[i] - t0) << ',' << tx << ',' << ty << ',';
        if (has_gps) out << gx << ',' << gy;
        else         out << ',';                    // blank: no fix this row
        out << ',' << ekf.x() << ',' << ekf.y()
            << ',' << dr_x << ',' << dr_y << '\n';
    }

    const double rmse_ekf = std::sqrt(sum_sq_ekf / static_cast<double>(n_ekf));
    const double rmse_dr  = std::sqrt(sum_sq_dr  / static_cast<double>(n_ekf));
    const double rmse_gps =
        (n_gps > 0) ? std::sqrt(sum_sq_gps / static_cast<double>(n_gps)) : 0.0;

    std::printf("=== KITTI dead-reckoning + GPS EKF ===\n");
    std::printf("  frames               : %zu\n", frames.size());
    std::printf("  GPS fixes            : %zu (every %d frames, sigma %.1f m)\n",
                n_gps, gps_period, gps_sigma);
    std::printf("  dead-reckoning only  : %.3f m RMSE (drifts, no GPS)\n", rmse_dr);
    std::printf("  GPS only             : %.3f m RMSE (noisy, no drift)\n", rmse_gps);
    std::printf("  EKF (fused)          : %.3f m RMSE\n", rmse_ekf);
    if (rmse_gps > 0.0)
        std::printf("  EKF vs GPS           : %.1f%% lower error\n",
                    100.0 * (1.0 - rmse_ekf / rmse_gps));
    std::printf("  CSV                  : %s\n", csv.c_str());
    return 0;
}
