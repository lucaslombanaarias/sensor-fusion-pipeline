#!/usr/bin/env python3
"""plot_trajectory.py — XY trajectory plot for the KITTI IMU/GPS EKF.

Reads the CSV from apps/ekf_localization and draws the ground-truth path,
the noisy GPS fixes, the dead-reckoning-only path (drifts away), and the
fused EKF estimate (hugs the truth). Two panels: the full extent so the
dead-reckoning drift is visible, and a zoom on the truth/GPS/EKF cluster.

Usage:
    python3 scripts/plot_trajectory.py kitti_ekf.csv [out.png]

Dependencies: pip install matplotlib
"""

import csv
import sys
from pathlib import Path


def col(rows, name):
    out = []
    for r in rows:
        v = r.get(name, "")
        out.append(float(v) if v not in ("", None) else None)
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    path = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else "trajectory.png"
    if not Path(path).exists():
        sys.exit(f"error: {path} not found")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("error: matplotlib not installed. Run: pip install matplotlib")

    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))

    tx, ty = col(rows, "truth_x"), col(rows, "truth_y")
    ex, ey = col(rows, "ekf_x"), col(rows, "ekf_y")
    dx, dy = col(rows, "dr_x"), col(rows, "dr_y")
    gx = [v for v in col(rows, "gps_x") if v is not None]
    gy = [v for v in col(rows, "gps_y") if v is not None]

    def draw(ax, zoom):
        if not zoom and any(v is not None for v in dx):
            ax.plot([v for v in dx], [v for v in dy], color="#c9a227",
                    linewidth=1.0, label="dead-reckoning only", zorder=1)
        ax.plot(tx, ty, color="#222222", linewidth=2.0,
                label="ground truth", zorder=3)
        ax.scatter(gx, gy, s=10, color="#c1666b", alpha=0.5,
                   label="GPS fixes", zorder=2)
        ax.plot(ex, ey, color="#2a6f97", linewidth=1.3,
                label="EKF (fused)", zorder=4)
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)
        if zoom:
            ax.set_xlim(min(tx) - 10, max(tx) + 10)
            ax.set_ylim(min(ty) - 10, max(ty) + 10)
            ax.set_title("Zoom: truth vs GPS vs EKF")
        else:
            ax.set_title("Full extent (dead-reckoning drifts away)")
            ax.legend(loc="best", fontsize=9)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("KITTI vehicle localization — dead-reckoning + GPS EKF",
                 fontsize=14, fontweight="bold")
    draw(axes[0], zoom=False)
    draw(axes[1], zoom=True)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out, dpi=120)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
