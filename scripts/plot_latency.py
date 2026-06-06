#!/usr/bin/env python3
"""plot_latency.py — visualize the fusion pipeline's timing and fused
state from a CSV produced by the `sfp` binary.

Produces a 2x2 figure:
  - top-left:  per-tick fusion latency over time (ns)
  - top-right: latency distribution (histogram)
  - bottom-left: per-tick loop jitter over time (us)
  - bottom-right: fused channel values over time

This is the only part of the project that uses a non-stdlib dependency
(matplotlib). The C++ side has zero external dependencies; plotting is
deliberately kept out-of-process so the pipeline stays portable. Run it
separately to generate the PNG for the README.

Usage:
    python3 scripts/plot_latency.py fusion_log.csv [output.png]

Dependencies:
    pip install matplotlib pandas
"""

import sys
import csv
from pathlib import Path


def load_csv(path):
    """Load the CSV with the stdlib csv module (no pandas required for
    loading; we only need pandas/matplotlib for plotting)."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        for row in reader:
            rows.append(row)
    return fieldnames, rows


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    csv_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else "latency.png"

    if not Path(csv_path).exists():
        print(f"error: {csv_path} not found")
        sys.exit(1)

    try:
        import matplotlib
        matplotlib.use("Agg")  # headless backend, no display needed
        import matplotlib.pyplot as plt
    except ImportError:
        print("error: matplotlib not installed. Run: pip install matplotlib")
        sys.exit(1)

    fieldnames, rows = load_csv(csv_path)

    # Known columns; everything between jitter_us and channel_mask is a
    # fused channel.
    fixed = {"tick", "t_seconds", "latency_ns", "jitter_us", "channel_mask"}
    channels = [c for c in fieldnames if c not in fixed]

    t = [float(r["t_seconds"]) for r in rows]
    latency_ns = [float(r["latency_ns"]) for r in rows]
    jitter_us = [float(r["jitter_us"]) for r in rows]

    # Summary stats for titles.
    def stats(xs):
        n = len(xs)
        mean = sum(xs) / n if n else 0.0
        var = sum((x - mean) ** 2 for x in xs) / (n - 1) if n > 1 else 0.0
        return mean, var ** 0.5, max(xs) if xs else 0.0

    lat_mean, lat_std, lat_max = stats(latency_ns)
    jit_mean, jit_std, jit_max = stats(jitter_us)

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(f"Fusion pipeline timing — {Path(csv_path).name}",
                 fontsize=14, fontweight="bold")

    # Latency over time.
    ax = axes[0][0]
    ax.plot(t, latency_ns, linewidth=0.5, color="#2a6f97")
    ax.set_title(f"Fusion latency  (mean {lat_mean:.0f} ns, "
                 f"max {lat_max:.0f} ns)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("latency (ns)")
    ax.grid(True, alpha=0.3)

    # Latency histogram. Clip the long tail at the 99th percentile so the
    # bulk of the distribution is visible.
    ax = axes[0][1]
    sorted_lat = sorted(latency_ns)
    p99 = sorted_lat[int(len(sorted_lat) * 0.99)] if sorted_lat else 1
    clipped = [x for x in latency_ns if x <= p99]
    ax.hist(clipped, bins=60, color="#2a6f97", alpha=0.8)
    ax.set_title("Latency distribution (clipped at p99)")
    ax.set_xlabel("latency (ns)")
    ax.set_ylabel("count")
    ax.grid(True, alpha=0.3)

    # Jitter over time.
    ax = axes[1][0]
    ax.plot(t, jitter_us, linewidth=0.5, color="#bb3e03")
    ax.set_title(f"Loop jitter  (mean {jit_mean:.1f} us, "
                 f"stddev {jit_std:.1f} us)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("jitter (us)")
    ax.grid(True, alpha=0.3)

    # Fused channels over time.
    ax = axes[1][1]
    for ch in channels:
        vals = [float(r[ch]) for r in rows]
        ax.plot(t, vals, linewidth=0.7, label=ch)
    ax.set_title("Fused state estimate")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("value")
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_path, dpi=120)
    print(f"wrote {out_path}")
    print(f"  latency: mean {lat_mean:.0f} ns, stddev {lat_std:.0f} ns, "
          f"max {lat_max:.0f} ns")
    print(f"  jitter:  mean {jit_mean:.1f} us, stddev {jit_std:.1f} us, "
          f"max {jit_max:.1f} us")


if __name__ == "__main__":
    main()
