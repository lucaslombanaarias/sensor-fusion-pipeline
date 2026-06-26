#!/usr/bin/env python3
"""plot_fusion.py — show what the fusion actually buys you.

Overlays the Position channel from a raw (unfiltered) run against a
filtered run, on top of the true trajectory, so the smoothing is visible
at a glance. This is the "money plot" for the README.

Generate the two CSVs first, e.g.:

    ./build/sfp --config robotics --duration 8 --no-filter --csv raw.csv
    ./build/sfp --config robotics --duration 8 --kalman    --csv kf.csv
    python3 scripts/plot_fusion.py --raw raw.csv --filtered kf.csv \
        --truth-slope 0.5 --label "Kalman" -o benchmarks/fusion_kalman.png

Dependencies: pip install matplotlib
"""

import argparse
import csv
import sys
from pathlib import Path


def load_channel(path, channel):
    t, vals = [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if channel not in (reader.fieldnames or []):
            sys.exit(f"error: column '{channel}' not in {path}; "
                     f"have {reader.fieldnames}")
        for row in reader:
            # Only plot ticks where the channel actually carried a value.
            mask = int(row.get("channel_mask", "0"))
            if "position" == channel and not (mask & (1 << 3)):
                # Position is channel index 3 (see SensorType); skip blanks.
                continue
            t.append(float(row["t_seconds"]))
            vals.append(float(row[channel]))
    return t, vals


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--raw", required=True, help="unfiltered CSV (baseline)")
    ap.add_argument("--filtered", required=True, help="filtered CSV")
    ap.add_argument("--channel", default="position", help="channel to plot")
    ap.add_argument("--truth-slope", type=float, default=None,
                    help="ground truth = slope * t (line, or residual zero)")
    ap.add_argument("--residual", action="store_true",
                    help="plot estimate - truth so the noise is visible "
                         "regardless of the absolute scale (needs --truth-slope)")
    ap.add_argument("--label", default="filtered", help="filtered series label")
    ap.add_argument("-o", "--out", default="fusion.png")
    args = ap.parse_args()
    if args.residual and args.truth_slope is None:
        sys.exit("error: --residual requires --truth-slope")

    for p in (args.raw, args.filtered):
        if not Path(p).exists():
            sys.exit(f"error: {p} not found")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("error: matplotlib not installed. Run: pip install matplotlib")

    tr, raw = load_channel(args.raw, args.channel)
    tf, filt = load_channel(args.filtered, args.channel)

    def rms(ts, vs):
        errs = [v - args.truth_slope * t for t, v in zip(ts, vs)]
        return (sum(e * e for e in errs) / len(errs)) ** 0.5 if errs else 0.0

    fig, ax = plt.subplots(figsize=(11, 6))

    if args.residual:
        rms_raw, rms_filt = rms(tr, raw), rms(tf, filt)
        raw = [v - args.truth_slope * t for t, v in zip(tr, raw)]
        filt = [v - args.truth_slope * t for t, v in zip(tf, filt)]
        ax.axhline(0.0, linewidth=1.2, color="#222222", linestyle="--",
                   label="zero error", zorder=2)
        ax.plot(tr, raw, linewidth=0.6, color="#c1666b",
                label=f"raw error (RMS {rms_raw:.4f})", zorder=1)
        ax.plot(tf, filt, linewidth=1.4, color="#2a6f97",
                label=f"{args.label} error (RMS {rms_filt:.4f})", zorder=3)
        ax.set_title(f"{args.channel.capitalize()} tracking error: "
                     f"raw vs {args.label} fusion")
        ax.set_ylabel(f"{args.channel} error (estimate - truth)")
    else:
        ax.plot(tr, raw, linewidth=0.6, color="#bbbbbb",
                label=f"raw {args.channel}", zorder=1)
        if args.truth_slope is not None:
            span = [min(tr + tf), max(tr + tf)]
            ax.plot(span, [args.truth_slope * x for x in span],
                    linewidth=1.5, color="#222222", linestyle="--",
                    label="ground truth", zorder=2)
        ax.plot(tf, filt, linewidth=1.6, color="#2a6f97",
                label=f"{args.label} estimate", zorder=3)
        ax.set_title(f"{args.channel.capitalize()}: raw vs {args.label} fusion")
        ax.set_ylabel(args.channel)

    ax.set_xlabel("time (s)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
