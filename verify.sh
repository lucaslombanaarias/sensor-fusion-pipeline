#!/usr/bin/env bash
# verify.sh — build everything, run the test suite, and do a short demo
# run of both configs. A quick way to confirm the project works on your
# machine after unzipping.
#
# Usage: ./verify.sh
set -euo pipefail

cd "$(dirname "$0")"

echo "=== Building (make) ==="
make

echo
echo "=== Running test suite ==="
make test

echo
echo "=== Demo: battery config, 5 s ==="
./build/sfp --config battery --duration 5 --csv battery_demo.csv

echo
echo "=== Demo: robotics config, 5 s (complementary filter on) ==="
./build/sfp --config robotics --duration 5 --csv robotics_demo.csv

echo
echo "=== Demo: robotics config, 5 s (Kalman filter on) ==="
./build/sfp --config robotics --duration 5 --kalman --csv robotics_kalman_demo.csv

echo
echo "=== Demo: lock-free vs locked comparison, battery, 5 s ==="
./build/sfp --config battery --duration 5 --compare --csv battery_compare.csv

echo
echo "Done. CSVs written to the current directory."
echo "To plot (needs 'pip install matplotlib'):"
echo "    python3 scripts/plot_latency.py battery_demo.csv latency.png"
