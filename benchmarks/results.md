# Benchmark results

All numbers below are from the `sfp` binary built with `-O2` on the
machine described in "Environment." Reproduce with:

```bash
./build/sfp --config battery --duration 30 --compare --csv out.csv
python3 scripts/plot_latency.py out.csv benchmarks/battery_latency.png
```

## Headline numbers (battery config, 200 Hz estimator, 4 sensors)

| metric                 | value             |
|------------------------|-------------------|
| Estimator loop rate    | 199.4 Hz (target 200) |
| Fusion latency, mean   | 0.38 µs (376 ns)  |
| Fusion latency, max    | 7.6 µs            |
| Loop jitter, mean      | 35.2 µs           |
| Loop jitter, stddev    | 102 µs            |
| Loop jitter, max       | 5.7 ms (one scheduler stall) |
| Throughput             | ~6000 fused estimates / 30 s |
| Sensor samples fused   | ~50,800           |
| Sensor drops           | 0                 |
| Log drops              | 0                 |

The latency-over-time and jitter plots:

![latency](battery_latency.png)

The fusion latency distribution is roughly normal, centered near 350 ns,
with a thin tail to ~700 ns and occasional isolated spikes to a few
microseconds (those are the estimator thread being briefly preempted
*inside* the timed region). Jitter is dominated by the OS scheduler:
mostly under 100 µs with rare sub-millisecond and, once in this run, a
single ~5.7 ms stall. This is the expected behavior on a non-realtime
Linux kernel without CPU pinning or `SCHED_FIFO`.

## Lock-free vs locked

Same pipeline, run twice — once with the lock-free SPSC buffers, once
with the mutex-backed buffers. Three representative runs:

| run | lock-free latency mean | locked latency mean | lock-free advantage |
|-----|------------------------|---------------------|---------------------|
| 1   | 0.39 µs                | 0.74 µs             | 1.92x               |
| 2   | 0.36 µs                | 0.67 µs             | 1.84x               |
| 3   | 0.44 µs                | 0.84 µs             | 1.91x               |

The lock-free buffer consistently delivers ~1.85–1.92x lower mean fusion
latency. The win comes from the drain path: at each tick the timed region
pops from four sensor buffers (the single log-record push happens just
after, outside the measured window). The locked variant takes and
releases a mutex on every one of those pops; the lock-free variant does a
pair of atomic loads/stores with no kernel involvement and no contention
(single producer, single consumer per buffer).

**On jitter, the two are statistically indistinguishable.** Jitter is
governed by when the OS wakes the estimator thread, which the buffer
choice doesn't affect. The large jitter spikes (tens of milliseconds)
appear in whichever run happened to catch a scheduler stall — sometimes
the lock-free run, sometimes the locked run — confirming they're
environmental noise, not a property of the data structure. Mean fusion
latency is the metric where the buffer choice actually shows up, because
that's the part of the loop the buffer is on the critical path for.

## Why mean latency, not max

For the lock-free-vs-locked comparison the *mean* is the honest metric.
The *max* of either run is set by a single OS preemption landing inside
the timed fusion region, which is independent of the buffer and varies
run to run. Averaging over ~6000 ticks washes out those outliers and
exposes the steady-state cost difference, which is what the buffer
design actually controls.

## Robotics config and the complementary filter

The robotics config (`--config robotics`) fuses two position encoders,
a velocity tachometer, and a force load cell, and turns on the
complementary filter for the position channel.

```bash
./build/sfp --config robotics --duration 30 --csv robot.csv
python3 scripts/plot_latency.py robot.csv benchmarks/robotics_latency.png
```

Timing is the same order as the battery config (mean fusion latency
~0.35 µs, 199.9 Hz, ~90,000 samples fused, zero drops). The interesting
part is the fused-state panel:

![robotics](robotics_latency.png)

The position trace (blue) is a clean straight ramp from 0 to ~15 rad,
which is exactly `velocity × time` for the 0.5 rad/s sweep. The force
channel (green) shows its raw ±0.3 N measurement noise for comparison —
the position would look just as noisy if it were the raw encoder, but
the complementary filter blends in the velocity integral and smooths it.

The filter's effect is quantified directly in `test_estimator`:

- **Position RMS error: 0.049 → ~0.007 with the filter on** (~85%
  reduction) for a noisy encoder (σ = 0.05) paired with a clean
  velocity sensor.
- **Coasting:** with a 5 Hz encoder under a 200 Hz loop, the filter
  produces a position estimate on **100% of ticks** by integrating
  velocity, versus the ~2.5% of ticks that carry a fresh encoder
  reading. The estimate stays monotonic under positive velocity.

The blend is `pos = α·measured + (1−α)·(prev + v·dt)` with α = 0.05,
giving a time constant of about 95 ms at the 5 ms loop period.

## Kalman filter (`--kalman`)

`--kalman` swaps a 2-state constant-velocity Kalman filter in for the
complementary filter on the Position/Velocity channels:

```bash
./build/sfp --config robotics --duration 30 --kalman --csv robot_kf.csv
```

On the same noisy-encoder / clean-velocity setup as the complementary
filter, `test_estimator` measures:

- **Position RMS error: ~0.048 → ~0.008 with the filter on (~83%
  reduction).** Comparable headline accuracy to the complementary
  filter, but the Kalman filter weights each measurement by its own
  noise variance instead of using a fixed blend constant, and carries a
  full covariance so the Position and Velocity estimates are coupled.

The coupling is the qualitative difference, shown directly in
`test_kalman`: fed a **position-only** ramp `p = v·t` with *no* velocity
measurements at all, the filter recovers the velocity exactly (2.0 in
the test) through the off-diagonal covariance term — something neither
the per-channel average nor the complementary filter can do. The unit
tests also confirm the covariance shrinks as measurements accumulate and
stays symmetric and positive-semidefinite over thousands of steps.

Timing is unchanged from the other configs (the update is a handful of
scalar operations, no matrix inversion): mean fusion latency stays
around 1 µs at 199–200 Hz with zero drops.

## Environment

- Compiler: g++ 13.3.0, `-std=c++17 -O2`
- Kernel: stock Linux, no realtime patches, no CPU pinning
- Spin window: 50 µs (the estimator busy-waits the final 50 µs to a
  deadline after a coarse `sleep_until`)

On dedicated hardware with `SCHED_FIFO` and the estimator pinned to an
isolated core, jitter would tighten by roughly another order of
magnitude and the millisecond-scale spikes would disappear. The numbers
here are deliberately from a vanilla environment so they're
reproducible without root or kernel configuration.
