# Teleoperation quality analyzer

This package is the read-only, offline consumer of immutable ROS 2 teleoperation run
artifacts. It is intentionally separate from `src/wujihand` and the ROS control graph.

Version `0.1.2` supports `wujihand.teleoperation_tick_trace.v1` and computes only facts
available in the current 20-topic recording profile:

- artifact, checksum, topic, status and two-sided tick integrity;
- Tracker/Glove intrinsic rate, interval, sequence and raw observation quality;
- full-window trace-selected input rate and receipt inbox accounting;
- control rate, tick jitter, target-period miss ratio, source age and stage durations;
- arm/hand safety state, mapping, IK and retargeting distributions;
- q7/q20 command to applied-q27 composition invariants;
- post-step simulated joint tracking error;
- dynamic scene-object trajectory.

It deliberately does not estimate task success, contact quality, fingertip pose, real-hardware
tracking, normalized cross-chain error or command-feedback lag when the required truth,
analysis range or pre-registered dynamic window is absent.

The analyzer refuses an incomplete run, verifies every path in `checksums.sha256`, never writes
inside the input run, and atomically creates a new output directory containing JSON/CSV tables,
PNG figures, an HTML report and output checksums.

Run from a ROS 2 Jazzy shell in which the recorded custom interfaces are sourced:

```bash
python tools/analysis/analyze_teleoperation_run.py \
  --run-root /path/to/complete-run \
  --output-root /path/to/new-analysis-directory
```

The default performance references are the already documented NV-5.1 targets: 60 Hz control,
P95 tick interval at most 20 ms, and P95 active-source age below 20 ms. They are reported as
planned targets, not silently promoted to data-release thresholds.
