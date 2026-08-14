# Teleoperation quality analyzer

This package is the read-only, offline consumer of immutable ROS 2 teleoperation run
artifacts. It is intentionally separate from `src/wujihand` and the ROS control graph.

Version `0.3.0` reads both `wujihand.teleoperation_tick_trace.v1` and `.v2`. It validates the
current explicit 120/30/15 scheduler facts and adds fail-closed validation for the dual
synthetic D405 bundles, deterministic 30 Hz stamps, raw RTX frame identities, CameraInfo,
dynamic/static TF closure and manifest/receipt calibration provenance:

The ROS wire contracts are separately versioned as `TeleoperationTickTrace` (v1) and
`TeleoperationTickTraceV2` (v2), so old MCAP payloads remain genuinely deserializable.

- artifact, checksum, topic, status and two-sided tick integrity;
- Tracker/Glove intrinsic rate, interval, sequence and raw observation quality;
- full-window trace-selected input rate and receipt inbox accounting;
- control rate, tick jitter, target-period miss ratio, source age and stage durations;
- scheduler lateness/missed slots, four physics substeps per control target, real-time factor and
  GUI render cadence when the v2 trace is present;
- arm/hand safety state, mapping, IK and retargeting distributions;
- q7/q20 command to applied-q27 composition invariants;
- post-step simulated joint tracking error;
- dynamic scene-object trajectory.
- per-side RGB/depth/CameraInfo/truth bundle counts and completed-frame cadence;
- dual-camera identity alignment, 640x480 `rgb8`/`32FC1` payloads and finite-depth ratios;
- hand-base-to-optical static extrinsics plus world-to-hand dynamic TF closure.

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

The default performance references are the current NV-5.1 targets: 30 Hz control,
15 Hz GUI preview, P95 tick interval at most 35 ms, and P95
active-source age below 20 ms. They are reported as planned targets, not silently promoted to
data-release thresholds.
