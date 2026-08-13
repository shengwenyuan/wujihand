# Wuji Hand2 hardware bring-up

Independent, bounded utilities for Wuji Hand2 Beta1 hardware qualification. The package
depends on Wuji SDK 2026.8.3, but never imports the main `wujihand`, ROS, Isaac, MuJoCo, or dataset
runtime.

The qualification commands remain read-only. H3 adds one separate, interactive right-hand S1
sequence executor. It cannot write parameters, clear faults/origins, change networking, or upgrade
firmware, and it never imports ROS, Isaac, MuJoCo, NERO, Glove, or dataset code.

## Read-only qualification

```bash
wujihand-hand2-hardware qualify-readonly \
  --serial <HAND2_SN> \
  --address <HAND2_IP:PORT> \
  --side right \
  --firmware 2.2.3 \
  --hardware 0.2.0 \
  --duration-s 60 \
  --output-dir artifacts/validation/<RUN_ID>
```

The command checks the discovered address and device type before connecting by serial number. It
then allows a three-second read-only warm-up before validating identity, the fixed q20 label/NID
layout, state and diagnostic sequences, errors, limits, finite values, temperature, and clean
disconnect behavior. The only communication Gate is the `joint_diagnostics` response-rate floor:
every joint must remain at or above `85%`. Transport, timeout, and `comm_diag` counters remain in the
receipt but do not independently fail a run.

## Temperature listener

```bash
wujihand-hand2-hardware monitor-temperature \
  --serial <HAND2_SN> \
  --address <HAND2_IP:PORT> \
  --side right \
  --firmware 2.2.3 \
  --hardware 0.2.0 \
  --duration-s 600 \
  --sample-period-s 1 \
  --max-rise-c 5 \
  --output-dir artifacts/validation/<RUN_ID>
```

The listener prints and records one bounded temperature summary per interval. Communication evidence
is written to `communication.jsonl`; `85–99%` response windows, transient `comm_diag` reports,
counter deltas, and transport observations do not truncate the run. A response window below `85%`,
missing/stale joint streams, a missing q20 joint, nonzero device errors, non-Ready status, active
limits, invalid values, or a configured temperature guard still stops it.

`--max-rise-c` is a project qualification guard relative to the first sample, not a vendor thermal
rating. An absolute `--max-temperature-c` can be supplied only when a separately approved limit
exists.

## H3 sequential S1 bench

`bench-joint-sequence` is intentionally interactive and limited to the five right-hand S1 flexion
axes, in thumb/index/middle/ring/pinky order. A single empty-line confirmation covers the complete
declared sequence and happens before the SDK connects; this avoids accumulating unconsumed 1 kHz
subscription data while waiting for operator input. The executor then runs a fresh 30-second
preflight, reads existing MIT parameters and effort limits without changing them, and measures a
fresh static baseline before every step.

The H3 pilot also applies a provisional project ceiling of `58 °C` MCU temperature, rounded from
the earlier `+5 °C` observation stop near `57.88 °C`. This is deliberately conservative and is not
presented as a Wuji vendor rating; the Beta1 documentation does not publish a final thermal limit.

Each step sends a complete q20 frame with zero velocity/effort feed-forward, enables only one S1
axis, ramps by the configured positive delta (default `0.12 rad`, hard maximum `0.15 rad`), returns
to the measured baseline, disables that joint, and verifies the post-disable readback before moving
to the next finger. A response window below `85%`, state/diagnostic staleness, a missing joint, fault,
limit, non-target motion, excess temperature rise, Ctrl-C, or command error enters the fail-closed
stop path. Communication counters remain diagnostic evidence. A failed disable escalates to the
whole-hand SDK emergency stop.

The command requires the frozen limited-waiver identifier and a TTY:

```bash
wujihand-hand2-hardware bench-joint-sequence \
  --serial <RIGHT_HAND2_SN> \
  --address <RIGHT_HAND2_IP:PORT> \
  --side right \
  --firmware 2.2.3 \
  --hardware 0.2.0 \
  --profile right-s1-flexion-v1 \
  --delta-rad 0.12 \
  --waiver-id H2-WAIVER-20260812-RIGHT-S1-SEQUENCE \
  --output-dir artifacts/validation/<RUN_ID>
```

Automatic success still requires the operator's physical direction/isolation observation for every
step before the H3 result is accepted. The joints run serially, never simultaneously. This executor
is not a teleoperation or whole-hand API.

`85%` is a project acceptance floor for the currently pinned Beta1 right-hand stack, based on the
2026-08-13 bench evidence. It is not a Wuji rating and must be reviewed after any hardware, firmware,
SDK, topology, or command-rate change.
