# Wuji Hand2 hardware bring-up

Independent, bounded utilities for Wuji Hand2 Beta1 hardware qualification. The package
depends on Wuji SDK 2026.8.3, but never imports the main `wujihand`, ROS, Isaac, MuJoCo, or dataset
runtime.

The qualification commands remain read-only. A separate interactive sequence executor provides the
bounded H3 right-hand S1 profile and the H4-A right-hand q20 isolation profile. It cannot write
parameters, clear faults/origins, change networking, or upgrade firmware, and it never imports ROS,
Isaac, MuJoCo, NERO, Glove, or dataset code.

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

The H3/H4 bench profiles apply a provisional project ceiling of `65 °C` MCU temperature and retain
the independent `+5 °C` rise-from-start stop. Version 0.5.1 raised the former `58 °C` absolute
ceiling after normal-duration bench evidence showed that value was too conservative. Every
temperature remains in the receipt. Neither threshold is a Wuji vendor rating; the Beta1
documentation does not publish a final thermal limit.

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

The legacy H3 `--waiver-id` spelling remains available. New invocations should use
`--scope-id H2-WAIVER-20260812-RIGHT-S1-SEQUENCE`.

## H4-A q20 isolation bench

The H4-A profile reuses the same executor and safety path, but expands the declared sequence to all
20 joints in protocol order: thumb through pinky, S1 through S4 within each finger. It still enables
exactly one joint at a time, sends complete q20 commands, and returns and disables the selected joint
before continuing. The default positive `0.12 rad` motion is a mapping test; the physical direction
of S2/S3/S4 remains unqualified until an operator observes the live run.

Version 0.5.2 keeps the H3 minimum observed excursion at `5%` of the requested delta and uses `1%`
for H4-A (`0.0012 rad` at the default delta). H4-A is an identity/isolation audit rather than a
tracking-performance benchmark, and every axis still requires operator direction/isolation
observation. Zero observed motion still fails.

```bash
wujihand-hand2-hardware bench-joint-sequence \
  --serial <RIGHT_HAND2_SN> \
  --address <RIGHT_HAND2_IP:PORT> \
  --side right \
  --firmware 2.2.3 \
  --hardware 0.2.0 \
  --profile right-q20-isolation-v1 \
  --delta-rad 0.12 \
  --scope-id H4-RIGHT-Q20-ISOLATION-V1 \
  --output-dir artifacts/validation/<RUN_ID>
```

H4-A does not authorize simultaneous joints, grasping, load, project teleoperation, or recording.
Functional whole-hand evidence may come from the pinned official Wuji SDK teleoperation example;
the hardware package deliberately does not duplicate that example or add an H4-B gesture executor.

`85%` is a project acceptance floor for the currently pinned Beta1 right-hand stack, based on the
2026-08-13 bench evidence. It is not a Wuji rating and must be reviewed after any hardware, firmware,
SDK, topology, or command-rate change.
