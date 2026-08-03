"""Command-line entry point for one immutable ROS 2 run."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path

from .metrics import AnalysisConfig
from .pipeline import analyze_run


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a versioned offline quality report from one complete ROS 2 run."
    )
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--expected-control-hz", type=float, default=60.0)
    parser.add_argument("--control-rate-tolerance-fraction", type=float, default=0.02)
    parser.add_argument("--p95-tick-interval-limit-ms", type=float, default=20.0)
    parser.add_argument("--p95-active-input-age-limit-ms", type=float, default=20.0)
    parser.add_argument("--q27-composition-atol-rad", type=float, default=1e-12)
    return parser


def run(argv: Sequence[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    try:
        output = analyze_run(
            arguments.run_root,
            arguments.output_root,
            config=AnalysisConfig(
                expected_control_hz=arguments.expected_control_hz,
                control_rate_tolerance_fraction=(arguments.control_rate_tolerance_fraction),
                p95_tick_interval_limit_ms=arguments.p95_tick_interval_limit_ms,
                p95_comparable_input_age_limit_ms=(arguments.p95_active_input_age_limit_ms),
                q27_composition_atol_rad=arguments.q27_composition_atol_rad,
            ),
        )
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"analysis failed: {exc}", file=sys.stderr)
        return 2
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    print(
        json.dumps(
            {
                "output_root": str(output),
                "run_id": summary["run_id"],
                "structural_gates_passed": summary["structural_gates_passed"],
                "planned_targets_passed": summary["planned_targets_passed"],
            },
            sort_keys=True,
        )
    )
    return 0


def main() -> None:
    raise SystemExit(run())


__all__ = ["main", "run"]
