#!/usr/bin/env python3
"""Run fail-closed hard gates for one normalized mini-dataset episode."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.dataset import (  # noqa: E402
    ReleaseDecision,
    ReleaseGateConfig,
    ReleaseGateResult,
    load_mini_dataset_profile,
    load_normalized_episode_artifact,
    validate_episode_release,
    write_release_decision_artifact,
)


DEFAULT_PROFILE = ROOT / "configs/profiles/isaac_nero_hand2_triview_q54_mini_dataset_v1.yaml"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--dataset-profile", type=Path, default=DEFAULT_PROFILE)
    return parser


def _live_preview_gate(run_root: Path, *, run_id: str, expected_hz: int) -> ReleaseGateResult:
    expected = {
        "release_role": "advisory_only",
        "configured_hz": expected_hz,
        "effective_tolerance_fraction": 0.10,
        "control_authority": False,
    }
    try:
        manifest = json.loads((run_root / "manifest.json").read_text(encoding="utf-8"))
        timing = manifest["simulation_timing"]
        receipt = json.loads(
            (run_root / "derived" / "live_preview" / "receipt.json").read_text(
                encoding="utf-8"
            )
        )
        effective_hz = float(receipt["effective_render_hz"])
        observed = {
            "external_preview_required": timing.get(
                "external_gui_preview_required"
            ),
            "configured_hz": receipt.get("configured_render_hz"),
            "effective_hz": effective_hz,
            "missed_periods": receipt.get("missed_render_periods"),
            "control_authority": receipt.get("control_authority"),
            "recorded_to_mcap": receipt.get("recorded_to_mcap"),
            "receipt_passed": receipt.get("passed"),
        }
        passed = bool(
            receipt.get("schema") == "wujihand.dataset_live_preview_receipt.v1"
            and receipt.get("run_id") == run_id
            and timing.get("external_gui_preview_required") is True
            and timing.get("external_gui_preview_hz") == expected_hz
            and receipt.get("configured_render_hz") == expected_hz
            and receipt.get("control_authority") is False
            and receipt.get("recorded_to_mcap") is False
            and math.isclose(effective_hz, expected_hz, rel_tol=0.10, abs_tol=0.0)
        )
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
        passed = False
        observed = {"error": f"{type(exc).__name__}:{exc}"}
    return ReleaseGateResult(
        name="external_live_gui_preview",
        passed=passed,
        expected=expected,
        observed=observed,
        reason="passed" if passed else "live_preview_missing_or_failed",
        severity="advisory",
    )


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        run_root = args.run_root.resolve()
        profile = load_mini_dataset_profile(ROOT, args.dataset_profile)
        normalized = load_normalized_episode_artifact(
            run_root / "derived" / "normalized",
            expected_run_id=run_root.name,
        )
        decision = validate_episode_release(
            normalized.facts,
            profile.q54,
            config=ReleaseGateConfig(
                control_rate_tolerance_fraction=(profile.release_control_rate_tolerance_fraction),
                minimum_real_time_factor=profile.release_minimum_real_time_factor,
                maximum_input_age_ms=profile.release_maximum_input_age_ms,
            ),
        )
        preview_gate = _live_preview_gate(
            run_root,
            run_id=decision.run_id,
            expected_hz=profile.gui_preview_hz,
        )
        decision = ReleaseDecision(
            run_id=decision.run_id,
            passed=decision.passed,
            gates=(*decision.gates, preview_gate),
        )
        artifact = write_release_decision_artifact(run_root, decision)
    except (OSError, ValueError) as exc:
        print(
            json.dumps(
                {
                    "schema": "wujihand.release_cli_result.v1",
                    "passed": False,
                    "error": str(exc),
                },
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2
    print(
        json.dumps(
            {
                "schema": "wujihand.release_cli_result.v1",
                "run_id": decision.run_id,
                "passed": decision.passed,
                "grade": decision.grade,
                "rejection_reasons": list(decision.rejection_reasons),
                "warning_reasons": list(decision.warning_reasons),
                "advisory_reasons": list(decision.advisory_reasons),
                "decision_sha256": artifact.decision_sha256,
                "output": str(artifact.root),
            },
            sort_keys=True,
        )
    )
    return 0 if decision.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
