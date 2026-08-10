#!/usr/bin/env python3
"""Run isolated Isaac rotation-ball trials and enforce the scene-profile gate."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.runtime import load_rotation_ball_config  # noqa: E402


DEFAULT_ISAAC_PYTHON = Path(
    "/home/yanziwei/software/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh"
)
DEFAULT_SCENE_PROFILE = ROOT / "configs/base/hand2_rotation_ball_v2026_6_27_v1.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int)
    parser.add_argument("--required-successes", type=int)
    parser.add_argument("--frames", type=int, default=1200)
    parser.add_argument("--isaac-python", type=Path, default=DEFAULT_ISAAC_PYTHON)
    parser.add_argument("--scene-profile", type=Path, default=DEFAULT_SCENE_PROFILE)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "artifacts/runs/isaac_hand2_rotation_ball_trials",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.scene_profile.is_file():
        raise SystemExit(f"rotation-ball scene profile not found: {args.scene_profile}")
    scene_config = load_rotation_ball_config(args.scene_profile)
    trial_count = (
        scene_config.qualification.trial_count if args.trials is None else args.trials
    )
    required_successes = (
        scene_config.qualification.required_successes
        if args.required_successes is None
        else args.required_successes
    )
    if trial_count < 1 or not 1 <= required_successes <= trial_count:
        raise SystemExit("required successes must be in [1, trials]")
    if args.frames < 1:
        raise SystemExit("frames must be positive")
    if not args.isaac_python.is_file():
        raise SystemExit(f"Isaac Python launcher not found: {args.isaac_python}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    runner = ROOT / "tools/run_isaac_hand2_rotation_ball.py"
    child_environment = os.environ.copy()
    for name in ("PYTHONPATH", "CONDA_PREFIX", "CONDA_DEFAULT_ENV", "VIRTUAL_ENV"):
        child_environment.pop(name, None)

    trials: list[dict[str, object]] = []
    for trial_index in range(1, trial_count + 1):
        trial_dir = args.output_dir / f"trial_{trial_index:02d}"
        trial_dir.mkdir(parents=True, exist_ok=True)
        for stale_name in ("validation.json", "error.txt"):
            (trial_dir / stale_name).unlink(missing_ok=True)
        result = subprocess.run(
            [
                str(args.isaac_python),
                str(runner),
                "--frames",
                str(args.frames),
                "--scene-profile",
                str(args.scene_profile),
                "--require-grasp-success",
                "--skip-screenshot",
                "--validation-output-dir",
                str(trial_dir),
            ],
            cwd=ROOT,
            env=child_environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        report_path = trial_dir / "validation.json"
        report: dict[str, object] | None = None
        report_error: str | None = None
        if report_path.is_file():
            try:
                decoded = json.loads(report_path.read_text(encoding="utf-8"))
                if not isinstance(decoded, dict):
                    raise ValueError("validation report root must be an object")
                report = decoded
            except (OSError, ValueError) as exc:
                report_error = f"{type(exc).__name__}: {exc}"
        passed = bool(
            result.returncode == 0
            and report is not None
            and report.get("structural_passed") is True
            and report.get("movement_observed") is True
            and report.get("grasp_passed") is True
        )
        trial = {
            "trial": trial_index,
            "passed": passed,
            "launcher_returncode": result.returncode,
            "validation": str(report_path),
            "validation_error": report_error,
            "grasp_pass_time_s": None if report is None else report.get("grasp_pass_time_s"),
            "best_hold_duration_s": (
                None if report is None else report.get("best_hold_duration_s")
            ),
            "max_flange_translation_error_m": (
                None if report is None else report.get("max_flange_translation_error_m")
            ),
        }
        trials.append(trial)
        print(
            f"trial {trial_index:02d}/{trial_count}: "
            f"{'PASS' if passed else 'FAIL'}",
            flush=True,
        )

    success_count = sum(bool(trial["passed"]) for trial in trials)
    summary = {
        "scene_profile": str(args.scene_profile),
        "trial_count": trial_count,
        "required_successes": required_successes,
        "success_count": success_count,
        "passed": success_count >= required_successes,
        "frames_per_trial": args.frames,
        "trials": trials,
    }
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
