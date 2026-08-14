#!/usr/bin/env python3
# ruff: noqa: E402  # Repository source paths are resolved before local imports.
"""Run one isolated, device-free ROS2-Isaac-GUI qualification and validate it."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))

from wujihand.runtime import new_run_id

DEFAULT_DEPLOYMENT = (
    "configs/deployments/"
    "isaac_nero_hand2_ros_dual_tframe_gripper_flange_collision_proxy_"
    "triview_q54_self_collision_v1.yaml"
)
DEFAULT_LOCAL_BINDING = "configs/local/workstation2_nv5_ros_v2.yaml"
PREVIEW_VALIDATOR = ROOT / "tools/validate_dataset_preview_fixture_qualification.py"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--deployment", default=DEFAULT_DEPLOYMENT)
    parser.add_argument("--local-runtime-binding", default=DEFAULT_LOCAL_BINDING)
    parser.add_argument(
        "--matched-chain-binding",
        help="Required by the explicit Description 8.3 deployment.",
    )
    parser.add_argument(
        "--record-chain-qualification",
        help="Versioned record-chain policy for the selected deployment.",
    )
    args = parser.parse_args()
    run_id = args.run_id or new_run_id(prefix="dataset-preview-qual")
    run_root = (ROOT / "artifacts/diagnostics/dataset-preview-qualification" / run_id).resolve()
    if run_root.exists():
        raise SystemExit(f"qualification run already exists: {run_root}")
    command = [
        "ros2",
        "launch",
        "wujihand_ros2",
        "dual_teleoperation.launch.py",
        f"project_root:={ROOT}",
        f"deployment:={args.deployment}",
        f"local_runtime_binding:={args.local_runtime_binding}",
        "gui:=true",
        "record:=true",
        f"run_id:={run_id}",
        "isaac_cpu_affinity:=0-15",
        "qualification_fixture:=true",
    ]
    if args.matched_chain_binding:
        command.append(f"matched_chain_binding:={args.matched_chain_binding}")
    if args.record_chain_qualification:
        command.append(
            "record_chain_qualification:="
            f"{args.record_chain_qualification}"
        )
    environment = os.environ.copy()
    python_paths = (
        str(ROOT),
        str(ROOT / "src"),
        str(ROOT / "analysis/teleoperation_quality/src"),
    )
    inherited_python_path = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = os.pathsep.join(
        (*python_paths, *(() if not inherited_python_path else (inherited_python_path,)))
    )
    launch = subprocess.run(command, cwd=ROOT, check=False, env=environment)
    validation = subprocess.run(
        ["python3", str(PREVIEW_VALIDATOR), "--run-root", str(run_root)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        env=environment,
        text=True,
    )
    destination = run_root / "qualification/receipt.json"
    try:
        result = json.loads(destination.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(
            f"DATASET PREVIEW QUALIFICATION FAILED: run_id={run_id} "
            f"error={type(exc).__name__}:{exc}",
            flush=True,
        )
        return 2
    print(
        "DATASET PREVIEW QUALIFICATION "
        f"{'PASSED' if result['passed'] else 'FAILED'}: "
        f"run_id={run_id} failures={result['failures']} root={run_root}",
        flush=True,
    )
    return (
        0
        if launch.returncode == 0
        and validation.returncode == 0
        and result["passed"]
        else 2
    )


if __name__ == "__main__":
    raise SystemExit(main())
