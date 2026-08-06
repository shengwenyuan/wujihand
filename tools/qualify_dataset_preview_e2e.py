#!/usr/bin/env python3
# ruff: noqa: E402  # Repository source paths are resolved before local imports.
"""Run one isolated, device-free ROS2-Isaac-GUI qualification and validate it."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))

from wujihand.runtime import new_run_id

from tools.validate_dataset_preview_fixture_qualification import validate


DEFAULT_DEPLOYMENT = (
    "configs/deployments/isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v3.yaml"
)
DEFAULT_LOCAL_BINDING = "configs/local/workstation2_nv5_ros_v2.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--deployment", default=DEFAULT_DEPLOYMENT)
    parser.add_argument("--local-runtime-binding", default=DEFAULT_LOCAL_BINDING)
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
    try:
        result = validate(run_root)
    except BaseException as exc:
        print(
            f"DATASET PREVIEW QUALIFICATION FAILED: run_id={run_id} "
            f"error={type(exc).__name__}:{exc}",
            flush=True,
        )
        return 2
    destination = run_root / "qualification/receipt.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)
    print(
        "DATASET PREVIEW QUALIFICATION "
        f"{'PASSED' if result['passed'] else 'FAILED'}: "
        f"run_id={run_id} failures={result['failures']} root={run_root}",
        flush=True,
    )
    return 0 if launch.returncode == 0 and result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
