from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from typing import Any

from launch import LaunchContext


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))


def _module() -> Any:
    path = (
        ROOT
        / "ros2/wujihand_ros2/launch/"
        "dual_teleoperation.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "dual_teleoperation_launch",
        path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("launch module could not be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _actions(deployment: str) -> list[object]:
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "project_root": str(ROOT),
            "deployment": deployment,
            "local_runtime_binding": (
                "configs/examples/"
                "workstation2_nv5_ros_local_runtime_binding.example.yaml"
            ),
            "gui": "false",
            "frames": "1",
            "record": "false",
        }
    )
    return _module()._processes(context)


def test_full_and_arm_only_launch_expand_distinct_source_graphs() -> None:
    full = _actions(
        "configs/deployments/"
        "isaac_nero_hand2_ros_dual_live_v2.yaml"
    )
    arms = _actions(
        "configs/deployments/"
        "isaac_nero_hand2_ros_dual_arm_only_live_v2.yaml"
    )

    assert len(full) == 4
    assert len(arms) == 3
