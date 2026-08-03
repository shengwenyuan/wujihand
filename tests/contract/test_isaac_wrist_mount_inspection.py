from __future__ import annotations

import ast
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).parents[2]
TOOL = ROOT / "tools/run_isaac_nero_hand2_gemini305_mount_inspection.py"
ADAPTER = ROOT / "src/wujihand/adapters/simulation/nero_hand2_gemini305_mount.py"
RUNTIME = ROOT / "src/wujihand/runtime/isaac_wrist_mount_inspection.py"


def _top_level_imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    modules: set[str] = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            modules.add(node.module or "")
    return modules


def test_tool_and_overlay_defer_backend_imports_until_after_preflight() -> None:
    forbidden = ("isaacsim", "omni", "pxr")
    assert not {
        module
        for module in _top_level_imports(TOOL) | _top_level_imports(ADAPTER)
        if module.startswith(forbidden)
    }

    source = TOOL.read_text(encoding="utf-8")
    assert source.index("preflight_isaac_wrist_mount_inspection(") < source.index(
        "from isaacsim import SimulationApp"
    )
    assert "input(" not in source
    assert "rclpy" not in source
    assert "can_adapter" not in source


def test_tool_help_needs_no_isaac_installation() -> None:
    result = subprocess.run(
        [sys.executable, str(TOOL), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "--mount-stl" in result.stdout
    assert "--initial-view" in result.stdout
    assert "--gui" in result.stdout


def test_runtime_keeps_session_composition_out_of_the_simulation_adapter() -> None:
    adapter_source = ADAPTER.read_text(encoding="utf-8")
    runtime_source = RUNTIME.read_text(encoding="utf-8")

    assert "wujihand.runtime" not in adapter_source
    assert "SessionResolver" in runtime_source
    assert "DualNeroHand2IsaacScene" in runtime_source
    assert "child_base_link_path" in runtime_source
    assert "visual-only" in runtime_source
    assert "UsdPhysics.CollisionAPI" in runtime_source
