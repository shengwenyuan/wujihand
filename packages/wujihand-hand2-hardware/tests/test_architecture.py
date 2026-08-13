import ast
from pathlib import Path

PACKAGE = Path(__file__).parents[1] / "src" / "wujihand_hand2_hardware"
FORBIDDEN_IMPORTS = ("omni", "pxr", "mujoco", "rclpy", "wujihand")
ALWAYS_FORBIDDEN_CALLS = {
    "clear_fault",
    "clear_origin",
    "set",
    "set_origin",
}
RAW_WRITE_CALLS = {"disable", "emergency_stop", "enable", "joint_command", "publish", "send"}
SDK_MOTION_ADAPTER = PACKAGE / "sdk_motion.py"


def test_hardware_package_has_no_main_runtime_dependency() -> None:
    for path in PACKAGE.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module]
            else:
                continue
            assert not any(name.startswith(FORBIDDEN_IMPORTS) for name in names), path


def test_parameter_origin_and_fault_mutations_remain_absent() -> None:
    for path in PACKAGE.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        called = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        assert not called.intersection(ALWAYS_FORBIDDEN_CALLS), (
            path,
            called & ALWAYS_FORBIDDEN_CALLS,
        )


def test_raw_sdk_writes_exist_only_in_motion_adapter() -> None:
    for path in PACKAGE.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        called = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        if path == SDK_MOTION_ADAPTER:
            assert {
                "enable",
                "disable",
                "emergency_stop",
                "joint_command",
                "publish",
                "send",
            } <= called
        else:
            assert not called.intersection(RAW_WRITE_CALLS), (path, called & RAW_WRITE_CALLS)
