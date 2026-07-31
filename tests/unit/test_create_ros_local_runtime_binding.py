from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "create_ros_local_runtime_binding",
    ROOT / "tools/create_ros_local_runtime_binding.py",
)
assert SPEC is not None
assert SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_runtime_path_preserves_virtual_environment_symlink(
    tmp_path: Path,
) -> None:
    interpreter = tmp_path / "python3.12"
    interpreter.touch()
    virtual_environment = tmp_path / "venv"
    virtual_environment.mkdir()
    symlink = virtual_environment / "python"
    symlink.symlink_to(interpreter)

    assert MODULE._runtime_path(symlink) == str(symlink)
