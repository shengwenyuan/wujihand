from __future__ import annotations

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).parents[2]
PREFLIGHT = ROOT / "tools/preflight_wuji_hand2_matched_chain.py"
ISAAC_RUNNER = ROOT / "tools/qualify_isaac_wuji_hand2_matched_chain.py"


def test_matched_chain_runner_preflights_before_isaac_and_has_no_business_hardware() -> None:
    source = ISAAC_RUNNER.read_text(encoding="utf-8")

    assert source.index("preflight_wuji_hand2_matched_chain(") < source.index(
        "from isaacsim import SimulationApp"
    )
    assert 'choices=("stub", "glove")' in source
    assert 'choices=("dynamic", "kinematic-diagnostic")' in source
    for recorded_field in (
        '"landmark_positions_m"',
        '"retarget_q20_rad"',
        '"command_q20_rad"',
        '"feedback_q20_rad"',
        '"drive_inventory"',
    ):
        assert recorded_field in source
    for forbidden in (
        "Nero",
        "tracker",
        "rclpy",
        "dataset_writer",
        "can_adapter",
        "Hand2Hardware",
    ):
        assert forbidden not in source


def test_preflight_tool_does_not_import_isaac_or_connect_a_glove() -> None:
    source = PREFLIGHT.read_text(encoding="utf-8")

    assert "isaacsim" not in source
    assert "WujiGloveHandSkeletonAdapter" not in source
    assert "connect(" not in source


def test_matched_chain_tool_help_needs_neither_sdk_nor_isaac() -> None:
    for tool in (PREFLIGHT, ISAAC_RUNNER):
        result = subprocess.run(
            [sys.executable, str(tool), "--help"],
            check=False,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr
        assert "--local-binding" in result.stdout
        assert "--side" in result.stdout
        assert "--input" in result.stdout


def test_matched_chain_runner_forbids_kinematic_glove_input_before_sdk_import() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(ISAAC_RUNNER),
            "--side",
            "left",
            "--input",
            "glove",
            "--control-mode",
            "kinematic-diagnostic",
            "--local-binding",
            "unused.yaml",
            "--output-dir",
            "unused-output",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "Glove input requires --control-mode dynamic" in result.stderr
    assert "No module named 'wuji_sdk'" not in result.stderr
