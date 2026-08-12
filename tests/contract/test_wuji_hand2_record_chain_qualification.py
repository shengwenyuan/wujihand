from __future__ import annotations

from pathlib import Path
import os
import subprocess
import sys

from wujihand.domain.dataset_recording import DatasetSourceMode
from wujihand.runtime import RosDeploymentResolver


ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools/preflight_wuji_hand2_record_chain.py"
LAUNCH = ROOT / "ros2/wujihand_ros2/launch/dual_teleoperation.launch.py"
RUNNER = ROOT / "tools/run_isaac_nero_hand2_ros.py"
PREVIEW_VALIDATOR = ROOT / "tools/validate_dataset_preview_fixture_qualification.py"
DEPLOYMENT = (
    "configs/deployments/"
    "isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v2026_8_3_v1.yaml"
)


def test_versioned_record_chain_resolves_without_changing_historical_entry() -> None:
    resolver = RosDeploymentResolver(ROOT)
    local = "configs/examples/workstation2_nv5_ros_local_runtime_binding.example.yaml"

    current = resolver.resolve(DEPLOYMENT, local_binding=local, verify_artifacts=False)
    historical = resolver.resolve(
        "configs/deployments/isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v3.yaml",
        local_binding=local,
        verify_artifacts=False,
    )

    assert {item.asset.revision for item in current.session.instances if "hand_" in item.instance_id} == {
        "beta1_description_v2026_8_3"
    }
    assert {
        item.asset.revision for item in historical.session.instances if "hand_" in item.instance_id
    } == {"beta1_description_v2026_6_27"}
    assert current.session.assembly_path.endswith("_v2026_8_3_v1.yaml")
    assert historical.session.assembly_path.endswith("_v2026_6_27_v1.yaml")
    current_mounts = {
        item.parent.instance: item.transform.quat_wxyz
        for item in current.session.assembly.attachments
        if item.parent.frame == "link7"
    }
    historical_mounts = {
        item.parent.instance: item.transform.quat_wxyz
        for item in historical.session.assembly.attachments
        if item.parent.frame == "link7"
    }
    assert current_mounts == {
        "nero_left": (0.5, 0.5, -0.5, -0.5),
        "nero_right": (0.5, -0.5, -0.5, 0.5),
    }
    assert set(historical_mounts.values()) == {
        (0.7071067811865476, 0.0, 0.7071067811865475, 0.0)
    }


def test_record_chain_tool_help_needs_neither_sdk_nor_isaac() -> None:
    result = subprocess.run(
        [sys.executable, str(TOOL), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "--matched-chain-binding" in result.stdout
    assert "--local-runtime-binding" in result.stdout
    source = TOOL.read_text(encoding="utf-8")
    assert "isaacsim" not in source
    assert "connect(" not in source


def test_launch_preflights_before_processes_and_injects_the_8_3_overlay() -> None:
    source = LAUNCH.read_text(encoding="utf-8")

    assert source.index("preflight_command = [") < source.index("actions: list[object] = []")
    assert '"PYTHONPATH": sdk_pythonpath' in source
    assert "PYTHONNOUSERSITE" not in source
    assert 'name="dataset_live_preview"' in source
    preview = source[source.index('name="dataset_live_preview"') :]
    assert "additional_env=runtime_environment" in preview
    assert 'consumer_command.extend(["--chain-preflight"' in source
    assert 'consumer_command.extend(["--dataset-source-mode", dataset_source_mode])' in source
    assert '"--dataset-source-mode",' in source
    assert "record_qualification or not record" in source


def test_live_qualification_is_explicitly_dataset_ineligible() -> None:
    runner = RUNNER.read_text(encoding="utf-8")

    assert DatasetSourceMode.LIVE_QUALIFICATION.value == "live_qualification"
    assert (
        "DATASET_ELIGIBLE = DATASET_SOURCE_MODE is DatasetSourceMode.LIVE_TELEOPERATION"
        in runner
    )
    assert "Description 8.3 recording remains qualification-only" not in runner
    assert "record-chain preflight dataset policy differs from runtime" in runner


def test_preview_validator_pins_versioned_hand_link_inventories() -> None:
    source = PREVIEW_VALIDATOR.read_text(encoding="utf-8")

    assert '"left_hand": 27' in source
    assert '"left_hand": 26' in source
    assert "preview component inventory is not qualified" in source


def test_dataset_and_runtime_import_in_both_orders() -> None:
    inherited = os.environ.copy()
    source_root = str(ROOT / "src")
    inherited["PYTHONPATH"] = source_root + (
        f":{inherited['PYTHONPATH']}" if inherited.get("PYTHONPATH") else ""
    )
    snippets = (
        "import wujihand.dataset; import wujihand.runtime",
        "import wujihand.runtime; import wujihand.dataset",
    )
    for snippet in snippets:
        result = subprocess.run(
            [sys.executable, "-c", snippet],
            cwd=ROOT,
            env=inherited,
            check=False,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr
