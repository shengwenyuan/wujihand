from __future__ import annotations

from pathlib import Path

from wujihand.domain import HandSide
from wujihand.runtime import RosDeploymentResolver
from wujihand.runtime.wuji_hand2_record_chain import (
    load_record_chain_qualification_policy,
)

ROOT = Path(__file__).resolve().parents[2]
LOCAL = "configs/examples/workstation2_nv5_ros_local_runtime_binding.example.yaml"
CURRENT = (
    "configs/deployments/"
    "isaac_nero_hand2_ros_dual_tframe_gripper_flange_collision_proxy_"
    "triview_q54_self_collision_v1.yaml"
)
POLICY = ROOT / (
    "configs/qualifications/"
    "isaac_nero_hand2_tframe_gripper_flange_collision_proxy_"
    "self_collision_record_chain_v1.yaml"
)


def test_current_entry_closes_the_single_ros_recording_graph_and_scene_policy() -> None:
    current = RosDeploymentResolver(ROOT).resolve(
        CURRENT, local_binding=LOCAL, verify_artifacts=False
    )

    assert tuple(process.process_id for process in current.deployment.processes) == (
        "vive_source",
        "glove_source",
        "isaac_consumer",
    )
    assert len(current.route_plan.routes) == 4
    assert current.control_profile.physics_hz == 120
    assert current.control_profile.tracker.max_consecutive_ik_failures == 3
    assert current.session.workcell.workcell_id == "isaac_dual_nero_tframe_candidate_20260811_v1"
    assert current.session.assembly.assembly_id == (
        "nero_dual_hand2_d405_wrist_rig_tframe_gripper_flange_collision_proxy_v1"
    )
    assert current.session.session.dataset_profile is not None
    assert current.session.session.dataset_profile.expected_id == (
        "isaac_nero_hand2_triview_q54_mini_dataset_120_30_15_v1"
    )
    for side in ("left", "right"):
        assert current.session.instance(f"nero_{side}").asset.asset_id == (
            "agilex_nero_gripper_flange_collision_proxy"
        )
    assert current.self_collision_profile_id == (
        "isaac_nero_hand2_self_collision_filtered_pairs_gripper_flange_collision_proxy_v1"
    )
    assert current.deployment.tracking_setup.qualification_status == "pending"


def test_tframe_record_policy_keeps_task_scene_outside_the_session() -> None:
    policy = load_record_chain_qualification_policy(POLICY)

    assert policy.task_scene is not None
    assert policy.task_scene.path == ("configs/scenes/isaac_robolab_banana_bowl_low_table_v2.yaml")
    assert policy.nero.assembly_attachment_quaternion(HandSide.LEFT) == (
        0.0,
        0.0,
        1.0,
        0.0,
    )
    assert policy.nero.assembly_attachment_quaternion(HandSide.RIGHT) == (
        0.0,
        0.0,
        1.0,
        0.0,
    )

    session_text = (
        ROOT
        / "configs/sessions/"
        "isaac_nero_dual_hand2_tframe_gripper_flange_collision_proxy_"
        "triview_q54_self_collision_v1.yaml"
    ).read_text(encoding="utf-8")
    assert "task_scene" not in session_text
    assert "banana" not in session_text


def test_both_isaac_processes_receive_the_same_task_scene_receipt() -> None:
    launch = (ROOT / "ros2/wujihand_ros2/launch/dual_teleoperation.launch.py").read_text(
        encoding="utf-8"
    )
    consumer = (ROOT / "tools/run_isaac_nero_hand2_ros.py").read_text(encoding="utf-8")
    preview = (ROOT / "tools/run_isaac_dataset_live_preview.py").read_text(encoding="utf-8")
    qualifier = (ROOT / "tools/qualify_dataset_preview_e2e.py").read_text(encoding="utf-8")
    validator = (ROOT / "tools/validate_dataset_preview_fixture_qualification.py").read_text(
        encoding="utf-8"
    )

    assert 'consumer_command.extend(["--chain-preflight"' in launch
    assert '"--dataset-source-mode",' in launch
    assert "Description 8.3 recording remains qualification-only" not in consumer
    assert "preview_command.extend(" in launch
    assert '"--chain-preflight", str(chain_preflight_path)' in launch
    assert '"--wait-for-node"' in launch
    assert "DATASET LIVE PREVIEW WAITING" in preview
    assert "_wait_for_ros_node(" in preview
    assert "resolve_record_chain_workcell_plan" in consumer
    assert "resolve_record_chain_workcell_plan" in preview
    assert "load_nero_dual_simulation_startup_profile" in consumer
    assert "if QUALIFICATION.teleport_to_initial_position:" in consumer
    assert "load_nero_dual_simulation_startup_profile" in preview
    assert "PREVIEW_BACKGROUND_COLOR_RGB = (0.50, 0.50, 0.50)" in preview
    assert "PREVIEW_ANTI_ALIASING_MODE = 2" in preview
    assert "PREVIEW_MINIMAL_SHADING_MODE = 2" in preview
    assert "PREVIEW_THREAD_LIMIT_CAP = 10" in preview
    assert "ISAAC_CPU_THREAD_LIMIT_CAP = 14" in consumer
    assert 'preview_settings.set_bool("/rtx/shadows/enabled", False)' in preview
    assert 'preview_settings.set_bool("/rtx/ambientOcclusion/enabled", False)' in preview
    assert '"render_under_period"' in preview
    assert '"--record-chain-qualification"' in qualifier
    assert '["python3", str(PREVIEW_VALIDATOR), "--run-root", str(run_root)]' in qualifier
    assert "triview_q54_self_collision_120_30_15_v1" in validator
    assert '"task_scene_and_preview_visual_policy"' in validator
