from __future__ import annotations

from pathlib import Path

from wujihand.domain import HandSide
from wujihand.runtime import RosDeploymentResolver
from wujihand.runtime.wuji_hand2_record_chain import (
    load_record_chain_qualification_policy,
)

ROOT = Path(__file__).resolve().parents[2]
LOCAL = "configs/examples/workstation2_nv5_ros_local_runtime_binding.example.yaml"
BASELINE = (
    "configs/deployments/isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v2026_8_3_v1.yaml"
)
TFRAME = "configs/deployments/isaac_nero_hand2_ros_dual_tframe_triview_q54_v2026_8_3_v1.yaml"
TFRAME_SELF_COLLISION = (
    "configs/deployments/"
    "isaac_nero_hand2_ros_dual_tframe_triview_q54_self_collision_v2026_8_3_v1.yaml"
)
TFRAME_GRIPPER_FLANGE_SELF_COLLISION = (
    "configs/deployments/"
    "isaac_nero_hand2_ros_dual_tframe_gripper_flange_triview_q54_"
    "self_collision_v2026_8_3_v1.yaml"
)
TFRAME_GRIPPER_FLANGE_COLLISION_PROXY_SELF_COLLISION = (
    "configs/deployments/"
    "isaac_nero_hand2_ros_dual_tframe_gripper_flange_collision_proxy_"
    "triview_q54_self_collision_v1.yaml"
)
POLICY = ROOT / "configs/qualifications/isaac_nero_hand2_tframe_record_chain_v2026_8_3_v1.yaml"


def test_tframe_entry_reuses_the_validated_ros_recording_graph() -> None:
    resolver = RosDeploymentResolver(ROOT)
    baseline = resolver.resolve(BASELINE, local_binding=LOCAL, verify_artifacts=False)
    tframe = resolver.resolve(TFRAME, local_binding=LOCAL, verify_artifacts=False)

    assert tframe.deployment.processes == baseline.deployment.processes
    assert tframe.deployment.sources == baseline.deployment.sources
    assert tframe.deployment.control_bindings == baseline.deployment.control_bindings
    assert tframe.deployment.node_bindings == baseline.deployment.node_bindings
    assert tframe.deployment.qos_profile == baseline.deployment.qos_profile
    assert tframe.deployment.root_namespace == baseline.deployment.root_namespace
    assert tframe.session.session.dataset_profile == baseline.session.session.dataset_profile
    assert tframe.control_profile.physics_hz == baseline.control_profile.physics_hz == 120
    assert tframe.control_profile.tracker == baseline.control_profile.tracker
    assert tframe.control_profile.kinematics == baseline.control_profile.kinematics
    assert tframe.control_profile.arm_supervision == baseline.control_profile.arm_supervision
    assert tframe.control_profile.glove == baseline.control_profile.glove
    assert tframe.control_profile.hand_supervision == baseline.control_profile.hand_supervision

    assert tframe.session.workcell.workcell_id == ("isaac_dual_nero_tframe_candidate_20260811_v1")
    assert tframe.session.assembly.assembly_id == (
        "nero_dual_hand2_d405_wrist_rig_tframe_v2026_8_3_v1"
    )
    assert "tabletop" not in tframe.mapping.provenance
    assert tframe.deployment.tracking_setup.qualification_status == "pending"


def test_self_collision_entry_only_changes_session_runtime_policy() -> None:
    resolver = RosDeploymentResolver(ROOT)
    baseline = resolver.resolve(TFRAME, local_binding=LOCAL, verify_artifacts=False)
    enabled = resolver.resolve(
        TFRAME_SELF_COLLISION,
        local_binding=LOCAL,
        verify_artifacts=False,
    )

    assert enabled.deployment.processes == baseline.deployment.processes
    assert enabled.deployment.sources == baseline.deployment.sources
    assert enabled.deployment.control_bindings == baseline.deployment.control_bindings
    assert enabled.route_plan.routes == baseline.route_plan.routes
    assert baseline.self_collision_profile_path is None
    assert enabled.self_collision_profile_id == (
        "isaac_nero_hand2_self_collision_filtered_pairs_v2026_8_3_v1"
    )
    assert enabled.deployment_hash != baseline.deployment_hash


def test_gripper_flange_entry_only_replaces_scene_owned_flange_facts() -> None:
    resolver = RosDeploymentResolver(ROOT)
    previous = resolver.resolve(
        TFRAME_SELF_COLLISION,
        local_binding=LOCAL,
        verify_artifacts=False,
    )
    current = resolver.resolve(
        TFRAME_GRIPPER_FLANGE_SELF_COLLISION,
        local_binding=LOCAL,
        verify_artifacts=False,
    )

    assert current.deployment.processes == previous.deployment.processes
    assert current.deployment.sources == previous.deployment.sources
    assert current.deployment.control_bindings == previous.deployment.control_bindings
    assert current.deployment.node_bindings == previous.deployment.node_bindings
    assert current.deployment.qos_profile == previous.deployment.qos_profile
    assert current.route_plan.routes == previous.route_plan.routes
    assert current.control_profile == previous.control_profile
    assert current.session.session.dataset_profile == previous.session.session.dataset_profile
    assert current.session.assembly.assembly_id == (
        "nero_dual_hand2_d405_wrist_rig_tframe_gripper_flange_candidate_v2026_8_3_v1"
    )
    assert len(current.session.instances) == 8
    assert current.session.instance("nero_left").asset.asset_id == (
        "agilex_nero_gripper_flange"
    )
    assert current.session.instance("nero_right").binding.binding_id == (
        "agilex_nero_gripper_flange_f6642ce0_isaac_6_0_1_v1"
    )
    assert current.self_collision_profile_id == (
        "isaac_nero_hand2_self_collision_filtered_pairs_gripper_flange_v2026_8_3_v1"
    )


def test_collision_proxy_entry_only_replaces_the_arm_collision_representation() -> None:
    resolver = RosDeploymentResolver(ROOT)
    previous = resolver.resolve(
        TFRAME_GRIPPER_FLANGE_SELF_COLLISION,
        local_binding=LOCAL,
        verify_artifacts=False,
    )
    current = resolver.resolve(
        TFRAME_GRIPPER_FLANGE_COLLISION_PROXY_SELF_COLLISION,
        local_binding=LOCAL,
        verify_artifacts=False,
    )

    assert current.deployment.processes == previous.deployment.processes
    assert current.deployment.sources == previous.deployment.sources
    assert current.deployment.control_bindings == previous.deployment.control_bindings
    assert current.deployment.node_bindings == previous.deployment.node_bindings
    assert current.deployment.qos_profile == previous.deployment.qos_profile
    assert current.route_plan.routes == previous.route_plan.routes
    assert current.control_profile == previous.control_profile
    assert current.session.session.dataset_profile == previous.session.session.dataset_profile
    assert current.session.assembly.assembly_id == (
        "nero_dual_hand2_d405_wrist_rig_tframe_gripper_flange_collision_proxy_v1"
    )
    for side in ("left", "right"):
        assert current.session.instance(f"nero_{side}").asset.asset_id == (
            "agilex_nero_gripper_flange_collision_proxy"
        )
    assert current.self_collision_profile_id == (
        "isaac_nero_hand2_self_collision_filtered_pairs_gripper_flange_collision_proxy_v1"
    )


def test_tframe_record_policy_keeps_task_scene_outside_the_session() -> None:
    policy = load_record_chain_qualification_policy(POLICY)

    assert policy.task_scene is not None
    assert policy.task_scene.path == ("configs/scenes/isaac_robolab_banana_bowl_low_table_v2.yaml")
    assert policy.nero.assembly_attachment_quaternion(HandSide.LEFT) == (
        0.5,
        -0.5,
        -0.5,
        0.5,
    )
    assert policy.nero.assembly_attachment_quaternion(HandSide.RIGHT) == (
        0.5,
        -0.5,
        -0.5,
        0.5,
    )

    session_text = (
        ROOT / "configs/sessions/isaac_nero_dual_hand2_tframe_triview_q54_v2026_8_3_v1.yaml"
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
    assert "PREVIEW_BACKGROUND_COLOR_RGB = (0.30, 0.30, 0.30)" in preview
    assert '"render_under_50_ms"' in preview
    assert '"--record-chain-qualification"' in qualifier
    assert '["python3", str(PREVIEW_VALIDATOR), "--run-root", str(run_root)]' in qualifier
    assert "isaac_nero_hand2_ros_dual_tframe_triview_q54_v2026_8_3_v1" in validator
    assert "gripper_flange_collision_proxy_" in validator
    assert '"task_scene_and_preview_visual_policy"' in validator
