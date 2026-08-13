from __future__ import annotations

from pathlib import Path

from wujihand.runtime import (
    ConfigRepository,
    RosDeploymentResolver,
)


ROOT = Path(__file__).parents[2]
FULL = ROOT / "configs/deployments/isaac_nero_hand2_ros_dual_live_v2.yaml"
ARMS = ROOT / "configs/deployments/isaac_nero_hand2_ros_dual_arm_only_live_v2.yaml"
BANANA_BOWL = (
    ROOT / "configs/deployments/isaac_nero_hand2_ros_dual_robolab_banana_bowl_live_v2.yaml"
)
LOCAL = ROOT / "configs/examples/workstation2_nv5_ros_local_runtime_binding.example.yaml"
TFRAME_SELF_COLLISION = (
    ROOT / "configs/deployments/"
    "isaac_nero_hand2_ros_dual_tframe_gripper_flange_collision_proxy_"
    "triview_q54_self_collision_v1.yaml"
)


def test_ros_resolver_closes_full_and_arm_only_graphs() -> None:
    resolver = RosDeploymentResolver(ROOT)
    local = ConfigRepository(ROOT).load_ros_local_runtime_binding(LOCAL)

    full = resolver.resolve(FULL, local_binding=local)
    arms = resolver.resolve(ARMS, local_binding=local)

    assert full.session.session.session_id == ("isaac_nero_dual_hand2_d405_wrist_rig_teleop_v1")
    assert len(full.session.instances) == 8
    assert full.qos_profile.policy("camera_image").reliability == "reliable"
    assert full.qos_profile.policy("tracker_sample").depth == 1
    assert len(full.route_plan.routes) == 4
    assert len(full.deployment_hash) == 64
    assert len(full.local_binding_hash) == 64
    assert {route.source.kind for route in arms.route_plan.routes} == {
        "vive_tracker",
        "hand_rest_fixture",
    }


def test_ros_resolver_closes_banana_bowl_rich_workcell() -> None:
    resolver = RosDeploymentResolver(ROOT)
    local = ConfigRepository(ROOT).load_ros_local_runtime_binding(LOCAL)

    resolved = resolver.resolve(BANANA_BOWL, local_binding=local)

    assert resolved.session.session.session_id == (
        "isaac_nero_dual_hand2_d405_wrist_rig_robolab_banana_bowl_teleop_v1"
    )
    assert resolved.session.workcell.workcell_id == ("isaac_robolab_banana_bowl_dual_station_v1")
    assert len(resolved.route_plan.routes) == 4
    assert resolved.deployment.report_root.endswith("robolab-banana-bowl")


def test_ros_resolver_closes_session_owned_self_collision_profile() -> None:
    resolved = RosDeploymentResolver(ROOT).resolve(
        TFRAME_SELF_COLLISION,
        local_binding=LOCAL,
        verify_artifacts=False,
    )

    assert resolved.self_collision_profile_id == (
        "isaac_nero_hand2_self_collision_filtered_pairs_gripper_flange_collision_proxy_v1"
    )
    assert resolved.self_collision_profile_path == (
        "configs/profiles/"
        "isaac_nero_hand2_self_collision_filtered_pairs_gripper_flange_collision_proxy_v1.yaml"
    )
    assert len(resolved.self_collision_profile_sha256 or "") == 64
