from __future__ import annotations

from pathlib import Path

from wujihand.runtime import (
    ConfigRepository,
    RosDeploymentResolver,
)


ROOT = Path(__file__).parents[2]
FULL = (
    ROOT
    / "configs/deployments/isaac_nero_hand2_ros_dual_live_v2.yaml"
)
ARMS = (
    ROOT
    / "configs/deployments/"
    "isaac_nero_hand2_ros_dual_arm_only_live_v2.yaml"
)
BANANA_BOWL = (
    ROOT
    / "configs/deployments/"
    "isaac_nero_hand2_ros_dual_robolab_banana_bowl_live_v2.yaml"
)
LOCAL = (
    ROOT
    / "configs/examples/"
    "workstation2_nv5_ros_local_runtime_binding.example.yaml"
)


def test_ros_resolver_closes_full_and_arm_only_graphs() -> None:
    resolver = RosDeploymentResolver(ROOT)
    local = ConfigRepository(ROOT).load_ros_local_runtime_binding(LOCAL)

    full = resolver.resolve(FULL, local_binding=local)
    arms = resolver.resolve(ARMS, local_binding=local)

    assert full.session.session.session_id == (
        "isaac_nero_dual_hand2_teleop_v1"
    )
    assert full.qos_profile.policy("tracker_sample").depth == 1
    assert len(full.route_plan.routes) == 4
    assert len(full.deployment_hash) == 64
    assert len(full.local_binding_hash) == 64
    assert {
        route.source.kind for route in arms.route_plan.routes
    } == {"vive_tracker", "hand_rest_fixture"}


def test_ros_resolver_closes_banana_bowl_rich_workcell() -> None:
    resolver = RosDeploymentResolver(ROOT)
    local = ConfigRepository(ROOT).load_ros_local_runtime_binding(LOCAL)

    resolved = resolver.resolve(BANANA_BOWL, local_binding=local)

    assert resolved.session.session.session_id == (
        "isaac_nero_dual_hand2_robolab_banana_bowl_teleop_v1"
    )
    assert resolved.session.workcell.workcell_id == (
        "isaac_robolab_banana_bowl_dual_station_v1"
    )
    assert len(resolved.route_plan.routes) == 4
    assert resolved.deployment.report_root.endswith(
        "robolab-banana-bowl"
    )
