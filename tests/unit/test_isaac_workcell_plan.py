from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from wujihand.runtime import (
    ConfigRepository,
    SessionResolver,
    resolve_isaac_workcell_plan,
)
from wujihand.runtime.isaac_dual_scene import (
    workcell_frame_position,
    workcell_pose,
)


ROOT = Path(__file__).parents[2]


def test_primitive_workcell_compiles_without_a_backend_profile() -> None:
    repository = ConfigRepository(ROOT)
    workcell = repository.load_workcell(
        "configs/workcells/isaac_hand2_table_v1.yaml"
    )

    plan = resolve_isaac_workcell_plan(ROOT, workcell)

    assert plan.profile_id is None
    assert plan.imports == ()
    assert plan.policies.ground == "project"
    assert {operation.entity_id for operation in plan.primitives} == {
        "ground",
        "table",
    }


def test_robolab_workcell_compiles_to_content_identity_and_hybrid_ops() -> None:
    repository = ConfigRepository(ROOT)
    workcell = repository.load_workcell(
        "configs/workcells/isaac_robolab_banana_bowl_dual_station_v1.yaml"
    )

    plan = resolve_isaac_workcell_plan(ROOT, workcell)
    mapping = plan.to_mapping()

    assert plan.profile_id == "isaac_robolab_banana_bowl_v1"
    assert plan.imports[0].pose.position_m == (0.0, -0.52, 0.8)
    assert plan.imports[0].pose.quat_wxyz == pytest.approx(
        (2**-0.5, 0.0, 0.0, 2**-0.5)
    )
    assert plan.imports[0].content.relative_path == (
        "assets/scenes/banana_bowl.usda"
    )
    assert plan.lighting.content is not None
    assert plan.lighting.content.relative_path == (
        "assets/backgrounds/indoors/photo_studio_01_2k.hdr"
    )
    assert plan.lighting.intensity == 800.0
    assert plan.lighting.exposure == 0.0
    assert plan.lighting.visible_in_primary_ray is False
    assert plan.lighting.background_color_rgb == (0.12, 0.12, 0.12)
    assert plan.primitives == ()
    assert plan.fixed_rigid_body_paths == (
        "/World/Environment/robolab_banana_bowl/table",
        "/World/Environment/robolab_banana_bowl/bowl",
    )
    assert mapping["fixed_rigid_body_paths"] == list(
        plan.fixed_rigid_body_paths
    )
    encoded = json.dumps(mapping, sort_keys=True)
    assert str(ROOT) not in encoded
    assert "local_runtime_path" not in encoded


def test_each_robolab_layout_is_a_distinct_resolved_session() -> None:
    resolver = SessionResolver(ROOT)
    paths = (
        "configs/sessions/isaac_nero_dual_hand2_robolab_base_empty_v1.yaml",
        "configs/sessions/isaac_nero_dual_hand2_robolab_banana_bowl_v1.yaml",
        "configs/sessions/isaac_nero_dual_hand2_robolab_workdesk_v1.yaml",
    )

    sessions = tuple(resolver.resolve(path) for path in paths)

    assert len({session.session_hash for session in sessions}) == len(paths)
    assert {
        session.workcell.mount("table_near_left").mount_id
        for session in sessions
    } == {"table_near_left"}


def test_banana_bowl_teleop_keeps_the_qualified_mount_baseline() -> None:
    session = SessionResolver(ROOT).resolve(
        "configs/sessions/"
        "isaac_nero_dual_hand2_robolab_banana_bowl_teleop_v1.yaml"
    )
    baseline = SessionResolver(ROOT).resolve(
        "configs/sessions/"
        "isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml"
    )
    left_mount = session.workcell.mount("table_near_left")
    right_mount = session.workcell.mount("table_near_right")
    baseline_left_mount = baseline.workcell.mount(
        "nero_left_simulation_nominal_mount"
    )
    baseline_right_mount = baseline.workcell.mount(
        "nero_right_simulation_nominal_mount"
    )
    left = left_mount.transform.position_m
    right = right_mount.transform.position_m

    assert session.session.runtime_role == "teleop_consumer"
    assert session.session.runtime.transport_contract == (
        "wujihand.dual_teleoperation.v1"
    )
    assert left_mount.frame == "simulation_nominal_table_top"
    assert right_mount.frame == "simulation_nominal_table_top"
    assert left == pytest.approx((-0.32, -0.52, 0.0))
    assert right == pytest.approx((0.32, -0.52, 0.0))
    assert right[0] - left[0] == pytest.approx(0.64)
    assert left[1:] == right[1:]
    assert workcell_pose(
        session,
        left_mount.frame,
        left_mount.transform,
    ) == workcell_pose(
        baseline,
        baseline_left_mount.frame,
        baseline_left_mount.transform,
    )
    assert workcell_pose(
        session,
        right_mount.frame,
        right_mount.transform,
    ) == workcell_pose(
        baseline,
        baseline_right_mount.frame,
        baseline_right_mount.transform,
    )


def test_banana_bowl_scene_camera_is_centered_and_covers_task_table() -> None:
    session = SessionResolver(ROOT).resolve(
        "configs/sessions/"
        "isaac_nero_dual_hand2_triview_q54_mini_dataset_v1.yaml"
    )
    left = workcell_pose(
        session,
        session.workcell.mount("table_near_left").frame,
        session.workcell.mount("table_near_left").transform,
    ).position_m
    right = workcell_pose(
        session,
        session.workcell.mount("table_near_right").frame,
        session.workcell.mount("table_near_right").transform,
    ).position_m
    eye = workcell_frame_position(
        session,
        "simulation_nominal_camera_oblique_eye",
    )
    target = workcell_frame_position(
        session,
        "simulation_nominal_camera_oblique_target",
    )

    assert eye == pytest.approx((0.0, -0.52, 1.50))
    assert eye[:2] == pytest.approx(
        tuple((left[index] + right[index]) / 2.0 for index in range(2))
    )
    assert target == pytest.approx((0.0, 0.03, 0.80))
    downward_pitch_deg = math.degrees(
        math.atan2(eye[2] - target[2], target[1] - eye[1])
    )
    assert downward_pitch_deg == pytest.approx(51.8427734126)

    forward_delta = tuple(target[index] - eye[index] for index in range(3))
    forward_norm = math.sqrt(sum(value**2 for value in forward_delta))
    forward = tuple(value / forward_norm for value in forward_delta)
    right = (1.0, 0.0, 0.0)
    up = (0.0, -forward[2], forward[1])

    def dot(first: tuple[float, ...], second: tuple[float, ...]) -> float:
        return sum(lhs * rhs for lhs, rhs in zip(first, second, strict=True))

    horizontal_half_fov_deg = 45.0
    vertical_half_fov_deg = math.degrees(
        math.atan(math.tan(math.radians(horizontal_half_fov_deg)) * 480.0 / 640.0)
    )
    # Source-locked table_oak world bounds after the +90-degree Workcell alignment.
    tabletop_corners = (
        (x, y, 0.8029985)
        for x in (-0.5, 0.5)
        for y in (-0.3228054, 0.3771946)
    )
    for corner in tabletop_corners:
        relative = tuple(corner[index] - eye[index] for index in range(3))
        depth = dot(relative, forward)
        horizontal_angle_deg = math.degrees(
            math.atan2(abs(dot(relative, right)), depth)
        )
        vertical_angle_deg = math.degrees(
            math.atan2(abs(dot(relative, up)), depth)
        )
        assert depth > 0.0
        assert horizontal_angle_deg < horizontal_half_fov_deg
        assert vertical_angle_deg < vertical_half_fov_deg
