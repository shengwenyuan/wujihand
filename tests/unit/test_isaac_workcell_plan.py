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
    )
    assert mapping["fixed_rigid_body_paths"] == list(
        plan.fixed_rigid_body_paths
    )
    encoded = json.dumps(mapping, sort_keys=True)
    assert str(ROOT) not in encoded
    assert "local_runtime_path" not in encoded


def test_tframe_composes_the_banana_task_scene_without_changing_session() -> None:
    repository = ConfigRepository(ROOT)
    workcell = repository.load_workcell(
        "configs/workcells/isaac_dual_nero_tframe_candidate_20260811_v1.yaml"
    )
    task_scene = (
        "configs/scenes/isaac_robolab_banana_bowl_low_table_v2.yaml"
    )

    base_plan = resolve_isaac_workcell_plan(ROOT, workcell)
    task_plan = resolve_isaac_workcell_plan(
        ROOT,
        workcell,
        task_scene=task_scene,
    )

    assert base_plan.task_scene_profile_id is None
    assert len(base_plan.imports) == 1
    assert task_plan.workcell_id == base_plan.workcell_id
    assert task_plan.profile_id == base_plan.profile_id
    assert task_plan.task_scene_profile_id == (
        "isaac_robolab_banana_bowl_low_table_v2"
    )
    assert task_plan.task_scene_profile_path == task_scene
    assert tuple(operation.import_id for operation in task_plan.imports) == (
        "dual_nero_tframe",
        "robolab_banana_bowl_task",
    )
    task_import = task_plan.imports[1]
    assert task_import.pose.position_m == pytest.approx(
        (-0.0000582, 0.0028054, 0.1970015)
    )
    assert task_import.excluded_prim_paths == (
        "GroundPlane",
        "franka_table",
        "table",
    )
    assert task_plan.fixed_rigid_body_paths == ()
    assert dict(task_plan.dynamic_rigid_body_paths) == {
        "banana": "/World/Environment/robolab_banana_bowl_task/banana",
        "bowl": "/World/Environment/robolab_banana_bowl_task/bowl",
    }
    task_entities = {
        operation.entity_id: operation
        for operation in task_plan.primitives
        if operation.entity_id != "ground"
    }
    assert set(task_entities) == {
        "banana_task_table_top",
        "banana_task_table_near_left_leg",
        "banana_task_table_near_right_leg",
        "banana_task_table_far_left_leg",
        "banana_task_table_far_right_leg",
    }
    assert task_entities["banana_task_table_top"].pose.position_m == (
        0.0,
        0.55,
        0.18,
    )
    for entity_id, operation in task_entities.items():
        if not entity_id.endswith("_leg"):
            continue
        assert operation.pose.position_m[2] == pytest.approx(0.09)
        assert operation.entity.primitive.size_m is not None
        assert (
            operation.pose.position_m[2]
            - operation.entity.primitive.size_m[2] / 2.0
        ) == pytest.approx(0.0)


def test_tframe_composes_a_second_robolab_scene_without_object_special_cases() -> None:
    repository = ConfigRepository(ROOT)
    workcell = repository.load_workcell(
        "configs/workcells/isaac_dual_nero_tframe_candidate_20260811_v1.yaml"
    )

    plan = resolve_isaac_workcell_plan(
        ROOT,
        workcell,
        task_scene="configs/scenes/isaac_robolab_colored_blocks_low_table_v1.yaml",
    )

    assert plan.task_scene_profile_id == (
        "isaac_robolab_colored_blocks_low_table_v1"
    )
    assert plan.fixed_rigid_body_paths == ()
    assert dict(plan.dynamic_rigid_body_paths) == {
        color: f"/World/Environment/robolab_colored_blocks_task/{color}"
        for color in (
            "blue_block",
            "green_block",
            "red_block",
            "yellow_block",
        )
    }


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
