from __future__ import annotations

import math

import numpy as np
import pytest

from wujihand.adapters.simulation.hand2_ball_scene import (
    Hand2BallConfig,
    contact_groups_from_force_matrix,
    default_ball_contact_filters,
)
from wujihand.adapters.simulation.hand2_grasp import (
    BallLiftCriteria,
    BallLiftEvaluator,
    BallLiftSample,
)
from wujihand.adapters.simulation.hand2_rotation_mount import (
    Hand2RotationMountConfig,
    discover_rotation_mount_dofs,
    principal_axes_joint_frame_quaternion,
    quaternion_wxyz_to_d6_rpy_degrees,
    unwrap_periodic_degrees,
)
from wujihand.domain import HAND2_RIGHT_LAYOUT
from wujihand.domain.pose import multiply_quaternions_wxyz


def test_default_configs_are_physically_consistent() -> None:
    mount = Hand2RotationMountConfig()
    ball = Hand2BallConfig()

    assert mount.flange_position_m[2] > ball.table_top_z_m
    assert ball.center_xyz_m[2] - ball.radius_m == pytest.approx(ball.table_top_z_m)
    assert mount.roll_limit_rad < math.pi / 2.0
    assert mount.pitch_limit_rad < math.pi / 2.0


@pytest.mark.parametrize(
    "kwargs",
    [
        {"flange_orientation_wxyz": (2.0, 0.0, 0.0, 0.0)},
        {"roll_limit_rad": math.pi / 2.0},
        {"drive_max_force": math.inf},
        {"mount_prim_path": "World/Hand2Mount"},
    ],
)
def test_rotation_mount_config_rejects_unsafe_values(kwargs: dict[str, object]) -> None:
    with pytest.raises(ValueError):
        Hand2RotationMountConfig(**kwargs)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "kwargs",
    [
        {"center_xyz_m": (0.15, 0.0, 0.39)},
        {"radius_m": 0.0},
        {"mass_kg": -0.1},
        {"static_friction": 0.5, "dynamic_friction": 0.8},
        {"restitution": 1.1},
    ],
)
def test_ball_config_rejects_invalid_or_interpenetrating_values(
    kwargs: dict[str, object],
) -> None:
    with pytest.raises(ValueError):
        Hand2BallConfig(**kwargs)  # type: ignore[arg-type]


def test_contact_force_matrix_is_classified_by_explicit_filter_order() -> None:
    filters = default_ball_contact_filters()
    forces = np.zeros((1, len(filters.labels), 3), dtype=np.float64)
    forces[0, filters.labels.index("thumb"), 2] = 0.2
    forces[0, filters.labels.index("middle"), 0] = -0.1
    forces[0, -1, 2] = 0.049

    contacts = contact_groups_from_force_matrix(forces, filters, threshold_n=0.05)

    assert contacts == frozenset({"thumb", "middle"})


def _lift_sample(
    time_s: float,
    *,
    contacts: frozenset[str] = frozenset({"thumb", "index", "middle"}),
    relative: tuple[float, float, float] | None = (0.15, 0.0, -0.06),
) -> BallLiftSample:
    return BallLiftSample(
        time_s=time_s,
        ball_center_xyz_m=(0.15, 0.0, 0.43),
        contact_groups=contacts,
        ball_in_palm_xyz_m=relative,
    )


def test_ball_lift_evaluator_requires_continuous_physx_contact_hold() -> None:
    evaluator = BallLiftEvaluator(BallLiftCriteria(min_hold_s=0.5))

    starting = evaluator.update(_lift_sample(1.0))
    passing = evaluator.update(_lift_sample(1.5, relative=(0.151, 0.0, -0.06)))

    assert not starting.passed
    assert starting.qualified
    assert starting.reasons == ("hold_window_incomplete",)
    assert passing.passed
    assert passing.qualified
    assert passing.hold_duration_s == pytest.approx(0.5)
    assert passing.palm_relative_slip_m == pytest.approx(0.001)


def test_ball_lift_evaluator_resets_for_table_contact_or_slip() -> None:
    evaluator = BallLiftEvaluator(BallLiftCriteria(min_hold_s=0.2))
    evaluator.update(_lift_sample(0.0))

    table = evaluator.update(
        _lift_sample(0.2, contacts=frozenset({"thumb", "index", "middle", "table"}))
    )
    restarted = evaluator.update(_lift_sample(0.3))
    slipped = evaluator.update(_lift_sample(0.5, relative=(0.16, 0.0, -0.06)))

    assert not table.passed
    assert not table.qualified
    assert "table_contact_present" in table.reasons
    assert restarted.hold_duration_s == 0.0
    assert not slipped.passed
    assert not slipped.qualified
    assert "palm_relative_slip_exceeded" in slipped.reasons
    assert slipped.hold_duration_s == 0.0


def test_rotation_mount_dofs_are_discovered_by_name_and_path_not_position() -> None:
    finger_names = list(HAND2_RIGHT_LAYOUT.names)
    dof_names = (
        finger_names[10:]
        + [
            "wrist_rotation_joint:rotZ",
            "wrist_rotation_joint:rotX",
        ]
        + finger_names[:10]
        + ["wrist_rotation_joint:rotY"]
    )
    rotation_path = "/World/Hand2Mount/wrist_rotation_joint"
    dof_paths = [
        rotation_path if name.startswith("wrist_rotation_joint") else f"/World/Hand2/joints/{name}"
        for name in dof_names
    ]

    partition = discover_rotation_mount_dofs(
        dof_names, dof_paths, HAND2_RIGHT_LAYOUT.names, rotation_path
    )

    assert tuple(dof_names[index] for index in partition.wrist_indices_xyz) == (
        "wrist_rotation_joint:rotX",
        "wrist_rotation_joint:rotY",
        "wrist_rotation_joint:rotZ",
    )
    assert tuple(dof_names[index] for index in partition.finger_indices_q20) == tuple(
        HAND2_RIGHT_LAYOUT.names
    )


def test_rotation_mount_dof_discovery_fails_closed_on_wrong_path() -> None:
    finger_names = list(HAND2_RIGHT_LAYOUT.names)
    names = ["mount:rotX", "mount:rotY", "mount:rotZ"] + finger_names
    rotation_path = "/World/Hand2Mount/wrist_rotation_joint"
    paths = [rotation_path] * 3 + [f"/World/Hand2/joints/{name}" for name in finger_names]
    paths[3] = "/World/Hand2/joints/not_the_named_joint"

    with pytest.raises(RuntimeError, match="unexpected articulation DOFs"):
        discover_rotation_mount_dofs(names, paths, finger_names, rotation_path)


def test_rotation_mount_dof_discovery_accepts_isaac_51_numeric_d6_suffixes() -> None:
    rotation_path = "/World/Hand2Mount/wrist_rotation_joint"
    names = ["wrist_rotation_joint:0", "wrist_rotation_joint:1"] + list(
        HAND2_RIGHT_LAYOUT.names
    ) + ["wrist_rotation_joint:2"]
    paths = [
        rotation_path if name.startswith("wrist_rotation_joint:") else f"/World/Hand2/{name}"
        for name in names
    ]

    partition = discover_rotation_mount_dofs(
        names,
        paths,
        HAND2_RIGHT_LAYOUT.names,
        rotation_path,
    )

    assert partition.wrist_indices_xyz == (0, 1, 22)
    assert partition.wrist_names_xyz == (
        "wrist_rotation_joint:0",
        "wrist_rotation_joint:1",
        "wrist_rotation_joint:2",
    )


def test_anchor_local_quaternion_maps_to_d6_intrinsic_zyx_degrees() -> None:
    half_angle = math.radians(45.0)
    yaw_90 = (math.cos(half_angle), 0.0, 0.0, math.sin(half_angle))

    roll, pitch, yaw = quaternion_wxyz_to_d6_rpy_degrees(yaw_90)

    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(90.0)


def test_periodic_yaw_target_unwraps_across_half_turn() -> None:
    assert unwrap_periodic_degrees(-179.0, 179.0) == pytest.approx(181.0)
    assert unwrap_periodic_degrees(1.0, 359.0) == pytest.approx(361.0)
    assert unwrap_periodic_degrees(179.0, -179.0) == pytest.approx(-181.0)


def test_principal_axes_rotation_is_split_equally_across_d6_frames() -> None:
    principal = np.asarray((0.9554235, -0.22348082, 0.17272286, -0.08596107))

    half = principal_axes_joint_frame_quaternion(principal)

    np.testing.assert_allclose(
        multiply_quaternions_wxyz(half, half),
        principal / np.linalg.norm(principal),
        atol=1e-7,
    )
    np.testing.assert_allclose(
        principal_axes_joint_frame_quaternion(-principal),
        half,
        atol=1e-7,
    )
