from __future__ import annotations

from typing import Any
from pathlib import Path

import numpy as np
import pytest

from wujihand.dataset.profile import load_q54_joint_profile
from wujihand.domain.dataset_recording import SimulationFramePhase
from wujihand.runtime.isaac_dual_scene import (
    DualNeroHand2IsaacScene,
    SceneKinematicLinkSnapshot,
    SceneRigidBodySnapshot,
)


ROOT = Path(__file__).parents[2]


class _FakeArticulation:
    def __init__(self) -> None:
        self.positions: np.ndarray | None = None
        self.velocities: np.ndarray | None = None

    def set_joint_positions(self, values: np.ndarray) -> None:
        self.positions = values.copy()

    def set_joint_velocities(self, values: np.ndarray) -> None:
        self.velocities = values.copy()

    def get_joint_velocities(self) -> np.ndarray:
        return np.arange(27, dtype=np.float64)[np.newaxis, :]


class _FakeRigidPrim:
    def __init__(self) -> None:
        self.pose: tuple[np.ndarray, np.ndarray] | None = None
        self.linear_velocity: np.ndarray | None = None
        self.angular_velocity: np.ndarray | None = None

    def set_world_pose(self, *, position: np.ndarray, orientation: np.ndarray) -> None:
        self.pose = (position.copy(), orientation.copy())

    def set_linear_velocity(self, value: np.ndarray) -> None:
        self.linear_velocity = value.copy()

    def set_angular_velocity(self, value: np.ndarray) -> None:
        self.angular_velocity = value.copy()


def _scene_without_isaac(monkeypatch: pytest.MonkeyPatch) -> DualNeroHand2IsaacScene:
    scene = DualNeroHand2IsaacScene.__new__(DualNeroHand2IsaacScene)
    monkeypatch.setattr(scene, "rigid_body_snapshots", lambda: ())
    return scene


def test_camera_replay_snapshot_reuses_post_physics_feedback(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    scene = _scene_without_isaac(monkeypatch)
    left = np.arange(27, dtype=np.float64)
    right = -left

    snapshot = scene.camera_replay_snapshot(
        q27_by_side={"left": left, "right": right},
    )

    assert dict(snapshot.q27_by_side) == {
        "left": tuple(left),
        "right": tuple(right),
    }
    assert snapshot.rigid_bodies == ()


def test_camera_replay_snapshot_validates_and_restores_q27(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    scene = _scene_without_isaac(monkeypatch)
    with pytest.raises(ValueError, match="cover left and right"):
        scene.camera_replay_snapshot(q27_by_side={"left": np.zeros(27)})
    with pytest.raises(ValueError, match="invalid right"):
        scene.camera_replay_snapshot(
            q27_by_side={"left": np.zeros(27), "right": np.zeros(26)},
        )

    snapshot = scene.camera_replay_snapshot(
        q27_by_side={"left": np.ones(27), "right": -np.ones(27)},
    )
    articulations = {side: _FakeArticulation() for side in ("left", "right")}
    scene.articulations = articulations  # type: ignore[assignment]
    scene.dynamic_workcell_prims = {}

    scene.restore_camera_replay_snapshot(snapshot)

    for side, expected in (("left", 1.0), ("right", -1.0)):
        articulation: Any = articulations[side]
        assert articulation.positions is not None
        assert articulation.positions.shape == (1, 27)
        assert np.all(articulation.positions == expected)
        assert articulation.velocities is not None
        assert np.all(articulation.velocities == 0.0)


def test_dataset_state_uses_backend_qdot_and_restores_exact_pre_action(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    profile = load_q54_joint_profile(
        ROOT,
        "configs/profiles/isaac_nero_hand2_q54_dataset_v1.yaml",
    )
    scene = DualNeroHand2IsaacScene.__new__(DualNeroHand2IsaacScene)
    scene.articulations = {side: _FakeArticulation() for side in ("left", "right")}
    assert np.array_equal(scene.feedback_qdot27("left"), np.arange(27))

    banana_path = "/World/Environment/robolab_banana_bowl/banana"
    scene.dataset_dynamic_object_paths = {"banana": banana_path}
    scene.dataset_kinematic_link_paths = {("left", "palm"): "/World/Robots/Hand2Left/l_base_link"}
    monkeypatch.setattr(
        scene,
        "rigid_body_snapshots",
        lambda: (
            SceneRigidBodySnapshot(
                prim_path=banana_path,
                position_m=(0.1, 0.2, 0.3),
                quat_wxyz=(1.0, 0.0, 0.0, 0.0),
                linear_velocity_m_s=(0.01, 0.02, 0.03),
                angular_velocity_deg_s=(90.0, 0.0, 0.0),
                kinematic_enabled=False,
            ),
        ),
    )
    monkeypatch.setattr(
        scene,
        "kinematic_link_snapshots",
        lambda: (
            SceneKinematicLinkSnapshot(
                side="left",
                logical_link_id="palm",
                prim_path="/World/Robots/Hand2Left/l_base_link",
                position_m=(0.4, 0.5, 0.6),
                quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            ),
        ),
    )
    left = np.arange(27, dtype=np.float64) / 100.0
    right = -left
    left_velocity = np.arange(27, dtype=np.float64) / 10.0
    right_velocity = -left_velocity

    frame = scene.create_dataset_state_frame(
        run_id="episode-001",
        control_index=7,
        phase=SimulationFramePhase.PRE_ACTION,
        simulation_time_s=0.5,
        physics_boundary_index=14,
        q54_profile=profile,
        q27_by_side={"left": left, "right": right},
        qdot27_by_side={"left": left_velocity, "right": right_velocity},
    )

    assert frame.q54_rad == profile.assemble_from_q27(
        left_q27_rad=left,
        right_q27_rad=right,
    )
    assert frame.qdot54_rad_s == profile.assemble_velocity_from_q27(
        left_qdot27_rad_s=left_velocity,
        right_qdot27_rad_s=right_velocity,
    )
    assert frame.rigid_bodies[0].angular_velocity_rad_s == pytest.approx((np.pi / 2.0, 0.0, 0.0))

    rigid = _FakeRigidPrim()
    scene.dynamic_workcell_prims = {banana_path: rigid}
    scene.restore_dataset_state_frame(frame, q54_profile=profile)

    for side, expected_position, expected_velocity in (
        ("left", left, left_velocity),
        ("right", right, right_velocity),
    ):
        articulation = scene.articulations[side]
        assert articulation.positions is not None
        assert articulation.velocities is not None
        assert np.allclose(articulation.positions[0], expected_position)
        assert np.allclose(articulation.velocities[0], expected_velocity)
    assert rigid.pose is not None
    assert rigid.linear_velocity is not None
    assert rigid.angular_velocity is not None
    assert np.allclose(rigid.angular_velocity, (np.pi / 2.0, 0.0, 0.0))

    post_frame = scene.create_dataset_state_frame(
        run_id="episode-001",
        control_index=7,
        phase=SimulationFramePhase.POST_ACTION,
        simulation_time_s=0.5,
        physics_boundary_index=14,
        q54_profile=profile,
        q27_by_side={"left": left, "right": right},
        qdot27_by_side={"left": left_velocity, "right": right_velocity},
    )
    with pytest.raises(ValueError, match="pre_action"):
        scene.restore_dataset_state_frame(post_frame, q54_profile=profile)
