from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from wujihand.runtime.isaac_dual_scene import DualNeroHand2IsaacScene


class _FakeArticulation:
    def __init__(self) -> None:
        self.positions: np.ndarray | None = None
        self.velocities: np.ndarray | None = None

    def set_joint_positions(self, values: np.ndarray) -> None:
        self.positions = values.copy()

    def set_joint_velocities(self, values: np.ndarray) -> None:
        self.velocities = values.copy()


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
