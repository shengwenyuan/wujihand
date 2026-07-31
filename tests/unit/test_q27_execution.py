from __future__ import annotations

import numpy as np
import pytest

from wujihand.adapters.simulation import IsaacQ27ExecutionAdapter
from wujihand.application.teleoperation import (
    compose_partitioned_q27_target,
)


class Articulation:
    def __init__(self) -> None:
        self.feedback = np.arange(27, dtype=np.float64)[None, :]
        self.applied: list[np.ndarray] = []

    def get_joint_positions(self) -> np.ndarray:
        return self.feedback.copy()

    def set_joint_position_targets(self, positions: object) -> None:
        self.applied.append(np.asarray(positions, dtype=np.float64))


def test_q27_composition_and_adapter_apply_are_atomic_per_side() -> None:
    arm_indices = (0, 2, 4, 6, 8, 10, 12)
    hand_indices = tuple(index for index in range(27) if index not in arm_indices)
    arm = np.linspace(0.0, 0.6, 7)
    hand = np.linspace(-1.0, 1.0, 20)
    target = compose_partitioned_q27_target(
        side="right",
        arm_indices_q7=arm_indices,
        hand_indices_q20=hand_indices,
        arm_q7=arm,
        hand_q20=hand,
    )
    left = Articulation()
    right = Articulation()
    execution = IsaacQ27ExecutionAdapter(
        {"left": left, "right": right}
    )

    execution.apply_target_q27(target)

    assert left.applied == []
    assert len(right.applied) == 1
    assert right.applied[0].shape == (1, 27)
    np.testing.assert_allclose(
        right.applied[0][0, np.asarray(arm_indices)],
        arm,
    )
    np.testing.assert_allclose(
        right.applied[0][0, np.asarray(hand_indices)],
        hand,
    )
    np.testing.assert_allclose(
        execution.read_feedback_q27("left"),
        np.arange(27),
    )


def test_q27_composition_rejects_overlap_and_non_finite_values() -> None:
    with pytest.raises(ValueError, match="disjoint"):
        compose_partitioned_q27_target(
            side="left",
            arm_indices_q7=tuple(range(7)),
            hand_indices_q20=tuple(range(6, 26)),
            arm_q7=np.zeros(7),
            hand_q20=np.zeros(20),
        )

    with pytest.raises(ValueError, match="finite"):
        compose_partitioned_q27_target(
            side="left",
            arm_indices_q7=tuple(range(7)),
            hand_indices_q20=tuple(range(7, 27)),
            arm_q7=np.full(7, np.nan),
            hand_q20=np.zeros(20),
        )
