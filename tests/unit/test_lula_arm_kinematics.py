from __future__ import annotations

import numpy as np
import pytest

from wujihand.adapters.simulation import LulaArmKinematicsAdapter
from wujihand.domain import JointLayout
from wujihand.ports import ArmKinematicsPort


def layout() -> JointLayout:
    return JointLayout(
        names=tuple(f"joint{index}" for index in range(1, 8)),
        lower=(-2.0,) * 7,
        upper=(2.0,) * 7,
        velocity=(1.0,) * 7,
    )


class FakeLulaSolver:
    def __init__(
        self,
        *,
        candidate: object = (0.1,) * 7,
        success: object = True,
    ) -> None:
        self.candidate = candidate
        self.success = success
        self.forward_rotation: object = np.eye(3)
        self.forward_calls: list[tuple[str, tuple[float, ...]]] = []
        self.inverse_calls: list[dict[str, object]] = []

    def compute_forward_kinematics(
        self,
        frame_name: str,
        joint_positions: np.ndarray,
    ) -> tuple[object, object]:
        self.forward_calls.append(
            (frame_name, tuple(float(value) for value in joint_positions))
        )
        position = np.asarray(
            (joint_positions[0], joint_positions[1], joint_positions[2]),
            dtype=np.float64,
        )
        return position, self.forward_rotation

    def compute_inverse_kinematics(
        self,
        frame_name: str,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        *,
        warm_start: np.ndarray,
        position_tolerance: float,
        orientation_tolerance: float,
    ) -> tuple[object, object]:
        self.inverse_calls.append(
            {
                "frame_name": frame_name,
                "target_position": tuple(target_position),
                "target_orientation": tuple(target_orientation),
                "warm_start": tuple(warm_start),
                "position_tolerance": position_tolerance,
                "orientation_tolerance": orientation_tolerance,
            }
        )
        return self.candidate, self.success


def adapter(solver: FakeLulaSolver) -> LulaArmKinematicsAdapter:
    return LulaArmKinematicsAdapter(
        solver=solver,
        layout=layout(),
        frame_name="link7",
        position_tolerance_m=0.002,
        orientation_tolerance_rad=0.02,
    )


def test_adapter_structurally_implements_port_and_validates_fk() -> None:
    solver = FakeLulaSolver()
    subject = adapter(solver)

    assert isinstance(subject, ArmKinematicsPort)
    pose = subject.forward((0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 0.0))

    assert pose.position_m == pytest.approx((0.4, 0.5, 0.6))
    assert pose.quat_wxyz == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert solver.forward_calls == [
        ("link7", (0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 0.0))
    ]


def test_accepted_ik_carries_candidate_residuals_and_exact_tolerances() -> None:
    solver = FakeLulaSolver(candidate=(0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0))
    subject = adapter(solver)

    result = subject.solve(
        target_position_m=(0.11, 0.18, 0.30),
        target_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        warm_start_q7_rad=(0.0,) * 7,
    )

    assert result.succeeded
    assert result.solver_reported_success
    assert result.candidate_q7_rad == pytest.approx(
        (0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0)
    )
    assert result.position_residual_m == pytest.approx(
        np.linalg.norm((-0.01, 0.02, 0.0))
    )
    assert result.orientation_residual_rad == 0.0
    assert result.reason == "ik_accepted"
    assert solver.inverse_calls == [
        {
            "frame_name": "link7",
            "target_position": (0.11, 0.18, 0.30),
            "target_orientation": (1.0, 0.0, 0.0, 0.0),
            "warm_start": (0.0,) * 7,
            "position_tolerance": 0.002,
            "orientation_tolerance": 0.02,
        }
    ]


def test_solver_rejection_preserves_valid_candidate_for_diagnostics() -> None:
    subject = adapter(FakeLulaSolver(success=False))

    result = subject.solve(
        target_position_m=(0.1, 0.1, 0.1),
        target_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        warm_start_q7_rad=(0.0,) * 7,
    )

    assert not result.succeeded
    assert not result.solver_reported_success
    assert result.candidate_q7_rad == pytest.approx((0.1,) * 7)
    assert result.reason == "solver_rejected_candidate"


@pytest.mark.parametrize(
    "candidate",
    (
        (0.0,) * 6,
        (0.0, 0.0, 0.0, 0.0, 0.0, np.nan),
        "not-a-vector",
    ),
)
def test_invalid_solver_candidate_is_fail_closed(candidate: object) -> None:
    subject = adapter(FakeLulaSolver(candidate=candidate, success=True))

    result = subject.solve(
        target_position_m=(0.1, 0.1, 0.1),
        target_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        warm_start_q7_rad=(0.0,) * 7,
    )

    assert not result.succeeded
    assert result.solver_reported_success
    assert result.candidate_q7_rad is None
    assert result.reason == "invalid_solver_candidate"


def test_invalid_fk_orientation_is_a_backend_fault() -> None:
    solver = FakeLulaSolver()
    solver.forward_rotation = np.zeros((3, 3))
    subject = adapter(solver)

    with pytest.raises(RuntimeError, match="invalid orientation"):
        subject.forward((0.0,) * 7)


def test_configuration_requires_q7_and_positive_tolerances() -> None:
    solver = FakeLulaSolver()
    q6 = JointLayout(
        names=tuple(f"joint{index}" for index in range(1, 7)),
        lower=(-2.0,) * 6,
        upper=(2.0,) * 6,
        velocity=(1.0,) * 6,
    )
    with pytest.raises(ValueError, match="seven-joint"):
        LulaArmKinematicsAdapter(solver=solver, layout=q6)
    with pytest.raises(ValueError, match="position_tolerance_m"):
        LulaArmKinematicsAdapter(
            solver=solver,
            layout=layout(),
            position_tolerance_m=0.0,
        )
