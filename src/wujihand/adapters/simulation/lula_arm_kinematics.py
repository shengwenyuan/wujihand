"""Isaac Lula implementation of the backend-neutral arm kinematics port."""

from __future__ import annotations

from collections.abc import Sequence
import math
from typing import Protocol

import numpy as np
import numpy.typing as npt

from wujihand.domain.joints import JointLayout
from wujihand.domain.pose import (
    quaternion_geodesic_distance_rad,
    rotation_matrix_to_quaternion_wxyz,
    validate_frame_id,
    validate_unit_quaternion_wxyz,
)
from wujihand.ports import (
    ArmEndEffectorPose,
    ArmKinematicsResult,
)


FloatArray = npt.NDArray[np.float64]


class LulaKinematicsSolverLike(Protocol):
    """Subset of Isaac's Lula solver used by this adapter."""

    def compute_forward_kinematics(
        self,
        frame_name: str,
        joint_positions: FloatArray,
    ) -> tuple[object, object]: ...

    def compute_inverse_kinematics(
        self,
        frame_name: str,
        target_position: FloatArray,
        target_orientation: FloatArray,
        *,
        warm_start: FloatArray,
        position_tolerance: float,
        orientation_tolerance: float,
    ) -> tuple[object, object]: ...


class LulaArmKinematicsAdapter:
    """Validate Lula q7/FK/IK values at the application boundary."""

    def __init__(
        self,
        *,
        solver: LulaKinematicsSolverLike,
        layout: JointLayout,
        frame_name: str = "link7",
        position_tolerance_m: float = 0.002,
        orientation_tolerance_rad: float = 0.02,
    ) -> None:
        if not callable(
            getattr(solver, "compute_forward_kinematics", None)
        ) or not callable(
            getattr(solver, "compute_inverse_kinematics", None)
        ):
            raise TypeError("solver does not provide the Lula kinematics API")
        if not isinstance(layout, JointLayout) or layout.size != 7:
            raise ValueError("layout must be a seven-joint JointLayout")
        validate_frame_id(frame_name)
        for field, value, upper in (
            ("position_tolerance_m", position_tolerance_m, 0.1),
            (
                "orientation_tolerance_rad",
                orientation_tolerance_rad,
                math.pi,
            ),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or not 0.0 < float(value) <= upper
            ):
                raise ValueError(
                    f"{field} must be finite and in (0, {upper}]"
                )

        self.solver = solver
        self.layout = layout
        self.frame_name = frame_name
        self.position_tolerance_m = float(position_tolerance_m)
        self.orientation_tolerance_rad = float(
            orientation_tolerance_rad
        )

    def forward(self, q7_rad: Sequence[float]) -> ArmEndEffectorPose:
        q7 = self.layout.validate_vector(q7_rad)
        raw_position, raw_rotation = (
            self.solver.compute_forward_kinematics(
                self.frame_name,
                q7.copy(),
            )
        )
        position = np.asarray(raw_position, dtype=np.float64)
        rotation = np.asarray(raw_rotation, dtype=np.float64)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise RuntimeError("Lula FK returned an invalid position")
        try:
            quaternion = rotation_matrix_to_quaternion_wxyz(rotation)
        except ValueError as exc:
            raise RuntimeError(
                "Lula FK returned an invalid orientation"
            ) from exc
        return ArmEndEffectorPose(
            position_m=(
                float(position[0]),
                float(position[1]),
                float(position[2]),
            ),
            quat_wxyz=(
                float(quaternion[0]),
                float(quaternion[1]),
                float(quaternion[2]),
                float(quaternion[3]),
            ),
        )

    def solve(
        self,
        *,
        target_position_m: Sequence[float],
        target_orientation_wxyz: Sequence[float],
        warm_start_q7_rad: Sequence[float],
    ) -> ArmKinematicsResult:
        target_position = np.asarray(
            target_position_m,
            dtype=np.float64,
        )
        if (
            target_position.shape != (3,)
            or not np.isfinite(target_position).all()
        ):
            raise ValueError("target_position_m must be a finite 3-vector")
        target_orientation = validate_unit_quaternion_wxyz(
            target_orientation_wxyz
        )
        warm_start = self.layout.validate_vector(warm_start_q7_rad)
        raw_candidate, raw_success = (
            self.solver.compute_inverse_kinematics(
                self.frame_name,
                target_position.copy(),
                target_orientation.copy(),
                warm_start=warm_start.copy(),
                position_tolerance=self.position_tolerance_m,
                orientation_tolerance=self.orientation_tolerance_rad,
            )
        )
        solver_reported_success = bool(raw_success)
        try:
            candidate = self.layout.validate_vector(
                np.asarray(raw_candidate, dtype=np.float64)
            )
        except (TypeError, ValueError, OverflowError):
            return ArmKinematicsResult(
                succeeded=False,
                solver_reported_success=solver_reported_success,
                candidate_q7_rad=None,
                position_residual_m=None,
                orientation_residual_rad=None,
                reason="invalid_solver_candidate",
            )

        candidate_pose = self.forward(
            tuple(float(value) for value in candidate)
        )
        position_residual = float(
            np.linalg.norm(
                np.asarray(candidate_pose.position_m, dtype=np.float64)
                - target_position
            )
        )
        orientation_residual = quaternion_geodesic_distance_rad(
            candidate_pose.quat_wxyz,
            target_orientation,
        )
        return ArmKinematicsResult(
            succeeded=solver_reported_success,
            solver_reported_success=solver_reported_success,
            candidate_q7_rad=tuple(float(value) for value in candidate),
            position_residual_m=position_residual,
            orientation_residual_rad=orientation_residual,
            reason=(
                "ik_accepted"
                if solver_reported_success
                else "solver_rejected_candidate"
            ),
        )


__all__ = [
    "LulaArmKinematicsAdapter",
    "LulaKinematicsSolverLike",
]
