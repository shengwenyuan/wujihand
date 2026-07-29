"""Strict Session compatibility leaf for NV-4 native dual teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Self

from .common import (
    ConfigRef,
    finite_number,
    positive_number,
    require_exact_mapping,
    require_string,
    validate_identifier,
)


NATIVE_DUAL_TELEOPERATION_PROFILE_SCHEMA = (
    "wujihand.native_dual_teleoperation_profile.v1"
)
NATIVE_DUAL_TELEOPERATION_PROFILE_ID = (
    "isaac_nero_hand2_native_dual_teleoperation_v1"
)
NATIVE_DUAL_TELEOPERATION_PROFILE_STATUS = (
    "simulation_live_pending_hil"
)
NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT = (
    "wujihand.native_dual_teleoperation.v1"
)


def _bounded_seconds(value: object, *, field: str) -> float:
    result = positive_number(value, field=field)
    if result > 5.0:
        raise ValueError(f"{field} must be at most 5 seconds")
    return result


def _unit_interval(value: object, *, field: str) -> float:
    result = finite_number(value, field=field)
    if not 0.0 <= result <= 1.0:
        raise ValueError(f"{field} must be in [0, 1]")
    return result


def _positive_int(
    value: object,
    *,
    field: str,
    maximum: int,
) -> int:
    if type(value) is not int or not 1 <= value <= maximum:
        raise ValueError(
            f"{field} must be an integer in [1, {maximum}]"
        )
    return value


@dataclass(frozen=True, slots=True)
class NativeDualTrackerPolicy:
    stable_after_s: float
    max_sample_gap_s: float
    stale_after_s: float
    minimum_quality: float
    max_consecutive_ik_failures: int

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str,
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "stable_after_s",
                    "max_sample_gap_s",
                    "stale_after_s",
                    "minimum_quality",
                    "max_consecutive_ik_failures",
                }
            ),
            field=field,
        )
        return cls(
            stable_after_s=_bounded_seconds(
                data["stable_after_s"],
                field=f"{field}.stable_after_s",
            ),
            max_sample_gap_s=_bounded_seconds(
                data["max_sample_gap_s"],
                field=f"{field}.max_sample_gap_s",
            ),
            stale_after_s=_bounded_seconds(
                data["stale_after_s"],
                field=f"{field}.stale_after_s",
            ),
            minimum_quality=_unit_interval(
                data["minimum_quality"],
                field=f"{field}.minimum_quality",
            ),
            max_consecutive_ik_failures=_positive_int(
                data["max_consecutive_ik_failures"],
                field=f"{field}.max_consecutive_ik_failures",
                maximum=100,
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "stable_after_s": self.stable_after_s,
            "max_sample_gap_s": self.max_sample_gap_s,
            "stale_after_s": self.stale_after_s,
            "minimum_quality": self.minimum_quality,
            "max_consecutive_ik_failures": (
                self.max_consecutive_ik_failures
            ),
        }


@dataclass(frozen=True, slots=True)
class NativeDualKinematicsPolicy:
    adapter_id: str
    end_effector_frame: str
    position_tolerance_m: float
    orientation_tolerance_rad: float

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str,
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "adapter_id",
                    "end_effector_frame",
                    "position_tolerance_m",
                    "orientation_tolerance_rad",
                }
            ),
            field=field,
        )
        adapter_id = validate_identifier(
            data["adapter_id"],
            field=f"{field}.adapter_id",
        )
        if adapter_id != "isaac_lula_q7_v1":
            raise ValueError(
                f"{field}.adapter_id must be 'isaac_lula_q7_v1'"
            )
        position_tolerance_m = positive_number(
            data["position_tolerance_m"],
            field=f"{field}.position_tolerance_m",
        )
        if position_tolerance_m > 0.1:
            raise ValueError(
                f"{field}.position_tolerance_m must be at most 0.1"
            )
        orientation_tolerance_rad = positive_number(
            data["orientation_tolerance_rad"],
            field=f"{field}.orientation_tolerance_rad",
        )
        if orientation_tolerance_rad > 3.141592653589793:
            raise ValueError(
                f"{field}.orientation_tolerance_rad must be at most pi"
            )
        return cls(
            adapter_id=adapter_id,
            end_effector_frame=validate_identifier(
                data["end_effector_frame"],
                field=f"{field}.end_effector_frame",
            ),
            position_tolerance_m=position_tolerance_m,
            orientation_tolerance_rad=orientation_tolerance_rad,
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "adapter_id": self.adapter_id,
            "end_effector_frame": self.end_effector_frame,
            "position_tolerance_m": self.position_tolerance_m,
            "orientation_tolerance_rad": self.orientation_tolerance_rad,
        }


@dataclass(frozen=True, slots=True)
class NativeDualSupervisorPolicy:
    stale_after_s: float
    velocity_scale: float

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str,
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"stale_after_s", "velocity_scale"}),
            field=field,
        )
        velocity_scale = positive_number(
            data["velocity_scale"],
            field=f"{field}.velocity_scale",
        )
        if velocity_scale > 1.0:
            raise ValueError(
                f"{field}.velocity_scale must be at most 1"
            )
        return cls(
            stale_after_s=_bounded_seconds(
                data["stale_after_s"],
                field=f"{field}.stale_after_s",
            ),
            velocity_scale=velocity_scale,
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "stale_after_s": self.stale_after_s,
            "velocity_scale": self.velocity_scale,
        }


@dataclass(frozen=True, slots=True)
class NativeDualGlovePolicy:
    max_observation_age_s: float
    minimum_landmark_confidence: float
    success_landmark_confidence: float

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str,
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "max_observation_age_s",
                    "minimum_landmark_confidence",
                    "success_landmark_confidence",
                }
            ),
            field=field,
        )
        minimum = _unit_interval(
            data["minimum_landmark_confidence"],
            field=f"{field}.minimum_landmark_confidence",
        )
        success = _unit_interval(
            data["success_landmark_confidence"],
            field=f"{field}.success_landmark_confidence",
        )
        if minimum > success:
            raise ValueError(
                f"{field}.minimum_landmark_confidence must not exceed "
                "success_landmark_confidence"
            )
        return cls(
            max_observation_age_s=_bounded_seconds(
                data["max_observation_age_s"],
                field=f"{field}.max_observation_age_s",
            ),
            minimum_landmark_confidence=minimum,
            success_landmark_confidence=success,
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "max_observation_age_s": self.max_observation_age_s,
            "minimum_landmark_confidence": (
                self.minimum_landmark_confidence
            ),
            "success_landmark_confidence": (
                self.success_landmark_confidence
            ),
        }


@dataclass(frozen=True, slots=True)
class NativeDualTeleoperationProfile:
    schema: str
    profile_id: str
    status: str
    base_qualification: ConfigRef
    transport_contract: str
    physics_hz: int
    tracker: NativeDualTrackerPolicy
    kinematics: NativeDualKinematicsPolicy
    arm_supervision: NativeDualSupervisorPolicy
    glove: NativeDualGlovePolicy
    hand_supervision: NativeDualSupervisorPolicy

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "native dual teleoperation profile",
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "schema",
                    "profile_id",
                    "status",
                    "base_qualification",
                    "transport_contract",
                    "physics_hz",
                    "tracker",
                    "kinematics",
                    "arm_supervision",
                    "glove",
                    "hand_supervision",
                }
            ),
            field=field,
        )
        schema = require_string(
            data["schema"],
            field=f"{field}.schema",
        )
        if schema != NATIVE_DUAL_TELEOPERATION_PROFILE_SCHEMA:
            raise ValueError(
                f"{field}.schema must be "
                f"{NATIVE_DUAL_TELEOPERATION_PROFILE_SCHEMA!r}"
            )
        profile_id = validate_identifier(
            data["profile_id"],
            field=f"{field}.profile_id",
        )
        if profile_id != NATIVE_DUAL_TELEOPERATION_PROFILE_ID:
            raise ValueError(
                f"{field}.profile_id must be "
                f"{NATIVE_DUAL_TELEOPERATION_PROFILE_ID!r}"
            )
        status = validate_identifier(
            data["status"],
            field=f"{field}.status",
        )
        if status != NATIVE_DUAL_TELEOPERATION_PROFILE_STATUS:
            raise ValueError(
                f"{field}.status must be "
                f"{NATIVE_DUAL_TELEOPERATION_PROFILE_STATUS!r}"
            )
        transport_contract = validate_identifier(
            data["transport_contract"],
            field=f"{field}.transport_contract",
        )
        if (
            transport_contract
            != NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT
        ):
            raise ValueError(
                f"{field}.transport_contract must be "
                f"{NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT!r}"
            )
        return cls(
            schema=schema,
            profile_id=profile_id,
            status=status,
            base_qualification=ConfigRef.from_mapping(
                data["base_qualification"],
                field=f"{field}.base_qualification",
            ),
            transport_contract=transport_contract,
            physics_hz=_positive_int(
                data["physics_hz"],
                field=f"{field}.physics_hz",
                maximum=1000,
            ),
            tracker=NativeDualTrackerPolicy.from_mapping(
                data["tracker"],
                field=f"{field}.tracker",
            ),
            kinematics=NativeDualKinematicsPolicy.from_mapping(
                data["kinematics"],
                field=f"{field}.kinematics",
            ),
            arm_supervision=NativeDualSupervisorPolicy.from_mapping(
                data["arm_supervision"],
                field=f"{field}.arm_supervision",
            ),
            glove=NativeDualGlovePolicy.from_mapping(
                data["glove"],
                field=f"{field}.glove",
            ),
            hand_supervision=NativeDualSupervisorPolicy.from_mapping(
                data["hand_supervision"],
                field=f"{field}.hand_supervision",
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "profile_id": self.profile_id,
            "status": self.status,
            "base_qualification": self.base_qualification.to_mapping(),
            "transport_contract": self.transport_contract,
            "physics_hz": self.physics_hz,
            "tracker": self.tracker.to_mapping(),
            "kinematics": self.kinematics.to_mapping(),
            "arm_supervision": self.arm_supervision.to_mapping(),
            "glove": self.glove.to_mapping(),
            "hand_supervision": self.hand_supervision.to_mapping(),
        }


__all__ = [
    "NATIVE_DUAL_TELEOPERATION_PROFILE_ID",
    "NATIVE_DUAL_TELEOPERATION_PROFILE_SCHEMA",
    "NATIVE_DUAL_TELEOPERATION_PROFILE_STATUS",
    "NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT",
    "NativeDualGlovePolicy",
    "NativeDualKinematicsPolicy",
    "NativeDualSupervisorPolicy",
    "NativeDualTeleoperationProfile",
    "NativeDualTrackerPolicy",
]
