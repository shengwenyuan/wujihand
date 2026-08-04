"""Strict profile for staged NERO—Hand 2 self-collision qualification."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, cast

import yaml


SELF_COLLISION_QUALIFICATION_SCHEMA = (
    "wujihand.nero_hand2_self_collision_qualification.v1"
)
SELF_COLLISION_QUALIFICATION_PROFILE_ID = (
    "isaac_nero_hand2_self_collision_qualification_v1"
)
SELF_COLLISION_FILTER_SCHEMA = (
    "wujihand.nero_hand2_self_collision_filtered_pairs.v1"
)
SELF_COLLISION_FILTER_PROFILE_ID = (
    "isaac_nero_hand2_self_collision_filtered_pairs_v1"
)


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(Mapping[str, object], value)


def _exact_mapping(
    value: object,
    *,
    expected: frozenset[str],
    field: str,
) -> Mapping[str, object]:
    result = _mapping(value, field=field)
    if frozenset(result) != expected:
        raise ValueError(
            f"{field} keys differ: missing={sorted(expected - frozenset(result))}, "
            f"unexpected={sorted(frozenset(result) - expected)}"
        )
    return result


def _positive_int(value: object, *, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{field} must be a positive integer")
    return value


def _non_negative_int(value: object, *, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    return value


def _finite(value: object, *, field: str, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.0) or result < 0.0:
        raise ValueError(f"{field} must be finite and {'positive' if positive else 'non-negative'}")
    return result


@dataclass(frozen=True, slots=True)
class SelfCollisionPhaseFrames:
    settle_rest: int
    observe_rest: int
    close_trajectory: int
    hold_grasp: int
    open_trajectory: int
    final_rest: int


@dataclass(frozen=True, slots=True)
class SelfCollisionThresholds:
    contact_report_threshold_n: float
    contact_separation_epsilon_m: float
    maximum_arm_target_error_rad: float
    maximum_hand_target_error_rad: float
    maximum_hold_drift_rad: float
    maximum_unexplained_rest_penetration_m: float
    maximum_any_self_penetration_m: float
    maximum_unexplained_rest_contact_frames: int
    maximum_cross_side_contact_frames: int
    transform_translation_tolerance_m: float
    transform_rotation_tolerance: float


@dataclass(frozen=True, slots=True)
class NeroHand2SelfCollisionQualificationProfile:
    profile_id: str
    physics_hz: int
    hand_amplitude_rad: float
    phases: SelfCollisionPhaseFrames
    thresholds: SelfCollisionThresholds
    collision_mesh_contract: tuple[tuple[str, str], ...]
    assumptions: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class SelfCollisionFilteredPair:
    pair_id: str
    sides: tuple[str, ...]
    first_rigid_body_name: str
    second_rigid_body_name: str
    evidence: tuple[tuple[str, object], ...]


@dataclass(frozen=True, slots=True)
class NeroHand2SelfCollisionFilterProfile:
    profile_id: str
    source_contract: tuple[tuple[str, str], ...]
    filtered_pairs: tuple[SelfCollisionFilteredPair, ...]
    assumptions: tuple[str, ...]


def load_nero_hand2_self_collision_qualification_profile(
    path: str | Path,
) -> NeroHand2SelfCollisionQualificationProfile:
    raw: Any = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    data = _exact_mapping(
        raw,
        expected=frozenset(
            {
                "schema",
                "profile_id",
                "status",
                "physics_hz",
                "hand_amplitude_rad",
                "phases",
                "thresholds",
                "collision_mesh_contract",
                "assumptions",
            }
        ),
        field="self-collision qualification profile",
    )


    if data["schema"] != SELF_COLLISION_QUALIFICATION_SCHEMA:
        raise ValueError("unsupported self-collision qualification schema")
    if data["profile_id"] != SELF_COLLISION_QUALIFICATION_PROFILE_ID:
        raise ValueError("unexpected self-collision qualification profile ID")
    if data["status"] != "simulation_only":
        raise ValueError("self-collision qualification profile must be simulation_only")

    phases = _exact_mapping(
        data["phases"],
        expected=frozenset(
            {
                "settle_rest_frames",
                "observe_rest_frames",
                "close_trajectory_frames",
                "hold_grasp_frames",
                "open_trajectory_frames",
                "final_rest_frames",
            }
        ),
        field="self-collision qualification profile.phases",
    )
    thresholds = _exact_mapping(
        data["thresholds"],
        expected=frozenset(
            {
                "contact_report_threshold_n",
                "contact_separation_epsilon_m",
                "maximum_arm_target_error_rad",
                "maximum_hand_target_error_rad",
                "maximum_hold_drift_rad",
                "maximum_unexplained_rest_penetration_m",
                "maximum_any_self_penetration_m",
                "maximum_unexplained_rest_contact_frames",
                "maximum_cross_side_contact_frames",
                "transform_translation_tolerance_m",
                "transform_rotation_tolerance",
            }
        ),
        field="self-collision qualification profile.thresholds",
    )
    collision_contract = _mapping(
        data["collision_mesh_contract"], field="collision_mesh_contract"
    )
    assumptions = data["assumptions"]
    if (
        not isinstance(assumptions, list)
        or not assumptions
        or any(not isinstance(value, str) or not value for value in assumptions)
    ):
        raise ValueError("assumptions must contain non-blank strings")
    return NeroHand2SelfCollisionQualificationProfile(
        profile_id=SELF_COLLISION_QUALIFICATION_PROFILE_ID,
        physics_hz=_positive_int(data["physics_hz"], field="physics_hz"),
        hand_amplitude_rad=_finite(
            data["hand_amplitude_rad"], field="hand_amplitude_rad", positive=True
        ),
        phases=SelfCollisionPhaseFrames(
            settle_rest=_positive_int(
                phases["settle_rest_frames"], field="settle_rest_frames"
            ),
            observe_rest=_positive_int(
                phases["observe_rest_frames"], field="observe_rest_frames"
            ),
            close_trajectory=_positive_int(
                phases["close_trajectory_frames"], field="close_trajectory_frames"
            ),
            hold_grasp=_positive_int(
                phases["hold_grasp_frames"], field="hold_grasp_frames"
            ),
            open_trajectory=_positive_int(
                phases["open_trajectory_frames"], field="open_trajectory_frames"
            ),
            final_rest=_positive_int(
                phases["final_rest_frames"], field="final_rest_frames"
            ),
        ),
        thresholds=SelfCollisionThresholds(
            contact_report_threshold_n=_finite(
                thresholds["contact_report_threshold_n"],
                field="contact_report_threshold_n",
            ),
            contact_separation_epsilon_m=_finite(
                thresholds["contact_separation_epsilon_m"],
                field="contact_separation_epsilon_m",
                positive=True,
            ),
            maximum_arm_target_error_rad=_finite(
                thresholds["maximum_arm_target_error_rad"],
                field="maximum_arm_target_error_rad",
                positive=True,
            ),
            maximum_hand_target_error_rad=_finite(
                thresholds["maximum_hand_target_error_rad"],
                field="maximum_hand_target_error_rad",
                positive=True,
            ),
            maximum_hold_drift_rad=_finite(
                thresholds["maximum_hold_drift_rad"],
                field="maximum_hold_drift_rad",
                positive=True,
            ),
            maximum_unexplained_rest_penetration_m=_finite(
                thresholds["maximum_unexplained_rest_penetration_m"],
                field="maximum_unexplained_rest_penetration_m",
                positive=True,
            ),
            maximum_any_self_penetration_m=_finite(
                thresholds["maximum_any_self_penetration_m"],
                field="maximum_any_self_penetration_m",
                positive=True,
            ),
            maximum_unexplained_rest_contact_frames=_non_negative_int(
                thresholds["maximum_unexplained_rest_contact_frames"],
                field="maximum_unexplained_rest_contact_frames",
            ),
            maximum_cross_side_contact_frames=_non_negative_int(
                thresholds["maximum_cross_side_contact_frames"],
                field="maximum_cross_side_contact_frames",
            ),
            transform_translation_tolerance_m=_finite(
                thresholds["transform_translation_tolerance_m"],
                field="transform_translation_tolerance_m",
                positive=True,
            ),
            transform_rotation_tolerance=_finite(
                thresholds["transform_rotation_tolerance"],
                field="transform_rotation_tolerance",
                positive=True,
            ),
        ),
        collision_mesh_contract=tuple(
            sorted((str(key), str(value)) for key, value in collision_contract.items())
        ),
        assumptions=tuple(cast(list[str], assumptions)),
    )


def load_nero_hand2_self_collision_filter_profile(
    path: str | Path,
) -> NeroHand2SelfCollisionFilterProfile:
    raw: Any = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    data = _exact_mapping(
        raw,
        expected=frozenset(
            {
                "schema",
                "profile_id",
                "status",
                "source_contract",
                "filtered_pairs",
                "assumptions",
            }
        ),
        field="self-collision filter profile",
    )
    if data["schema"] != SELF_COLLISION_FILTER_SCHEMA:
        raise ValueError("unsupported self-collision filter schema")
    if data["profile_id"] != SELF_COLLISION_FILTER_PROFILE_ID:
        raise ValueError("unexpected self-collision filter profile ID")
    if data["status"] != "simulation_only_evidence_based":
        raise ValueError("self-collision filter profile must be evidence based")
    source_contract = _mapping(data["source_contract"], field="source_contract")
    raw_pairs = data["filtered_pairs"]
    if not isinstance(raw_pairs, list) or not raw_pairs:
        raise ValueError("filtered_pairs must be a non-empty list")
    pairs: list[SelfCollisionFilteredPair] = []
    for index, raw_pair in enumerate(raw_pairs):
        field = f"filtered_pairs[{index}]"
        pair = _exact_mapping(
            raw_pair,
            expected=frozenset(
                {
                    "pair_id",
                    "sides",
                    "first_rigid_body_name",
                    "second_rigid_body_name",
                    "evidence",
                }
            ),
            field=field,
        )
        sides = pair["sides"]
        if (
            not isinstance(sides, list)
            or not sides
            or any(side not in {"left", "right"} for side in sides)
            or len(set(cast(list[str], sides))) != len(sides)
        ):
            raise ValueError(f"{field}.sides must contain unique left/right values")
        first = pair["first_rigid_body_name"]
        second = pair["second_rigid_body_name"]
        if (
            not isinstance(first, str)
            or not first
            or not isinstance(second, str)
            or not second
            or first == second
        ):
            raise ValueError(f"{field} must name two distinct rigid bodies")
        evidence = _mapping(pair["evidence"], field=f"{field}.evidence")
        if not evidence:
            raise ValueError(f"{field}.evidence must not be empty")
        pair_id = pair["pair_id"]
        if not isinstance(pair_id, str) or not pair_id:
            raise ValueError(f"{field}.pair_id must be non-blank")
        pairs.append(
            SelfCollisionFilteredPair(
                pair_id=pair_id,
                sides=tuple(cast(list[str], sides)),
                first_rigid_body_name=first,
                second_rigid_body_name=second,
                evidence=tuple(sorted(evidence.items())),
            )
        )
    if len({pair.pair_id for pair in pairs}) != len(pairs):
        raise ValueError("filtered pair IDs must be unique")
    assumptions = data["assumptions"]
    if (
        not isinstance(assumptions, list)
        or not assumptions
        or any(not isinstance(value, str) or not value for value in assumptions)
    ):
        raise ValueError("assumptions must contain non-blank strings")
    return NeroHand2SelfCollisionFilterProfile(
        profile_id=SELF_COLLISION_FILTER_PROFILE_ID,
        source_contract=tuple(
            sorted((str(key), str(value)) for key, value in source_contract.items())
        ),
        filtered_pairs=tuple(pairs),
        assumptions=tuple(cast(list[str], assumptions)),
    )


def author_isaac_self_collision_filters(
    stage: object,
    *,
    arm_prim_paths: Mapping[str, str],
    enabled_sides: frozenset[str],
    profile: NeroHand2SelfCollisionFilterProfile,
) -> tuple[tuple[str, str, str], ...]:
    """Author only evidence-backed rigid-body filtered pairs."""

    from pxr import Sdf, UsdPhysics  # type: ignore[import-not-found]

    if not enabled_sides <= {"left", "right"}:
        raise ValueError("enabled_sides must contain only left/right")
    authored: list[tuple[str, str, str]] = []
    for rule in profile.filtered_pairs:
        for side in rule.sides:
            if side not in enabled_sides:
                continue
            root = arm_prim_paths.get(side)
            if root is None:
                raise RuntimeError(f"missing NERO root for filtered-pair side {side!r}")
            matches: dict[str, list[Any]] = {
                rule.first_rigid_body_name: [],
                rule.second_rigid_body_name: [],
            }
            for prim in stage.Traverse():  # type: ignore[attr-defined]
                path = str(prim.GetPath())
                if not path.startswith(root.rstrip("/") + "/"):
                    continue
                name = str(prim.GetName())
                if name in matches and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    matches[name].append(prim)
            if any(len(values) != 1 for values in matches.values()):
                resolved = {
                    name: [str(item.GetPath()) for item in values]
                    for name, values in matches.items()
                }
                raise RuntimeError(
                    f"filtered pair {rule.pair_id!r} did not resolve uniquely: "
                    f"{resolved}"
                )
            first = matches[rule.first_rigid_body_name][0]
            second = matches[rule.second_rigid_body_name][0]
            first_path = str(first.GetPath())
            second_path = str(second.GetPath())
            api = UsdPhysics.FilteredPairsAPI.Apply(first)
            relationship = api.CreateFilteredPairsRel()
            relationship.AddTarget(Sdf.Path(second_path))
            if Sdf.Path(second_path) not in relationship.GetTargets():
                raise RuntimeError(f"failed to author filtered pair {rule.pair_id!r}")
            authored.append((rule.pair_id, first_path, second_path))
    return tuple(authored)


__all__ = [
    "NeroHand2SelfCollisionFilterProfile",
    "NeroHand2SelfCollisionQualificationProfile",
    "SELF_COLLISION_FILTER_PROFILE_ID",
    "SELF_COLLISION_FILTER_SCHEMA",
    "SELF_COLLISION_QUALIFICATION_PROFILE_ID",
    "SELF_COLLISION_QUALIFICATION_SCHEMA",
    "SelfCollisionPhaseFrames",
    "SelfCollisionFilteredPair",
    "SelfCollisionThresholds",
    "author_isaac_self_collision_filters",
    "load_nero_hand2_self_collision_filter_profile",
    "load_nero_hand2_self_collision_qualification_profile",
]
