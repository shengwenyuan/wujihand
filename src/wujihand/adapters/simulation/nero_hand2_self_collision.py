"""Strict profile for staged NERO—Hand 2 self-collision qualification."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Literal, cast

import yaml


SELF_COLLISION_QUALIFICATION_SCHEMA = "wujihand.nero_hand2_self_collision_qualification.v1"
SELF_COLLISION_QUALIFICATION_PROFILE_ID = "isaac_nero_hand2_self_collision_qualification_v1"
SELF_COLLISION_FILTER_SCHEMA = "wujihand.nero_hand2_self_collision_filtered_pairs.v1"
SELF_COLLISION_FILTER_SCHEMA_V2 = "wujihand.nero_hand2_self_collision_filtered_pairs.v2"
SELF_COLLISION_FILTER_PROFILE_ID = "isaac_nero_hand2_self_collision_filtered_pairs_v1"
SELF_COLLISION_CONTACT_TARGET_SCHEMA = "wujihand.nero_hand2_self_collision_contact_target.v1"
SELF_COLLISION_Q7_SWEEP_SCHEMA = "wujihand.nero_hand2_self_collision_q7_sweep.v1"


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


def _signed_finite(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
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
    first_instance: Literal["arm", "hand"]
    first_rigid_body_name: str
    second_instance: Literal["arm", "hand"]
    second_rigid_body_name: str
    evidence: tuple[tuple[str, object], ...]


@dataclass(frozen=True, slots=True)
class NeroHand2SelfCollisionFilterProfile:
    profile_id: str
    source_contract: tuple[tuple[str, str], ...]
    filtered_pairs: tuple[SelfCollisionFilteredPair, ...]
    assumptions: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class NeroHand2SelfCollisionContactTargetProfile:
    profile_id: str
    hand2_source: str
    targets: tuple[tuple[str, tuple[float, ...]], ...]
    evidence: tuple[tuple[str, object], ...]

    def target(self, side: str) -> tuple[float, ...]:
        for target_side, q20 in self.targets:
            if target_side == side:
                return q20
        raise KeyError(side)


@dataclass(frozen=True, slots=True)
class SelfCollisionQ7Waypoint:
    name: str
    overrides_rad: tuple[tuple[str, float], ...]


@dataclass(frozen=True, slots=True)
class NeroHand2SelfCollisionQ7SweepProfile:
    profile_id: str
    transition_frames: int
    hold_frames: int
    limit_tolerance_rad: float
    maximum_hold_error_rad: float
    maximum_feedback_envelope_excess_rad: float
    minimum_expected_joint_range_rad: float
    waypoints: tuple[SelfCollisionQ7Waypoint, ...]


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
    profile_id = data["profile_id"]
    if not isinstance(profile_id, str) or not profile_id:
        raise ValueError("self-collision qualification profile ID must be non-blank")
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
    collision_contract = _mapping(data["collision_mesh_contract"], field="collision_mesh_contract")
    assumptions = data["assumptions"]
    if (
        not isinstance(assumptions, list)
        or not assumptions
        or any(not isinstance(value, str) or not value for value in assumptions)
    ):
        raise ValueError("assumptions must contain non-blank strings")
    return NeroHand2SelfCollisionQualificationProfile(
        profile_id=profile_id,
        physics_hz=_positive_int(data["physics_hz"], field="physics_hz"),
        hand_amplitude_rad=_finite(
            data["hand_amplitude_rad"], field="hand_amplitude_rad", positive=True
        ),
        phases=SelfCollisionPhaseFrames(
            settle_rest=_positive_int(phases["settle_rest_frames"], field="settle_rest_frames"),
            observe_rest=_positive_int(phases["observe_rest_frames"], field="observe_rest_frames"),
            close_trajectory=_positive_int(
                phases["close_trajectory_frames"], field="close_trajectory_frames"
            ),
            hold_grasp=_positive_int(phases["hold_grasp_frames"], field="hold_grasp_frames"),
            open_trajectory=_positive_int(
                phases["open_trajectory_frames"], field="open_trajectory_frames"
            ),
            final_rest=_positive_int(phases["final_rest_frames"], field="final_rest_frames"),
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
    schema = data["schema"]
    if schema not in {SELF_COLLISION_FILTER_SCHEMA, SELF_COLLISION_FILTER_SCHEMA_V2}:
        raise ValueError("unsupported self-collision filter schema")
    profile_id = data["profile_id"]
    if not isinstance(profile_id, str) or not profile_id:
        raise ValueError("self-collision filter profile ID must be non-blank")
    if data["status"] != "simulation_only_evidence_based":
        raise ValueError("self-collision filter profile must be evidence based")
    source_contract = _mapping(data["source_contract"], field="source_contract")
    raw_pairs = data["filtered_pairs"]
    if not isinstance(raw_pairs, list) or not raw_pairs:
        raise ValueError("filtered_pairs must be a non-empty list")
    pairs: list[SelfCollisionFilteredPair] = []
    for index, raw_pair in enumerate(raw_pairs):
        field = f"filtered_pairs[{index}]"
        pair_fields = {
            "pair_id",
            "sides",
            "first_rigid_body_name",
            "second_rigid_body_name",
            "evidence",
        }
        if schema == SELF_COLLISION_FILTER_SCHEMA_V2:
            pair_fields |= {"first_instance", "second_instance"}
        pair = _exact_mapping(
            raw_pair,
            expected=frozenset(pair_fields),
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
        first_instance = pair.get("first_instance", "arm")
        second_instance = pair.get("second_instance", "arm")
        if first_instance not in {"arm", "hand"} or second_instance not in {
            "arm",
            "hand",
        }:
            raise ValueError(f"{field} instances must be arm or hand")
        pairs.append(
            SelfCollisionFilteredPair(
                pair_id=pair_id,
                sides=tuple(cast(list[str], sides)),
                first_instance=cast(Literal["arm", "hand"], first_instance),
                first_rigid_body_name=first,
                second_instance=cast(Literal["arm", "hand"], second_instance),
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
        profile_id=profile_id,
        source_contract=tuple(
            sorted((str(key), str(value)) for key, value in source_contract.items())
        ),
        filtered_pairs=tuple(pairs),
        assumptions=tuple(cast(list[str], assumptions)),
    )


def load_nero_hand2_self_collision_contact_target_profile(
    path: str | Path,
) -> NeroHand2SelfCollisionContactTargetProfile:
    raw: Any = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    data = _exact_mapping(
        raw,
        expected=frozenset(
            {"schema", "profile_id", "status", "hand2_source", "targets", "evidence"}
        ),
        field="self-collision contact target profile",
    )
    if data["schema"] != SELF_COLLISION_CONTACT_TARGET_SCHEMA:
        raise ValueError("unsupported self-collision contact target schema")
    if data["status"] != "simulation_only_recorded_glove_fixture":
        raise ValueError("self-collision contact target profile has unexpected status")
    profile_id = data["profile_id"]
    hand2_source = data["hand2_source"]
    if not isinstance(profile_id, str) or not profile_id:
        raise ValueError("self-collision contact target profile ID must be non-blank")
    if not isinstance(hand2_source, str) or not hand2_source:
        raise ValueError("self-collision contact target Hand2 source must be non-blank")
    raw_targets = _exact_mapping(
        data["targets"],
        expected=frozenset({"left", "right"}),
        field="self-collision contact targets",
    )
    targets: list[tuple[str, tuple[float, ...]]] = []
    for side in ("left", "right"):
        raw_q20 = raw_targets[side]
        if not isinstance(raw_q20, list) or len(raw_q20) != 20:
            raise ValueError(f"self-collision contact target {side} must contain q20")
        q20 = tuple(float(value) for value in raw_q20)
        if not all(math.isfinite(value) for value in q20):
            raise ValueError(f"self-collision contact target {side} must be finite")
        targets.append((side, q20))
    evidence = _mapping(data["evidence"], field="self-collision contact target evidence")
    if not evidence:
        raise ValueError("self-collision contact target evidence must not be empty")
    return NeroHand2SelfCollisionContactTargetProfile(
        profile_id=profile_id,
        hand2_source=hand2_source,
        targets=tuple(targets),
        evidence=tuple(sorted(evidence.items())),
    )


def load_nero_hand2_self_collision_q7_sweep_profile(
    path: str | Path,
) -> NeroHand2SelfCollisionQ7SweepProfile:
    raw: Any = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    data = _exact_mapping(
        raw,
        expected=frozenset(
            {
                "schema",
                "profile_id",
                "status",
                "transition_frames",
                "hold_frames",
                "thresholds",
                "waypoints",
            }
        ),
        field="self-collision q7 sweep profile",
    )
    if data["schema"] != SELF_COLLISION_Q7_SWEEP_SCHEMA:
        raise ValueError("unsupported self-collision q7 sweep schema")
    if data["status"] != "simulation_only":
        raise ValueError("self-collision q7 sweep profile must be simulation_only")
    profile_id = data["profile_id"]
    if not isinstance(profile_id, str) or not profile_id:
        raise ValueError("self-collision q7 sweep profile ID must be non-blank")
    thresholds = _exact_mapping(
        data["thresholds"],
        expected=frozenset(
            {
                "limit_tolerance_rad",
                "maximum_hold_error_rad",
                "maximum_feedback_envelope_excess_rad",
                "minimum_expected_joint_range_rad",
            }
        ),
        field="self-collision q7 sweep thresholds",
    )
    raw_waypoints = data["waypoints"]
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("self-collision q7 sweep waypoints must be a non-empty list")
    waypoints: list[SelfCollisionQ7Waypoint] = []
    for index, raw_waypoint in enumerate(raw_waypoints):
        waypoint = _exact_mapping(
            raw_waypoint,
            expected=frozenset({"name", "overrides_rad"}),
            field=f"self-collision q7 sweep waypoints[{index}]",
        )
        name = waypoint["name"]
        overrides = _mapping(
            waypoint["overrides_rad"],
            field=f"self-collision q7 sweep waypoints[{index}].overrides_rad",
        )
        if not isinstance(name, str) or not name or not overrides:
            raise ValueError("each q7 sweep waypoint must have a name and overrides")
        values = tuple(
            sorted(
                (
                    joint,
                    _signed_finite(value, field=f"q7 sweep waypoint {name!r}.{joint}"),
                )
                for joint, value in overrides.items()
            )
        )
        waypoints.append(SelfCollisionQ7Waypoint(name=name, overrides_rad=values))
    if len({waypoint.name for waypoint in waypoints}) != len(waypoints):
        raise ValueError("self-collision q7 sweep waypoint names must be unique")
    return NeroHand2SelfCollisionQ7SweepProfile(
        profile_id=profile_id,
        transition_frames=_positive_int(data["transition_frames"], field="transition_frames"),
        hold_frames=_positive_int(data["hold_frames"], field="hold_frames"),
        limit_tolerance_rad=_finite(
            thresholds["limit_tolerance_rad"], field="limit_tolerance_rad", positive=True
        ),
        maximum_hold_error_rad=_finite(
            thresholds["maximum_hold_error_rad"],
            field="maximum_hold_error_rad",
            positive=True,
        ),
        maximum_feedback_envelope_excess_rad=_finite(
            thresholds["maximum_feedback_envelope_excess_rad"],
            field="maximum_feedback_envelope_excess_rad",
            positive=True,
        ),
        minimum_expected_joint_range_rad=_finite(
            thresholds["minimum_expected_joint_range_rad"],
            field="minimum_expected_joint_range_rad",
            positive=True,
        ),
        waypoints=tuple(waypoints),
    )


def author_isaac_self_collision_filters(
    stage: object,
    *,
    arm_prim_paths: Mapping[str, str],
    hand_prim_paths: Mapping[str, str],
    enabled_sides: frozenset[str],
    profile: NeroHand2SelfCollisionFilterProfile,
) -> tuple[tuple[str, str, str], ...]:
    """Author only evidence-backed rigid-body filtered pairs."""

    from pxr import Sdf, UsdPhysics  # type: ignore[import-not-found]

    if not enabled_sides <= {"left", "right"}:
        raise ValueError("enabled_sides must contain only left/right")

    def rigid_body(instance: Literal["arm", "hand"], side: str, name: str) -> Any:
        roots = arm_prim_paths if instance == "arm" else hand_prim_paths
        root = roots.get(side)
        if root is None:
            raise RuntimeError(f"missing {instance} root for filtered-pair side {side!r}")
        matches = [
            prim
            for prim in stage.Traverse()  # type: ignore[attr-defined]
            if str(prim.GetPath()).startswith(root.rstrip("/") + "/")
            and str(prim.GetName()) == name
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ]
        if len(matches) != 1:
            paths = [str(item.GetPath()) for item in matches]
            raise RuntimeError(
                f"filtered rigid body {instance}/{side}/{name!r} did not resolve uniquely: {paths}"
            )
        return matches[0]

    authored: list[tuple[str, str, str]] = []
    for rule in profile.filtered_pairs:
        for side in rule.sides:
            if side not in enabled_sides:
                continue
            first = rigid_body(
                rule.first_instance,
                side,
                rule.first_rigid_body_name,
            )
            second = rigid_body(
                rule.second_instance,
                side,
                rule.second_rigid_body_name,
            )
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
    "NeroHand2SelfCollisionQ7SweepProfile",
    "SELF_COLLISION_FILTER_PROFILE_ID",
    "SELF_COLLISION_FILTER_SCHEMA",
    "SELF_COLLISION_QUALIFICATION_PROFILE_ID",
    "SELF_COLLISION_QUALIFICATION_SCHEMA",
    "SelfCollisionPhaseFrames",
    "SelfCollisionFilteredPair",
    "SelfCollisionQ7Waypoint",
    "SelfCollisionThresholds",
    "author_isaac_self_collision_filters",
    "load_nero_hand2_self_collision_filter_profile",
    "load_nero_hand2_self_collision_contact_target_profile",
    "load_nero_hand2_self_collision_q7_sweep_profile",
    "load_nero_hand2_self_collision_qualification_profile",
]
