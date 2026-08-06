"""Strict closure between dataset manifests and per-frame simulation truth."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from typing import cast

from wujihand.domain.dataset_recording import SimulationStateFrame

from .profile import Q54JointProfile


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(Mapping[str, object], value)


def _sequence(value: object, *, field: str) -> Sequence[object]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{field} must be a sequence")
    return cast(Sequence[object], value)


def _string_sequence(value: object, *, field: str) -> tuple[str, ...]:
    values = _sequence(value, field=field)
    if any(not isinstance(item, str) or not item for item in values):
        raise ValueError(f"{field} must contain non-empty strings")
    return tuple(cast(Sequence[str], values))


def validate_q54_runtime_inventory(
    value: object,
    *,
    profile: Q54JointProfile,
) -> tuple[str, ...]:
    """Validate the complete runtime q54 name/index/limit inventory."""

    inventory = _mapping(value, field="q54 runtime inventory")
    expected_keys = {
        "schema",
        "profile_id",
        "profile_sha256",
        "canonical_names",
        "left_runtime_names",
        "right_runtime_names",
        "canonical_source_indices",
        "runtime_limits_rad",
    }
    if set(inventory) != expected_keys:
        raise ValueError("q54 runtime inventory keys differ from schema")
    if inventory["schema"] != "wujihand.q54_runtime_inventory.v1":
        raise ValueError("q54 runtime inventory schema differs")
    if (
        inventory["profile_id"] != profile.profile_id
        or inventory["profile_sha256"] != profile.file_sha256
    ):
        raise ValueError("q54 runtime inventory profile identity differs")

    canonical_names = _string_sequence(
        inventory["canonical_names"],
        field="q54 canonical names",
    )
    left_names = _string_sequence(
        inventory["left_runtime_names"],
        field="left q27 runtime names",
    )
    right_names = _string_sequence(
        inventory["right_runtime_names"],
        field="right q27 runtime names",
    )
    expected_runtime_names: dict[str, tuple[str, ...]] = {}
    for side in ("left", "right"):
        joints = tuple(item for item in profile.joints if item.side == side)
        expected_runtime_names[side] = tuple(
            next(
                item.source_joint_name
                for item in joints
                if item.source_index_q27 == runtime_index
            )
            for runtime_index in range(27)
        )
    if canonical_names != profile.canonical_names:
        raise ValueError("q54 canonical names differ from the pinned profile")
    if left_names != expected_runtime_names["left"]:
        raise ValueError("left q27 runtime name/order inventory differs")
    if right_names != expected_runtime_names["right"]:
        raise ValueError("right q27 runtime name/order inventory differs")

    indices_raw = _sequence(
        inventory["canonical_source_indices"],
        field="q54 canonical source indices",
    )
    if any(type(item) is not int for item in indices_raw):
        raise ValueError("q54 canonical source indices must be integers")
    indices = tuple(cast(Sequence[int], indices_raw))
    expected_indices = tuple(item.source_index_q27 for item in profile.joints)
    if indices != expected_indices:
        raise ValueError("q54 canonical source indices differ from the pinned profile")

    limits_raw = _sequence(inventory["runtime_limits_rad"], field="q54 runtime limits")
    if len(limits_raw) != 54:
        raise ValueError("q54 runtime limits must contain exactly 54 pairs")
    for index, (joint, raw_limit) in enumerate(zip(profile.joints, limits_raw, strict=True)):
        pair = _sequence(raw_limit, field=f"q54 runtime limits[{index}]")
        if len(pair) != 2 or any(
            isinstance(item, bool) or not isinstance(item, (int, float)) for item in pair
        ):
            raise ValueError(f"q54 runtime limits[{index}] must contain two finite numbers")
        actual = tuple(float(cast(int | float, item)) for item in pair)
        if any(not math.isfinite(item) for item in actual):
            raise ValueError(f"q54 runtime limits[{index}] must contain two finite numbers")
        expected = (
            (
                joint.lower_rad - joint.zero_offset_rad,
                joint.upper_rad - joint.zero_offset_rad,
            )
            if joint.sign == 1
            else (
                joint.zero_offset_rad - joint.upper_rad,
                joint.zero_offset_rad - joint.lower_rad,
            )
        )
        if any(
            not math.isclose(observed, pinned, rel_tol=0.0, abs_tol=1e-4)
            for observed, pinned in zip(actual, expected, strict=True)
        ):
            raise ValueError(f"q54 runtime limits[{index}] differ from the pinned profile")
    return canonical_names


def parse_dataset_truth_inventories(
    dataset_manifest: Mapping[str, object],
) -> tuple[dict[str, str], dict[tuple[str, str], str]]:
    """Parse the deliberately small 008 banana and bilateral link inventory."""

    objects_raw = _mapping(
        dataset_manifest.get("dynamic_object_inventory"),
        field="dynamic object inventory",
    )
    if set(objects_raw) != {"banana"}:
        raise ValueError("008 dynamic object inventory must contain only banana")
    object_path = objects_raw["banana"]
    if not isinstance(object_path, str) or not object_path.startswith("/"):
        raise ValueError("banana inventory path must be an absolute USD prim path")
    objects = {"banana": object_path}

    links_raw = _sequence(
        dataset_manifest.get("kinematic_link_inventory"),
        field="kinematic link inventory",
    )
    links: dict[tuple[str, str], str] = {}
    for index, item in enumerate(links_raw):
        record = _mapping(item, field=f"kinematic link inventory[{index}]")
        if set(record) != {"side", "logical_link_id", "prim_path"}:
            raise ValueError(f"kinematic link inventory[{index}] keys differ")
        side = record["side"]
        logical_id = record["logical_link_id"]
        path = record["prim_path"]
        if side not in {"left", "right"} or not isinstance(logical_id, str):
            raise ValueError(f"kinematic link inventory[{index}] identity differs")
        if not isinstance(path, str) or not path.startswith("/"):
            raise ValueError(f"kinematic link inventory[{index}] prim path differs")
        key = (side, logical_id)
        if key in links:
            raise ValueError("kinematic link inventory contains a duplicate identity")
        links[key] = path
    expected_ids = {
        "arm_link7",
        "palm",
        "thumb_tip",
        "index_finger_tip",
        "middle_finger_tip",
        "ring_finger_tip",
        "pinky_tip",
    }
    expected_keys = {
        (side, logical_id)
        for side in ("left", "right")
        for logical_id in expected_ids
    }
    if set(links) != expected_keys or len(set(links.values())) != 14:
        raise ValueError("008 kinematic inventory must contain 14 unique bilateral link truths")
    return objects, links


def validate_state_truth_inventory(
    frame: SimulationStateFrame,
    *,
    run_id: str,
    objects: Mapping[str, str],
    links: Mapping[tuple[str, str], str],
) -> None:
    """Require one state frame to close exactly against its signed manifest."""

    if frame.run_id != run_id or frame.episode_id != run_id:
        raise ValueError("simulation-state run/episode identity differs")
    if frame.expected_rigid_body_count != len(objects):
        raise ValueError("simulation-state rigid-body expected count differs from manifest")
    if frame.expected_kinematic_link_count != len(links):
        raise ValueError("simulation-state kinematic-link expected count differs from manifest")
    actual_objects = {item.logical_object_id: item for item in frame.rigid_bodies}
    if {key: item.prim_path for key, item in actual_objects.items()} != objects:
        raise ValueError("simulation-state rigid-body inventory differs from manifest")
    if any(not item.valid or item.kinematic for item in actual_objects.values()):
        raise ValueError("dataset dynamic rigid-body truth must be valid and non-kinematic")
    actual_links = {(item.side, item.logical_link_id): item for item in frame.kinematic_links}
    if {key: item.prim_path for key, item in actual_links.items()} != links:
        raise ValueError("simulation-state kinematic-link inventory differs from manifest")
    if any(not item.valid for item in actual_links.values()):
        raise ValueError("dataset kinematic-link truth must be valid")


__all__ = [
    "parse_dataset_truth_inventories",
    "validate_q54_runtime_inventory",
    "validate_state_truth_inventory",
]
