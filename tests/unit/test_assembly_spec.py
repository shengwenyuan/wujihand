from __future__ import annotations

from typing import Any

import pytest

from wujihand.specs import AssemblySpec


def _instance(instance_id: str) -> dict[str, Any]:
    return {
        "instance_id": instance_id,
        "asset": {
            "path": f"configs/assets/{instance_id}.yaml",
            "expected_id": instance_id,
        },
        "role": "robot",
        "namespace": f"{instance_id}_ns",
    }


def _attachment(parent: str, child: str) -> dict[str, Any]:
    return {
        "attachment_id": f"{parent}_to_{child}",
        "parent": {"instance": parent, "frame": "flange"},
        "child": {"instance": child, "frame": "base"},
        "transform": {
            "position_m": [0.0, 0.0, 0.0],
            "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
        },
        "assumption": "identity_until_measured",
    }


def _assembly() -> dict[str, Any]:
    return {
        "schema": "wujihand.assembly_spec.v1",
        "assembly_id": "dual_root_fixture",
        "instances": [_instance("left_arm"), _instance("left_hand"), _instance("right_arm")],
        "roots": ["left_arm", "right_arm"],
        "attachments": [_attachment("left_arm", "left_hand")],
    }


def test_assembly_round_trip_supports_a_multi_root_forest() -> None:
    assembly = AssemblySpec.from_mapping(_assembly())

    assert AssemblySpec.from_mapping(assembly.to_mapping()) == assembly
    assert assembly.roots == ("left_arm", "right_arm")
    assert assembly.instance("left_hand").namespace == "left_hand_ns"


def test_assembly_rejects_extra_keys_duplicate_namespace_and_bad_quaternion() -> None:
    extra = _assembly()
    extra["backend"] = "isaac"
    with pytest.raises(ValueError, match="unexpected"):
        AssemblySpec.from_mapping(extra)

    duplicate_namespace = _assembly()
    duplicate_namespace["instances"][1]["namespace"] = "left_arm_ns"
    with pytest.raises(ValueError, match="namespace values must be unique"):
        AssemblySpec.from_mapping(duplicate_namespace)

    bad_quaternion = _assembly()
    bad_quaternion["attachments"][0]["transform"]["quat_wxyz"] = [0.0, 0.0, 0.0, 0.0]
    with pytest.raises(ValueError, match="unit length"):
        AssemblySpec.from_mapping(bad_quaternion)


def test_assembly_rejects_unknown_instances_multiple_parents_and_wrong_roots() -> None:
    unknown = _assembly()
    unknown["attachments"][0]["child"]["instance"] = "missing"
    with pytest.raises(ValueError, match="unknown child"):
        AssemblySpec.from_mapping(unknown)

    multiple_parents = _assembly()
    multiple_parents["attachments"].append(_attachment("right_arm", "left_hand"))
    with pytest.raises(ValueError, match="multiple parents"):
        AssemblySpec.from_mapping(multiple_parents)

    wrong_roots = _assembly()
    wrong_roots["roots"] = ["left_arm"]
    with pytest.raises(ValueError, match="exactly name the forest roots"):
        AssemblySpec.from_mapping(wrong_roots)


def test_assembly_rejects_attachment_cycle() -> None:
    cyclic = _assembly()
    cyclic["attachments"] = [
        _attachment("left_arm", "left_hand"),
        _attachment("left_hand", "left_arm"),
    ]
    cyclic["roots"] = ["right_arm"]

    with pytest.raises(ValueError, match="acyclic forest"):
        AssemblySpec.from_mapping(cyclic)
