from __future__ import annotations

from copy import deepcopy
from typing import Any

import pytest

from wujihand.specs import BackendBinding


def _binding() -> dict[str, Any]:
    return {
        "schema": "wujihand.backend_binding.v1",
        "binding_id": "hand2_mujoco_v1",
        "asset_id": "wuji_hand2_beta1_right",
        "asset_revision": "beta1",
        "asset_side": "right",
        "backend": "mujoco",
        "namespace_policy": "preserve",
        "loader": "mjcf",
        "artifact": {
            "source": "wuji-description",
            "source_revision": "commit:aee64892ebcf8e3237bedc30231bb09476cbc71d",
            "path": "hand2_beta/body/mjcf/right.xml",
        },
        "resource_trees": [
            {
                "source": "wuji-description",
                "source_revision": "commit:aee64892ebcf8e3237bedc30231bb09476cbc71d",
                "path": "hand2_beta/body/meshes/right",
            }
        ],
        "root": "r_base_link",
        "frame_map": {"hand_base": "r_base_link"},
        "group_bindings": [
            {
                "group_id": "finger_joints",
                "joints": ["finger_1", "finger_2"],
                "actuators": ["finger_1", "finger_2"],
            }
        ],
        "builder": None,
        "compatibility_profile": "configs/profiles/hand2.yaml",
    }


def test_backend_binding_round_trip() -> None:
    binding = BackendBinding.from_mapping(_binding())

    assert BackendBinding.from_mapping(binding.to_mapping()) == binding
    assert binding.backend_frame("hand_base") == "r_base_link"
    assert binding.group_binding("finger_joints").joints == (
        "finger_1",
        "finger_2",
    )


def test_backend_binding_rejects_unknown_key_loader_mismatch_and_unsafe_artifact() -> None:
    extra = _binding()
    extra["world_pose"] = [0.0, 0.0, 0.0]
    with pytest.raises(ValueError, match="unexpected"):
        BackendBinding.from_mapping(extra)

    mismatch = _binding()
    mismatch["backend"] = "isaac"
    with pytest.raises(ValueError, match="mjcf requires backend mujoco"):
        BackendBinding.from_mapping(mismatch)

    unsafe = _binding()
    unsafe["artifact"]["path"] = "../right.xml"
    with pytest.raises(ValueError, match="safe project-relative"):
        BackendBinding.from_mapping(unsafe)

    ambiguous_revision = _binding()
    ambiguous_revision["artifact"]["source_revision"] = (
        "aee64892ebcf8e3237bedc30231bb09476cbc71d"
    )
    with pytest.raises(ValueError, match="kind:value"):
        BackendBinding.from_mapping(ambiguous_revision)

    malformed_revision = _binding()
    malformed_revision["artifact"]["source_revision"] = "commit:not-a-hash"
    with pytest.raises(ValueError, match="commit must be"):
        BackendBinding.from_mapping(malformed_revision)


def test_backend_binding_requires_an_explicit_namespace_policy() -> None:
    prefix = _binding()
    prefix["namespace_policy"] = "prefix"
    assert BackendBinding.from_mapping(prefix).namespace_policy == "prefix"

    ambiguous = _binding()
    ambiguous["namespace_policy"] = "auto"
    with pytest.raises(ValueError, match="namespace_policy must be one of"):
        BackendBinding.from_mapping(ambiguous)


def test_backend_binding_pins_asset_revision_and_side() -> None:
    binding = BackendBinding.from_mapping(_binding())
    assert binding.asset_revision == "beta1"
    assert binding.asset_side == "right"

    invalid_side = _binding()
    invalid_side["asset_side"] = "starboard"
    with pytest.raises(ValueError, match="asset_side must be one of"):
        BackendBinding.from_mapping(invalid_side)


def test_backend_binding_rejects_overlapping_joint_and_actuator_mappings() -> None:
    duplicate = _binding()
    second = deepcopy(duplicate["group_bindings"][0])
    second["group_id"] = "wrist"
    second["joints"] = ["finger_2", "wrist"]
    second["actuators"] = ["finger_2", "wrist"]
    duplicate["group_bindings"].append(second)

    with pytest.raises(ValueError, match="joints must not overlap"):
        BackendBinding.from_mapping(duplicate)


def test_procedural_binding_uses_closed_builder_registry() -> None:
    procedural = _binding()
    procedural.update(
        {
            "backend": "isaac",
            "loader": "procedural",
            "artifact": None,
            "resource_trees": [],
            "builder": "hand2_rotation_mount_d6_v1",
        }
    )
    assert BackendBinding.from_mapping(procedural).artifact is None

    wrong_backend = deepcopy(procedural)
    wrong_backend["backend"] = "mujoco"
    with pytest.raises(ValueError, match="supported only by isaac"):
        BackendBinding.from_mapping(wrong_backend)

    procedural["builder"] = "arbitrary_python"
    with pytest.raises(ValueError, match="builder must be one of"):
        BackendBinding.from_mapping(procedural)
