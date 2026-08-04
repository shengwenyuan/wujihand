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


def _passive_binding(*, sensor_profile: str | None = None) -> dict[str, Any]:
    return {
        "schema": "wujihand.backend_binding.v2",
        "binding_id": "nero_hand2_beta1_d405_mount_v2_right_isaac_v1",
        "asset_id": "nero_hand2_beta1_d405_mount_v2_right",
        "asset_revision": "v2",
        "asset_side": "right",
        "backend": "isaac",
        "namespace_policy": "prefix",
        "loader": "mesh",
        "artifact": {
            "source": "d405-wrist-rig-assets",
            "source_revision": f"sha256:{'a' * 64}",
            "path": "mount_right_visual.stl",
        },
        "collision_artifact": {
            "source": "d405-wrist-rig-assets",
            "source_revision": f"sha256:{'a' * 64}",
            "path": "mount_right_collision.json",
        },
        "resource_trees": [],
        "root": "mount",
        "frame_map": {
            "hand_interface": "HandInterface",
            "camera_interface": "CameraInterface",
        },
        "group_bindings": [],
        "builder": None,
        "compatibility_profile": None,
        "sensor_profile": sensor_profile,
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


def test_backend_binding_v2_round_trips_passive_mesh_representation() -> None:
    binding = BackendBinding.from_mapping(_passive_binding())

    assert binding.group_bindings == ()
    assert binding.loader == "mesh"
    assert binding.collision_artifact is not None
    assert binding.sensor_profile is None
    assert BackendBinding.from_mapping(binding.to_mapping()) == binding


def test_backend_binding_v2_accepts_explicit_sensor_profile() -> None:
    value = _passive_binding(
        sensor_profile="configs/profiles/isaac_d405_synthetic_wide_angle_140_v1.yaml"
    )

    binding = BackendBinding.from_mapping(value)

    assert binding.sensor_profile == value["sensor_profile"]


def test_backend_binding_versions_preserve_loader_and_group_boundaries() -> None:
    v1_empty = _binding()
    v1_empty["group_bindings"] = []
    with pytest.raises(ValueError, match="must not be empty"):
        BackendBinding.from_mapping(v1_empty)

    v1_mesh = _binding()
    v1_mesh["loader"] = "mesh"
    with pytest.raises(ValueError, match="requires backend binding v2"):
        BackendBinding.from_mapping(v1_mesh)

    v1_with_passive_fields = _binding()
    v1_with_passive_fields["collision_artifact"] = None
    v1_with_passive_fields["sensor_profile"] = None
    with pytest.raises(ValueError, match="unexpected"):
        BackendBinding.from_mapping(v1_with_passive_fields)
