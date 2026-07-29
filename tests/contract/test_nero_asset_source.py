from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import cast

import numpy as np
import pytest
import yaml

from wujihand.adapters.simulation.nero_model import (
    NERO_JOINT_NAMES,
    NeroModelProfile,
    load_nero_model_profile,
)
from wujihand.adapters.simulation.nero_urdf_import import (
    NERO_ASSET_TRANSFORMER_EXTENSION_VERSION,
    NERO_ASSET_TRANSFORMER_RULES_EXTENSION_VERSION,
    NERO_IMPORTER_EXTENSION_VERSION,
    NERO_SOURCE_COMMIT,
    NERO_SOURCE_MESH_TREE_PATH,
    NERO_SOURCE_MESH_TREE_SHA256,
    NERO_SOURCE_URDF_PATH,
    NERO_SOURCE_URDF_SHA256,
    load_nero_urdf_import_recipe,
)
from wujihand.integrity import sha256_file
from wujihand.runtime.config_repository import ConfigRepository
from wujihand.runtime.source_lock import SourceLock, SourceRecord


ROOT = Path(__file__).parents[2]
NERO_ASSET = "configs/assets/agilex_nero_v1.yaml"
NERO_PROFILE = "configs/profiles/agilex_nero_q7_provisional_v1.yaml"
NERO_BINDING = "configs/bindings/isaac/agilex_nero_f6642ce0_isaac_6_0_1_v1.yaml"
LEFT_HAND_JOINTS = (
    "l_thumb_cmc_flex",
    "l_thumb_cmc_abd",
    "l_thumb_mcp",
    "l_thumb_ip",
    "l_index_finger_mcp_flex",
    "l_index_finger_mcp_abd",
    "l_index_finger_pip",
    "l_index_finger_dip",
    "l_middle_finger_mcp_flex",
    "l_middle_finger_mcp_abd",
    "l_middle_finger_pip",
    "l_middle_finger_dip",
    "l_ring_finger_mcp_flex",
    "l_ring_finger_mcp_abd",
    "l_ring_finger_pip",
    "l_ring_finger_dip",
    "l_pinky_mcp_flex",
    "l_pinky_mcp_abd",
    "l_pinky_pip",
    "l_pinky_dip",
)


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    assert isinstance(value, Mapping), f"{field} must be a mapping"
    assert all(isinstance(key, str) for key in value), f"{field} keys must be strings"
    return cast(Mapping[str, object], value)


def _string(value: object, *, field: str) -> str:
    assert isinstance(value, str) and value, f"{field} must be a non-blank string"
    return value


def _provenance(profile: NeroModelProfile, key: str) -> Mapping[str, object]:
    return _mapping(profile.provenance.get(key), field=f"provenance.{key}")


def _record_or_skip(source_lock: SourceLock, name: str) -> SourceRecord:
    try:
        return source_lock.record(name)
    except ValueError:
        pytest.skip(
            f"NERO provenance source {name!r} is not yet represented in "
            "third_party/sources.lock.yaml"
        )


def test_nero_asset_owns_backend_neutral_q7_identity() -> None:
    repository = ConfigRepository(ROOT)
    asset = repository.load_asset(NERO_ASSET)
    profile = load_nero_model_profile(ROOT / NERO_PROFILE)

    assert asset.asset_id == "agilex_nero"
    assert asset.revision == "model_f6642ce0_v1"
    assert asset.kind == "robot_arm"
    assert asset.side == "none"
    assert asset.frame_name("base") == profile.base_frame == "base_link"
    assert asset.frame_name("forearm_proximal") == "link4"
    assert asset.frame_name("forearm_distal") == "link5"
    assert asset.frame_name("wrist_housing") == "link6"
    assert asset.frame_name("tool_flange") == profile.tool_flange_frame == "link7"
    group = asset.control_group("arm_joints")
    assert group.layout_id == profile.layout_id == "agilex_nero_q7_v1"
    assert group.dof_count == len(NERO_JOINT_NAMES) == 7
    assert group.command_interface == "position"
    assert asset.canonical_profile == group.joint_profile == NERO_PROFILE
    assert asset.provenance_source == ("third_party/sources.lock.yaml#agilex-agx-arm-urdf")


def test_nero_recipe_and_source_lock_share_one_exact_urdf_pin() -> None:
    repository = ConfigRepository(ROOT)
    source_lock = SourceLock.load(repository)
    source = source_lock.record("agilex-agx-arm-urdf")
    recipe = load_nero_urdf_import_recipe(
        ROOT / "configs/profiles/agilex_nero_isaac_6_0_1_import_v1.yaml"
    )

    assert dict(source.revision)["commit"] == recipe.source_commit == NERO_SOURCE_COMMIT
    assert (
        source.expected_artifact_hash(NERO_SOURCE_URDF_PATH)
        == recipe.urdf_sha256
        == NERO_SOURCE_URDF_SHA256
    )
    assert (
        source.expected_tree_hash(NERO_SOURCE_MESH_TREE_PATH)
        == recipe.mesh_tree_sha256
        == NERO_SOURCE_MESH_TREE_SHA256
    )


@pytest.mark.parametrize(
    ("provenance_key", "source_name"),
    (
        ("urdf", "agilex-agx-arm-urdf"),
        ("sdk", "agilex-pyagxarm"),
    ),
)
def test_nero_q7_file_provenance_matches_existing_source_lock_records(
    provenance_key: str,
    source_name: str,
) -> None:
    profile = load_nero_model_profile(ROOT / NERO_PROFILE)
    source_lock = SourceLock.load(ConfigRepository(ROOT))
    provenance = _provenance(profile, provenance_key)
    source = _record_or_skip(source_lock, source_name)
    revision = dict(source.revision)
    path = _string(provenance.get("path"), field=f"{provenance_key}.path")

    assert revision["url"] == provenance["repository"]
    assert revision["commit"] == provenance["commit"]
    assert source.expected_artifact_hash(path) == provenance["sha256"]


def test_nero_q7_ros2_provenance_matches_existing_source_lock_records() -> None:
    profile = load_nero_model_profile(ROOT / NERO_PROFILE)
    source_lock = SourceLock.load(ConfigRepository(ROOT))
    provenance = _provenance(profile, "ros2")
    ros2_source = _record_or_skip(source_lock, "agilex-agx-arm-ros2")
    urdf_source = _record_or_skip(source_lock, "agilex-agx-arm-urdf")

    assert dict(ros2_source.revision)["url"] == provenance["repository"]
    assert dict(ros2_source.revision)["commit"] == provenance["commit"]
    assert provenance["urdf_submodule_commit"] == dict(urdf_source.revision)["commit"]
    assert provenance["flange_frame"] == profile.tool_flange_frame
    assert provenance["tcp_frame"] == "tcp_link"
    assert dict(ros2_source.artifacts) == {
        ".gitmodules": ("9a1cb4b5cfb8d3fa6c085bc164f6e5b39eb606b9b484a98412985bb454364fac"),
        "src/agx_arm_moveit/config/agx_arm.urdf.xacro": (
            "ab4b8b16f8ce532a9a448f4032befc3cce85377671a55e2bc560bad74d817443"
        ),
    }


def test_nero_q7_manual_provenance_matches_existing_source_lock_record() -> None:
    profile = load_nero_model_profile(ROOT / NERO_PROFILE)
    source_lock = SourceLock.load(ConfigRepository(ROOT))
    provenance = _provenance(profile, "manual")
    source = _record_or_skip(source_lock, "agilex-nero-user-manual-v1")
    revision = dict(source.revision)

    assert revision["url"] == provenance["source_url"]
    assert revision["sha256"] == provenance["original_sha256"]
    assert provenance["title"] == "NERO用户手册"
    assert provenance["version"] == "V1.0.0"
    assert provenance["sections"] == [
        "1.2 性能参数",
        "3.3 机械臂DH参数说明",
        "6.8.1关节限制设置",
    ]


def test_nero_body_qr_source_is_pinned_and_urdf_is_its_conservative_subset() -> None:
    profile = load_nero_model_profile(ROOT / NERO_PROFILE)
    source_lock = SourceLock.load(ConfigRepository(ROOT))
    source = source_lock.record("agilex-nero-7f-body-qr-2026-07-17")
    revision = dict(source.revision)

    assert revision == {
        "kind": "documentation",
        "sha256": "67663ff94a05e642a43162c2ff4a1a95d1926a6236114f9904d1544b66e9c700",
        "url": "https://qr61.cn/oMm9uo/q4oW6ZW",
    }

    qr_lower_rad = np.deg2rad(
        np.asarray([-157.0, -102.0, -160.0, -60.0, -160.0, -44.0, -97.0])
    )
    qr_upper_rad = np.deg2rad(
        np.asarray([157.0, 102.0, 160.0, 125.0, 160.0, 57.0, 97.0])
    )
    urdf_lower = np.asarray(profile.layout.lower)
    urdf_upper = np.asarray(profile.layout.upper)

    assert np.all(urdf_lower > qr_lower_rad)
    assert np.all(urdf_upper < qr_upper_rad)


def test_nero_isaac_binding_locks_the_reproducible_derived_package() -> None:
    repository = ConfigRepository(ROOT)
    source_lock = SourceLock.load(repository)
    asset = repository.load_asset(NERO_ASSET)
    profile = load_nero_model_profile(ROOT / NERO_PROFILE)
    binding = repository.load_binding(NERO_BINDING)
    source = source_lock.record("agilex-nero-isaac-6-0-1")

    assert binding.asset_id == asset.asset_id == "agilex_nero"
    assert binding.asset_revision == asset.revision == "model_f6642ce0_v1"
    assert binding.asset_side == asset.side == "none"
    assert binding.backend == "isaac"
    assert binding.namespace_policy == "prefix"
    assert binding.root == binding.backend_frame(asset.frame_name("base"))
    assert dict(binding.frame_map) == {
        profile.base_frame: profile.base_frame,
        asset.frame_name("forearm_proximal"): "link4",
        asset.frame_name("forearm_distal"): "link5",
        asset.frame_name("wrist_housing"): "link6",
        profile.tool_flange_frame: profile.tool_flange_frame,
    }
    assert binding.group_binding("arm_joints").joints == profile.layout.names == NERO_JOINT_NAMES
    assert binding.artifact is not None
    resolved_artifact = source_lock.resolve(binding.artifact)
    resolved_trees = tuple(
        source_lock.resolve(resource, tree=True) for resource in binding.resource_trees
    )

    assert resolved_artifact.source == source
    assert resolved_artifact.relative_path == binding.artifact.path
    assert resolved_artifact.expected_sha256 == source.expected_artifact_hash(binding.artifact.path)
    assert resolved_artifact.absolute_path == (
        ROOT / source.local_runtime_path / binding.artifact.path
    )
    assert tuple(tree.relative_path for tree in resolved_trees) == ("nero_description",)
    assert (
        resolved_trees[0].expected_sha256
        == source.expected_tree_hash(binding.resource_trees[0].path)
        == dict(source.revision)["sha256"]
        == binding.artifact.source_revision.removeprefix("sha256:")
    )
    assert len(source.expected_artifact_hash("nero_description.import.json")) == 64


def test_nero_generated_source_locks_its_derivation_code_and_versions() -> None:
    document = yaml.safe_load((ROOT / "third_party/sources.lock.yaml").read_text(encoding="utf-8"))
    root = _mapping(document, field="source lock")
    sources = root.get("sources")
    assert isinstance(sources, list)
    generated = next(
        _mapping(source, field="generated NERO source")
        for source in sources
        if isinstance(source, Mapping) and source.get("name") == "agilex-nero-isaac-6-0-1"
    )
    generator_path = _string(
        generated.get("generated_by"),
        field="generated_by",
    )
    adapter_path = _string(
        generated.get("import_adapter"),
        field="import_adapter",
    )
    recipe_path = _string(
        generated.get("generator_recipe"),
        field="generator_recipe",
    )

    assert generated["generator_sha256"] == sha256_file(ROOT / generator_path)
    assert generated["import_adapter_sha256"] == sha256_file(ROOT / adapter_path)
    assert generated["generator_recipe_sha256"] == sha256_file(ROOT / recipe_path)
    assert generated["isaac_distribution_version"] == "6.0.1.0"
    assert generated["urdf_importer_extension_version"] == NERO_IMPORTER_EXTENSION_VERSION
    assert (
        generated["asset_transformer_extension_version"] == NERO_ASSET_TRANSFORMER_EXTENSION_VERSION
    )
    assert (
        generated["asset_transformer_rules_extension_version"]
        == NERO_ASSET_TRANSFORMER_RULES_EXTENSION_VERSION
    )


def test_left_hand_asset_is_backend_neutral_and_side_explicit() -> None:
    asset = ConfigRepository(ROOT).load_asset("configs/assets/wuji_hand2_beta1_left_v1.yaml")

    assert asset.asset_id == "wuji_hand2_beta1_left"
    assert asset.product == "wuji_hand_2"
    assert asset.side == "left"
    assert asset.frame_name("base") == "hand_base"
    assert asset.frame_name("pose_command") == "hand2_left_neutral"
    group = asset.control_group("finger_joints")
    assert group.layout_id == "wuji_hand2_left_firmware_v1"
    assert group.dof_count == 20


def test_left_hand_source_is_locked_to_the_existing_right_hand_tag() -> None:
    source = SourceLock.load(ConfigRepository(ROOT)).record("wuji-description")

    assert dict(source.revision) == {
        "commit": "aee64892ebcf8e3237bedc30231bb09476cbc71d",
        "kind": "git",
        "tag": "v2026.6.27",
        "url": "https://github.com/wuji-technology/wuji-description.git",
    }
    assert (
        source.expected_artifact_hash("hand2_beta/body/urdf/left.urdf")
        == "d93de699b5bbe0a573ea9725ebb7e6ad6547b6ed3593f7f5591664ee220648f0"
    )
    assert (
        source.expected_artifact_hash("hand2_beta/body/usd/left/wujihand.usd")
        == "646287f10ac0a2097bf602facc02c9af17f0f1cf8982c38037f69bb695492eca"
    )
    assert (
        source.expected_tree_hash("hand2_beta/body/meshes/left")
        == "0913af28e4b94e4dedb0d2169018a684f100dec0795f38457ad63924e959dda2"
    )
    assert (
        source.expected_tree_hash("hand2_beta/body/usd/left")
        == "bbe2f1939f3e36ddf58b5a3864440a618e0be9118c3d5cb9ce5132b929cd9d22"
    )
    assert (
        source.expected_tree_hash("hand2_beta/body/usd/right")
        == "1992b90288b6d414cc487e2bc37ba6853970cae7cb74b6177bf56d46342383b4"
    )


def test_physical_hand_bindings_are_namespaced_and_lock_supporting_usd() -> None:
    repository = ConfigRepository(ROOT)
    left = repository.load_binding(
        "configs/bindings/isaac/wuji_hand2_beta1_left_v2026_6_27_physical_v1.yaml"
    )
    right = repository.load_binding(
        "configs/bindings/isaac/wuji_hand2_beta1_right_v2026_6_27_physical_v1.yaml"
    )

    assert left.binding_id.endswith("_physical")
    assert right.binding_id.endswith("_physical")
    assert left.namespace_policy == right.namespace_policy == "prefix"
    assert left.asset_side == "left"
    assert right.asset_side == "right"
    assert left.root == "l_base_link"
    assert right.root == "r_base_link"
    left_joints = left.group_binding("finger_joints").joints
    right_joints = right.group_binding("finger_joints").joints
    assert left_joints == LEFT_HAND_JOINTS
    assert len(left_joints) == len(right_joints) == 20
    assert tuple(name.removeprefix("l_") for name in left_joints) == tuple(
        name.removeprefix("r_") for name in right_joints
    )
    assert left.artifact is not None
    assert right.artifact is not None
    assert left.artifact.source_revision == right.artifact.source_revision
    assert left.artifact.path.replace("/left/", "/right/") == right.artifact.path
    assert tuple(tree.path for tree in left.resource_trees) == (
        "hand2_beta/body/usd/left",
        "hand2_beta/body/meshes/left",
    )
    assert tuple(tree.path for tree in right.resource_trees) == (
        "hand2_beta/body/usd/right",
        "hand2_beta/body/meshes/right",
    )
    source_lock = SourceLock.load(repository)
    assert source_lock.resolve(left.artifact).relative_path == left.artifact.path
    assert source_lock.resolve(right.artifact).relative_path == right.artifact.path
    for resource in (*left.resource_trees, *right.resource_trees):
        resolved = source_lock.resolve(resource, tree=True)
        assert resolved.relative_path == resource.path
        assert len(resolved.expected_sha256) == 64
