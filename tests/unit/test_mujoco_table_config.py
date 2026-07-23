from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from wujihand.adapters.simulation import load_fr3_model_profile
from wujihand.adapters.simulation.mujoco_fr3_hand2 import sha256_tree
from wujihand.runtime import load_mujoco_table_scene_config


ROOT = Path(__file__).parents[2]
SCENE = ROOT / "configs/base/mujoco_fr3v2_hand2_right_table_v1.yaml"
ARM_PROFILE = ROOT / "configs/profiles/fr3_v2_menagerie_71f066a.yaml"


def test_scene_pins_long_side_pedestal_timing_light_and_attachment() -> None:
    config = load_mujoco_table_scene_config(SCENE)

    assert config.name == "mujoco_fr3v2_hand2_right_table_v1"
    assert config.table.size_m == pytest.approx((1.60, 1.00, 0.06))
    assert config.arm_pedestal.adjacent_table_edge == "y_max"
    assert config.arm_pedestal.center_xy_m[1] == pytest.approx(
        config.table.y_max_m
        + config.arm_pedestal.bottom_edge_gap_m
        + config.arm_pedestal.bottom_size_m[1] / 2.0
    )
    assert config.arm_mount.forward_axis == "local_+x"
    assert config.arm_mount.position_m == pytest.approx(
        (*config.arm_pedestal.center_xy_m, config.arm_pedestal.top_z_m)
    )
    assert config.arm_pedestal.top_z_m < config.table.top_z_m
    assert config.arm_mount.joint2_clearance_above_table_m == pytest.approx(0.053)
    assert np.linalg.norm(config.observation_light.direction) == pytest.approx(1.0)
    assert config.observation_light.direction[2] < 0.0
    assert config.observation_light.cast_shadow is False
    assert config.control.physics_substeps * config.physics.timestep_s == pytest.approx(
        1.0 / config.control.rate_hz
    )
    assert config.hand_attachment.parent_body == "fr3v2_link8"
    assert config.hand_attachment.child_body == "r_base_link"
    assert (
        config.hand_attachment.assumption
        == "identity_until_physical_adapter_transform_is_measured"
    )


def test_scene_rejects_timing_with_fractional_control_tick(tmp_path: Path) -> None:
    data = yaml.safe_load(SCENE.read_text(encoding="utf-8"))
    data["control"]["physics_substeps"] = 4
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="control period"):
        load_mujoco_table_scene_config(invalid)


def test_scene_rejects_mount_that_is_not_centered_on_pedestal(tmp_path: Path) -> None:
    data = yaml.safe_load(SCENE.read_text(encoding="utf-8"))
    data["arm_mount"]["position_m"][1] += 0.01
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="centered on the pedestal"):
        load_mujoco_table_scene_config(invalid)


def test_scene_rejects_pedestal_that_is_not_outside_long_edge(tmp_path: Path) -> None:
    data = yaml.safe_load(SCENE.read_text(encoding="utf-8"))
    data["arm_pedestal"]["center_xy_m"][1] -= 0.10
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="outside y_max"):
        load_mujoco_table_scene_config(invalid)


def test_scene_rejects_mount_orientation_that_does_not_face_center(tmp_path: Path) -> None:
    data = yaml.safe_load(SCENE.read_text(encoding="utf-8"))
    data["arm_mount"]["quat_wxyz"] = [2**-0.5, 0.0, 0.0, 2**-0.5]
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="face the table center"):
        load_mujoco_table_scene_config(invalid)


def test_scene_rejects_mount_roll_even_when_forward_axis_faces_center(
    tmp_path: Path,
) -> None:
    data = yaml.safe_load(SCENE.read_text(encoding="utf-8"))
    data["arm_mount"]["quat_wxyz"] = [
        0.7064337722,
        0.0308435646,
        -0.0308435646,
        -0.7064337722,
    ]
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="must remain upright"):
        load_mujoco_table_scene_config(invalid)


def test_scene_rejects_light_with_zero_direction(tmp_path: Path) -> None:
    data = yaml.safe_load(SCENE.read_text(encoding="utf-8"))
    data["observation_light"]["direction"] = [0.0, 0.0, 0.0]
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="must be non-zero"):
        load_mujoco_table_scene_config(invalid)


def test_scene_rejects_observation_light_that_casts_shadows(tmp_path: Path) -> None:
    data = yaml.safe_load(SCENE.read_text(encoding="utf-8"))
    data["observation_light"]["cast_shadow"] = True
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="must remain shadow-free"):
        load_mujoco_table_scene_config(invalid)


def test_scene_rejects_unknown_pedestal_key(tmp_path: Path) -> None:
    data = yaml.safe_load(SCENE.read_text(encoding="utf-8"))
    data["arm_pedestal"]["height_typo"] = data["arm_pedestal"]["height_m"]
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="arm_pedestal keys differ from schema"):
        load_mujoco_table_scene_config(invalid)


def test_scene_rejects_nonidentity_transform_under_identity_assumption(tmp_path: Path) -> None:
    data = yaml.safe_load(SCENE.read_text(encoding="utf-8"))
    data["hand_attachment"]["position_m"] = [0.0, 0.0, 0.01]
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="identity transform"):
        load_mujoco_table_scene_config(invalid)


def test_fr3_profile_exposes_only_menagerie_position_contract() -> None:
    profile = load_fr3_model_profile(ARM_PROFILE)

    assert profile.names == tuple(f"fr3v2_joint{index}" for index in range(1, 8))
    assert profile.flange_body_name == "fr3v2_link8"
    assert profile.provenance["commit"] == "71f066ad0be9cd271f7ed58c030243ef157af9f4"
    np.testing.assert_allclose(
        profile.home_position, [0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853]
    )
    with pytest.raises(ValueError, match="joint range"):
        profile.validate_position([0.0] * 7)


def test_asset_tree_hash_is_path_sensitive_and_detects_content_change(tmp_path: Path) -> None:
    assets = tmp_path / "assets"
    nested = assets / "nested"
    nested.mkdir(parents=True)
    (assets / "a.mesh").write_bytes(b"alpha")
    (nested / "b.mesh").write_bytes(b"beta")

    original = sha256_tree(assets)
    (nested / "b.mesh").write_bytes(b"changed")

    assert len(original) == 64
    assert sha256_tree(assets) != original
