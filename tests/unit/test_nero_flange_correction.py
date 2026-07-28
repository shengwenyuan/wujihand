from __future__ import annotations

from copy import deepcopy
from hashlib import sha256
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt
import pytest
import yaml

from wujihand.adapters.simulation.nero_flange_correction import (
    NERO_FLANGE_CORRECTION_ID,
    load_nero_flange_frame_correction,
    materialize_corrected_nero_urdf,
)
from wujihand.adapters.simulation.nero_urdf_import import load_nero_urdf_facts


ROOT = Path(__file__).parents[2]
PROFILE = (
    ROOT / "configs/profiles/agilex_nero_7f_flange_frame_correction_v1.yaml"
)
SOURCE_URDF = (
    ROOT / "third_party/src/agx_arm_urdf/nero/urdf/nero_description.urdf"
)


def _profile_mapping() -> dict[str, Any]:
    value = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def _write_profile(tmp_path: Path, value: dict[str, Any]) -> Path:
    path = tmp_path / "flange-correction.yaml"
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    return path


def _quaternion_product(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> npt.NDArray[np.float64]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.asarray(
        (
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        )
    )


def test_profile_owns_one_source_locked_j7_correction_and_identity_assembly() -> None:
    profile = load_nero_flange_frame_correction(PROFILE)

    assert profile.correction_id == NERO_FLANGE_CORRECTION_ID
    assert profile.joint_name == "joint7"
    assert (profile.parent_link, profile.child_link) == ("link6", "link7")
    assert profile.flange_normal_axis_local_xyz == (0.0, 0.0, 1.0)
    assert profile.flange_clocking_axis_local_xyz == (1.0, 0.0, 0.0)
    assert profile.assembly_position_m == (0.0, 0.0, 0.0)
    assert profile.assembly_quat_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert _quaternion_product(
        profile.source_origin_quat_wxyz,
        profile.origin_post_rotation_quat_wxyz,
    ) == pytest.approx(profile.corrected_origin_quat_wxyz)
    assert "current_q7_zero_hand_world_pose_is_preserved" in profile.assumptions
    assert "no_right_angle_adapter_is_present" in profile.assumptions


def test_materialized_urdf_changes_only_j7_origin_and_preserves_source(
    tmp_path: Path,
) -> None:
    profile = load_nero_flange_frame_correction(PROFILE)
    source_before = SOURCE_URDF.read_bytes()
    output = tmp_path / "nero-corrected.urdf"

    materialize_corrected_nero_urdf(SOURCE_URDF, output, profile)

    assert SOURCE_URDF.read_bytes() == source_before
    assert sha256(source_before).hexdigest() == profile.source_urdf_sha256
    source = load_nero_urdf_facts(SOURCE_URDF)
    corrected = load_nero_urdf_facts(output)
    assert corrected.inertials == source.inertials
    for source_joint, corrected_joint in zip(source.joints, corrected.joints):
        if source_joint.name == "joint7":
            assert corrected_joint.origin_xyz_m == source_joint.origin_xyz_m
            assert corrected_joint.origin_quaternion_wxyz == pytest.approx(
                profile.corrected_origin_quat_wxyz
            )
            assert corrected_joint != source_joint
        else:
            assert corrected_joint == source_joint


def test_materializer_refuses_source_overwrite(tmp_path: Path) -> None:
    profile = load_nero_flange_frame_correction(PROFILE)

    with pytest.raises(ValueError, match="must not overwrite"):
        materialize_corrected_nero_urdf(SOURCE_URDF, SOURCE_URDF, profile)


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (
            lambda value: value["correction"].update(
                {"corrected_origin_quat_wxyz": [1.0, 0.0, 0.0, 0.0]}
            ),
            "source \\* correction",
        ),
        (
            lambda value: value["frames"].update(
                {"flange_clocking_axis_local_xyz": [0.0, 0.0, 1.0]}
            ),
            "orthogonal",
        ),
        (
            lambda value: value["assembly_contract"].update(
                {"quat_wxyz": [2.0**-0.5, 0.0, 2.0**-0.5, 0.0]}
            ),
            "must be identity",
        ),
        (
            lambda value: value.update({"unexpected": True}),
            "keys differ",
        ),
    ),
)
def test_profile_rejects_frame_and_schema_drift(
    tmp_path: Path,
    mutation: Any,
    message: str,
) -> None:
    value = deepcopy(_profile_mapping())
    mutation(value)

    with pytest.raises(ValueError, match=message):
        load_nero_flange_frame_correction(_write_profile(tmp_path, value))
