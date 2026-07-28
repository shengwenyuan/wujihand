"""Apply one explicit NERO J7/tool-flange frame correction.

The pinned vendor URDF and derived USD remain immutable source evidence.  This
module loads a revisioned Binding compatibility profile and applies the same
fixed J7-origin correction to the live Isaac stage and to a generated Lula
URDF.  Assembly can therefore express the direct flange-to-Hand 2 mount as an
identity transform.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import cast
import xml.etree.ElementTree as ET

import numpy as np
import yaml


NERO_FLANGE_CORRECTION_SCHEMA = "wujihand.nero_flange_frame_correction.v1"
NERO_FLANGE_CORRECTION_ID = "agilex_nero_7f_flange_frame_correction_v1"
NERO_FLANGE_CORRECTION_STATUS = (
    "simulation_binding_correction_owner_confirmed_pending_device_readback"
)

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


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


def _string(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-blank string")
    return value


def _vector(
    value: object,
    *,
    size: int,
    field: str,
) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{field} must contain exactly {size} values")
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise ValueError(f"{field} must contain only numbers")
    result = tuple(float(item) for item in value)
    if not np.isfinite(result).all():
        raise ValueError(f"{field} must contain only finite values")
    return result


def _unit_quaternion(value: object, *, field: str) -> Quaternion:
    result = cast(Quaternion, _vector(value, size=4, field=field))
    if not math.isclose(float(np.linalg.norm(result)), 1.0, abs_tol=1e-9):
        raise ValueError(f"{field} must be a unit quaternion")
    return result


def _unit_axis(value: object, *, field: str) -> Vector3:
    result = cast(Vector3, _vector(value, size=3, field=field))
    if not math.isclose(float(np.linalg.norm(result)), 1.0, abs_tol=1e-9):
        raise ValueError(f"{field} must be a unit vector")
    return result


def _quaternion_product(left: Quaternion, right: Quaternion) -> Quaternion:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    result = (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )
    norm = float(np.linalg.norm(result))
    return cast(Quaternion, tuple(value / norm for value in result))


def _rpy_quaternion(rpy: Vector3) -> Quaternion:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _same_quaternion(left: Sequence[float], right: Sequence[float]) -> bool:
    left_array = np.asarray(left, dtype=np.float64)
    right_array = np.asarray(right, dtype=np.float64)
    return bool(
        np.allclose(left_array, right_array, atol=1e-7)
        or np.allclose(left_array, -right_array, atol=1e-7)
    )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True, slots=True)
class NeroFlangeFrameCorrection:
    """Source-locked J7 origin correction and direct-mount contract."""

    correction_id: str
    status: str
    source_urdf_path: str
    source_urdf_sha256: str
    joint_name: str
    parent_link: str
    child_link: str
    source_origin_xyz_m: Vector3
    source_origin_quat_wxyz: Quaternion
    origin_post_rotation_quat_wxyz: Quaternion
    corrected_origin_quat_wxyz: Quaternion
    corrected_origin_rpy_rad: Vector3
    flange_normal_axis_local_xyz: Vector3
    flange_clocking_axis_local_xyz: Vector3
    assembly_position_m: Vector3
    assembly_quat_wxyz: Quaternion
    assumptions: tuple[str, ...]


def load_nero_flange_frame_correction(
    path: str | Path,
) -> NeroFlangeFrameCorrection:
    """Load and cross-check the exact NERO flange correction profile."""

    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    data = _exact_mapping(
        raw,
        expected=frozenset(
            {
                "schema",
                "correction_id",
                "status",
                "source",
                "correction",
                "frames",
                "assembly_contract",
                "assumptions",
            }
        ),
        field="NERO flange correction",
    )
    if data["schema"] != NERO_FLANGE_CORRECTION_SCHEMA:
        raise ValueError("unsupported NERO flange correction schema")
    if data["correction_id"] != NERO_FLANGE_CORRECTION_ID:
        raise ValueError("unexpected NERO flange correction ID")
    if data["status"] != NERO_FLANGE_CORRECTION_STATUS:
        raise ValueError("unexpected NERO flange correction status")

    source = _exact_mapping(
        data["source"],
        expected=frozenset(
            {
                "urdf_path",
                "urdf_sha256",
                "joint_name",
                "parent_link",
                "child_link",
                "origin_xyz_m",
                "origin_quat_wxyz",
            }
        ),
        field="NERO flange correction.source",
    )
    correction = _exact_mapping(
        data["correction"],
        expected=frozenset(
            {
                "origin_post_rotation_quat_wxyz",
                "corrected_origin_quat_wxyz",
                "corrected_origin_rpy_rad",
            }
        ),
        field="NERO flange correction.correction",
    )
    frames = _exact_mapping(
        data["frames"],
        expected=frozenset(
            {
                "flange_normal_axis_local_xyz",
                "flange_clocking_axis_local_xyz",
            }
        ),
        field="NERO flange correction.frames",
    )
    assembly = _exact_mapping(
        data["assembly_contract"],
        expected=frozenset({"position_m", "quat_wxyz"}),
        field="NERO flange correction.assembly_contract",
    )

    source_quat = _unit_quaternion(
        source["origin_quat_wxyz"],
        field="NERO flange correction.source.origin_quat_wxyz",
    )
    post_quat = _unit_quaternion(
        correction["origin_post_rotation_quat_wxyz"],
        field="NERO flange correction.correction.origin_post_rotation_quat_wxyz",
    )
    corrected_quat = _unit_quaternion(
        correction["corrected_origin_quat_wxyz"],
        field="NERO flange correction.correction.corrected_origin_quat_wxyz",
    )
    corrected_rpy = cast(
        Vector3,
        _vector(
            correction["corrected_origin_rpy_rad"],
            size=3,
            field="NERO flange correction.correction.corrected_origin_rpy_rad",
        ),
    )
    if not _same_quaternion(
        _quaternion_product(source_quat, post_quat),
        corrected_quat,
    ):
        raise ValueError("corrected J7 origin quaternion does not equal source * correction")
    if not _same_quaternion(_rpy_quaternion(corrected_rpy), corrected_quat):
        raise ValueError("corrected J7 RPY and quaternion disagree")

    normal_axis = _unit_axis(
        frames["flange_normal_axis_local_xyz"],
        field="NERO flange correction.frames.flange_normal_axis_local_xyz",
    )
    clocking_axis = _unit_axis(
        frames["flange_clocking_axis_local_xyz"],
        field="NERO flange correction.frames.flange_clocking_axis_local_xyz",
    )
    if not math.isclose(float(np.dot(normal_axis, clocking_axis)), 0.0, abs_tol=1e-9):
        raise ValueError("flange normal and clocking axes must be orthogonal")

    assembly_position = cast(
        Vector3,
        _vector(
            assembly["position_m"],
            size=3,
            field="NERO flange correction.assembly_contract.position_m",
        ),
    )
    assembly_quat = _unit_quaternion(
        assembly["quat_wxyz"],
        field="NERO flange correction.assembly_contract.quat_wxyz",
    )
    if assembly_position != (0.0, 0.0, 0.0) or not _same_quaternion(
        assembly_quat,
        (1.0, 0.0, 0.0, 0.0),
    ):
        raise ValueError("corrected direct flange-to-hand assembly must be identity")

    assumptions_raw = data["assumptions"]
    if (
        not isinstance(assumptions_raw, list)
        or not assumptions_raw
        or any(not isinstance(value, str) or not value for value in assumptions_raw)
        or len(set(assumptions_raw)) != len(assumptions_raw)
    ):
        raise ValueError("NERO flange correction assumptions must be unique strings")

    return NeroFlangeFrameCorrection(
        correction_id=NERO_FLANGE_CORRECTION_ID,
        status=NERO_FLANGE_CORRECTION_STATUS,
        source_urdf_path=_string(
            source["urdf_path"],
            field="NERO flange correction.source.urdf_path",
        ),
        source_urdf_sha256=_string(
            source["urdf_sha256"],
            field="NERO flange correction.source.urdf_sha256",
        ),
        joint_name=_string(
            source["joint_name"],
            field="NERO flange correction.source.joint_name",
        ),
        parent_link=_string(
            source["parent_link"],
            field="NERO flange correction.source.parent_link",
        ),
        child_link=_string(
            source["child_link"],
            field="NERO flange correction.source.child_link",
        ),
        source_origin_xyz_m=cast(
            Vector3,
            _vector(
                source["origin_xyz_m"],
                size=3,
                field="NERO flange correction.source.origin_xyz_m",
            ),
        ),
        source_origin_quat_wxyz=source_quat,
        origin_post_rotation_quat_wxyz=post_quat,
        corrected_origin_quat_wxyz=corrected_quat,
        corrected_origin_rpy_rad=corrected_rpy,
        flange_normal_axis_local_xyz=normal_axis,
        flange_clocking_axis_local_xyz=clocking_axis,
        assembly_position_m=assembly_position,
        assembly_quat_wxyz=assembly_quat,
        assumptions=tuple(assumptions_raw),
    )


def materialize_corrected_nero_urdf(
    source_path: str | Path,
    output_path: str | Path,
    profile: NeroFlangeFrameCorrection,
) -> Path:
    """Generate a Lula URDF with only the approved J7 origin correction."""

    source = Path(source_path).resolve()
    output = Path(output_path).resolve()
    if source == output:
        raise ValueError("corrected NERO URDF must not overwrite the pinned source")
    if _sha256_file(source) != profile.source_urdf_sha256:
        raise RuntimeError("pinned NERO URDF hash differs from flange correction profile")

    tree = ET.parse(source)
    root = tree.getroot()
    joint = root.find(f"joint[@name='{profile.joint_name}']")
    if joint is None:
        raise RuntimeError(f"pinned NERO URDF has no {profile.joint_name}")
    parent = joint.find("parent")
    child = joint.find("child")
    origin = joint.find("origin")
    if (
        parent is None
        or child is None
        or origin is None
        or parent.attrib.get("link") != profile.parent_link
        or child.attrib.get("link") != profile.child_link
    ):
        raise RuntimeError("pinned NERO J7 topology differs from correction profile")
    xyz = tuple(float(value) for value in origin.attrib["xyz"].split())
    rpy = cast(Vector3, tuple(float(value) for value in origin.attrib["rpy"].split()))
    if not np.allclose(xyz, profile.source_origin_xyz_m, atol=1e-9) or not _same_quaternion(
        _rpy_quaternion(rpy),
        profile.source_origin_quat_wxyz,
    ):
        raise RuntimeError("pinned NERO J7 origin differs from correction profile")

    origin.set(
        "rpy",
        " ".join(f"{value:.16g}" for value in profile.corrected_origin_rpy_rad),
    )
    ET.indent(tree, space="    ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output


def apply_isaac_nero_flange_frame_correction(
    stage: object,
    *,
    link7_path: str,
    joint7_path: str,
    profile: NeroFlangeFrameCorrection,
) -> None:
    """Apply the approved J7 correction to one referenced NERO before physics."""

    from pxr import Gf, UsdGeom, UsdPhysics  # type: ignore[import-not-found]

    link7_prim = stage.GetPrimAtPath(link7_path)  # type: ignore[attr-defined]
    joint7_prim = stage.GetPrimAtPath(joint7_path)  # type: ignore[attr-defined]
    if not link7_prim.IsValid() or not link7_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f"NERO link7 rigid body is missing: {link7_path}")
    if not joint7_prim.IsValid() or not joint7_prim.IsA(UsdPhysics.RevoluteJoint):
        raise RuntimeError(f"NERO joint7 is missing: {joint7_path}")

    link7_xform = UsdGeom.Xformable(link7_prim)
    local_matrix = link7_xform.GetLocalTransformation()
    local_position = tuple(float(value) for value in local_matrix.ExtractTranslation())
    source_rotation = local_matrix.ExtractRotationQuat()
    source_quat = (
        float(source_rotation.GetReal()),
        *(float(value) for value in source_rotation.GetImaginary()),
    )
    if not np.allclose(local_position, profile.source_origin_xyz_m, atol=1e-7):
        raise RuntimeError("Isaac NERO link7 source position differs from correction profile")
    if not _same_quaternion(source_quat, profile.source_origin_quat_wxyz):
        raise RuntimeError("Isaac NERO link7 source rotation differs from correction profile")

    orient_ops = [
        operation
        for operation in link7_xform.GetOrderedXformOps()
        if operation.GetOpType() == UsdGeom.XformOp.TypeOrient
    ]
    if len(orient_ops) != 1:
        raise RuntimeError("Isaac NERO link7 must expose exactly one orient xform op")
    orient_ops[0].Set(Gf.Quatf(*profile.corrected_origin_quat_wxyz))

    joint7 = UsdPhysics.RevoluteJoint(joint7_prim)
    joint_position = tuple(float(value) for value in joint7.GetLocalPos0Attr().Get())
    joint_rotation_value = joint7.GetLocalRot0Attr().Get()
    joint_quat = (
        float(joint_rotation_value.GetReal()),
        *(float(value) for value in joint_rotation_value.GetImaginary()),
    )
    if not np.allclose(joint_position, profile.source_origin_xyz_m, atol=1e-7):
        raise RuntimeError("Isaac NERO joint7 source position differs from correction profile")
    if not _same_quaternion(joint_quat, profile.source_origin_quat_wxyz):
        raise RuntimeError("Isaac NERO joint7 source rotation differs from correction profile")
    joint7.GetLocalRot0Attr().Set(Gf.Quatf(*profile.corrected_origin_quat_wxyz))


__all__ = [
    "NERO_FLANGE_CORRECTION_ID",
    "NERO_FLANGE_CORRECTION_SCHEMA",
    "NERO_FLANGE_CORRECTION_STATUS",
    "NeroFlangeFrameCorrection",
    "apply_isaac_nero_flange_frame_correction",
    "load_nero_flange_frame_correction",
    "materialize_corrected_nero_urdf",
]
