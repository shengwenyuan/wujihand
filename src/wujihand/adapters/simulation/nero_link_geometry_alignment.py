"""Apply a source-locked NERO link-geometry alignment in Isaac.

The pinned URDF/USD and every joint frame remain immutable.  This Binding
overlay rotates only the selected link's visual and collision representations,
then rotates the corresponding center of mass and principal inertia axes.  It
therefore corrects an Isaac representation without changing NERO kinematics,
Lula, the Assembly transform, or the Hand 2 world pose.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray
import yaml


NERO_LINK_GEOMETRY_ALIGNMENT_SCHEMA = (
    "wujihand.nero_link_geometry_alignment.v1"
)
NERO_LINK_GEOMETRY_ALIGNMENT_ID = (
    "agilex_nero_7f_link6_geometry_alignment_v1"
)
NERO_LINK_GEOMETRY_ALIGNMENT_STATUS = (
    "simulation_binding_geometry_alignment_approved_pending_cad"
)

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or any(
        not isinstance(key, str) for key in value
    ):
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
            f"{field} keys differ: "
            f"missing={sorted(expected - frozenset(result))}, "
            f"unexpected={sorted(frozenset(result) - expected)}"
        )
    return result


def _string(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-blank string")
    return value


def _positive_float(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{field} must be positive and finite")
    return result


def _vector(
    value: object,
    *,
    size: int,
    field: str,
) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{field} must contain exactly {size} values")
    if any(
        isinstance(item, bool) or not isinstance(item, (int, float))
        for item in value
    ):
        raise ValueError(f"{field} must contain only numbers")
    result = tuple(float(item) for item in value)
    if not np.isfinite(result).all():
        raise ValueError(f"{field} must contain only finite values")
    return result


def _unit_quaternion(value: object, *, field: str) -> Quaternion:
    result = cast(Quaternion, _vector(value, size=4, field=field))
    if not math.isclose(
        float(np.linalg.norm(result)),
        1.0,
        rel_tol=0.0,
        abs_tol=1e-6,
    ):
        raise ValueError(f"{field} must be a unit quaternion")
    normalized = tuple(value / float(np.linalg.norm(result)) for value in result)
    return cast(Quaternion, normalized)


def _unit_axis(value: object, *, field: str) -> Vector3:
    result = cast(Vector3, _vector(value, size=3, field=field))
    if not math.isclose(
        float(np.linalg.norm(result)),
        1.0,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise ValueError(f"{field} must be a unit vector")
    return result


def _positive_vector(value: object, *, field: str) -> Vector3:
    result = cast(Vector3, _vector(value, size=3, field=field))
    if any(component <= 0.0 for component in result):
        raise ValueError(f"{field} values must be positive")
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


def _rotation_matrix(quaternion: Quaternion) -> NDArray[np.float64]:
    w, x, y, z = quaternion
    return np.asarray(
        (
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ),
            (
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ),
            (
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
        ),
        dtype=np.float64,
    )


def _same_quaternion(
    left: Sequence[float],
    right: Sequence[float],
    *,
    atol: float = 1e-6,
) -> bool:
    left_array = np.asarray(left, dtype=np.float64)
    right_array = np.asarray(right, dtype=np.float64)
    return bool(
        np.allclose(left_array, right_array, atol=atol, rtol=0.0)
        or np.allclose(left_array, -right_array, atol=atol, rtol=0.0)
    )


@dataclass(frozen=True, slots=True)
class NeroLinkGeometryAlignment:
    """Typed NERO Binding correction for one link representation."""

    alignment_id: str
    status: str
    source_urdf_path: str
    source_urdf_sha256: str
    link_name: str
    source_cylinder_axis_local_xyz: Vector3
    visual_child_name: str
    collision_child_name: str
    source_child_quat_wxyz: Quaternion
    source_mass_kg: float
    source_center_of_mass_m: Vector3
    source_diagonal_inertia_kg_m2: Vector3
    source_principal_axes_quat_wxyz: Quaternion
    geometry_post_rotation_quat_wxyz: Quaternion
    corrected_cylinder_axis_local_xyz: Vector3
    corrected_center_of_mass_m: Vector3
    corrected_principal_axes_quat_wxyz: Quaternion
    assumptions: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class NeroLinkGeometryAlignmentHandles:
    """Stable stage paths modified by one Binding overlay."""

    link_path: str
    visual_path: str
    collision_path: str


def load_nero_link_geometry_alignment(
    path: str | Path,
) -> NeroLinkGeometryAlignment:
    """Load and algebraically cross-check the exact link6 profile."""

    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    data = _exact_mapping(
        raw,
        expected=frozenset(
            {
                "schema",
                "alignment_id",
                "status",
                "source",
                "isaac_representation",
                "correction",
                "assumptions",
            }
        ),
        field="NERO link geometry alignment",
    )
    if data["schema"] != NERO_LINK_GEOMETRY_ALIGNMENT_SCHEMA:
        raise ValueError("unsupported NERO link geometry alignment schema")
    if data["alignment_id"] != NERO_LINK_GEOMETRY_ALIGNMENT_ID:
        raise ValueError("unexpected NERO link geometry alignment ID")
    if data["status"] != NERO_LINK_GEOMETRY_ALIGNMENT_STATUS:
        raise ValueError("unexpected NERO link geometry alignment status")

    source = _exact_mapping(
        data["source"],
        expected=frozenset(
            {
                "urdf_path",
                "urdf_sha256",
                "link_name",
                "cylinder_axis_local_xyz",
            }
        ),
        field="NERO link geometry alignment.source",
    )
    representation = _exact_mapping(
        data["isaac_representation"],
        expected=frozenset(
            {
                "visual_child_name",
                "collision_child_name",
                "source_child_quat_wxyz",
                "source_mass_kg",
                "source_center_of_mass_m",
                "source_diagonal_inertia_kg_m2",
                "source_principal_axes_quat_wxyz",
            }
        ),
        field="NERO link geometry alignment.isaac_representation",
    )
    correction = _exact_mapping(
        data["correction"],
        expected=frozenset(
            {
                "geometry_post_rotation_quat_wxyz",
                "corrected_cylinder_axis_local_xyz",
                "corrected_center_of_mass_m",
                "corrected_principal_axes_quat_wxyz",
            }
        ),
        field="NERO link geometry alignment.correction",
    )

    source_axis = _unit_axis(
        source["cylinder_axis_local_xyz"],
        field="source.cylinder_axis_local_xyz",
    )
    source_child_quat = _unit_quaternion(
        representation["source_child_quat_wxyz"],
        field="isaac_representation.source_child_quat_wxyz",
    )
    if not _same_quaternion(source_child_quat, (1.0, 0.0, 0.0, 0.0)):
        raise ValueError("source link geometry child transform must be identity")
    source_mass = _positive_float(
        representation["source_mass_kg"],
        field="isaac_representation.source_mass_kg",
    )
    source_com = cast(
        Vector3,
        _vector(
            representation["source_center_of_mass_m"],
            size=3,
            field="isaac_representation.source_center_of_mass_m",
        ),
    )
    source_inertia = _positive_vector(
        representation["source_diagonal_inertia_kg_m2"],
        field="isaac_representation.source_diagonal_inertia_kg_m2",
    )
    source_principal = _unit_quaternion(
        representation["source_principal_axes_quat_wxyz"],
        field="isaac_representation.source_principal_axes_quat_wxyz",
    )
    post_rotation = _unit_quaternion(
        correction["geometry_post_rotation_quat_wxyz"],
        field="correction.geometry_post_rotation_quat_wxyz",
    )
    corrected_axis = _unit_axis(
        correction["corrected_cylinder_axis_local_xyz"],
        field="correction.corrected_cylinder_axis_local_xyz",
    )
    corrected_com = cast(
        Vector3,
        _vector(
            correction["corrected_center_of_mass_m"],
            size=3,
            field="correction.corrected_center_of_mass_m",
        ),
    )
    corrected_principal = _unit_quaternion(
        correction["corrected_principal_axes_quat_wxyz"],
        field="correction.corrected_principal_axes_quat_wxyz",
    )
    rotation = _rotation_matrix(post_rotation)
    if not np.allclose(
        rotation @ np.asarray(source_axis),
        corrected_axis,
        atol=1e-9,
        rtol=0.0,
    ):
        raise ValueError("corrected cylinder axis does not equal rotation * source axis")
    if not np.allclose(
        rotation @ np.asarray(source_com),
        corrected_com,
        atol=1e-10,
        rtol=0.0,
    ):
        raise ValueError("corrected center of mass does not equal rotation * source")
    if not _same_quaternion(
        _quaternion_product(post_rotation, source_principal),
        corrected_principal,
    ):
        raise ValueError("corrected principal axes do not equal rotation * source")

    assumptions_raw = data["assumptions"]
    if (
        not isinstance(assumptions_raw, list)
        or not assumptions_raw
        or any(
            not isinstance(value, str) or not value.strip()
            for value in assumptions_raw
        )
        or len(set(assumptions_raw)) != len(assumptions_raw)
    ):
        raise ValueError("NERO link geometry assumptions must be unique strings")

    return NeroLinkGeometryAlignment(
        alignment_id=NERO_LINK_GEOMETRY_ALIGNMENT_ID,
        status=NERO_LINK_GEOMETRY_ALIGNMENT_STATUS,
        source_urdf_path=_string(source["urdf_path"], field="source.urdf_path"),
        source_urdf_sha256=_string(
            source["urdf_sha256"], field="source.urdf_sha256"
        ),
        link_name=_string(source["link_name"], field="source.link_name"),
        source_cylinder_axis_local_xyz=source_axis,
        visual_child_name=_string(
            representation["visual_child_name"],
            field="isaac_representation.visual_child_name",
        ),
        collision_child_name=_string(
            representation["collision_child_name"],
            field="isaac_representation.collision_child_name",
        ),
        source_child_quat_wxyz=source_child_quat,
        source_mass_kg=source_mass,
        source_center_of_mass_m=source_com,
        source_diagonal_inertia_kg_m2=source_inertia,
        source_principal_axes_quat_wxyz=source_principal,
        geometry_post_rotation_quat_wxyz=post_rotation,
        corrected_cylinder_axis_local_xyz=corrected_axis,
        corrected_center_of_mass_m=corrected_com,
        corrected_principal_axes_quat_wxyz=corrected_principal,
        assumptions=tuple(assumptions_raw),
    )


def _quat_tuple(value: Any) -> Quaternion:
    imaginary = value.GetImaginary()
    return (
        float(value.GetReal()),
        float(imaginary[0]),
        float(imaginary[1]),
        float(imaginary[2]),
    )


def apply_isaac_nero_link_geometry_alignment(
    stage: Any,
    *,
    link_path: str,
    profile: NeroLinkGeometryAlignment,
) -> NeroLinkGeometryAlignmentHandles:
    """Apply the profile to one referenced NERO before physics starts."""

    from pxr import Gf, UsdGeom, UsdPhysics  # type: ignore[import-not-found]

    link = stage.GetPrimAtPath(link_path)
    if not link.IsValid() or not link.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f"NERO rigid link is missing: {link_path}")
    if link.GetName() != profile.link_name:
        raise RuntimeError(
            f"NERO alignment expected {profile.link_name}, got {link.GetName()}"
        )
    direct_children = {child.GetName(): child for child in link.GetChildren()}
    expected_children = {
        profile.visual_child_name,
        profile.collision_child_name,
    }
    if not expected_children.issubset(direct_children):
        raise RuntimeError(
            "NERO link representation children differ: "
            f"missing={sorted(expected_children - set(direct_children))}"
        )

    for child_name in sorted(expected_children):
        child = direct_children[child_name]
        xform = UsdGeom.Xformable(child)
        if not Gf.IsClose(
            xform.GetLocalTransformation(),
            Gf.Matrix4d(1.0),
            1e-12,
        ):
            raise RuntimeError(
                f"NERO {profile.link_name}/{child_name} source transform is not identity"
            )
        orient_ops = [
            operation
            for operation in xform.GetOrderedXformOps()
            if operation.GetOpType() == UsdGeom.XformOp.TypeOrient
        ]
        if len(orient_ops) != 1 or not _same_quaternion(
            _quat_tuple(orient_ops[0].Get()),
            profile.source_child_quat_wxyz,
        ):
            raise RuntimeError(
                f"NERO {profile.link_name}/{child_name} source orient differs"
            )
        orient_ops[0].Set(Gf.Quatd(*profile.geometry_post_rotation_quat_wxyz))

    mass = UsdPhysics.MassAPI(link)
    source_mass = mass.GetMassAttr().Get()
    source_com = mass.GetCenterOfMassAttr().Get()
    source_inertia = mass.GetDiagonalInertiaAttr().Get()
    source_principal = mass.GetPrincipalAxesAttr().Get()
    if (
        source_mass is None
        or source_com is None
        or source_inertia is None
        or source_principal is None
    ):
        raise RuntimeError(f"NERO {profile.link_name} mass properties are incomplete")
    if not math.isclose(
        float(source_mass),
        profile.source_mass_kg,
        rel_tol=0.0,
        abs_tol=1e-7,
    ):
        raise RuntimeError(f"NERO {profile.link_name} mass differs from profile")
    if not np.allclose(
        tuple(source_com),
        profile.source_center_of_mass_m,
        atol=1e-9,
        rtol=0.0,
    ):
        raise RuntimeError(
            f"NERO {profile.link_name} center of mass differs from profile"
        )
    if not np.allclose(
        tuple(source_inertia),
        profile.source_diagonal_inertia_kg_m2,
        atol=1e-10,
        rtol=0.0,
    ):
        raise RuntimeError(
            f"NERO {profile.link_name} diagonal inertia differs from profile"
        )
    if not _same_quaternion(
        _quat_tuple(source_principal),
        profile.source_principal_axes_quat_wxyz,
    ):
        raise RuntimeError(
            f"NERO {profile.link_name} principal axes differ from profile"
        )
    mass.GetCenterOfMassAttr().Set(
        Gf.Vec3f(*profile.corrected_center_of_mass_m)
    )
    mass.GetPrincipalAxesAttr().Set(
        Gf.Quatf(*profile.corrected_principal_axes_quat_wxyz)
    )

    return NeroLinkGeometryAlignmentHandles(
        link_path=link_path,
        visual_path=str(direct_children[profile.visual_child_name].GetPath()),
        collision_path=str(
            direct_children[profile.collision_child_name].GetPath()
        ),
    )


__all__ = [
    "NERO_LINK_GEOMETRY_ALIGNMENT_ID",
    "NERO_LINK_GEOMETRY_ALIGNMENT_SCHEMA",
    "NERO_LINK_GEOMETRY_ALIGNMENT_STATUS",
    "NeroLinkGeometryAlignment",
    "NeroLinkGeometryAlignmentHandles",
    "apply_isaac_nero_link_geometry_alignment",
    "load_nero_link_geometry_alignment",
]
