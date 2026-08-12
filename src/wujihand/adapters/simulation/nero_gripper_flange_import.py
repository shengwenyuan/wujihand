"""Pure helpers for the pinned NERO gripper-flange Isaac candidate."""

from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass, replace
import math
from pathlib import Path
from typing import Any, cast
import xml.etree.ElementTree as ET

import yaml

from .nero_model import NERO_CHILD_LINKS, NERO_JOINT_NAMES
from .nero_urdf_import import NeroUrdfFacts


PROFILE_SCHEMA = "wujihand.nero_gripper_flange_import.v1"
OFFICIAL_MESH_PREFIX = "package://agx_arm_description/agx_arm_urdf/"

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class NeroGripperFlangeImportProfile:
    profile_id: str
    status: str
    base_recipe: str
    source_lock_id: str
    source_commit: str
    xacro_path: str
    ros_package_name: str
    flange_post_rotation_rpy_rad: Vector3
    output_root: str
    robot_name: str
    assumptions: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class NeroGripperFlangeFacts:
    link_name: str
    joint_name: str
    parent_link: str
    position_m: Vector3
    quaternion_wxyz: Quaternion
    mass_kg: float
    center_of_mass_m: Vector3
    visual_mesh_uri: str
    collision_mesh_uri: str


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(Mapping[str, object], value)


def _string(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-blank string")
    return value


def _project_path(value: object, *, field: str) -> str:
    text = _string(value, field=field)
    path = Path(text)
    if path.is_absolute() or ".." in path.parts or path.as_posix() != text:
        raise ValueError(f"{field} must be a normalized project-relative path")
    return text


def _vector(value: str | None, *, field: str) -> Vector3:
    if value is None:
        raise ValueError(f"{field} is missing")
    result = tuple(float(component) for component in value.split())
    if len(result) != 3 or not all(math.isfinite(component) for component in result):
        raise ValueError(f"{field} must contain three finite values")
    return result


def _profile_vector(value: object, *, field: str) -> Vector3:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{field} must contain three values")
    if any(
        isinstance(component, bool) or not isinstance(component, (int, float))
        for component in value
    ):
        raise ValueError(f"{field} must contain only numbers")
    result = cast(Vector3, tuple(float(component) for component in value))
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{field} must contain finite values")
    return result


def _quaternion_from_rpy(rpy: Vector3) -> Quaternion:
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


def _quaternion_product(left: Quaternion, right: Quaternion) -> Quaternion:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def _rpy_from_quaternion(quaternion: Quaternion) -> Vector3:
    w, x, y, z = quaternion
    return (
        math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
        math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))),
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
    )


def _format_vector(vector: Vector3) -> str:
    return " ".join(f"{component:.17g}" for component in vector)


def _post_rotate_orientation(origin: ET.Element, rotation_rpy: Vector3) -> None:
    orientation = _quaternion_product(
        _quaternion_from_rpy(_vector(origin.attrib.get("rpy"), field="origin.rpy")),
        _quaternion_from_rpy(rotation_rpy),
    )
    origin.attrib["rpy"] = _format_vector(_rpy_from_quaternion(orientation))


def load_nero_gripper_flange_import_profile(
    path: str | Path,
) -> NeroGripperFlangeImportProfile:
    data = _mapping(yaml.safe_load(Path(path).read_text(encoding="utf-8")), field="profile")
    if data.get("schema") != PROFILE_SCHEMA:
        raise ValueError("unsupported NERO gripper-flange import profile")
    source = _mapping(data.get("flange_source"), field="profile.flange_source")
    flange_clocking = _mapping(
        data.get("gripper_flange_clocking"),
        field="profile.gripper_flange_clocking",
    )
    output = _mapping(data.get("output"), field="profile.output")
    assumptions = data.get("assumptions")
    if not isinstance(assumptions, list):
        raise ValueError("profile.assumptions must be a list")
    if flange_clocking.get("link_name") != "gripper_flange":
        raise ValueError("profile.gripper_flange_clocking.link_name must be gripper_flange")
    flange_rotation = _profile_vector(
        flange_clocking.get("post_rotation_rpy_rad"),
        field="profile.gripper_flange_clocking.post_rotation_rpy_rad",
    )
    if not all(
        math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
        for actual, expected in zip(
            flange_rotation,
            (0.0, 0.0, 0.0),
            strict=True,
        )
    ):
        raise ValueError("gripper flange must retain the pinned vendor clocking")
    return NeroGripperFlangeImportProfile(
        profile_id=_string(data.get("profile_id"), field="profile.profile_id"),
        status=_string(data.get("status"), field="profile.status"),
        base_recipe=_project_path(data.get("base_recipe"), field="profile.base_recipe"),
        source_lock_id=_string(source.get("lock_id"), field="profile.flange_source.lock_id"),
        source_commit=_string(source.get("commit"), field="profile.flange_source.commit"),
        xacro_path=_project_path(
            source.get("xacro_path"), field="profile.flange_source.xacro_path"
        ),
        ros_package_name=_string(
            source.get("ros_package_name"), field="profile.flange_source.ros_package_name"
        ),
        flange_post_rotation_rpy_rad=flange_rotation,
        output_root=_project_path(output.get("root"), field="profile.output.root"),
        robot_name=_string(output.get("robot_name"), field="profile.output.robot_name"),
        assumptions=tuple(
            _string(value, field=f"profile.assumptions[{index}]")
            for index, value in enumerate(assumptions)
        ),
    )


def _mesh_uri(link: ET.Element, kind: str) -> str:
    mesh = link.find(f"{kind}/geometry/mesh")
    if mesh is None:
        raise ValueError(f"gripper_flange has no {kind} mesh")
    return _string(mesh.attrib.get("filename"), field=f"gripper_flange.{kind}.mesh")


def load_nero_gripper_flange_facts(path: str | Path) -> NeroGripperFlangeFacts:
    root = ET.parse(Path(path)).getroot()
    link = root.find("link[@name='gripper_flange']")
    joint = root.find("joint[@name='gripper_flange_joint']")
    if link is None or joint is None or joint.attrib.get("type") != "fixed":
        raise ValueError("pinned flange xacro must contain its one fixed link and joint")
    origin = joint.find("origin")
    inertial_origin = link.find("inertial/origin")
    mass = link.find("inertial/mass")
    parent = joint.find("parent")
    child = joint.find("child")
    if None in (origin, inertial_origin, mass, parent, child):
        raise ValueError("pinned flange xacro is incomplete")
    mass_kg = float(cast(ET.Element, mass).attrib.get("value", "nan"))
    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise ValueError("pinned flange mass must be positive")
    if cast(ET.Element, child).attrib.get("link") != "gripper_flange":
        raise ValueError("pinned flange joint child differs from gripper_flange")
    return NeroGripperFlangeFacts(
        link_name="gripper_flange",
        joint_name="gripper_flange_joint",
        parent_link=_string(
            cast(ET.Element, parent).attrib.get("link"), field="gripper_flange.parent"
        ),
        position_m=_vector(cast(ET.Element, origin).attrib.get("xyz"), field="joint.xyz"),
        quaternion_wxyz=_quaternion_from_rpy(
            _vector(cast(ET.Element, origin).attrib.get("rpy"), field="joint.rpy")
        ),
        mass_kg=mass_kg,
        center_of_mass_m=_vector(
            cast(ET.Element, inertial_origin).attrib.get("xyz"), field="inertial.xyz"
        ),
        visual_mesh_uri=_mesh_uri(link, "visual"),
        collision_mesh_uri=_mesh_uri(link, "collision"),
    )


def build_nero_gripper_flange_urdf(
    *,
    base_urdf: str | Path,
    flange_xacro: str | Path,
    output: str | Path,
    flange_package_name: str,
    flange_post_rotation_rpy_rad: Vector3,
) -> NeroGripperFlangeFacts:
    base = ET.parse(Path(base_urdf))
    extension = ET.parse(Path(flange_xacro))
    facts = load_nero_gripper_flange_facts(flange_xacro)
    link = extension.getroot().find("link[@name='gripper_flange']")
    joint = extension.getroot().find("joint[@name='gripper_flange_joint']")
    if link is None or joint is None:
        raise ValueError("pinned flange xacro is incomplete")
    link_copy = deepcopy(link)
    for mesh in link_copy.findall("./*/geometry/mesh"):
        uri = _string(mesh.attrib.get("filename"), field="gripper_flange mesh URI")
        if not uri.startswith(OFFICIAL_MESH_PREFIX):
            raise ValueError(f"unexpected gripper-flange mesh URI: {uri!r}")
        mesh.attrib["filename"] = (
            f"package://{flange_package_name}/{uri.removeprefix(OFFICIAL_MESH_PREFIX)}"
        )
    joint_copy = deepcopy(joint)
    base.getroot().append(link_copy)
    base.getroot().append(joint_copy)
    effective_origin = cast(ET.Element, joint_copy.find("origin"))
    if effective_origin is None:
        raise ValueError("fixed flange origin is incomplete")
    _post_rotate_orientation(effective_origin, flange_post_rotation_rpy_rad)
    facts = replace(
        facts,
        position_m=_vector(effective_origin.attrib.get("xyz"), field="effective joint.xyz"),
        quaternion_wxyz=_quaternion_from_rpy(
            _vector(effective_origin.attrib.get("rpy"), field="effective joint.rpy")
        ),
    )
    ET.indent(base, space="  ")
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    base.write(destination, encoding="utf-8", xml_declaration=True)
    return facts


def _quaternion(value: object) -> Quaternion:
    quaternion = cast(Any, value)
    imaginary = tuple(float(component) for component in quaternion.GetImaginary())
    return (
        float(quaternion.GetReal()),
        imaginary[0],
        imaginary[1],
        imaginary[2],
    )


def _quaternions_equivalent(left: Quaternion, right: Quaternion) -> bool:
    return math.isclose(
        abs(sum(a * b for a, b in zip(left, right, strict=True))), 1.0, abs_tol=1e-5
    )


def inspect_nero_gripper_flange_usd(
    path: str | Path,
    *,
    base_facts: NeroUrdfFacts,
    flange: NeroGripperFlangeFacts,
) -> dict[str, object]:
    """Reject q7 drift and qualify the added fixed flange representation."""

    from pxr import Usd, UsdGeom, UsdPhysics  # type: ignore[import-not-found]

    stage = Usd.Stage.Open(str(Path(path).resolve()))
    if stage is None or not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0):
        raise RuntimeError("NERO flange USD must be a meter-scale readable stage")
    rigid_bodies: dict[str, str] = {}
    revolute: dict[str, object] = {}
    fixed: dict[str, object] = {}
    for prim in stage.Traverse():
        name = prim.GetName()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_bodies[name] = str(prim.GetPath())
        if prim.IsA(UsdPhysics.RevoluteJoint):
            revolute[name] = UsdPhysics.RevoluteJoint(prim)
        if prim.IsA(UsdPhysics.FixedJoint):
            fixed[name] = UsdPhysics.FixedJoint(prim)
    collisions = [
        str(prim.GetPath())
        for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
        if prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    expected_bodies = {"base_link", *NERO_CHILD_LINKS, flange.link_name}
    if set(rigid_bodies) != expected_bodies or set(revolute) != set(NERO_JOINT_NAMES):
        raise RuntimeError("NERO flange USD changed the pinned q7 rigid-body topology")
    if set(fixed) != {"world_to_base_link", flange.joint_name}:
        raise RuntimeError("NERO flange USD fixed-joint inventory differs")
    for source_joint in base_facts.joints:
        joint = cast(Any, revolute[source_joint.name])
        lower = float(joint.GetLowerLimitAttr().Get())
        upper = float(joint.GetUpperLimitAttr().Get())
        if not math.isclose(
            lower, math.degrees(source_joint.lower_rad), abs_tol=1e-3
        ) or not math.isclose(upper, math.degrees(source_joint.upper_rad), abs_tol=1e-3):
            raise RuntimeError(f"{source_joint.name} limits drifted in the flange USD")
    flange_joint = cast(Any, fixed[flange.joint_name])
    if [str(value) for value in flange_joint.GetBody0Rel().GetTargets()] != [
        rigid_bodies[flange.parent_link]
    ] or [str(value) for value in flange_joint.GetBody1Rel().GetTargets()] != [
        rigid_bodies[flange.link_name]
    ]:
        raise RuntimeError("gripper_flange_joint body relationship differs from source")
    position = tuple(float(value) for value in flange_joint.GetLocalPos0Attr().Get())
    flange_rotation = _quaternion(flange_joint.GetLocalRot0Attr().Get())
    if any(
        not math.isclose(a, b, abs_tol=1e-6)
        for a, b in zip(position, flange.position_m, strict=True)
    ):
        raise RuntimeError("gripper_flange_joint translation differs from source")
    if not _quaternions_equivalent(flange_rotation, flange.quaternion_wxyz):
        raise RuntimeError("gripper_flange_joint rotation differs from source")
    flange_prim = stage.GetPrimAtPath(rigid_bodies[flange.link_name])
    mass = float(UsdPhysics.MassAPI(flange_prim).GetMassAttr().Get())
    if not math.isclose(mass, flange.mass_kg, abs_tol=1e-6):
        raise RuntimeError("gripper_flange mass differs from source")
    flange_collisions = [
        value for value in collisions if value.startswith(f"{rigid_bodies[flange.link_name]}/")
    ]
    if len(flange_collisions) != 1:
        raise RuntimeError("gripper_flange must expose one collision mesh")
    collision_prim = stage.GetPrimAtPath(flange_collisions[0])
    approximation = str(UsdPhysics.MeshCollisionAPI(collision_prim).GetApproximationAttr().Get())
    if approximation != "convexHull":
        raise RuntimeError("gripper_flange collision must use convexHull")
    return {
        "q7_joint_count": len(revolute),
        "rigid_body_paths": dict(sorted(rigid_bodies.items())),
        "fixed_joint_paths": {
            name: str(cast(Any, joint).GetPath()) for name, joint in sorted(fixed.items())
        },
        "gripper_flange": {
            "position_m": list(position),
            "quaternion_wxyz": list(flange_rotation),
            "mass_kg": mass,
            "collision_path": flange_collisions[0],
            "collision_approximation": approximation,
        },
        "pinned_nero_representation": "unchanged",
    }


__all__ = [
    "NeroGripperFlangeFacts",
    "NeroGripperFlangeImportProfile",
    "build_nero_gripper_flange_urdf",
    "inspect_nero_gripper_flange_usd",
    "load_nero_gripper_flange_facts",
    "load_nero_gripper_flange_import_profile",
]
