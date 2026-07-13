"""Author and validate a rotation-only Hand 2 mount in Isaac Sim 5.1.

The upstream Hand 2 USD stays read-only.  This adapter authors an overlay on the
live stage with the following topology::

    world --FixedJoint--> collisionless anchor --generic D6--> r_base_link

The D6 joint locks all translations, limits roll/pitch, leaves yaw periodic, and
uses finite force drives for all three angular axes.  ``pxr`` is imported lazily
so the configuration and DOF-discovery helpers remain usable in fast CPU tests.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import PurePosixPath
from typing import Any, Sequence

import numpy as np


_ROTATION_AXES = ("rotX", "rotY", "rotZ")
_TRANSLATION_AXES = ("transX", "transY", "transZ")


def _validate_absolute_prim_path(value: str, field_name: str) -> None:
    if not value.startswith("/") or value == "/" or "//" in value:
        raise ValueError(f"{field_name} must be an absolute USD prim path")


def _finite_vector(values: Sequence[float], size: int, field_name: str) -> tuple[float, ...]:
    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (size,):
        raise ValueError(f"{field_name} must have shape {(size,)}, got {vector.shape}")
    if not np.isfinite(vector).all():
        raise ValueError(f"{field_name} contains NaN or infinity")
    return tuple(float(value) for value in vector)


def principal_axes_joint_frame_quaternion(
    principal_axes_wxyz: Sequence[float],
) -> tuple[float, float, float, float]:
    """Split a child mass-principal rotation equally across both D6 frames."""

    principal = np.asarray(
        _finite_vector(principal_axes_wxyz, 4, "principal_axes_wxyz"),
        dtype=np.float64,
    )
    if not math.isclose(float(np.linalg.norm(principal)), 1.0, abs_tol=1e-5):
        raise ValueError("principal_axes_wxyz must be a unit quaternion")
    if principal[0] < 0.0:
        principal *= -1.0
    half_rotation = principal + np.asarray((1.0, 0.0, 0.0, 0.0))
    norm = float(np.linalg.norm(half_rotation))
    if norm <= 1e-12:
        raise ValueError("principal_axes_wxyz has an ambiguous half rotation")
    half_rotation /= norm
    return (
        float(half_rotation[0]),
        float(half_rotation[1]),
        float(half_rotation[2]),
        float(half_rotation[3]),
    )


@dataclass(frozen=True, slots=True)
class Hand2RotationMountConfig:
    """Geometry and finite-drive parameters for the right Hand 2 mount.

    Angular limits are expressed in radians at the project boundary.  USD
    angular ``LimitAPI`` and ``DriveAPI.targetPosition`` values are authored in
    degrees, as required by USD Physics.
    """

    hand_prim_path: str = "/World/Hand2"
    mount_prim_path: str = "/World/Hand2Mount"
    flange_position_m: tuple[float, float, float] = (0.0, 0.0, 0.465)
    flange_orientation_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    roll_limit_rad: float = math.radians(89.0)
    pitch_limit_rad: float = math.radians(89.0)
    drive_stiffness: float = 25.0
    drive_damping: float = 2.5
    drive_max_force: float = 8.0
    anchor_mass_kg: float = 1.0

    def __post_init__(self) -> None:
        _validate_absolute_prim_path(self.hand_prim_path, "hand_prim_path")
        _validate_absolute_prim_path(self.mount_prim_path, "mount_prim_path")
        if self.mount_prim_path == self.hand_prim_path:
            raise ValueError("mount_prim_path must differ from hand_prim_path")
        _finite_vector(self.flange_position_m, 3, "flange_position_m")
        quaternion = np.asarray(
            _finite_vector(self.flange_orientation_wxyz, 4, "flange_orientation_wxyz")
        )
        if not math.isclose(float(np.linalg.norm(quaternion)), 1.0, abs_tol=1e-6):
            raise ValueError("flange_orientation_wxyz must be a unit quaternion")
        for name, limit in (
            ("roll_limit_rad", self.roll_limit_rad),
            ("pitch_limit_rad", self.pitch_limit_rad),
        ):
            if not math.isfinite(limit) or not 0.0 < limit < math.pi / 2.0:
                raise ValueError(f"{name} must be finite and strictly between 0 and pi/2")
        for name, value, allow_zero in (
            ("drive_stiffness", self.drive_stiffness, False),
            ("drive_damping", self.drive_damping, True),
            ("drive_max_force", self.drive_max_force, False),
            ("anchor_mass_kg", self.anchor_mass_kg, False),
        ):
            valid = value >= 0.0 if allow_zero else value > 0.0
            if not math.isfinite(value) or not valid:
                comparator = "non-negative" if allow_zero else "positive"
                raise ValueError(f"{name} must be finite and {comparator}")


@dataclass(frozen=True, slots=True)
class RotationMountPaths:
    """Authored USD paths needed by the Isaac runner."""

    hand_prim_path: str
    base_link_path: str
    disabled_root_joint_path: str
    mount_prim_path: str
    anchor_body_path: str
    anchor_joint_path: str
    rotation_joint_path: str

    @property
    def articulation_root_path(self) -> str:
        """The single prim carrying ``ArticulationRootAPI`` after authoring."""

        return self.anchor_joint_path


@dataclass(frozen=True, slots=True)
class RotationMountHandles:
    """Stable runner-facing result without leaking PXR objects across layers."""

    paths: RotationMountPaths
    config: Hand2RotationMountConfig
    joint_frame_quat_wxyz: tuple[float, float, float, float]

    @property
    def drive_axes(self) -> tuple[str, str, str]:
        return _ROTATION_AXES


@dataclass(frozen=True, slots=True)
class RotationMountDofPartition:
    """Runtime indices split into D6 rotXYZ and canonical Hand 2 q20 order."""

    wrist_indices_xyz: tuple[int, int, int]
    finger_indices_q20: tuple[int, ...]
    wrist_names_xyz: tuple[str, str, str]

    @property
    def all_indices(self) -> tuple[int, ...]:
        return self.wrist_indices_xyz + self.finger_indices_q20


def _mount_paths(config: Hand2RotationMountConfig) -> RotationMountPaths:
    hand = config.hand_prim_path.rstrip("/")
    mount = config.mount_prim_path.rstrip("/")
    return RotationMountPaths(
        hand_prim_path=hand,
        base_link_path=f"{hand}/r_base_link",
        disabled_root_joint_path=f"{hand}/root_joint",
        mount_prim_path=mount,
        anchor_body_path=f"{mount}/flange_anchor",
        anchor_joint_path=f"{mount}/world_fixed_joint",
        rotation_joint_path=f"{mount}/wrist_rotation_joint",
    )


def _set_world_transform(
    xformable: Any, position: Sequence[float], quaternion: Sequence[float]
) -> None:
    from pxr import Gf, UsdGeom  # type: ignore[import-not-found]

    xformable.ClearXformOpOrder()
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(Gf.Quatd(*quaternion))
    matrix.SetTranslateOnly(Gf.Vec3d(*position))
    xformable.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(matrix)


def _copy_articulation_attributes(source_prim: Any, target_prim: Any) -> None:
    """Preserve authored solver/self-collision settings from the upstream root."""

    for attribute in source_prim.GetAttributes():
        if (
            not attribute.GetName().startswith("physxArticulation:")
            or not attribute.HasAuthoredValue()
        ):
            continue
        target = target_prim.CreateAttribute(
            attribute.GetName(), attribute.GetTypeName(), custom=attribute.IsCustom()
        )
        target.Set(attribute.Get())


def author_rotation_mount(
    stage: Any,
    config: Hand2RotationMountConfig | None = None,
    *,
    hand_prim_path: str | None = None,
) -> RotationMountHandles:
    """Author the rotation-only overlay and return paths used by the runner.

    The function is intentionally fail-closed: it accepts only the pinned Hand 2
    topology (one fixed world root targeting ``r_base_link``), refuses an
    existing mount, requires meter stage units, and verifies that exactly one
    relevant articulation root remains.

    Args:
        stage: A live ``pxr.Usd.Stage`` after the Hand 2 reference is added.
        config: Validated mount parameters.  Defaults to the upright MVP values.
        hand_prim_path: Convenience override for runners that mount the asset at
            a non-default path.  It may not conflict with ``config``.
    """

    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    if config is None:
        config = Hand2RotationMountConfig(hand_prim_path=hand_prim_path or "/World/Hand2")
    elif hand_prim_path is not None and hand_prim_path != config.hand_prim_path:
        raise ValueError("hand_prim_path conflicts with config.hand_prim_path")
    paths = _mount_paths(config)

    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    if not math.isclose(meters_per_unit, 1.0, abs_tol=1e-9):
        raise RuntimeError(f"Hand 2 mount requires meter stage units, got {meters_per_unit}")
    if stage.GetPrimAtPath(paths.mount_prim_path).IsValid():
        raise RuntimeError(f"rotation mount already exists: {paths.mount_prim_path}")

    hand_prim = stage.GetPrimAtPath(paths.hand_prim_path)
    base_prim = stage.GetPrimAtPath(paths.base_link_path)
    old_root_prim = stage.GetPrimAtPath(paths.disabled_root_joint_path)
    if not hand_prim.IsValid() or not hand_prim.IsA(UsdGeom.Xformable):
        raise RuntimeError(f"Hand 2 Xform is missing: {paths.hand_prim_path}")
    if not base_prim.IsValid() or not base_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f"Hand 2 rigid base link is missing: {paths.base_link_path}")
    if not old_root_prim.IsValid() or not old_root_prim.IsA(UsdPhysics.FixedJoint):
        raise RuntimeError(f"expected upstream fixed root joint: {paths.disabled_root_joint_path}")
    principal_axes = UsdPhysics.MassAPI(base_prim).GetPrincipalAxesAttr().Get()
    if principal_axes is None:
        principal_axes_quaternion = (1.0, 0.0, 0.0, 0.0)
    else:
        imaginary = principal_axes.GetImaginary()
        principal_axes_quaternion = (
            float(principal_axes.GetReal()),
            float(imaginary[0]),
            float(imaginary[1]),
            float(imaginary[2]),
        )
    # PhysX expresses a generic articulation joint's angular coordinates in
    # the two bodies' mass-principal frames.  With identity local frames, a
    # drive axis is therefore conjugated by the child principal-axis rotation;
    # copying the full rotation swaps the bias to the opposite side.  Applying
    # the quaternion square root to both local joint frames splits that basis
    # change equally and makes rotX/Y/Z coincide with the visible base-link
    # frame while preserving the neutral pose.
    try:
        joint_frame_quaternion = principal_axes_joint_frame_quaternion(
            principal_axes_quaternion
        )
    except ValueError as exc:
        raise RuntimeError("r_base_link principal axes are invalid") from exc
    old_root = UsdPhysics.FixedJoint(old_root_prim)
    if list(old_root.GetBody0Rel().GetTargets()) or list(old_root.GetBody1Rel().GetTargets()) != [
        Sdf.Path(paths.base_link_path)
    ]:
        raise RuntimeError("upstream Hand 2 root topology no longer matches world -> r_base_link")

    relevant_prefixes = (paths.hand_prim_path + "/", paths.mount_prim_path + "/")
    roots_before = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        and str(prim.GetPath()).startswith(relevant_prefixes)
    ]
    if roots_before != [paths.disabled_root_joint_path]:
        raise RuntimeError(f"expected one upstream Hand 2 articulation root, found {roots_before}")

    # Move the complete referenced hand before physics initialization.  This is
    # stage authoring, not a per-frame teleport, and keeps all imported links in
    # a coherent initial pose.
    _set_world_transform(
        UsdGeom.Xformable(hand_prim),
        config.flange_position_m,
        config.flange_orientation_wxyz,
    )

    mount_xform = UsdGeom.Xform.Define(stage, paths.mount_prim_path)
    _set_world_transform(mount_xform, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    anchor_xform = UsdGeom.Xform.Define(stage, paths.anchor_body_path)
    _set_world_transform(
        anchor_xform,
        config.flange_position_m,
        config.flange_orientation_wxyz,
    )
    anchor_prim = anchor_xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(anchor_prim)
    UsdPhysics.MassAPI.Apply(anchor_prim).CreateMassAttr(config.anchor_mass_kg)

    root_joint = UsdPhysics.FixedJoint.Define(stage, paths.anchor_joint_path)
    root_joint.CreateBody1Rel().SetTargets([Sdf.Path(paths.anchor_body_path)])
    root_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*config.flange_position_m))
    root_joint.CreateLocalRot0Attr().Set(Gf.Quatf(*config.flange_orientation_wxyz))
    root_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
    root_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    root_prim = root_joint.GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(root_prim)
    PhysxSchema.PhysxArticulationAPI.Apply(root_prim)
    _copy_articulation_attributes(old_root_prim, root_prim)

    rotation_joint = UsdPhysics.Joint.Define(stage, paths.rotation_joint_path)
    rotation_joint.CreateBody0Rel().SetTargets([Sdf.Path(paths.anchor_body_path)])
    rotation_joint.CreateBody1Rel().SetTargets([Sdf.Path(paths.base_link_path)])
    rotation_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
    rotation_joint.CreateLocalRot0Attr().Set(Gf.Quatf(*joint_frame_quaternion))
    rotation_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
    rotation_joint.CreateLocalRot1Attr().Set(Gf.Quatf(*joint_frame_quaternion))
    rotation_prim = rotation_joint.GetPrim()

    # A LimitAPI with low > high locks a generic D6 axis in USD Physics.
    for axis in _TRANSLATION_AXES:
        limit = UsdPhysics.LimitAPI.Apply(rotation_prim, axis)
        limit.CreateLowAttr(1.0)
        limit.CreateHighAttr(-1.0)
    for axis, limit_rad in (
        ("rotX", config.roll_limit_rad),
        ("rotY", config.pitch_limit_rad),
    ):
        limit = UsdPhysics.LimitAPI.Apply(rotation_prim, axis)
        limit.CreateLowAttr(-math.degrees(limit_rad))
        limit.CreateHighAttr(math.degrees(limit_rad))
    # rotZ intentionally has no LimitAPI: yaw is periodic/unbounded.
    for axis in _ROTATION_AXES:
        drive = UsdPhysics.DriveAPI.Apply(rotation_prim, axis)
        drive.CreateTypeAttr("force")
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateStiffnessAttr(config.drive_stiffness)
        drive.CreateDampingAttr(config.drive_damping)
        drive.CreateMaxForceAttr(config.drive_max_force)

    old_root.CreateJointEnabledAttr(False)
    old_root_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    if old_root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise RuntimeError("failed to remove ArticulationRootAPI from the upstream fixed root")

    roots_after = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        and str(prim.GetPath()).startswith(relevant_prefixes)
    ]
    if roots_after != [paths.anchor_joint_path]:
        raise RuntimeError(f"expected one rotation-mount articulation root, found {roots_after}")
    if old_root.GetJointEnabledAttr().Get() is not False:
        raise RuntimeError("failed to disable the upstream world fixed joint")

    return RotationMountHandles(
        paths=paths,
        config=config,
        joint_frame_quat_wxyz=joint_frame_quaternion,
    )


def _d6_axis_from_name(name: str) -> str | None:
    # Isaac Sim 5.1 exposes generic-joint rotational DOFs either with semantic
    # suffixes (``:rotX``) or with PhysX's stable free-axis ordinal
    # (``:0/:1/:2`` == rotX/rotY/rotZ after transXYZ are locked).  The joint
    # path is still checked independently by ``discover_rotation_mount_dofs``.
    for suffix, axis in ((":0", "rotX"), (":1", "rotY"), (":2", "rotZ")):
        if name.endswith(suffix):
            return axis
    compact = "".join(character for character in name.lower() if character.isalnum())
    for axis in _ROTATION_AXES:
        if compact.endswith(axis.lower()):
            return axis
    return None


def discover_rotation_mount_dofs(
    dof_names: Sequence[str],
    dof_paths: Sequence[str],
    finger_joint_names_q20: Sequence[str],
    rotation_joint_path: str,
) -> RotationMountDofPartition:
    """Discover the 3+20 runtime split by both joint name and USD path.

    No count-, prefix-, or order-only fallback is allowed.  Any unexpected,
    duplicate, missing, or unidentifiable DOF raises ``RuntimeError`` so a
    runner cannot silently send a wrist target to a finger joint.
    """

    names = tuple(str(name) for name in dof_names)
    paths = tuple(str(path) for path in dof_paths)
    fingers = tuple(str(name) for name in finger_joint_names_q20)
    _validate_absolute_prim_path(rotation_joint_path, "rotation_joint_path")
    if len(names) != len(paths):
        raise RuntimeError("dof_names and dof_paths must have equal length")
    if len(names) != len(fingers) + 3:
        raise RuntimeError(f"expected {len(fingers) + 3} runtime DOFs, got {len(names)}")
    if len(fingers) != 20 or len(set(fingers)) != 20:
        raise RuntimeError("finger_joint_names_q20 must be the unique Hand 2 q20 layout")

    wrist_by_axis: dict[str, int] = {}
    finger_by_name: dict[str, int] = {}
    unexpected: list[tuple[str, str]] = []
    finger_set = set(fingers)
    for index, (name, path) in enumerate(zip(names, paths, strict=True)):
        if path == rotation_joint_path:
            axis = _d6_axis_from_name(name)
            if axis is None or axis in wrist_by_axis:
                raise RuntimeError(
                    f"D6 wrist DOF name must uniquely end in rotX/rotY/rotZ, got {name!r}"
                )
            wrist_by_axis[axis] = index
        elif name in finger_set and PurePosixPath(path).name == name:
            if name in finger_by_name:
                raise RuntimeError(f"duplicate Hand 2 finger DOF: {name}")
            finger_by_name[name] = index
        else:
            unexpected.append((name, path))
    if unexpected:
        raise RuntimeError(f"unexpected articulation DOFs: {unexpected}")
    missing_axes = [axis for axis in _ROTATION_AXES if axis not in wrist_by_axis]
    missing_fingers = [name for name in fingers if name not in finger_by_name]
    if missing_axes or missing_fingers:
        raise RuntimeError(
            f"incomplete rotation mount layout: missing_axes={missing_axes}, "
            f"missing_fingers={missing_fingers}"
        )

    wrist_indices = tuple(wrist_by_axis[axis] for axis in _ROTATION_AXES)
    wrist_names = tuple(names[index] for index in wrist_indices)
    return RotationMountDofPartition(
        wrist_indices_xyz=(wrist_indices[0], wrist_indices[1], wrist_indices[2]),
        finger_indices_q20=tuple(finger_by_name[name] for name in fingers),
        wrist_names_xyz=(wrist_names[0], wrist_names[1], wrist_names[2]),
    )


def quaternion_wxyz_to_d6_rpy_degrees(
    quaternion_wxyz: Sequence[float],
) -> tuple[float, float, float]:
    """Map an anchor-local quaternion to D6 rotX/rotY/rotZ targets.

    The decomposition is intrinsic ZYX: ``R = Rz(yaw) Ry(pitch) Rx(roll)``.
    Consequently the returned tuple maps directly to ``(rotX, rotY, rotZ)``.
    The caller must first express the desired relative rotation in the fixed
    anchor frame.  Exact +/-90 degree pitch is rejected by mount limits.
    """

    w, x, y, z = _finite_vector(quaternion_wxyz, 4, "quaternion_wxyz")
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isclose(norm, 1.0, abs_tol=1e-6):
        raise ValueError("quaternion_wxyz must be a unit quaternion")
    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)
    sin_pitch = float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    pitch = math.asin(sin_pitch)
    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def unwrap_periodic_degrees(angle_deg: float, reference_deg: float) -> float:
    """Return the 360-degree equivalent of ``angle_deg`` nearest a reference."""

    if not math.isfinite(angle_deg) or not math.isfinite(reference_deg):
        raise ValueError("angle_deg and reference_deg must be finite")
    turns = round((reference_deg - angle_deg) / 360.0)
    return angle_deg + 360.0 * turns


def set_rotation_mount_targets_rpy(
    stage: Any,
    handles: RotationMountHandles,
    roll_pitch_yaw_rad: Sequence[float],
) -> tuple[float, float, float]:
    """Set finite D6 angular-drive targets and return authored degrees."""

    from pxr import UsdPhysics

    roll, pitch, yaw = _finite_vector(roll_pitch_yaw_rad, 3, "roll_pitch_yaw_rad")
    config = handles.config
    if abs(roll) > config.roll_limit_rad + 1e-12:
        raise ValueError("roll target exceeds configured D6 limit")
    if abs(pitch) > config.pitch_limit_rad + 1e-12:
        raise ValueError("pitch target exceeds configured D6 limit")
    targets_deg = (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
    joint_prim = stage.GetPrimAtPath(handles.paths.rotation_joint_path)
    if not joint_prim.IsValid() or not joint_prim.IsA(UsdPhysics.Joint):
        raise RuntimeError(f"rotation D6 joint is missing: {handles.paths.rotation_joint_path}")
    for axis, target_deg in zip(_ROTATION_AXES, targets_deg, strict=True):
        drive = UsdPhysics.DriveAPI.Get(joint_prim, axis)
        if not drive:
            raise RuntimeError(f"rotation D6 drive is missing for {axis}")
        drive.GetTargetPositionAttr().Set(target_deg)
    return targets_deg


def set_rotation_mount_target_quaternion(
    stage: Any,
    handles: RotationMountHandles,
    anchor_local_quaternion_wxyz: Sequence[float],
    *,
    previous_yaw_target_deg: float | None = None,
) -> tuple[float, float, float]:
    """Set a quaternion target, preserving the nearest continuous yaw branch."""

    roll_deg, pitch_deg, yaw_deg = quaternion_wxyz_to_d6_rpy_degrees(
        anchor_local_quaternion_wxyz
    )
    if previous_yaw_target_deg is not None:
        yaw_deg = unwrap_periodic_degrees(yaw_deg, previous_yaw_target_deg)
    targets_deg = (roll_deg, pitch_deg, yaw_deg)
    targets_rad = tuple(math.radians(value) for value in targets_deg)
    return set_rotation_mount_targets_rpy(stage, handles, targets_rad)


__all__ = [
    "Hand2RotationMountConfig",
    "RotationMountDofPartition",
    "RotationMountHandles",
    "RotationMountPaths",
    "author_rotation_mount",
    "discover_rotation_mount_dofs",
    "principal_axes_joint_frame_quaternion",
    "quaternion_wxyz_to_d6_rpy_degrees",
    "set_rotation_mount_target_quaternion",
    "set_rotation_mount_targets_rpy",
    "unwrap_periodic_degrees",
]
