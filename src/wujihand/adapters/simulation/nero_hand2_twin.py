"""Author and validate one physical NERO + Hand 2 articulation in Isaac.

The upstream assets remain read-only.  The live-stage overlay disables the
Hand 2 world root joint and connects its rigid base to NERO ``link7`` with one
fixed joint.  PhysX then exposes one q27 articulation per side:

``NERO q7 + side-specific Hand 2 q20``.

``pxr`` is imported lazily so configuration and DOF-discovery helpers remain
usable in fast CPU tests.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import PurePosixPath
from typing import Any, Sequence

import numpy as np


def _absolute_prim_path(value: str, *, field: str) -> str:
    if not isinstance(value, str) or not value.startswith("/") or value == "/" or "//" in value:
        raise ValueError(f"{field} must be an absolute USD prim path")
    return value.rstrip("/")


def _finite_vector(
    values: Sequence[float],
    *,
    size: int,
    field: str,
) -> tuple[float, ...]:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (size,):
        raise ValueError(f"{field} must have shape {(size,)}, got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{field} contains NaN or infinity")
    return tuple(float(value) for value in array)


@dataclass(frozen=True, slots=True)
class NeroHand2AttachmentConfig:
    """Resolved paths and the backend-neutral flange attachment transform."""

    side: str
    nero_prim_path: str
    hand_prim_path: str
    nero_articulation_root_path: str
    parent_link_path: str
    child_base_link_path: str
    hand_root_joint_path: str
    attachment_joint_path: str
    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    enable_self_collisions: bool = False

    def __post_init__(self) -> None:
        if self.side not in {"left", "right"}:
            raise ValueError("side must be 'left' or 'right'")
        for field in (
            "nero_prim_path",
            "hand_prim_path",
            "nero_articulation_root_path",
            "parent_link_path",
            "child_base_link_path",
            "hand_root_joint_path",
            "attachment_joint_path",
        ):
            object.__setattr__(
                self,
                field,
                _absolute_prim_path(getattr(self, field), field=field),
            )
        if self.hand_prim_path == self.nero_prim_path:
            raise ValueError("hand_prim_path and nero_prim_path must differ")
        if not self.parent_link_path.startswith(self.nero_prim_path + "/"):
            raise ValueError("parent_link_path must belong to the NERO instance")
        if not self.nero_articulation_root_path.startswith(self.nero_prim_path + "/"):
            raise ValueError("nero_articulation_root_path must belong to the NERO instance")
        if not self.parent_link_path.startswith(self.nero_articulation_root_path + "/"):
            raise ValueError("parent_link_path must belong to the configured NERO articulation")
        for field in ("child_base_link_path", "hand_root_joint_path"):
            if not getattr(self, field).startswith(self.hand_prim_path + "/"):
                raise ValueError(f"{field} must belong to the Hand 2 instance")
        position = _finite_vector(self.position_m, size=3, field="position_m")
        quaternion = _finite_vector(self.quat_wxyz, size=4, field="quat_wxyz")
        if not math.isclose(
            float(np.linalg.norm(quaternion)),
            1.0,
            rel_tol=0.0,
            abs_tol=1e-6,
        ):
            raise ValueError("quat_wxyz must be a unit quaternion")
        if type(self.enable_self_collisions) is not bool:
            raise ValueError("enable_self_collisions must be a bool")
        object.__setattr__(self, "position_m", position)
        object.__setattr__(self, "quat_wxyz", quaternion)


@dataclass(frozen=True, slots=True)
class NeroHand2AttachmentHandles:
    """Stable paths authored for one side without returning PXR objects."""

    config: NeroHand2AttachmentConfig
    disabled_hand_root_joint_path: str
    attachment_joint_path: str
    articulation_root_path: str


@dataclass(frozen=True, slots=True)
class NeroHand2DofPartition:
    """Runtime q27 indices in the two canonical control-group orders."""

    arm_indices_q7: tuple[int, ...]
    hand_indices_q20: tuple[int, ...]

    @property
    def all_indices(self) -> tuple[int, ...]:
        return self.arm_indices_q7 + self.hand_indices_q20


def _set_world_matrix(xformable: Any, matrix: Any) -> None:
    from pxr import UsdGeom  # type: ignore[import-not-found]

    prim = xformable.GetPrim()
    parent_to_world = UsdGeom.XformCache().GetParentToWorldTransform(prim)
    local_matrix = matrix * parent_to_world.GetInverse()
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(local_matrix)


def _attachment_matrix(config: NeroHand2AttachmentConfig) -> Any:
    from pxr import Gf

    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(Gf.Quatd(*config.quat_wxyz))
    matrix.SetTranslateOnly(Gf.Vec3d(*config.position_m))
    return matrix


def _merge_hand_articulation_attributes(
    hand_root_prim: Any,
    nero_root_prim: Any,
) -> None:
    """Retain non-collision Hand 2 solver settings without hiding conflicts."""

    self_collision_name = "physxArticulation:enabledSelfCollisions"
    for source in hand_root_prim.GetAttributes():
        name = source.GetName()
        if (
            not name.startswith("physxArticulation:")
            or name == self_collision_name
            or not source.HasAuthoredValue()
        ):
            continue
        target = nero_root_prim.GetAttribute(name)
        if target.IsValid() and target.HasAuthoredValue():
            if target.Get() != source.Get():
                raise RuntimeError(f"NERO and Hand 2 articulation settings conflict at {name}")
            continue
        if not target.IsValid():
            target = nero_root_prim.CreateAttribute(
                name,
                source.GetTypeName(),
                custom=source.IsCustom(),
            )
        target.Set(source.Get())


def author_nero_hand2_attachment(
    stage: Any,
    config: NeroHand2AttachmentConfig,
) -> NeroHand2AttachmentHandles:
    """Merge one referenced Hand 2 into one referenced NERO articulation.

    The function must run before physics initialization.  It rejects unexpected
    upstream topology and intentionally refuses ``enable_self_collisions=True``
    until an explicit NERO collision-filter qualification is implemented.
    """

    if config.enable_self_collisions:
        raise RuntimeError(
            "Hand 2 internal self-collision requires explicit NERO collision "
            "filtering and is not qualified by the NV-2 default"
        )

    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    if not math.isclose(meters_per_unit, 1.0, abs_tol=1e-9):
        raise RuntimeError(
            f"NERO + Hand 2 attachment requires meter stage units, got {meters_per_unit}"
        )
    if stage.GetPrimAtPath(config.attachment_joint_path).IsValid():
        raise RuntimeError(f"attachment joint already exists: {config.attachment_joint_path}")

    nero_prim = stage.GetPrimAtPath(config.nero_prim_path)
    hand_prim = stage.GetPrimAtPath(config.hand_prim_path)
    articulation_root = stage.GetPrimAtPath(config.nero_articulation_root_path)
    parent_link = stage.GetPrimAtPath(config.parent_link_path)
    child_base = stage.GetPrimAtPath(config.child_base_link_path)
    hand_root_prim = stage.GetPrimAtPath(config.hand_root_joint_path)
    if not nero_prim.IsValid() or not nero_prim.IsA(UsdGeom.Xformable):
        raise RuntimeError(f"NERO reference root is missing: {config.nero_prim_path}")
    if not hand_prim.IsValid() or not hand_prim.IsA(UsdGeom.Xformable):
        raise RuntimeError(f"Hand 2 reference root is missing: {config.hand_prim_path}")
    if not articulation_root.IsValid() or not articulation_root.HasAPI(
        UsdPhysics.ArticulationRootAPI
    ):
        raise RuntimeError(
            f"NERO articulation root is missing: {config.nero_articulation_root_path}"
        )
    if not parent_link.IsValid() or not parent_link.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f"NERO flange rigid body is missing: {config.parent_link_path}")
    if not child_base.IsValid() or not child_base.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f"Hand 2 base rigid body is missing: {config.child_base_link_path}")
    if not hand_root_prim.IsValid() or not hand_root_prim.IsA(UsdPhysics.FixedJoint):
        raise RuntimeError(f"Hand 2 upstream root joint is missing: {config.hand_root_joint_path}")

    hand_root = UsdPhysics.FixedJoint(hand_root_prim)
    if hand_root.GetJointEnabledAttr().Get() is False:
        raise RuntimeError("Hand 2 upstream root joint is already disabled")
    if list(hand_root.GetBody0Rel().GetTargets()):
        raise RuntimeError("Hand 2 upstream root joint must be fixed to world")
    if list(hand_root.GetBody1Rel().GetTargets()) != [Sdf.Path(config.child_base_link_path)]:
        raise RuntimeError("Hand 2 upstream root joint no longer targets the configured base link")
    roots_before = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        and (
            str(prim.GetPath()).startswith(config.nero_prim_path + "/")
            or str(prim.GetPath()).startswith(config.hand_prim_path + "/")
        )
    ]
    if sorted(roots_before) != sorted(
        [config.nero_articulation_root_path, config.hand_root_joint_path]
    ):
        raise RuntimeError(
            f"expected one NERO and one Hand 2 articulation root, found {roots_before}"
        )

    # Hand ``base_link`` is identity-relative to the referenced root in the
    # pinned asset.  Place it at the desired flange transform before physics
    # starts to avoid a constraint-snap impulse.
    hand_base_xformable = UsdGeom.Xformable(child_base)
    if hand_base_xformable.GetResetXformStack():
        raise RuntimeError("Hand 2 base link must not reset the transform stack")
    hand_base_local = hand_base_xformable.GetLocalTransformation()
    if not Gf.IsClose(hand_base_local, Gf.Matrix4d(1.0), 1e-12):
        raise RuntimeError("Hand 2 base link is no longer identity-relative to its reference root")
    parent_world = UsdGeom.XformCache().GetLocalToWorldTransform(parent_link)
    desired_hand_world = _attachment_matrix(config) * parent_world
    _set_world_matrix(UsdGeom.Xformable(hand_prim), desired_hand_world)

    joint = UsdPhysics.FixedJoint.Define(stage, config.attachment_joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(config.parent_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(config.child_base_link_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*config.position_m))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(*config.quat_wxyz))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

    hand_root.CreateJointEnabledAttr(False)
    hand_root_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    if hand_root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise RuntimeError("failed to remove ArticulationRootAPI from the Hand 2 world root")
    physx_root = PhysxSchema.PhysxArticulationAPI.Apply(articulation_root)
    _merge_hand_articulation_attributes(hand_root_prim, articulation_root)
    physx_root.CreateEnabledSelfCollisionsAttr(False)
    # A disabled referenced joint can still be parsed by PhysX and emit a
    # misleading disjoint-body "snap" warning.  Deactivation is a stage-local
    # overlay opinion: it leaves the pinned source USD untouched and ensures
    # the obsolete world constraint is not instantiated at all.
    hand_root_prim.SetActive(False)

    roots_after = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        and (
            str(prim.GetPath()).startswith(config.nero_prim_path + "/")
            or str(prim.GetPath()).startswith(config.hand_prim_path + "/")
        )
    ]
    if sorted(roots_after) != [config.nero_articulation_root_path]:
        raise RuntimeError(f"expected one merged articulation root, found {roots_after}")
    if hand_root_prim.IsActive() or hand_root.GetJointEnabledAttr().Get() is not False:
        raise RuntimeError("failed to disable the Hand 2 world root joint")
    if physx_root.GetEnabledSelfCollisionsAttr().Get() is not False:
        raise RuntimeError("merged articulation self-collision must be disabled")

    return NeroHand2AttachmentHandles(
        config=config,
        disabled_hand_root_joint_path=config.hand_root_joint_path,
        attachment_joint_path=config.attachment_joint_path,
        articulation_root_path=config.nero_articulation_root_path,
    )


def discover_nero_hand2_dofs(
    dof_names: Sequence[str],
    dof_paths: Sequence[str],
    arm_joint_names_q7: Sequence[str],
    hand_joint_names_q20: Sequence[str],
    *,
    nero_prim_path: str,
    hand_prim_path: str,
) -> NeroHand2DofPartition:
    """Validate and partition one q27 articulation by name and USD joint path."""

    names = tuple(str(name) for name in dof_names)
    paths = tuple(str(path) for path in dof_paths)
    arm = tuple(str(name) for name in arm_joint_names_q7)
    hand = tuple(str(name) for name in hand_joint_names_q20)
    try:
        nero_prefix = _absolute_prim_path(nero_prim_path, field="nero_prim_path")
        hand_prefix = _absolute_prim_path(hand_prim_path, field="hand_prim_path")
    except ValueError as exc:
        raise RuntimeError("runtime DOF ownership prefixes are invalid") from exc
    if nero_prefix == hand_prefix:
        raise RuntimeError("runtime NERO and Hand 2 prefixes must differ")
    if len(names) != len(paths):
        raise RuntimeError("dof_names and dof_paths must have equal length")
    if len(arm) != 7 or len(set(arm)) != 7:
        raise RuntimeError("arm_joint_names_q7 must contain seven unique joints")
    if len(hand) != 20 or len(set(hand)) != 20:
        raise RuntimeError("hand_joint_names_q20 must contain twenty unique joints")
    if set(arm) & set(hand):
        raise RuntimeError("arm and hand joint names must be disjoint")
    if len(names) != 27 or len(set(names)) != 27:
        raise RuntimeError(f"expected 27 unique runtime DOFs, got {len(names)}")

    index_by_name: dict[str, int] = {}
    expected = set(arm) | set(hand)
    unexpected: list[tuple[str, str]] = []
    for index, (name, path) in enumerate(zip(names, paths, strict=True)):
        expected_prefix = nero_prefix if name in arm else hand_prefix if name in hand else None
        if (
            expected_prefix is None
            or not PurePosixPath(path).is_absolute()
            or PurePosixPath(path).name != name
            or not path.startswith(expected_prefix + "/")
        ):
            unexpected.append((name, path))
            continue
        index_by_name[name] = index
    missing = sorted(expected - set(index_by_name))
    if unexpected or missing:
        raise RuntimeError(
            f"composite q27 layout mismatch: unexpected={unexpected}, missing={missing}"
        )
    return NeroHand2DofPartition(
        arm_indices_q7=tuple(index_by_name[name] for name in arm),
        hand_indices_q20=tuple(index_by_name[name] for name in hand),
    )


__all__ = [
    "NeroHand2AttachmentConfig",
    "NeroHand2AttachmentHandles",
    "NeroHand2DofPartition",
    "author_nero_hand2_attachment",
    "discover_nero_hand2_dofs",
]
