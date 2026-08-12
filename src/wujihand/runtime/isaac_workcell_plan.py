"""Resolve Workcell v1 into source-neutral Isaac stage operations."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
from pathlib import Path
from typing import cast

from wujihand.specs import (
    ISAAC_STATIC_USD_WORKCELL_SCHEMA,
    EntitySpec,
    IsaacSceneExpectations,
    IsaacStaticUsdWorkcellProfile,
    IsaacTaskSceneProfile,
    IsaacWorkcellPolicies,
    PoseSpec,
    WorkcellSpec,
)

from .config_repository import ConfigRepository
from .source_lock import ResolvedContentRef, SourceLock
from .yaml_loader import load_yaml_strict


RESOLVED_ISAAC_WORKCELL_PLAN_SCHEMA = "wujihand.resolved_isaac_workcell_plan.v1"


@dataclass(frozen=True, slots=True)
class ResolvedIsaacUsdImport:
    import_id: str
    content: ResolvedContentRef
    composition: str
    pose: PoseSpec
    excluded_prim_paths: tuple[str, ...]
    fixed_rigid_body_paths: tuple[str, ...]
    expectations: IsaacSceneExpectations

    @property
    def prim_path(self) -> str:
        return f"/World/Environment/{self.import_id}"

    def to_mapping(self) -> dict[str, object]:
        return {
            "import_id": self.import_id,
            "prim_path": self.prim_path,
            "content": self.content.identity_mapping(),
            "composition": self.composition,
            "pose": self.pose.to_mapping(),
            "excluded_prim_paths": list(self.excluded_prim_paths),
            "fixed_rigid_body_paths": list(
                self.fixed_rigid_body_paths
            ),
            "expectations": self.expectations.to_mapping(),
        }


@dataclass(frozen=True, slots=True)
class ResolvedIsaacPrimitive:
    entity_id: str
    pose: PoseSpec
    entity: EntitySpec

    @property
    def prim_path(self) -> str:
        return f"/World/Workcell/{self.entity_id}"

    def to_mapping(self) -> dict[str, object]:
        return {
            "entity_id": self.entity_id,
            "prim_path": self.prim_path,
            "pose": self.pose.to_mapping(),
            "primitive": self.entity.primitive.to_mapping(),
            "mobility": self.entity.mobility,
            "mass_kg": self.entity.mass_kg,
        }


@dataclass(frozen=True, slots=True)
class ResolvedIsaacLighting:
    mode: str
    content: ResolvedContentRef | None
    intensity: float
    exposure: float
    visible_in_primary_ray: bool
    background_color_rgb: tuple[float, float, float]

    def to_mapping(self) -> dict[str, object]:
        return {
            "mode": self.mode,
            "content": (
                None
                if self.content is None
                else self.content.identity_mapping()
            ),
            "intensity": self.intensity,
            "exposure": self.exposure,
            "visible_in_primary_ray": self.visible_in_primary_ray,
            "background_color_rgb": list(self.background_color_rgb),
        }


@dataclass(frozen=True, slots=True)
class ResolvedIsaacWorkcellPlan:
    """Deterministic operations; absolute content locators stay out of mappings."""

    schema: str
    workcell_id: str
    profile_id: str | None
    profile_path: str | None
    task_scene_profile_id: str | None
    task_scene_profile_path: str | None
    imports: tuple[ResolvedIsaacUsdImport, ...]
    primitives: tuple[ResolvedIsaacPrimitive, ...]
    fixed_rigid_body_paths: tuple[str, ...]
    dynamic_rigid_body_paths: tuple[tuple[str, str], ...]
    policies: IsaacWorkcellPolicies
    lighting: ResolvedIsaacLighting
    expectations: IsaacSceneExpectations | None
    frame_ids: tuple[str, ...]
    mount_ids: tuple[str, ...]
    entity_ids: tuple[str, ...]

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "workcell_id": self.workcell_id,
            "profile_id": self.profile_id,
            "profile_path": self.profile_path,
            "task_scene_profile_id": self.task_scene_profile_id,
            "task_scene_profile_path": self.task_scene_profile_path,
            "imports": [operation.to_mapping() for operation in self.imports],
            "primitives": [
                operation.to_mapping() for operation in self.primitives
            ],
            "fixed_rigid_body_paths": list(
                self.fixed_rigid_body_paths
            ),
            "dynamic_rigid_body_paths": dict(
                self.dynamic_rigid_body_paths
            ),
            "policies": self.policies.to_mapping(),
            "lighting": self.lighting.to_mapping(),
            "expectations": (
                None
                if self.expectations is None
                else self.expectations.to_mapping()
            ),
            "inventory": {
                "frames": list(self.frame_ids),
                "mounts": list(self.mount_ids),
                "entities": list(self.entity_ids),
            },
        }


def resolve_isaac_workcell_plan(
    project_root: str | Path,
    workcell: WorkcellSpec,
    *,
    task_scene: str | Path | None = None,
    verify_content: bool = False,
) -> ResolvedIsaacWorkcellPlan:
    """Compile a Workcell without importing Isaac Sim or pxr."""

    repository = ConfigRepository(project_root)
    source_lock = SourceLock.load(repository)
    profile, profile_path = _load_typed_profile(repository, workcell)
    frame_poses = _frame_poses(workcell)
    primitives = [
        ResolvedIsaacPrimitive(
            entity_id=entity.entity_id,
            pose=_compose(frame_poses[entity.frame], entity.transform),
            entity=entity,
        )
        for entity in workcell.entities
    ]

    if profile is None:
        policies = IsaacWorkcellPolicies(
            ground=(
                "project"
                if any(
                    entity.primitive.kind == "plane"
                    for entity in workcell.entities
                )
                else "none"
            ),
            physics_scene="project",
            camera="project",
            collision="preserve",
            fixed_rigid_body_paths=(),
        )
        imports: list[ResolvedIsaacUsdImport] = []
        lighting = ResolvedIsaacLighting(
            mode="project",
            content=None,
            intensity=900.0,
            exposure=0.0,
            visible_in_primary_ray=True,
            background_color_rgb=(0.12, 0.12, 0.12),
        )
        expectations = None
        profile_id = None
    else:
        if profile.frame not in frame_poses:
            raise ValueError(
                f"Isaac workcell profile references unknown frame {profile.frame!r}"
            )
        scene = source_lock.resolve_content(
            profile.scene.artifact,
            expected_sha256=profile.scene.expected_sha256,
            verify=verify_content,
        )
        imports = [
            ResolvedIsaacUsdImport(
                import_id=profile.import_id,
                content=scene,
                composition=profile.composition,
                pose=_compose(
                    frame_poses[profile.frame],
                    profile.transform,
                ),
                excluded_prim_paths=(),
                fixed_rigid_body_paths=(
                    profile.policies.fixed_rigid_body_paths
                ),
                expectations=profile.expectations,
            ),
        ]
        lighting_content = (
            None
            if profile.lighting.content is None
            else source_lock.resolve_content(
                profile.lighting.content.artifact,
                expected_sha256=(
                    profile.lighting.content.expected_sha256
                ),
                verify=verify_content,
            )
        )
        policies = profile.policies
        lighting = ResolvedIsaacLighting(
            mode=profile.lighting.mode,
            content=lighting_content,
            intensity=profile.lighting.intensity,
            exposure=profile.lighting.exposure,
            visible_in_primary_ray=(
                profile.lighting.visible_in_primary_ray
            ),
            background_color_rgb=profile.lighting.background_color_rgb,
        )
        expectations = profile.expectations
        profile_id = profile.profile_id

    task_scene_profile_id: str | None = None
    task_scene_profile_path: str | None = None
    dynamic_rigid_body_paths: tuple[tuple[str, str], ...] = ()
    if task_scene is not None:
        task_profile, task_scene_profile_path = _load_task_scene_profile(
            repository,
            task_scene,
        )
        if task_profile.frame not in frame_poses:
            raise ValueError(
                "Isaac task scene references unknown frame "
                f"{task_profile.frame!r}"
            )
        if any(
            operation.import_id == task_profile.import_id
            for operation in imports
        ):
            raise ValueError(
                f"duplicate Isaac import_id {task_profile.import_id!r}"
            )
        task_content = source_lock.resolve_content(
            task_profile.scene.artifact,
            expected_sha256=task_profile.scene.expected_sha256,
            verify=verify_content,
        )
        imports.append(
            ResolvedIsaacUsdImport(
                import_id=task_profile.import_id,
                content=task_content,
                composition=task_profile.composition,
                pose=_compose(
                    frame_poses[task_profile.frame],
                    task_profile.transform,
                ),
                excluded_prim_paths=task_profile.excluded_prim_paths,
                fixed_rigid_body_paths=(
                    task_profile.fixed_rigid_body_paths
                ),
                expectations=task_profile.expectations,
            )
        )
        known_entity_ids = {
            operation.entity_id for operation in primitives
        }
        for entity in task_profile.entities:
            if entity.frame not in frame_poses:
                raise ValueError(
                    "Isaac task scene entity references unknown frame "
                    f"{entity.frame!r}"
                )
            if entity.entity_id in known_entity_ids:
                raise ValueError(
                    f"duplicate Isaac entity_id {entity.entity_id!r}"
                )
            primitives.append(
                ResolvedIsaacPrimitive(
                    entity_id=entity.entity_id,
                    pose=_compose(
                        frame_poses[entity.frame],
                        entity.transform,
                    ),
                    entity=entity,
                )
            )
            known_entity_ids.add(entity.entity_id)
        task_scene_profile_id = task_profile.profile_id
        task_import = imports[-1]
        dynamic_rigid_body_paths = tuple(
            (
                logical_id,
                f"{task_import.prim_path}/{relative_path}",
            )
            for logical_id, relative_path in task_profile.dynamic_rigid_bodies
        )

    fixed_rigid_body_paths = tuple(
        f"{operation.prim_path}/{relative_path}"
        for operation in imports
        for relative_path in operation.fixed_rigid_body_paths
    )

    return ResolvedIsaacWorkcellPlan(
        schema=RESOLVED_ISAAC_WORKCELL_PLAN_SCHEMA,
        workcell_id=workcell.workcell_id,
        profile_id=profile_id,
        profile_path=profile_path,
        task_scene_profile_id=task_scene_profile_id,
        task_scene_profile_path=task_scene_profile_path,
        imports=tuple(imports),
        primitives=tuple(primitives),
        fixed_rigid_body_paths=fixed_rigid_body_paths,
        dynamic_rigid_body_paths=dynamic_rigid_body_paths,
        policies=policies,
        lighting=lighting,
        expectations=expectations,
        frame_ids=(
            workcell.world_frame,
            *(frame.frame_id for frame in workcell.frames),
        ),
        mount_ids=tuple(mount.mount_id for mount in workcell.mounts),
        entity_ids=tuple(operation.entity_id for operation in primitives),
    )


def _load_task_scene_profile(
    repository: ConfigRepository,
    reference: str | Path,
) -> tuple[IsaacTaskSceneProfile, str]:
    path = repository.resolve_project_path(
        reference,
        field="Isaac task scene profile",
    )
    return (
        repository.load_isaac_task_scene_profile(path),
        repository.project_relative(
            path,
            field="Isaac task scene profile",
        ),
    )


def _load_typed_profile(
    repository: ConfigRepository,
    workcell: WorkcellSpec,
) -> tuple[IsaacStaticUsdWorkcellProfile | None, str | None]:
    reference = workcell.compatibility_profile
    if reference is None:
        return None, None
    path = repository.resolve_project_path(
        reference,
        field="workcell compatibility profile",
    )
    document = load_yaml_strict(path.read_text(encoding="utf-8"))
    if not isinstance(document, Mapping):
        raise ValueError(f"workcell compatibility profile must be a mapping: {path}")
    mapping = cast(Mapping[str, object], document)
    if mapping.get("schema") != ISAAC_STATIC_USD_WORKCELL_SCHEMA:
        return None, None
    return (
        IsaacStaticUsdWorkcellProfile.from_mapping(mapping),
        repository.project_relative(
            path,
            field="workcell compatibility profile",
        ),
    )


def _frame_poses(workcell: WorkcellSpec) -> dict[str, PoseSpec]:
    poses = {
        workcell.world_frame: PoseSpec(
            position_m=(0.0, 0.0, 0.0),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        )
    }
    pending = {frame.frame_id: frame for frame in workcell.frames}
    while pending:
        progressed = False
        for frame_id, frame in tuple(pending.items()):
            parent = poses.get(frame.parent)
            if parent is None:
                continue
            poses[frame_id] = _compose(parent, frame.transform)
            del pending[frame_id]
            progressed = True
        if not progressed:
            raise RuntimeError("validated Workcell frame graph could not be resolved")
    return poses


def _compose(parent: PoseSpec, child: PoseSpec) -> PoseSpec:
    pw, px, py, pz = parent.quat_wxyz
    x, y, z = child.position_m
    rotated = (
        (1 - 2 * (py * py + pz * pz)) * x
        + 2 * (px * py - pz * pw) * y
        + 2 * (px * pz + py * pw) * z,
        2 * (px * py + pz * pw) * x
        + (1 - 2 * (px * px + pz * pz)) * y
        + 2 * (py * pz - px * pw) * z,
        2 * (px * pz - py * pw) * x
        + 2 * (py * pz + px * pw) * y
        + (1 - 2 * (px * px + py * py)) * z,
    )
    cw, cx, cy, cz = child.quat_wxyz
    quaternion = (
        pw * cw - px * cx - py * cy - pz * cz,
        pw * cx + px * cw + py * cz - pz * cy,
        pw * cy - px * cz + py * cw + pz * cx,
        pw * cz + px * cy - py * cx + pz * cw,
    )
    norm = math.sqrt(sum(component * component for component in quaternion))
    return PoseSpec(
        position_m=(
            parent.position_m[0] + rotated[0],
            parent.position_m[1] + rotated[1],
            parent.position_m[2] + rotated[2],
        ),
        quat_wxyz=cast(
            tuple[float, float, float, float],
            tuple(component / norm for component in quaternion),
        ),
    )


__all__ = [
    "RESOLVED_ISAAC_WORKCELL_PLAN_SCHEMA",
    "ResolvedIsaacLighting",
    "ResolvedIsaacPrimitive",
    "ResolvedIsaacUsdImport",
    "ResolvedIsaacWorkcellPlan",
    "resolve_isaac_workcell_plan",
]
