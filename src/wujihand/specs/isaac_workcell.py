"""Typed Isaac USD compatibility leaf for Workcell v1."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import re
from typing import Self

from .backend_binding import ArtifactSpec
from .common import (
    PoseSpec,
    finite_number,
    positive_number,
    require_exact_mapping,
    require_sequence,
    require_string,
    validate_identifier,
)
from .workcell import EntitySpec


ISAAC_STATIC_USD_WORKCELL_SCHEMA = "wujihand.isaac_static_usd_workcell.v1"
ISAAC_TASK_SCENE_SCHEMA = "wujihand.isaac_task_scene.v2"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_GROUND_POLICIES = frozenset({"preserve", "project", "none"})
_PHYSICS_SCENE_POLICIES = frozenset({"preserve", "project"})
_CAMERA_POLICIES = frozenset({"preserve", "project"})
_LIGHTING_MODES = frozenset({"preserve", "project", "selected_hdr"})
_PRIM_PATH_COMPONENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _choice(
    value: object,
    *,
    choices: frozenset[str],
    field: str,
) -> str:
    result = require_string(value, field=field)
    if result not in choices:
        raise ValueError(f"{field} must be one of {sorted(choices)}")
    return result


def _sha256(value: object, *, field: str) -> str:
    result = require_string(value, field=field)
    if _SHA256.fullmatch(result) is None:
        raise ValueError(f"{field} must be a lowercase SHA-256")
    return result


def _non_negative_int(value: object, *, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    return value


def _boolean(value: object, *, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{field} must be a boolean")
    return value


def _color3(
    value: object,
    *,
    field: str,
) -> tuple[float, float, float]:
    items = require_sequence(value, field=field)
    if len(items) != 3:
        raise ValueError(f"{field} must contain exactly three values")
    channels = tuple(
        finite_number(item, field=f"{field}[{index}]")
        for index, item in enumerate(items)
    )
    if any(channel < 0.0 or channel > 1.0 for channel in channels):
        raise ValueError(f"{field} values must be in [0, 1]")
    return (channels[0], channels[1], channels[2])


def _relative_prim_paths(
    value: object,
    *,
    field: str,
) -> tuple[str, ...]:
    items = require_sequence(value, field=field)
    result: list[str] = []
    for index, item in enumerate(items):
        path = require_string(item, field=f"{field}[{index}]")
        components = path.split("/")
        if not components or any(
            _PRIM_PATH_COMPONENT.fullmatch(component) is None
            for component in components
        ):
            raise ValueError(
                f"{field}[{index}] must be a relative USD prim path"
            )
        result.append(path)
    if len(set(result)) != len(result):
        raise ValueError(f"{field} must not contain duplicates")
    return tuple(result)


def _dynamic_rigid_bodies(
    value: object,
    *,
    field: str,
) -> tuple[tuple[str, str], ...]:
    if not isinstance(value, Mapping) or any(
        not isinstance(key, str) for key in value
    ):
        raise ValueError(f"{field} must be a string-keyed mapping")
    result = tuple(
        (
            validate_identifier(logical_id, field=f"{field}.{logical_id}"),
            _relative_prim_paths(
                [relative_path],
                field=f"{field}.{logical_id}",
            )[0],
        )
        for logical_id, relative_path in sorted(value.items())
    )
    paths = tuple(path for _, path in result)
    if len(set(paths)) != len(paths):
        raise ValueError(f"{field} USD prim paths must be unique")
    return result


@dataclass(frozen=True, slots=True)
class ContentSpec:
    """A source-locked path with a digest duplicated into the profile identity."""

    artifact: ArtifactSpec
    expected_sha256: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {"source", "source_revision", "path", "expected_sha256"}
            ),
            field=field,
        )
        return cls(
            artifact=ArtifactSpec.from_mapping(
                {
                    "source": data["source"],
                    "source_revision": data["source_revision"],
                    "path": data["path"],
                },
                field=field,
            ),
            expected_sha256=_sha256(
                data["expected_sha256"],
                field=f"{field}.expected_sha256",
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            **self.artifact.to_mapping(),
            "expected_sha256": self.expected_sha256,
        }


@dataclass(frozen=True, slots=True)
class IsaacWorkcellPolicies:
    ground: str
    physics_scene: str
    camera: str
    collision: str
    fixed_rigid_body_paths: tuple[str, ...]

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        base_keys = frozenset(
            {"ground", "physics_scene", "camera", "collision"}
        )
        has_fixed_overrides = bool(
            isinstance(value, Mapping)
            and "fixed_rigid_body_paths" in value
        )
        data = require_exact_mapping(
            value,
            expected=(
                base_keys | {"fixed_rigid_body_paths"}
                if has_fixed_overrides
                else base_keys
            ),
            field=field,
        )
        collision = require_string(
            data["collision"],
            field=f"{field}.collision",
        )
        if collision != "preserve":
            raise ValueError(f"{field}.collision must be 'preserve'")
        return cls(
            ground=_choice(
                data["ground"],
                choices=_GROUND_POLICIES,
                field=f"{field}.ground",
            ),
            physics_scene=_choice(
                data["physics_scene"],
                choices=_PHYSICS_SCENE_POLICIES,
                field=f"{field}.physics_scene",
            ),
            camera=_choice(
                data["camera"],
                choices=_CAMERA_POLICIES,
                field=f"{field}.camera",
            ),
            collision=collision,
            fixed_rigid_body_paths=(
                _relative_prim_paths(
                    data["fixed_rigid_body_paths"],
                    field=f"{field}.fixed_rigid_body_paths",
                )
                if has_fixed_overrides
                else ()
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "ground": self.ground,
            "physics_scene": self.physics_scene,
            "camera": self.camera,
            "collision": self.collision,
            "fixed_rigid_body_paths": list(
                self.fixed_rigid_body_paths
            ),
        }


@dataclass(frozen=True, slots=True)
class IsaacDomeLightingSpec:
    mode: str
    content: ContentSpec | None
    intensity: float
    exposure: float
    visible_in_primary_ray: bool
    background_color_rgb: tuple[float, float, float]

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "mode",
                    "content",
                    "intensity",
                    "exposure",
                    "visible_in_primary_ray",
                    "background_color_rgb",
                }
            ),
            field=field,
        )
        mode = _choice(
            data["mode"],
            choices=_LIGHTING_MODES,
            field=f"{field}.mode",
        )
        raw_content = data["content"]
        if mode == "selected_hdr":
            content = ContentSpec.from_mapping(
                raw_content,
                field=f"{field}.content",
            )
        else:
            if raw_content is not None:
                raise ValueError(
                    f"{field}.content must be null unless mode is selected_hdr"
                )
            content = None
        return cls(
            mode=mode,
            content=content,
            intensity=positive_number(
                data["intensity"],
                field=f"{field}.intensity",
            ),
            exposure=finite_number(
                data["exposure"],
                field=f"{field}.exposure",
            ),
            visible_in_primary_ray=_boolean(
                data["visible_in_primary_ray"],
                field=f"{field}.visible_in_primary_ray",
            ),
            background_color_rgb=_color3(
                data["background_color_rgb"],
                field=f"{field}.background_color_rgb",
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "mode": self.mode,
            "content": (
                None if self.content is None else self.content.to_mapping()
            ),
            "intensity": self.intensity,
            "exposure": self.exposure,
            "visible_in_primary_ray": self.visible_in_primary_ray,
            "background_color_rgb": list(self.background_color_rgb),
        }


@dataclass(frozen=True, slots=True)
class IsaacSceneExpectations:
    default_prim: str
    meters_per_unit: float
    up_axis: str
    min_colliders: int

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "default_prim",
                    "meters_per_unit",
                    "up_axis",
                    "min_colliders",
                }
            ),
            field=field,
        )
        up_axis = require_string(data["up_axis"], field=f"{field}.up_axis")
        if up_axis not in {"Y", "Z"}:
            raise ValueError(f"{field}.up_axis must be Y or Z")
        return cls(
            default_prim=validate_identifier(
                data["default_prim"],
                field=f"{field}.default_prim",
            ),
            meters_per_unit=positive_number(
                data["meters_per_unit"],
                field=f"{field}.meters_per_unit",
            ),
            up_axis=up_axis,
            min_colliders=_non_negative_int(
                data["min_colliders"],
                field=f"{field}.min_colliders",
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "default_prim": self.default_prim,
            "meters_per_unit": self.meters_per_unit,
            "up_axis": self.up_axis,
            "min_colliders": self.min_colliders,
        }


@dataclass(frozen=True, slots=True)
class IsaacStaticUsdWorkcellProfile:
    """One source-locked scene import plus singleton and lighting policies."""

    schema: str
    profile_id: str
    import_id: str
    scene: ContentSpec
    composition: str
    frame: str
    transform: PoseSpec
    policies: IsaacWorkcellPolicies
    lighting: IsaacDomeLightingSpec
    expectations: IsaacSceneExpectations

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "Isaac static USD workcell profile",
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "schema",
                    "profile_id",
                    "import_id",
                    "scene",
                    "composition",
                    "frame",
                    "transform",
                    "policies",
                    "lighting",
                    "expectations",
                }
            ),
            field=field,
        )
        schema = require_string(data["schema"], field=f"{field}.schema")
        if schema != ISAAC_STATIC_USD_WORKCELL_SCHEMA:
            raise ValueError(
                f"{field}.schema must be {ISAAC_STATIC_USD_WORKCELL_SCHEMA!r}"
            )
        composition = require_string(
            data["composition"],
            field=f"{field}.composition",
        )
        if composition != "reference":
            raise ValueError(f"{field}.composition must be 'reference'")
        return cls(
            schema=schema,
            profile_id=validate_identifier(
                data["profile_id"],
                field=f"{field}.profile_id",
            ),
            import_id=validate_identifier(
                data["import_id"],
                field=f"{field}.import_id",
            ),
            scene=ContentSpec.from_mapping(
                data["scene"],
                field=f"{field}.scene",
            ),
            composition=composition,
            frame=validate_identifier(
                data["frame"],
                field=f"{field}.frame",
            ),
            transform=PoseSpec.from_mapping(
                data["transform"],
                field=f"{field}.transform",
            ),
            policies=IsaacWorkcellPolicies.from_mapping(
                data["policies"],
                field=f"{field}.policies",
            ),
            lighting=IsaacDomeLightingSpec.from_mapping(
                data["lighting"],
                field=f"{field}.lighting",
            ),
            expectations=IsaacSceneExpectations.from_mapping(
                data["expectations"],
                field=f"{field}.expectations",
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "profile_id": self.profile_id,
            "import_id": self.import_id,
            "scene": self.scene.to_mapping(),
            "composition": self.composition,
            "frame": self.frame,
            "transform": self.transform.to_mapping(),
            "policies": self.policies.to_mapping(),
            "lighting": self.lighting.to_mapping(),
            "expectations": self.expectations.to_mapping(),
        }


@dataclass(frozen=True, slots=True)
class IsaacTaskSceneProfile:
    """One independently selectable task scene layered over a Workcell."""

    schema: str
    profile_id: str
    import_id: str
    scene: ContentSpec
    composition: str
    frame: str
    transform: PoseSpec
    excluded_prim_paths: tuple[str, ...]
    fixed_rigid_body_paths: tuple[str, ...]
    dynamic_rigid_bodies: tuple[tuple[str, str], ...]
    entities: tuple[EntitySpec, ...]
    expectations: IsaacSceneExpectations

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "Isaac task scene profile",
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "schema",
                    "profile_id",
                    "import_id",
                    "scene",
                    "composition",
                    "frame",
                    "transform",
                    "excluded_prim_paths",
                    "fixed_rigid_body_paths",
                    "dynamic_rigid_bodies",
                    "entities",
                    "expectations",
                }
            ),
            field=field,
        )
        schema = require_string(data["schema"], field=f"{field}.schema")
        if schema != ISAAC_TASK_SCENE_SCHEMA:
            raise ValueError(
                f"{field}.schema must be {ISAAC_TASK_SCENE_SCHEMA!r}"
            )
        composition = require_string(
            data["composition"],
            field=f"{field}.composition",
        )
        if composition != "reference":
            raise ValueError(f"{field}.composition must be 'reference'")
        entities = tuple(
            EntitySpec.from_mapping(
                item,
                field=f"{field}.entities[{index}]",
            )
            for index, item in enumerate(
                require_sequence(data["entities"], field=f"{field}.entities")
            )
        )
        entity_ids = tuple(entity.entity_id for entity in entities)
        if len(set(entity_ids)) != len(entity_ids):
            raise ValueError(f"{field}.entities entity_id values must be unique")
        fixed_rigid_body_paths = _relative_prim_paths(
            data["fixed_rigid_body_paths"],
            field=f"{field}.fixed_rigid_body_paths",
        )
        dynamic_rigid_bodies = _dynamic_rigid_bodies(
            data["dynamic_rigid_bodies"],
            field=f"{field}.dynamic_rigid_bodies",
        )
        if set(fixed_rigid_body_paths) & {
            path for _, path in dynamic_rigid_bodies
        }:
            raise ValueError(
                f"{field} cannot declare one rigid body as both fixed and dynamic"
            )
        return cls(
            schema=schema,
            profile_id=validate_identifier(
                data["profile_id"],
                field=f"{field}.profile_id",
            ),
            import_id=validate_identifier(
                data["import_id"],
                field=f"{field}.import_id",
            ),
            scene=ContentSpec.from_mapping(
                data["scene"],
                field=f"{field}.scene",
            ),
            composition=composition,
            frame=validate_identifier(
                data["frame"],
                field=f"{field}.frame",
            ),
            transform=PoseSpec.from_mapping(
                data["transform"],
                field=f"{field}.transform",
            ),
            excluded_prim_paths=_relative_prim_paths(
                data["excluded_prim_paths"],
                field=f"{field}.excluded_prim_paths",
            ),
            fixed_rigid_body_paths=fixed_rigid_body_paths,
            dynamic_rigid_bodies=dynamic_rigid_bodies,
            entities=entities,
            expectations=IsaacSceneExpectations.from_mapping(
                data["expectations"],
                field=f"{field}.expectations",
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "profile_id": self.profile_id,
            "import_id": self.import_id,
            "scene": self.scene.to_mapping(),
            "composition": self.composition,
            "frame": self.frame,
            "transform": self.transform.to_mapping(),
            "excluded_prim_paths": list(self.excluded_prim_paths),
            "fixed_rigid_body_paths": list(self.fixed_rigid_body_paths),
            "dynamic_rigid_bodies": dict(self.dynamic_rigid_bodies),
            "entities": [entity.to_mapping() for entity in self.entities],
            "expectations": self.expectations.to_mapping(),
        }


__all__ = [
    "ISAAC_STATIC_USD_WORKCELL_SCHEMA",
    "ISAAC_TASK_SCENE_SCHEMA",
    "ContentSpec",
    "IsaacDomeLightingSpec",
    "IsaacSceneExpectations",
    "IsaacStaticUsdWorkcellProfile",
    "IsaacTaskSceneProfile",
    "IsaacWorkcellPolicies",
]
