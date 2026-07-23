"""Backend-neutral workcell frame, mount, and entity specification."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Self, cast

from .common import (
    PoseSpec,
    optional_project_reference,
    positive_number,
    positive_vector,
    require_exact_mapping,
    require_sequence,
    require_string,
    validate_identifier,
)


WORKCELL_SCHEMA = "wujihand.workcell.v1"
SUPPORTED_PRIMITIVES = frozenset({"plane", "box", "sphere", "frustum"})
SUPPORTED_MOBILITY = frozenset({"fixed", "dynamic"})


@dataclass(frozen=True, slots=True)
class PrimitiveSpec:
    """Closed primitive vocabulary used by first-generation workcell specs."""

    kind: str
    size_m: tuple[float, float, float] | None = None
    radius_m: float | None = None
    height_m: float | None = None
    top_size_m: tuple[float, float] | None = None
    bottom_size_m: tuple[float, float] | None = None

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "primitive") -> Self:
        if not isinstance(value, Mapping):
            raise ValueError(f"{field} must be a mapping")
        kind = require_string(value.get("kind"), field=f"{field}.kind")
        if kind not in SUPPORTED_PRIMITIVES:
            raise ValueError(
                f"{field}.kind must be one of {sorted(SUPPORTED_PRIMITIVES)}"
            )
        if kind == "plane":
            require_exact_mapping(value, expected=frozenset({"kind"}), field=field)
            return cls(kind=kind)
        if kind == "box":
            data = require_exact_mapping(
                value, expected=frozenset({"kind", "size_m"}), field=field
            )
            return cls(
                kind=kind,
                size_m=cast(
                    tuple[float, float, float],
                    positive_vector(data["size_m"], size=3, field=f"{field}.size_m"),
                ),
            )
        if kind == "sphere":
            data = require_exact_mapping(
                value, expected=frozenset({"kind", "radius_m"}), field=field
            )
            return cls(
                kind=kind,
                radius_m=positive_number(
                    data["radius_m"], field=f"{field}.radius_m"
                ),
            )
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {"kind", "height_m", "top_size_m", "bottom_size_m"}
            ),
            field=field,
        )
        return cls(
            kind=kind,
            height_m=positive_number(data["height_m"], field=f"{field}.height_m"),
            top_size_m=cast(
                tuple[float, float],
                positive_vector(
                    data["top_size_m"], size=2, field=f"{field}.top_size_m"
                ),
            ),
            bottom_size_m=cast(
                tuple[float, float],
                positive_vector(
                    data["bottom_size_m"], size=2, field=f"{field}.bottom_size_m"
                ),
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        if self.kind == "plane":
            return {"kind": self.kind}
        if self.kind == "box":
            return {"kind": self.kind, "size_m": list(self.size_m or ())}
        if self.kind == "sphere":
            return {"kind": self.kind, "radius_m": self.radius_m}
        return {
            "kind": self.kind,
            "height_m": self.height_m,
            "top_size_m": list(self.top_size_m or ()),
            "bottom_size_m": list(self.bottom_size_m or ()),
        }


@dataclass(frozen=True, slots=True)
class WorkcellFrameSpec:
    """A semantic frame whose parent is the world or another workcell frame."""

    frame_id: str
    parent: str
    transform: PoseSpec

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "frame") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"frame_id", "parent", "transform"}),
            field=field,
        )
        return cls(
            frame_id=validate_identifier(data["frame_id"], field=f"{field}.frame_id"),
            parent=validate_identifier(data["parent"], field=f"{field}.parent"),
            transform=PoseSpec.from_mapping(
                data["transform"], field=f"{field}.transform"
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "frame_id": self.frame_id,
            "parent": self.parent,
            "transform": self.transform.to_mapping(),
        }


@dataclass(frozen=True, slots=True)
class MountSpec:
    """Named placement slot expressed relative to a semantic workcell frame."""

    mount_id: str
    frame: str
    transform: PoseSpec

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "mount") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"mount_id", "frame", "transform"}),
            field=field,
        )
        return cls(
            mount_id=validate_identifier(data["mount_id"], field=f"{field}.mount_id"),
            frame=validate_identifier(data["frame"], field=f"{field}.frame"),
            transform=PoseSpec.from_mapping(
                data["transform"], field=f"{field}.transform"
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "mount_id": self.mount_id,
            "frame": self.frame,
            "transform": self.transform.to_mapping(),
        }


@dataclass(frozen=True, slots=True)
class EntitySpec:
    """Physical workcell entity using a closed primitive and mobility contract."""

    entity_id: str
    frame: str
    transform: PoseSpec
    primitive: PrimitiveSpec
    mobility: str
    mass_kg: float | None

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "entity") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "entity_id",
                    "frame",
                    "transform",
                    "primitive",
                    "mobility",
                    "mass_kg",
                }
            ),
            field=field,
        )
        mobility = require_string(data["mobility"], field=f"{field}.mobility")
        if mobility not in SUPPORTED_MOBILITY:
            raise ValueError(
                f"{field}.mobility must be one of {sorted(SUPPORTED_MOBILITY)}"
            )
        if mobility == "fixed":
            if data["mass_kg"] is not None:
                raise ValueError(f"{field}.mass_kg must be null for a fixed entity")
            mass_kg = None
        else:
            mass_kg = positive_number(data["mass_kg"], field=f"{field}.mass_kg")
        primitive = PrimitiveSpec.from_mapping(
            data["primitive"], field=f"{field}.primitive"
        )
        if mobility == "dynamic" and primitive.kind == "plane":
            raise ValueError(f"{field} dynamic plane is not supported")
        return cls(
            entity_id=validate_identifier(
                data["entity_id"], field=f"{field}.entity_id"
            ),
            frame=validate_identifier(data["frame"], field=f"{field}.frame"),
            transform=PoseSpec.from_mapping(
                data["transform"], field=f"{field}.transform"
            ),
            primitive=primitive,
            mobility=mobility,
            mass_kg=mass_kg,
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "entity_id": self.entity_id,
            "frame": self.frame,
            "transform": self.transform.to_mapping(),
            "primitive": self.primitive.to_mapping(),
            "mobility": self.mobility,
            "mass_kg": self.mass_kg,
        }


def _validate_frame_graph(
    world_frame: str, frames: tuple[WorkcellFrameSpec, ...], *, field: str
) -> None:
    frame_ids = {frame.frame_id for frame in frames}
    if world_frame in frame_ids:
        raise ValueError(f"{field}.frames must not redeclare world_frame")
    known = frame_ids | {world_frame}
    parent_by_frame = {frame.frame_id: frame.parent for frame in frames}
    for frame in frames:
        if frame.parent not in known:
            raise ValueError(
                f"{field}.frames references unknown parent {frame.parent!r}"
            )

    state: dict[str, int] = {}

    def visit(frame_id: str) -> None:
        if frame_id == world_frame:
            return
        marker = state.get(frame_id, 0)
        if marker == 1:
            raise ValueError(f"{field}.frames must form an acyclic graph")
        if marker == 2:
            return
        state[frame_id] = 1
        visit(parent_by_frame[frame_id])
        state[frame_id] = 2

    for frame_id in frame_ids:
        visit(frame_id)


@dataclass(frozen=True, slots=True)
class WorkcellSpec:
    """World geometry and semantic placement slots, excluding robot instances."""

    schema: str
    workcell_id: str
    world_frame: str
    frames: tuple[WorkcellFrameSpec, ...]
    mounts: tuple[MountSpec, ...]
    entities: tuple[EntitySpec, ...]
    compatibility_profile: str | None

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "workcell") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "schema",
                    "workcell_id",
                    "world_frame",
                    "frames",
                    "mounts",
                    "entities",
                    "compatibility_profile",
                }
            ),
            field=field,
        )
        schema = require_string(data["schema"], field=f"{field}.schema")
        if schema != WORKCELL_SCHEMA:
            raise ValueError(f"{field}.schema must be {WORKCELL_SCHEMA!r}")
        world_frame = validate_identifier(
            data["world_frame"], field=f"{field}.world_frame"
        )
        frames = tuple(
            WorkcellFrameSpec.from_mapping(item, field=f"{field}.frames[{index}]")
            for index, item in enumerate(
                require_sequence(data["frames"], field=f"{field}.frames")
            )
        )
        frame_ids = tuple(frame.frame_id for frame in frames)
        if len(set(frame_ids)) != len(frame_ids):
            raise ValueError(f"{field}.frames frame_id values must be unique")
        _validate_frame_graph(world_frame, frames, field=field)
        known_frames = set(frame_ids) | {world_frame}

        mounts = tuple(
            MountSpec.from_mapping(item, field=f"{field}.mounts[{index}]")
            for index, item in enumerate(
                require_sequence(data["mounts"], field=f"{field}.mounts")
            )
        )
        if not mounts:
            raise ValueError(f"{field}.mounts must not be empty")
        mount_ids = tuple(mount.mount_id for mount in mounts)
        if len(set(mount_ids)) != len(mount_ids):
            raise ValueError(f"{field}.mounts mount_id values must be unique")
        for mount in mounts:
            if mount.frame not in known_frames:
                raise ValueError(
                    f"{field}.mounts references unknown frame {mount.frame!r}"
                )

        entities = tuple(
            EntitySpec.from_mapping(item, field=f"{field}.entities[{index}]")
            for index, item in enumerate(
                require_sequence(data["entities"], field=f"{field}.entities")
            )
        )
        entity_ids = tuple(entity.entity_id for entity in entities)
        if len(set(entity_ids)) != len(entity_ids):
            raise ValueError(f"{field}.entities entity_id values must be unique")
        for entity in entities:
            if entity.frame not in known_frames:
                raise ValueError(
                    f"{field}.entities references unknown frame {entity.frame!r}"
                )

        return cls(
            schema=schema,
            workcell_id=validate_identifier(
                data["workcell_id"], field=f"{field}.workcell_id"
            ),
            world_frame=world_frame,
            frames=frames,
            mounts=mounts,
            entities=entities,
            compatibility_profile=optional_project_reference(
                data["compatibility_profile"],
                field=f"{field}.compatibility_profile",
            ),
        )

    def mount(self, mount_id: str) -> MountSpec:
        for mount in self.mounts:
            if mount.mount_id == mount_id:
                return mount
        raise KeyError(mount_id)

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "workcell_id": self.workcell_id,
            "world_frame": self.world_frame,
            "frames": [frame.to_mapping() for frame in self.frames],
            "mounts": [mount.to_mapping() for mount in self.mounts],
            "entities": [entity.to_mapping() for entity in self.entities],
            "compatibility_profile": self.compatibility_profile,
        }


__all__ = [
    "EntitySpec",
    "MountSpec",
    "PrimitiveSpec",
    "SUPPORTED_MOBILITY",
    "SUPPORTED_PRIMITIVES",
    "WORKCELL_SCHEMA",
    "WorkcellFrameSpec",
    "WorkcellSpec",
]
