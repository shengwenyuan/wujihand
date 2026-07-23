"""Backend representation binding for a backend-neutral asset manifest."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import re
from typing import Self, cast

from .asset import SUPPORTED_SIDES
from .common import (
    optional_project_reference,
    require_exact_mapping,
    require_sequence,
    require_string,
    validate_identifier,
    validate_project_reference,
)


BACKEND_BINDING_SCHEMA = "wujihand.backend_binding.v1"
SUPPORTED_BACKENDS = frozenset({"mujoco", "isaac"})
SUPPORTED_LOADERS = frozenset({"mjcf", "usd", "procedural"})
SUPPORTED_NAMESPACE_POLICIES = frozenset({"preserve", "prefix"})
BUILDER_REGISTRY = frozenset({"hand2_rotation_mount_d6_v1"})
SUPPORTED_SOURCE_REVISION_KINDS = frozenset({"commit", "tag", "sha256"})
_GIT_COMMIT = re.compile(r"(?:[0-9a-f]{40}|[0-9a-f]{64})")
_SHA256 = re.compile(r"[0-9a-f]{64}")


def _unique_strings(value: object, *, field: str, allow_empty: bool) -> tuple[str, ...]:
    items = tuple(
        require_string(item, field=f"{field}[{index}]")
        for index, item in enumerate(require_sequence(value, field=field))
    )
    if not allow_empty and not items:
        raise ValueError(f"{field} must not be empty")
    if len(set(items)) != len(items):
        raise ValueError(f"{field} values must be unique")
    return items


def _frame_pairs(value: object, *, field: str) -> tuple[tuple[str, str], ...]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} keys must be strings")
    mapping = cast(Mapping[str, object], value)
    pairs = tuple(
        sorted(
            (
                validate_identifier(canonical_frame, field=f"{field} canonical frame"),
                require_string(name, field=f"{field}.{canonical_frame}"),
            )
            for canonical_frame, name in mapping.items()
        )
    )
    if not pairs:
        raise ValueError(f"{field} must not be empty")
    return pairs


def _source_revision(value: object, *, field: str) -> str:
    revision = require_string(value, field=field)
    kind, separator, identity = revision.partition(":")
    if (
        not separator
        or kind not in SUPPORTED_SOURCE_REVISION_KINDS
        or not identity
        or ":" in identity
    ):
        raise ValueError(
            f"{field} must be kind:value with kind in "
            f"{sorted(SUPPORTED_SOURCE_REVISION_KINDS)}"
        )
    if kind == "commit" and _GIT_COMMIT.fullmatch(identity) is None:
        raise ValueError(f"{field} commit must be a lowercase 40/64-character hash")
    if kind == "sha256" and _SHA256.fullmatch(identity) is None:
        raise ValueError(f"{field} sha256 must be a lowercase 64-character hash")
    return revision


@dataclass(frozen=True, slots=True)
class ArtifactSpec:
    """Artifact or resource-tree path owned by a source-lock entry."""

    source: str
    source_revision: str
    path: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "artifact") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"source", "source_revision", "path"}),
            field=field,
        )
        return cls(
            source=validate_identifier(data["source"], field=f"{field}.source"),
            source_revision=_source_revision(
                data["source_revision"], field=f"{field}.source_revision"
            ),
            path=validate_project_reference(data["path"], field=f"{field}.path"),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "source": self.source,
            "source_revision": self.source_revision,
            "path": self.path,
        }


@dataclass(frozen=True, slots=True)
class GroupBindingSpec:
    """Map one canonical control group to backend joints and actuators."""

    group_id: str
    joints: tuple[str, ...]
    actuators: tuple[str, ...]

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "group_binding") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"group_id", "joints", "actuators"}),
            field=field,
        )
        return cls(
            group_id=validate_identifier(data["group_id"], field=f"{field}.group_id"),
            joints=_unique_strings(
                data["joints"], field=f"{field}.joints", allow_empty=False
            ),
            actuators=_unique_strings(
                data["actuators"], field=f"{field}.actuators", allow_empty=True
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "group_id": self.group_id,
            "joints": list(self.joints),
            "actuators": list(self.actuators),
        }


@dataclass(frozen=True, slots=True)
class BackendBinding:
    """Pinned backend representation of one stable asset identity."""

    schema: str
    binding_id: str
    asset_id: str
    asset_revision: str
    asset_side: str
    backend: str
    namespace_policy: str
    loader: str
    artifact: ArtifactSpec | None
    resource_trees: tuple[ArtifactSpec, ...]
    root: str
    frame_map: tuple[tuple[str, str], ...]
    group_bindings: tuple[GroupBindingSpec, ...]
    builder: str | None
    compatibility_profile: str | None

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "binding") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "schema",
                    "binding_id",
                    "asset_id",
                    "asset_revision",
                    "asset_side",
                    "backend",
                    "namespace_policy",
                    "loader",
                    "artifact",
                    "resource_trees",
                    "root",
                    "frame_map",
                    "group_bindings",
                    "builder",
                    "compatibility_profile",
                }
            ),
            field=field,
        )
        schema = require_string(data["schema"], field=f"{field}.schema")
        if schema != BACKEND_BINDING_SCHEMA:
            raise ValueError(f"{field}.schema must be {BACKEND_BINDING_SCHEMA!r}")
        asset_side = require_string(data["asset_side"], field=f"{field}.asset_side")
        if asset_side not in SUPPORTED_SIDES:
            raise ValueError(
                f"{field}.asset_side must be one of {sorted(SUPPORTED_SIDES)}"
            )
        backend = require_string(data["backend"], field=f"{field}.backend")
        if backend not in SUPPORTED_BACKENDS:
            raise ValueError(f"{field}.backend must be one of {sorted(SUPPORTED_BACKENDS)}")
        namespace_policy = require_string(
            data["namespace_policy"], field=f"{field}.namespace_policy"
        )
        if namespace_policy not in SUPPORTED_NAMESPACE_POLICIES:
            raise ValueError(
                f"{field}.namespace_policy must be one of "
                f"{sorted(SUPPORTED_NAMESPACE_POLICIES)}"
            )
        loader = require_string(data["loader"], field=f"{field}.loader")
        if loader not in SUPPORTED_LOADERS:
            raise ValueError(f"{field}.loader must be one of {sorted(SUPPORTED_LOADERS)}")
        if loader == "mjcf" and backend != "mujoco":
            raise ValueError(f"{field}.loader mjcf requires backend mujoco")
        if loader == "usd" and backend != "isaac":
            raise ValueError(f"{field}.loader usd requires backend isaac")

        artifact = (
            None
            if data["artifact"] is None
            else ArtifactSpec.from_mapping(data["artifact"], field=f"{field}.artifact")
        )
        builder = (
            None
            if data["builder"] is None
            else require_string(data["builder"], field=f"{field}.builder")
        )
        if loader == "procedural":
            if backend != "isaac":
                raise ValueError(
                    f"{field}.loader procedural is currently supported only by isaac"
                )
            if artifact is not None:
                raise ValueError(f"{field}.artifact must be null for procedural loader")
            if builder not in BUILDER_REGISTRY:
                raise ValueError(
                    f"{field}.builder must be one of {sorted(BUILDER_REGISTRY)} "
                    "for procedural loader"
                )
        elif artifact is None or builder is not None:
            raise ValueError(
                f"{field} artifact loaders require artifact and a null builder"
            )

        resource_trees = tuple(
            ArtifactSpec.from_mapping(item, field=f"{field}.resource_trees[{index}]")
            for index, item in enumerate(
                require_sequence(data["resource_trees"], field=f"{field}.resource_trees")
            )
        )
        resource_keys = tuple((item.source, item.path) for item in resource_trees)
        if len(set(resource_keys)) != len(resource_keys):
            raise ValueError(f"{field}.resource_trees entries must be unique")

        groups = tuple(
            GroupBindingSpec.from_mapping(
                item, field=f"{field}.group_bindings[{index}]"
            )
            for index, item in enumerate(
                require_sequence(data["group_bindings"], field=f"{field}.group_bindings")
            )
        )
        if not groups:
            raise ValueError(f"{field}.group_bindings must not be empty")
        group_ids = tuple(group.group_id for group in groups)
        if len(set(group_ids)) != len(group_ids):
            raise ValueError(f"{field}.group_bindings group_id values must be unique")
        all_joints = tuple(joint for group in groups for joint in group.joints)
        if len(set(all_joints)) != len(all_joints):
            raise ValueError(f"{field}.group_bindings joints must not overlap")
        all_actuators = tuple(actuator for group in groups for actuator in group.actuators)
        if len(set(all_actuators)) != len(all_actuators):
            raise ValueError(f"{field}.group_bindings actuators must not overlap")

        return cls(
            schema=schema,
            binding_id=validate_identifier(
                data["binding_id"], field=f"{field}.binding_id"
            ),
            asset_id=validate_identifier(data["asset_id"], field=f"{field}.asset_id"),
            asset_revision=validate_identifier(
                data["asset_revision"], field=f"{field}.asset_revision"
            ),
            asset_side=asset_side,
            backend=backend,
            namespace_policy=namespace_policy,
            loader=loader,
            artifact=artifact,
            resource_trees=resource_trees,
            root=require_string(data["root"], field=f"{field}.root"),
            frame_map=_frame_pairs(data["frame_map"], field=f"{field}.frame_map"),
            group_bindings=groups,
            builder=builder,
            compatibility_profile=optional_project_reference(
                data["compatibility_profile"],
                field=f"{field}.compatibility_profile",
            ),
        )

    def backend_frame(self, canonical_frame: str) -> str:
        for candidate, name in self.frame_map:
            if candidate == canonical_frame:
                return name
        raise KeyError(canonical_frame)

    def group_binding(self, group_id: str) -> GroupBindingSpec:
        for binding in self.group_bindings:
            if binding.group_id == group_id:
                return binding
        raise KeyError(group_id)

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "binding_id": self.binding_id,
            "asset_id": self.asset_id,
            "asset_revision": self.asset_revision,
            "asset_side": self.asset_side,
            "backend": self.backend,
            "namespace_policy": self.namespace_policy,
            "loader": self.loader,
            "artifact": None if self.artifact is None else self.artifact.to_mapping(),
            "resource_trees": [tree.to_mapping() for tree in self.resource_trees],
            "root": self.root,
            "frame_map": dict(self.frame_map),
            "group_bindings": [group.to_mapping() for group in self.group_bindings],
            "builder": self.builder,
            "compatibility_profile": self.compatibility_profile,
        }


__all__ = [
    "ArtifactSpec",
    "BACKEND_BINDING_SCHEMA",
    "BUILDER_REGISTRY",
    "BackendBinding",
    "GroupBindingSpec",
    "SUPPORTED_BACKENDS",
    "SUPPORTED_LOADERS",
    "SUPPORTED_NAMESPACE_POLICIES",
    "SUPPORTED_SOURCE_REVISION_KINDS",
]
