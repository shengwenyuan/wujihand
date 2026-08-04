"""Backend-neutral asset manifest specification."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Self, cast

from .common import (
    optional_project_reference,
    require_exact_mapping,
    require_sequence,
    require_string,
    validate_identifier,
    validate_project_reference,
)


ASSET_MANIFEST_SCHEMA_V1 = "wujihand.asset_manifest.v1"
ASSET_MANIFEST_SCHEMA_V2 = "wujihand.asset_manifest.v2"
ASSET_MANIFEST_SCHEMA = ASSET_MANIFEST_SCHEMA_V1
CONTROLLED_ASSET_KINDS = frozenset(
    {"robot_arm", "robot_hand", "virtual_mechanism"}
)
PASSIVE_ASSET_KINDS = frozenset({"passive_component", "simulated_sensor"})
SUPPORTED_ASSET_KINDS = CONTROLLED_ASSET_KINDS | PASSIVE_ASSET_KINDS
SUPPORTED_SIDES = frozenset({"left", "right", "none"})


def _frame_pairs(value: object, *, field: str) -> tuple[tuple[str, str], ...]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} keys must be strings")
    mapping = cast(Mapping[str, object], value)
    pairs = tuple(
        sorted(
            (
                validate_identifier(role, field=f"{field} role"),
                validate_identifier(name, field=f"{field}.{role}"),
            )
            for role, name in mapping.items()
        )
    )
    if not pairs:
        raise ValueError(f"{field} must not be empty")
    names = tuple(name for _, name in pairs)
    if len(set(names)) != len(names):
        raise ValueError(f"{field} canonical frame names must be unique")
    return pairs


@dataclass(frozen=True, slots=True)
class ControlGroupSpec:
    """Canonical command contract for one semantic group of an asset."""

    group_id: str
    semantic: str
    layout_id: str
    dof_count: int
    command_interface: str
    joint_profile: str | None

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "control_group") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "group_id",
                    "semantic",
                    "layout_id",
                    "dof_count",
                    "command_interface",
                    "joint_profile",
                }
            ),
            field=field,
        )
        dof_count = data["dof_count"]
        if (
            isinstance(dof_count, bool)
            or not isinstance(dof_count, int)
            or dof_count < 1
        ):
            raise ValueError(f"{field}.dof_count must be a positive integer")
        return cls(
            group_id=validate_identifier(data["group_id"], field=f"{field}.group_id"),
            semantic=validate_identifier(data["semantic"], field=f"{field}.semantic"),
            layout_id=validate_identifier(data["layout_id"], field=f"{field}.layout_id"),
            dof_count=dof_count,
            command_interface=validate_identifier(
                data["command_interface"], field=f"{field}.command_interface"
            ),
            joint_profile=optional_project_reference(
                data["joint_profile"], field=f"{field}.joint_profile"
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "group_id": self.group_id,
            "semantic": self.semantic,
            "layout_id": self.layout_id,
            "dof_count": self.dof_count,
            "command_interface": self.command_interface,
            "joint_profile": self.joint_profile,
        }


@dataclass(frozen=True, slots=True)
class AssetManifest:
    """Stable asset identity independent of simulator representation."""

    schema: str
    asset_id: str
    revision: str
    kind: str
    product: str
    side: str
    canonical_profile: str | None
    frames: tuple[tuple[str, str], ...]
    control_groups: tuple[ControlGroupSpec, ...]
    provenance_source: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "asset") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "schema",
                    "asset_id",
                    "revision",
                    "kind",
                    "product",
                    "side",
                    "canonical_profile",
                    "frames",
                    "control_groups",
                    "provenance_source",
                }
            ),
            field=field,
        )
        schema = require_string(data["schema"], field=f"{field}.schema")
        if schema not in {ASSET_MANIFEST_SCHEMA_V1, ASSET_MANIFEST_SCHEMA_V2}:
            raise ValueError(
                f"{field}.schema must be one of "
                f"{[ASSET_MANIFEST_SCHEMA_V1, ASSET_MANIFEST_SCHEMA_V2]}"
            )
        kind = require_string(data["kind"], field=f"{field}.kind")
        supported_kinds = (
            CONTROLLED_ASSET_KINDS
            if schema == ASSET_MANIFEST_SCHEMA_V1
            else SUPPORTED_ASSET_KINDS
        )
        if kind not in supported_kinds:
            raise ValueError(
                f"{field}.kind must be one of {sorted(supported_kinds)}"
            )
        side = require_string(data["side"], field=f"{field}.side")
        if side not in SUPPORTED_SIDES:
            raise ValueError(f"{field}.side must be one of {sorted(SUPPORTED_SIDES)}")
        groups = tuple(
            ControlGroupSpec.from_mapping(
                item, field=f"{field}.control_groups[{index}]"
            )
            for index, item in enumerate(
                require_sequence(data["control_groups"], field=f"{field}.control_groups")
            )
        )
        if schema == ASSET_MANIFEST_SCHEMA_V1 and not groups:
            raise ValueError(f"{field}.control_groups must not be empty")
        if schema == ASSET_MANIFEST_SCHEMA_V2:
            if kind in CONTROLLED_ASSET_KINDS and not groups:
                raise ValueError(
                    f"{field}.control_groups must not be empty for controlled assets"
                )
            if kind in PASSIVE_ASSET_KINDS and groups:
                raise ValueError(
                    f"{field}.control_groups must be empty for passive assets"
                )
        group_ids = tuple(group.group_id for group in groups)
        if len(set(group_ids)) != len(group_ids):
            raise ValueError(f"{field}.control_groups group_id values must be unique")
        return cls(
            schema=schema,
            asset_id=validate_identifier(data["asset_id"], field=f"{field}.asset_id"),
            revision=validate_identifier(data["revision"], field=f"{field}.revision"),
            kind=kind,
            product=validate_identifier(data["product"], field=f"{field}.product"),
            side=side,
            canonical_profile=optional_project_reference(
                data["canonical_profile"], field=f"{field}.canonical_profile"
            ),
            frames=_frame_pairs(data["frames"], field=f"{field}.frames"),
            control_groups=groups,
            provenance_source=validate_project_reference(
                data["provenance_source"], field=f"{field}.provenance_source"
            ),
        )

    def frame_name(self, role: str) -> str:
        for candidate, name in self.frames:
            if candidate == role:
                return name
        raise KeyError(role)

    def control_group(self, group_id: str) -> ControlGroupSpec:
        for group in self.control_groups:
            if group.group_id == group_id:
                return group
        raise KeyError(group_id)

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "asset_id": self.asset_id,
            "revision": self.revision,
            "kind": self.kind,
            "product": self.product,
            "side": self.side,
            "canonical_profile": self.canonical_profile,
            "frames": dict(self.frames),
            "control_groups": [group.to_mapping() for group in self.control_groups],
            "provenance_source": self.provenance_source,
        }


__all__ = [
    "ASSET_MANIFEST_SCHEMA",
    "ASSET_MANIFEST_SCHEMA_V1",
    "ASSET_MANIFEST_SCHEMA_V2",
    "AssetManifest",
    "CONTROLLED_ASSET_KINDS",
    "ControlGroupSpec",
    "PASSIVE_ASSET_KINDS",
    "SUPPORTED_ASSET_KINDS",
    "SUPPORTED_SIDES",
]
