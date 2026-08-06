"""Session composition-root specification."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Self, cast

from .common import (
    ConfigRef,
    optional_project_reference,
    require_exact_mapping,
    require_sequence,
    require_string,
    validate_identifier,
)


SESSION_SCHEMA_V1 = "wujihand.session.v1"
SESSION_SCHEMA_V2 = "wujihand.session.v2"
SESSION_SCHEMA = SESSION_SCHEMA_V1
SUPPORTED_BACKENDS = frozenset({"mujoco", "isaac"})
SUPPORTED_RUNTIME_ROLES = frozenset(
    {"simulation", "teleop_producer", "teleop_consumer", "qualification"}
)


def _reference_pairs(value: object, *, field: str) -> tuple[tuple[str, ConfigRef], ...]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} keys must be strings")
    mapping = cast(Mapping[str, object], value)
    return tuple(
        sorted(
            (
                validate_identifier(key, field=f"{field} key"),
                ConfigRef.from_mapping(item, field=f"{field}.{key}"),
            )
            for key, item in mapping.items()
        )
    )


def _placement_pairs(value: object, *, field: str) -> tuple[tuple[str, str], ...]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} keys must be strings")
    mapping = cast(Mapping[str, object], value)
    return tuple(
        sorted(
            (
                validate_identifier(key, field=f"{field} key"),
                validate_identifier(item, field=f"{field}.{key}"),
            )
            for key, item in mapping.items()
        )
    )


@dataclass(frozen=True, slots=True)
class ControlLayoutSpec:
    """Route one asset-instance control group through an explicit layout."""

    instance_id: str
    group_id: str
    layout_id: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "control_layout") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"instance_id", "group_id", "layout_id"}),
            field=field,
        )
        return cls(
            instance_id=validate_identifier(data["instance_id"], field=f"{field}.instance_id"),
            group_id=validate_identifier(data["group_id"], field=f"{field}.group_id"),
            layout_id=validate_identifier(data["layout_id"], field=f"{field}.layout_id"),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "instance_id": self.instance_id,
            "group_id": self.group_id,
            "layout_id": self.layout_id,
        }


@dataclass(frozen=True, slots=True)
class RuntimeSpec:
    """Runtime compatibility leaf plus explicit transport and layouts."""

    compatibility_profile: str | None
    transport_contract: str | None
    control_layouts: tuple[ControlLayoutSpec, ...]

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "runtime") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"compatibility_profile", "transport_contract", "control_layouts"}),
            field=field,
        )
        layouts = tuple(
            ControlLayoutSpec.from_mapping(item, field=f"{field}.control_layouts[{index}]")
            for index, item in enumerate(
                require_sequence(data["control_layouts"], field=f"{field}.control_layouts")
            )
        )
        routes = tuple((layout.instance_id, layout.group_id) for layout in layouts)
        if len(set(routes)) != len(routes):
            raise ValueError(f"{field}.control_layouts must route each instance/group at most once")
        return cls(
            compatibility_profile=optional_project_reference(
                data["compatibility_profile"],
                field=f"{field}.compatibility_profile",
            ),
            transport_contract=(
                None
                if data["transport_contract"] is None
                else validate_identifier(
                    data["transport_contract"], field=f"{field}.transport_contract"
                )
            ),
            control_layouts=layouts,
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "compatibility_profile": self.compatibility_profile,
            "transport_contract": self.transport_contract,
            "control_layouts": [layout.to_mapping() for layout in self.control_layouts],
        }


@dataclass(frozen=True, slots=True)
class SessionSpec:
    """Top-level composition of one backend execution or producer role."""

    schema: str
    session_id: str
    backend: str
    runtime_role: str
    assembly: ConfigRef
    workcell: ConfigRef
    bindings: tuple[tuple[str, ConfigRef], ...]
    placements: tuple[tuple[str, str], ...]
    runtime: RuntimeSpec
    dataset_profile: ConfigRef | None = None

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "session") -> Self:
        if not isinstance(value, Mapping):
            raise ValueError(f"{field} must be a mapping")
        schema = require_string(value.get("schema"), field=f"{field}.schema")
        if schema not in {SESSION_SCHEMA_V1, SESSION_SCHEMA_V2}:
            raise ValueError(
                f"{field}.schema must be {SESSION_SCHEMA_V1!r} or {SESSION_SCHEMA_V2!r}"
            )
        expected = {
            "schema",
            "session_id",
            "backend",
            "runtime_role",
            "assembly",
            "workcell",
            "bindings",
            "placements",
            "runtime",
        }
        if schema == SESSION_SCHEMA_V2:
            expected.add("dataset_profile")
        data = require_exact_mapping(
            value,
            expected=frozenset(expected),
            field=field,
        )
        backend = require_string(data["backend"], field=f"{field}.backend")
        if backend not in SUPPORTED_BACKENDS:
            raise ValueError(f"{field}.backend must be one of {sorted(SUPPORTED_BACKENDS)}")
        runtime_role = require_string(data["runtime_role"], field=f"{field}.runtime_role")
        if runtime_role not in SUPPORTED_RUNTIME_ROLES:
            raise ValueError(
                f"{field}.runtime_role must be one of {sorted(SUPPORTED_RUNTIME_ROLES)}"
            )
        bindings = _reference_pairs(data["bindings"], field=f"{field}.bindings")
        if not bindings:
            raise ValueError(f"{field}.bindings must not be empty")
        placements = _placement_pairs(data["placements"], field=f"{field}.placements")
        if not placements:
            raise ValueError(f"{field}.placements must not be empty")
        return cls(
            schema=schema,
            session_id=validate_identifier(data["session_id"], field=f"{field}.session_id"),
            backend=backend,
            runtime_role=runtime_role,
            assembly=ConfigRef.from_mapping(data["assembly"], field=f"{field}.assembly"),
            workcell=ConfigRef.from_mapping(data["workcell"], field=f"{field}.workcell"),
            bindings=bindings,
            placements=placements,
            runtime=RuntimeSpec.from_mapping(data["runtime"], field=f"{field}.runtime"),
            dataset_profile=(
                None
                if schema == SESSION_SCHEMA_V1
                else ConfigRef.from_mapping(
                    data["dataset_profile"],
                    field=f"{field}.dataset_profile",
                )
            ),
        )

    def binding_for(self, instance_id: str) -> ConfigRef:
        for candidate, reference in self.bindings:
            if candidate == instance_id:
                return reference
        raise KeyError(instance_id)

    def mount_for(self, root_instance_id: str) -> str:
        for candidate, mount_id in self.placements:
            if candidate == root_instance_id:
                return mount_id
        raise KeyError(root_instance_id)

    def to_mapping(self) -> dict[str, object]:
        result: dict[str, object] = {
            "schema": self.schema,
            "session_id": self.session_id,
            "backend": self.backend,
            "runtime_role": self.runtime_role,
            "assembly": self.assembly.to_mapping(),
            "workcell": self.workcell.to_mapping(),
            "bindings": {
                instance_id: reference.to_mapping() for instance_id, reference in self.bindings
            },
            "placements": dict(self.placements),
            "runtime": self.runtime.to_mapping(),
        }
        if self.schema == SESSION_SCHEMA_V2:
            if self.dataset_profile is None:
                raise RuntimeError("Session v2 is missing its dataset profile")
            result["dataset_profile"] = self.dataset_profile.to_mapping()
        elif self.dataset_profile is not None:
            raise RuntimeError("Session v1 cannot carry a dataset profile")
        return result


__all__ = [
    "ControlLayoutSpec",
    "RuntimeSpec",
    "SESSION_SCHEMA",
    "SESSION_SCHEMA_V1",
    "SESSION_SCHEMA_V2",
    "SUPPORTED_BACKENDS",
    "SUPPORTED_RUNTIME_ROLES",
    "SessionSpec",
]
