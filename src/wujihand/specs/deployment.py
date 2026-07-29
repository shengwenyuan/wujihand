"""Immutable runtime-deployment specifications outside the five asset layers."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import PurePath
from typing import Self

from .common import (
    ConfigRef,
    require_exact_mapping,
    require_sequence,
    require_string,
    validate_identifier,
    validate_project_reference,
)


DEPLOYMENT_SCHEMA = "wujihand.deployment.v1"
LOCAL_DEVICE_BINDING_SCHEMA = "wujihand.local_device_binding.v1"

SUPPORTED_PROCESS_LIFECYCLES = frozenset({"in_process", "managed"})
SUPPORTED_SOURCE_KINDS = frozenset(
    {
        "vive_tracker",
        "wuji_glove",
        "arm_hold_fixture",
        "hand_rest_fixture",
    }
)
SUPPORTED_DEPLOYMENT_SIDES = frozenset({"left", "right"})
SUPPORTED_TRACKING_QUALIFICATION_STATES = frozenset({"pending", "qualified"})
LIVE_SOURCE_KINDS = frozenset({"vive_tracker", "wuji_glove"})
FIXTURE_SOURCE_KINDS = SUPPORTED_SOURCE_KINDS - LIVE_SOURCE_KINDS


def _identifier_sequence(value: object, *, field: str) -> tuple[str, ...]:
    items = tuple(
        validate_identifier(item, field=f"{field}[{index}]")
        for index, item in enumerate(require_sequence(value, field=field))
    )
    if len(set(items)) != len(items):
        raise ValueError(f"{field} values must be unique")
    return items


def _bounded_string(value: object, *, field: str) -> str:
    text = require_string(value, field=field)
    if len(text) > 512:
        raise ValueError(f"{field} must contain at most 512 characters")
    return text


@dataclass(frozen=True, slots=True)
class DeploymentProcessSpec:
    """One registry-backed process owned by a deployment launcher."""

    process_id: str
    component_id: str
    lifecycle: str
    depends_on: tuple[str, ...]

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "process") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {"process_id", "component_id", "lifecycle", "depends_on"}
            ),
            field=field,
        )
        process_id = validate_identifier(
            data["process_id"], field=f"{field}.process_id"
        )
        lifecycle = require_string(data["lifecycle"], field=f"{field}.lifecycle")
        if lifecycle not in SUPPORTED_PROCESS_LIFECYCLES:
            raise ValueError(
                f"{field}.lifecycle must be one of "
                f"{sorted(SUPPORTED_PROCESS_LIFECYCLES)}"
            )
        depends_on = _identifier_sequence(
            data["depends_on"], field=f"{field}.depends_on"
        )
        if process_id in depends_on:
            raise ValueError(f"{field}.depends_on must not contain its own process_id")
        return cls(
            process_id=process_id,
            component_id=validate_identifier(
                data["component_id"], field=f"{field}.component_id"
            ),
            lifecycle=lifecycle,
            depends_on=depends_on,
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "process_id": self.process_id,
            "component_id": self.component_id,
            "lifecycle": self.lifecycle,
            "depends_on": list(self.depends_on),
        }


@dataclass(frozen=True, slots=True)
class DeploymentSourceSpec:
    """Bind one canonical live or fixture source to a runtime process."""

    source_id: str
    kind: str
    side: str
    logical_role: str
    process_id: str
    local_binding_key: str | None

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "source") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "source_id",
                    "kind",
                    "side",
                    "logical_role",
                    "process_id",
                    "local_binding_key",
                }
            ),
            field=field,
        )
        kind = require_string(data["kind"], field=f"{field}.kind")
        if kind not in SUPPORTED_SOURCE_KINDS:
            raise ValueError(
                f"{field}.kind must be one of {sorted(SUPPORTED_SOURCE_KINDS)}"
            )
        side = require_string(data["side"], field=f"{field}.side")
        if side not in SUPPORTED_DEPLOYMENT_SIDES:
            raise ValueError(
                f"{field}.side must be one of {sorted(SUPPORTED_DEPLOYMENT_SIDES)}"
            )
        raw_binding = data["local_binding_key"]
        local_binding_key = (
            None
            if raw_binding is None
            else validate_identifier(
                raw_binding, field=f"{field}.local_binding_key"
            )
        )
        if kind in LIVE_SOURCE_KINDS and local_binding_key is None:
            raise ValueError(f"{field}.local_binding_key is required for live sources")
        if kind in FIXTURE_SOURCE_KINDS and local_binding_key is not None:
            raise ValueError(f"{field}.local_binding_key must be null for fixture sources")
        return cls(
            source_id=validate_identifier(
                data["source_id"], field=f"{field}.source_id"
            ),
            kind=kind,
            side=side,
            logical_role=validate_identifier(
                data["logical_role"], field=f"{field}.logical_role"
            ),
            process_id=validate_identifier(
                data["process_id"], field=f"{field}.process_id"
            ),
            local_binding_key=local_binding_key,
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "source_id": self.source_id,
            "kind": self.kind,
            "side": self.side,
            "logical_role": self.logical_role,
            "process_id": self.process_id,
            "local_binding_key": self.local_binding_key,
        }


@dataclass(frozen=True, slots=True)
class ControlSourceBindingSpec:
    """Bind one source to one control route already declared by Session."""

    instance_id: str
    group_id: str
    source_id: str

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "control_binding",
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"instance_id", "group_id", "source_id"}),
            field=field,
        )
        return cls(
            instance_id=validate_identifier(
                data["instance_id"], field=f"{field}.instance_id"
            ),
            group_id=validate_identifier(
                data["group_id"], field=f"{field}.group_id"
            ),
            source_id=validate_identifier(
                data["source_id"], field=f"{field}.source_id"
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "instance_id": self.instance_id,
            "group_id": self.group_id,
            "source_id": self.source_id,
        }


@dataclass(frozen=True, slots=True)
class TrackingSetupSpec:
    """Versioned shared tracking setup plus its mapping calibration artifact."""

    setup_revision: str
    tracking_frame: str
    qualification_status: str
    mapping: ConfigRef

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "tracking_setup",
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "setup_revision",
                    "tracking_frame",
                    "qualification_status",
                    "mapping",
                }
            ),
            field=field,
        )
        status = require_string(
            data["qualification_status"],
            field=f"{field}.qualification_status",
        )
        if status not in SUPPORTED_TRACKING_QUALIFICATION_STATES:
            raise ValueError(
                f"{field}.qualification_status must be one of "
                f"{sorted(SUPPORTED_TRACKING_QUALIFICATION_STATES)}"
            )
        return cls(
            setup_revision=validate_identifier(
                data["setup_revision"], field=f"{field}.setup_revision"
            ),
            tracking_frame=validate_identifier(
                data["tracking_frame"], field=f"{field}.tracking_frame"
            ),
            qualification_status=status,
            mapping=ConfigRef.from_mapping(
                data["mapping"], field=f"{field}.mapping"
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "setup_revision": self.setup_revision,
            "tracking_frame": self.tracking_frame,
            "qualification_status": self.qualification_status,
            "mapping": self.mapping.to_mapping(),
        }


@dataclass(frozen=True, slots=True)
class DeploymentSpec:
    """One explicit runtime root that references exactly one five-layer Session."""

    schema: str
    deployment_id: str
    session: ConfigRef
    local_binding_id: str
    tracking_setup: TrackingSetupSpec
    processes: tuple[DeploymentProcessSpec, ...]
    sources: tuple[DeploymentSourceSpec, ...]
    control_bindings: tuple[ControlSourceBindingSpec, ...]
    report_root: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "deployment") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "schema",
                    "deployment_id",
                    "session",
                    "local_binding_id",
                    "tracking_setup",
                    "processes",
                    "sources",
                    "control_bindings",
                    "report_root",
                }
            ),
            field=field,
        )
        schema = require_string(data["schema"], field=f"{field}.schema")
        if schema != DEPLOYMENT_SCHEMA:
            raise ValueError(f"{field}.schema must be {DEPLOYMENT_SCHEMA!r}")
        processes = tuple(
            DeploymentProcessSpec.from_mapping(
                item, field=f"{field}.processes[{index}]"
            )
            for index, item in enumerate(
                require_sequence(data["processes"], field=f"{field}.processes")
            )
        )
        sources = tuple(
            DeploymentSourceSpec.from_mapping(
                item, field=f"{field}.sources[{index}]"
            )
            for index, item in enumerate(
                require_sequence(data["sources"], field=f"{field}.sources")
            )
        )
        control_bindings = tuple(
            ControlSourceBindingSpec.from_mapping(
                item, field=f"{field}.control_bindings[{index}]"
            )
            for index, item in enumerate(
                require_sequence(
                    data["control_bindings"],
                    field=f"{field}.control_bindings",
                )
            )
        )
        if not processes:
            raise ValueError(f"{field}.processes must not be empty")
        if not sources:
            raise ValueError(f"{field}.sources must not be empty")
        if not control_bindings:
            raise ValueError(f"{field}.control_bindings must not be empty")
        _validate_unique(
            (process.process_id for process in processes),
            field=f"{field}.processes process_id",
        )
        _validate_unique(
            (source.source_id for source in sources),
            field=f"{field}.sources source_id",
        )
        _validate_unique(
            (
                source.local_binding_key
                for source in sources
                if source.local_binding_key is not None
            ),
            field=f"{field}.sources local_binding_key",
        )
        _validate_unique(
            (
                f"{binding.instance_id}/{binding.group_id}"
                for binding in control_bindings
            ),
            field=f"{field}.control_bindings route",
        )
        process_ids = {process.process_id for process in processes}
        for process in processes:
            unknown = sorted(set(process.depends_on) - process_ids)
            if unknown:
                raise ValueError(
                    f"{field}.processes {process.process_id!r} depends on "
                    f"unknown processes: {unknown}"
                )
        _validate_process_dag(processes, field=f"{field}.processes")
        unknown_processes = sorted(
            {source.process_id for source in sources} - process_ids
        )
        if unknown_processes:
            raise ValueError(
                f"{field}.sources reference unknown processes: {unknown_processes}"
            )
        source_ids = {source.source_id for source in sources}
        bound_source_ids = tuple(binding.source_id for binding in control_bindings)
        if len(set(bound_source_ids)) != len(bound_source_ids):
            raise ValueError(
                f"{field}.control_bindings must bind each source exactly once"
            )
        if set(bound_source_ids) != source_ids:
            raise ValueError(
                f"{field}.control_bindings must exactly cover deployment sources"
            )
        if not any(source.kind == "vive_tracker" for source in sources):
            raise ValueError(f"{field}.sources must include at least one VIVE Tracker")
        if not any(source.kind == "wuji_glove" for source in sources):
            raise ValueError(f"{field}.sources must include at least one Wuji Glove")
        return cls(
            schema=schema,
            deployment_id=validate_identifier(
                data["deployment_id"], field=f"{field}.deployment_id"
            ),
            session=ConfigRef.from_mapping(
                data["session"], field=f"{field}.session"
            ),
            local_binding_id=validate_identifier(
                data["local_binding_id"], field=f"{field}.local_binding_id"
            ),
            tracking_setup=TrackingSetupSpec.from_mapping(
                data["tracking_setup"], field=f"{field}.tracking_setup"
            ),
            processes=processes,
            sources=sources,
            control_bindings=control_bindings,
            report_root=validate_project_reference(
                data["report_root"], field=f"{field}.report_root"
            ),
        )

    def source(self, source_id: str) -> DeploymentSourceSpec:
        for source in self.sources:
            if source.source_id == source_id:
                return source
        raise KeyError(source_id)

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "deployment_id": self.deployment_id,
            "session": self.session.to_mapping(),
            "local_binding_id": self.local_binding_id,
            "tracking_setup": self.tracking_setup.to_mapping(),
            "processes": [process.to_mapping() for process in self.processes],
            "sources": [source.to_mapping() for source in self.sources],
            "control_bindings": [
                binding.to_mapping() for binding in self.control_bindings
            ],
            "report_root": self.report_root,
        }


@dataclass(frozen=True, slots=True)
class LocalSourceBindingSpec:
    """One ignored, host-local device identity and endpoint binding."""

    binding_key: str
    source_kind: str
    device_identity: str
    endpoint: str
    calibration_id: str

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "local_source_binding",
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "binding_key",
                    "source_kind",
                    "device_identity",
                    "endpoint",
                    "calibration_id",
                }
            ),
            field=field,
        )
        source_kind = require_string(
            data["source_kind"], field=f"{field}.source_kind"
        )
        if source_kind not in LIVE_SOURCE_KINDS:
            raise ValueError(
                f"{field}.source_kind must be one of {sorted(LIVE_SOURCE_KINDS)}"
            )
        return cls(
            binding_key=validate_identifier(
                data["binding_key"], field=f"{field}.binding_key"
            ),
            source_kind=source_kind,
            device_identity=_bounded_string(
                data["device_identity"], field=f"{field}.device_identity"
            ),
            endpoint=_bounded_string(data["endpoint"], field=f"{field}.endpoint"),
            calibration_id=validate_identifier(
                data["calibration_id"], field=f"{field}.calibration_id"
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "binding_key": self.binding_key,
            "source_kind": self.source_kind,
            "device_identity": self.device_identity,
            "endpoint": self.endpoint,
            "calibration_id": self.calibration_id,
        }


@dataclass(frozen=True, slots=True)
class LocalProcessBindingSpec:
    """One host-local executable selected for a managed process."""

    process_id: str
    executable: str
    environment_id: str

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "local_process_binding",
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {"process_id", "executable", "environment_id"}
            ),
            field=field,
        )
        executable = _bounded_string(
            data["executable"],
            field=f"{field}.executable",
        )
        if not PurePath(executable).is_absolute():
            raise ValueError(f"{field}.executable must be an absolute path")
        return cls(
            process_id=validate_identifier(
                data["process_id"],
                field=f"{field}.process_id",
            ),
            executable=executable,
            environment_id=validate_identifier(
                data["environment_id"],
                field=f"{field}.environment_id",
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "process_id": self.process_id,
            "executable": self.executable,
            "environment_id": self.environment_id,
        }


@dataclass(frozen=True, slots=True)
class LocalDeviceBindingSpec:
    """Host-local identities kept outside committed DeploymentSpec files."""

    schema: str
    binding_id: str
    host_id: str
    processes: tuple[LocalProcessBindingSpec, ...]
    sources: tuple[LocalSourceBindingSpec, ...]

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "local_device_binding",
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {"schema", "binding_id", "host_id", "processes", "sources"}
            ),
            field=field,
        )
        schema = require_string(data["schema"], field=f"{field}.schema")
        if schema != LOCAL_DEVICE_BINDING_SCHEMA:
            raise ValueError(
                f"{field}.schema must be {LOCAL_DEVICE_BINDING_SCHEMA!r}"
            )
        processes = tuple(
            LocalProcessBindingSpec.from_mapping(
                item,
                field=f"{field}.processes[{index}]",
            )
            for index, item in enumerate(
                require_sequence(
                    data["processes"],
                    field=f"{field}.processes",
                )
            )
        )
        sources = tuple(
            LocalSourceBindingSpec.from_mapping(
                item, field=f"{field}.sources[{index}]"
            )
            for index, item in enumerate(
                require_sequence(data["sources"], field=f"{field}.sources")
            )
        )
        if not sources:
            raise ValueError(f"{field}.sources must not be empty")
        _validate_unique(
            (process.process_id for process in processes),
            field=f"{field}.processes process_id",
        )
        _validate_unique(
            (source.binding_key for source in sources),
            field=f"{field}.sources binding_key",
        )
        return cls(
            schema=schema,
            binding_id=validate_identifier(
                data["binding_id"], field=f"{field}.binding_id"
            ),
            host_id=validate_identifier(data["host_id"], field=f"{field}.host_id"),
            processes=processes,
            sources=sources,
        )

    def process(self, process_id: str) -> LocalProcessBindingSpec:
        for process in self.processes:
            if process.process_id == process_id:
                return process
        raise KeyError(process_id)

    def source(self, binding_key: str) -> LocalSourceBindingSpec:
        for source in self.sources:
            if source.binding_key == binding_key:
                return source
        raise KeyError(binding_key)

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "binding_id": self.binding_id,
            "host_id": self.host_id,
            "processes": [process.to_mapping() for process in self.processes],
            "sources": [source.to_mapping() for source in self.sources],
        }


def _validate_unique(values: Iterable[str], *, field: str) -> None:
    items = tuple(values)
    if len(set(items)) != len(items):
        raise ValueError(f"{field} values must be unique")


def _validate_process_dag(
    processes: tuple[DeploymentProcessSpec, ...],
    *,
    field: str,
) -> None:
    dependencies = {
        process.process_id: process.depends_on for process in processes
    }
    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(process_id: str) -> None:
        if process_id in visiting:
            raise ValueError(f"{field} dependency graph must be acyclic")
        if process_id in visited:
            return
        visiting.add(process_id)
        for dependency in dependencies[process_id]:
            visit(dependency)
        visiting.remove(process_id)
        visited.add(process_id)

    for process_id in dependencies:
        visit(process_id)


__all__ = [
    "DEPLOYMENT_SCHEMA",
    "FIXTURE_SOURCE_KINDS",
    "LIVE_SOURCE_KINDS",
    "LOCAL_DEVICE_BINDING_SCHEMA",
    "ControlSourceBindingSpec",
    "DeploymentProcessSpec",
    "DeploymentSourceSpec",
    "DeploymentSpec",
    "LocalDeviceBindingSpec",
    "LocalProcessBindingSpec",
    "LocalSourceBindingSpec",
    "TrackingSetupSpec",
]
