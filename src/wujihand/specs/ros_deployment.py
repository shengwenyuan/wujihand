"""Strict ROS 2 deployment contracts layered above the five asset layers."""

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
)
from .deployment import (
    ControlSourceBindingSpec,
    DeploymentProcessSpec,
    DeploymentSourceSpec,
    DeploymentSpec,
    LocalSourceBindingSpec,
    TrackingSetupSpec,
)


ROS_DEPLOYMENT_SCHEMA = "wujihand.deployment.v2"
ROS_LOCAL_RUNTIME_BINDING_SCHEMA = "wujihand.ros_local_runtime_binding.v2"
ROS_QOS_PROFILE_SCHEMA = "wujihand.ros_qos_profile.v1"

_HISTORY_KINDS = frozenset({"keep_last"})
_RELIABILITY_KINDS = frozenset({"best_effort", "reliable"})
_DURABILITY_KINDS = frozenset({"volatile", "transient_local"})


def _unique(values: Iterable[str], *, field: str) -> None:
    items = tuple(values)
    if len(set(items)) != len(items):
        raise ValueError(f"{field} values must be unique")


def _optional_identifier(value: object, *, field: str) -> str | None:
    if value is None:
        return None
    return validate_identifier(value, field=field)


def _positive_int(value: object, *, field: str, maximum: int) -> int:
    if type(value) is not int or not 1 <= value <= maximum:
        raise ValueError(f"{field} must be an integer in [1, {maximum}]")
    return value


def _optional_positive_int(
    value: object,
    *,
    field: str,
    maximum: int,
) -> int | None:
    if value is None:
        return None
    return _positive_int(value, field=field, maximum=maximum)


@dataclass(frozen=True, slots=True)
class RosRemapSpec:
    source: str
    target: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"source", "target"}),
            field=field,
        )
        return cls(
            source=validate_identifier(data["source"], field=f"{field}.source"),
            target=validate_identifier(data["target"], field=f"{field}.target"),
        )

    def to_mapping(self) -> dict[str, object]:
        return {"source": self.source, "target": self.target}


@dataclass(frozen=True, slots=True)
class RosNodeBindingSpec:
    process_id: str
    node_name: str
    remaps: tuple[RosRemapSpec, ...]

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"process_id", "node_name", "remaps"}),
            field=field,
        )
        remaps = tuple(
            RosRemapSpec.from_mapping(
                item,
                field=f"{field}.remaps[{index}]",
            )
            for index, item in enumerate(
                require_sequence(data["remaps"], field=f"{field}.remaps")
            )
        )
        _unique(
            (remap.source for remap in remaps),
            field=f"{field}.remaps source",
        )
        return cls(
            process_id=validate_identifier(
                data["process_id"],
                field=f"{field}.process_id",
            ),
            node_name=validate_identifier(
                data["node_name"],
                field=f"{field}.node_name",
            ),
            remaps=remaps,
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "process_id": self.process_id,
            "node_name": self.node_name,
            "remaps": [remap.to_mapping() for remap in self.remaps],
        }


@dataclass(frozen=True, slots=True)
class RosDeploymentSpec:
    """Deployment v2 reusing all v1 route/source/tracking validation."""

    core: DeploymentSpec
    root_namespace: str
    interface_set_id: str
    qos_profile: ConfigRef
    node_bindings: tuple[RosNodeBindingSpec, ...]
    execution_owner_process_id: str
    recorder_process_id: str | None

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "ROS deployment",
    ) -> Self:
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
                    "root_namespace",
                    "interface_set_id",
                    "qos_profile",
                    "node_bindings",
                    "execution_owner_process_id",
                    "recorder_process_id",
                }
            ),
            field=field,
        )
        schema = require_string(data["schema"], field=f"{field}.schema")
        if schema != ROS_DEPLOYMENT_SCHEMA:
            raise ValueError(
                f"{field}.schema must be {ROS_DEPLOYMENT_SCHEMA!r}"
            )
        core_mapping = {
            key: data[key]
            for key in (
                "deployment_id",
                "session",
                "local_binding_id",
                "tracking_setup",
                "processes",
                "sources",
                "control_bindings",
                "report_root",
            )
        }
        core_mapping["schema"] = "wujihand.deployment.v1"
        core = DeploymentSpec.from_mapping(core_mapping, field=field)
        nodes = tuple(
            RosNodeBindingSpec.from_mapping(
                item,
                field=f"{field}.node_bindings[{index}]",
            )
            for index, item in enumerate(
                require_sequence(
                    data["node_bindings"],
                    field=f"{field}.node_bindings",
                )
            )
        )
        _unique(
            (node.process_id for node in nodes),
            field=f"{field}.node_bindings process_id",
        )
        _unique(
            (node.node_name for node in nodes),
            field=f"{field}.node_bindings node_name",
        )
        process_ids = {process.process_id for process in core.processes}
        owner = validate_identifier(
            data["execution_owner_process_id"],
            field=f"{field}.execution_owner_process_id",
        )
        if owner not in process_ids:
            raise ValueError(
                f"{field}.execution_owner_process_id references an unknown process"
            )
        recorder = _optional_identifier(
            data["recorder_process_id"],
            field=f"{field}.recorder_process_id",
        )
        if recorder is not None and recorder not in process_ids:
            raise ValueError(
                f"{field}.recorder_process_id references an unknown process"
            )
        expected_node_processes = process_ids - (
            set() if recorder is None else {recorder}
        )
        if {node.process_id for node in nodes} != expected_node_processes:
            raise ValueError(
                f"{field}.node_bindings must cover every non-recorder process"
            )
        if recorder == owner:
            raise ValueError(
                f"{field}.recorder_process_id must not own execution"
            )
        return cls(
            core=core,
            root_namespace=validate_identifier(
                data["root_namespace"],
                field=f"{field}.root_namespace",
            ),
            interface_set_id=validate_identifier(
                data["interface_set_id"],
                field=f"{field}.interface_set_id",
            ),
            qos_profile=ConfigRef.from_mapping(
                data["qos_profile"],
                field=f"{field}.qos_profile",
            ),
            node_bindings=nodes,
            execution_owner_process_id=owner,
            recorder_process_id=recorder,
        )

    @property
    def deployment_id(self) -> str:
        return self.core.deployment_id

    @property
    def session(self) -> ConfigRef:
        return self.core.session

    @property
    def local_binding_id(self) -> str:
        return self.core.local_binding_id

    @property
    def tracking_setup(self) -> TrackingSetupSpec:
        return self.core.tracking_setup

    @property
    def processes(self) -> tuple[DeploymentProcessSpec, ...]:
        return self.core.processes

    @property
    def sources(self) -> tuple[DeploymentSourceSpec, ...]:
        return self.core.sources

    @property
    def control_bindings(self) -> tuple[ControlSourceBindingSpec, ...]:
        return self.core.control_bindings

    @property
    def report_root(self) -> str:
        return self.core.report_root

    def source(self, source_id: str) -> DeploymentSourceSpec:
        return self.core.source(source_id)

    def to_mapping(self) -> dict[str, object]:
        result = self.core.to_mapping()
        result.update(
            {
                "schema": ROS_DEPLOYMENT_SCHEMA,
                "root_namespace": self.root_namespace,
                "interface_set_id": self.interface_set_id,
                "qos_profile": self.qos_profile.to_mapping(),
                "node_bindings": [
                    node.to_mapping() for node in self.node_bindings
                ],
                "execution_owner_process_id": (
                    self.execution_owner_process_id
                ),
                "recorder_process_id": self.recorder_process_id,
            }
        )
        return result


@dataclass(frozen=True, slots=True)
class RosQosPolicySpec:
    channel: str
    history: str
    depth: int
    reliability: str
    durability: str
    deadline_ms: int | None
    lifespan_ms: int | None

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "channel",
                    "history",
                    "depth",
                    "reliability",
                    "durability",
                    "deadline_ms",
                    "lifespan_ms",
                }
            ),
            field=field,
        )
        history = require_string(data["history"], field=f"{field}.history")
        reliability = require_string(
            data["reliability"],
            field=f"{field}.reliability",
        )
        durability = require_string(
            data["durability"],
            field=f"{field}.durability",
        )
        if history not in _HISTORY_KINDS:
            raise ValueError(f"{field}.history must be keep_last")
        if reliability not in _RELIABILITY_KINDS:
            raise ValueError(
                f"{field}.reliability must be best_effort or reliable"
            )
        if durability not in _DURABILITY_KINDS:
            raise ValueError(
                f"{field}.durability must be volatile or transient_local"
            )
        return cls(
            channel=validate_identifier(
                data["channel"],
                field=f"{field}.channel",
            ),
            history=history,
            depth=_positive_int(
                data["depth"],
                field=f"{field}.depth",
                maximum=1000,
            ),
            reliability=reliability,
            durability=durability,
            deadline_ms=_optional_positive_int(
                data["deadline_ms"],
                field=f"{field}.deadline_ms",
                maximum=60_000,
            ),
            lifespan_ms=_optional_positive_int(
                data["lifespan_ms"],
                field=f"{field}.lifespan_ms",
                maximum=60_000,
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "channel": self.channel,
            "history": self.history,
            "depth": self.depth,
            "reliability": self.reliability,
            "durability": self.durability,
            "deadline_ms": self.deadline_ms,
            "lifespan_ms": self.lifespan_ms,
        }


@dataclass(frozen=True, slots=True)
class RosQosProfileSpec:
    schema: str
    profile_id: str
    policies: tuple[RosQosPolicySpec, ...]

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "ROS QoS profile",
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"schema", "profile_id", "policies"}),
            field=field,
        )
        schema = require_string(data["schema"], field=f"{field}.schema")
        if schema != ROS_QOS_PROFILE_SCHEMA:
            raise ValueError(
                f"{field}.schema must be {ROS_QOS_PROFILE_SCHEMA!r}"
            )
        policies = tuple(
            RosQosPolicySpec.from_mapping(
                item,
                field=f"{field}.policies[{index}]",
            )
            for index, item in enumerate(
                require_sequence(
                    data["policies"],
                    field=f"{field}.policies",
                )
            )
        )
        if not policies:
            raise ValueError(f"{field}.policies must not be empty")
        _unique(
            (policy.channel for policy in policies),
            field=f"{field}.policies channel",
        )
        return cls(
            schema=schema,
            profile_id=validate_identifier(
                data["profile_id"],
                field=f"{field}.profile_id",
            ),
            policies=policies,
        )

    def policy(self, channel: str) -> RosQosPolicySpec:
        for policy in self.policies:
            if policy.channel == channel:
                return policy
        raise KeyError(channel)

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "profile_id": self.profile_id,
            "policies": [policy.to_mapping() for policy in self.policies],
        }


@dataclass(frozen=True, slots=True)
class RosLocalProcessBindingSpec:
    process_id: str
    executable: str
    environment_id: str
    setup_scripts: tuple[str, ...]

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "process_id",
                    "executable",
                    "environment_id",
                    "setup_scripts",
                }
            ),
            field=field,
        )
        executable = require_string(
            data["executable"],
            field=f"{field}.executable",
        )
        if not PurePath(executable).is_absolute():
            raise ValueError(f"{field}.executable must be an absolute path")
        scripts = tuple(
            require_string(item, field=f"{field}.setup_scripts[{index}]")
            for index, item in enumerate(
                require_sequence(
                    data["setup_scripts"],
                    field=f"{field}.setup_scripts",
                )
            )
        )
        if any(not PurePath(script).is_absolute() for script in scripts):
            raise ValueError(
                f"{field}.setup_scripts entries must be absolute paths"
            )
        _unique(scripts, field=f"{field}.setup_scripts")
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
            setup_scripts=scripts,
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "process_id": self.process_id,
            "executable": self.executable,
            "environment_id": self.environment_id,
            "setup_scripts": list(self.setup_scripts),
        }


@dataclass(frozen=True, slots=True)
class RosLocalRuntimeBindingSpec:
    schema: str
    binding_id: str
    host_id: str
    ros_domain_id: int
    rmw_implementation: str
    dds_profile: str | None
    processes: tuple[RosLocalProcessBindingSpec, ...]
    sources: tuple[LocalSourceBindingSpec, ...]

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "ROS local runtime binding",
    ) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "schema",
                    "binding_id",
                    "host_id",
                    "ros_domain_id",
                    "rmw_implementation",
                    "dds_profile",
                    "processes",
                    "sources",
                }
            ),
            field=field,
        )
        schema = require_string(data["schema"], field=f"{field}.schema")
        if schema != ROS_LOCAL_RUNTIME_BINDING_SCHEMA:
            raise ValueError(
                f"{field}.schema must be "
                f"{ROS_LOCAL_RUNTIME_BINDING_SCHEMA!r}"
            )
        domain_id = data["ros_domain_id"]
        if type(domain_id) is not int or not 0 <= domain_id <= 232:
            raise ValueError(f"{field}.ros_domain_id must be in [0, 232]")
        processes = tuple(
            RosLocalProcessBindingSpec.from_mapping(
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
                item,
                field=f"{field}.sources[{index}]",
            )
            for index, item in enumerate(
                require_sequence(data["sources"], field=f"{field}.sources")
            )
        )
        if not processes or not sources:
            raise ValueError(
                f"{field}.processes and {field}.sources must not be empty"
            )
        _unique(
            (process.process_id for process in processes),
            field=f"{field}.processes process_id",
        )
        _unique(
            (source.binding_key for source in sources),
            field=f"{field}.sources binding_key",
        )
        raw_dds_profile = data["dds_profile"]
        dds_profile = (
            None
            if raw_dds_profile is None
            else require_string(
                raw_dds_profile,
                field=f"{field}.dds_profile",
            )
        )
        if dds_profile is not None and not PurePath(dds_profile).is_absolute():
            raise ValueError(f"{field}.dds_profile must be an absolute path")
        return cls(
            schema=schema,
            binding_id=validate_identifier(
                data["binding_id"],
                field=f"{field}.binding_id",
            ),
            host_id=validate_identifier(
                data["host_id"],
                field=f"{field}.host_id",
            ),
            ros_domain_id=domain_id,
            rmw_implementation=validate_identifier(
                data["rmw_implementation"],
                field=f"{field}.rmw_implementation",
            ),
            dds_profile=dds_profile,
            processes=processes,
            sources=sources,
        )

    def process(self, process_id: str) -> RosLocalProcessBindingSpec:
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
            "ros_domain_id": self.ros_domain_id,
            "rmw_implementation": self.rmw_implementation,
            "dds_profile": self.dds_profile,
            "processes": [process.to_mapping() for process in self.processes],
            "sources": [source.to_mapping() for source in self.sources],
        }


__all__ = [
    "ROS_DEPLOYMENT_SCHEMA",
    "ROS_LOCAL_RUNTIME_BINDING_SCHEMA",
    "ROS_QOS_PROFILE_SCHEMA",
    "RosDeploymentSpec",
    "RosLocalProcessBindingSpec",
    "RosLocalRuntimeBindingSpec",
    "RosNodeBindingSpec",
    "RosQosPolicySpec",
    "RosQosProfileSpec",
    "RosRemapSpec",
]
