"""Resolve one runtime DeploymentSpec around one closed five-layer Session."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import cast

from wujihand.adapters.storage import (
    TrackerWorkcellMapping,
    load_tracker_workcell_mapping,
)
from wujihand.specs import (
    DeploymentProcessSpec,
    DeploymentSourceSpec,
    DeploymentSpec,
    LocalDeviceBindingSpec,
    LocalProcessBindingSpec,
    LocalSourceBindingSpec,
)

from .config_repository import ConfigRepository
from .session_resolver import ResolvedSession, SessionResolver
from .source_lock import sha256_file


RESOLVED_DEPLOYMENT_SCHEMA = "wujihand.resolved_deployment.v1"

_ALLOWED_SOURCE_KINDS_BY_ASSET_KIND = {
    "robot_arm": frozenset({"vive_tracker", "arm_hold_fixture"}),
    "robot_hand": frozenset({"wuji_glove", "hand_rest_fixture"}),
}


@dataclass(frozen=True, slots=True)
class ResolvedDeploymentSource:
    """One canonical source plus its optional host-local live binding."""

    source: DeploymentSourceSpec
    local_binding: LocalSourceBindingSpec | None

    def redacted_mapping(self) -> dict[str, object]:
        local = self.local_binding
        return {
            "source": self.source.to_mapping(),
            "local_binding": (
                None
                if local is None
                else {
                    "binding_key": local.binding_key,
                    "source_kind": local.source_kind,
                    "device_identity_sha256": _sha256_text(local.device_identity),
                    "endpoint_sha256": _sha256_text(local.endpoint),
                    "calibration_id": local.calibration_id,
                }
            ),
        }


@dataclass(frozen=True, slots=True)
class ResolvedDeploymentProcess:
    """One deployment process plus its optional host-local executable."""

    process: DeploymentProcessSpec
    local_binding: LocalProcessBindingSpec | None

    def redacted_mapping(self) -> dict[str, object]:
        local = self.local_binding
        return {
            "process": self.process.to_mapping(),
            "local_binding": (
                None
                if local is None
                else {
                    "process_id": local.process_id,
                    "executable_sha256": _sha256_text(local.executable),
                    "environment_id": local.environment_id,
                }
            ),
        }


@dataclass(frozen=True, slots=True)
class ResolvedDeployment:
    """Resolved deployment with raw local bindings kept out of report snapshots."""

    config_path: str
    deployment: DeploymentSpec
    session: ResolvedSession
    mapping_path: str
    mapping: TrackerWorkcellMapping
    mapping_sha256: str
    local_binding_id: str
    local_host_id: str
    processes: tuple[ResolvedDeploymentProcess, ...]
    sources: tuple[ResolvedDeploymentSource, ...]
    snapshot_json: str
    deployment_hash: str
    local_binding_hash: str

    @property
    def tracking_qualified(self) -> bool:
        return self.deployment.tracking_setup.qualification_status == "qualified"

    def source(self, source_id: str) -> ResolvedDeploymentSource:
        for source in self.sources:
            if source.source.source_id == source_id:
                return source
        raise KeyError(source_id)

    def process(self, process_id: str) -> ResolvedDeploymentProcess:
        for process in self.processes:
            if process.process.process_id == process_id:
                return process
        raise KeyError(process_id)

    def to_mapping(self) -> dict[str, object]:
        value = json.loads(self.snapshot_json)
        if not isinstance(value, dict):
            raise RuntimeError("resolved deployment snapshot is not a mapping")
        return {
            **cast(dict[str, object], value),
            "deployment_hash": self.deployment_hash,
            "local_binding_hash": self.local_binding_hash,
        }


class DeploymentResolver:
    """Close Deployment, Session, mapping and host-local device references."""

    def __init__(self, project_root: str | Path) -> None:
        self.repository = ConfigRepository(project_root)
        self.session_resolver = SessionResolver(project_root)

    def resolve(
        self,
        deployment_path: str | Path,
        *,
        local_binding: LocalDeviceBindingSpec | str | Path,
        verify_artifacts: bool = False,
    ) -> ResolvedDeployment:
        config_path = self.repository.project_relative(
            deployment_path,
            field="deployment config",
        )
        deployment = self.repository.load_deployment(deployment_path)

        # Loading the ConfigRef first enforces its expected stable Session ID.
        self.repository.load_session(deployment.session)
        session = self.session_resolver.resolve(
            deployment.session.path,
            verify_artifacts=verify_artifacts,
        )

        mapping_path = deployment.tracking_setup.mapping.path
        mapping_file = self.repository.resolve_project_path(
            mapping_path,
            field="Tracker mapping calibration",
        )
        mapping = load_tracker_workcell_mapping(mapping_file)
        if mapping.mapping_id != deployment.tracking_setup.mapping.expected_id:
            raise ValueError(
                "tracking mapping reference expected "
                f"{deployment.tracking_setup.mapping.expected_id!r}, "
                f"loaded {mapping.mapping_id!r}"
            )
        if mapping.tracking_frame != deployment.tracking_setup.tracking_frame:
            raise ValueError(
                "tracking mapping frame must match Deployment tracking setup"
            )
        if mapping.workcell_frame != session.workcell.world_frame:
            raise ValueError(
                "tracking mapping workcell frame must match Session Workcell world frame"
            )

        self._validate_control_bindings(deployment, session)
        local = (
            local_binding
            if isinstance(local_binding, LocalDeviceBindingSpec)
            else self.repository.load_local_device_binding(local_binding)
        )
        if local.binding_id != deployment.local_binding_id:
            raise ValueError(
                f"deployment expects local binding {deployment.local_binding_id!r}, "
                f"loaded {local.binding_id!r}"
            )
        processes = self._resolve_processes(deployment, local)
        sources = self._resolve_sources(deployment, local)

        mapping_sha256 = sha256_file(mapping_file)
        deployment_payload = {
            "schema": RESOLVED_DEPLOYMENT_SCHEMA,
            "config_path": config_path,
            "deployment": deployment.to_mapping(),
            "mapping_path": mapping_path,
            "mapping_sha256": mapping_sha256,
        }
        deployment_json = _canonical_json(deployment_payload)
        deployment_hash = hashlib.sha256(deployment_json.encode()).hexdigest()

        local_payload = {
            "binding_id": local.binding_id,
            "host_id": local.host_id,
            "processes": [
                process.local_binding.to_mapping()
                for process in processes
                if process.local_binding is not None
            ],
            "sources": [
                {
                    "source_id": source.source.source_id,
                    "binding": source.local_binding.to_mapping(),
                }
                for source in sources
                if source.local_binding is not None
            ],
        }
        local_binding_hash = hashlib.sha256(
            _canonical_json(local_payload).encode()
        ).hexdigest()
        snapshot = {
            **deployment_payload,
            "session_hash": session.session_hash,
            "local_binding": {
                "binding_id": local.binding_id,
                "host_id": local.host_id,
                "processes": [
                    process.redacted_mapping() for process in processes
                ],
                "sources": [source.redacted_mapping() for source in sources],
            },
        }
        return ResolvedDeployment(
            config_path=config_path,
            deployment=deployment,
            session=session,
            mapping_path=mapping_path,
            mapping=mapping,
            mapping_sha256=mapping_sha256,
            local_binding_id=local.binding_id,
            local_host_id=local.host_id,
            processes=processes,
            sources=sources,
            snapshot_json=_canonical_json(snapshot),
            deployment_hash=deployment_hash,
            local_binding_hash=local_binding_hash,
        )

    @staticmethod
    def _validate_control_bindings(
        deployment: DeploymentSpec,
        session: ResolvedSession,
    ) -> None:
        expected_routes = {
            (layout.instance_id, layout.group_id)
            for layout in session.session.runtime.control_layouts
        }
        actual_routes = {
            (binding.instance_id, binding.group_id)
            for binding in deployment.control_bindings
        }
        if actual_routes != expected_routes:
            raise ValueError(
                "deployment control bindings must exactly cover Session routes: "
                f"expected={sorted(expected_routes)}, actual={sorted(actual_routes)}"
            )
        for binding in deployment.control_bindings:
            source = deployment.source(binding.source_id)
            instance = session.instance(binding.instance_id)
            try:
                instance.asset.control_group(binding.group_id)
            except KeyError as exc:
                raise ValueError(
                    f"deployment route references unknown control group "
                    f"{binding.instance_id}/{binding.group_id}"
                ) from exc
            allowed = _ALLOWED_SOURCE_KINDS_BY_ASSET_KIND.get(instance.asset.kind)
            if allowed is None or source.kind not in allowed:
                raise ValueError(
                    f"source kind {source.kind!r} cannot command asset kind "
                    f"{instance.asset.kind!r}"
                )
            target_side = _resolved_instance_side(session, binding.instance_id)
            if source.side != target_side:
                raise ValueError(
                    f"source side {source.side!r} does not match target side "
                    f"{target_side!r} for {binding.instance_id}/{binding.group_id}"
                )

    @staticmethod
    def _resolve_processes(
        deployment: DeploymentSpec,
        local: LocalDeviceBindingSpec,
    ) -> tuple[ResolvedDeploymentProcess, ...]:
        managed_ids = {
            process.process_id
            for process in deployment.processes
            if process.lifecycle == "managed"
        }
        local_ids = {process.process_id for process in local.processes}
        if local_ids != managed_ids:
            raise ValueError(
                "local process bindings must exactly cover managed processes: "
                f"expected={sorted(managed_ids)}, actual={sorted(local_ids)}"
            )
        return tuple(
            ResolvedDeploymentProcess(
                process=process,
                local_binding=(
                    local.process(process.process_id)
                    if process.lifecycle == "managed"
                    else None
                ),
            )
            for process in deployment.processes
        )

    @staticmethod
    def _resolve_sources(
        deployment: DeploymentSpec,
        local: LocalDeviceBindingSpec,
    ) -> tuple[ResolvedDeploymentSource, ...]:
        resolved: list[ResolvedDeploymentSource] = []
        for source in deployment.sources:
            if source.local_binding_key is None:
                resolved.append(
                    ResolvedDeploymentSource(source=source, local_binding=None)
                )
                continue
            try:
                binding = local.source(source.local_binding_key)
            except KeyError as exc:
                raise ValueError(
                    f"local binding is missing key {source.local_binding_key!r}"
                ) from exc
            if binding.source_kind != source.kind:
                raise ValueError(
                    f"local binding {binding.binding_key!r} kind "
                    f"{binding.source_kind!r} does not match source {source.kind!r}"
                )
            resolved.append(
                ResolvedDeploymentSource(source=source, local_binding=binding)
            )
        return tuple(sorted(resolved, key=lambda item: item.source.source_id))


def _canonical_json(value: object) -> str:
    return json.dumps(
        value,
        ensure_ascii=True,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def _sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode()).hexdigest()


def _resolved_instance_side(
    session: ResolvedSession,
    instance_id: str,
) -> str:
    instance = session.instance(instance_id)
    if instance.asset.side in {"left", "right"}:
        return instance.asset.side
    attached_sides = {
        session.instance(attachment.child.instance).asset.side
        for attachment in session.assembly.attachments
        if attachment.parent.instance == instance_id
        and session.instance(attachment.child.instance).asset.side in {"left", "right"}
    }
    if len(attached_sides) != 1:
        raise ValueError(
            f"cannot derive one operational side for deployment target {instance_id!r}"
        )
    return next(iter(attached_sides))


__all__ = [
    "RESOLVED_DEPLOYMENT_SCHEMA",
    "DeploymentResolver",
    "ResolvedDeployment",
    "ResolvedDeploymentProcess",
    "ResolvedDeploymentSource",
]
