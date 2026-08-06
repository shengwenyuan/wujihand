"""Cross-layer validation and deterministic resolution for Session specs."""

from __future__ import annotations

from collections import Counter
from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import cast

from wujihand.specs import (
    ASSET_MANIFEST_SCHEMA_V1,
    ASSET_MANIFEST_SCHEMA_V2,
    BACKEND_BINDING_SCHEMA_V1,
    BACKEND_BINDING_SCHEMA_V2,
    AssemblySpec,
    AssetManifest,
    BackendBinding,
    DUAL_TELEOPERATION_CONTRACT,
    SessionSpec,
    WorkcellSpec,
    NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT,
    PASSIVE_ASSET_KINDS,
)

from .config_repository import ConfigRepository
from .source_lock import ResolvedArtifact, SourceLock, SourceRecord, sha256_file


RESOLVED_SESSION_SCHEMA = "wujihand.resolved_session.v1"
OverrideScalar = str | int | float | bool | None


@dataclass(frozen=True, slots=True)
class ResolvedInstance:
    """One assembly instance with its manifest, backend binding, and artifacts."""

    instance_id: str
    asset_path: str
    asset: AssetManifest
    binding_path: str
    binding: BackendBinding
    artifact: ResolvedArtifact | None
    collision_artifact: ResolvedArtifact | None
    resource_trees: tuple[ResolvedArtifact, ...]
    namespace: str

    def qualify_backend_name(self, name: str) -> str:
        if self.binding.namespace_policy == "preserve":
            return name
        return f"{self.namespace}:{name}"

    @property
    def effective_root(self) -> str:
        return self.qualify_backend_name(self.binding.root)

    def to_mapping(self) -> dict[str, object]:
        mapping: dict[str, object] = {
            "instance_id": self.instance_id,
            "asset_path": self.asset_path,
            "asset": self.asset.to_mapping(),
            "binding_path": self.binding_path,
            "binding": self.binding.to_mapping(),
            "artifact": (None if self.artifact is None else self.artifact.to_mapping()),
            "resource_trees": [resource.to_mapping() for resource in self.resource_trees],
            "namespace": self.namespace,
            "effective_root": self.effective_root,
            "effective_frame_map": {
                canonical: self.qualify_backend_name(backend_name)
                for canonical, backend_name in self.binding.frame_map
            },
            "effective_group_bindings": {
                group.group_id: {
                    "joints": [self.qualify_backend_name(name) for name in group.joints],
                    "actuators": [self.qualify_backend_name(name) for name in group.actuators],
                }
                for group in self.binding.group_bindings
            },
        }
        if self.binding.schema == BACKEND_BINDING_SCHEMA_V2:
            mapping["collision_artifact"] = (
                None if self.collision_artifact is None else self.collision_artifact.to_mapping()
            )
        return mapping


@dataclass(frozen=True, slots=True)
class ResolvedOverride:
    """Typed override value plus content identity when it names a file."""

    key: str
    value_type: str
    value: OverrideScalar
    file_sha256: str | None

    def to_mapping(self) -> dict[str, object]:
        return {
            "type": self.value_type,
            "value": self.value,
            "file_sha256": self.file_sha256,
        }


@dataclass(frozen=True, slots=True)
class ResolvedSession:
    """Closed immutable composition plus a checkout-independent semantic hash."""

    config_path: str
    session: SessionSpec
    assembly_path: str
    assembly: AssemblySpec
    workcell_path: str
    workcell: WorkcellSpec
    instances: tuple[ResolvedInstance, ...]
    source_records: tuple[SourceRecord, ...]
    referenced_file_hashes: tuple[tuple[str, str], ...]
    overrides: tuple[ResolvedOverride, ...]
    snapshot_json: str
    session_hash: str

    def instance(self, instance_id: str) -> ResolvedInstance:
        for instance in self.instances:
            if instance.instance_id == instance_id:
                return instance
        raise KeyError(instance_id)

    @property
    def snapshot(self) -> Mapping[str, object]:
        """Return a fresh deep copy of the canonical immutable snapshot."""

        value = json.loads(self.snapshot_json)
        if not isinstance(value, Mapping):
            raise RuntimeError("resolved snapshot is not a mapping")
        return cast(Mapping[str, object], value)

    def to_mapping(self) -> dict[str, object]:
        return {
            **self.snapshot,
            "session_hash": self.session_hash,
        }


class SessionResolver:
    """Resolve every Session reference and enforce all five-layer contracts."""

    def __init__(
        self,
        project_root: str | Path,
        *,
        source_lock_path: str | Path = "third_party/sources.lock.yaml",
    ) -> None:
        self.repository = ConfigRepository(project_root)
        self.source_lock = SourceLock.load(self.repository, source_lock_path)

    def resolve(
        self,
        session_path: str | Path,
        *,
        verify_artifacts: bool = False,
        overrides: Mapping[str, str | Path | int | float | bool | None] | None = None,
    ) -> ResolvedSession:
        """Resolve a Session without importing a simulator or device SDK."""

        config_path = self.repository.project_relative(session_path, field="session config")
        session = self.repository.load_session(session_path)
        assembly = self.repository.load_assembly(session.assembly)
        workcell = self.repository.load_workcell(session.workcell)
        assembly_path = session.assembly.path
        workcell_path = session.workcell.path

        instance_ids = tuple(instance.instance_id for instance in assembly.instances)
        binding_instance_ids = tuple(instance_id for instance_id, _ in session.bindings)
        if set(binding_instance_ids) != set(instance_ids):
            raise ValueError(
                "session.bindings must exactly cover assembly instances: "
                f"expected={sorted(instance_ids)}, actual={sorted(binding_instance_ids)}"
            )
        placement_roots = tuple(root for root, _ in session.placements)
        if set(placement_roots) != set(assembly.roots):
            raise ValueError(
                "session.placements must exactly cover assembly roots: "
                f"expected={sorted(assembly.roots)}, actual={sorted(placement_roots)}"
            )
        mount_ids = {mount.mount_id for mount in workcell.mounts}
        missing_mounts = sorted(
            mount_id for _, mount_id in session.placements if mount_id not in mount_ids
        )
        if missing_mounts:
            raise ValueError(
                f"session.placements references unknown workcell mounts: {missing_mounts}"
            )

        resolved_instances: list[ResolvedInstance] = []
        used_source_names: set[str] = set()
        referenced_paths: set[str] = set()
        if session.dataset_profile is not None:
            if session.backend != "isaac" or session.runtime_role != "teleop_consumer":
                raise ValueError("dataset Session v2 requires an Isaac teleop_consumer runtime")
            referenced_paths.add(
                self.repository.validate_profile_reference(session.dataset_profile)
            )
        for instance in assembly.instances:
            asset = self.repository.load_asset(instance.asset)
            binding_ref = session.binding_for(instance.instance_id)
            binding = self.repository.load_binding(binding_ref)
            if binding.asset_id != asset.asset_id:
                raise ValueError(
                    f"binding {binding.binding_id!r} targets asset {binding.asset_id!r}, "
                    f"but instance {instance.instance_id!r} uses {asset.asset_id!r}"
                )
            if binding.asset_revision != asset.revision:
                raise ValueError(
                    f"binding {binding.binding_id!r} targets asset revision "
                    f"{binding.asset_revision!r}, but manifest declares "
                    f"{asset.revision!r}"
                )
            if binding.asset_side != asset.side:
                raise ValueError(
                    f"binding {binding.binding_id!r} targets asset side "
                    f"{binding.asset_side!r}, but manifest declares {asset.side!r}"
                )
            if binding.backend != session.backend:
                raise ValueError(
                    f"binding {binding.binding_id!r} backend {binding.backend!r} "
                    f"does not match session backend {session.backend!r}"
                )
            self._validate_asset_binding(asset, binding)
            project_provenance = self._validate_asset_provenance(asset)
            if project_provenance is not None:
                referenced_paths.add(project_provenance)
            else:
                lock_prefix = f"{self.source_lock.lock_path}#"
                if asset.provenance_source.startswith(lock_prefix):
                    used_source_names.add(asset.provenance_source.removeprefix(lock_prefix))
            if asset.canonical_profile is not None:
                referenced_paths.add(asset.canonical_profile)
            for group in asset.control_groups:
                if group.joint_profile is not None:
                    referenced_paths.add(group.joint_profile)
            if binding.compatibility_profile is not None:
                referenced_paths.add(binding.compatibility_profile)
            if binding.sensor_profile is not None:
                self.repository.load_isaac_camera_profile(binding.sensor_profile)
                referenced_paths.add(binding.sensor_profile)

            artifact = (
                None
                if binding.artifact is None
                else self.source_lock.resolve(
                    binding.artifact,
                    verify=verify_artifacts,
                )
            )
            collision_artifact = (
                None
                if binding.collision_artifact is None
                else self.source_lock.resolve(
                    binding.collision_artifact,
                    verify=verify_artifacts,
                )
            )
            resources = tuple(
                self.source_lock.resolve(
                    resource,
                    tree=True,
                    verify=verify_artifacts,
                )
                for resource in binding.resource_trees
            )
            if artifact is not None:
                used_source_names.add(artifact.source.name)
            if collision_artifact is not None:
                used_source_names.add(collision_artifact.source.name)
            used_source_names.update(resource.source.name for resource in resources)
            resolved_instances.append(
                ResolvedInstance(
                    instance_id=instance.instance_id,
                    asset_path=instance.asset.path,
                    asset=asset,
                    binding_path=binding_ref.path,
                    binding=binding,
                    artifact=artifact,
                    collision_artifact=collision_artifact,
                    resource_trees=resources,
                    namespace=instance.namespace,
                )
            )

        self._validate_backend_symbols(tuple(resolved_instances))

        self._validate_attachments(assembly, tuple(resolved_instances))
        self._validate_control_layouts(session, tuple(resolved_instances))
        self._validate_runtime_contract(session)
        if session.runtime.transport_contract == NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT:
            profile_path = session.runtime.compatibility_profile
            if profile_path is None:
                raise ValueError("native dual teleoperation requires a compatibility profile")
            live_profile = self.repository.load_native_dual_teleoperation_profile(profile_path)
            if live_profile.transport_contract != session.runtime.transport_contract:
                raise ValueError("native dual profile and Session transport contracts differ")
            referenced_paths.add(
                self.repository.validate_profile_reference(live_profile.base_qualification)
            )
        elif session.runtime.transport_contract == DUAL_TELEOPERATION_CONTRACT:
            profile_path = session.runtime.compatibility_profile
            if profile_path is None:
                raise ValueError("dual teleoperation requires a compatibility profile")
            dual_profile = self.repository.load_dual_teleoperation_profile(profile_path)
            if dual_profile.transport_contract != session.runtime.transport_contract:
                raise ValueError("dual teleoperation profile and Session contracts differ")
            referenced_paths.add(
                self.repository.validate_profile_reference(dual_profile.base_qualification)
            )

        if workcell.compatibility_profile is not None:
            referenced_paths.add(workcell.compatibility_profile)
        if session.runtime.compatibility_profile is not None:
            referenced_paths.add(session.runtime.compatibility_profile)
        referenced_file_hashes = tuple(
            (
                path,
                sha256_file(
                    self.repository.resolve_project_path(
                        path, field=f"compatibility/profile {path}"
                    )
                ),
            )
            for path in sorted(referenced_paths)
        )
        source_records = tuple(self.source_lock.record(name) for name in sorted(used_source_names))
        normalized_overrides = tuple(
            _resolve_override(str(key), value, project_root=self.repository.project_root)
            for key, value in sorted((overrides or {}).items(), key=lambda item: str(item[0]))
        )
        snapshot: dict[str, object] = {
            "schema": RESOLVED_SESSION_SCHEMA,
            "config_path": config_path,
            "session": session.to_mapping(),
            "assembly_path": assembly_path,
            "assembly": assembly.to_mapping(),
            "workcell_path": workcell_path,
            "workcell": workcell.to_mapping(),
            "instances": [
                instance.to_mapping()
                for instance in sorted(resolved_instances, key=lambda item: item.instance_id)
            ],
            "source_lock": self.source_lock.lock_path,
            "sources": [record.to_mapping() for record in source_records],
            "referenced_file_hashes": dict(referenced_file_hashes),
            "overrides": {override.key: override.to_mapping() for override in normalized_overrides},
        }
        snapshot_json = json.dumps(
            snapshot,
            ensure_ascii=True,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        session_hash = hashlib.sha256(snapshot_json.encode()).hexdigest()
        return ResolvedSession(
            config_path=config_path,
            session=session,
            assembly_path=assembly_path,
            assembly=assembly,
            workcell_path=workcell_path,
            workcell=workcell,
            instances=tuple(sorted(resolved_instances, key=lambda item: item.instance_id)),
            source_records=source_records,
            referenced_file_hashes=referenced_file_hashes,
            overrides=normalized_overrides,
            snapshot_json=snapshot_json,
            session_hash=session_hash,
        )

    def _validate_asset_binding(self, asset: AssetManifest, binding: BackendBinding) -> None:
        schema_pairs = {
            (ASSET_MANIFEST_SCHEMA_V1, BACKEND_BINDING_SCHEMA_V1),
            (ASSET_MANIFEST_SCHEMA_V2, BACKEND_BINDING_SCHEMA_V2),
        }
        if (asset.schema, binding.schema) not in schema_pairs:
            raise ValueError(
                f"asset {asset.asset_id!r} schema {asset.schema!r} and binding "
                f"{binding.binding_id!r} schema {binding.schema!r} must use matching versions"
            )
        asset_frame_names = {name for _, name in asset.frames}
        binding_frame_names = {name for name, _ in binding.frame_map}
        if binding_frame_names != asset_frame_names:
            raise ValueError(
                f"binding {binding.binding_id!r} frame_map must exactly cover "
                f"asset {asset.asset_id!r} frames"
            )
        asset_group_ids = {group.group_id for group in asset.control_groups}
        binding_group_ids = {group.group_id for group in binding.group_bindings}
        if binding_group_ids != asset_group_ids:
            raise ValueError(
                f"binding {binding.binding_id!r} group_bindings must exactly cover "
                f"asset {asset.asset_id!r} control groups"
            )
        for control_group in asset.control_groups:
            group_binding = binding.group_binding(control_group.group_id)
            if len(group_binding.joints) != control_group.dof_count:
                raise ValueError(
                    f"binding {binding.binding_id!r} group "
                    f"{control_group.group_id!r} exposes "
                    f"{len(group_binding.joints)} joints; asset requires "
                    f"{control_group.dof_count}"
                )
            if (
                binding.backend == "mujoco"
                and len(group_binding.actuators) != control_group.dof_count
            ):
                raise ValueError(
                    f"MuJoCo binding {binding.binding_id!r} group "
                    f"{control_group.group_id!r} requires one actuator per DoF"
                )
        passive = asset.kind in PASSIVE_ASSET_KINDS
        if passive:
            if binding.namespace_policy != "prefix":
                raise ValueError(
                    f"passive binding {binding.binding_id!r} must use prefix namespace policy"
                )
            if binding.loader != "mesh":
                raise ValueError(f"passive binding {binding.binding_id!r} must use the mesh loader")
            if binding.artifact is None or binding.collision_artifact is None:
                raise ValueError(
                    f"passive binding {binding.binding_id!r} requires visual and collision artifacts"
                )
            if asset.kind == "simulated_sensor" and binding.sensor_profile is None:
                raise ValueError(
                    f"simulated sensor binding {binding.binding_id!r} requires a sensor profile"
                )
            if asset.kind == "passive_component" and binding.sensor_profile is not None:
                raise ValueError(
                    f"passive component binding {binding.binding_id!r} must not use a sensor profile"
                )
        elif binding.collision_artifact is not None or binding.sensor_profile is not None:
            raise ValueError(
                f"controlled binding {binding.binding_id!r} must not declare passive representation fields"
            )
        if binding.artifact is not None:
            self.source_lock.resolve(binding.artifact)
        if binding.collision_artifact is not None:
            self.source_lock.resolve(binding.collision_artifact)
        for resource in binding.resource_trees:
            self.source_lock.resolve(resource, tree=True)

    def _validate_asset_provenance(self, asset: AssetManifest) -> str | None:
        lock_prefix = f"{self.source_lock.lock_path}#"
        if asset.provenance_source.startswith(lock_prefix):
            source_name = asset.provenance_source.removeprefix(lock_prefix)
            if not source_name:
                raise ValueError(
                    f"asset {asset.asset_id!r} provenance source is missing its lock ID"
                )
            self.source_lock.record(source_name)
            return None
        if any(record.name == asset.provenance_source for record in self.source_lock.records):
            raise ValueError(
                f"asset {asset.asset_id!r} source-lock provenance must use "
                f"{self.source_lock.lock_path}#<source-id>"
            )
        self.repository.resolve_project_path(
            asset.provenance_source,
            field=f"asset {asset.asset_id} provenance source",
        )
        return asset.provenance_source

    def _validate_attachments(
        self,
        assembly: AssemblySpec,
        instances: tuple[ResolvedInstance, ...],
    ) -> None:
        assets = {instance.instance_id: instance.asset for instance in instances}
        for attachment in assembly.attachments:
            for endpoint_name, endpoint in (
                ("parent", attachment.parent),
                ("child", attachment.child),
            ):
                asset = assets[endpoint.instance]
                frame_names = {name for _, name in asset.frames}
                if endpoint.frame not in frame_names:
                    raise ValueError(
                        f"attachment {attachment.attachment_id!r} {endpoint_name} "
                        f"frame {endpoint.frame!r} is not declared by asset "
                        f"{asset.asset_id!r}"
                    )

    def _validate_control_layouts(
        self,
        session: SessionSpec,
        instances: tuple[ResolvedInstance, ...],
    ) -> None:
        resolved = {instance.instance_id: instance for instance in instances}
        expected_routes = {
            (instance.instance_id, group.group_id)
            for instance in instances
            for group in instance.asset.control_groups
        }
        actual_routes = {
            (layout.instance_id, layout.group_id) for layout in session.runtime.control_layouts
        }
        if actual_routes != expected_routes:
            raise ValueError(
                "session runtime control layouts must exactly cover all asset "
                f"control groups: expected={sorted(expected_routes)}, "
                f"actual={sorted(actual_routes)}"
            )
        for layout in session.runtime.control_layouts:
            instance = resolved.get(layout.instance_id)
            if instance is None:
                raise ValueError(
                    f"control layout references unknown instance {layout.instance_id!r}"
                )
            try:
                group = instance.asset.control_group(layout.group_id)
            except KeyError as exc:
                raise ValueError(
                    f"control layout references unknown group {layout.group_id!r} "
                    f"on instance {layout.instance_id!r}"
                ) from exc
            if group.layout_id != layout.layout_id:
                raise ValueError(
                    f"control layout {layout.instance_id}/{layout.group_id} expects "
                    f"{layout.layout_id!r}, asset declares {group.layout_id!r}"
                )

    @staticmethod
    def _validate_backend_symbols(
        instances: tuple[ResolvedInstance, ...],
    ) -> None:
        for category in ("roots", "frames", "joints", "actuators"):
            owners: dict[str, str] = {}
            collisions: set[str] = set()
            for instance in instances:
                for name in SessionResolver._backend_symbols(instance, category):
                    previous = owners.setdefault(name, instance.instance_id)
                    if previous != instance.instance_id:
                        collisions.add(name)
            if collisions:
                raise ValueError(
                    f"session backend {category} collide across instances: "
                    f"{sorted(collisions)}; use prefix namespace policies"
                )

    @staticmethod
    def _backend_symbols(
        instance: ResolvedInstance,
        category: str,
    ) -> set[str]:
        if category == "roots":
            return {instance.effective_root}
        if category == "frames":
            names = {name for _, name in instance.binding.frame_map}
        elif category == "joints":
            names = {name for group in instance.binding.group_bindings for name in group.joints}
        elif category == "actuators":
            names = {name for group in instance.binding.group_bindings for name in group.actuators}
        else:
            raise ValueError(f"unknown backend symbol category: {category}")
        return {instance.qualify_backend_name(name) for name in names}

    @staticmethod
    def _validate_runtime_contract(session: SessionSpec) -> None:
        transport = session.runtime.transport_contract
        if session.runtime_role in {"teleop_producer", "teleop_consumer"}:
            if transport is None:
                raise ValueError(f"{session.runtime_role} session requires a transport contract")
        elif transport is not None:
            raise ValueError(
                f"{session.runtime_role} session must not declare a transport contract"
            )


def validate_transport_pair(producer: ResolvedSession, consumer: ResolvedSession) -> None:
    """Validate producer/consumer wire and semantic layout compatibility."""

    if producer.session.runtime_role != "teleop_producer":
        raise ValueError("producer session must have runtime_role teleop_producer")
    if consumer.session.runtime_role != "teleop_consumer":
        raise ValueError("consumer session must have runtime_role teleop_consumer")
    producer_contract = producer.session.runtime.transport_contract
    consumer_contract = consumer.session.runtime.transport_contract
    if producer_contract != consumer_contract:
        raise ValueError(
            "teleop transport contracts differ: "
            f"producer={producer_contract!r}, consumer={consumer_contract!r}"
        )
    producer_layouts = _transport_layout_signature(producer)
    consumer_layouts = _transport_layout_signature(consumer)
    if producer_layouts != consumer_layouts:
        raise ValueError("teleop producer and consumer control layouts are incompatible")


def _transport_layout_signature(
    resolved: ResolvedSession,
) -> Counter[tuple[str, str, str, str, str, str, int, str]]:
    signatures: Counter[tuple[str, str, str, str, str, str, int, str]] = Counter()
    for route in resolved.session.runtime.control_layouts:
        try:
            instance = resolved.instance(route.instance_id)
        except KeyError as exc:
            raise ValueError("teleop control layouts reference an unresolved instance") from exc
        group = instance.asset.control_group(route.group_id)
        signatures[
            (
                instance.asset.product,
                instance.asset.revision,
                instance.asset.side,
                group.group_id,
                group.semantic,
                group.layout_id,
                group.dof_count,
                group.command_interface,
            )
        ] += 1
    return signatures


def _resolve_override(
    key: str,
    value: str | Path | int | float | bool | None,
    *,
    project_root: Path,
) -> ResolvedOverride:
    if isinstance(value, Path):
        value_type = "path"
        normalized_value: OverrideScalar = _normalize_path(value, project_root)
    elif value is None:
        value_type = "null"
        normalized_value = None
    elif isinstance(value, bool):
        value_type = "bool"
        normalized_value = value
    elif isinstance(value, int):
        value_type = "int"
        normalized_value = value
    elif isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"override {key!r} must be finite")
        value_type = "float"
        normalized_value = value
    else:
        value_type = "string"
        normalized_value = value
    file_path = _override_file(value, project_root)
    return ResolvedOverride(
        key=key,
        value_type=value_type,
        value=normalized_value,
        file_sha256=None if file_path is None else sha256_file(file_path),
    )


def _normalize_path(value: Path, project_root: Path) -> str:
    candidate = value if value.is_absolute() else project_root / value
    resolved = candidate.resolve()
    try:
        return resolved.relative_to(project_root).as_posix()
    except ValueError:
        return resolved.as_posix()


def _override_file(
    value: str | Path | int | float | bool | None,
    project_root: Path,
) -> Path | None:
    if not isinstance(value, (str, Path)) or not value:
        return None
    raw = Path(value)
    candidate = raw if raw.is_absolute() else project_root / raw
    resolved = candidate.resolve()
    return resolved if resolved.is_file() else None


__all__ = [
    "RESOLVED_SESSION_SCHEMA",
    "ResolvedInstance",
    "ResolvedOverride",
    "ResolvedSession",
    "SessionResolver",
    "validate_transport_pair",
]
