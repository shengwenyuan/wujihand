"""Resolve ROS deployment v2 without importing ROS or device SDKs."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

from wujihand.adapters.storage import (
    TrackerWorkcellMapping,
    load_tracker_workcell_mapping,
)
from wujihand.specs import (
    DualTeleoperationProfile,
    RosDeploymentSpec,
    RosLocalRuntimeBindingSpec,
    RosQosProfileSpec,
)

from .config_repository import ConfigRepository
from .dual_route_plan import (
    DualTeleoperationRoutePlan,
    build_dual_teleoperation_route_plan,
)
from .session_resolver import ResolvedSession, SessionResolver
from .source_lock import sha256_file


@dataclass(frozen=True, slots=True)
class ResolvedRosDeployment:
    config_path: str
    deployment: RosDeploymentSpec
    local_binding: RosLocalRuntimeBindingSpec
    session: ResolvedSession
    qos_profile: RosQosProfileSpec
    control_profile: DualTeleoperationProfile
    mapping: TrackerWorkcellMapping
    mapping_path: str
    mapping_sha256: str
    route_plan: DualTeleoperationRoutePlan
    deployment_hash: str
    local_binding_hash: str


class RosDeploymentResolver:
    """Close ROS graph, five layers, QoS, mapping and local runtime."""

    def __init__(self, project_root: str | Path) -> None:
        self.repository = ConfigRepository(project_root)
        self.session_resolver = SessionResolver(project_root)

    def resolve(
        self,
        deployment_path: str | Path,
        *,
        local_binding: RosLocalRuntimeBindingSpec | str | Path,
        verify_artifacts: bool = False,
    ) -> ResolvedRosDeployment:
        config_path = self.repository.project_relative(
            deployment_path,
            field="ROS deployment config",
        )
        deployment = self.repository.load_ros_deployment(deployment_path)
        self.repository.load_session(deployment.session)
        session = self.session_resolver.resolve(
            deployment.session.path,
            verify_artifacts=verify_artifacts,
        )
        local = (
            local_binding
            if isinstance(local_binding, RosLocalRuntimeBindingSpec)
            else self.repository.load_ros_local_runtime_binding(local_binding)
        )
        if local.binding_id != deployment.local_binding_id:
            raise ValueError(
                "ROS deployment and local runtime binding IDs differ"
            )
        process_ids = {process.process_id for process in deployment.processes}
        local_process_ids = {
            process.process_id for process in local.processes
        }
        if not process_ids.issubset(local_process_ids):
            raise ValueError(
                "ROS local runtime binding is missing deployment processes"
            )
        live_binding_keys = {
            source.local_binding_key
            for source in deployment.sources
            if source.local_binding_key is not None
        }
        local_sources = {
            source.binding_key: source for source in local.sources
        }
        if not live_binding_keys.issubset(local_sources):
            raise ValueError(
                "ROS local runtime binding is missing live source bindings"
            )
        self._validate_routes(deployment, session)
        route_plan = build_dual_teleoperation_route_plan(
            deployment,
            local_sources=local_sources,
        )
        qos = self.repository.load_ros_qos_profile(
            deployment.qos_profile
        )
        compatibility_profile = (
            session.session.runtime.compatibility_profile
        )
        if compatibility_profile is None:
            raise ValueError(
                "ROS Session is missing its dual teleoperation profile"
            )
        control_profile = (
            self.repository.load_dual_teleoperation_profile(
                compatibility_profile
            )
        )
        mapping_path = deployment.tracking_setup.mapping.path
        mapping_file = self.repository.resolve_project_path(
            mapping_path,
            field="ROS Tracker mapping calibration",
        )
        mapping = load_tracker_workcell_mapping(mapping_file)
        if (
            mapping.mapping_id
            != deployment.tracking_setup.mapping.expected_id
        ):
            raise ValueError("ROS tracking mapping identity differs")
        if (
            mapping.tracking_frame
            != deployment.tracking_setup.tracking_frame
        ):
            raise ValueError("ROS tracking frame differs from mapping")
        if mapping.workcell_frame != session.workcell.world_frame:
            raise ValueError("ROS mapping and Workcell frames differ")
        deployment_hash = _sha256_mapping(
            {
                "config_path": config_path,
                "deployment": deployment.to_mapping(),
                "session_hash": session.session_hash,
                "qos": qos.to_mapping(),
                "mapping_sha256": sha256_file(mapping_file),
            }
        )
        local_binding_hash = _sha256_mapping(
            {
                "binding_id": local.binding_id,
                "host_id": local.host_id,
                "ros_domain_id": local.ros_domain_id,
                "rmw_implementation": local.rmw_implementation,
                "dds_profile": local.dds_profile,
                "processes": [
                    {
                        "process_id": process.process_id,
                        "environment_id": process.environment_id,
                        "executable_sha256": _sha256_text(
                            process.executable
                        ),
                        "setup_script_sha256": [
                            _sha256_text(script)
                            for script in process.setup_scripts
                        ],
                    }
                    for process in local.processes
                    if process.process_id in process_ids
                ],
                "sources": [
                    {
                        "binding_key": source.binding_key,
                        "source_kind": source.source_kind,
                        "device_identity_sha256": _sha256_text(
                            source.device_identity
                        ),
                        "endpoint_sha256": _sha256_text(source.endpoint),
                        "calibration_id": source.calibration_id,
                    }
                    for source in local.sources
                    if source.binding_key in live_binding_keys
                ],
            }
        )
        mapping_sha256 = sha256_file(mapping_file)
        return ResolvedRosDeployment(
            config_path=config_path,
            deployment=deployment,
            local_binding=local,
            session=session,
            qos_profile=qos,
            control_profile=control_profile,
            mapping=mapping,
            mapping_path=mapping_path,
            mapping_sha256=mapping_sha256,
            route_plan=route_plan,
            deployment_hash=deployment_hash,
            local_binding_hash=local_binding_hash,
        )

    @staticmethod
    def _validate_routes(
        deployment: RosDeploymentSpec,
        session: ResolvedSession,
    ) -> None:
        expected = {
            (layout.instance_id, layout.group_id)
            for layout in session.session.runtime.control_layouts
        }
        actual = {
            (binding.instance_id, binding.group_id)
            for binding in deployment.control_bindings
        }
        if actual != expected:
            raise ValueError(
                "ROS deployment routes must exactly cover Session layouts"
            )


def _sha256_mapping(value: object) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=True,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )
    return hashlib.sha256(encoded.encode()).hexdigest()


def _sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode()).hexdigest()


__all__ = ["ResolvedRosDeployment", "RosDeploymentResolver"]
