"""Explicit host-local binding projection for the ROS 2 process graph."""

from __future__ import annotations

from dataclasses import dataclass

from wujihand.specs import (
    LocalDeviceBindingSpec,
    RosLocalProcessBindingSpec,
    RosLocalRuntimeBindingSpec,
)


@dataclass(frozen=True, slots=True)
class RosProcessEnvironment:
    executable: str
    environment_id: str
    setup_scripts: tuple[str, ...]


def build_ros_local_runtime_binding(
    native: LocalDeviceBindingSpec,
    *,
    binding_id: str,
    ros_domain_id: int,
    rmw_implementation: str,
    vive: RosProcessEnvironment,
    glove: RosProcessEnvironment,
    isaac: RosProcessEnvironment,
    dds_profile: str | None = None,
) -> RosLocalRuntimeBindingSpec:
    """Reuse native device facts while requiring explicit ROS environments."""

    environments = {
        "vive_source": vive,
        "glove_source": glove,
        "isaac_consumer": isaac,
    }
    return RosLocalRuntimeBindingSpec.from_mapping(
        {
            "schema": "wujihand.ros_local_runtime_binding.v2",
            "binding_id": binding_id,
            "host_id": native.host_id,
            "ros_domain_id": ros_domain_id,
            "rmw_implementation": rmw_implementation,
            "dds_profile": dds_profile,
            "processes": [
                RosLocalProcessBindingSpec(
                    process_id=process_id,
                    executable=environment.executable,
                    environment_id=environment.environment_id,
                    setup_scripts=environment.setup_scripts,
                ).to_mapping()
                for process_id, environment in environments.items()
            ],
            "sources": [source.to_mapping() for source in native.sources],
        }
    )


__all__ = [
    "RosProcessEnvironment",
    "build_ros_local_runtime_binding",
]
