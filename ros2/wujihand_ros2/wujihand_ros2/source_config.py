"""Pure device configuration derived from the resolved ROS Deployment."""

from __future__ import annotations

from dataclasses import dataclass

from wujihand.adapters.input import OpenVrTrackerStreamConfig
from wujihand.domain import HandSide
from wujihand.runtime import ResolvedRosDeployment


@dataclass(frozen=True, slots=True)
class GloveSourceConfig:
    side: HandSide
    source_id: str
    serial_number: str
    calibration_id: str
    transform_id: str
    device_name: str


def vive_stream_configs(
    resolved: ResolvedRosDeployment,
) -> tuple[OpenVrTrackerStreamConfig, ...]:
    configs: list[OpenVrTrackerStreamConfig] = []
    for route in resolved.route_plan.routes:
        if route.source.kind != "vive_tracker":
            continue
        if route.local_binding is None:
            raise ValueError(f"{route.side} Tracker binding is missing")
        configs.append(
            OpenVrTrackerStreamConfig(
                tracker_serial=route.local_binding.device_identity,
                stream_id=route.source.source_id,
                logical_role=route.source.logical_role,
                tracking_frame=resolved.mapping.tracking_frame,
            )
        )
    if not configs:
        raise ValueError("ROS VIVE source has no Tracker routes")
    return tuple(sorted(configs, key=lambda item: item.stream_id))


def glove_source_configs(
    resolved: ResolvedRosDeployment,
) -> tuple[GloveSourceConfig, ...]:
    configs: list[GloveSourceConfig] = []
    for route in resolved.route_plan.routes:
        if route.source.kind != "wuji_glove":
            continue
        if route.local_binding is None:
            raise ValueError(f"{route.side} Glove binding is missing")
        side = HandSide(route.side)
        configs.append(
            GloveSourceConfig(
                side=side,
                source_id=route.source.source_id,
                serial_number=route.local_binding.device_identity,
                calibration_id=route.local_binding.calibration_id,
                transform_id="wuji_glove.hand_skeleton.v1",
                device_name=f"nv5_ros_glove_{side.value}",
            )
        )
    return tuple(sorted(configs, key=lambda item: item.side.value))


__all__ = [
    "GloveSourceConfig",
    "glove_source_configs",
    "vive_stream_configs",
]
