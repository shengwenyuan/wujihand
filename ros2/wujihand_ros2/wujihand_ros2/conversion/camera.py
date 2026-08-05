"""ROS conversion for one completed synthetic wrist-camera transaction."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray

from wujihand.runtime.isaac_d405_camera_capture import (
    SIMULATION_CAMERA_FRAME_SCHEMA,
    SimulationCameraFrame,
    SimulationCameraStaticInventory,
)


CAMERA_RECTIFICATION_ROW_MAJOR = (
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
)


@dataclass(frozen=True, slots=True)
class SimulationCameraRosMessages:
    color: Any
    depth: Any
    camera_info: Any
    truth: Any


def simulation_camera_frame_to_messages(
    frame: SimulationCameraFrame,
    inventory: SimulationCameraStaticInventory,
) -> SimulationCameraRosMessages:
    """Convert one identity-joined frame without resampling either payload."""

    if (
        frame.side != inventory.side
        or frame.optical_frame_id != inventory.optical_frame_id
        or frame.hand_base_frame_id != inventory.hand_base_frame_id
    ):
        raise ValueError("camera frame and static inventory identities differ")
    from sensor_msgs.msg import CameraInfo, Image  # type: ignore[import-not-found]
    from wujihand_interfaces.msg import (  # type: ignore[import-not-found]
        SimulationCameraFrameTruth,
    )

    profile = inventory.profile
    stamp = _stamp(frame.stamp_ns)
    color = Image()
    color.header.stamp = stamp
    color.header.frame_id = frame.optical_frame_id
    color.height = profile.capture.height_px
    color.width = profile.capture.width_px
    color.encoding = profile.rgb.output_encoding
    color.is_bigendian = 0
    color.step = profile.capture.width_px * 3
    color.data = np.ascontiguousarray(frame.rgb).tobytes(order="C")

    depth = Image()
    depth.header.stamp = _stamp(frame.stamp_ns)
    depth.header.frame_id = frame.optical_frame_id
    depth.height = profile.capture.height_px
    depth.width = profile.capture.width_px
    depth.encoding = profile.depth.output_encoding
    depth.is_bigendian = 0
    depth.step = profile.capture.width_px * 4
    depth.data = frame.depth.astype("<f4", copy=False).tobytes(order="C")

    camera_info = CameraInfo()
    camera_info.header.stamp = _stamp(frame.stamp_ns)
    camera_info.header.frame_id = frame.optical_frame_id
    camera_info.height = profile.capture.height_px
    camera_info.width = profile.capture.width_px
    camera_info.distortion_model = profile.optics.distortion_model
    camera_info.d = list(profile.optics.distortion_coefficients)
    camera_info.k = list(inventory.calibration.k_row_major)
    camera_info.r = list(CAMERA_RECTIFICATION_ROW_MAJOR)
    camera_info.p = list(inventory.calibration.p_row_major)

    truth = SimulationCameraFrameTruth()
    truth.schema = SIMULATION_CAMERA_FRAME_SCHEMA
    truth.run_id = frame.run_id
    truth.side = frame.side
    truth.camera_frame_index = frame.camera_frame_index
    truth.stamp_ns = frame.stamp_ns
    truth.world_frame_id = inventory.world_frame_id
    truth.hand_base_frame_id = frame.hand_base_frame_id
    truth.optical_frame_id = frame.optical_frame_id
    truth.control_tick_id = frame.control_tick_id
    truth.physics_substep_index = frame.physics_substep_index
    truth.capture_sim_time_s = frame.capture_sim_time_s
    truth.host_capture_start_ns = frame.host_capture_start_ns
    truth.host_capture_end_ns = frame.host_capture_end_ns
    truth.world_from_hand_base_row_major = _flatten(frame.world_from_hand_base)
    truth.world_from_camera_optical_row_major = _flatten(frame.world_from_camera_optical)
    truth.hand_base_from_camera_optical_row_major = _flatten(frame.hand_base_from_camera_optical)
    truth.completed_frame_identity = frame.completed_frame_identity
    truth.reference_time_numerator = frame.reference_time_numerator
    truth.reference_time_denominator = frame.reference_time_denominator
    return SimulationCameraRosMessages(
        color=color,
        depth=depth,
        camera_info=camera_info,
        truth=truth,
    )


def camera_static_transform(inventory: SimulationCameraStaticInventory) -> Any:
    """Create the sole hand-base → optical static TF edge."""

    from geometry_msgs.msg import TransformStamped  # type: ignore[import-not-found]

    message = TransformStamped()
    message.header.stamp = _stamp(0)
    message.header.frame_id = inventory.hand_base_frame_id
    message.child_frame_id = inventory.optical_frame_id
    _assign_transform(message.transform, inventory.hand_base_from_camera_optical)
    return message


def camera_dynamic_transform(
    frame: SimulationCameraFrame,
    inventory: SimulationCameraStaticInventory,
) -> Any:
    """Create the unique world → hand-base dynamic TF edge for this graph."""

    from geometry_msgs.msg import TransformStamped

    if frame.side != inventory.side:
        raise ValueError("camera frame and TF inventory sides differ")
    message = TransformStamped()
    message.header.stamp = _stamp(frame.stamp_ns)
    message.header.frame_id = inventory.world_frame_id
    message.child_frame_id = inventory.hand_base_frame_id
    _assign_transform(message.transform, frame.world_from_hand_base)
    return message


def _assign_transform(target: Any, matrix: tuple[tuple[float, ...], ...]) -> None:
    value = np.asarray(matrix, dtype=np.float64)
    if value.shape != (4, 4):
        raise ValueError("TF source matrix must be 4x4")
    quaternion = _quaternion_xyzw(value[:3, :3])
    target.translation.x = float(value[0, 3])
    target.translation.y = float(value[1, 3])
    target.translation.z = float(value[2, 3])
    target.rotation.x = quaternion[0]
    target.rotation.y = quaternion[1]
    target.rotation.z = quaternion[2]
    target.rotation.w = quaternion[3]


def _quaternion_xyzw(
    rotation: NDArray[np.float64],
) -> tuple[float, float, float, float]:
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError("TF rotation must be a finite 3x3 matrix")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
        elif axis == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / scale
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / scale
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
    values = np.asarray((x, y, z, w), dtype=np.float64)
    values /= np.linalg.norm(values)
    return cast(
        tuple[float, float, float, float],
        tuple(float(item) for item in values),
    )


def _stamp(stamp_ns: int) -> Any:
    if stamp_ns < 0:
        raise ValueError("ROS simulation stamp cannot be negative")
    from builtin_interfaces.msg import Time  # type: ignore[import-not-found]

    result = Time()
    result.sec = stamp_ns // 1_000_000_000
    result.nanosec = stamp_ns % 1_000_000_000
    return result


def _flatten(matrix: tuple[tuple[float, ...], ...]) -> list[float]:
    return [float(item) for row in matrix for item in row]


__all__ = [
    "SimulationCameraRosMessages",
    "camera_dynamic_transform",
    "camera_static_transform",
    "simulation_camera_frame_to_messages",
]
