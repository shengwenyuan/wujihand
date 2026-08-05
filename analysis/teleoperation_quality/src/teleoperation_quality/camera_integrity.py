"""Fail-closed normalization of synthetic D405 messages and TF edges."""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction
from typing import Any, cast

import numpy as np

from .model import CameraFrameRecord, Matrix4, Side, TransformRecord

CAMERA_TRUTH_SCHEMA = "wujihand.simulation_camera_frame_truth.v2"
CAMERA_RATE_HZ = 30
CAMERA_WIDTH_PX = 640
CAMERA_HEIGHT_PX = 480
_MATRIX_ATOL = 1e-8
_CAMERA_TOPIC_TYPES = {
    "color/image_raw": "sensor_msgs/msg/Image",
    "depth/image_raw": "sensor_msgs/msg/Image",
    "camera_info": "sensor_msgs/msg/CameraInfo",
    "frame_truth": "wujihand_interfaces/msg/SimulationCameraFrameTruth",
}


@dataclass(frozen=True, slots=True)
class _ImageFact:
    stamp_ns: int
    frame_id: str
    bag_time_ns: int
    width_px: int
    height_px: int
    encoding: str
    payload_bytes: int
    finite_depth_pixels: int | None


@dataclass(frozen=True, slots=True)
class _CameraInfoFact:
    stamp_ns: int
    frame_id: str
    bag_time_ns: int
    width_px: int
    height_px: int
    distortion_model: str
    k_row_major: tuple[float, ...]
    d: tuple[float, ...]
    r_row_major: tuple[float, ...]
    p_row_major: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class _TruthFact:
    side: Side
    camera_frame_index: int
    stamp_ns: int
    world_frame_id: str
    hand_base_frame_id: str
    optical_frame_id: str
    control_tick_id: int
    physics_substep_index: int
    capture_sim_time_s: float
    host_capture_start_ns: int
    host_capture_end_ns: int
    reference_time_numerator: int
    reference_time_denominator: int
    bag_time_ns: int
    world_from_hand_base: Matrix4
    world_from_camera_optical: Matrix4
    hand_base_from_camera_optical: Matrix4


def expected_camera_message_type(topic: str) -> str | None:
    for suffix, message_type in _CAMERA_TOPIC_TYPES.items():
        if topic.endswith(f"/wrist_camera/{suffix}"):
            return message_type
    if topic in {"/tf", "/tf_static"}:
        return "tf2_msgs/msg/TFMessage"
    return None


def is_camera_topic(topic: str) -> bool:
    return "/wrist_camera/" in topic


def _side_from_topic(topic: str) -> Side:
    for side in ("left", "right"):
        if f"/{side}/wrist_camera/" in topic:
            return side
    raise ValueError(f"camera topic has no left/right side: {topic!r}")


def _stamp_ns(header: Any) -> int:
    seconds = int(header.stamp.sec)
    nanoseconds = int(header.stamp.nanosec)
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("ROS camera header stamp is invalid")
    return seconds * 1_000_000_000 + nanoseconds


def _finite_vector(value: object, size: int, *, field: str) -> tuple[float, ...]:
    result = tuple(float(item) for item in cast(Any, value))
    if len(result) != size or not np.isfinite(np.asarray(result, dtype=np.float64)).all():
        raise ValueError(f"{field} must contain {size} finite values")
    return result


def _matrix4(value: object, *, field: str) -> Matrix4:
    flat = _finite_vector(value, 16, field=field)
    matrix = np.asarray(flat, dtype=np.float64).reshape(4, 4)
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), rtol=0.0, atol=_MATRIX_ATOL):
        raise ValueError(f"{field} must be homogeneous")
    rotation = matrix[:3, :3]
    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3),
        rtol=0.0,
        atol=_MATRIX_ATOL,
    ) or not np.isclose(np.linalg.det(rotation), 1.0, rtol=0.0, atol=_MATRIX_ATOL):
        raise ValueError(f"{field} must contain a proper rigid rotation")
    return cast(Matrix4, tuple(tuple(float(item) for item in row) for row in matrix))


def _as_array(matrix: Matrix4) -> np.ndarray[Any, np.dtype[np.float64]]:
    return np.asarray(matrix, dtype=np.float64)


def _assert_matrix_equal(first: Matrix4, second: Matrix4, *, field: str) -> None:
    if not np.allclose(
        _as_array(first),
        _as_array(second),
        rtol=0.0,
        atol=_MATRIX_ATOL,
    ):
        raise ValueError(f"{field} does not close")


def _image(message: Any, *, bag_time_ns: int, depth: bool) -> _ImageFact:
    width = int(message.width)
    height = int(message.height)
    expected_encoding = "32FC1" if depth else "rgb8"
    bytes_per_pixel = 4 if depth else 3
    if width != CAMERA_WIDTH_PX or height != CAMERA_HEIGHT_PX:
        raise ValueError("synthetic D405 image resolution must be 640x480")
    if str(message.encoding) != expected_encoding:
        raise ValueError(f"synthetic D405 image encoding must be {expected_encoding}")
    if bool(message.is_bigendian):
        raise ValueError("synthetic D405 image payload must be little-endian")
    if int(message.step) != width * bytes_per_pixel:
        raise ValueError("synthetic D405 image step is inconsistent")
    payload = bytes(message.data)
    if len(payload) != width * height * bytes_per_pixel:
        raise ValueError("synthetic D405 image payload size is inconsistent")
    finite_depth_pixels = None
    if depth:
        values = np.frombuffer(payload, dtype="<f4")
        finite = np.isfinite(values)
        canonical_invalid = values.view("<u4") == 0x7FC00000
        if (values[finite] <= 0.0).any() or not np.logical_or(finite, canonical_invalid).all():
            raise ValueError("synthetic D405 depth violates finite-Z/canonical-qNaN policy")
        finite_depth_pixels = int(finite.sum())
    return _ImageFact(
        stamp_ns=_stamp_ns(message.header),
        frame_id=str(message.header.frame_id),
        bag_time_ns=bag_time_ns,
        width_px=width,
        height_px=height,
        encoding=expected_encoding,
        payload_bytes=len(payload),
        finite_depth_pixels=finite_depth_pixels,
    )


def _camera_info(message: Any, *, bag_time_ns: int) -> _CameraInfoFact:
    width = int(message.width)
    height = int(message.height)
    if width != CAMERA_WIDTH_PX or height != CAMERA_HEIGHT_PX:
        raise ValueError("synthetic D405 CameraInfo resolution must be 640x480")
    distortion_model = str(message.distortion_model)
    if distortion_model != "plumb_bob":
        raise ValueError("synthetic D405 distortion model must be plumb_bob")
    return _CameraInfoFact(
        stamp_ns=_stamp_ns(message.header),
        frame_id=str(message.header.frame_id),
        bag_time_ns=bag_time_ns,
        width_px=width,
        height_px=height,
        distortion_model=distortion_model,
        k_row_major=_finite_vector(message.k, 9, field="CameraInfo K"),
        d=_finite_vector(message.d, 5, field="CameraInfo D"),
        r_row_major=_finite_vector(message.r, 9, field="CameraInfo R"),
        p_row_major=_finite_vector(message.p, 12, field="CameraInfo P"),
    )


def _rational_stamp_ns(numerator: int, denominator: int) -> int:
    if numerator < 0 or denominator <= 0:
        raise ValueError("camera completed-frame rational identity is invalid")
    quotient, remainder = divmod(numerator * 1_000_000_000, denominator)
    return quotient + int(remainder * 2 >= denominator)


def _truth(message: Any, *, bag_time_ns: int, expected_run_id: str) -> _TruthFact:
    if str(message.schema) != CAMERA_TRUTH_SCHEMA:
        raise ValueError("synthetic D405 frame truth schema is invalid")
    if str(message.run_id) != expected_run_id:
        raise ValueError("synthetic D405 frame truth run_id mismatch")
    side_value = str(message.side)
    if side_value not in {"left", "right"}:
        raise ValueError("synthetic D405 truth side must be left or right")
    side = cast(Side, side_value)
    numerator = int(message.reference_time_numerator)
    denominator = int(message.reference_time_denominator)
    stamp_ns = int(message.stamp_ns)
    reference_stamp_ns = _rational_stamp_ns(numerator, denominator)
    if abs(stamp_ns - reference_stamp_ns) > 1_000:
        raise ValueError("camera truth stamp exceeds rational frame identity tolerance")
    if str(message.completed_frame_identity) != f"{numerator}/{denominator}":
        raise ValueError("camera completed-frame identity string is inconsistent")
    capture_sim_time_s = float(message.capture_sim_time_s)
    if not math.isfinite(capture_sim_time_s) or not math.isclose(
        capture_sim_time_s,
        float(Fraction(numerator, denominator)),
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ValueError("camera capture simulation time is inconsistent")
    host_start = int(message.host_capture_start_ns)
    host_end = int(message.host_capture_end_ns)
    if host_start < 0 or host_end < host_start:
        raise ValueError("camera host capture interval is invalid")
    world_from_hand = _matrix4(
        message.world_from_hand_base_row_major,
        field="world_from_hand_base",
    )
    world_from_camera = _matrix4(
        message.world_from_camera_optical_row_major,
        field="world_from_camera_optical",
    )
    hand_from_camera = _matrix4(
        message.hand_base_from_camera_optical_row_major,
        field="hand_base_from_camera_optical",
    )
    composed = cast(
        Matrix4,
        tuple(
            tuple(float(item) for item in row)
            for row in (_as_array(world_from_hand) @ _as_array(hand_from_camera))
        ),
    )
    _assert_matrix_equal(composed, world_from_camera, field="camera truth extrinsic chain")
    expected_hand_frame = f"wujihand_{side}_hand_base"
    expected_optical_frame = f"wujihand_{side}_wrist_camera_optical"
    if (
        str(message.world_frame_id) != "world"
        or str(message.hand_base_frame_id) != expected_hand_frame
        or str(message.optical_frame_id) != expected_optical_frame
    ):
        raise ValueError("synthetic D405 truth frame IDs are inconsistent")
    return _TruthFact(
        side=side,
        camera_frame_index=int(message.camera_frame_index),
        stamp_ns=stamp_ns,
        world_frame_id="world",
        hand_base_frame_id=expected_hand_frame,
        optical_frame_id=expected_optical_frame,
        control_tick_id=int(message.control_tick_id),
        physics_substep_index=int(message.physics_substep_index),
        capture_sim_time_s=capture_sim_time_s,
        host_capture_start_ns=host_start,
        host_capture_end_ns=host_end,
        reference_time_numerator=numerator,
        reference_time_denominator=denominator,
        bag_time_ns=bag_time_ns,
        world_from_hand_base=world_from_hand,
        world_from_camera_optical=world_from_camera,
        hand_base_from_camera_optical=hand_from_camera,
    )


def _transform_matrix(transform: Any) -> Matrix4:
    translation = np.asarray(
        (
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        ),
        dtype=np.float64,
    )
    quaternion = np.asarray(
        (
            float(transform.rotation.x),
            float(transform.rotation.y),
            float(transform.rotation.z),
            float(transform.rotation.w),
        ),
        dtype=np.float64,
    )
    if not np.isfinite(translation).all() or not np.isfinite(quaternion).all():
        raise ValueError("TF transform contains non-finite values")
    norm = float(np.linalg.norm(quaternion))
    if not np.isclose(norm, 1.0, rtol=0.0, atol=1e-6):
        raise ValueError("TF quaternion must be normalized")
    x, y, z, w = quaternion
    rotation = np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return _matrix4(matrix.reshape(-1), field="TF transform")


class CameraIntegrityAccumulator:
    """Collect ROS messages, then require exact dual-camera frame bundles."""

    def __init__(self, *, expected_run_id: str | None) -> None:
        self._expected_run_id = expected_run_id
        self._camera_topics: set[str] = set()
        self._bundles: dict[tuple[Side, int], dict[str, object]] = {}
        self._transforms: list[TransformRecord] = []

    def observe_camera(self, topic: str, message: Any, *, bag_time_ns: int) -> None:
        if self._expected_run_id is None:
            raise ValueError("expected_run_id is required to validate D405 camera messages")
        side = _side_from_topic(topic)
        self._camera_topics.add(topic)
        fact: _ImageFact | _CameraInfoFact | _TruthFact
        if topic.endswith("/color/image_raw"):
            kind = "color"
            fact = _image(message, bag_time_ns=bag_time_ns, depth=False)
            stamp_ns = fact.stamp_ns
        elif topic.endswith("/depth/image_raw"):
            kind = "depth"
            fact = _image(message, bag_time_ns=bag_time_ns, depth=True)
            stamp_ns = fact.stamp_ns
        elif topic.endswith("/camera_info"):
            kind = "info"
            fact = _camera_info(message, bag_time_ns=bag_time_ns)
            stamp_ns = fact.stamp_ns
        elif topic.endswith("/frame_truth"):
            kind = "truth"
            fact = _truth(
                message,
                bag_time_ns=bag_time_ns,
                expected_run_id=self._expected_run_id,
            )
            truth = fact
            if truth.side != side:
                raise ValueError("camera topic side differs from frame truth")
            stamp_ns = truth.stamp_ns
        else:
            raise ValueError(f"unsupported camera topic: {topic!r}")
        bundle = self._bundles.setdefault((side, stamp_ns), {})
        if kind in bundle:
            raise ValueError(f"duplicate {side} camera {kind} at stamp {stamp_ns}")
        bundle[kind] = fact

    def observe_tf(self, topic: str, message: Any, *, bag_time_ns: int) -> None:
        static = topic == "/tf_static"
        for value in message.transforms:
            parent = str(value.header.frame_id)
            child = str(value.child_frame_id)
            if not parent or not child or parent == child:
                raise ValueError("TF edge must have distinct non-empty frames")
            self._transforms.append(
                TransformRecord(
                    static=static,
                    bag_time_ns=bag_time_ns,
                    stamp_ns=_stamp_ns(value.header),
                    parent_frame_id=parent,
                    child_frame_id=child,
                    parent_from_child=_transform_matrix(value.transform),
                )
            )

    def finalize(
        self, *, declared_topics: set[str]
    ) -> tuple[
        tuple[CameraFrameRecord, ...],
        tuple[TransformRecord, ...],
    ]:
        declared_camera_topics = {topic for topic in declared_topics if is_camera_topic(topic)}
        if not declared_camera_topics:
            if self._camera_topics or self._bundles:
                raise ValueError("camera messages were observed without declared camera topics")
            return (), tuple(self._transforms)
        expected_topics = {
            topic
            for topic in declared_camera_topics
            if expected_camera_message_type(topic) is not None
        }
        if expected_topics != declared_camera_topics or len(declared_camera_topics) != 8:
            raise ValueError("D405 recording must declare exactly eight supported camera topics")
        for side in ("left", "right"):
            for suffix in _CAMERA_TOPIC_TYPES:
                matches = [
                    topic
                    for topic in declared_camera_topics
                    if f"/{side}/wrist_camera/{suffix}" in topic
                ]
                if len(matches) != 1:
                    raise ValueError(f"D405 topic inventory lacks unique {side} {suffix}")
        frames = tuple(
            self._bundle_to_record(side, stamp_ns, bundle)
            for (side, stamp_ns), bundle in sorted(
                self._bundles.items(),
                key=lambda item: (item[0][0], item[0][1]),
            )
        )
        self._validate_sequences(frames)
        self._validate_tf_closure(frames)
        return frames, tuple(self._transforms)

    @staticmethod
    def _bundle_to_record(
        side: Side,
        stamp_ns: int,
        bundle: dict[str, object],
    ) -> CameraFrameRecord:
        if set(bundle) != {"color", "depth", "info", "truth"}:
            raise ValueError(f"incomplete {side} camera bundle at {stamp_ns}: {sorted(bundle)}")
        color = cast(_ImageFact, bundle["color"])
        depth = cast(_ImageFact, bundle["depth"])
        info = cast(_CameraInfoFact, bundle["info"])
        truth = cast(_TruthFact, bundle["truth"])
        if truth.side != side or any(
            value != stamp_ns
            for value in (color.stamp_ns, depth.stamp_ns, info.stamp_ns, truth.stamp_ns)
        ):
            raise ValueError("camera bundle identity is inconsistent")
        if any(
            frame_id != truth.optical_frame_id
            for frame_id in (color.frame_id, depth.frame_id, info.frame_id)
        ):
            raise ValueError("camera bundle frame_id is inconsistent")
        assert depth.finite_depth_pixels is not None
        return CameraFrameRecord(
            side=side,
            camera_frame_index=truth.camera_frame_index,
            stamp_ns=stamp_ns,
            world_frame_id=truth.world_frame_id,
            hand_base_frame_id=truth.hand_base_frame_id,
            optical_frame_id=truth.optical_frame_id,
            control_tick_id=truth.control_tick_id,
            physics_substep_index=truth.physics_substep_index,
            capture_sim_time_s=truth.capture_sim_time_s,
            host_capture_start_ns=truth.host_capture_start_ns,
            host_capture_end_ns=truth.host_capture_end_ns,
            reference_time_numerator=truth.reference_time_numerator,
            reference_time_denominator=truth.reference_time_denominator,
            color_bag_time_ns=color.bag_time_ns,
            depth_bag_time_ns=depth.bag_time_ns,
            camera_info_bag_time_ns=info.bag_time_ns,
            truth_bag_time_ns=truth.bag_time_ns,
            width_px=color.width_px,
            height_px=color.height_px,
            color_encoding=color.encoding,
            depth_encoding=depth.encoding,
            color_payload_bytes=color.payload_bytes,
            depth_payload_bytes=depth.payload_bytes,
            finite_depth_pixels=depth.finite_depth_pixels,
            distortion_model=info.distortion_model,
            k_row_major=info.k_row_major,
            d=info.d,
            r_row_major=info.r_row_major,
            p_row_major=info.p_row_major,
            world_from_hand_base=truth.world_from_hand_base,
            world_from_camera_optical=truth.world_from_camera_optical,
            hand_base_from_camera_optical=truth.hand_base_from_camera_optical,
        )

    @staticmethod
    def _validate_sequences(frames: tuple[CameraFrameRecord, ...]) -> None:
        by_side = {
            side: sorted(
                (frame for frame in frames if frame.side == side),
                key=lambda frame: frame.camera_frame_index,
            )
            for side in ("left", "right")
        }
        for side, records in by_side.items():
            if not records:
                raise ValueError(f"D405 recording has no {side} camera frames")
            if [record.camera_frame_index for record in records] != list(range(len(records))):
                raise ValueError(f"{side} D405 camera frame index has a gap or duplicate")
            for index, record in enumerate(records):
                if record.control_tick_id != index * 2 + 1:
                    raise ValueError(f"{side} D405 frame is not on the second control tick")
                if record.physics_substep_index != index * 4 + 3:
                    raise ValueError(f"{side} D405 frame is not on the fourth physics substep")
                if index:
                    previous = records[index - 1]
                    if record.stamp_ns - previous.stamp_ns not in (
                        33_333_333,
                        33_333_334,
                    ):
                        raise ValueError(f"{side} D405 completed-frame cadence is not 30 Hz")
        left = by_side["left"]
        right = by_side["right"]
        if len(left) != len(right) or any(
            (first.camera_frame_index, first.stamp_ns)
            != (second.camera_frame_index, second.stamp_ns)
            for first, second in zip(left, right, strict=True)
        ):
            raise ValueError("left/right D405 completed-frame identities do not align")

    def _validate_tf_closure(self, frames: tuple[CameraFrameRecord, ...]) -> None:
        dynamic: dict[tuple[str, str, int], Matrix4] = {}
        static: dict[tuple[str, str], Matrix4] = {}
        for record in self._transforms:
            if record.parent_frame_id == "world" and record.child_frame_id.endswith(
                "_wrist_camera_optical"
            ):
                raise ValueError("direct world-to-optical TF edge is forbidden")
            if record.static:
                key = (record.parent_frame_id, record.child_frame_id)
                if key in static:
                    raise ValueError(f"duplicate static TF edge: {key!r}")
                static[key] = record.parent_from_child
            else:
                key_dynamic = (
                    record.parent_frame_id,
                    record.child_frame_id,
                    record.stamp_ns,
                )
                if key_dynamic in dynamic:
                    raise ValueError(f"duplicate dynamic TF edge: {key_dynamic!r}")
                dynamic[key_dynamic] = record.parent_from_child
        for frame in frames:
            dynamic_key = (frame.world_frame_id, frame.hand_base_frame_id, frame.stamp_ns)
            static_key = (frame.hand_base_frame_id, frame.optical_frame_id)
            if dynamic_key not in dynamic or static_key not in static:
                raise ValueError("camera truth has no exact dynamic/static TF chain")
            _assert_matrix_equal(
                dynamic[dynamic_key],
                frame.world_from_hand_base,
                field="dynamic TF versus camera truth",
            )
            _assert_matrix_equal(
                static[static_key],
                frame.hand_base_from_camera_optical,
                field="static TF versus camera truth",
            )


__all__ = [
    "CAMERA_HEIGHT_PX",
    "CAMERA_RATE_HZ",
    "CAMERA_TRUTH_SCHEMA",
    "CAMERA_WIDTH_PX",
    "CameraIntegrityAccumulator",
    "expected_camera_message_type",
    "is_camera_topic",
]
