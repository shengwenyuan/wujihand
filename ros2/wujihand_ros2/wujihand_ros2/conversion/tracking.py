"""ROS conversion for canonical rigid-body tracking contracts."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Protocol

from wujihand.domain import (
    TrackedRigidBodySample,
    TrackingLifecycleEvent,
    TrackingLifecycleKind,
    TrackingState,
)

from ._message import new_message


class TrackedSampleMessage(Protocol):
    schema: str
    stream_id: str
    device_serial: str
    logical_role: str
    producer_instance: str
    transport_epoch: int
    tracking_setup_revision: str
    sequence: int
    tracking_frame: str
    pose_valid: bool
    position_m: Sequence[float]
    quat_wxyz: Sequence[float]
    connected: bool
    tracking_state: str
    has_quality: bool
    quality: float
    host_time_ns: int
    has_device_time: bool
    device_time_ns: int
    clock_domain: str


class TrackingLifecycleMessage(Protocol):
    schema: str
    producer_instance: str
    tracking_setup_revision: str
    stream_ids: Sequence[str]
    kind: str
    reason: str
    sequence: int
    has_old_transport_epoch: bool
    old_transport_epoch: int
    has_new_transport_epoch: bool
    new_transport_epoch: int
    host_time_ns: int
    clock_domain: str


def tracked_sample_to_message(
    sample: TrackedRigidBodySample,
    *,
    factory: Callable[[], TrackedSampleMessage] | None = None,
) -> TrackedSampleMessage:
    message = new_message(factory, class_name="TrackedRigidBodySample")
    message.schema = sample.schema
    message.stream_id = sample.stream_id
    message.device_serial = sample.device_serial
    message.logical_role = sample.logical_role
    message.producer_instance = sample.producer_instance
    message.transport_epoch = sample.transport_epoch
    message.tracking_setup_revision = sample.tracking_setup_revision
    message.sequence = sample.sequence
    message.tracking_frame = sample.tracking_frame
    message.pose_valid = sample.pose_valid
    message.position_m = (
        (0.0, 0.0, 0.0) if sample.position_m is None else sample.position_m
    )
    message.quat_wxyz = (
        (0.0, 0.0, 0.0, 0.0)
        if sample.quat_wxyz is None
        else sample.quat_wxyz
    )
    message.connected = sample.connected
    message.tracking_state = sample.tracking_state.value
    message.has_quality = sample.quality is not None
    message.quality = 0.0 if sample.quality is None else sample.quality
    message.host_time_ns = sample.host_time_ns
    message.has_device_time = sample.device_time_ns is not None
    message.device_time_ns = (
        0 if sample.device_time_ns is None else sample.device_time_ns
    )
    message.clock_domain = sample.clock_domain
    return message


def tracked_sample_from_message(
    message: TrackedSampleMessage,
) -> TrackedRigidBodySample:
    position = tuple(float(value) for value in message.position_m)
    quaternion = tuple(float(value) for value in message.quat_wxyz)
    if len(position) != 3 or len(quaternion) != 4:
        raise ValueError("tracking message pose arrays have invalid shape")
    if not message.pose_valid and (
        any(value != 0.0 for value in position)
        or any(value != 0.0 for value in quaternion)
    ):
        raise ValueError("invalid tracking pose must use zero wire sentinels")
    if not message.has_quality and message.quality != 0.0:
        raise ValueError("missing tracking quality must use a zero wire sentinel")
    if not message.has_device_time and message.device_time_ns != 0:
        raise ValueError("missing device time must use a zero wire sentinel")
    return TrackedRigidBodySample(
        schema=message.schema,
        stream_id=message.stream_id,
        device_serial=message.device_serial,
        logical_role=message.logical_role,
        producer_instance=message.producer_instance,
        transport_epoch=message.transport_epoch,
        tracking_setup_revision=message.tracking_setup_revision,
        sequence=message.sequence,
        tracking_frame=message.tracking_frame,
        position_m=position if message.pose_valid else None,
        quat_wxyz=quaternion if message.pose_valid else None,
        connected=message.connected,
        pose_valid=message.pose_valid,
        tracking_state=TrackingState(message.tracking_state),
        quality=float(message.quality) if message.has_quality else None,
        host_time_ns=message.host_time_ns,
        device_time_ns=(
            message.device_time_ns if message.has_device_time else None
        ),
        clock_domain=message.clock_domain,
    )


def lifecycle_event_to_message(
    event: TrackingLifecycleEvent,
    *,
    factory: Callable[[], TrackingLifecycleMessage] | None = None,
) -> TrackingLifecycleMessage:
    message = new_message(factory, class_name="TrackingLifecycleEvent")
    message.schema = event.schema
    message.producer_instance = event.producer_instance
    message.tracking_setup_revision = event.tracking_setup_revision
    message.stream_ids = event.stream_ids
    message.kind = event.kind.value
    message.reason = event.reason
    message.sequence = event.sequence
    message.has_old_transport_epoch = event.old_transport_epoch is not None
    message.old_transport_epoch = (
        0 if event.old_transport_epoch is None else event.old_transport_epoch
    )
    message.has_new_transport_epoch = event.new_transport_epoch is not None
    message.new_transport_epoch = (
        0 if event.new_transport_epoch is None else event.new_transport_epoch
    )
    message.host_time_ns = event.host_time_ns
    message.clock_domain = event.clock_domain
    return message


def lifecycle_event_from_message(
    message: TrackingLifecycleMessage,
) -> TrackingLifecycleEvent:
    if (
        not message.has_old_transport_epoch
        and message.old_transport_epoch != 0
    ):
        raise ValueError("missing old epoch must use a zero wire sentinel")
    if (
        not message.has_new_transport_epoch
        and message.new_transport_epoch != 0
    ):
        raise ValueError("missing new epoch must use a zero wire sentinel")
    return TrackingLifecycleEvent(
        schema=message.schema,
        producer_instance=message.producer_instance,
        tracking_setup_revision=message.tracking_setup_revision,
        stream_ids=tuple(message.stream_ids),
        kind=TrackingLifecycleKind(message.kind),
        reason=message.reason,
        sequence=message.sequence,
        old_transport_epoch=(
            message.old_transport_epoch
            if message.has_old_transport_epoch
            else None
        ),
        new_transport_epoch=(
            message.new_transport_epoch
            if message.has_new_transport_epoch
            else None
        ),
        host_time_ns=message.host_time_ns,
        clock_domain=message.clock_domain,
    )
