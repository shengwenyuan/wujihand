from __future__ import annotations

from dataclasses import FrozenInstanceError, fields

import numpy as np
import pytest

from wujihand.domain import (
    CLUTCH_EVENT_SCHEMA,
    HOST_MONOTONIC_CLOCK_DOMAIN,
    TRACKED_RIGID_BODY_SAMPLE_SCHEMA,
    TRACKING_POSITION_UNIT,
    TRACKING_QUATERNION_CONVENTION,
    TRACKING_QUATERNION_ORDER,
    ClutchEdge,
    ClutchEvent,
    TrackedRigidBodySample,
    TrackingState,
)
from wujihand.ports import TrackerInventoryItem, TrackingInputPort, TrackingPoll


def running_sample(**overrides: object) -> TrackedRigidBodySample:
    values: dict[str, object] = {
        "stream_id": "vive.right.operator",
        "device_serial": "LHR-24B6E288",
        "logical_role": "operator_tracker_right",
        "sequence": 7,
        "tracking_frame": "vive_tracking",
        "position_m": (0.25, -0.5, 1.2),
        "quat_wxyz": (1.0, 0.0, 0.0, 0.0),
        "connected": True,
        "pose_valid": True,
        "tracking_state": TrackingState.RUNNING,
        "quality": 0.8,
        "host_time_ns": 123,
        "device_time_ns": None,
    }
    values.update(overrides)
    return TrackedRigidBodySample(**values)  # type: ignore[arg-type]


def lost_sample(**overrides: object) -> TrackedRigidBodySample:
    values: dict[str, object] = {
        "stream_id": "vive.right.operator",
        "device_serial": "LHR-24B6E288",
        "logical_role": "operator_tracker_right",
        "sequence": 8,
        "tracking_frame": "vive_tracking",
        "position_m": None,
        "quat_wxyz": None,
        "connected": True,
        "pose_valid": False,
        "tracking_state": TrackingState.LOST,
        "quality": None,
        "host_time_ns": 124,
        "device_time_ns": None,
    }
    values.update(overrides)
    return TrackedRigidBodySample(**values)  # type: ignore[arg-type]


def test_running_sample_freezes_metric_active_wxyz_contract() -> None:
    position = np.asarray([0.25, -0.5, 1.2])
    quaternion = np.asarray([1.0, 0.0, 0.0, 0.0])
    sample = running_sample(position_m=position, quat_wxyz=quaternion)
    position[:] = 0.0
    quaternion[:] = 0.0

    assert sample.schema == TRACKED_RIGID_BODY_SAMPLE_SCHEMA
    assert sample.clock_domain == HOST_MONOTONIC_CLOCK_DOMAIN
    assert TRACKING_POSITION_UNIT == "m"
    assert TRACKING_QUATERNION_ORDER == "wxyz"
    assert TRACKING_QUATERNION_CONVENTION == "active"
    assert sample.position_m == (0.25, -0.5, 1.2)
    assert sample.quat_wxyz == (1.0, 0.0, 0.0, 0.0)
    with pytest.raises(FrozenInstanceError):
        sample.sequence = 9  # type: ignore[misc]


def test_lost_sample_never_carries_a_stale_or_partial_pose() -> None:
    sample = lost_sample()

    assert sample.connected
    assert not sample.pose_valid
    assert sample.tracking_state is TrackingState.LOST
    assert sample.position_m is None
    assert sample.quat_wxyz is None
    assert sample.quality is None

    with pytest.raises(ValueError, match="must not carry"):
        lost_sample(position_m=(0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="must not carry"):
        lost_sample(quat_wxyz=(1.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="quality=None"):
        lost_sample(quality=0.0)


@pytest.mark.parametrize(
    "state",
    (
        TrackingState.UNINITIALIZED,
        TrackingState.CALIBRATING,
        TrackingState.OUT_OF_RANGE,
        TrackingState.ROTATION_ONLY,
        TrackingState.LOST,
    ),
)
def test_non_running_states_are_explicitly_non_actionable(
    state: TrackingState,
) -> None:
    sample = lost_sample(tracking_state=state, connected=True)
    assert not sample.pose_valid
    assert sample.position_m is None
    assert sample.quat_wxyz is None


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("schema", "wrong.v1", "schema"),
        ("stream_id", "bad stream", "stream_id"),
        ("sequence", True, "sequence"),
        ("sequence", -1, "sequence"),
        ("tracking_state", "running", "TrackingState"),
        ("connected", 1, "booleans"),
        ("pose_valid", 1, "booleans"),
        ("position_m", (0.0, np.nan, 0.0), "position_m"),
        ("position_m", (True, 0.0, 0.0), "position_m"),
        ("quat_wxyz", (2.0, 0.0, 0.0, 0.0), "unit norm"),
        ("quality", 0.0, "quality"),
        ("quality", np.inf, "quality"),
        ("host_time_ns", -1, "host_time_ns"),
        ("device_time_ns", -1, "device_time_ns"),
        ("clock_domain", "unix_wall_clock", "clock_domain"),
    ),
)
def test_running_sample_rejects_malformed_or_ambiguous_fields(
    field: str,
    value: object,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        running_sample(**{field: value})


def test_tracking_state_pose_and_connection_are_strictly_coupled() -> None:
    with pytest.raises(ValueError, match="RUNNING requires"):
        lost_sample(tracking_state=TrackingState.RUNNING)
    with pytest.raises(ValueError, match="only RUNNING"):
        running_sample(tracking_state=TrackingState.CALIBRATING)
    with pytest.raises(ValueError, match="connected device"):
        running_sample(connected=False)
    with pytest.raises(ValueError, match="UNINITIALIZED or LOST"):
        lost_sample(
            connected=False,
            tracking_state=TrackingState.OUT_OF_RANGE,
        )


def test_clutch_event_has_matching_identity_and_monotonic_epoch_semantics() -> None:
    pressed = ClutchEvent(
        stream_id="vive.right.operator",
        device_serial="LHR-24B6E288",
        logical_role="operator_tracker_right",
        input_id="tracker.system_button",
        edge=ClutchEdge.PRESSED,
        sequence=3,
        host_time_ns=500,
        epoch_request=True,
    )
    released = ClutchEvent(
        stream_id=pressed.stream_id,
        device_serial=pressed.device_serial,
        logical_role=pressed.logical_role,
        input_id=pressed.input_id,
        edge=ClutchEdge.RELEASED,
        sequence=4,
        host_time_ns=510,
        epoch_request=False,
    )

    assert pressed.schema == CLUTCH_EVENT_SCHEMA
    assert pressed.clock_domain == HOST_MONOTONIC_CLOCK_DOMAIN
    assert released.edge is ClutchEdge.RELEASED
    with pytest.raises(FrozenInstanceError):
        pressed.epoch_request = False  # type: ignore[misc]


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("schema", "wrong.v1", "schema"),
        ("device_serial", "", "device_serial"),
        ("edge", "pressed", "ClutchEdge"),
        ("sequence", True, "sequence"),
        ("host_time_ns", -1, "host_time_ns"),
        ("clock_domain", "unix", "clock_domain"),
        ("epoch_request", 1, "boolean"),
    ),
)
def test_clutch_event_rejects_invalid_fields(
    field: str,
    value: object,
    message: str,
) -> None:
    values: dict[str, object] = {
        "stream_id": "vive.right.operator",
        "device_serial": "LHR-24B6E288",
        "logical_role": "operator_tracker_right",
        "input_id": "tracker.system_button",
        "edge": ClutchEdge.PRESSED,
        "sequence": 3,
        "host_time_ns": 500,
        "epoch_request": False,
    }
    values[field] = value
    with pytest.raises(ValueError, match=message):
        ClutchEvent(**values)  # type: ignore[arg-type]


def test_release_edge_cannot_request_a_new_epoch() -> None:
    with pytest.raises(ValueError, match="PRESSED"):
        ClutchEvent(
            stream_id="vive.right.operator",
            device_serial="LHR-24B6E288",
            logical_role="operator_tracker_right",
            input_id="tracker.system_button",
            edge=ClutchEdge.RELEASED,
            sequence=4,
            host_time_ns=510,
            epoch_request=True,
        )


class FakeTrackingInput:
    item = TrackerInventoryItem(
        serial="LHR-24B6E288",
        device_class="generic_tracker",
        model="VIVE Tracker 3.0",
        manufacturer="HTC Corporation",
        connected=True,
    )

    def inventory(self) -> tuple[TrackerInventoryItem, ...]:
        return (self.item,)

    def start(self) -> TrackerInventoryItem:
        return self.item

    def poll(self, *, host_time_ns: int | None = None) -> TrackingPoll:
        return TrackingPoll(
            sample=running_sample(host_time_ns=123 if host_time_ns is None else host_time_ns)
        )

    def close(self) -> None:
        return None


def test_tracking_port_is_structural_and_device_sdk_independent() -> None:
    source = FakeTrackingInput()
    assert isinstance(source, TrackingInputPort)
    assert source.inventory() == (source.item,)
    assert source.start() == source.item
    result = source.poll(host_time_ns=700)
    assert result.sample.pose_valid
    assert result.sample.host_time_ns == 700
    assert result.clutch_events == ()
    source.close()


def test_tracking_poll_is_atomic_immutable_and_identity_checked() -> None:
    sample = running_sample()
    event = ClutchEvent(
        stream_id=sample.stream_id,
        device_serial=sample.device_serial,
        logical_role=sample.logical_role,
        input_id="tracker.system_button",
        edge=ClutchEdge.PRESSED,
        sequence=3,
        host_time_ns=sample.host_time_ns,
        epoch_request=True,
    )
    poll = TrackingPoll(sample=sample, clutch_events=(event,))

    assert poll.clutch_events == (event,)
    with pytest.raises(FrozenInstanceError):
        poll.sample = lost_sample()  # type: ignore[misc]

    wrong_stream = ClutchEvent(
        stream_id="vive.left.operator",
        device_serial=sample.device_serial,
        logical_role=sample.logical_role,
        input_id="tracker.system_button",
        edge=ClutchEdge.PRESSED,
        sequence=4,
        host_time_ns=sample.host_time_ns,
        epoch_request=True,
    )
    with pytest.raises(ValueError, match="identity"):
        TrackingPoll(sample=sample, clutch_events=(wrong_stream,))


def test_tracker_inventory_is_strict_immutable_and_has_no_device_index() -> None:
    item = FakeTrackingInput.item
    assert tuple(field.name for field in fields(item)) == (
        "serial",
        "device_class",
        "model",
        "manufacturer",
        "connected",
    )
    with pytest.raises(FrozenInstanceError):
        item.connected = False  # type: ignore[misc]

    for field, value in (
        ("serial", ""),
        ("device_class", " tracker"),
        ("model", "tracker\n3"),
        ("manufacturer", "HTC "),
        ("connected", 1),
    ):
        values: dict[str, object] = {
            "serial": "LHR-24B6E288",
            "device_class": "generic_tracker",
            "model": "VIVE Tracker 3.0",
            "manufacturer": "HTC Corporation",
            "connected": True,
        }
        values[field] = value
        with pytest.raises(ValueError):
            TrackerInventoryItem(**values)  # type: ignore[arg-type]
