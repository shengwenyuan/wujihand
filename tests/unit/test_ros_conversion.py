from __future__ import annotations

from types import SimpleNamespace
from typing import Any

import pytest

from wujihand.domain import (
    CanonicalHandObservation,
    HandLandmark,
    HandSide,
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    TrackedRigidBodySample,
    TrackingLifecycleEvent,
    TrackingLifecycleKind,
    TrackingState,
)
from wujihand_ros2.conversion import (
    HandObservationTransportEnvelope,
    hand_envelope_from_message,
    hand_envelope_to_message,
    lifecycle_event_from_message,
    lifecycle_event_to_message,
    tracked_sample_from_message,
    tracked_sample_to_message,
)


def message_factory() -> Any:
    return SimpleNamespace()


def running_sample() -> TrackedRigidBodySample:
    return TrackedRigidBodySample(
        stream_id="tracker_right",
        device_serial="tracker-right",
        logical_role="operator_right",
        producer_instance="vive-source-1",
        transport_epoch=3,
        tracking_setup_revision="workstation2_v1",
        sequence=17,
        tracking_frame="vive_tracking",
        position_m=(0.1, -0.2, 0.3),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        connected=True,
        pose_valid=True,
        tracking_state=TrackingState.RUNNING,
        quality=0.9,
        host_time_ns=1_000_000,
        device_time_ns=None,
    )


def hand_envelope() -> HandObservationTransportEnvelope:
    landmarks = tuple(
        HandLandmark(
            name=name,
            position_m=(index * 0.001, 0.01, -0.02),
            confidence=0.95,
        )
        for index, name in enumerate(MEDIAPIPE_HAND_LANDMARK_NAMES)
    )
    observation = CanonicalHandObservation(
        side=HandSide.LEFT,
        sequence=12,
        source_id="glove_left",
        calibration_id="glove_left_default_v1",
        transform_id="glove_left_to_mediapipe_v1",
        source_time_ns=900_000,
        receive_time_ns=1_000_000,
        device_time_ns=15,
        device_clock_domain="wuji_device",
        frame_id="glove_left",
        landmarks=landmarks,
    )
    return HandObservationTransportEnvelope(
        producer_instance="glove-source-1",
        transport_epoch=2,
        observation=observation,
    )


def test_tracking_sample_round_trip_is_lossless() -> None:
    sample = running_sample()

    message = tracked_sample_to_message(sample, factory=message_factory)

    assert tracked_sample_from_message(message) == sample
    assert tuple(message.quat_wxyz) == (1.0, 0.0, 0.0, 0.0)


def test_invalid_tracking_state_cannot_hide_a_last_pose() -> None:
    sample = TrackedRigidBodySample(
        stream_id="tracker_left",
        device_serial="tracker-left",
        logical_role="operator_left",
        producer_instance="vive-source-1",
        transport_epoch=1,
        tracking_setup_revision="workstation2_v1",
        sequence=2,
        tracking_frame="vive_tracking",
        position_m=None,
        quat_wxyz=None,
        connected=True,
        pose_valid=False,
        tracking_state=TrackingState.LOST,
        quality=None,
        host_time_ns=1_000_000,
        device_time_ns=None,
    )
    message = tracked_sample_to_message(sample, factory=message_factory)
    message.position_m = (1.0, 0.0, 0.0)

    with pytest.raises(ValueError, match="zero wire sentinels"):
        tracked_sample_from_message(message)


def test_tracking_lifecycle_round_trip_preserves_optional_epochs() -> None:
    event = TrackingLifecycleEvent(
        producer_instance="vive-source-1",
        tracking_setup_revision="workstation2_v1",
        stream_ids=("tracker_left", "tracker_right"),
        kind=TrackingLifecycleKind.STARTED,
        reason="activated",
        sequence=0,
        old_transport_epoch=None,
        new_transport_epoch=1,
        host_time_ns=1_000_000,
    )

    message = lifecycle_event_to_message(event, factory=message_factory)

    assert lifecycle_event_from_message(message) == event
    assert message.has_old_transport_epoch is False
    assert message.old_transport_epoch == 0


def test_hand_envelope_round_trip_preserves_transport_and_domain() -> None:
    envelope = hand_envelope()

    message = hand_envelope_to_message(envelope, factory=message_factory)

    assert hand_envelope_from_message(message) == envelope
    assert len(message.landmark_names) == 21
    assert len(message.landmark_positions_m) == 63


def test_hand_envelope_rejects_wrong_order_and_hidden_position() -> None:
    message = hand_envelope_to_message(
        hand_envelope(),
        factory=message_factory,
    )
    names = list(message.landmark_names)
    names[0], names[1] = names[1], names[0]
    message.landmark_names = names

    with pytest.raises(ValueError, match="canonical MediaPipe"):
        hand_envelope_from_message(message)

    message = hand_envelope_to_message(
        hand_envelope(),
        factory=message_factory,
    )
    valid = list(message.landmark_valid)
    valid[0] = False
    message.landmark_valid = valid
    with pytest.raises(ValueError, match="zero position"):
        hand_envelope_from_message(message)
