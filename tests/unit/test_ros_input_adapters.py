from __future__ import annotations

from dataclasses import replace
from threading import Event, Thread
from types import SimpleNamespace
from typing import Any

import pytest

from wujihand.domain import HandSide, TrackingLifecycleEvent, TrackingLifecycleKind
from wujihand.domain import (
    CanonicalHandObservation,
    HandLandmark,
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    TrackedRigidBodySample,
    TrackingState,
)
from wujihand.ports import NoHandObservationAvailable
from wujihand_ros2.conversion import (
    hand_envelope_to_message,
    lifecycle_event_to_message,
    tracked_sample_to_message,
)
from wujihand_ros2.input_adapters import (
    RosHandObservationInputAdapter,
    RosInputSynchronization,
    RosTrackerInputAdapter,
    TrackerInputIdentity,
)
from wujihand_ros2.conversion import HandObservationTransportEnvelope


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
    return HandObservationTransportEnvelope(
        producer_instance="glove-source-1",
        transport_epoch=2,
        observation=CanonicalHandObservation(
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
        ),
    )


def tracker_input() -> RosTrackerInputAdapter:
    sample = running_sample()
    return RosTrackerInputAdapter(
        TrackerInputIdentity(
            stream_id=sample.stream_id,
            device_serial=sample.device_serial,
            logical_role=sample.logical_role,
            tracking_setup_revision=sample.tracking_setup_revision,
            tracking_frame=sample.tracking_frame,
        )
    )


def test_tracker_callback_is_latest_only_and_rejects_future_time() -> None:
    adapter = tracker_input()
    sample = running_sample()
    message = tracked_sample_to_message(sample, factory=message_factory)

    assert adapter.offer_message(message)
    assert adapter.receive_available(now_ns=sample.host_time_ns - 1) == ()
    assert adapter.metrics.rejected_future_time == 1

    message.sequence += 1
    message.host_time_ns += 1
    assert adapter.offer_message(message)
    assert adapter.receive_available(now_ns=message.host_time_ns) == (
        replace(
            sample,
            sequence=message.sequence,
            host_time_ns=message.host_time_ns,
        ),
    )
    assert adapter.selected is not None
    assert adapter.selected.sample.sequence == message.sequence
    assert adapter.selected.callback_time_ns >= message.host_time_ns


def test_tracker_lifecycle_clears_pending_sample_and_invalidates_reference() -> None:
    adapter = tracker_input()
    sample = running_sample()
    assert adapter.offer_message(tracked_sample_to_message(sample, factory=message_factory))
    event = TrackingLifecycleEvent(
        producer_instance=sample.producer_instance,
        tracking_setup_revision=sample.tracking_setup_revision,
        stream_ids=(sample.stream_id,),
        kind=TrackingLifecycleKind.RESET,
        reason="fixture_reset",
        sequence=1,
        old_transport_epoch=sample.transport_epoch,
        new_transport_epoch=sample.transport_epoch + 1,
        host_time_ns=sample.host_time_ns + 1,
    )

    assert adapter.offer_lifecycle_message(
        lifecycle_event_to_message(event, factory=message_factory)
    )
    assert adapter.receive_available(now_ns=event.host_time_ns) == ()
    assert adapter.take_reference_invalidation()
    assert not adapter.take_reference_invalidation()


def test_hand_callback_exposes_nonblocking_input_port() -> None:
    envelope = hand_envelope()
    observation = envelope.observation
    adapter = RosHandObservationInputAdapter(
        side=HandSide.LEFT,
        source_id=observation.source_id,
        calibration_id=observation.calibration_id,
        transform_id=observation.transform_id,
    )
    adapter.start()
    with pytest.raises(NoHandObservationAvailable):
        adapter.poll(receive_time_ns=observation.receive_time_ns)

    assert adapter.offer_message(hand_envelope_to_message(envelope, factory=message_factory))
    assert adapter.poll(receive_time_ns=observation.receive_time_ns) == observation
    assert adapter.selected is not None
    assert adapter.selected.envelope == envelope
    assert adapter.selected.callback_time_ns >= observation.receive_time_ns
    with pytest.raises(NoHandObservationAvailable):
        adapter.poll(receive_time_ns=observation.receive_time_ns)
    adapter.close()


def test_message_epoch_changes_are_signalled_once() -> None:
    tracker = tracker_input()
    sample = running_sample()
    assert tracker.offer_message(tracked_sample_to_message(sample, factory=message_factory))
    newer = replace(sample, transport_epoch=4, sequence=0)
    assert tracker.offer_message(tracked_sample_to_message(newer, factory=message_factory))
    assert tracker.take_reference_invalidation()
    assert not tracker.take_reference_invalidation()

    envelope = hand_envelope()
    observation = envelope.observation
    hand = RosHandObservationInputAdapter(
        side=observation.side,
        source_id=observation.source_id,
        calibration_id=observation.calibration_id,
        transform_id=observation.transform_id,
    )
    assert hand.offer_message(hand_envelope_to_message(envelope, factory=message_factory))
    assert hand.offer_message(
        hand_envelope_to_message(
            replace(
                envelope,
                transport_epoch=envelope.transport_epoch + 1,
                observation=replace(observation, sequence=0),
            ),
            factory=message_factory,
        )
    )
    assert hand.take_epoch_change()
    assert not hand.take_epoch_change()


def test_shared_tick_snapshot_excludes_callbacks_arriving_during_snapshot() -> None:
    synchronization = RosInputSynchronization()
    sample = running_sample()
    tracker = RosTrackerInputAdapter(
        tracker_input().identity,
        synchronization=synchronization,
    )
    envelope = hand_envelope()
    observation = envelope.observation
    hand = RosHandObservationInputAdapter(
        side=observation.side,
        source_id=observation.source_id,
        calibration_id=observation.calibration_id,
        transform_id=observation.transform_id,
        synchronization=synchronization,
    )
    hand.start()
    assert tracker.offer_message(tracked_sample_to_message(sample, factory=message_factory))
    assert hand.offer_message(hand_envelope_to_message(envelope, factory=message_factory))

    next_sample = replace(sample, sequence=sample.sequence + 1, host_time_ns=1_000_001)
    next_envelope = replace(
        envelope,
        observation=replace(
            observation,
            sequence=observation.sequence + 1,
            source_time_ns=900_001,
            receive_time_ns=1_000_001,
        ),
    )
    tracker_attempting = Event()
    hand_attempting = Event()

    def offer_tracker() -> None:
        tracker_attempting.set()
        assert tracker.offer_message(
            tracked_sample_to_message(next_sample, factory=message_factory)
        )

    def offer_hand() -> None:
        hand_attempting.set()
        assert hand.offer_message(
            hand_envelope_to_message(next_envelope, factory=message_factory)
        )

    tracker_thread = Thread(target=offer_tracker)
    hand_thread = Thread(target=offer_hand)
    with synchronization.locked():
        tracker_thread.start()
        hand_thread.start()
        assert tracker_attempting.wait(timeout=1.0)
        assert hand_attempting.wait(timeout=1.0)
        tracker_snapshot = tracker.snapshot_for_tick(now_ns=1_000_001)
        hand_snapshot = hand.snapshot_for_tick(receive_time_ns=1_000_001)

    tracker_thread.join(timeout=1.0)
    hand_thread.join(timeout=1.0)
    assert not tracker_thread.is_alive()
    assert not hand_thread.is_alive()
    assert tracker_snapshot.selection is not None
    assert tracker_snapshot.selection.sample.sequence == sample.sequence
    assert hand_snapshot.selection is not None
    assert hand_snapshot.selection.envelope.observation.sequence == observation.sequence
    assert tracker.receive_available(now_ns=1_000_001)[0].sequence == sample.sequence
    assert hand.poll(receive_time_ns=1_000_001).sequence == observation.sequence

    with synchronization.locked():
        tracker_snapshot = tracker.snapshot_for_tick(now_ns=1_000_001)
        hand_snapshot = hand.snapshot_for_tick(receive_time_ns=1_000_001)
    assert tracker_snapshot.selection is not None
    assert tracker_snapshot.selection.sample.sequence == next_sample.sequence
    assert hand_snapshot.selection is not None
    assert hand_snapshot.selection.envelope.observation.sequence == (
        next_envelope.observation.sequence
    )
