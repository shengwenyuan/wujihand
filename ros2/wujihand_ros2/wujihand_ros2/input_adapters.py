"""ROS callback boundaries exposing canonical, non-blocking input ports."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Protocol

from wujihand.domain import (
    CanonicalHandObservation,
    HandSide,
    TrackedRigidBodySample,
    TrackingLifecycleKind,
)
from wujihand.ports import NoHandObservationAvailable

from .conversion import (
    HandObservationTransportEnvelope,
    hand_envelope_from_message,
    lifecycle_event_from_message,
    tracked_sample_from_message,
)
from .conversion.hand import HandObservationMessage
from .conversion.tracking import (
    TrackedSampleMessage,
    TrackingLifecycleMessage,
)
from .inbox import EpochInboxMetrics, LatestEpochInbox


class TrackerSampleMessageLike(TrackedSampleMessage, Protocol):
    """Structural ROS Tracker message type."""


class TrackerLifecycleMessageLike(TrackingLifecycleMessage, Protocol):
    """Structural ROS Tracker lifecycle message type."""


class HandObservationMessageLike(HandObservationMessage, Protocol):
    """Structural ROS hand-observation message type."""


@dataclass(frozen=True, slots=True)
class TrackerInputIdentity:
    stream_id: str
    device_serial: str
    logical_role: str
    tracking_setup_revision: str
    tracking_frame: str


@dataclass(frozen=True, slots=True)
class RosInputMetrics:
    rejected_contract: int
    rejected_identity: int
    rejected_future_time: int
    lifecycle_resets: int


@dataclass(frozen=True, slots=True)
class RosTrackerSelection:
    sample: TrackedRigidBodySample
    callback_time_ns: int


@dataclass(frozen=True, slots=True)
class RosHandSelection:
    envelope: HandObservationTransportEnvelope
    callback_time_ns: int


class RosTrackerInputAdapter:
    """Convert callbacks to one latest canonical Tracker sample."""

    def __init__(self, identity: TrackerInputIdentity) -> None:
        self.identity = identity
        self._inbox: LatestEpochInbox[RosTrackerSelection] = LatestEpochInbox()
        self._rejected_contract = 0
        self._rejected_identity = 0
        self._rejected_future_time = 0
        self._lifecycle_resets = 0
        self._reference_invalidation_pending = False
        self._selected: RosTrackerSelection | None = None

    @property
    def metrics(self) -> RosInputMetrics:
        return RosInputMetrics(
            rejected_contract=self._rejected_contract,
            rejected_identity=self._rejected_identity,
            rejected_future_time=self._rejected_future_time,
            lifecycle_resets=self._lifecycle_resets,
        )

    @property
    def inbox_metrics(self) -> EpochInboxMetrics:
        return self._inbox.metrics

    @property
    def selected(self) -> RosTrackerSelection | None:
        """Observation selected by the most recent control-tick drain."""

        return self._selected

    def offer_message(self, message: TrackerSampleMessageLike) -> bool:
        callback_time_ns = time.monotonic_ns()
        try:
            sample = tracked_sample_from_message(message)
        except (TypeError, ValueError):
            self._rejected_contract += 1
            return False
        expected = self.identity
        if (
            sample.stream_id != expected.stream_id
            or sample.device_serial != expected.device_serial
            or sample.logical_role != expected.logical_role
            or sample.tracking_setup_revision != expected.tracking_setup_revision
            or sample.tracking_frame != expected.tracking_frame
        ):
            self._rejected_identity += 1
            return False
        rebinds = self._inbox.metrics.rebinds
        accepted = self._inbox.offer(
            RosTrackerSelection(
                sample=sample,
                callback_time_ns=callback_time_ns,
            ),
            producer_instance=sample.producer_instance,
            transport_epoch=sample.transport_epoch,
            sequence=sample.sequence,
        )
        if accepted and self._inbox.metrics.rebinds > rebinds:
            self._reference_invalidation_pending = True
        return accepted

    def offer_lifecycle_message(
        self,
        message: TrackerLifecycleMessageLike,
    ) -> bool:
        try:
            event = lifecycle_event_from_message(message)
        except (TypeError, ValueError):
            self._rejected_contract += 1
            return False
        if (
            self.identity.stream_id not in event.stream_ids
            or event.tracking_setup_revision != self.identity.tracking_setup_revision
        ):
            self._rejected_identity += 1
            return False
        if event.kind in {
            TrackingLifecycleKind.STARTED,
            TrackingLifecycleKind.REBOUND,
            TrackingLifecycleKind.RESET,
            TrackingLifecycleKind.STOPPED,
        }:
            self._inbox.clear()
            self._lifecycle_resets += 1
            self._reference_invalidation_pending = True
            self._selected = None
        return True

    def receive_available(
        self,
        *,
        now_ns: int,
    ) -> tuple[TrackedRigidBodySample, ...]:
        if type(now_ns) is not int or now_ns < 0:
            raise ValueError("now_ns must be a non-negative integer")
        self._selected = None
        selection = self._inbox.drain()
        if selection is None:
            return ()
        sample = selection.sample
        if sample.host_time_ns > now_ns:
            self._rejected_future_time += 1
            return ()
        self._selected = selection
        return (sample,)

    def take_reference_invalidation(self) -> bool:
        pending = self._reference_invalidation_pending
        self._reference_invalidation_pending = False
        return pending


class RosHandObservationInputAdapter:
    """Expose a callback-fed hand inbox as HandObservationInputPort."""

    def __init__(
        self,
        *,
        side: HandSide,
        source_id: str,
        calibration_id: str,
        transform_id: str,
    ) -> None:
        self.side = side
        self.source_id = source_id
        self.calibration_id = calibration_id
        self.transform_id = transform_id
        self._inbox: LatestEpochInbox[RosHandSelection] = LatestEpochInbox()
        self._started = False
        self._closed = False
        self._rejected_contract = 0
        self._rejected_identity = 0
        self._rejected_future_time = 0
        self._lifecycle_resets = 0
        self._epoch_change_pending = False
        self._selected: RosHandSelection | None = None

    @property
    def metrics(self) -> RosInputMetrics:
        return RosInputMetrics(
            rejected_contract=self._rejected_contract,
            rejected_identity=self._rejected_identity,
            rejected_future_time=self._rejected_future_time,
            lifecycle_resets=self._lifecycle_resets,
        )

    @property
    def inbox_metrics(self) -> EpochInboxMetrics:
        return self._inbox.metrics

    @property
    def selected(self) -> RosHandSelection | None:
        """Observation selected by the most recent control-tick poll."""

        return self._selected

    def offer_message(self, message: HandObservationMessageLike) -> bool:
        callback_time_ns = time.monotonic_ns()
        try:
            envelope = hand_envelope_from_message(message)
        except (TypeError, ValueError):
            self._rejected_contract += 1
            return False
        observation = envelope.observation
        if (
            observation.side is not self.side
            or observation.source_id != self.source_id
            or observation.calibration_id != self.calibration_id
            or observation.transform_id != self.transform_id
        ):
            self._rejected_identity += 1
            return False
        rebinds = self._inbox.metrics.rebinds
        accepted = self._offer_envelope(
            envelope,
            callback_time_ns=callback_time_ns,
        )
        if accepted and self._inbox.metrics.rebinds > rebinds:
            self._epoch_change_pending = True
        return accepted

    def start(self) -> None:
        if self._closed:
            raise RuntimeError("ROS hand input is closed")
        if self._started:
            raise RuntimeError("ROS hand input is already started")
        self._started = True

    def poll(
        self,
        *,
        receive_time_ns: int | None = None,
    ) -> CanonicalHandObservation:
        if not self._started or self._closed:
            raise RuntimeError("ROS hand input must be started before poll")
        self._selected = None
        selection = self._inbox.drain()
        if selection is None:
            raise NoHandObservationAvailable
        observation = selection.envelope.observation
        if receive_time_ns is not None and observation.receive_time_ns > receive_time_ns:
            self._rejected_future_time += 1
            raise NoHandObservationAvailable
        self._selected = selection
        return observation

    def close(self) -> None:
        self._inbox.clear()
        self._started = False
        self._closed = True
        self._selected = None

    def take_epoch_change(self) -> bool:
        pending = self._epoch_change_pending
        self._epoch_change_pending = False
        return pending

    def _offer_envelope(
        self,
        envelope: HandObservationTransportEnvelope,
        *,
        callback_time_ns: int,
    ) -> bool:
        return self._inbox.offer(
            RosHandSelection(
                envelope=envelope,
                callback_time_ns=callback_time_ns,
            ),
            producer_instance=envelope.producer_instance,
            transport_epoch=envelope.transport_epoch,
            sequence=envelope.observation.sequence,
        )


__all__ = [
    "RosHandSelection",
    "RosHandObservationInputAdapter",
    "RosInputMetrics",
    "RosTrackerInputAdapter",
    "RosTrackerSelection",
    "TrackerInputIdentity",
]
