"""ROS callback boundaries exposing canonical, non-blocking input ports."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from threading import RLock
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


class RosInputSynchronization:
    """One reentrant lock shared by callback-fed inputs in a consumer."""

    def __init__(self) -> None:
        self._lock = RLock()

    @contextmanager
    def locked(self) -> Iterator[None]:
        with self._lock:
            yield


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


@dataclass(frozen=True, slots=True)
class RosTrackerTickSnapshot:
    selection: RosTrackerSelection | None
    reference_invalidated: bool


@dataclass(frozen=True, slots=True)
class RosHandTickSnapshot:
    selection: RosHandSelection | None
    epoch_changed: bool


class RosTrackerInputAdapter:
    """Convert callbacks to one latest canonical Tracker sample."""

    def __init__(
        self,
        identity: TrackerInputIdentity,
        *,
        synchronization: RosInputSynchronization | None = None,
    ) -> None:
        self.identity = identity
        self._synchronization = synchronization or RosInputSynchronization()
        self._inbox: LatestEpochInbox[RosTrackerSelection] = LatestEpochInbox()
        self._rejected_contract = 0
        self._rejected_identity = 0
        self._rejected_future_time = 0
        self._lifecycle_resets = 0
        self._reference_invalidation_pending = False
        self._selected: RosTrackerSelection | None = None
        self._explicit_snapshot_mode = False
        self._snapshot_ready = False
        self._snapshot_samples: tuple[TrackedRigidBodySample, ...] = ()

    @property
    def metrics(self) -> RosInputMetrics:
        with self._synchronization.locked():
            return RosInputMetrics(
                rejected_contract=self._rejected_contract,
                rejected_identity=self._rejected_identity,
                rejected_future_time=self._rejected_future_time,
                lifecycle_resets=self._lifecycle_resets,
            )

    @property
    def inbox_metrics(self) -> EpochInboxMetrics:
        with self._synchronization.locked():
            return self._inbox.metrics

    @property
    def selected(self) -> RosTrackerSelection | None:
        """Observation selected by the most recent control-tick drain."""

        with self._synchronization.locked():
            return self._selected

    def offer_message(self, message: TrackerSampleMessageLike) -> bool:
        try:
            sample = tracked_sample_from_message(message)
        except (TypeError, ValueError):
            with self._synchronization.locked():
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
            with self._synchronization.locked():
                self._rejected_identity += 1
            return False
        with self._synchronization.locked():
            callback_time_ns = time.monotonic_ns()
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
            with self._synchronization.locked():
                self._rejected_contract += 1
            return False
        if (
            self.identity.stream_id not in event.stream_ids
            or event.tracking_setup_revision != self.identity.tracking_setup_revision
        ):
            with self._synchronization.locked():
                self._rejected_identity += 1
            return False
        if event.kind in {
            TrackingLifecycleKind.STARTED,
            TrackingLifecycleKind.REBOUND,
            TrackingLifecycleKind.RESET,
            TrackingLifecycleKind.STOPPED,
        }:
            with self._synchronization.locked():
                self._inbox.clear()
                self._lifecycle_resets += 1
                self._reference_invalidation_pending = True
        return True

    def snapshot_for_tick(self, *, now_ns: int) -> RosTrackerTickSnapshot:
        """Move one callback-owned latest sample into control-thread state."""

        if type(now_ns) is not int or now_ns < 0:
            raise ValueError("now_ns must be a non-negative integer")
        with self._synchronization.locked():
            self._explicit_snapshot_mode = True
            return self._snapshot_for_tick_locked(
                now_ns=now_ns,
                consume_invalidation=True,
            )

    def receive_available(
        self,
        *,
        now_ns: int,
    ) -> tuple[TrackedRigidBodySample, ...]:
        if type(now_ns) is not int or now_ns < 0:
            raise ValueError("now_ns must be a non-negative integer")
        with self._synchronization.locked():
            if not self._explicit_snapshot_mode:
                self._snapshot_for_tick_locked(
                    now_ns=now_ns,
                    consume_invalidation=False,
                )
            if not self._snapshot_ready:
                return ()
            self._snapshot_ready = False
            samples = self._snapshot_samples
            self._snapshot_samples = ()
            return samples

    def take_reference_invalidation(self) -> bool:
        with self._synchronization.locked():
            pending = self._reference_invalidation_pending
            self._reference_invalidation_pending = False
            return pending

    def _snapshot_for_tick_locked(
        self,
        *,
        now_ns: int,
        consume_invalidation: bool,
    ) -> RosTrackerTickSnapshot:
        self._snapshot_ready = True
        self._snapshot_samples = ()
        self._selected = None
        selection = self._inbox.drain()
        if selection is not None:
            sample = selection.sample
            if sample.host_time_ns > now_ns:
                self._rejected_future_time += 1
            else:
                self._selected = selection
                self._snapshot_samples = (sample,)
        invalidated = self._reference_invalidation_pending
        if consume_invalidation:
            self._reference_invalidation_pending = False
        return RosTrackerTickSnapshot(
            selection=self._selected,
            reference_invalidated=invalidated,
        )


class RosHandObservationInputAdapter:
    """Expose a callback-fed hand inbox as HandObservationInputPort."""

    def __init__(
        self,
        *,
        side: HandSide,
        source_id: str,
        calibration_id: str,
        transform_id: str,
        synchronization: RosInputSynchronization | None = None,
    ) -> None:
        self.side = side
        self.source_id = source_id
        self.calibration_id = calibration_id
        self.transform_id = transform_id
        self._synchronization = synchronization or RosInputSynchronization()
        self._inbox: LatestEpochInbox[RosHandSelection] = LatestEpochInbox()
        self._started = False
        self._closed = False
        self._rejected_contract = 0
        self._rejected_identity = 0
        self._rejected_future_time = 0
        self._lifecycle_resets = 0
        self._epoch_change_pending = False
        self._selected: RosHandSelection | None = None
        self._explicit_snapshot_mode = False
        self._snapshot_ready = False
        self._snapshot_observation: CanonicalHandObservation | None = None

    @property
    def metrics(self) -> RosInputMetrics:
        with self._synchronization.locked():
            return RosInputMetrics(
                rejected_contract=self._rejected_contract,
                rejected_identity=self._rejected_identity,
                rejected_future_time=self._rejected_future_time,
                lifecycle_resets=self._lifecycle_resets,
            )

    @property
    def inbox_metrics(self) -> EpochInboxMetrics:
        with self._synchronization.locked():
            return self._inbox.metrics

    @property
    def selected(self) -> RosHandSelection | None:
        """Observation selected by the most recent control-tick poll."""

        with self._synchronization.locked():
            return self._selected

    def offer_message(self, message: HandObservationMessageLike) -> bool:
        try:
            envelope = hand_envelope_from_message(message)
        except (TypeError, ValueError):
            with self._synchronization.locked():
                self._rejected_contract += 1
            return False
        observation = envelope.observation
        if (
            observation.side is not self.side
            or observation.source_id != self.source_id
            or observation.calibration_id != self.calibration_id
            or observation.transform_id != self.transform_id
        ):
            with self._synchronization.locked():
                self._rejected_identity += 1
            return False
        with self._synchronization.locked():
            callback_time_ns = time.monotonic_ns()
            rebinds = self._inbox.metrics.rebinds
            accepted = self._offer_envelope(
                envelope,
                callback_time_ns=callback_time_ns,
            )
            if accepted and self._inbox.metrics.rebinds > rebinds:
                self._epoch_change_pending = True
            return accepted

    def start(self) -> None:
        with self._synchronization.locked():
            if self._closed:
                raise RuntimeError("ROS hand input is closed")
            if self._started:
                raise RuntimeError("ROS hand input is already started")
            self._started = True

    def snapshot_for_tick(
        self,
        *,
        receive_time_ns: int,
    ) -> RosHandTickSnapshot:
        """Move one callback-owned latest observation into control-thread state."""

        if type(receive_time_ns) is not int or receive_time_ns < 0:
            raise ValueError("receive_time_ns must be a non-negative integer")
        with self._synchronization.locked():
            self._explicit_snapshot_mode = True
            return self._snapshot_for_tick_locked(
                receive_time_ns=receive_time_ns,
                consume_epoch_change=True,
            )

    def poll(
        self,
        *,
        receive_time_ns: int | None = None,
    ) -> CanonicalHandObservation:
        with self._synchronization.locked():
            if not self._started or self._closed:
                raise RuntimeError("ROS hand input must be started before poll")
            if not self._explicit_snapshot_mode:
                effective_time_ns = (
                    time.monotonic_ns() if receive_time_ns is None else receive_time_ns
                )
                self._snapshot_for_tick_locked(
                    receive_time_ns=effective_time_ns,
                    consume_epoch_change=False,
                )
            if not self._snapshot_ready:
                raise NoHandObservationAvailable
            self._snapshot_ready = False
            observation = self._snapshot_observation
            self._snapshot_observation = None
            if observation is None:
                raise NoHandObservationAvailable
            return observation

    def close(self) -> None:
        with self._synchronization.locked():
            self._inbox.clear()
            self._started = False
            self._closed = True
            self._selected = None
            self._snapshot_ready = False
            self._snapshot_observation = None

    def take_epoch_change(self) -> bool:
        with self._synchronization.locked():
            pending = self._epoch_change_pending
            self._epoch_change_pending = False
            return pending

    def _snapshot_for_tick_locked(
        self,
        *,
        receive_time_ns: int,
        consume_epoch_change: bool,
    ) -> RosHandTickSnapshot:
        if not self._started or self._closed:
            raise RuntimeError("ROS hand input must be started before snapshot")
        self._snapshot_ready = True
        self._snapshot_observation = None
        self._selected = None
        selection = self._inbox.drain()
        if selection is not None:
            observation = selection.envelope.observation
            if observation.receive_time_ns > receive_time_ns:
                self._rejected_future_time += 1
            else:
                self._selected = selection
                self._snapshot_observation = observation
        epoch_changed = self._epoch_change_pending
        if consume_epoch_change:
            self._epoch_change_pending = False
        return RosHandTickSnapshot(
            selection=self._selected,
            epoch_changed=epoch_changed,
        )

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
    "RosHandTickSnapshot",
    "RosHandSelection",
    "RosHandObservationInputAdapter",
    "RosInputMetrics",
    "RosInputSynchronization",
    "RosTrackerInputAdapter",
    "RosTrackerSelection",
    "RosTrackerTickSnapshot",
    "TrackerInputIdentity",
]
