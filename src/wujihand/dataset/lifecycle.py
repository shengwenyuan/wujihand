"""Fail-closed one-run-one-episode lifecycle used by the live recorder.

The state machine does not own signals or ROS publishers.  The caller records a
stop request and then closes the currently executing control tick before asking
the state machine for ``STOP_REQUESTED``.  This keeps the final tick boundary
explicit and testable without signal-handler timing in the domain layer.
"""

from __future__ import annotations

from dataclasses import dataclass
from numbers import Integral

from wujihand.domain.dataset_recording import (
    DatasetEpisodeBoundary,
    DatasetEpisodeEvent,
    DatasetSourceMode,
)
from wujihand.domain.recording import validate_recording_token, validate_run_id


@dataclass(frozen=True, slots=True)
class EpisodeReadiness:
    recorder_ready: bool
    inputs_ready: bool
    references_ready: bool
    scene_settled: bool

    @property
    def ready(self) -> bool:
        return all(
            (
                self.recorder_ready,
                self.inputs_ready,
                self.references_ready,
                self.scene_settled,
            )
        )


class DatasetEpisodeLifecycle:
    """Emit the sole legal boundary sequence for one recorded run."""

    def __init__(
        self,
        *,
        run_id: str,
        collection_id: str,
        source_mode: DatasetSourceMode,
        dataset_eligible: bool,
    ) -> None:
        self.run_id = validate_run_id(run_id)
        self.collection_id = validate_recording_token(
            collection_id,
            field="collection_id",
        )
        if not isinstance(source_mode, DatasetSourceMode):
            raise ValueError("source_mode must be a DatasetSourceMode")
        if type(dataset_eligible) is not bool:
            raise ValueError("dataset_eligible must be a boolean")
        if source_mode is not DatasetSourceMode.LIVE_TELEOPERATION and dataset_eligible:
            raise ValueError("only live teleoperation can be dataset eligible")
        self.source_mode = source_mode
        self.dataset_eligible = dataset_eligible
        self._events: list[DatasetEpisodeBoundary] = []
        self._readiness = EpisodeReadiness(False, False, False, False)
        self._pending_signal: int | None = None
        self._final_control_index: int | None = None

    @property
    def boundaries(self) -> tuple[DatasetEpisodeBoundary, ...]:
        return tuple(self._events)

    @property
    def pending_stop(self) -> bool:
        return self._pending_signal is not None

    def opened(self, *, host_time_ns: int, reason: str = "run_opened") -> DatasetEpisodeBoundary:
        if self._events:
            raise RuntimeError("episode is already opened")
        return self._append(
            DatasetEpisodeEvent.OPENED,
            reason=reason,
            host_time_ns=host_time_ns,
            control_index=None,
            simulation_time_s=None,
        )

    def ready(
        self,
        *,
        host_time_ns: int,
        simulation_time_s: float,
        readiness: EpisodeReadiness,
        reason: str = "all_readiness_gates_closed",
    ) -> DatasetEpisodeBoundary:
        self._require_last(DatasetEpisodeEvent.OPENED)
        if not readiness.ready:
            raise RuntimeError("episode cannot become ready before every readiness gate")
        self._readiness = readiness
        return self._append(
            DatasetEpisodeEvent.READY,
            reason=reason,
            host_time_ns=host_time_ns,
            control_index=None,
            simulation_time_s=simulation_time_s,
        )

    def recording(
        self,
        *,
        host_time_ns: int,
        control_index: int,
        simulation_time_s: float,
        reason: str = "first_complete_candidate_tick",
    ) -> DatasetEpisodeBoundary:
        self._require_last(DatasetEpisodeEvent.READY)
        return self._append(
            DatasetEpisodeEvent.RECORDING,
            reason=reason,
            host_time_ns=host_time_ns,
            control_index=control_index,
            simulation_time_s=simulation_time_s,
        )

    def request_stop(self, signal_number: int) -> None:
        if (
            isinstance(signal_number, bool)
            or not isinstance(signal_number, Integral)
            or signal_number <= 0
        ):
            raise ValueError("stop signal must be a positive integer")
        self._require_last(DatasetEpisodeEvent.RECORDING)
        if self._pending_signal is not None:
            return
        self._pending_signal = int(signal_number)

    def complete_final_tick(
        self,
        *,
        host_time_ns: int,
        control_index: int,
        simulation_time_s: float,
        reason: str = "requested_after_complete_control_tick",
    ) -> DatasetEpisodeBoundary:
        self._require_last(DatasetEpisodeEvent.RECORDING)
        if self._pending_signal is None:
            raise RuntimeError("no stop request is pending")
        self._final_control_index = control_index
        return self._append(
            DatasetEpisodeEvent.STOP_REQUESTED,
            reason=reason,
            host_time_ns=host_time_ns,
            control_index=control_index,
            simulation_time_s=simulation_time_s,
            requested_signal=self._pending_signal,
            effective_final_control_index=control_index,
        )

    def closed(
        self,
        *,
        host_time_ns: int,
        simulation_time_s: float,
        reason: str = "consumer_and_publishers_drained",
    ) -> DatasetEpisodeBoundary:
        self._require_last(DatasetEpisodeEvent.STOP_REQUESTED)
        if self._final_control_index is None:
            raise RuntimeError("episode has no complete final control tick")
        return self._append(
            DatasetEpisodeEvent.CLOSED,
            reason=reason,
            host_time_ns=host_time_ns,
            control_index=self._final_control_index,
            simulation_time_s=simulation_time_s,
            effective_final_control_index=self._final_control_index,
        )

    def _require_last(self, expected: DatasetEpisodeEvent) -> None:
        actual = None if not self._events else self._events[-1].event
        if actual is not expected:
            raise RuntimeError(
                f"episode lifecycle expected {expected.value}, got "
                f"{None if actual is None else actual.value}"
            )

    def _append(
        self,
        event: DatasetEpisodeEvent,
        *,
        reason: str,
        host_time_ns: int,
        control_index: int | None,
        simulation_time_s: float | None,
        requested_signal: int | None = None,
        effective_final_control_index: int | None = None,
    ) -> DatasetEpisodeBoundary:
        if self._events and host_time_ns < self._events[-1].host_time_ns:
            raise ValueError("episode boundary host times must be monotonic")
        boundary = DatasetEpisodeBoundary(
            run_id=self.run_id,
            episode_id=self.run_id,
            collection_id=self.collection_id,
            event=event,
            reason=reason,
            host_time_ns=host_time_ns,
            control_index=control_index,
            tick_id=control_index,
            simulation_time_s=simulation_time_s,
            recorder_ready=self._readiness.recorder_ready,
            inputs_ready=self._readiness.inputs_ready,
            references_ready=self._readiness.references_ready,
            scene_settled=self._readiness.scene_settled,
            source_mode=self.source_mode,
            dataset_eligible=self.dataset_eligible,
            requested_signal=requested_signal,
            effective_final_control_index=effective_final_control_index,
        )
        self._events.append(boundary)
        return boundary


__all__ = ["DatasetEpisodeLifecycle", "EpisodeReadiness"]
