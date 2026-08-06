from __future__ import annotations

import signal

import pytest

from wujihand.dataset.lifecycle import DatasetEpisodeLifecycle, EpisodeReadiness
from wujihand.domain.dataset_recording import DatasetEpisodeEvent, DatasetSourceMode


READY = EpisodeReadiness(True, True, True, True)


def _lifecycle() -> DatasetEpisodeLifecycle:
    return DatasetEpisodeLifecycle(
        run_id="episode-001",
        collection_id="mini-v1",
        source_mode=DatasetSourceMode.LIVE_TELEOPERATION,
        dataset_eligible=True,
    )


def test_lifecycle_preserves_complete_tick_after_signal() -> None:
    lifecycle = _lifecycle()
    lifecycle.opened(host_time_ns=1)
    lifecycle.ready(host_time_ns=2, simulation_time_s=0.0, readiness=READY)
    lifecycle.recording(host_time_ns=3, control_index=12, simulation_time_s=0.2)
    lifecycle.request_stop(signal.SIGINT)
    assert lifecycle.pending_stop
    lifecycle.complete_final_tick(
        host_time_ns=4,
        control_index=18,
        simulation_time_s=0.3,
    )
    lifecycle.closed(host_time_ns=5, simulation_time_s=0.3)

    assert tuple(item.event for item in lifecycle.boundaries) == tuple(DatasetEpisodeEvent)
    assert lifecycle.boundaries[-2].effective_final_control_index == 18
    assert lifecycle.boundaries[-1].effective_final_control_index == 18


def test_ready_gate_fails_closed() -> None:
    lifecycle = _lifecycle()
    lifecycle.opened(host_time_ns=1)
    with pytest.raises(RuntimeError, match="every readiness gate"):
        lifecycle.ready(
            host_time_ns=2,
            simulation_time_s=0.0,
            readiness=EpisodeReadiness(True, True, False, True),
        )


def test_stop_boundary_requires_signal_and_complete_tick() -> None:
    lifecycle = _lifecycle()
    lifecycle.opened(host_time_ns=1)
    lifecycle.ready(host_time_ns=2, simulation_time_s=0.0, readiness=READY)
    lifecycle.recording(host_time_ns=3, control_index=0, simulation_time_s=0.0)
    with pytest.raises(RuntimeError, match="no stop request"):
        lifecycle.complete_final_tick(
            host_time_ns=4,
            control_index=0,
            simulation_time_s=1.0 / 60.0,
        )


def test_synthetic_episode_cannot_be_dataset_eligible() -> None:
    with pytest.raises(ValueError, match="only live teleoperation"):
        DatasetEpisodeLifecycle(
            run_id="fixture-001",
            collection_id="mini-v1",
            source_mode=DatasetSourceMode.SYNTHETIC_FIXTURE,
            dataset_eligible=True,
        )
