from __future__ import annotations

from types import SimpleNamespace
from typing import Any

import pytest

from wujihand.domain.dataset_recording import (
    DatasetEpisodeBoundary,
    DatasetEpisodeEvent,
    DatasetSourceMode,
    DynamicRigidBodyTruth,
    KinematicLinkTruth,
    SimulationFramePhase,
    SimulationStateFrame,
)
from wujihand_ros2.conversion import (
    dataset_episode_boundary_from_message,
    dataset_episode_boundary_to_message,
    simulation_state_frame_from_message,
    simulation_state_frame_to_message,
)


def _factory() -> Any:
    return SimpleNamespace()


def test_dataset_episode_boundary_ros_round_trip() -> None:
    boundary = DatasetEpisodeBoundary(
        run_id="episode-001",
        episode_id="episode-001",
        collection_id="mini-v1",
        event=DatasetEpisodeEvent.STOP_REQUESTED,
        reason="signal_after_complete_tick",
        host_time_ns=100,
        control_index=20,
        tick_id=20,
        simulation_time_s=0.35,
        recorder_ready=True,
        inputs_ready=True,
        references_ready=True,
        scene_settled=True,
        source_mode=DatasetSourceMode.LIVE_TELEOPERATION,
        dataset_eligible=True,
        requested_signal=2,
        effective_final_control_index=20,
    )

    message = dataset_episode_boundary_to_message(boundary, factory=_factory)

    assert dataset_episode_boundary_from_message(message) == boundary


def test_simulation_state_ros_round_trip_preserves_nested_truth_and_digest() -> None:
    body = DynamicRigidBodyTruth(
        logical_object_id="banana",
        prim_path="/World/Environment/task/banana",
        position_m=(0.1, 0.2, 0.9),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        linear_velocity_m_s=(0.0, 0.0, 0.0),
        angular_velocity_rad_s=(0.0, 0.0, 0.0),
        sleeping=False,
        kinematic=False,
        valid=True,
    )
    link = KinematicLinkTruth(
        side="left",
        logical_link_id="index_tip",
        prim_path="/World/Robots/Hand2Left/l_index_finger_tip",
        position_m=(0.0, 0.0, 1.0),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        valid=True,
    )
    frame = SimulationStateFrame.create(
        run_id="episode-001",
        episode_id="episode-001",
        control_index=4,
        tick_id=4,
        phase=SimulationFramePhase.PRE_ACTION,
        simulation_time_s=4.0 / 60.0,
        physics_boundary_index=8,
        q54_rad=(0.1,) * 54,
        qdot54_rad_s=(0.2,) * 54,
        rigid_bodies=(body,),
        kinematic_links=(link,),
        expected_rigid_body_count=1,
        expected_kinematic_link_count=1,
    )

    message = simulation_state_frame_to_message(
        frame,
        factory=_factory,
        rigid_body_factory=_factory,
        kinematic_link_factory=_factory,
    )

    assert simulation_state_frame_from_message(message) == frame


def test_dataset_ros_reader_rejects_unknown_schema() -> None:
    boundary = DatasetEpisodeBoundary(
        run_id="episode-001",
        episode_id="episode-001",
        collection_id="mini-v1",
        event=DatasetEpisodeEvent.OPENED,
        reason="opened",
        host_time_ns=1,
        control_index=None,
        tick_id=None,
        simulation_time_s=None,
        recorder_ready=False,
        inputs_ready=False,
        references_ready=False,
        scene_settled=False,
        source_mode=DatasetSourceMode.LIVE_TELEOPERATION,
        dataset_eligible=True,
    )
    message = dataset_episode_boundary_to_message(boundary, factory=_factory)
    message.schema = "wujihand.dataset_episode_boundary.v2"

    with pytest.raises(ValueError, match="schema differs"):
        dataset_episode_boundary_from_message(message)
