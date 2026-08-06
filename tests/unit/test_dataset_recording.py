from __future__ import annotations

from dataclasses import replace

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


def _boundary(event: DatasetEpisodeEvent, **overrides: object) -> DatasetEpisodeBoundary:
    values: dict[str, object] = {
        "run_id": "episode-001",
        "episode_id": "episode-001",
        "collection_id": "mini-v1",
        "event": event,
        "reason": event.value,
        "host_time_ns": 100,
        "control_index": None,
        "tick_id": None,
        "simulation_time_s": None,
        "recorder_ready": True,
        "inputs_ready": True,
        "references_ready": True,
        "scene_settled": True,
        "source_mode": DatasetSourceMode.LIVE_TELEOPERATION,
        "dataset_eligible": True,
    }
    values.update(overrides)
    return DatasetEpisodeBoundary(**values)  # type: ignore[arg-type]


def test_episode_ready_requires_every_gate_and_live_source() -> None:
    ready = _boundary(DatasetEpisodeEvent.READY)

    assert ready.to_mapping()["schema"] == "wujihand.dataset_episode_boundary.v1"
    assert DatasetEpisodeBoundary.from_mapping(ready.to_mapping()) == ready
    with pytest.raises(ValueError, match="readiness gate"):
        _boundary(DatasetEpisodeEvent.READY, recorder_ready=False)
    with pytest.raises(ValueError, match="cannot be dataset eligible"):
        _boundary(
            DatasetEpisodeEvent.OPENED,
            source_mode=DatasetSourceMode.SYNTHETIC_FIXTURE,
            dataset_eligible=True,
        )

    invalid_boolean = ready.to_mapping()
    invalid_boolean["recorder_ready"] = 1
    with pytest.raises(ValueError, match="must be a boolean"):
        DatasetEpisodeBoundary.from_mapping(invalid_boolean)


def test_stop_and_closed_bind_to_one_complete_final_tick() -> None:
    stop = _boundary(
        DatasetEpisodeEvent.STOP_REQUESTED,
        control_index=17,
        tick_id=17,
        simulation_time_s=1.25,
        requested_signal=2,
        effective_final_control_index=17,
    )
    closed = _boundary(
        DatasetEpisodeEvent.CLOSED,
        control_index=17,
        tick_id=17,
        simulation_time_s=1.25,
        effective_final_control_index=17,
    )

    assert stop.effective_final_control_index == closed.effective_final_control_index
    with pytest.raises(ValueError, match="effective final"):
        replace(stop, effective_final_control_index=16)


def _body() -> DynamicRigidBodyTruth:
    return DynamicRigidBodyTruth(
        logical_object_id="banana",
        prim_path="/World/Environment/Banana",
        position_m=(0.1, 0.2, 0.3),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        linear_velocity_m_s=(0.0, 0.0, 0.0),
        angular_velocity_rad_s=(0.0, 0.0, 0.0),
        sleeping=False,
        kinematic=False,
        valid=True,
    )


def _link() -> KinematicLinkTruth:
    return KinematicLinkTruth(
        side="left",
        logical_link_id="palm",
        prim_path="/World/Robots/left/hand/palm",
        position_m=(0.0, 0.0, 0.5),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        valid=True,
    )


def test_simulation_frame_digest_closes_q54_scene_and_links() -> None:
    frame = SimulationStateFrame.create(
        run_id="episode-001",
        episode_id="episode-001",
        control_index=4,
        tick_id=4,
        phase=SimulationFramePhase.PRE_ACTION,
        simulation_time_s=2.0,
        physics_boundary_index=8,
        q54_rad=(float(index) for index in range(54)),
        qdot54_rad_s=(0.0 for _ in range(54)),
        rigid_bodies=(_body() for _ in range(1)),
        kinematic_links=(_link() for _ in range(1)),
        expected_rigid_body_count=1,
        expected_kinematic_link_count=1,
    )

    assert len(frame.q54_rad) == 54
    assert frame.payload_digest_sha256 == frame.calculate_payload_digest()
    assert frame.to_mapping()["phase"] == "pre_action"
    assert SimulationStateFrame.from_mapping(frame.to_mapping()) == frame
    with pytest.raises(ValueError, match="does not match"):
        replace(frame, payload_digest_sha256="0" * 64)


def test_post_state_reuse_requires_exact_adjacent_no_advance_closure() -> None:
    post = SimulationStateFrame.create(
        run_id="episode-001",
        episode_id="episode-001",
        control_index=4,
        tick_id=4,
        phase=SimulationFramePhase.POST_ACTION,
        simulation_time_s=2.0,
        physics_boundary_index=10,
        q54_rad=(float(index) for index in range(54)),
        qdot54_rad_s=(0.0 for _ in range(54)),
        rigid_bodies=(_body(),),
        kinematic_links=(_link(),),
        expected_rigid_body_count=1,
        expected_kinematic_link_count=1,
    )

    pre = post.as_next_pre_action(
        control_index=5,
        simulation_time_s=2.0,
        physics_boundary_index=10,
        q54_rad=post.q54_rad,
        qdot54_rad_s=post.qdot54_rad_s,
    )

    assert pre.phase is SimulationFramePhase.PRE_ACTION
    assert pre.control_index == 5
    assert pre.rigid_bodies is post.rigid_bodies
    assert pre.kinematic_links is post.kinematic_links
    with pytest.raises(ValueError, match="control-index adjacent"):
        post.as_next_pre_action(
            control_index=6,
            simulation_time_s=2.0,
            physics_boundary_index=10,
            q54_rad=post.q54_rad,
            qdot54_rad_s=post.qdot54_rad_s,
        )
    with pytest.raises(ValueError, match="live q54"):
        post.as_next_pre_action(
            control_index=5,
            simulation_time_s=2.0,
            physics_boundary_index=10,
            q54_rad=(0.0,) * 54,
            qdot54_rad_s=post.qdot54_rad_s,
        )


def test_simulation_frame_rejects_missing_inventory_and_duplicate_links() -> None:
    with pytest.raises(ValueError, match="closure count"):
        SimulationStateFrame.create(
            run_id="episode-001",
            episode_id="episode-001",
            control_index=0,
            tick_id=0,
            phase=SimulationFramePhase.PRE_ACTION,
            simulation_time_s=0.0,
            physics_boundary_index=0,
            q54_rad=(0.0,) * 54,
            qdot54_rad_s=(0.0,) * 54,
            rigid_bodies=(),
            kinematic_links=(),
            expected_rigid_body_count=1,
            expected_kinematic_link_count=0,
        )


def test_invalid_truth_uses_explicit_zero_quaternion_sentinel() -> None:
    invalid = KinematicLinkTruth(
        side="right",
        logical_link_id="index_tip",
        prim_path="/World/Robots/right/hand/index_tip",
        position_m=(0.0, 0.0, 0.0),
        quat_wxyz=(0.0, 0.0, 0.0, 0.0),
        valid=False,
    )

    assert invalid.valid is False
    with pytest.raises(ValueError, match="zero quaternion"):
        replace(invalid, quat_wxyz=(1.0, 0.0, 0.0, 0.0))
