from __future__ import annotations

from collections.abc import Sequence

import numpy as np
import pytest

from wujihand.application.supervision import (
    JointCommandSupervisor,
    SafetyState,
)
from wujihand.application.teleoperation import (
    InteractiveTrackerArmController,
    InteractiveTrackerArmState,
    RelativeTrackerPoseMapper,
    TrackerArmSimulationController,
    TrackerReferenceReadinessGate,
)
from wujihand.domain import JointLayout, TrackedRigidBodySample, TrackingState
from wujihand.domain.pose import euler_zyx_to_quaternion_wxyz
from wujihand.ports import ArmEndEffectorPose, ArmKinematicsResult


SERIALS = {
    "left": "LHR-LEFT",
    "right": "LHR-RIGHT",
}


def sample(
    side: str,
    *,
    sequence: int,
    host_time_ns: int,
    position_m: tuple[float, float, float] = (1.0, 2.0, 3.0),
    quat_wxyz: tuple[float, float, float, float] = (
        1.0,
        0.0,
        0.0,
        0.0,
    ),
    state: TrackingState = TrackingState.RUNNING,
) -> TrackedRigidBodySample:
    valid = state is TrackingState.RUNNING
    return TrackedRigidBodySample(
        stream_id=f"vive.{side}",
        device_serial=SERIALS[side],
        logical_role=f"operator_{side}",
        producer_instance="openvr_dual_tracker",
        transport_epoch=7,
        tracking_setup_revision="workstation2_standing_v1",
        sequence=sequence,
        tracking_frame="vive_tracking",
        position_m=position_m if valid else None,
        quat_wxyz=quat_wxyz if valid else None,
        connected=True,
        pose_valid=valid,
        tracking_state=state,
        quality=1.0 if valid else None,
        host_time_ns=host_time_ns,
        device_time_ns=None,
    )


class FakeKinematics:
    def __init__(
        self,
        *,
        results: Sequence[ArmKinematicsResult] = (),
        pose: ArmEndEffectorPose | None = None,
    ) -> None:
        self.results = list(results)
        self.pose = pose or ArmEndEffectorPose(
            position_m=(0.4, 0.5, 0.6),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        )
        self.forward_inputs: list[tuple[float, ...]] = []
        self.solve_inputs: list[
            tuple[
                tuple[float, ...],
                tuple[float, ...],
                tuple[float, ...],
            ]
        ] = []

    def forward(
        self,
        q7_rad: Sequence[float],
    ) -> ArmEndEffectorPose:
        self.forward_inputs.append(tuple(q7_rad))
        return self.pose

    def solve(
        self,
        *,
        target_position_m: Sequence[float],
        target_orientation_wxyz: Sequence[float],
        warm_start_q7_rad: Sequence[float],
    ) -> ArmKinematicsResult:
        self.solve_inputs.append(
            (
                tuple(target_position_m),
                tuple(target_orientation_wxyz),
                tuple(warm_start_q7_rad),
            )
        )
        if not self.results:
            raise AssertionError("unexpected IK solve")
        return self.results.pop(0)


def ik_success(value: float = 0.1) -> ArmKinematicsResult:
    return ArmKinematicsResult(
        succeeded=True,
        solver_reported_success=True,
        candidate_q7_rad=(value,) * 7,
        position_residual_m=0.001,
        orientation_residual_rad=0.002,
        reason="ik_accepted",
    )


def ik_failure(reason: str = "position_residual_exceeded") -> ArmKinematicsResult:
    return ArmKinematicsResult(
        succeeded=False,
        solver_reported_success=True,
        candidate_q7_rad=None,
        position_residual_m=0.05,
        orientation_residual_rad=0.01,
        reason=reason,
    )


def arm_layout() -> JointLayout:
    return JointLayout(
        names=tuple(f"joint{i}" for i in range(1, 8)),
        lower=(-2.0,) * 7,
        upper=(2.0,) * 7,
        velocity=(10.0,) * 7,
    )


def controller(
    side: str,
    *,
    kinematics: FakeKinematics,
    stable_after_s: float = 0.1,
) -> TrackerArmSimulationController:
    identity = {
        "stream_id": f"vive.{side}",
        "device_serial": SERIALS[side],
        "logical_role": f"operator_{side}",
        "tracking_frame": "vive_tracking",
    }
    mapper = RelativeTrackerPoseMapper(
        **identity,
        tracker_to_workcell=(
            (0.0, 0.0, -1.0),
            (-1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
        ),
        translation_scale=1.0,
        max_translation_delta_m=0.4,
        rotation_scale=1.0,
        max_rotation_delta_rad=np.deg2rad(90.0),
        stale_after_s=0.25,
    )
    readiness = TrackerReferenceReadinessGate(
        **identity,
        stable_after_s=stable_after_s,
        max_sample_gap_s=0.25,
    )
    supervisor = JointCommandSupervisor(
        arm_layout(),
        (0.0,) * 7,
        stale_after_s=0.25,
        velocity_scale=1.0,
    )
    return TrackerArmSimulationController(
        side=side,
        readiness=readiness,
        tracker=InteractiveTrackerArmController(mapper),
        kinematics=kinematics,
        supervisor=supervisor,
    )


def establish_reference(
    subject: TrackerArmSimulationController,
    side: str,
) -> None:
    subject.start(now_ns=0)
    first = subject.step(
        [sample(side, sequence=0, host_time_ns=100_000_000)],
        feedback_q7_rad=(0.0,) * 7,
        now_ns=110_000_000,
    )
    established = subject.step(
        [sample(side, sequence=1, host_time_ns=200_000_000)],
        feedback_q7_rad=(0.0,) * 7,
        now_ns=210_000_000,
    )
    assert not first.reference_established
    assert established.reference_established


def test_waits_for_continuous_running_and_observes_every_drained_state() -> None:
    kinematics = FakeKinematics()
    subject = controller("right", kinematics=kinematics)
    subject.start(now_ns=0)

    first = subject.step(
        [sample("right", sequence=0, host_time_ns=100_000_000)],
        feedback_q7_rad=(0.0,) * 7,
        now_ns=110_000_000,
    )
    interrupted = subject.step(
        [
            sample("right", sequence=1, host_time_ns=210_000_000),
            sample(
                "right",
                sequence=2,
                host_time_ns=220_000_000,
                state=TrackingState.CALIBRATING,
            ),
        ],
        feedback_q7_rad=(0.0,) * 7,
        now_ns=230_000_000,
    )

    assert first.reason == "stabilizing_running"
    assert interrupted.reason == "tracking_calibrating"
    assert not interrupted.reference_established
    assert kinematics.forward_inputs == []
    assert interrupted.safety.state is SafetyState.DEGRADED


def test_combined_translation_and_rotation_produces_one_q7_decision() -> None:
    kinematics = FakeKinematics(results=[ik_success(0.2)])
    subject = controller("right", kinematics=kinematics)
    establish_reference(subject, "right")

    moved = subject.step(
        [
            sample(
                "right",
                sequence=2,
                host_time_ns=300_000_000,
                position_m=(1.1, 1.8, 3.2),
                quat_wxyz=euler_zyx_to_quaternion_wxyz(
                    yaw=0.1,
                    pitch=-0.2,
                    roll=0.3,
                ),
            )
        ],
        feedback_q7_rad=(0.0,) * 7,
        now_ns=310_000_000,
    )

    assert moved.state is InteractiveTrackerArmState.TRACKING
    assert moved.mapping is not None
    assert moved.mapping.accepted
    assert moved.mapping.world_delta_m == pytest.approx((-0.2, -0.1, -0.2))
    assert moved.mapping.rotation_delta_rad == pytest.approx(
        np.linalg.norm((0.3, -0.2, 0.1)),
        rel=0.02,
    )
    assert moved.kinematics is not None
    assert moved.kinematics.succeeded
    assert moved.safety.state is SafetyState.TRACKING
    assert kinematics.solve_inputs[0][2] == pytest.approx((0.0,) * 7)


def test_fifth_consecutive_ik_failure_revokes_only_current_reference() -> None:
    kinematics = FakeKinematics(results=[ik_failure()] * 5)
    subject = controller("right", kinematics=kinematics)
    establish_reference(subject, "right")

    previous_command = subject.supervisor.last_command.copy()
    decisions = []
    for index in range(5):
        decisions.append(
            subject.step(
                [
                    sample(
                        "right",
                        sequence=index + 2,
                        host_time_ns=(index + 3) * 100_000_000,
                        position_m=(1.0 + 0.01 * (index + 1), 2.0, 3.0),
                    )
                ],
                feedback_q7_rad=(0.0,) * 7,
                now_ns=(index + 3) * 100_000_000 + 10_000_000,
            )
        )

    for decision in decisions[:4]:
        assert not decision.reference_revoked
        np.testing.assert_array_equal(
            decision.safety.command,
            previous_command,
        )
    assert decisions[-1].reference_revoked
    assert decisions[-1].state is InteractiveTrackerArmState.WAITING_REFERENCE
    assert subject.tracker.requires_reference
    assert subject.tracker.ik_recoveries == 1
    np.testing.assert_array_equal(
        decisions[-1].safety.command,
        previous_command,
    )


def test_left_failure_does_not_change_right_controller() -> None:
    left = controller(
        "left",
        kinematics=FakeKinematics(results=[ik_failure()] * 5),
    )
    right = controller(
        "right",
        kinematics=FakeKinematics(results=[ik_success()] * 5),
    )
    establish_reference(left, "left")
    establish_reference(right, "right")

    for index in range(5):
        host_time_ns = (index + 3) * 100_000_000
        left.step(
            [
                sample(
                    "left",
                    sequence=index + 2,
                    host_time_ns=host_time_ns,
                )
            ],
            feedback_q7_rad=(0.0,) * 7,
            now_ns=host_time_ns + 10_000_000,
        )
        right.step(
            [
                sample(
                    "right",
                    sequence=index + 2,
                    host_time_ns=host_time_ns,
                )
            ],
            feedback_q7_rad=(0.0,) * 7,
            now_ns=host_time_ns + 10_000_000,
        )

    assert left.tracker.requires_reference
    assert not right.tracker.requires_reference
    assert left.tracker.ik_recoveries == 1
    assert right.tracker.ik_recoveries == 0


def test_transport_epoch_invalidation_is_side_local() -> None:
    left = controller("left", kinematics=FakeKinematics())
    right = controller("right", kinematics=FakeKinematics())
    establish_reference(left, "left")
    establish_reference(right, "right")

    left.invalidate_reference()

    assert left.tracker.requires_reference
    assert not right.tracker.requires_reference
