"""Side-neutral Tracker-to-simulated-arm application orchestration.

This module joins canonical tracking, relative pose mapping, backend-neutral
kinematics, and joint supervision.  It deliberately owns no UDP, OpenVR,
Isaac, GUI, or process lifecycle objects.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from wujihand.application.supervision import (
    JointCommandSupervisor,
    SafetyDecision,
    SafetyState,
)
from wujihand.domain import TrackedRigidBodySample
from wujihand.ports import ArmKinematicsPort, ArmKinematicsResult

from .tracker_arm import (
    InteractiveTrackerArmController,
    InteractiveTrackerArmState,
    InteractiveTrackerArmStep,
    TrackerPoseDecision,
    TrackerReferenceReadinessGate,
)


@dataclass(frozen=True, slots=True)
class TrackerArmSimulationStep:
    """One complete, backend-neutral control decision for one robot arm."""

    side: str
    state: InteractiveTrackerArmState
    mapping: TrackerPoseDecision | None
    kinematics: ArmKinematicsResult | None
    safety: SafetyDecision
    reference_epoch: int
    reference_established: bool
    reference_revoked: bool
    reason: str


class TrackerArmSimulationController:
    """Drive one simulated arm without coupling its peer or the GUI lifetime."""

    def __init__(
        self,
        *,
        side: str,
        readiness: TrackerReferenceReadinessGate,
        tracker: InteractiveTrackerArmController,
        kinematics: ArmKinematicsPort,
        supervisor: JointCommandSupervisor,
    ) -> None:
        if side not in {"left", "right"}:
            raise ValueError("side must be 'left' or 'right'")
        if not isinstance(readiness, TrackerReferenceReadinessGate):
            raise TypeError(
                "readiness must be a TrackerReferenceReadinessGate"
            )
        if not isinstance(tracker, InteractiveTrackerArmController):
            raise TypeError(
                "tracker must be an InteractiveTrackerArmController"
            )
        if not isinstance(kinematics, ArmKinematicsPort):
            raise TypeError("kinematics must implement ArmKinematicsPort")
        if not isinstance(supervisor, JointCommandSupervisor):
            raise TypeError(
                "supervisor must be a JointCommandSupervisor"
            )
        mapper = tracker.mapper
        readiness_identity = (
            readiness.stream_id,
            readiness.device_serial,
            readiness.logical_role,
            readiness.tracking_frame,
        )
        mapper_identity = (
            mapper.stream_id,
            mapper.device_serial,
            mapper.logical_role,
            mapper.tracking_frame,
        )
        if readiness_identity != mapper_identity:
            raise ValueError(
                "readiness and Tracker mapper identities must match"
            )
        if supervisor.layout.size != 7:
            raise ValueError("arm supervisor layout must contain seven joints")

        self.side = side
        self.readiness = readiness
        self.tracker = tracker
        self.kinematics = kinematics
        self.supervisor = supervisor

    def start(self, *, now_ns: int) -> SafetyDecision:
        """Begin one bounded run at the configured rest command."""

        self.readiness.reset()
        self.tracker.reset()
        return self.supervisor.arm(now_ns)

    def invalidate_reference(self) -> None:
        """Invalidate one side after a transport or tracking-setup epoch change."""

        self.readiness.reset()
        self.tracker.invalidate_reference()

    def reset(
        self,
        rest_q7_rad: Sequence[float],
        *,
        now_ns: int,
    ) -> SafetyDecision:
        """Restore one arm and require a fresh relative Tracker reference."""

        self.readiness.reset()
        self.tracker.reset()
        return self.supervisor.reset(rest_q7_rad, now_ns=now_ns)

    def step(
        self,
        samples: Sequence[TrackedRigidBodySample],
        *,
        feedback_q7_rad: Sequence[float],
        now_ns: int,
    ) -> TrackerArmSimulationStep:
        """Consume one side's drained observations and emit one q7 decision."""

        if self.supervisor.state is SafetyState.DISARMED:
            raise RuntimeError("controller must be started before step")
        observations = tuple(samples)
        if any(
            type(sample) is not TrackedRigidBodySample
            for sample in observations
        ):
            raise TypeError("samples must contain canonical Tracker observations")

        if self.tracker.requires_reference:
            return self._step_waiting_for_reference(
                observations,
                feedback_q7_rad=feedback_q7_rad,
                now_ns=now_ns,
            )

        interactive = self.tracker.advance(
            observations[-1] if observations else None,
            now_ns=now_ns,
        )
        mapping = interactive.mapping
        if mapping is None or mapping.requires_reference:
            self.readiness.reset()
            safety = self.supervisor.hold(
                now_ns=now_ns,
                reason="tracker_reference_required_hold",
            )
            return self._decision(
                interactive=interactive,
                kinematics=None,
                safety=safety,
                reference_revoked=True,
                reason=interactive.reason,
            )
        if not mapping.accepted:
            safety = self.supervisor.hold(
                now_ns=now_ns,
                reason="tracker_non_actionable_hold",
            )
            return self._decision(
                interactive=interactive,
                kinematics=None,
                safety=safety,
                reason=mapping.reason,
            )

        assert mapping.target_position_m is not None
        assert mapping.target_orientation_wxyz is not None
        result = self.kinematics.solve(
            target_position_m=mapping.target_position_m,
            target_orientation_wxyz=mapping.target_orientation_wxyz,
            warm_start_q7_rad=tuple(
                float(value) for value in self.supervisor.last_command
            ),
        )
        if result.succeeded:
            assert result.candidate_q7_rad is not None
            self.tracker.record_ik_result(True)
            safety = self.supervisor.step(
                result.candidate_q7_rad,
                now_ns=now_ns,
                input_time_ns=mapping.input_host_time_ns,
            )
            return self._decision(
                interactive=interactive,
                kinematics=result,
                safety=safety,
                reason=result.reason,
            )

        reference_revoked = self.tracker.record_ik_result(False)
        if reference_revoked:
            self.readiness.reset()
        safety = self.supervisor.hold(
            now_ns=now_ns,
            reason=(
                "ik_failure_reference_revoked_hold"
                if reference_revoked
                else "ik_failure_hold"
            ),
        )
        return self._decision(
            interactive=interactive,
            kinematics=result,
            safety=safety,
            reference_revoked=reference_revoked,
            reason=(
                f"ik_failure_reference_revoked:{result.reason}"
                if reference_revoked
                else f"ik_failure_hold:{result.reason}"
            ),
        )

    def close(self) -> SafetyDecision:
        """End this controller's bounded run at rest."""

        self.readiness.reset()
        self.tracker.reset()
        return self.supervisor.disarm()

    def _step_waiting_for_reference(
        self,
        observations: tuple[TrackedRigidBodySample, ...],
        *,
        feedback_q7_rad: Sequence[float],
        now_ns: int,
    ) -> TrackerArmSimulationStep:
        ready_sample: TrackedRigidBodySample | None = None
        readiness_reason = "waiting_for_tracker_sample"
        for sample in observations:
            readiness = self.readiness.observe(sample)
            readiness_reason = readiness.reason
            ready_sample = sample if readiness.ready else None

        if ready_sample is None:
            safety = self.supervisor.hold(
                now_ns=now_ns,
                reason="waiting_for_tracker_reference_hold",
            )
            return TrackerArmSimulationStep(
                side=self.side,
                state=self.tracker.state,
                mapping=None,
                kinematics=None,
                safety=safety,
                reference_epoch=self.tracker.reference_epoch,
                reference_established=False,
                reference_revoked=False,
                reason=readiness_reason,
            )

        current_pose = self.kinematics.forward(feedback_q7_rad)
        interactive = self.tracker.establish_reference(
            ready_sample,
            current_pose.position_m,
            current_pose.quat_wxyz,
            now_ns=now_ns,
        )
        established = interactive.mapping is not None
        safety = self.supervisor.hold(
            now_ns=now_ns,
            reason=(
                "tracker_reference_established_hold"
                if established
                else "tracker_reference_rejected_hold"
            ),
        )
        return self._decision(
            interactive=interactive,
            kinematics=None,
            safety=safety,
            reference_established=established,
            reason=interactive.reason,
        )

    def _decision(
        self,
        *,
        interactive: InteractiveTrackerArmStep,
        kinematics: ArmKinematicsResult | None,
        safety: SafetyDecision,
        reference_established: bool = False,
        reference_revoked: bool = False,
        reason: str,
    ) -> TrackerArmSimulationStep:
        return TrackerArmSimulationStep(
            side=self.side,
            state=self.tracker.state,
            mapping=interactive.mapping,
            kinematics=kinematics,
            safety=safety,
            reference_epoch=self.tracker.reference_epoch,
            reference_established=reference_established,
            reference_revoked=reference_revoked,
            reason=reason,
        )


__all__ = [
    "TrackerArmSimulationController",
    "TrackerArmSimulationStep",
]
