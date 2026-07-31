"""Shared application composition for native and ROS Isaac entry points."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt

from wujihand.adapters.retargeting import WujiHand2RetargetAdapter
from wujihand.adapters.storage import TrackerWorkcellMapping
from wujihand.application.supervision import JointCommandSupervisor
from wujihand.application.teleoperation import (
    DualTeleoperationCycle,
    GloveHand2ControllerSet,
    GloveHand2SimulationController,
    InteractiveTrackerArmController,
    RelativeTrackerPoseMapper,
    TrackerArmSimulationController,
    TrackerReferenceReadinessGate,
    TrackerSampleInputPort,
)
from wujihand.domain import HandSide
from wujihand.ports import HandObservationInputPort
from wujihand.specs import DualTeleoperationProfile

from wujihand.adapters.simulation.lula_arm_kinematics import (
    LulaArmKinematicsAdapter,
)
from wujihand.adapters.simulation.nero_model import NERO_JOINT_NAMES

from .dual_route_plan import DualTeleoperationRoutePlan
from .isaac_dual_scene import DualNeroHand2IsaacScene


@dataclass(slots=True)
class DualTeleoperationApplication:
    cycle: DualTeleoperationCycle
    arm_controllers: dict[str, TrackerArmSimulationController]
    hand_controllers: GloveHand2ControllerSet
    arm_indices: dict[str, npt.NDArray[np.int64]]
    _started: bool = False

    def start(self, *, now_ns: int) -> None:
        if self._started:
            raise RuntimeError("dual teleoperation application is started")
        started_arms: list[TrackerArmSimulationController] = []
        try:
            for side in sorted(self.arm_controllers):
                controller = self.arm_controllers[side]
                controller.start(now_ns=now_ns)
                started_arms.append(controller)
            self.hand_controllers.start(now_ns=now_ns)
        except Exception:
            for controller in reversed(started_arms):
                controller.close()
            raise
        self._started = True

    def close(self) -> None:
        first_error: Exception | None = None
        try:
            self.hand_controllers.close()
        except Exception as exc:
            first_error = exc
        for controller in reversed(tuple(self.arm_controllers.values())):
            try:
                controller.close()
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        self._started = False
        if first_error is not None:
            raise first_error


def build_dual_teleoperation_application(
    *,
    scene: DualNeroHand2IsaacScene,
    route_plan: DualTeleoperationRoutePlan,
    profile: DualTeleoperationProfile,
    mapping: TrackerWorkcellMapping,
    tracker_inputs: Mapping[str, TrackerSampleInputPort],
    hand_inputs: Mapping[HandSide, HandObservationInputPort],
    lula_description: Path,
    lula_urdf: Path,
) -> DualTeleoperationApplication:
    """Build the shared controllers without owning transport lifetimes."""

    from isaacsim.robot_motion.motion_generation import (  # type: ignore[import-not-found]
        LulaKinematicsSolver,
    )

    tracker_routes = {
        side: route_plan.route(f"nero_{side}", "arm_joints")
        for side in ("left", "right")
        if route_plan.route(
            f"nero_{side}",
            "arm_joints",
        ).source.kind
        == "vive_tracker"
    }
    glove_routes = {
        HandSide(side): route_plan.route(
            f"hand_{side}",
            "finger_joints",
        )
        for side in ("left", "right")
        if route_plan.route(
            f"hand_{side}",
            "finger_joints",
        ).source.kind
        == "wuji_glove"
    }
    if set(tracker_inputs) != set(tracker_routes):
        raise ValueError(
            "Tracker inputs must exactly cover live arm routes"
        )
    if set(hand_inputs) != set(glove_routes):
        raise ValueError(
            "hand inputs must exactly cover live hand routes"
        )

    runtime_by_side = {runtime.side: runtime for runtime in scene.sides}
    arm_indices = {
        side: np.asarray(
            scene.partitions[side].arm_indices_q7,
            dtype=np.int64,
        )
        for side in ("left", "right")
    }
    arm_controllers: dict[str, TrackerArmSimulationController] = {}
    for side, route in tracker_routes.items():
        runtime = runtime_by_side[side]
        solver = LulaKinematicsSolver(
            str(lula_description),
            str(lula_urdf),
        )
        if tuple(solver.get_joint_names()) != NERO_JOINT_NAMES:
            raise RuntimeError(
                f"{side} Lula cspace differs from canonical q7"
            )
        if profile.kinematics.end_effector_frame not in (
            solver.get_all_frame_names()
        ):
            raise RuntimeError(
                f"{side} Lula model does not expose "
                f"{profile.kinematics.end_effector_frame}"
            )
        solver.set_robot_base_pose(
            np.asarray(runtime.mount_pose.position_m, dtype=np.float64),
            np.asarray(runtime.mount_pose.quat_wxyz, dtype=np.float64),
        )
        kinematics = LulaArmKinematicsAdapter(
            solver=solver,
            layout=scene.arm_profiles[side].layout,
            frame_name=profile.kinematics.end_effector_frame,
            position_tolerance_m=(
                profile.kinematics.position_tolerance_m
            ),
            orientation_tolerance_rad=(
                profile.kinematics.orientation_tolerance_rad
            ),
        )
        local = route.local_binding
        if local is None:
            raise RuntimeError(f"{side} Tracker binding is missing")
        identity = {
            "stream_id": route.source.source_id,
            "device_serial": local.device_identity,
            "logical_role": route.source.logical_role,
            "tracking_frame": mapping.tracking_frame,
        }
        mapper = RelativeTrackerPoseMapper(
            **identity,
            tracker_to_workcell=mapping.tracker_to_workcell,
            translation_scale=mapping.translation_scale,
            max_translation_delta_m=mapping.max_translation_delta_m,
            rotation_scale=mapping.rotation_scale,
            max_rotation_delta_rad=mapping.max_rotation_delta_rad,
            stale_after_s=profile.tracker.stale_after_s,
            min_quality=profile.tracker.minimum_quality,
            translation_enabled=True,
            rotation_enabled=True,
        )
        feedback_q7 = scene.feedback_q27(side)[arm_indices[side]]
        scene.arm_targets[side] = feedback_q7.copy()
        arm_controllers[side] = TrackerArmSimulationController(
            side=side,
            readiness=TrackerReferenceReadinessGate(
                **identity,
                stable_after_s=profile.tracker.stable_after_s,
                max_sample_gap_s=profile.tracker.max_sample_gap_s,
            ),
            tracker=InteractiveTrackerArmController(
                mapper,
                max_consecutive_ik_failures=(
                    profile.tracker.max_consecutive_ik_failures
                ),
            ),
            kinematics=kinematics,
            supervisor=JointCommandSupervisor(
                scene.arm_profiles[side].layout,
                tuple(float(value) for value in feedback_q7),
                stale_after_s=profile.arm_supervision.stale_after_s,
                velocity_scale=profile.arm_supervision.velocity_scale,
            ),
        )

    glove_controllers: dict[
        HandSide,
        GloveHand2SimulationController,
    ] = {}
    for side, route in glove_routes.items():
        side_name = side.value
        local = route.local_binding
        if local is None:
            raise RuntimeError(f"{side_name} Glove binding is missing")
        glove_controllers[side] = GloveHand2SimulationController(
            side,
            hand_inputs[side],
            WujiHand2RetargetAdapter(
                side,
                max_observation_age_s=(
                    profile.glove.max_observation_age_s
                ),
                minimum_landmark_confidence=(
                    profile.glove.minimum_landmark_confidence
                ),
                success_landmark_confidence=(
                    profile.glove.success_landmark_confidence
                ),
            ),
            JointCommandSupervisor(
                scene.hand_profiles[side_name].layout,
                tuple(
                    float(value)
                    for value in scene.hand_targets[side_name]
                ),
                stale_after_s=profile.hand_supervision.stale_after_s,
                velocity_scale=profile.hand_supervision.velocity_scale,
            ),
        )
    hand_set = GloveHand2ControllerSet(glove_controllers)
    return DualTeleoperationApplication(
        cycle=DualTeleoperationCycle(
            arm_inputs=tracker_inputs,
            arm_controllers=arm_controllers,
            hand_controllers=hand_set,
        ),
        arm_controllers=arm_controllers,
        hand_controllers=hand_set,
        arm_indices=arm_indices,
    )


__all__ = [
    "DualTeleoperationApplication",
    "build_dual_teleoperation_application",
]
