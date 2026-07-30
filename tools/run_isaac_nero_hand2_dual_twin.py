#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Run the NV-4 native dual NERO + Hand 2 teleoperation Deployment.

The default resolves one DeploymentSpec, its single five-layer live Session,
one managed OpenVR producer, and the configured Tracker/Glove sources.  It
commands only the two simulated q27 articulations: no ROS, CAN, NERO hardware,
or Hand 2 hardware command path is present.

The historical NV-2 scripted and isolated live paths remain temporarily
available only when ``--session`` is explicit; NV-4F moves those paths to a
dedicated qualification entry point.
"""

from __future__ import annotations

import argparse
from collections import deque
from collections import Counter
from collections.abc import Callable
from datetime import datetime, timezone
from dataclasses import dataclass
from importlib.metadata import version
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, cast

import numpy as np
import numpy.typing as npt


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.application.supervision import JointCommandSupervisor
from wujihand.application.qualification import (
    FULL_SCRIPTED_Q27_SETTLING_POLICY,
    GLOVE_LIVE_Q27_READINESS_POLICY,
    Hand2QualificationTarget,
    Q27ReadinessPolicy,
    build_hand2_qualification_targets,
    partition_hand2_single_digit_indices,
    q27_window_max_delta_rad,
    qualification_gate_exit_code,
)
from wujihand.application.teleoperation import (
    GloveHand2ControllerSet,
    GloveHand2SimulationController,
    InteractiveTrackerArmController,
    InteractiveTrackerArmState,
    JointLimitMargin,
    RelativeTrackerPoseMapper,
    TrackerReferenceReadiness,
    TrackerReferenceReadinessGate,
    TrackerArmSimulationController,
    TrackerTargetMotion,
    compose_q27_hand_target,
    joint_limit_margins,
    nearest_joint_limit_margin,
    tracker_target_motion,
)
from wujihand.adapters.storage import (
    TrackerWorkcellMapping,
    load_tracker_workcell_mapping,
)
from wujihand.adapters.observability import (
    DurationRecorder,
    TimedHandObservationInputAdapter,
    TimedRetargetAdapter,
)
from wujihand.domain import HandSide
from wujihand.domain.joints import JointLayout
from wujihand.domain.pose import (
    quaternion_geodesic_distance_rad,
    rotation_matrix_to_quaternion_wxyz,
)
from wujihand.runtime import (
    ConfigRepository,
    DeploymentResolver,
    ManagedOpenVrProducer,
    ResolvedDeployment,
    SessionResolver,
    build_native_dual_runtime_plan,
    build_openvr_producer_launch,
)
from wujihand.specs import (
    AttachmentSpec,
    NativeDualTeleoperationProfile,
    PoseSpec,
)


DEFAULT_SESSION = (
    ROOT / "configs/sessions/isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml"
)
DEFAULT_DEPLOYMENT = (
    ROOT
    / "configs/deployments/"
    "isaac_nero_hand2_native_dual_live_v1.yaml"
)
DEFAULT_LOCAL_BINDING = (
    ROOT / "configs/local/workstation2_nv4_v1.yaml"
)
DEFAULT_TRACKER_MAPPING = ROOT / "configs/calibrations/vive_tracker_workcell_workstation2.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--deployment",
        type=Path,
        help=(
            "NV-4 DeploymentSpec; defaults to the native dual live "
            "deployment when --session is absent."
        ),
    )
    parser.add_argument(
        "--local-binding",
        type=Path,
        default=DEFAULT_LOCAL_BINDING,
        help="Ignored host-local Tracker/Glove/process binding.",
    )
    parser.add_argument(
        "--session",
        type=Path,
        help="Temporary explicit entry to the historical NV-2 qualification path.",
    )
    parser.add_argument(
        "--gui",
        action=argparse.BooleanOptionalAction,
        default=None,
    )
    parser.add_argument("--frames-per-phase", type=int, default=240)
    parser.add_argument("--arm-amplitude-rad", type=float, default=0.08)
    parser.add_argument("--hand-amplitude-rad", type=float, default=0.40)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument(
        "--top-screenshot",
        type=Path,
        help="Optional second screenshot using the Workcell top camera frames.",
    )
    parser.add_argument(
        "--interface-screenshot",
        type=Path,
        help=(
            "Optional close-up using the Workcell right flange-to-Hand2 interface camera frames."
        ),
    )
    parser.add_argument(
        "--glove-live",
        action="store_true",
        help="Opt in to bounded Wuji Glove control of one simulated Hand 2.",
    )
    parser.add_argument("--glove-side", choices=("left", "right"))
    parser.add_argument("--glove-serial")
    parser.add_argument("--glove-address")
    parser.add_argument("--glove-frames", type=int, default=240)
    parser.add_argument(
        "--glove-calibration-id",
        help=(
            "Explicit calibration/user revision for live provenance, for example "
            "'wuji_sdk.default_user.builtin.sdk_2026.7.21'."
        ),
    )
    parser.add_argument(
        "--verify-artifacts",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--tracker-live",
        action="store_true",
        help=(
            "Opt in to canonical Tracker pose control of only the simulated "
            "right NERO; GUI mode is interactive and headless mode is bounded."
        ),
    )
    parser.add_argument("--tracker-serial")
    parser.add_argument("--tracker-udp-port", type=int, default=49154)
    parser.add_argument(
        "--tracker-frames",
        type=int,
        default=2400,
        help="Frame limit for headless Tracker qualification; ignored by GUI live mode.",
    )
    parser.add_argument(
        "--tracker-mapping",
        type=Path,
        default=DEFAULT_TRACKER_MAPPING,
        help="Simulation-only Tracker-to-workcell calibration YAML.",
    )
    parser.add_argument(
        "--tracker-rotation",
        action="store_true",
        help="Map relative Tracker orientation as a bounded link7 target.",
    )
    parser.add_argument(
        "--tracker-freeze-translation",
        action="store_true",
        help="Hold the reference link7 position for an isolated rotation test.",
    )
    parser.add_argument(
        "--tracker-scale",
        type=float,
        help="Optional translation-scale override for the mapping profile.",
    )
    parser.add_argument(
        "--tracker-max-delta-m",
        type=float,
        help="Optional per-axis translation clamp override.",
    )
    parser.add_argument(
        "--tracker-rotation-scale",
        type=float,
        help="Optional relative rotation-scale override.",
    )
    parser.add_argument(
        "--tracker-max-rotation-deg",
        type=float,
        help="Optional shortest-angle rotation clamp override.",
    )
    parser.add_argument("--tracker-stale-s", type=float, default=0.25)
    parser.add_argument(
        "--tracker-reference-stable-s",
        type=float,
        default=0.25,
        help=(
            "Continuous canonical RUNNING duration required before a headless "
            "qualification reference may be established."
        ),
    )
    parser.add_argument(
        "--tracker-auto-reference",
        action="store_true",
        help=("Compatibility option; Tracker reference acquisition is now always automatic."),
    )
    return parser.parse_args()


ARGS = parse_args()
NATIVE_DUAL_LIVE = ARGS.session is None
if ARGS.deployment is not None and not NATIVE_DUAL_LIVE:
    raise SystemExit("--deployment and --session are mutually exclusive")
if NATIVE_DUAL_LIVE and any(
    (
        ARGS.glove_live,
        ARGS.tracker_live,
        ARGS.glove_side is not None,
        ARGS.glove_serial is not None,
        ARGS.glove_address is not None,
        ARGS.glove_calibration_id is not None,
        ARGS.tracker_serial is not None,
        ARGS.tracker_auto_reference,
        ARGS.tracker_rotation,
        ARGS.tracker_freeze_translation,
        ARGS.tracker_scale is not None,
        ARGS.tracker_max_delta_m is not None,
        ARGS.tracker_rotation_scale is not None,
        ARGS.tracker_max_rotation_deg is not None,
    )
):
    raise SystemExit(
        "NV-4 live source/mapping policy comes only from "
        "DeploymentSpec/Session/local binding; legacy live flags require "
        "an explicit --session qualification run"
    )
if ARGS.gui is None:
    ARGS.gui = NATIVE_DUAL_LIVE
if NATIVE_DUAL_LIVE and not ARGS.gui:
    raise SystemExit("NV-4 native live currently requires the Isaac GUI")
if ARGS.frames_per_phase < 1:
    raise SystemExit("--frames-per-phase must be positive")
if not 0.01 <= ARGS.arm_amplitude_rad <= 0.15:
    raise SystemExit("--arm-amplitude-rad must be between 0.01 and 0.15")
if not 0.05 <= ARGS.hand_amplitude_rad <= 0.60:
    raise SystemExit("--hand-amplitude-rad must be between 0.05 and 0.60")
if ARGS.glove_frames < 1:
    raise SystemExit("--glove-frames must be positive")
if ARGS.glove_serial is not None and ARGS.glove_address is not None:
    raise SystemExit("--glove-serial and --glove-address are mutually exclusive")
if ARGS.glove_live and ARGS.glove_side is None:
    raise SystemExit("--glove-side is required with --glove-live")
if ARGS.glove_live and ARGS.glove_calibration_id is None:
    raise SystemExit("--glove-calibration-id is required with --glove-live")
if not ARGS.glove_live and any(
    value is not None
    for value in (
        ARGS.glove_side,
        ARGS.glove_serial,
        ARGS.glove_address,
        ARGS.glove_calibration_id,
    )
):
    raise SystemExit("Glove selection options require --glove-live")
if ARGS.tracker_live and ARGS.glove_live:
    raise SystemExit("--tracker-live and --glove-live are mutually exclusive")
if ARGS.tracker_live and ARGS.tracker_serial is None:
    raise SystemExit("--tracker-serial is required with --tracker-live")
if not ARGS.tracker_live and ARGS.tracker_serial is not None:
    raise SystemExit("--tracker-serial requires --tracker-live")
if not ARGS.tracker_live and ARGS.tracker_auto_reference:
    raise SystemExit("--tracker-auto-reference requires --tracker-live")
if not ARGS.tracker_live and ARGS.tracker_rotation:
    raise SystemExit("--tracker-rotation requires --tracker-live")
if ARGS.tracker_freeze_translation and not ARGS.tracker_rotation:
    raise SystemExit("--tracker-freeze-translation requires --tracker-rotation")
if not 1 <= ARGS.tracker_udp_port <= 65535:
    raise SystemExit("--tracker-udp-port must be in [1, 65535]")
if ARGS.tracker_frames < 1:
    raise SystemExit("--tracker-frames must be positive")
if ARGS.tracker_scale is not None and not 0.0 < ARGS.tracker_scale <= 1.0:
    raise SystemExit("--tracker-scale must be in (0, 1]")
if ARGS.tracker_max_delta_m is not None and not 0.0 < ARGS.tracker_max_delta_m <= 0.15:
    raise SystemExit("--tracker-max-delta-m must be in (0, 0.15]")
if ARGS.tracker_rotation_scale is not None and not 0.0 < ARGS.tracker_rotation_scale <= 1.0:
    raise SystemExit("--tracker-rotation-scale must be in (0, 1]")
if ARGS.tracker_max_rotation_deg is not None and not 0.0 < ARGS.tracker_max_rotation_deg <= 45.0:
    raise SystemExit("--tracker-max-rotation-deg must be in (0, 45]")
if not 0.05 <= ARGS.tracker_stale_s <= 1.0:
    raise SystemExit("--tracker-stale-s must be in [0.05, 1.0]")
if not 0.10 <= ARGS.tracker_reference_stable_s <= 2.0:
    raise SystemExit("--tracker-reference-stable-s must be in [0.10, 2.0]")

RESOLVED_DEPLOYMENT: ResolvedDeployment | None = None
NATIVE_PROFILE: NativeDualTeleoperationProfile | None = None
TRACKER_MAPPING: TrackerWorkcellMapping | None = None
TRACKER_MAPPING_PATH: Path | None = None
TRACKER_TRANSLATION_SCALE: float | None = None
TRACKER_MAX_TRANSLATION_DELTA_M: float | None = None
TRACKER_ROTATION_SCALE: float | None = None
TRACKER_MAX_ROTATION_DELTA_RAD: float | None = None
if NATIVE_DUAL_LIVE:
    deployment_path = (
        DEFAULT_DEPLOYMENT
        if ARGS.deployment is None
        else ARGS.deployment
    )
    try:
        RESOLVED_DEPLOYMENT = DeploymentResolver(ROOT).resolve(
            deployment_path,
            local_binding=ARGS.local_binding,
            verify_artifacts=ARGS.verify_artifacts,
        )
    except (FileNotFoundError, ValueError) as exc:
        raise SystemExit(f"NV-4 deployment preflight failed: {exc}") from exc
    RESOLVED = RESOLVED_DEPLOYMENT.session
    profile_path = RESOLVED.session.runtime.compatibility_profile
    if profile_path is None:
        raise SystemExit(
            "NV-4 live Session is missing its compatibility profile"
        )
    NATIVE_PROFILE = (
        ConfigRepository(ROOT).load_native_dual_teleoperation_profile(
            profile_path
        )
    )
    TRACKER_MAPPING = RESOLVED_DEPLOYMENT.mapping
    TRACKER_MAPPING_PATH = (
        ROOT / RESOLVED_DEPLOYMENT.mapping_path
    ).resolve()
    TRACKER_TRANSLATION_SCALE = TRACKER_MAPPING.translation_scale
    TRACKER_MAX_TRANSLATION_DELTA_M = (
        TRACKER_MAPPING.max_translation_delta_m
    )
    TRACKER_ROTATION_SCALE = TRACKER_MAPPING.rotation_scale
    TRACKER_MAX_ROTATION_DELTA_RAD = (
        TRACKER_MAPPING.max_rotation_delta_rad
    )
else:
    assert ARGS.session is not None
    RESOLVED = SessionResolver(ROOT).resolve(
        ARGS.session,
        verify_artifacts=ARGS.verify_artifacts,
    )
    if (
        RESOLVED.session.backend != "isaac"
        or RESOLVED.session.runtime_role != "simulation"
    ):
        raise SystemExit("NV-2 runner requires an Isaac simulation Session")
    if RESOLVED.session.runtime.transport_contract is not None:
        raise SystemExit(
            "NV-2 scripted runner must not declare a transport contract"
        )

if ARGS.tracker_live:
    TRACKER_MAPPING_PATH = (
        ARGS.tracker_mapping if ARGS.tracker_mapping.is_absolute() else ROOT / ARGS.tracker_mapping
    ).resolve()
    try:
        TRACKER_MAPPING = load_tracker_workcell_mapping(TRACKER_MAPPING_PATH)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    if TRACKER_MAPPING.workcell_frame != RESOLVED.workcell.world_frame:
        raise SystemExit(
            "Tracker mapping workcell_frame differs from resolved Workcell "
            f"world_frame: {TRACKER_MAPPING.workcell_frame!r} != "
            f"{RESOLVED.workcell.world_frame!r}"
        )
    TRACKER_TRANSLATION_SCALE = (
        TRACKER_MAPPING.translation_scale if ARGS.tracker_scale is None else ARGS.tracker_scale
    )
    TRACKER_MAX_TRANSLATION_DELTA_M = (
        TRACKER_MAPPING.max_translation_delta_m
        if ARGS.tracker_max_delta_m is None
        else ARGS.tracker_max_delta_m
    )
    TRACKER_ROTATION_SCALE = (
        TRACKER_MAPPING.rotation_scale
        if ARGS.tracker_rotation_scale is None
        else ARGS.tracker_rotation_scale
    )
    TRACKER_MAX_ROTATION_DELTA_RAD = math.radians(
        TRACKER_MAPPING.max_rotation_delta_deg
        if ARGS.tracker_max_rotation_deg is None
        else ARGS.tracker_max_rotation_deg
    )

from isaacsim import SimulationApp  # type: ignore[import-not-found]


simulation_app = SimulationApp(
    {
        "headless": not ARGS.gui,
        "width": 1280,
        "height": 800,
        "anti_aliasing": 0,
    }
)

from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics  # type: ignore[import-not-found]

from isaacsim.core.api import World  # type: ignore[import-not-found]
from isaacsim.core.api.objects import FixedCuboid  # type: ignore[import-not-found]
from isaacsim.core.prims import Articulation  # type: ignore[import-not-found]
from isaacsim.core.utils.stage import (  # type: ignore[import-not-found]
    add_reference_to_stage,
)
from isaacsim.core.utils.viewports import (  # type: ignore[import-not-found]
    set_camera_view,
)
from isaacsim.robot_motion.motion_generation import (  # type: ignore[import-not-found]
    LulaKinematicsSolver,
)
from wujihand.adapters.transport import UdpTrackingSampleReceiver
from wujihand.adapters.simulation import (
    Hand2ModelProfile,
    LulaArmKinematicsAdapter,
    NeroHand2AttachmentConfig,
    NeroHand2AttachmentHandles,
    NeroHand2DofPartition,
    NeroLinkGeometryAlignmentHandles,
    apply_isaac_nero_link_geometry_alignment,
    author_nero_hand2_attachment,
    discover_nero_hand2_dofs,
    load_hand2_model_profile,
    load_nero_dual_tabletop_qualification_profile,
    load_nero_link_geometry_alignment,
)
from wujihand.adapters.simulation.nero_model import (
    NERO_JOINT_NAMES,
    NeroModelProfile,
    load_nero_model_profile,
)
from wujihand.adapters.input import (
    KeyboardResetInputAdapter,
    NoHandSkeletonFrameAvailable,
    WujiGloveHandSkeletonAdapter,
)
from wujihand.adapters.retargeting import WujiHand2RetargetAdapter
from wujihand.integrity import sha256_file


PHYSICS_HZ = 120
FEEDBACK_LIMIT_TOLERANCE_RAD = 0.02
MOTION_ISOLATION_TOLERANCE_RAD = 0.03
LIVE_HAND_RESPONSE_MIN_RAD = 0.01
SCRIPTED_RESPONSE_MIN_RAD = 0.01
SCRIPTED_RESPONSE_FRACTION = 0.10
RESET_INITIAL_TOLERANCE_RAD = 0.08
GLOVE_LIVE_ARM_FEEDBACK_TOLERANCE_RAD = 0.05
TRACKER_REFERENCE_WAIT_S = 10.0
TRACKER_RESPONSE_MIN_RAD = 0.005
TRACKER_ROTATION_RESPONSE_MIN_RAD = math.radians(1.0)
TRACKER_LEFT_FEEDBACK_TOLERANCE_RAD = 0.03
TRACKER_HAND_FEEDBACK_TOLERANCE_RAD = 0.10
TRACKER_STREAM_ID = "vive.right"
TRACKER_LOGICAL_ROLE = "operator_right"
TRACKER_PRODUCER_INSTANCE = "openvr_single_tracker"
TRACKER_TRANSPORT_EPOCH = 0
TRACKER_SETUP_REVISION = "steamvr_standing_unqualified"
TRACKER_MAX_CONSECUTIVE_IK_FAILURES = 5
TRACKER_DIAGNOSTIC_HISTORY_LIMIT = 64
NERO_LULA_DESCRIPTION = ROOT / "configs/profiles/agilex_nero_lula_kinematics_v1.yaml"
SCREENSHOT_CAMERA_PRIM_PATH = "/OmniverseKit_Persp"
OBLIQUE_CAMERA_EYE_FRAME = "simulation_nominal_camera_oblique_eye"
OBLIQUE_CAMERA_TARGET_FRAME = "simulation_nominal_camera_oblique_target"
TOP_CAMERA_EYE_FRAME = "simulation_nominal_camera_top_eye"
TOP_CAMERA_TARGET_FRAME = "simulation_nominal_camera_top_target"
INTERFACE_CAMERA_EYE_FRAME = "simulation_nominal_camera_right_interface_eye"
INTERFACE_CAMERA_TARGET_FRAME = "simulation_nominal_camera_right_interface_target"


@dataclass(frozen=True, slots=True)
class Pose:
    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class SideRuntime:
    side: str
    arm_instance_id: str
    hand_instance_id: str
    arm_asset: Path
    hand_asset: Path
    arm_profile: Path
    hand_profile: Path
    arm_prim_path: str
    hand_prim_path: str
    mount_pose: Pose
    attachment: AttachmentSpec


@dataclass(frozen=True, slots=True)
class ScriptedHandPhase:
    """One q20 target plus the feedback snapshots used to qualify it."""

    target: Hand2QualificationTarget
    baseline_key: str
    command_key: str
    commanded_profile_indices: tuple[int, ...]
    commanded_runtime_indices: tuple[int, ...]


def _quat_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def _rotation_matrix(
    quaternion: tuple[float, float, float, float],
) -> npt.NDArray[np.float64]:
    w, x, y, z = quaternion
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _compose(parent: Pose, child: PoseSpec | Pose) -> Pose:
    parent_position = np.asarray(parent.position_m, dtype=np.float64)
    child_position = np.asarray(child.position_m, dtype=np.float64)
    position = parent_position + _rotation_matrix(parent.quat_wxyz) @ child_position
    quaternion = _quat_multiply(parent.quat_wxyz, child.quat_wxyz)
    return Pose(
        position_m=cast(tuple[float, float, float], tuple(position.tolist())),
        quat_wxyz=quaternion,
    )


def _workcell_frame_pose(frame_id: str, cache: dict[str, Pose]) -> Pose:
    if frame_id in cache:
        return cache[frame_id]
    frame = next(item for item in RESOLVED.workcell.frames if item.frame_id == frame_id)
    parent = _workcell_frame_pose(frame.parent, cache)
    result = _compose(parent, frame.transform)
    cache[frame_id] = result
    return result


def _workcell_pose(frame_id: str, local: PoseSpec) -> Pose:
    cache = {
        RESOLVED.workcell.world_frame: Pose(
            position_m=(0.0, 0.0, 0.0),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        )
    }
    return _compose(_workcell_frame_pose(frame_id, cache), local)


def _workcell_frame_position(frame_id: str) -> tuple[float, float, float]:
    cache = {
        RESOLVED.workcell.world_frame: Pose(
            position_m=(0.0, 0.0, 0.0),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        )
    }
    return _workcell_frame_pose(frame_id, cache).position_m


def _profile_path(instance_id: str, group_id: str) -> Path:
    instance = RESOLVED.instance(instance_id)
    profile = instance.asset.control_group(group_id).joint_profile
    if profile is None:
        raise RuntimeError(f"{instance_id}/{group_id} has no joint profile")
    path = ROOT / profile
    if not path.is_file():
        raise RuntimeError(f"joint profile not found: {path}")
    return path


def _side_runtimes() -> tuple[SideRuntime, SideRuntime]:
    result: list[SideRuntime] = []
    for attachment in RESOLVED.assembly.attachments:
        arm = RESOLVED.instance(attachment.parent.instance)
        hand = RESOLVED.instance(attachment.child.instance)
        side = hand.asset.side
        if side not in {"left", "right"}:
            raise RuntimeError("Hand 2 attachment must declare an explicit side")
        if arm.asset.product != "agilex_nero" or hand.asset.product != "wuji_hand_2":
            raise RuntimeError("NV-2 attachment must connect NERO to Hand 2")
        if (
            RESOLVED.assembly.instance(arm.instance_id).role != "arm"
            or RESOLVED.assembly.instance(hand.instance_id).role != "end_effector"
        ):
            raise RuntimeError("NV-2 attachment roles must be arm -> end_effector")
        if attachment.parent.frame != arm.asset.frame_name(
            "tool_flange"
        ) or attachment.child.frame != hand.asset.frame_name("base"):
            raise RuntimeError("NV-2 attachment must connect the semantic tool flange to hand base")
        if arm.binding.loader != "usd" or hand.binding.loader != "usd":
            raise RuntimeError("NV-2 physical twin requires USD bindings")
        if arm.artifact is None or hand.artifact is None:
            raise RuntimeError("NV-2 instances require source-locked USD artifacts")
        mount = RESOLVED.workcell.mount(RESOLVED.session.mount_for(arm.instance_id))
        mount_pose = _workcell_pose(mount.frame, mount.transform)
        title = side.capitalize()
        result.append(
            SideRuntime(
                side=side,
                arm_instance_id=arm.instance_id,
                hand_instance_id=hand.instance_id,
                arm_asset=arm.artifact.absolute_path,
                hand_asset=hand.artifact.absolute_path,
                arm_profile=_profile_path(arm.instance_id, "arm_joints"),
                hand_profile=_profile_path(hand.instance_id, "finger_joints"),
                arm_prim_path=f"/World/Robots/Nero{title}",
                hand_prim_path=f"/World/Robots/Hand2{title}",
                mount_pose=mount_pose,
                attachment=attachment,
            )
        )
    if {item.side for item in result} != {"left", "right"} or len(result) != 2:
        raise RuntimeError("NV-2 assembly must contain one left and one right attachment")
    return cast(
        tuple[SideRuntime, SideRuntime],
        tuple(sorted(result, key=lambda item: item.side)),
    )


SIDES = _side_runtimes()
alignment_profile_references = {
    RESOLVED.instance(runtime.arm_instance_id).binding.compatibility_profile for runtime in SIDES
}
if None in alignment_profile_references or len(alignment_profile_references) != 1:
    raise RuntimeError(
        "both NERO Binding instances must reference one link geometry alignment profile"
    )
ALIGNMENT_PROFILE_REFERENCE = cast(str, next(iter(alignment_profile_references)))
ALIGNMENT_PROFILE_PATH = ROOT / ALIGNMENT_PROFILE_REFERENCE
ALIGNMENT_PROFILE = load_nero_link_geometry_alignment(ALIGNMENT_PROFILE_PATH)
NERO_LULA_SOURCE_URDF = (ROOT / ALIGNMENT_PROFILE.source_urdf_path).resolve()
if RESOLVED.session.runtime.compatibility_profile is None:
    raise RuntimeError("NV-2 tabletop qualification requires a Session compatibility profile")
TABLETOP_PROFILE_PATH = (
    ROOT
    / (
        RESOLVED.session.runtime.compatibility_profile
        if NATIVE_PROFILE is None
        else NATIVE_PROFILE.base_qualification.path
    )
)
TABLETOP_PROFILE = load_nero_dual_tabletop_qualification_profile(TABLETOP_PROFILE_PATH)


def _set_world_pose(prim: Any, pose: Pose) -> None:
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(Gf.Quatd(*pose.quat_wxyz))
    matrix.SetTranslateOnly(Gf.Vec3d(*pose.position_m))
    xformable.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(matrix)


def _one_prim(
    stage: Any,
    *,
    prefix: str,
    name: str | None = None,
    articulation_root: bool = False,
    rigid_body: bool = False,
    fixed_joint: bool = False,
) -> Any:
    matches = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(prefix.rstrip("/") + "/"):
            continue
        if name is not None and prim.GetName() != name:
            continue
        if articulation_root and not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        if rigid_body and not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        if fixed_joint and not prim.IsA(UsdPhysics.FixedJoint):
            continue
        matches.append(prim)
    if len(matches) != 1:
        raise RuntimeError(
            f"expected one prim below {prefix} for name={name!r}, "
            f"articulation_root={articulation_root}, rigid_body={rigid_body}, "
            f"fixed_joint={fixed_joint}; found {[str(item.GetPath()) for item in matches]}"
        )
    return matches[0]


def _dof_paths(articulation: Articulation) -> tuple[str, ...]:
    raw = getattr(articulation, "_dof_paths", None)
    paths = np.asarray(raw, dtype=object)
    if paths.ndim != 2 or paths.shape[0] != 1:
        raise RuntimeError(f"expected one articulation DOF-path row, got {paths.shape}")
    result = tuple(str(path) for path in paths[0])
    if len(result) != len(articulation.dof_names):
        raise RuntimeError("Isaac DOF path/name counts differ")
    return result


def _positions(articulation: Articulation) -> npt.NDArray[np.float64]:
    values = np.asarray(articulation.get_joint_positions(), dtype=np.float64)
    if values.shape != (1, 27) or not np.isfinite(values).all():
        raise RuntimeError(f"invalid q27 feedback shape/value: {values.shape}")
    return cast(npt.NDArray[np.float64], values[0].copy())


def _world_axis(
    stage: Any,
    prim_path: str,
    local_axis_xyz: tuple[float, float, float],
) -> npt.NDArray[np.float64]:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid() or not prim.IsA(UsdGeom.Xformable):
        raise RuntimeError(f"axis measurement prim is invalid: {prim_path}")
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    direction = matrix.TransformDir(Gf.Vec3d(*local_axis_xyz))
    result = np.asarray(tuple(direction), dtype=np.float64)
    norm = float(np.linalg.norm(result))
    if not np.isfinite(result).all() or not np.isclose(norm, 1.0, atol=1e-6):
        raise RuntimeError(
            f"axis measurement is not a finite unit vector for {prim_path}: {result.tolist()}"
        )
    return result / norm


def _world_position(stage: Any, prim_path: str) -> npt.NDArray[np.float64]:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid() or not prim.IsA(UsdGeom.Xformable):
        raise RuntimeError(f"position measurement prim is invalid: {prim_path}")
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    result = np.asarray(tuple(matrix.ExtractTranslation()), dtype=np.float64)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise RuntimeError(
            f"position measurement is not a finite vector for {prim_path}: {result.tolist()}"
        )
    return result


def _world_point(
    stage: Any,
    prim_path: str,
    local_point_xyz: tuple[float, float, float],
) -> npt.NDArray[np.float64]:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid() or not prim.IsA(UsdGeom.Xformable):
        raise RuntimeError(f"point measurement prim is invalid: {prim_path}")
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    point = matrix.Transform(Gf.Vec3d(*local_point_xyz))
    result = np.asarray(tuple(point), dtype=np.float64)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise RuntimeError(
            f"point measurement is not a finite vector for {prim_path}: {result.tolist()}"
        )
    return result


def _step(world: World, frames: int, *, render: bool) -> None:
    for _ in range(frames):
        world.step(render=render)


def _full_target(
    partition: NeroHand2DofPartition,
    arm_q7: npt.NDArray[np.float64],
    hand_q20: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    baseline = np.zeros(27, dtype=np.float64)
    baseline[np.asarray(partition.arm_indices_q7)] = arm_q7
    return compose_q27_hand_target(
        baseline,
        partition.hand_indices_q20,
        hand_q20,
    )


def _capture_screenshot(
    world: World,
    path: Path,
    *,
    eye_m: tuple[float, float, float],
    target_m: tuple[float, float, float],
) -> dict[str, object]:
    import omni.kit.renderer_capture  # type: ignore[import-not-found]
    from omni.kit.viewport.utility import (  # type: ignore[import-not-found]
        capture_viewport_to_file,
        get_active_viewport,
    )

    path.parent.mkdir(parents=True, exist_ok=True)
    path.unlink(missing_ok=True)
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("active Isaac viewport is unavailable")
    viewport.camera_path = SCREENSHOT_CAMERA_PRIM_PATH
    set_camera_view(
        eye=np.asarray(eye_m),
        target=np.asarray(target_m),
        camera_prim_path=SCREENSHOT_CAMERA_PRIM_PATH,
        viewport_api=viewport,
    )
    _step(world, 3, render=True)
    camera_prim = world.scene.stage.GetPrimAtPath(SCREENSHOT_CAMERA_PRIM_PATH)
    if not camera_prim.IsValid() or not camera_prim.IsA(UsdGeom.Camera):
        raise RuntimeError(
            f"active screenshot camera prim is invalid: {SCREENSHOT_CAMERA_PRIM_PATH}"
        )
    camera_world_transform = UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    capture = capture_viewport_to_file(viewport, file_path=str(path))
    captured = simulation_app.run_coroutine(capture.wait_for_result(completion_frames=30))
    omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
    if not captured or not path.is_file():
        raise RuntimeError("Isaac viewport capture did not complete")
    return {
        "active_viewport_camera_path": str(viewport.camera_path),
        "camera_prim_valid": True,
        "camera_world_transform_row_major": [
            [float(camera_world_transform[row][column]) for column in range(4)] for row in range(4)
        ],
        "camera_world_position": [
            float(value) for value in camera_world_transform.ExtractTranslation()
        ],
        "render_settle_frames": 3,
    }


def _run_glove_live(
    world: World,
    *,
    articulations: dict[str, Articulation],
    partitions: dict[str, NeroHand2DofPartition],
    hand_profiles: dict[str, Hand2ModelProfile],
    hand_targets: dict[str, npt.NDArray[np.float64]],
    apply_targets: Callable[[], None],
) -> tuple[dict[str, object], dict[str, dict[str, list[float]]]]:
    side = HandSide(ARGS.glove_side)
    side_name = side.value
    other_side = "right" if side is HandSide.LEFT else "left"
    input_adapter = TimedHandObservationInputAdapter(
        WujiGloveHandSkeletonAdapter(
            side,
            source_id=f"wuji_glove.{side_name}.isaac_live",
            calibration_id=ARGS.glove_calibration_id,
            transform_id="wuji_glove.hand_skeleton.v1",
            serial_number=ARGS.glove_serial,
            address=ARGS.glove_address,
            device_name=f"nv2_glove_{side_name}",
        )
    )
    retargeter = TimedRetargetAdapter(WujiHand2RetargetAdapter(side))
    supervisor = JointCommandSupervisor(
        hand_profiles[side_name].layout,
        hand_targets[side_name].tolist(),
        stale_after_s=0.25,
        velocity_scale=1.0,
    )
    controller = GloveHand2SimulationController(
        side,
        input_adapter,
        retargeter,
        supervisor,
    )

    feedback = {
        "glove_live_before": {
            item: _positions(articulations[item]).tolist() for item in ("left", "right")
        }
    }
    initial_hand_target = hand_targets[side_name].copy()
    max_supervised_command_delta_rad = 0.0
    selected_hand_indices = np.asarray(
        partitions[side_name].hand_indices_q20,
        dtype=np.int64,
    )
    selected_arm_indices = np.asarray(
        partitions[side_name].arm_indices_q7,
        dtype=np.int64,
    )
    initial_selected_feedback = np.asarray(
        feedback["glove_live_before"][side_name],
        dtype=np.float64,
    )
    initial_other_feedback = np.asarray(
        feedback["glove_live_before"][other_side],
        dtype=np.float64,
    )
    max_selected_hand_feedback_delta_rad = 0.0
    max_selected_arm_feedback_delta_rad = 0.0
    max_other_side_feedback_delta_rad = 0.0
    accepted_frames = 0
    empty_polls = 0
    rejected_frames = 0
    degraded_intents = 0
    clamped_commands = 0
    rate_limited_commands = 0
    supervision_reasons: dict[str, int] = {}
    rejection_reasons: dict[str, int] = {}
    minimum_landmark_confidences: list[float] = []
    last_retarget_model_id: str | None = None
    last_retarget_config_id: str | None = None
    command_apply_timing = DurationRecorder()
    simulation_step_timing = DurationRecorder()
    feedback_read_timing = DurationRecorder()
    frame_processing_timing = DurationRecorder()
    frame_period_ns = round(1_000_000_000 / PHYSICS_HZ)
    started_ns = time.monotonic_ns()
    controller_armed_ns: int | None = None
    first_intent_ns: int | None = None
    first_command_change_ns: int | None = None
    last_tick_ns = started_ns
    deadline_ns = started_ns
    print("GLOVE LIVE CONNECTING: opening Wuji hand_skeleton input.", flush=True)
    try:
        controller.start(now_ns=started_ns)
        controller_armed_ns = time.monotonic_ns()
        print(
            "GLOVE LIVE ARMED: waiting for the first canonical skeleton.",
            flush=True,
        )
        for _ in range(ARGS.glove_frames):
            deadline_ns += frame_period_ns
            frame_started_ns = time.monotonic_ns()
            tick_ns = max(time.monotonic_ns(), last_tick_ns + 1)
            try:
                step = controller.poll(now_ns=tick_ns)
            except NoHandSkeletonFrameAvailable:
                empty_polls += 1
                step = controller.advance_without_observation(now_ns=tick_ns)
            except (RuntimeError, ValueError) as exc:
                step = controller.reject_observation(
                    now_ns=tick_ns,
                    reason=f"input_rejected:{type(exc).__name__}",
                )
            if step.intent is not None:
                accepted_frames += 1
                if first_intent_ns is None:
                    first_intent_ns = time.monotonic_ns()
                    print(
                        "GLOVE LIVE ACTIVE: first q20 intent accepted; "
                        "operator motion now controls the simulated Hand2.",
                        flush=True,
                    )
                last_retarget_model_id = step.intent.retarget_model_id
                last_retarget_config_id = step.intent.retarget_config_id
                minimum_landmark_confidences.append(step.intent.retarget_confidence)
                if step.intent.retarget_status.value == "degraded":
                    degraded_intents += 1
            elif step.rejection_reason is not None:
                rejected_frames += 1
                rejection_reasons[step.rejection_reason] = (
                    rejection_reasons.get(step.rejection_reason, 0) + 1
                )

            decision = step.decision
            supervision_reasons[decision.reason] = supervision_reasons.get(decision.reason, 0) + 1
            clamped_commands += int(decision.position_clamped)
            rate_limited_commands += int(decision.rate_limited)
            max_supervised_command_delta_rad = max(
                max_supervised_command_delta_rad,
                float(np.max(np.abs(decision.command - initial_hand_target))),
            )
            if (
                first_command_change_ns is None
                and max_supervised_command_delta_rad >= LIVE_HAND_RESPONSE_MIN_RAD
            ):
                first_command_change_ns = time.monotonic_ns()
            hand_targets[side_name] = decision.command.copy()
            stage_started_ns = time.monotonic_ns()
            apply_targets()
            command_apply_timing.observe_ns(time.monotonic_ns() - stage_started_ns)
            stage_started_ns = time.monotonic_ns()
            world.step(render=ARGS.gui)
            simulation_step_timing.observe_ns(time.monotonic_ns() - stage_started_ns)
            stage_started_ns = time.monotonic_ns()
            selected_feedback = _positions(articulations[side_name])
            other_feedback = _positions(articulations[other_side])
            feedback_read_timing.observe_ns(time.monotonic_ns() - stage_started_ns)
            max_selected_hand_feedback_delta_rad = max(
                max_selected_hand_feedback_delta_rad,
                float(
                    np.max(
                        np.abs(
                            selected_feedback[selected_hand_indices]
                            - initial_selected_feedback[selected_hand_indices]
                        )
                    )
                ),
            )
            max_selected_arm_feedback_delta_rad = max(
                max_selected_arm_feedback_delta_rad,
                float(
                    np.max(
                        np.abs(
                            selected_feedback[selected_arm_indices]
                            - initial_selected_feedback[selected_arm_indices]
                        )
                    )
                ),
            )
            max_other_side_feedback_delta_rad = max(
                max_other_side_feedback_delta_rad,
                float(np.max(np.abs(other_feedback - initial_other_feedback))),
            )
            frame_processing_timing.observe_ns(time.monotonic_ns() - frame_started_ns)
            last_tick_ns = tick_ns
            remaining_s = (deadline_ns - time.monotonic_ns()) / 1_000_000_000
            if remaining_s > 0.0:
                time.sleep(remaining_s)
    finally:
        controller.close()

    finished_ns = time.monotonic_ns()
    feedback["glove_live_after"] = {
        item: _positions(articulations[item]).tolist() for item in ("left", "right")
    }
    selector_kind = (
        "serial"
        if ARGS.glove_serial is not None
        else "address"
        if ARGS.glove_address is not None
        else "handedness"
    )
    selector_value = (
        ARGS.glove_serial
        if ARGS.glove_serial is not None
        else ARGS.glove_address
        if ARGS.glove_address is not None
        else side_name
    )
    report: dict[str, object] = {
        "enabled": True,
        "side": side_name,
        "calibration_id": ARGS.glove_calibration_id,
        "degraded_intent_policy": (
            "complete finite skeletons are admitted; minimum landmark "
            "confidence <0.90 is DEGRADED and >=0.90 is SUCCESS"
        ),
        "confidence_policy": {
            "aggregation": "minimum_of_21_landmarks",
            "hard_rejection_floor": 0.0,
            "success_threshold": 0.9,
            "low_confidence_action": "admit_as_degraded_intent",
        },
        "supervisor_policy": {
            "stale_after_s": 0.25,
            "velocity_scale": 1.0,
        },
        "selector": {
            "kind": selector_kind,
            "value": selector_value,
        },
        "simulation_frames": ARGS.glove_frames,
        "accepted_skeleton_frames": accepted_frames,
        "empty_polls": empty_polls,
        "rejected_skeleton_frames": rejected_frames,
        "degraded_intents": degraded_intents,
        "accepted_minimum_landmark_confidence": (
            {
                "minimum": min(minimum_landmark_confidences),
                "mean": (sum(minimum_landmark_confidences) / len(minimum_landmark_confidences)),
                "maximum": max(minimum_landmark_confidences),
            }
            if minimum_landmark_confidences
            else None
        ),
        "rejection_reasons": rejection_reasons,
        "supervision_reasons": supervision_reasons,
        "position_clamped_commands": clamped_commands,
        "rate_limited_commands": rate_limited_commands,
        "max_supervised_command_delta_rad": max_supervised_command_delta_rad,
        "selected_hand_max_feedback_delta_rad": (max_selected_hand_feedback_delta_rad),
        "selected_arm_max_feedback_delta_rad": (max_selected_arm_feedback_delta_rad),
        "last_retarget_model_id": last_retarget_model_id,
        "last_retarget_config_id": last_retarget_config_id,
        "wall_duration_s": (finished_ns - started_ns) / 1_000_000_000,
        "lifecycle": {
            "connect_to_armed_ms": (
                None
                if controller_armed_ns is None
                else (controller_armed_ns - started_ns) / 1_000_000.0
            ),
            "armed_to_first_intent_ms": (
                None
                if controller_armed_ns is None or first_intent_ns is None
                else (first_intent_ns - controller_armed_ns) / 1_000_000.0
            ),
            "armed_to_first_command_change_ms": (
                None
                if controller_armed_ns is None or first_command_change_ns is None
                else (first_command_change_ns - controller_armed_ns) / 1_000_000.0
            ),
        },
        "effective_loop_rate_hz": (
            ARGS.glove_frames / max((finished_ns - started_ns) / 1_000_000_000, 1e-9)
        ),
        "host_stage_timing": {
            "clock_domain": "host_monotonic",
            "input_poll": input_adapter.recorder.summary().to_report(),
            "retarget": retargeter.recorder.summary().to_report(),
            "command_apply": command_apply_timing.summary().to_report(),
            "simulation_step_render": simulation_step_timing.summary().to_report(),
            "feedback_read": feedback_read_timing.summary().to_report(),
            "frame_processing": frame_processing_timing.summary().to_report(),
            "scope_note": (
                "host call durations only; Wuji device timestamps are not "
                "host-comparable, so sensor acquisition latency is excluded"
            ),
        },
        "other_side_max_feedback_delta_rad": max_other_side_feedback_delta_rad,
    }
    return report, feedback


def _run_glove_live_qualification(
    world: World,
    *,
    articulations: dict[str, Articulation],
    partitions: dict[str, NeroHand2DofPartition],
    arm_profiles: dict[str, NeroModelProfile],
    hand_profiles: dict[str, Hand2ModelProfile],
    arm_targets: dict[str, npt.NDArray[np.float64]],
    hand_targets: dict[str, npt.NDArray[np.float64]],
    apply_targets: Callable[[], None],
    readiness_record: dict[str, object],
    readiness_feedback: dict[str, dict[str, list[float]]],
    articulation_root_paths: tuple[str, ...],
    arm_drive_runtime: dict[str, dict[str, list[float]]],
) -> int:
    """Run the isolated, startup-bounded Glove -> simulated Hand2 smoke."""

    side_name = cast(str, ARGS.glove_side)
    other_side = "left" if side_name == "right" else "right"
    initial_arm_commands = {side: arm_targets[side].copy() for side in ("left", "right")}
    initial_other_hand_command = hand_targets[other_side].copy()
    glove_report, live_feedback = _run_glove_live(
        world,
        articulations=articulations,
        partitions=partitions,
        hand_profiles=hand_profiles,
        hand_targets=hand_targets,
        apply_targets=apply_targets,
    )

    accounted_frames = (
        cast(int, glove_report["accepted_skeleton_frames"])
        + cast(int, glove_report["empty_polls"])
        + cast(int, glove_report["rejected_skeleton_frames"])
    )
    arm_commands_held = all(
        np.array_equal(arm_targets[side], initial_arm_commands[side]) for side in ("left", "right")
    )
    other_hand_command_held = bool(
        np.array_equal(hand_targets[other_side], initial_other_hand_command)
    )
    feedback_values = np.asarray(
        [
            values
            for phase in (*readiness_feedback.values(), *live_feedback.values())
            for values in phase.values()
        ],
        dtype=np.float64,
    )
    checks = {
        "bounded_run_completed": accounted_frames == ARGS.glove_frames,
        "fresh_skeleton_received": (cast(int, glove_report["accepted_skeleton_frames"]) > 0),
        "supervised_command_changed": (
            cast(float, glove_report["max_supervised_command_delta_rad"])
            >= LIVE_HAND_RESPONSE_MIN_RAD
        ),
        "selected_hand_responded": (
            cast(float, glove_report["selected_hand_max_feedback_delta_rad"])
            >= LIVE_HAND_RESPONSE_MIN_RAD
        ),
        "arm_commands_held": arm_commands_held,
        "other_hand_command_held": other_hand_command_held,
        "selected_arm_feedback_bounded": (
            cast(float, glove_report["selected_arm_max_feedback_delta_rad"])
            <= GLOVE_LIVE_ARM_FEEDBACK_TOLERANCE_RAD
        ),
        "other_side_feedback_bounded": (
            cast(float, glove_report["other_side_max_feedback_delta_rad"])
            <= MOTION_ISOLATION_TOLERANCE_RAD
        ),
        "readiness_wait_bounded": (
            cast(int, readiness_record["windows_run"])
            <= GLOVE_LIVE_Q27_READINESS_POLICY.maximum_windows
        ),
        "all_observed_feedback_finite": bool(np.isfinite(feedback_values).all()),
    }
    passed = all(checks.values())

    screenshot_runtime: dict[str, object] = {}
    oblique_camera_eye = _workcell_frame_position(OBLIQUE_CAMERA_EYE_FRAME)
    oblique_camera_target = _workcell_frame_position(OBLIQUE_CAMERA_TARGET_FRAME)
    if ARGS.screenshot is not None:
        screenshot_runtime = _capture_screenshot(
            world,
            ARGS.screenshot,
            eye_m=oblique_camera_eye,
            target_m=oblique_camera_target,
        )
    top_screenshot_runtime: dict[str, object] = {}
    top_camera_eye = _workcell_frame_position(TOP_CAMERA_EYE_FRAME)
    top_camera_target = _workcell_frame_position(TOP_CAMERA_TARGET_FRAME)
    if ARGS.top_screenshot is not None:
        top_screenshot_runtime = _capture_screenshot(
            world,
            ARGS.top_screenshot,
            eye_m=top_camera_eye,
            target_m=top_camera_target,
        )

    report = {
        "schema": "wujihand.isaac_glove_hand2_live_smoke.v1",
        "scope": (
            f"simulation-only {side_name} Hand2 q20; both NERO arms and "
            f"{other_side} Hand2 command-held; no ROS, CAN, NERO hardware, "
            "or Hand2 hardware"
        ),
        "session_id": RESOLVED.session.session_id,
        "session_hash": RESOLVED.session_hash,
        "five_layer_configuration_modified": False,
        "isaac_distribution": version("isaacsim"),
        "physics_hz": PHYSICS_HZ,
        "run_mode": "glove_live_only",
        "scripted_qualification_executed": False,
        "live_readiness": readiness_record,
        "topology": {
            "articulation_root_paths": list(articulation_root_paths),
            "partitions": {
                side: {
                    "arm_indices_q7": list(partitions[side].arm_indices_q7),
                    "hand_indices_q20": list(partitions[side].hand_indices_q20),
                }
                for side in ("left", "right")
            },
        },
        "qualification_runtime": {
            "arm_drive_gains": arm_drive_runtime,
            "selected_arm_feedback_tolerance_rad": (GLOVE_LIVE_ARM_FEEDBACK_TOLERANCE_RAD),
            "other_side_feedback_tolerance_rad": MOTION_ISOLATION_TOLERANCE_RAD,
            "arm_layout_ids": {side: arm_profiles[side].layout_id for side in ("left", "right")},
        },
        "command_ownership": {
            "selected_hand": side_name,
            "arm_commands_held": arm_commands_held,
            "other_hand_command_held": other_hand_command_held,
        },
        "glove_live": glove_report,
        "feedback": {
            "readiness": readiness_feedback,
            "live": live_feedback,
        },
        "checks": checks,
        "screenshot": {
            "path": (None if ARGS.screenshot is None else ARGS.screenshot.resolve().as_posix()),
            "camera_prim_path": SCREENSHOT_CAMERA_PRIM_PATH,
            "camera_eye_frame": OBLIQUE_CAMERA_EYE_FRAME,
            "camera_target_frame": OBLIQUE_CAMERA_TARGET_FRAME,
            **screenshot_runtime,
        },
        "top_screenshot": {
            "path": (
                None if ARGS.top_screenshot is None else ARGS.top_screenshot.resolve().as_posix()
            ),
            "camera_prim_path": SCREENSHOT_CAMERA_PRIM_PATH,
            "camera_eye_frame": TOP_CAMERA_EYE_FRAME,
            "camera_target_frame": TOP_CAMERA_TARGET_FRAME,
            **top_screenshot_runtime,
        },
        "passed": passed,
    }
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if ARGS.report is not None:
        ARGS.report.parent.mkdir(parents=True, exist_ok=True)
        ARGS.report.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return qualification_gate_exit_code(passed)


def _nero_link7_pose(
    solver: LulaKinematicsSolver,
    q7: npt.NDArray[np.float64],
) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    position_m, rotation = solver.compute_forward_kinematics("link7", q7)
    position = np.asarray(position_m, dtype=np.float64)
    rotation_matrix = np.asarray(rotation, dtype=np.float64)
    if (
        position.shape != (3,)
        or not np.isfinite(position).all()
        or rotation_matrix.shape != (3, 3)
        or not np.isfinite(rotation_matrix).all()
    ):
        raise RuntimeError("NERO Lula FK returned an invalid link7 pose")
    return position, rotation_matrix_to_quaternion_wxyz(rotation_matrix)


def _target_motion_diagnostic(
    motion: TrackerTargetMotion | None,
) -> dict[str, float] | None:
    if motion is None:
        return None
    return {
        "sample_interval_s": motion.sample_interval_s,
        "translation_step_m": motion.translation_step_m,
        "translation_speed_m_s": motion.translation_speed_m_s,
        "rotation_step_rad": motion.rotation_step_rad,
        "rotation_step_deg": math.degrees(motion.rotation_step_rad),
        "rotation_speed_rad_s": motion.rotation_speed_rad_s,
        "rotation_speed_deg_s": math.degrees(motion.rotation_speed_rad_s),
    }


def _joint_limit_margin_diagnostic(
    margin: JointLimitMargin,
) -> dict[str, object]:
    return {
        "joint_name": margin.joint_name,
        "position_rad": margin.position_rad,
        "position_deg": math.degrees(margin.position_rad),
        "lower_limit_rad": margin.lower_limit_rad,
        "lower_limit_deg": math.degrees(margin.lower_limit_rad),
        "upper_limit_rad": margin.upper_limit_rad,
        "upper_limit_deg": math.degrees(margin.upper_limit_rad),
        "nearest_limit": margin.nearest_limit,
        "margin_to_nearest_limit_rad": margin.margin_to_nearest_limit_rad,
        "margin_to_nearest_limit_deg": math.degrees(margin.margin_to_nearest_limit_rad),
        "within_limits": margin.within_limits,
    }


def _q7_limit_diagnostic(
    layout: JointLayout,
    q7: npt.NDArray[np.float64],
) -> dict[str, object]:
    margins = joint_limit_margins(layout, q7)
    nearest = min(
        margins,
        key=lambda item: item.margin_to_nearest_limit_rad,
    )
    return {
        "q7_rad": [float(value) for value in q7],
        "q7_deg": [math.degrees(float(value)) for value in q7],
        "all_within_limits": all(item.within_limits for item in margins),
        "nearest": _joint_limit_margin_diagnostic(nearest),
        "per_joint": [_joint_limit_margin_diagnostic(item) for item in margins],
    }


def _print_tracker_operator_instruction() -> None:
    if ARGS.tracker_freeze_translation:
        print(
            "rotation-only：保持 link7 参考位置；依次绕身体右向、前向、"
            "上向轴小幅旋转。仅右 NERO 应响应。",
            flush=True,
        )
    elif ARGS.tracker_rotation:
        print(
            "依次小幅测试：左右、前后、上下；再绕身体右向、前向、上向轴旋转。仅右 NERO 应响应。",
            flush=True,
        )
    else:
        print(
            "现在依次小幅移动 Tracker：左右、前后、上下；仅右 NERO 应响应。",
            flush=True,
        )


def _run_tracker_interactive(
    world: World,
    *,
    articulations: dict[str, Articulation],
    partitions: dict[str, NeroHand2DofPartition],
    solver: LulaKinematicsSolver,
    mapper: RelativeTrackerPoseMapper,
    supervisor: JointCommandSupervisor,
    arm_targets: dict[str, npt.NDArray[np.float64]],
    apply_targets: Callable[[], None],
) -> int:
    """Run a persistent GUI session independent from Tracker control state."""

    assert TRACKER_MAPPING is not None
    assert TRACKER_MAPPING_PATH is not None
    assert TRACKER_TRANSLATION_SCALE is not None
    assert TRACKER_MAX_TRANSLATION_DELTA_M is not None
    assert TRACKER_ROTATION_SCALE is not None
    assert TRACKER_MAX_ROTATION_DELTA_RAD is not None

    right_arm_indices = np.asarray(
        partitions["right"].arm_indices_q7,
        dtype=np.int64,
    )
    controller = InteractiveTrackerArmController(
        mapper,
        max_consecutive_ik_failures=(TRACKER_MAX_CONSECUTIVE_IK_FAILURES),
    )
    frame_period_ns = round(1_000_000_000 / PHYSICS_HZ)
    started_ns = time.monotonic_ns()
    next_deadline_ns = started_ns
    last_tick_ns = max(0, started_ns - 1)
    completed_frames = 0
    accepted_samples = 0
    held_frames = 0
    waiting_frames = 0
    ik_successes = 0
    ik_failures = 0
    translation_clamped_samples = 0
    rotation_clamped_samples = 0
    max_world_delta_m = 0.0
    max_target_rotation_delta_rad = 0.0
    max_target_translation_step_m = 0.0
    max_target_translation_speed_m_s = 0.0
    max_target_rotation_step_rad = 0.0
    max_target_rotation_speed_rad_s = 0.0
    supervisor_position_clamped_frames = 0
    supervisor_rate_limited_frames = 0
    supervision_reasons: dict[str, int] = {}
    reset_event_count = 0
    reset_cause_counts: dict[str, int] = {}
    reset_events: deque[dict[str, object]] = deque(maxlen=TRACKER_DIAGNOSTIC_HISTORY_LIMIT)
    reference_events: deque[dict[str, object]] = deque(maxlen=TRACKER_DIAGNOSTIC_HISTORY_LIMIT)
    ik_failure_events: deque[dict[str, object]] = deque(maxlen=TRACKER_DIAGNOSTIC_HISTORY_LIMIT)
    closest_successful_limit_margin: JointLimitMargin | None = None
    previous_target_position_m: npt.NDArray[np.float64] | None = None
    previous_target_orientation_wxyz: npt.NDArray[np.float64] | None = None
    previous_target_time_ns: int | None = None
    pending_reference_cause = "startup"
    pending_reference_reason = "initial_reference"
    supervisor_armed = False
    last_mapping_reason: str | None = None
    last_logged_state: InteractiveTrackerArmState | None = None
    termination_reason = "operator_closed"

    print(
        "TRACKER_INTERACTIVE_WAITING_REFERENCE "
        "policy=first_fresh_running gui_lifetime=operator_controlled",
        flush=True,
    )
    with UdpTrackingSampleReceiver(
        ARGS.tracker_udp_port,
        stream_id=TRACKER_STREAM_ID,
        device_serial=ARGS.tracker_serial,
        logical_role=TRACKER_LOGICAL_ROLE,
        producer_instance=TRACKER_PRODUCER_INSTANCE,
        transport_epoch=TRACKER_TRANSPORT_EPOCH,
        tracking_setup_revision=TRACKER_SETUP_REVISION,
        tracking_frame=TRACKER_MAPPING.tracking_frame,
    ) as receiver:
        try:
            while simulation_app.is_running():
                next_deadline_ns += frame_period_ns
                tick_ns = max(time.monotonic_ns(), last_tick_ns + 1)
                sample = receiver.receive_latest(now_ns=tick_ns)
                mapping = None
                ik_success: bool | None = None

                if controller.requires_reference:
                    if sample is not None:
                        current_right = _positions(articulations["right"])
                        current_q7 = current_right[right_arm_indices]
                        (
                            reference_position_m,
                            reference_orientation_wxyz,
                        ) = _nero_link7_pose(solver, current_q7)
                        reference_step = controller.establish_reference(
                            sample,
                            reference_position_m,
                            reference_orientation_wxyz,
                            now_ns=tick_ns,
                        )
                        mapping = reference_step.mapping
                        last_mapping_reason = reference_step.reason
                        if mapping is not None:
                            assert mapping.target_position_m is not None
                            assert mapping.target_orientation_wxyz is not None
                            assert mapping.input_host_time_ns is not None
                            previous_target_position_m = np.asarray(
                                mapping.target_position_m,
                                dtype=np.float64,
                            )
                            previous_target_orientation_wxyz = np.asarray(
                                mapping.target_orientation_wxyz,
                                dtype=np.float64,
                            )
                            previous_target_time_ns = mapping.input_host_time_ns
                            reference_events.append(
                                {
                                    "frame": completed_frames,
                                    "reference_epoch": (reference_step.reference_epoch),
                                    "trigger_cause": (pending_reference_cause),
                                    "trigger_reason": (pending_reference_reason),
                                    "tracker_sequence": sample.sequence,
                                    "tracker_host_time_ns": (sample.host_time_ns),
                                    "tracker_reference_position_m": (
                                        None
                                        if sample.position_m is None
                                        else list(sample.position_m)
                                    ),
                                    "tracker_reference_orientation_wxyz": (
                                        None
                                        if sample.quat_wxyz is None
                                        else list(sample.quat_wxyz)
                                    ),
                                    "tracker_tracking_state": sample.tracking_state.value,
                                    "tracker_quality": sample.quality,
                                    "reference_target_position_m": list(mapping.target_position_m),
                                    "reference_target_orientation_wxyz": (
                                        list(mapping.target_orientation_wxyz)
                                    ),
                                    "current_q7": _q7_limit_diagnostic(
                                        supervisor.layout,
                                        current_q7,
                                    ),
                                }
                            )
                            pending_reference_cause = "tracking"
                            pending_reference_reason = "reference_active"
                            if not supervisor_armed:
                                supervisor.arm(tick_ns)
                                supervisor_armed = True
                            print(
                                "TRACKER_REFERENCE_READY "
                                f"epoch={reference_step.reference_epoch} "
                                f"serial={sample.device_serial} "
                                f"position_m={list(sample.position_m or ())} "
                                "policy=first_fresh_running "
                                f"mapping_id={TRACKER_MAPPING.mapping_id}",
                                flush=True,
                            )
                            if reference_step.reference_epoch == 1:
                                _print_tracker_operator_instruction()
                    if controller.requires_reference:
                        waiting_frames += 1
                else:
                    control_step = controller.advance(
                        sample,
                        now_ns=tick_ns,
                    )
                    mapping = control_step.mapping
                    last_mapping_reason = control_step.reason
                    if mapping is not None and mapping.requires_reference:
                        reset_event_count += 1
                        reset_cause = "tracker_reference_loss"
                        reset_cause_counts[reset_cause] = reset_cause_counts.get(reset_cause, 0) + 1
                        reset_events.append(
                            {
                                "frame": completed_frames,
                                "reference_epoch": (control_step.reference_epoch),
                                "cause": reset_cause,
                                "reason": mapping.reason,
                                "tracker_sample": (
                                    None
                                    if sample is None
                                    else {
                                        "sequence": sample.sequence,
                                        "host_time_ns": (sample.host_time_ns),
                                        "connected": sample.connected,
                                        "pose_valid": sample.pose_valid,
                                        "tracking_state": (sample.tracking_state.value),
                                        "quality": sample.quality,
                                    }
                                ),
                            }
                        )
                        pending_reference_cause = reset_cause
                        pending_reference_reason = mapping.reason
                        previous_target_position_m = None
                        previous_target_orientation_wxyz = None
                        previous_target_time_ns = None
                    if mapping is not None and not mapping.requires_reference:
                        target_motion: TrackerTargetMotion | None = None
                        if mapping.accepted:
                            accepted_samples += 1
                            translation_clamped_samples += int(mapping.translation_clamped)
                            rotation_clamped_samples += int(mapping.rotation_clamped)
                            assert mapping.target_position_m is not None
                            assert mapping.target_orientation_wxyz is not None
                            assert mapping.input_host_time_ns is not None
                            target_position_m = np.asarray(
                                mapping.target_position_m,
                                dtype=np.float64,
                            )
                            target_orientation_wxyz = np.asarray(
                                mapping.target_orientation_wxyz,
                                dtype=np.float64,
                            )
                            if (
                                previous_target_position_m is not None
                                and previous_target_orientation_wxyz is not None
                                and previous_target_time_ns is not None
                            ):
                                target_motion = tracker_target_motion(
                                    previous_target_position_m,
                                    previous_target_orientation_wxyz,
                                    previous_target_time_ns,
                                    target_position_m,
                                    target_orientation_wxyz,
                                    mapping.input_host_time_ns,
                                )
                                max_target_translation_step_m = max(
                                    max_target_translation_step_m,
                                    target_motion.translation_step_m,
                                )
                                max_target_translation_speed_m_s = max(
                                    max_target_translation_speed_m_s,
                                    target_motion.translation_speed_m_s,
                                )
                                max_target_rotation_step_rad = max(
                                    max_target_rotation_step_rad,
                                    target_motion.rotation_step_rad,
                                )
                                max_target_rotation_speed_rad_s = max(
                                    max_target_rotation_speed_rad_s,
                                    target_motion.rotation_speed_rad_s,
                                )
                            previous_target_position_m = target_position_m
                            previous_target_orientation_wxyz = target_orientation_wxyz
                            previous_target_time_ns = mapping.input_host_time_ns
                        else:
                            held_frames += 1
                        if mapping.world_delta_m is not None:
                            max_world_delta_m = max(
                                max_world_delta_m,
                                float(np.linalg.norm(mapping.world_delta_m)),
                            )
                        if mapping.rotation_delta_rad is not None:
                            max_target_rotation_delta_rad = max(
                                max_target_rotation_delta_rad,
                                mapping.rotation_delta_rad,
                            )

                        assert mapping.target_position_m is not None
                        assert mapping.target_orientation_wxyz is not None
                        assert mapping.input_host_time_ns is not None
                        solution, solver_reported_success = solver.compute_inverse_kinematics(
                            "link7",
                            np.asarray(
                                mapping.target_position_m,
                                dtype=np.float64,
                            ),
                            np.asarray(
                                mapping.target_orientation_wxyz,
                                dtype=np.float64,
                            ),
                            warm_start=supervisor.last_command,
                            position_tolerance=0.002,
                            orientation_tolerance=0.02,
                        )
                        solver_candidate_q7: npt.NDArray[np.float64] | None
                        try:
                            solver_candidate_q7 = supervisor.layout.validate_vector(
                                np.asarray(
                                    solution,
                                    dtype=np.float64,
                                )
                            )
                        except (
                            TypeError,
                            ValueError,
                            OverflowError,
                        ):
                            ik_success = False
                            solver_candidate_q7 = None
                            candidate_q7 = supervisor.last_command.copy()
                        else:
                            ik_success = bool(solver_reported_success)
                            candidate_q7 = solver_candidate_q7

                        if ik_success:
                            ik_successes += 1
                            controller.record_ik_result(True)
                            nearest_margin = nearest_joint_limit_margin(
                                supervisor.layout,
                                candidate_q7,
                            )
                            if (
                                closest_successful_limit_margin is None
                                or nearest_margin.margin_to_nearest_limit_rad
                                < closest_successful_limit_margin.margin_to_nearest_limit_rad
                            ):
                                closest_successful_limit_margin = nearest_margin
                            supervision = supervisor.step(
                                candidate_q7,
                                now_ns=tick_ns,
                                input_time_ns=mapping.input_host_time_ns,
                            )
                            supervisor_position_clamped_frames += int(supervision.position_clamped)
                            supervisor_rate_limited_frames += int(supervision.rate_limited)
                            supervision_reasons[supervision.reason] = (
                                supervision_reasons.get(
                                    supervision.reason,
                                    0,
                                )
                                + 1
                            )
                            arm_targets["right"] = supervision.command.copy()
                            apply_targets()
                        else:
                            ik_failures += 1
                            current_right = _positions(articulations["right"])
                            current_q7 = current_right[right_arm_indices]
                            failure_event: dict[str, object] = {
                                "frame": completed_frames,
                                "reference_epoch": (controller.reference_epoch),
                                "consecutive_failure": (controller.consecutive_ik_failures + 1),
                                "mapping_reason": mapping.reason,
                                "target_position_m": list(mapping.target_position_m),
                                "target_orientation_wxyz": list(mapping.target_orientation_wxyz),
                                "world_delta_m": (
                                    None
                                    if mapping.world_delta_m is None
                                    else list(mapping.world_delta_m)
                                ),
                                "translation_clamped": (mapping.translation_clamped),
                                "rotation_clamped": (mapping.rotation_clamped),
                                "target_motion": (_target_motion_diagnostic(target_motion)),
                                "solver_reported_success": bool(solver_reported_success),
                                "solver_candidate_valid": (solver_candidate_q7 is not None),
                                "current_feedback_q7": (
                                    _q7_limit_diagnostic(
                                        supervisor.layout,
                                        current_q7,
                                    )
                                ),
                                "last_command_q7": _q7_limit_diagnostic(
                                    supervisor.layout,
                                    supervisor.last_command,
                                ),
                            }
                            if solver_candidate_q7 is not None:
                                (
                                    candidate_position_m,
                                    candidate_orientation_wxyz,
                                ) = _nero_link7_pose(
                                    solver,
                                    solver_candidate_q7,
                                )
                                failure_event["solver_candidate_q7"] = _q7_limit_diagnostic(
                                    supervisor.layout,
                                    solver_candidate_q7,
                                )
                                candidate_orientation_error_rad = (
                                    quaternion_geodesic_distance_rad(
                                        candidate_orientation_wxyz,
                                        mapping.target_orientation_wxyz,
                                    )
                                )
                                failure_event["solver_candidate_residual"] = {
                                    "position_m": float(
                                        np.linalg.norm(
                                            candidate_position_m
                                            - np.asarray(
                                                mapping.target_position_m,
                                                dtype=np.float64,
                                            )
                                        )
                                    ),
                                    "orientation_rad": candidate_orientation_error_rad,
                                    "orientation_deg": math.degrees(
                                        candidate_orientation_error_rad
                                    ),
                                }
                            ik_failure_events.append(failure_event)
                            if controller.record_ik_result(False):
                                reset_event_count += 1
                                reset_cause = "five_consecutive_ik_failures"
                                reset_cause_counts[reset_cause] = (
                                    reset_cause_counts.get(
                                        reset_cause,
                                        0,
                                    )
                                    + 1
                                )
                                reset_events.append(
                                    {
                                        "frame": completed_frames,
                                        "reference_epoch": (controller.reference_epoch),
                                        "cause": reset_cause,
                                        "reason": reset_cause,
                                        "last_failure": failure_event,
                                    }
                                )
                                pending_reference_cause = reset_cause
                                pending_reference_reason = reset_cause
                                previous_target_position_m = None
                                previous_target_orientation_wxyz = None
                                previous_target_time_ns = None
                                nearest_current = nearest_joint_limit_margin(
                                    supervisor.layout,
                                    current_q7,
                                )
                                print(
                                    "TRACKER_CONTROL_WAITING_REFERENCE "
                                    "reason=five_consecutive_ik_failures "
                                    "target_translation_speed_m_s="
                                    f"{0.0 if target_motion is None else target_motion.translation_speed_m_s:.6f} "
                                    "nearest_current_joint="
                                    f"{nearest_current.joint_name} "
                                    "nearest_current_margin_deg="
                                    f"{math.degrees(nearest_current.margin_to_nearest_limit_rad):.3f} "
                                    "gui=running",
                                    flush=True,
                                )

                if controller.state is not last_logged_state:
                    print(
                        "TRACKER_CONTROL_STATE "
                        f"state={controller.state.value} "
                        f"reason={last_mapping_reason or 'startup'} "
                        "gui=running",
                        flush=True,
                    )
                    last_logged_state = controller.state

                world.step(render=True)
                completed_frames += 1
                if not simulation_app.is_running():
                    termination_reason = "operator_closed"
                    break
                if completed_frames % 60 == 0:
                    print(
                        f"tracker_frame={completed_frames:06d} "
                        f"state={controller.state.value} "
                        f"reference_epoch={controller.reference_epoch} "
                        f"ik={'n/a' if ik_success is None else ('ok' if ik_success else 'hold')} "
                        f"last_reason={last_mapping_reason}",
                        flush=True,
                    )

                last_tick_ns = tick_ns
                remaining_s = (next_deadline_ns - time.monotonic_ns()) / 1_000_000_000
                if remaining_s > 0.0:
                    time.sleep(remaining_s)
        except KeyboardInterrupt:
            termination_reason = "operator_interrupt"

        report = {
            "schema": "wujihand.isaac_tracker_right_nero_interactive.v2",
            "scope": (
                "simulation-only right NERO q7 interactive control; "
                "no ROS, CAN, NERO hardware, or Hand 2 hardware"
            ),
            "session_id": RESOLVED.session.session_id,
            "session_hash": RESOLVED.session_hash,
            "session_mode": "interactive_gui",
            "five_layer_configuration_modified": False,
            "tracker": {
                "serial": ARGS.tracker_serial,
                "stream_id": TRACKER_STREAM_ID,
                "logical_role": TRACKER_LOGICAL_ROLE,
                "tracking_frame": TRACKER_MAPPING.tracking_frame,
                "udp_endpoint": f"127.0.0.1:{ARGS.tracker_udp_port}",
                "reference_policy": "first_fresh_running",
                "reference_epochs": controller.reference_epoch,
                "accepted_samples": accepted_samples,
                "held_frames": held_frames,
                "waiting_frames": waiting_frames,
                "receiver_accepted_drains": receiver.accepted,
                "receiver_rejected_datagrams": receiver.rejected,
                "final_control_state": controller.state.value,
            },
            "mapping": {
                "profile": str(TRACKER_MAPPING_PATH),
                "mapping_id": TRACKER_MAPPING.mapping_id,
                "tracker_to_workcell": [list(row) for row in TRACKER_MAPPING.tracker_to_workcell],
                "translation_scale": TRACKER_TRANSLATION_SCALE,
                "translation_enabled": (not ARGS.tracker_freeze_translation),
                "max_translation_delta_each_axis_m": (TRACKER_MAX_TRANSLATION_DELTA_M),
                "translation_clamped_samples": (translation_clamped_samples),
                "max_world_delta_norm_m": max_world_delta_m,
                "rotation_enabled": ARGS.tracker_rotation,
                "rotation_scale": TRACKER_ROTATION_SCALE,
                "max_rotation_delta_deg": math.degrees(TRACKER_MAX_ROTATION_DELTA_RAD),
                "rotation_clamped_samples": rotation_clamped_samples,
                "max_target_rotation_delta_deg": math.degrees(max_target_rotation_delta_rad),
            },
            "kinematics": {
                "solver": "Isaac Sim 6.0.1 LulaKinematicsSolver",
                "end_effector_frame": "link7",
                "ik_successes": ik_successes,
                "ik_failures": ik_failures,
                "ik_reference_recoveries": controller.ik_recoveries,
            },
            "diagnostics": {
                "history_limit": TRACKER_DIAGNOSTIC_HISTORY_LIMIT,
                "reset_semantics": (
                    "relative Tracker reference epoch reset; this does "
                    "not itself command the arm rest pose"
                ),
                "reset_event_count": reset_event_count,
                "reset_cause_counts": reset_cause_counts,
                "reset_events_retained": list(reset_events),
                "reference_events_retained": list(reference_events),
                "ik_failure_event_count": ik_failures,
                "ik_failure_events_retained": list(ik_failure_events),
                "target_motion_maxima": {
                    "translation_step_m": (max_target_translation_step_m),
                    "translation_speed_m_s": (max_target_translation_speed_m_s),
                    "rotation_step_rad": (max_target_rotation_step_rad),
                    "rotation_step_deg": math.degrees(max_target_rotation_step_rad),
                    "rotation_speed_rad_s": (max_target_rotation_speed_rad_s),
                    "rotation_speed_deg_s": math.degrees(max_target_rotation_speed_rad_s),
                },
                "closest_successful_solution_to_joint_limit": (
                    None
                    if closest_successful_limit_margin is None
                    else _joint_limit_margin_diagnostic(closest_successful_limit_margin)
                ),
                "supervisor": {
                    "position_clamped_frames": (supervisor_position_clamped_frames),
                    "rate_limited_frames": (supervisor_rate_limited_frames),
                    "reasons": supervision_reasons,
                },
                "joint_limit_source": (
                    "canonical simulation JointLayout from the pinned "
                    "NERO URDF profile; not physical-machine safety "
                    "limits"
                ),
            },
            "runtime": {
                "requested_frames": None,
                "completed_frames": completed_frames,
                "termination_reason": termination_reason,
                "last_mapping_reason": last_mapping_reason,
                "wall_duration_s": (time.monotonic_ns() - started_ns) / 1_000_000_000,
            },
            "qualification_evaluated": False,
            "passed": None,
        }
        encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
        if ARGS.report is not None:
            ARGS.report.parent.mkdir(parents=True, exist_ok=True)
            ARGS.report.write_text(encoded, encoding="utf-8")
        print(encoded, end="")
    return 0


def _run_tracker_live(
    world: World,
    *,
    articulations: dict[str, Articulation],
    partitions: dict[str, NeroHand2DofPartition],
    arm_profiles: dict[str, NeroModelProfile],
    arm_targets: dict[str, npt.NDArray[np.float64]],
    hand_targets: dict[str, npt.NDArray[np.float64]],
    apply_targets: Callable[[], None],
) -> int:
    """Run the isolated Tracker -> right NERO simulation smoke."""

    if (
        TRACKER_MAPPING is None
        or TRACKER_MAPPING_PATH is None
        or TRACKER_TRANSLATION_SCALE is None
        or TRACKER_MAX_TRANSLATION_DELTA_M is None
        or TRACKER_ROTATION_SCALE is None
        or TRACKER_MAX_ROTATION_DELTA_RAD is None
    ):
        raise RuntimeError("Tracker mapping was not resolved before Isaac startup")
    if sha256_file(NERO_LULA_SOURCE_URDF) != ALIGNMENT_PROFILE.source_urdf_sha256:
        raise RuntimeError("source-locked NERO URDF hash drifted")
    if not NERO_LULA_DESCRIPTION.is_file():
        raise RuntimeError(f"NERO Lula descriptor not found: {NERO_LULA_DESCRIPTION}")

    right_runtime = next(runtime for runtime in SIDES if runtime.side == "right")
    solver = LulaKinematicsSolver(
        str(NERO_LULA_DESCRIPTION),
        str(NERO_LULA_SOURCE_URDF),
    )
    if tuple(solver.get_joint_names()) != NERO_JOINT_NAMES:
        raise RuntimeError(
            f"NERO Lula cspace differs from the canonical q7 layout: {solver.get_joint_names()}"
        )
    if "link7" not in solver.get_all_frame_names():
        raise RuntimeError("NERO Lula model does not expose link7")
    solver.set_robot_base_pose(
        np.asarray(right_runtime.mount_pose.position_m, dtype=np.float64),
        np.asarray(right_runtime.mount_pose.quat_wxyz, dtype=np.float64),
    )

    initial_feedback = {side: _positions(articulations[side]) for side in ("left", "right")}
    initial_left_arm_command = arm_targets["left"].copy()
    initial_hand_commands = {side: hand_targets[side].copy() for side in ("left", "right")}
    right_arm_indices = np.asarray(
        partitions["right"].arm_indices_q7,
        dtype=np.int64,
    )
    right_hand_indices = np.asarray(
        partitions["right"].hand_indices_q20,
        dtype=np.int64,
    )
    initial_q7 = initial_feedback["right"][right_arm_indices].copy()
    arm_targets["right"] = initial_q7.copy()
    apply_targets()
    (
        reference_position_m,
        reference_orientation_wxyz,
    ) = _nero_link7_pose(solver, initial_q7)

    mapper = RelativeTrackerPoseMapper(
        stream_id=TRACKER_STREAM_ID,
        device_serial=ARGS.tracker_serial,
        logical_role=TRACKER_LOGICAL_ROLE,
        tracking_frame=TRACKER_MAPPING.tracking_frame,
        tracker_to_workcell=TRACKER_MAPPING.tracker_to_workcell,
        translation_scale=TRACKER_TRANSLATION_SCALE,
        max_translation_delta_m=TRACKER_MAX_TRANSLATION_DELTA_M,
        rotation_scale=TRACKER_ROTATION_SCALE,
        max_rotation_delta_rad=TRACKER_MAX_ROTATION_DELTA_RAD,
        stale_after_s=ARGS.tracker_stale_s,
        translation_enabled=not ARGS.tracker_freeze_translation,
        rotation_enabled=ARGS.tracker_rotation,
    )
    reference_readiness_gate = TrackerReferenceReadinessGate(
        stream_id=TRACKER_STREAM_ID,
        device_serial=ARGS.tracker_serial,
        logical_role=TRACKER_LOGICAL_ROLE,
        tracking_frame=TRACKER_MAPPING.tracking_frame,
        stable_after_s=ARGS.tracker_reference_stable_s,
        max_sample_gap_s=ARGS.tracker_stale_s,
    )
    supervisor = JointCommandSupervisor(
        arm_profiles["right"].layout,
        initial_q7,
        stale_after_s=ARGS.tracker_stale_s,
        velocity_scale=0.20,
    )

    if ARGS.gui:
        set_camera_view(
            eye=np.asarray(
                _workcell_frame_position(OBLIQUE_CAMERA_EYE_FRAME),
                dtype=np.float64,
            ),
            target=np.asarray(
                _workcell_frame_position(OBLIQUE_CAMERA_TARGET_FRAME),
                dtype=np.float64,
            ),
            camera_prim_path=SCREENSHOT_CAMERA_PRIM_PATH,
        )
        _step(world, 5, render=True)
        return _run_tracker_interactive(
            world,
            articulations=articulations,
            partitions=partitions,
            solver=solver,
            mapper=mapper,
            supervisor=supervisor,
            arm_targets=arm_targets,
            apply_targets=apply_targets,
        )

    with UdpTrackingSampleReceiver(
        ARGS.tracker_udp_port,
        stream_id=TRACKER_STREAM_ID,
        device_serial=ARGS.tracker_serial,
        logical_role=TRACKER_LOGICAL_ROLE,
        tracking_frame=TRACKER_MAPPING.tracking_frame,
    ) as receiver:
        reference_requested_ns = time.monotonic_ns()
        reference_deadline_ns = reference_requested_ns + round(
            TRACKER_REFERENCE_WAIT_S * 1_000_000_000
        )
        reference_sample = None
        reference_readiness: TrackerReferenceReadiness | None = None
        last_reference_error = "no canonical sample received"
        while time.monotonic_ns() < reference_deadline_ns:
            now_ns = time.monotonic_ns()
            candidates = receiver.receive_available(now_ns=now_ns)
            for candidate in candidates:
                if candidate.host_time_ns < reference_requested_ns:
                    continue
                reference_readiness = reference_readiness_gate.observe(candidate)
                last_reference_error = (
                    f"{reference_readiness.reason}; "
                    "continuous_running="
                    f"{reference_readiness.stable_duration_s:.3f}s/"
                    f"{ARGS.tracker_reference_stable_s:.3f}s"
                )
                if not reference_readiness.ready:
                    continue
                try:
                    mapper.arm(
                        candidate,
                        reference_position_m,
                        reference_orientation_wxyz,
                        now_ns=now_ns,
                    )
                except ValueError as exc:
                    last_reference_error = str(exc)
                else:
                    reference_sample = candidate
                    break
            if reference_sample is not None:
                break
            world.step(render=ARGS.gui)
            time.sleep(1.0 / PHYSICS_HZ)
        if reference_sample is None:
            raise RuntimeError(
                "no fresh actionable Tracker sample arrived within "
                f"{TRACKER_REFERENCE_WAIT_S:g}s: {last_reference_error}"
            )
        assert reference_readiness is not None
        assert reference_readiness.ready

        armed_ns = max(time.monotonic_ns(), reference_requested_ns + 1)
        supervisor.arm(armed_ns)
        print(
            "TRACKER_REFERENCE_READY "
            f"serial={reference_sample.device_serial} "
            f"position_m={list(reference_sample.position_m or ())} "
            f"mapping_id={TRACKER_MAPPING.mapping_id} "
            "continuous_running="
            f"{reference_readiness.stable_duration_s:.3f}s "
            "running_samples="
            f"{reference_readiness.consecutive_running_samples} "
            f"tracker_to_workcell={TRACKER_MAPPING.tracker_to_workcell} "
            "translation="
            f"{'frozen' if ARGS.tracker_freeze_translation else 'enabled'} "
            f"translation_scale={TRACKER_TRANSLATION_SCALE:g} "
            f"translation_clamp=±{TRACKER_MAX_TRANSLATION_DELTA_M:g}m "
            f"rotation={'enabled' if ARGS.tracker_rotation else 'disabled'} "
            f"rotation_scale={TRACKER_ROTATION_SCALE:g} "
            "rotation_clamp="
            f"±{math.degrees(TRACKER_MAX_ROTATION_DELTA_RAD):g}deg",
            flush=True,
        )
        _print_tracker_operator_instruction()

        frame_period_ns = round(1_000_000_000 / PHYSICS_HZ)
        next_deadline_ns = armed_ns
        last_tick_ns = armed_ns
        accepted_samples = 0
        translation_clamped_samples = 0
        rotation_clamped_samples = 0
        ik_successes = 0
        ik_failures = 0
        consecutive_ik_failures = 0
        completed_frames = 0
        termination_reason = "completed"
        max_world_delta_m = 0.0
        max_target_rotation_delta_rad = 0.0
        max_feedback_rotation_delta_rad = 0.0
        max_right_arm_feedback_delta_rad = 0.0
        max_right_hand_feedback_delta_rad = 0.0
        max_left_feedback_delta_rad = 0.0
        supervision_reasons: dict[str, int] = {}
        last_mapping = None

        for frame_index in range(ARGS.tracker_frames):
            next_deadline_ns += frame_period_ns
            tick_ns = max(time.monotonic_ns(), last_tick_ns + 1)
            sample = receiver.receive_latest(now_ns=tick_ns)
            mapping = mapper.advance(sample, now_ns=tick_ns)
            last_mapping = mapping
            if mapping.accepted:
                accepted_samples += 1
                translation_clamped_samples += int(mapping.translation_clamped)
                rotation_clamped_samples += int(mapping.rotation_clamped)
            if (
                mapping.requires_reference
                or mapping.target_position_m is None
                or mapping.target_orientation_wxyz is None
            ):
                termination_reason = mapping.reason
                break
            assert mapping.input_host_time_ns is not None
            if mapping.world_delta_m is not None:
                max_world_delta_m = max(
                    max_world_delta_m,
                    float(np.linalg.norm(mapping.world_delta_m)),
                )
            if mapping.rotation_delta_rad is not None:
                max_target_rotation_delta_rad = max(
                    max_target_rotation_delta_rad,
                    mapping.rotation_delta_rad,
                )

            solution, ik_success = solver.compute_inverse_kinematics(
                "link7",
                np.asarray(mapping.target_position_m, dtype=np.float64),
                np.asarray(
                    mapping.target_orientation_wxyz,
                    dtype=np.float64,
                ),
                warm_start=supervisor.last_command,
                position_tolerance=0.002,
                orientation_tolerance=0.02,
            )
            try:
                candidate_q7 = arm_profiles["right"].layout.validate_vector(
                    np.asarray(solution, dtype=np.float64)
                )
            except (TypeError, ValueError, OverflowError):
                ik_success = False
                candidate_q7 = supervisor.last_command.copy()

            if ik_success:
                ik_successes += 1
                consecutive_ik_failures = 0
                supervision = supervisor.step(
                    candidate_q7,
                    now_ns=tick_ns,
                    input_time_ns=mapping.input_host_time_ns,
                )
                supervision_reasons[supervision.reason] = (
                    supervision_reasons.get(supervision.reason, 0) + 1
                )
                arm_targets["right"] = supervision.command.copy()
                apply_targets()
            else:
                ik_failures += 1
                consecutive_ik_failures += 1
                if consecutive_ik_failures >= 5:
                    termination_reason = "five_consecutive_ik_failures"
                    break

            world.step(render=ARGS.gui)
            current_right = _positions(articulations["right"])
            current_left = _positions(articulations["left"])
            max_right_arm_feedback_delta_rad = max(
                max_right_arm_feedback_delta_rad,
                float(
                    np.max(
                        np.abs(
                            current_right[right_arm_indices]
                            - initial_feedback["right"][right_arm_indices]
                        )
                    )
                ),
            )
            max_right_hand_feedback_delta_rad = max(
                max_right_hand_feedback_delta_rad,
                float(
                    np.max(
                        np.abs(
                            current_right[right_hand_indices]
                            - initial_feedback["right"][right_hand_indices]
                        )
                    )
                ),
            )
            max_left_feedback_delta_rad = max(
                max_left_feedback_delta_rad,
                float(np.max(np.abs(current_left - initial_feedback["left"]))),
            )
            if ARGS.tracker_rotation:
                _, current_right_rotation = solver.compute_forward_kinematics(
                    "link7",
                    current_right[right_arm_indices],
                )
                current_right_orientation = rotation_matrix_to_quaternion_wxyz(
                    np.asarray(
                        current_right_rotation,
                        dtype=np.float64,
                    )
                )
                max_feedback_rotation_delta_rad = max(
                    max_feedback_rotation_delta_rad,
                    quaternion_geodesic_distance_rad(
                        reference_orientation_wxyz,
                        current_right_orientation,
                    ),
                )
            completed_frames += 1
            if frame_index % 60 == 0:
                world_delta = (
                    None
                    if mapping.world_delta_m is None
                    else [round(value, 4) for value in mapping.world_delta_m]
                )
                rotation_delta_deg = (
                    None
                    if mapping.rotation_delta_rad is None
                    else round(math.degrees(mapping.rotation_delta_rad), 2)
                )
                print(
                    f"tracker_frame={frame_index:04d} "
                    f"world_delta_m={world_delta} "
                    f"rotation_delta_deg={rotation_delta_deg} "
                    f"ik={'ok' if ik_success else 'hold'} "
                    "q7="
                    f"{[round(float(value), 3) for value in arm_targets['right']]}",
                    flush=True,
                )
            last_tick_ns = tick_ns
            remaining_s = (next_deadline_ns - time.monotonic_ns()) / 1_000_000_000
            if remaining_s > 0.0:
                time.sleep(remaining_s)

        completed = termination_reason == "completed" and completed_frames == ARGS.tracker_frames
        right_hand_command_held = bool(
            np.array_equal(
                hand_targets["right"],
                initial_hand_commands["right"],
            )
        )
        left_commands_held = bool(
            np.array_equal(arm_targets["left"], initial_left_arm_command)
            and np.array_equal(
                hand_targets["left"],
                initial_hand_commands["left"],
            )
        )
        rotation_target_exercised = bool(
            not ARGS.tracker_rotation
            or max_target_rotation_delta_rad >= TRACKER_ROTATION_RESPONSE_MIN_RAD
        )
        rotation_feedback_responded = bool(
            not ARGS.tracker_rotation
            or max_feedback_rotation_delta_rad >= TRACKER_ROTATION_RESPONSE_MIN_RAD
        )
        passed = bool(
            completed
            and accepted_samples > 0
            and ik_successes > 0
            and max_right_arm_feedback_delta_rad >= TRACKER_RESPONSE_MIN_RAD
            and rotation_target_exercised
            and rotation_feedback_responded
            and right_hand_command_held
            and left_commands_held
            and max_right_hand_feedback_delta_rad <= TRACKER_HAND_FEEDBACK_TOLERANCE_RAD
            and max_left_feedback_delta_rad <= TRACKER_LEFT_FEEDBACK_TOLERANCE_RAD
        )
        report = {
            "schema": "wujihand.isaac_tracker_right_nero_smoke.v2",
            "scope": (
                "simulation-only right NERO q7; left NERO and both Hand 2 held; "
                "no ROS, CAN, NERO hardware, or Hand 2 hardware"
            ),
            "session_id": RESOLVED.session.session_id,
            "session_hash": RESOLVED.session_hash,
            "five_layer_configuration_modified": False,
            "tracker": {
                "serial": ARGS.tracker_serial,
                "stream_id": TRACKER_STREAM_ID,
                "logical_role": TRACKER_LOGICAL_ROLE,
                "tracking_frame": TRACKER_MAPPING.tracking_frame,
                "udp_endpoint": f"127.0.0.1:{ARGS.tracker_udp_port}",
                "accepted_samples": accepted_samples,
                "receiver_accepted_drains": receiver.accepted,
                "receiver_rejected_datagrams": receiver.rejected,
                "reference_stability": {
                    "required_continuous_running_s": (ARGS.tracker_reference_stable_s),
                    "maximum_sample_gap_s": ARGS.tracker_stale_s,
                    "observed_continuous_running_s": (reference_readiness.stable_duration_s),
                    "consecutive_running_samples": (
                        reference_readiness.consecutive_running_samples
                    ),
                },
            },
            "mapping": {
                "mode": (
                    "relative_rotation_only"
                    if ARGS.tracker_freeze_translation
                    else (
                        "relative_se3" if ARGS.tracker_rotation else "relative_xyz_translation_only"
                    )
                ),
                "profile": str(TRACKER_MAPPING_PATH),
                "mapping_id": TRACKER_MAPPING.mapping_id,
                "scope": TRACKER_MAPPING.scope,
                "provenance": TRACKER_MAPPING.provenance,
                "workcell_frame": TRACKER_MAPPING.workcell_frame,
                "tracker_to_workcell": [list(row) for row in TRACKER_MAPPING.tracker_to_workcell],
                "translation_scale": TRACKER_TRANSLATION_SCALE,
                "translation_enabled": (not ARGS.tracker_freeze_translation),
                "max_translation_delta_each_axis_m": (TRACKER_MAX_TRANSLATION_DELTA_M),
                "stale_after_s": ARGS.tracker_stale_s,
                "translation_clamped_samples": (translation_clamped_samples),
                "max_world_delta_norm_m": max_world_delta_m,
                "rotation_enabled": ARGS.tracker_rotation,
                "rotation_scale": TRACKER_ROTATION_SCALE,
                "max_rotation_delta_deg": math.degrees(TRACKER_MAX_ROTATION_DELTA_RAD),
                "relative_rotation_semantics": (TRACKER_MAPPING.relative_rotation_semantics),
                "rotation_clamped_samples": rotation_clamped_samples,
                "max_target_rotation_delta_deg": math.degrees(max_target_rotation_delta_rad),
                "reference_link7_orientation_wxyz": (reference_orientation_wxyz.tolist()),
            },
            "kinematics": {
                "solver": "Isaac Sim 6.0.1 LulaKinematicsSolver",
                "descriptor": NERO_LULA_DESCRIPTION.relative_to(ROOT).as_posix(),
                "urdf": NERO_LULA_SOURCE_URDF.relative_to(ROOT).as_posix(),
                "urdf_sha256": ALIGNMENT_PROFILE.source_urdf_sha256,
                "end_effector_frame": "link7",
                "ik_successes": ik_successes,
                "ik_failures": ik_failures,
                "supervision_reasons": supervision_reasons,
            },
            "runtime": {
                "requested_frames": ARGS.tracker_frames,
                "completed_frames": completed_frames,
                "termination_reason": termination_reason,
                "last_mapping_reason": (None if last_mapping is None else last_mapping.reason),
            },
            "measurements": {
                "right_arm_max_feedback_delta_rad": (max_right_arm_feedback_delta_rad),
                "right_link7_max_rotation_feedback_delta_deg": (
                    math.degrees(max_feedback_rotation_delta_rad)
                ),
                "right_hand_max_feedback_delta_rad": (max_right_hand_feedback_delta_rad),
                "left_q27_max_feedback_delta_rad": (max_left_feedback_delta_rad),
                "right_hand_command_held": right_hand_command_held,
                "left_arm_and_hand_commands_held": left_commands_held,
            },
            "checks": {
                "bounded_run_completed": completed,
                "fresh_tracker_samples_received": accepted_samples > 0,
                "ik_succeeded": ik_successes > 0,
                "right_arm_responded": (
                    max_right_arm_feedback_delta_rad >= TRACKER_RESPONSE_MIN_RAD
                ),
                "rotation_target_exercised": rotation_target_exercised,
                "rotation_feedback_responded": (rotation_feedback_responded),
                "right_hand_command_held": right_hand_command_held,
                "right_hand_feedback_bounded": (
                    max_right_hand_feedback_delta_rad <= TRACKER_HAND_FEEDBACK_TOLERANCE_RAD
                ),
                "left_commands_held": left_commands_held,
                "left_articulation_feedback_bounded": (
                    max_left_feedback_delta_rad <= TRACKER_LEFT_FEEDBACK_TOLERANCE_RAD
                ),
            },
            "passed": passed,
        }
        encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
        if ARGS.report is not None:
            ARGS.report.parent.mkdir(parents=True, exist_ok=True)
            ARGS.report.write_text(encoded, encoding="utf-8")
        print(encoded, end="")
        return qualification_gate_exit_code(passed)


def _run_native_dual_live(
    world: World,
    *,
    articulations: dict[str, Articulation],
    partitions: dict[str, NeroHand2DofPartition],
    arm_profiles: dict[str, NeroModelProfile],
    hand_profiles: dict[str, Hand2ModelProfile],
    initial_arm_targets: dict[str, npt.NDArray[np.float64]],
    arm_targets: dict[str, npt.NDArray[np.float64]],
    hand_targets: dict[str, npt.NDArray[np.float64]],
    apply_targets: Callable[[], None],
) -> int:
    """Run the Deployment-owned NV-4 native dual simulation loop."""

    resolved = RESOLVED_DEPLOYMENT
    profile = NATIVE_PROFILE
    mapping = TRACKER_MAPPING
    if resolved is None or profile is None or mapping is None:
        raise RuntimeError(
            "native dual Deployment was not resolved before Isaac startup"
        )
    if sha256_file(NERO_LULA_SOURCE_URDF) != (
        ALIGNMENT_PROFILE.source_urdf_sha256
    ):
        raise RuntimeError("source-locked NERO URDF hash drifted")
    if not NERO_LULA_DESCRIPTION.is_file():
        raise RuntimeError(
            f"NERO Lula descriptor not found: {NERO_LULA_DESCRIPTION}"
        )

    plan = build_native_dual_runtime_plan(resolved)
    launch = build_openvr_producer_launch(resolved, ROOT)
    producer = ManagedOpenVrProducer(launch)
    keyboard_reset_input: KeyboardResetInputAdapter | None = None
    if plan.arm_reset_key is not None:
        import carb.input  # type: ignore[import-not-found]
        import omni.appwindow  # type: ignore[import-not-found]

        app_window = omni.appwindow.get_default_app_window()
        if app_window is None:
            raise RuntimeError(
                "debug runtime requires the Isaac application window"
            )
        keyboard = app_window.get_keyboard()
        if keyboard is None:
            raise RuntimeError(
                "debug runtime requires an Isaac keyboard input device"
            )
        reset_key = getattr(
            carb.input.KeyboardInput,
            plan.arm_reset_key,
            None,
        )
        if reset_key is None:
            raise RuntimeError(
                f"unsupported debug reset key {plan.arm_reset_key!r}"
            )
        keyboard_reset_input = KeyboardResetInputAdapter(
            event_source=carb.input.acquire_input_interface(),
            keyboard=keyboard,
            reset_key=reset_key,
            key_press_type=carb.input.KeyboardEventType.KEY_PRESS,
        )
    stream_by_side = {stream.side: stream for stream in launch.streams}
    if set(stream_by_side) != set(plan.live_sides):
        raise RuntimeError(
            "OpenVR streams differ from Deployment live arm ownership"
        )

    receivers = {
        side: UdpTrackingSampleReceiver(
            stream_by_side[side].udp_port,
            stream_id=stream_by_side[side].stream_id,
            device_serial=stream_by_side[side].device_serial,
            logical_role=stream_by_side[side].logical_role,
            producer_instance=launch.producer_instance,
            transport_epoch=launch.transport_epoch,
            tracking_setup_revision=launch.tracking_setup_revision,
            tracking_frame=launch.tracking_frame,
        )
        for side in plan.live_sides
    }
    side_runtime = {runtime.side: runtime for runtime in SIDES}
    arm_controllers: dict[str, TrackerArmSimulationController] = {}
    arm_indices = {
        side: np.asarray(
            partitions[side].arm_indices_q7,
            dtype=np.int64,
        )
        for side in ("left", "right")
    }
    started_ns = time.monotonic_ns()
    for side in plan.live_sides:
        runtime = side_runtime[side]
        solver = LulaKinematicsSolver(
            str(NERO_LULA_DESCRIPTION),
            str(NERO_LULA_SOURCE_URDF),
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
            layout=arm_profiles[side].layout,
            frame_name=profile.kinematics.end_effector_frame,
            position_tolerance_m=(
                profile.kinematics.position_tolerance_m
            ),
            orientation_tolerance_rad=(
                profile.kinematics.orientation_tolerance_rad
            ),
        )
        source = plan.side(side).arm
        local = source.local_binding
        if local is None:
            raise RuntimeError(f"{side} Tracker local binding is missing")
        identity = {
            "stream_id": stream_by_side[side].stream_id,
            "device_serial": local.device_identity,
            "logical_role": source.source.logical_role,
            "tracking_frame": mapping.tracking_frame,
        }
        mapper = RelativeTrackerPoseMapper(
            **identity,
            tracker_to_workcell=mapping.tracker_to_workcell,
            translation_scale=mapping.translation_scale,
            max_translation_delta_m=(
                mapping.max_translation_delta_m
            ),
            rotation_scale=mapping.rotation_scale,
            max_rotation_delta_rad=mapping.max_rotation_delta_rad,
            stale_after_s=profile.tracker.stale_after_s,
            min_quality=profile.tracker.minimum_quality,
            translation_enabled=True,
            rotation_enabled=True,
        )
        readiness = TrackerReferenceReadinessGate(
            **identity,
            stable_after_s=profile.tracker.stable_after_s,
            max_sample_gap_s=profile.tracker.max_sample_gap_s,
        )
        feedback_q7 = _positions(articulations[side])[
            arm_indices[side]
        ]
        arm_targets[side] = feedback_q7.copy()
        supervisor = JointCommandSupervisor(
            arm_profiles[side].layout,
            feedback_q7,
            stale_after_s=profile.arm_supervision.stale_after_s,
            velocity_scale=profile.arm_supervision.velocity_scale,
        )
        controller = TrackerArmSimulationController(
            side=side,
            readiness=readiness,
            tracker=InteractiveTrackerArmController(
                mapper,
                max_consecutive_ik_failures=(
                    profile.tracker.max_consecutive_ik_failures
                ),
            ),
            kinematics=kinematics,
            supervisor=supervisor,
        )
        controller.start(now_ns=started_ns)
        arm_controllers[side] = controller

    glove_controllers: dict[
        HandSide,
        GloveHand2SimulationController,
    ] = {}
    for side in plan.live_sides:
        source = plan.side(side).hand
        local = source.local_binding
        if local is None:
            raise RuntimeError(f"{side} Glove local binding is missing")
        hand_side = HandSide(side)
        input_adapter = WujiGloveHandSkeletonAdapter(
            hand_side,
            source_id=source.source.source_id,
            calibration_id=local.calibration_id,
            transform_id="wuji_glove.hand_skeleton.v1",
            serial_number=local.device_identity,
            device_name=f"nv4_glove_{side}",
        )
        retargeter = WujiHand2RetargetAdapter(
            hand_side,
            max_observation_age_s=profile.glove.max_observation_age_s,
            minimum_landmark_confidence=(
                profile.glove.minimum_landmark_confidence
            ),
            success_landmark_confidence=(
                profile.glove.success_landmark_confidence
            ),
        )
        supervisor = JointCommandSupervisor(
            hand_profiles[side].layout,
            hand_targets[side],
            stale_after_s=profile.hand_supervision.stale_after_s,
            velocity_scale=profile.hand_supervision.velocity_scale,
        )
        glove_controllers[hand_side] = (
            GloveHand2SimulationController(
                hand_side,
                input_adapter,
                retargeter,
                supervisor,
            )
        )
    gloves = GloveHand2ControllerSet(glove_controllers)

    for side in ("left", "right"):
        if side not in plan.live_sides:
            arm_targets[side] = initial_arm_targets[side].copy()
            hand_targets[side] = (
                hand_profiles[side].rest_position.copy()
            )
    apply_targets()
    set_camera_view(
        eye=np.asarray(
            _workcell_frame_position(OBLIQUE_CAMERA_EYE_FRAME),
            dtype=np.float64,
        ),
        target=np.asarray(
            _workcell_frame_position(OBLIQUE_CAMERA_TARGET_FRAME),
            dtype=np.float64,
        ),
        camera_prim_path=SCREENSHOT_CAMERA_PRIM_PATH,
    )
    _step(world, 5, render=True)

    arm_reasons = {side: Counter() for side in plan.live_sides}
    hand_reasons = {side: Counter() for side in plan.live_sides}
    references_established = Counter()
    references_revoked = Counter()
    ik_successes = Counter()
    ik_failures = Counter()
    max_source_skew_ns = 0
    producer_restarts = 0
    completed_frames = 0
    operator_arm_resets = 0
    last_tick_ns = started_ns
    loop_started_ns = time.monotonic_ns()
    print(
        "NV4 LIVE CONNECTING: opening configured Gloves and managed "
        "OpenVR producer.",
        flush=True,
    )
    try:
        if keyboard_reset_input is not None:
            keyboard_reset_input.start()
        gloves.start(now_ns=started_ns)
        lifecycle = producer.start()
        print(
            "NV4 LIVE READY: "
            f"deployment={resolved.deployment.deployment_id} "
            f"live_sides={list(plan.live_sides)} "
            f"transport_epoch={lifecycle.new_transport_epoch}; "
            "move the configured Tracker(s) and Glove(s) when ready"
            + (
                f"; press {plan.arm_reset_key} to reset both arm poses."
                if plan.arm_reset_key is not None
                else "."
            ),
            flush=True,
        )
        while simulation_app.is_running():
            tick_ns = max(time.monotonic_ns(), last_tick_ns + 1)
            if (
                keyboard_reset_input is not None
                and keyboard_reset_input.consume_reset_requested()
            ):
                for side in ("left", "right"):
                    restored = initial_arm_targets[side].copy()
                    arm_targets[side] = restored
                    articulations[side].set_joint_positions(
                        restored[np.newaxis, :],
                        joint_indices=arm_indices[side],
                    )
                    articulations[side].set_joint_velocities(
                        np.zeros((1, 7), dtype=np.float64),
                        joint_indices=arm_indices[side],
                    )
                for side, controller in arm_controllers.items():
                    controller.reset(
                        initial_arm_targets[side],
                        now_ns=tick_ns,
                    )
                apply_targets()
                world.step(render=True)
                completed_frames += 1
                operator_arm_resets += 1
                last_tick_ns = tick_ns
                print(
                    "NV4 DEBUG RESET: both arm poses restored; "
                    "active Tracker references cleared.",
                    flush=True,
                )
                continue
            try:
                producer.ensure_running()
            except RuntimeError:
                lifecycle = producer.restart()
                producer_restarts += 1
                assert lifecycle.new_transport_epoch is not None
                for side, receiver in receivers.items():
                    receiver.authorize_epoch(
                        producer_instance=launch.producer_instance,
                        transport_epoch=(
                            lifecycle.new_transport_epoch
                        ),
                        tracking_setup_revision=(
                            launch.tracking_setup_revision
                        ),
                    )
                    arm_controllers[side].invalidate_reference()
                print(
                    "NV4 OPENVR REBOUND: "
                    f"transport_epoch={lifecycle.new_transport_epoch}; "
                    "active arm references invalidated.",
                    flush=True,
                )

            fresh_source_times: list[int] = []
            arm_states: dict[str, str] = {}
            for side, controller in arm_controllers.items():
                samples = receivers[side].receive_available(
                    now_ns=tick_ns
                )
                step = controller.step(
                    samples,
                    feedback_q7_rad=_positions(
                        articulations[side]
                    )[arm_indices[side]],
                    now_ns=tick_ns,
                )
                arm_targets[side] = step.safety.command.copy()
                arm_reasons[side][step.reason] += 1
                arm_states[side] = step.state.value
                if step.reference_established:
                    references_established[side] += 1
                    print(
                        f"NV4 {side.upper()} ARM REFERENCE "
                        f"epoch={step.reference_epoch}",
                        flush=True,
                    )
                if step.reference_revoked:
                    references_revoked[side] += 1
                    print(
                        f"NV4 {side.upper()} ARM HOLD: {step.reason}",
                        flush=True,
                    )
                if step.kinematics is not None:
                    if step.kinematics.succeeded:
                        ik_successes[side] += 1
                    else:
                        ik_failures[side] += 1
                if (
                    step.mapping is not None
                    and step.mapping.input_host_time_ns is not None
                ):
                    fresh_source_times.append(
                        step.mapping.input_host_time_ns
                    )

            hand_steps = gloves.step(now_ns=tick_ns)
            hand_states: dict[str, str] = {}
            for labelled in hand_steps:
                side = labelled.side.value
                step = labelled.step
                hand_targets[side] = step.decision.command.copy()
                hand_reasons[side][
                    step.rejection_reason or step.decision.reason
                ] += 1
                hand_states[side] = step.decision.state.value
                if step.intent is not None:
                    fresh_source_times.append(
                        step.intent.source_observation.receive_time_ns
                    )

            if len(fresh_source_times) >= 2:
                max_source_skew_ns = max(
                    max_source_skew_ns,
                    max(fresh_source_times) - min(fresh_source_times),
                )
            apply_targets()
            world.step(render=True)
            completed_frames += 1
            if completed_frames % profile.physics_hz == 0:
                print(
                    "NV4 LIVE "
                    f"frame={completed_frames} "
                    f"arm={arm_states} hand={hand_states} "
                    f"ik_failures={dict(ik_failures)}",
                    flush=True,
                )
            last_tick_ns = tick_ns
    finally:
        if keyboard_reset_input is not None:
            keyboard_reset_input.close()
        producer.close()
        gloves.close()
        for controller in arm_controllers.values():
            controller.close()
        for receiver in receivers.values():
            receiver.close()

    report = {
        "schema": "wujihand.isaac_native_dual_teleoperation_run.v1",
        "scope": (
            "simulation-only dual NERO + Hand2; no ROS, CAN, NERO "
            "hardware, or Hand2 hardware commands"
        ),
        "deployment": resolved.to_mapping(),
        "session_id": RESOLVED.session.session_id,
        "session_hash": RESOLVED.session_hash,
        "compatibility_profile": {
            "profile_id": profile.profile_id,
            "status": profile.status,
            "path": RESOLVED.session.runtime.compatibility_profile,
        },
        "mapping": {
            "mapping_id": mapping.mapping_id,
            "translation_scale": mapping.translation_scale,
            "max_translation_delta_each_axis_m": (
                mapping.max_translation_delta_m
            ),
            "max_diagonal_delta_m": (
                math.sqrt(3.0) * mapping.max_translation_delta_m
            ),
            "rotation_scale": mapping.rotation_scale,
            "max_rotation_delta_deg": mapping.max_rotation_delta_deg,
        },
        "runtime": {
            "live_sides": list(plan.live_sides),
            "completed_frames": completed_frames,
            "operator_arm_resets": operator_arm_resets,
            "arm_reset_key": plan.arm_reset_key,
            "wall_duration_s": (
                time.monotonic_ns() - loop_started_ns
            )
            / 1_000_000_000,
            "producer_restarts": producer_restarts,
            "max_fresh_source_skew_ms": max_source_skew_ns / 1_000_000,
        },
        "arms": {
            side: {
                "reasons": dict(arm_reasons[side]),
                "references_established": references_established[side],
                "references_revoked": references_revoked[side],
                "ik_successes": ik_successes[side],
                "ik_failures": ik_failures[side],
                "udp_batches_accepted": receivers[side].accepted,
                "udp_datagrams_rejected": receivers[side].rejected,
            }
            for side in plan.live_sides
        },
        "hands": {
            side: {"reasons": dict(hand_reasons[side])}
            for side in plan.live_sides
        },
        "tracking_setup_qualified": resolved.tracking_qualified,
        "qualification_evaluated": False,
        "passed": None,
    }
    report_path = ARGS.report
    if report_path is None:
        timestamp = datetime.now(timezone.utc).strftime(
            "%Y%m%dT%H%M%SZ"
        )
        report_path = (
            ROOT
            / resolved.deployment.report_root
            / f"{resolved.deployment.deployment_id}-{timestamp}.json"
        )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(
            report,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    print(
        f"NV4 LIVE CLOSED: report={report_path.resolve()}",
        flush=True,
    )
    return 0


def main() -> int:
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / PHYSICS_HZ,
        rendering_dt=1.0 / 30.0,
        backend="numpy",
        device="cpu",
    )
    stage = world.scene.stage
    UsdGeom.Xform.Define(stage, "/World/Robots")
    UsdGeom.Xform.Define(stage, "/World/Attachments")
    world.scene.add_default_ground_plane()
    fixed_workcell_prim_paths: list[str] = []
    for entity in RESOLVED.workcell.entities:
        if entity.mobility != "fixed":
            raise RuntimeError(
                f"NV-2 nominal qualification requires fixed workcell entities: {entity.entity_id}"
            )
        if entity.primitive.kind == "plane":
            continue
        if entity.primitive.kind != "box" or entity.primitive.size_m is None:
            raise RuntimeError(
                f"NV-2 runner supports only plane/box workcell entities: {entity.entity_id}"
            )
        pose = _workcell_pose(entity.frame, entity.transform)
        world.scene.add(
            FixedCuboid(
                prim_path=f"/World/Workcell/{entity.entity_id}",
                name=entity.entity_id,
                position=np.asarray(pose.position_m, dtype=np.float64),
                orientation=np.asarray(pose.quat_wxyz, dtype=np.float64),
                scale=np.asarray(entity.primitive.size_m, dtype=np.float64),
                size=1.0,
                color=np.asarray((0.32, 0.20, 0.12), dtype=np.float64),
            )
        )
        fixed_workcell_prim_paths.append(f"/World/Workcell/{entity.entity_id}")
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(1000.0)

    authored: dict[str, NeroHand2AttachmentHandles] = {}
    geometry_alignments: dict[str, NeroLinkGeometryAlignmentHandles] = {}
    articulations: dict[str, Articulation] = {}
    partitions: dict[str, NeroHand2DofPartition] = {}
    arm_profiles: dict[str, NeroModelProfile] = {}
    hand_profiles: dict[str, Hand2ModelProfile] = {}
    initial_arm_targets: dict[str, npt.NDArray[np.float64]] = {}
    for runtime in SIDES:
        add_reference_to_stage(str(runtime.arm_asset), runtime.arm_prim_path)
        arm_root = stage.GetPrimAtPath(runtime.arm_prim_path)
        _set_world_pose(arm_root, runtime.mount_pose)
        parent_backend_name = RESOLVED.instance(runtime.arm_instance_id).binding.backend_frame(
            runtime.attachment.parent.frame
        )
        child_backend_name = RESOLVED.instance(runtime.hand_instance_id).binding.backend_frame(
            runtime.attachment.child.frame
        )
        parent_link = _one_prim(
            stage,
            prefix=runtime.arm_prim_path,
            name=parent_backend_name,
            rigid_body=True,
        )
        arm_asset = RESOLVED.instance(runtime.arm_instance_id).asset
        wrist_housing_backend_name = RESOLVED.instance(
            runtime.arm_instance_id
        ).binding.backend_frame(arm_asset.frame_name("wrist_housing"))
        wrist_housing = _one_prim(
            stage,
            prefix=runtime.arm_prim_path,
            name=wrist_housing_backend_name,
            rigid_body=True,
        )
        geometry_alignments[runtime.side] = apply_isaac_nero_link_geometry_alignment(
            stage,
            link_path=str(wrist_housing.GetPath()),
            profile=ALIGNMENT_PROFILE,
        )
        add_reference_to_stage(str(runtime.hand_asset), runtime.hand_prim_path)

        arm_articulation_root = _one_prim(
            stage,
            prefix=runtime.arm_prim_path,
            articulation_root=True,
        )
        child_base = _one_prim(
            stage,
            prefix=runtime.hand_prim_path,
            name=child_backend_name,
            rigid_body=True,
        )
        hand_root_joint = _one_prim(
            stage,
            prefix=runtime.hand_prim_path,
            articulation_root=True,
            fixed_joint=True,
        )
        config = NeroHand2AttachmentConfig(
            side=runtime.side,
            nero_prim_path=runtime.arm_prim_path,
            hand_prim_path=runtime.hand_prim_path,
            nero_articulation_root_path=str(arm_articulation_root.GetPath()),
            parent_link_path=str(parent_link.GetPath()),
            child_base_link_path=str(child_base.GetPath()),
            hand_root_joint_path=str(hand_root_joint.GetPath()),
            attachment_joint_path=(f"/World/Attachments/{runtime.attachment.attachment_id}"),
            position_m=runtime.attachment.transform.position_m,
            quat_wxyz=runtime.attachment.transform.quat_wxyz,
            enable_self_collisions=False,
        )
        authored[runtime.side] = author_nero_hand2_attachment(stage, config)
        articulations[runtime.side] = world.scene.add(
            Articulation(
                str(arm_articulation_root.GetPath()),
                name=f"nero_hand2_{runtime.side}",
            )
        )
        arm_profile = load_nero_model_profile(runtime.arm_profile)
        arm_profiles[runtime.side] = arm_profile
        hand_profiles[runtime.side] = load_hand2_model_profile(runtime.hand_profile)
        initial_arm_targets[runtime.side] = arm_profile.layout.validate_vector(
            TABLETOP_PROFILE.initial_position(
                runtime.arm_instance_id,
                "arm_joints",
                arm_profile.layout_id,
            )
        ).copy()

    expected_root_paths = tuple(
        sorted(handle.articulation_root_path for handle in authored.values())
    )

    def validate_articulations() -> tuple[
        dict[str, NeroHand2DofPartition],
        tuple[str, ...],
    ]:
        root_paths = tuple(
            sorted(
                str(prim.GetPath())
                for prim in stage.Traverse()
                if str(prim.GetPath()).startswith("/World/Robots/")
                and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            )
        )
        if root_paths != expected_root_paths:
            raise RuntimeError(
                "NV-2 stage must contain exactly the two expected q27 roots: "
                f"expected={expected_root_paths}, actual={root_paths}"
            )

        result: dict[str, NeroHand2DofPartition] = {}
        for runtime in SIDES:
            articulation = articulations[runtime.side]
            hand_profile = hand_profiles[runtime.side]
            dof_paths = _dof_paths(articulation)
            partition = discover_nero_hand2_dofs(
                articulation.dof_names,
                dof_paths,
                NERO_JOINT_NAMES,
                hand_profile.layout.names,
                nero_prim_path=runtime.arm_prim_path,
                hand_prim_path=runtime.hand_prim_path,
            )
            result[runtime.side] = partition
            limits = np.asarray(articulation.get_dof_limits(), dtype=np.float64)
            if limits.shape != (1, 27, 2):
                raise RuntimeError(
                    f"{runtime.side} q27 limits have unexpected shape {limits.shape}"
                )
            arm_limits = limits[0, np.asarray(partition.arm_indices_q7)]
            hand_limits = limits[0, np.asarray(partition.hand_indices_q20)]
            expected_arm = np.column_stack(
                (
                    arm_profiles[runtime.side].layout.lower,
                    arm_profiles[runtime.side].layout.upper,
                )
            )
            expected_hand = np.column_stack(
                (
                    hand_profile.layout.lower,
                    hand_profile.layout.upper,
                )
            )
            if not np.allclose(arm_limits, expected_arm, atol=1e-4):
                raise RuntimeError(f"{runtime.side} NERO q7 limits drifted")
            if not np.allclose(hand_limits, expected_hand, atol=1e-4):
                raise RuntimeError(f"{runtime.side} Hand 2 q20 limits drifted")
        return result, root_paths

    def apply_qualification_arm_drive_gains(
        current_partitions: dict[str, NeroHand2DofPartition],
    ) -> dict[str, dict[str, list[float]]]:
        configured = TABLETOP_PROFILE.arm_drive_gains
        result: dict[str, dict[str, list[float]]] = {}
        for side in ("left", "right"):
            joint_indices = np.asarray(
                current_partitions[side].arm_indices_q7,
                dtype=np.int64,
            )
            kps = np.full(
                (1, len(joint_indices)),
                configured.stiffness,
                dtype=np.float32,
            )
            kds = np.full(
                (1, len(joint_indices)),
                configured.damping,
                dtype=np.float32,
            )
            articulations[side].set_gains(
                kps=kps,
                kds=kds,
                joint_indices=joint_indices,
            )
            actual_kps, actual_kds = articulations[side].get_gains(
                joint_indices=joint_indices,
            )
            actual_kps = np.asarray(actual_kps, dtype=np.float64)
            actual_kds = np.asarray(actual_kds, dtype=np.float64)
            if (
                actual_kps.shape != kps.shape
                or actual_kds.shape != kds.shape
                or not np.allclose(actual_kps, kps)
                or not np.allclose(actual_kds, kds)
            ):
                raise RuntimeError(f"{side} qualification q7 drive gains were not applied")
            result[side] = {
                "stiffness": actual_kps[0].tolist(),
                "damping": actual_kds[0].tolist(),
            }
        return result

    world.reset()
    partitions, root_paths_before_reset = validate_articulations()
    arm_drive_runtime = apply_qualification_arm_drive_gains(partitions)

    external_fixed_collider_paths: list[str] = []
    for prefix in fixed_workcell_prim_paths:
        matches = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if (str(prim.GetPath()) == prefix or str(prim.GetPath()).startswith(prefix + "/"))
            and prim.HasAPI(UsdPhysics.CollisionAPI)
        ]
        if not matches:
            raise RuntimeError(f"fixed workcell entity has no authored external collider: {prefix}")
        external_fixed_collider_paths.extend(matches)
    external_fixed_collider_paths = sorted(set(external_fixed_collider_paths))
    if not external_fixed_collider_paths:
        raise RuntimeError("NV-2 nominal workcell has no fixed external collider")

    arm_targets = {side: initial_arm_targets[side].copy() for side in ("left", "right")}
    hand_targets = {side: hand_profiles[side].rest_position.copy() for side in ("left", "right")}

    def apply_targets() -> None:
        for side in ("left", "right"):
            arm_profile = arm_profiles[side]
            hand_profile = hand_profiles[side]
            if not (
                np.all(arm_targets[side] >= np.asarray(arm_profile.layout.lower))
                and np.all(arm_targets[side] <= np.asarray(arm_profile.layout.upper))
                and np.all(hand_targets[side] >= np.asarray(hand_profile.layout.lower))
                and np.all(hand_targets[side] <= np.asarray(hand_profile.layout.upper))
            ):
                raise RuntimeError(f"{side} scripted q7/q20 target exceeds limits")
            full = _full_target(
                partitions[side],
                arm_targets[side],
                hand_targets[side],
            )
            articulations[side].set_joint_position_targets(full[np.newaxis, :])

    feedback: dict[str, dict[str, list[float]]] = {}
    settling_records: dict[str, dict[str, object]] = {}

    def capture_feedback(key: str) -> None:
        if key in feedback:
            raise RuntimeError(f"duplicate feedback phase key: {key}")
        feedback[key] = {
            side: _positions(articulations[side]).tolist() for side in ("left", "right")
        }

    def alias_feedback(source_key: str, alias_key: str) -> None:
        if alias_key in feedback:
            raise RuntimeError(f"duplicate feedback phase key: {alias_key}")
        feedback[alias_key] = {side: list(feedback[source_key][side]) for side in ("left", "right")}

    def settle_until_stable(
        settling_id: str,
        *,
        policy: Q27ReadinessPolicy = FULL_SCRIPTED_Q27_SETTLING_POLICY,
    ) -> tuple[str, str]:
        """Apply one explicit bounded q27 readiness policy."""

        if settling_id in settling_records:
            raise RuntimeError(f"duplicate settling phase: {settling_id}")
        started_ns = time.monotonic_ns()
        window_keys: list[str] = []
        max_delta_history_rad: list[float] = []
        previous_key: str | None = None
        for window_number in range(1, policy.maximum_windows + 1):
            _step(world, policy.window_frames, render=ARGS.gui)
            current_key = f"{settling_id}_settle_{window_number:02d}"
            capture_feedback(current_key)
            window_keys.append(current_key)
            if previous_key is not None:
                max_delta_rad = q27_window_max_delta_rad(
                    feedback[previous_key],
                    feedback[current_key],
                )
                max_delta_history_rad.append(max_delta_rad)
                if (
                    window_number >= policy.minimum_windows
                    and max_delta_rad <= policy.max_window_delta_rad
                ):
                    settling_records[settling_id] = {
                        "policy_id": policy.policy_id,
                        "converged": True,
                        "proceeded_after_timeout": False,
                        "require_convergence": policy.require_convergence,
                        "window_frames": policy.window_frames,
                        "windows_run": window_number,
                        "window_keys": list(window_keys),
                        "max_delta_history_rad": list(max_delta_history_rad),
                        "peak_max_delta_rad": max(max_delta_history_rad),
                        "final_max_delta_rad": max_delta_rad,
                        "tolerance_rad": policy.max_window_delta_rad,
                        "measured_scope": "both q27 articulations",
                        "wall_duration_s": (time.monotonic_ns() - started_ns) / 1_000_000_000,
                    }
                    return previous_key, current_key
            previous_key = current_key

        settling_records[settling_id] = {
            "policy_id": policy.policy_id,
            "converged": False,
            "proceeded_after_timeout": not policy.require_convergence,
            "require_convergence": policy.require_convergence,
            "window_frames": policy.window_frames,
            "windows_run": policy.maximum_windows,
            "window_keys": list(window_keys),
            "max_delta_history_rad": list(max_delta_history_rad),
            "peak_max_delta_rad": max(max_delta_history_rad),
            "final_max_delta_rad": max_delta_history_rad[-1],
            "tolerance_rad": policy.max_window_delta_rad,
            "measured_scope": "both q27 articulations",
            "wall_duration_s": (time.monotonic_ns() - started_ns) / 1_000_000_000,
        }
        if policy.require_convergence:
            raise RuntimeError(
                f"{settling_id} did not settle within "
                f"{policy.maximum_windows} windows: "
                f"last max q27 delta={max_delta_history_rad[-1]:.6f} rad"
            )
        print(
            f"GLOVE LIVE READINESS: bounded warmup completed without strict "
            f"convergence ({max_delta_history_rad[-1]:.4f} rad); continuing "
            "because this path commands simulation only.",
            flush=True,
        )
        return window_keys[-2], window_keys[-1]

    apply_targets()
    if NATIVE_DUAL_LIVE:
        print(
            "NV4 LIVE READINESS: settling the shared two-q27 scene "
            "before opening external inputs.",
            flush=True,
        )
        settle_until_stable(
            "native_dual_live_ready",
            policy=GLOVE_LIVE_Q27_READINESS_POLICY,
        )
        return _run_native_dual_live(
            world,
            articulations=articulations,
            partitions=partitions,
            arm_profiles=arm_profiles,
            hand_profiles=hand_profiles,
            initial_arm_targets=initial_arm_targets,
            arm_targets=arm_targets,
            hand_targets=hand_targets,
            apply_targets=apply_targets,
        )

    if ARGS.glove_live:
        print(
            "GLOVE LIVE READINESS: running at most "
            f"{GLOVE_LIVE_Q27_READINESS_POLICY.maximum_windows * GLOVE_LIVE_Q27_READINESS_POLICY.window_frames} "
            "simulation warmup frames; scripted hand qualification is skipped.",
            flush=True,
        )
        settle_until_stable(
            "glove_live_ready",
            policy=GLOVE_LIVE_Q27_READINESS_POLICY,
        )
        readiness_record = settling_records["glove_live_ready"]
        readiness_feedback = {
            key: feedback[key] for key in cast(list[str], readiness_record["window_keys"])
        }
        print(
            "GLOVE LIVE READINESS COMPLETE: starting input connection.",
            flush=True,
        )
        return _run_glove_live_qualification(
            world,
            articulations=articulations,
            partitions=partitions,
            arm_profiles=arm_profiles,
            hand_profiles=hand_profiles,
            arm_targets=arm_targets,
            hand_targets=hand_targets,
            apply_targets=apply_targets,
            readiness_record=readiness_record,
            readiness_feedback=readiness_feedback,
            articulation_root_paths=root_paths_before_reset,
            arm_drive_runtime=arm_drive_runtime,
        )

    initial_previous_key, initial_final_key = settle_until_stable("initial")
    alias_feedback(initial_previous_key, "initial_settle_a")
    alias_feedback(initial_final_key, "initial")

    if ARGS.tracker_live:
        return _run_tracker_live(
            world,
            articulations=articulations,
            partitions=partitions,
            arm_profiles=arm_profiles,
            arm_targets=arm_targets,
            hand_targets=hand_targets,
            apply_targets=apply_targets,
        )

    arm_targets["left"][0] += ARGS.arm_amplitude_rad
    apply_targets()
    _step(world, ARGS.frames_per_phase, render=ARGS.gui)
    capture_feedback("left_arm")

    arm_targets["right"][1] -= ARGS.arm_amplitude_rad
    apply_targets()
    _step(world, ARGS.frames_per_phase, render=ARGS.gui)
    capture_feedback("right_arm")

    qualification_targets: dict[
        str,
        tuple[tuple[Hand2QualificationTarget, ...], Hand2QualificationTarget],
    ] = {}
    for side in ("left", "right"):
        qualification_targets[side] = build_hand2_qualification_targets(
            HandSide(side),
            hand_profiles[side].rest_position,
            amplitude_rad=ARGS.hand_amplitude_rad,
        )

    def run_hand_phase(target: Hand2QualificationTarget) -> ScriptedHandPhase:
        side = target.side.value
        profile = hand_profiles[side]
        for reset_side in ("left", "right"):
            hand_targets[reset_side] = hand_profiles[reset_side].rest_position.copy()
        apply_targets()
        _, stable_key = settle_until_stable(f"{target.phase_id}_baseline")
        baseline_key = f"{target.phase_id}_baseline"
        alias_feedback(stable_key, baseline_key)

        commanded_profile_indices = tuple(
            profile.layout.names.index(name) for name in target.commanded_joint_names
        )
        hand_targets[side] = profile.layout.validate_vector(target.q20_rad).copy()
        apply_targets()
        _step(world, ARGS.frames_per_phase, render=ARGS.gui)
        command_key = f"{target.phase_id}_command"
        capture_feedback(command_key)
        commanded_runtime_indices = tuple(
            partitions[side].hand_indices_q20[index] for index in commanded_profile_indices
        )
        return ScriptedHandPhase(
            target=target,
            baseline_key=baseline_key,
            command_key=command_key,
            commanded_profile_indices=commanded_profile_indices,
            commanded_runtime_indices=commanded_runtime_indices,
        )

    finger_phases: list[ScriptedHandPhase] = []
    combined_phases: list[ScriptedHandPhase] = []
    for side in ("left", "right"):
        singles, combined = qualification_targets[side]
        finger_phases.extend(run_hand_phase(target) for target in singles)
        combined_phases.append(run_hand_phase(combined))

    glove_live_report: dict[str, object] = {"enabled": False}

    partitions_before_reset = dict(partitions)
    world.reset()
    partitions_after_reset, root_paths_after_reset = validate_articulations()
    arm_drive_after_reset = apply_qualification_arm_drive_gains(partitions_after_reset)
    topology_stable_after_reset = (
        root_paths_after_reset == root_paths_before_reset
        and partitions_after_reset == partitions_before_reset
    )
    if not topology_stable_after_reset:
        raise RuntimeError("q27 topology or canonical partition changed across world.reset()")
    partitions = partitions_after_reset

    for side in ("left", "right"):
        arm_targets[side] = initial_arm_targets[side].copy()
        hand_targets[side] = hand_profiles[side].rest_position.copy()
    apply_targets()
    post_reset_previous_key, post_reset_final_key = settle_until_stable("post_reset")
    alias_feedback(post_reset_previous_key, "post_reset_settle_a")
    alias_feedback(post_reset_final_key, "post_reset_settle_b")

    recovery_side = "left"
    recovery_profile = hand_profiles[recovery_side]
    recovery_joint_name = "l_middle_finger_pip"
    recovery_profile_index = recovery_profile.layout.names.index(recovery_joint_name)
    recovery_runtime_index = partitions[recovery_side].hand_indices_q20[recovery_profile_index]
    recovery_delta_rad = min(ARGS.hand_amplitude_rad * 0.5, 0.20)
    hand_targets[recovery_side] = recovery_profile.rest_position.copy()
    hand_targets[recovery_side][recovery_profile_index] += recovery_delta_rad
    apply_targets()
    _step(world, ARGS.frames_per_phase, render=ARGS.gui)
    capture_feedback("post_reset_recovery")

    arrays = {
        phase: {side: np.asarray(values, dtype=np.float64) for side, values in phase_values.items()}
        for phase, phase_values in feedback.items()
    }
    left_partition = partitions["left"]
    right_partition = partitions["right"]

    def max_phase_delta(
        current: str,
        previous: str,
        side: str,
        indices: tuple[int, ...] | None = None,
    ) -> float:
        delta = arrays[current][side] - arrays[previous][side]
        if indices is not None:
            delta = delta[np.asarray(indices, dtype=np.int64)]
        return float(np.max(np.abs(delta)))

    def max_all_sides_delta(current: str, previous: str) -> float:
        return max(max_phase_delta(current, previous, side) for side in ("left", "right"))

    def response_threshold(command_delta_rad: float) -> float:
        return max(
            SCRIPTED_RESPONSE_MIN_RAD,
            command_delta_rad * SCRIPTED_RESPONSE_FRACTION,
        )

    recovery_digit_partition = partition_hand2_single_digit_indices(
        recovery_profile.layout.names,
        (recovery_joint_name,),
    )
    recovery_other_digit_runtime_indices = tuple(
        partitions[recovery_side].hand_indices_q20[index]
        for index in recovery_digit_partition.other_digit_indices
    )
    recovery_same_digit_uncommanded_runtime_indices = tuple(
        partitions[recovery_side].hand_indices_q20[index]
        for index in recovery_digit_partition.same_digit_uncommanded_indices
    )

    checks: dict[str, bool] = {
        "left_arm_responded": bool(
            arrays["left_arm"]["left"][left_partition.arm_indices_q7[0]]
            - arrays["initial"]["left"][left_partition.arm_indices_q7[0]]
            >= ARGS.arm_amplitude_rad * 0.25
        ),
        "right_isolated_during_left_arm": bool(
            max_phase_delta("left_arm", "initial", "right") <= MOTION_ISOLATION_TOLERANCE_RAD
        ),
        "left_hand_held_during_left_arm": bool(
            max_phase_delta(
                "left_arm",
                "initial",
                "left",
                left_partition.hand_indices_q20,
            )
            <= MOTION_ISOLATION_TOLERANCE_RAD
        ),
        "right_arm_responded": bool(
            arrays["right_arm"]["right"][right_partition.arm_indices_q7[1]]
            - arrays["left_arm"]["right"][right_partition.arm_indices_q7[1]]
            <= -ARGS.arm_amplitude_rad * 0.25
        ),
        "left_isolated_during_right_arm": bool(
            max_phase_delta("right_arm", "left_arm", "left") <= MOTION_ISOLATION_TOLERANCE_RAD
        ),
        "right_hand_held_during_right_arm": bool(
            max_phase_delta(
                "right_arm",
                "left_arm",
                "right",
                right_partition.hand_indices_q20,
            )
            <= MOTION_ISOLATION_TOLERANCE_RAD
        ),
        "fixed_external_workcell_collider_present": bool(external_fixed_collider_paths),
        "initial_two_window_settling_finite": bool(
            np.isfinite(arrays["initial_settle_a"]["left"]).all()
            and np.isfinite(arrays["initial_settle_a"]["right"]).all()
            and np.isfinite(arrays["initial"]["left"]).all()
            and np.isfinite(arrays["initial"]["right"]).all()
        ),
        "initial_two_window_settling_bounded": bool(
            max_all_sides_delta("initial", "initial_settle_a")
            <= FULL_SCRIPTED_Q27_SETTLING_POLICY.max_window_delta_rad
        ),
        "post_reset_two_window_settling_finite": bool(
            np.isfinite(arrays["post_reset_settle_a"]["left"]).all()
            and np.isfinite(arrays["post_reset_settle_a"]["right"]).all()
            and np.isfinite(arrays["post_reset_settle_b"]["left"]).all()
            and np.isfinite(arrays["post_reset_settle_b"]["right"]).all()
        ),
        "post_reset_two_window_settling_bounded": bool(
            max_all_sides_delta("post_reset_settle_b", "post_reset_settle_a")
            <= FULL_SCRIPTED_Q27_SETTLING_POLICY.max_window_delta_rad
        ),
        "post_command_reset_two_q27_revalidated": topology_stable_after_reset,
        "post_command_reset_returned_to_approved_initial": bool(
            max_all_sides_delta("post_reset_settle_b", "initial") <= RESET_INITIAL_TOLERANCE_RAD
        ),
        "post_reset_recovery_responded": bool(
            arrays["post_reset_recovery"][recovery_side][recovery_runtime_index]
            - arrays["post_reset_settle_b"][recovery_side][recovery_runtime_index]
            >= response_threshold(recovery_delta_rad)
        ),
        "post_reset_recovery_selected_arm_held": bool(
            max_phase_delta(
                "post_reset_recovery",
                "post_reset_settle_b",
                recovery_side,
                partitions[recovery_side].arm_indices_q7,
            )
            <= MOTION_ISOLATION_TOLERANCE_RAD
        ),
        "post_reset_recovery_same_hand_other_fingers_held": bool(
            max_phase_delta(
                "post_reset_recovery",
                "post_reset_settle_b",
                recovery_side,
                recovery_other_digit_runtime_indices,
            )
            <= MOTION_ISOLATION_TOLERANCE_RAD
        ),
        "post_reset_recovery_other_side_held": bool(
            max_phase_delta(
                "post_reset_recovery",
                "post_reset_settle_b",
                "right",
            )
            <= MOTION_ISOLATION_TOLERANCE_RAD
        ),
    }

    single_digit_motion_diagnostics: dict[str, dict[str, object]] = {}
    for phase in finger_phases:
        side = phase.target.side.value
        other_side = "right" if side == "left" else "left"
        partition = partitions[side]
        other_partition = partitions[other_side]
        commanded_runtime = phase.commanded_runtime_indices
        digit_partition = partition_hand2_single_digit_indices(
            hand_profiles[side].layout.names,
            phase.target.commanded_joint_names,
        )
        other_digit_runtime_indices = tuple(
            partition.hand_indices_q20[index] for index in digit_partition.other_digit_indices
        )
        same_digit_uncommanded_runtime_indices = tuple(
            partition.hand_indices_q20[index]
            for index in digit_partition.same_digit_uncommanded_indices
        )
        response = (
            arrays[phase.command_key][side][np.asarray(commanded_runtime, dtype=np.int64)]
            - arrays[phase.baseline_key][side][np.asarray(commanded_runtime, dtype=np.int64)]
        )
        checks[f"{phase.target.phase_id}_responded"] = bool(
            np.all(response >= response_threshold(phase.target.command_delta_rad))
        )
        other_digits_max_delta_rad = max_phase_delta(
            phase.command_key,
            phase.baseline_key,
            side,
            other_digit_runtime_indices,
        )
        same_digit_uncommanded_max_delta_rad = max_phase_delta(
            phase.command_key,
            phase.baseline_key,
            side,
            same_digit_uncommanded_runtime_indices,
        )
        checks[f"{phase.target.phase_id}_same_hand_other_fingers_held"] = bool(
            other_digits_max_delta_rad <= MOTION_ISOLATION_TOLERANCE_RAD
        )
        single_digit_motion_diagnostics[phase.target.phase_id] = {
            "commanded_digit": digit_partition.commanded_digit,
            "other_fingers_max_feedback_delta_rad": other_digits_max_delta_rad,
            "same_digit_uncommanded_max_feedback_delta_rad": (same_digit_uncommanded_max_delta_rad),
            "same_digit_linkage_is_gate_check": False,
            "other_fingers_isolation_tolerance_rad": (MOTION_ISOLATION_TOLERANCE_RAD),
        }
        checks[f"{phase.target.phase_id}_both_q7_held"] = bool(
            max_phase_delta(
                phase.command_key,
                phase.baseline_key,
                side,
                partition.arm_indices_q7,
            )
            <= MOTION_ISOLATION_TOLERANCE_RAD
            and max_phase_delta(
                phase.command_key,
                phase.baseline_key,
                other_side,
                other_partition.arm_indices_q7,
            )
            <= MOTION_ISOLATION_TOLERANCE_RAD
        )
        checks[f"{phase.target.phase_id}_other_side_held"] = bool(
            max_phase_delta(
                phase.command_key,
                phase.baseline_key,
                other_side,
            )
            <= MOTION_ISOLATION_TOLERANCE_RAD
        )

    for phase in combined_phases:
        side = phase.target.side.value
        other_side = "right" if side == "left" else "left"
        partition = partitions[side]
        other_partition = partitions[other_side]
        combined_runtime = np.asarray(
            phase.commanded_runtime_indices,
            dtype=np.int64,
        )
        response = (
            arrays[phase.command_key][side][combined_runtime]
            - arrays[phase.baseline_key][side][combined_runtime]
        )
        checks[f"{phase.target.phase_id}_commanded_joints_responded"] = bool(
            np.all(response >= response_threshold(phase.target.command_delta_rad))
        )
        checks[f"{phase.target.phase_id}_both_q7_held"] = bool(
            max_phase_delta(
                phase.command_key,
                phase.baseline_key,
                side,
                partition.arm_indices_q7,
            )
            <= MOTION_ISOLATION_TOLERANCE_RAD
            and max_phase_delta(
                phase.command_key,
                phase.baseline_key,
                other_side,
                other_partition.arm_indices_q7,
            )
            <= MOTION_ISOLATION_TOLERANCE_RAD
        )
        checks[f"{phase.target.phase_id}_other_side_held"] = bool(
            max_phase_delta(
                phase.command_key,
                phase.baseline_key,
                other_side,
            )
            <= MOTION_ISOLATION_TOLERANCE_RAD
        )

    all_feedback = np.vstack([values for phase in arrays.values() for values in phase.values()])
    checks["all_feedback_finite"] = bool(np.isfinite(all_feedback).all())

    def feedback_rows_within_limits(
        side: str,
        rows: npt.NDArray[np.float64],
    ) -> bool:
        partition = partitions[side]
        arm_profile = arm_profiles[side]
        hand_profile = hand_profiles[side]
        arm_values = rows[:, np.asarray(partition.arm_indices_q7)]
        hand_values = rows[:, np.asarray(partition.hand_indices_q20)]
        return bool(
            np.all(
                arm_values >= np.asarray(arm_profile.layout.lower) - FEEDBACK_LIMIT_TOLERANCE_RAD
            )
            and np.all(
                arm_values <= np.asarray(arm_profile.layout.upper) + FEEDBACK_LIMIT_TOLERANCE_RAD
            )
            and np.all(
                hand_values >= np.asarray(hand_profile.layout.lower) - FEEDBACK_LIMIT_TOLERANCE_RAD
            )
            and np.all(
                hand_values <= np.asarray(hand_profile.layout.upper) + FEEDBACK_LIMIT_TOLERANCE_RAD
            )
        )

    for side in ("left", "right"):
        side_rows = np.vstack([phase[side] for phase in arrays.values()])
        checks[f"{side}_feedback_within_limits"] = feedback_rows_within_limits(
            side,
            side_rows,
        )
        reset_rows = np.vstack(
            (
                arrays["post_reset_settle_a"][side],
                arrays["post_reset_settle_b"][side],
            )
        )
        checks[f"{side}_post_reset_feedback_within_limits"] = feedback_rows_within_limits(
            side, reset_rows
        )
        arm_indices = np.asarray(partitions[side].arm_indices_q7)
        initial_error = np.max(
            np.abs(arrays["initial"][side][arm_indices] - initial_arm_targets[side])
        )
        post_reset_error = np.max(
            np.abs(arrays["post_reset_settle_b"][side][arm_indices] - initial_arm_targets[side])
        )
        checks[f"{side}_initial_q7_target_reached"] = bool(
            initial_error <= TABLETOP_PROFILE.thresholds.initial_q7_max_error_rad
        )
        checks[f"{side}_post_reset_q7_target_reached"] = bool(
            post_reset_error <= TABLETOP_PROFILE.thresholds.initial_q7_max_error_rad
        )

    geometry = TABLETOP_PROFILE.geometry_contract
    thresholds = TABLETOP_PROFILE.thresholds
    geometry_measurements: dict[str, dict[str, object]] = {}
    for runtime in SIDES:
        side = runtime.side
        attachment = authored[side].config
        arm_asset = RESOLVED.instance(runtime.arm_instance_id).asset
        forearm_proximal = _one_prim(
            stage,
            prefix=runtime.arm_prim_path,
            name=arm_asset.frame_name("forearm_proximal"),
            rigid_body=True,
        )
        forearm_distal = _one_prim(
            stage,
            prefix=runtime.arm_prim_path,
            name=arm_asset.frame_name("forearm_distal"),
            rigid_body=True,
        )
        forearm_delta_world = _world_position(
            stage,
            str(forearm_distal.GetPath()),
        ) - _world_position(
            stage,
            str(forearm_proximal.GetPath()),
        )
        forearm_length_m = float(np.linalg.norm(forearm_delta_world))
        if not np.isfinite(forearm_length_m) or forearm_length_m <= 0.0:
            raise RuntimeError(f"{side} forearm measurement is degenerate")
        forearm_axis_world = forearm_delta_world / forearm_length_m
        link6_cylinder_axis_world = _world_axis(
            stage,
            geometry_alignments[side].link_path,
            ALIGNMENT_PROFILE.corrected_cylinder_axis_local_xyz,
        )
        hand_axis_world = _world_axis(
            stage,
            attachment.child_base_link_path,
            geometry.hand_longitudinal_axis_local_xyz,
        )
        palm_normal_world = _world_axis(
            stage,
            attachment.child_base_link_path,
            geometry.hand_palm_normal_axis_local_xyz,
        )
        port_axis_world = _rotation_matrix(runtime.mount_pose.quat_wxyz) @ np.asarray(
            geometry.base_port_axis_local_xyz,
            dtype=np.float64,
        )
        link6_cylinder_forearm_dot = float(np.dot(link6_cylinder_axis_world, forearm_axis_world))
        hand_base_face_parallel_dot = float(np.dot(hand_axis_world, link6_cylinder_axis_world))
        attachment_anchor_world_m = _world_point(
            stage,
            attachment.parent_link_path,
            attachment.position_m,
        )
        hand_base_origin_world_m = _world_position(stage, attachment.child_base_link_path)
        attachment_anchor_error_m = float(
            np.linalg.norm(attachment_anchor_world_m - hand_base_origin_world_m)
        )
        link6_positive_face_center_world_m = _world_point(
            stage,
            geometry_alignments[side].link_path,
            ALIGNMENT_PROFILE.corrected_cylinder_positive_face_center_local_xyz,
        )
        hand_base_mount_center_error_m = float(
            np.linalg.norm(hand_base_origin_world_m - link6_positive_face_center_world_m)
        )
        flange_origin_world_m = _world_position(stage, attachment.parent_link_path)
        base_port_inward_dot = float(
            np.dot(
                port_axis_world,
                np.asarray(geometry.table_inward_axis_world_xyz),
            )
        )
        hand_world_inward_dot = float(
            np.dot(
                hand_axis_world,
                np.asarray(geometry.table_inward_axis_world_xyz),
            )
        )
        hand_world_vertical_abs = float(abs(hand_axis_world[2]))
        forearm_world_vertical_abs = float(abs(forearm_axis_world[2]))
        hand_palm_down_dot = float(
            np.dot(
                palm_normal_world,
                np.asarray(geometry.table_down_axis_world_xyz),
            )
        )
        geometry_measurements[side] = {
            "link6_cylinder_axis_world_xyz": (link6_cylinder_axis_world.tolist()),
            "hand_longitudinal_axis_world_xyz": hand_axis_world.tolist(),
            "hand_palm_normal_axis_world_xyz": palm_normal_world.tolist(),
            "base_port_axis_world_xyz": port_axis_world.tolist(),
            "forearm_axis_world_xyz": forearm_axis_world.tolist(),
            "forearm_length_m": forearm_length_m,
            "link6_cylinder_forearm_dot": link6_cylinder_forearm_dot,
            "hand_base_face_parallel_dot": hand_base_face_parallel_dot,
            "attachment_anchor_error_m": attachment_anchor_error_m,
            "hand_base_mount_center_error_m": (hand_base_mount_center_error_m),
            "link6_positive_face_center_world_m": (link6_positive_face_center_world_m.tolist()),
            "attachment_anchor_world_m": (attachment_anchor_world_m.tolist()),
            "flange_origin_world_m": flange_origin_world_m.tolist(),
            "hand_base_origin_world_m": hand_base_origin_world_m.tolist(),
            "base_port_inward_dot": base_port_inward_dot,
            "hand_world_inward_dot": hand_world_inward_dot,
            "hand_world_vertical_abs": hand_world_vertical_abs,
            "forearm_world_vertical_abs": forearm_world_vertical_abs,
            "hand_palm_down_dot": hand_palm_down_dot,
        }
        checks[f"{side}_link6_cylinder_follows_forearm"] = bool(
            link6_cylinder_forearm_dot >= thresholds.link6_cylinder_forearm_min_dot
        )
        checks[f"{side}_hand_base_face_parallel_to_link6_face"] = bool(
            hand_base_face_parallel_dot >= thresholds.hand_base_face_parallel_min_dot
        )
        checks[f"{side}_attachment_anchors_coincident"] = bool(
            attachment_anchor_error_m <= thresholds.attachment_anchor_max_error_m
        )
        checks[f"{side}_hand_base_centered_on_link6_face"] = bool(
            hand_base_mount_center_error_m <= thresholds.hand_base_mount_center_max_error_m
        )
        checks[f"{side}_base_port_axis_inward"] = bool(
            base_port_inward_dot >= thresholds.base_port_inward_min_dot
        )
        checks[f"{side}_hand_axis_points_table_inward"] = bool(
            hand_world_inward_dot >= thresholds.hand_world_inward_min_dot
        )
        checks[f"{side}_hand_axis_near_horizontal"] = bool(
            hand_world_vertical_abs <= thresholds.hand_world_vertical_abs_max
        )
        checks[f"{side}_forearm_axis_near_horizontal"] = bool(
            forearm_world_vertical_abs <= thresholds.forearm_world_vertical_abs_max
        )
        checks[f"{side}_hand_palm_points_down"] = bool(
            hand_palm_down_dot >= thresholds.hand_palm_down_min_dot
        )

    screenshot_runtime: dict[str, object] = {}
    oblique_camera_eye = _workcell_frame_position(OBLIQUE_CAMERA_EYE_FRAME)
    oblique_camera_target = _workcell_frame_position(OBLIQUE_CAMERA_TARGET_FRAME)
    if ARGS.screenshot is not None:
        screenshot_runtime = _capture_screenshot(
            world,
            ARGS.screenshot,
            eye_m=oblique_camera_eye,
            target_m=oblique_camera_target,
        )
    top_screenshot_runtime: dict[str, object] = {}
    top_camera_eye = _workcell_frame_position(TOP_CAMERA_EYE_FRAME)
    top_camera_target = _workcell_frame_position(TOP_CAMERA_TARGET_FRAME)
    if ARGS.top_screenshot is not None:
        top_screenshot_runtime = _capture_screenshot(
            world,
            ARGS.top_screenshot,
            eye_m=top_camera_eye,
            target_m=top_camera_target,
        )
    interface_screenshot_runtime: dict[str, object] = {}
    interface_camera_eye = _workcell_frame_position(INTERFACE_CAMERA_EYE_FRAME)
    interface_camera_target = _workcell_frame_position(INTERFACE_CAMERA_TARGET_FRAME)
    if ARGS.interface_screenshot is not None:
        interface_screenshot_runtime = _capture_screenshot(
            world,
            ARGS.interface_screenshot,
            eye_m=interface_camera_eye,
            target_m=interface_camera_target,
        )
    passed = all(checks.values())
    report = {
        "schema": "wujihand.isaac_nero_dual_hand2_physical_smoke.v2",
        "scope": "NV-2 simulation only; no ROS, CAN, NERO hardware, or Hand 2 hardware",
        "session_id": RESOLVED.session.session_id,
        "session_hash": RESOLVED.session_hash,
        "isaac_distribution": version("isaacsim"),
        "physics_hz": PHYSICS_HZ,
        "frames_per_phase": ARGS.frames_per_phase,
        "self_collision_policy": (
            "merged_q27_disabled; external collisions retained; "
            "Hand2 internal self-collision not qualified"
        ),
        "instances": {
            runtime.side: {
                "arm_instance": runtime.arm_instance_id,
                "hand_instance": runtime.hand_instance_id,
                "arm_asset": runtime.arm_asset.as_posix(),
                "arm_asset_sha256": sha256_file(runtime.arm_asset),
                "hand_asset": runtime.hand_asset.as_posix(),
                "hand_asset_sha256": sha256_file(runtime.hand_asset),
                "articulation_root": authored[runtime.side].articulation_root_path,
                "dof_names": list(articulations[runtime.side].dof_names),
                "dof_paths": list(_dof_paths(articulations[runtime.side])),
                "arm_indices_q7": list(partitions[runtime.side].arm_indices_q7),
                "hand_indices_q20": list(partitions[runtime.side].hand_indices_q20),
                "mount_pose": {
                    "position_m": list(runtime.mount_pose.position_m),
                    "quat_wxyz": list(runtime.mount_pose.quat_wxyz),
                },
                "attachment_assumption": runtime.attachment.assumption,
                "attachment_transform": {
                    "position_m": list(runtime.attachment.transform.position_m),
                    "quat_wxyz": list(runtime.attachment.transform.quat_wxyz),
                },
            }
            for runtime in SIDES
        },
        "tabletop_qualification_profile": {
            "path": RESOLVED.session.runtime.compatibility_profile,
            "profile_id": TABLETOP_PROFILE.profile_id,
            "status": TABLETOP_PROFILE.status,
            "sha256": sha256_file(TABLETOP_PROFILE_PATH),
            "initial_arm_q7_rad": {
                side: initial_arm_targets[side].tolist() for side in ("left", "right")
            },
            "assumptions": list(TABLETOP_PROFILE.assumptions),
            "arm_drive_gains": {
                "configured": {
                    "stiffness": TABLETOP_PROFILE.arm_drive_gains.stiffness,
                    "damping": TABLETOP_PROFILE.arm_drive_gains.damping,
                },
                "initial_runtime": arm_drive_runtime,
                "post_reset_runtime": arm_drive_after_reset,
            },
            "geometry_measurements": geometry_measurements,
        },
        "nero_link_geometry_alignment": {
            "binding_profile": ALIGNMENT_PROFILE_REFERENCE,
            "alignment_id": ALIGNMENT_PROFILE.alignment_id,
            "status": ALIGNMENT_PROFILE.status,
            "sha256": sha256_file(ALIGNMENT_PROFILE_PATH),
            "source_urdf": NERO_LULA_SOURCE_URDF.relative_to(ROOT).as_posix(),
            "source_urdf_sha256": ALIGNMENT_PROFILE.source_urdf_sha256,
            "link_name": ALIGNMENT_PROFILE.link_name,
            "geometry_post_rotation_quat_wxyz": list(
                ALIGNMENT_PROFILE.geometry_post_rotation_quat_wxyz
            ),
            "source_cylinder_axis_local_xyz": list(
                ALIGNMENT_PROFILE.source_cylinder_axis_local_xyz
            ),
            "corrected_cylinder_axis_local_xyz": list(
                ALIGNMENT_PROFILE.corrected_cylinder_axis_local_xyz
            ),
            "source_cylinder_positive_face_center_local_xyz": list(
                ALIGNMENT_PROFILE.source_cylinder_positive_face_center_local_xyz
            ),
            "corrected_cylinder_positive_face_center_local_xyz": list(
                ALIGNMENT_PROFILE.corrected_cylinder_positive_face_center_local_xyz
            ),
            "stage_paths": {
                side: {
                    "link": geometry_alignments[side].link_path,
                    "visual": geometry_alignments[side].visual_path,
                    "collision": geometry_alignments[side].collision_path,
                }
                for side in ("left", "right")
            },
            "kinematics_modified": False,
            "lula_uses_pinned_source_urdf": True,
            "assumptions": list(ALIGNMENT_PROFILE.assumptions),
        },
        "targets": {
            "arm": {
                "left_joint1_delta_rad": ARGS.arm_amplitude_rad,
                "right_joint2_delta_rad": -ARGS.arm_amplitude_rad,
            },
            "single_finger_amplitude_rad": ARGS.hand_amplitude_rad,
            "single_finger_phases": [
                {
                    "phase_id": phase.target.phase_id,
                    "side": phase.target.side.value,
                    "joint_names": list(phase.target.commanded_joint_names),
                    "command_delta_rad": phase.target.command_delta_rad,
                }
                for phase in finger_phases
            ],
            "combined_hand_phases": [
                {
                    "phase_id": phase.target.phase_id,
                    "side": phase.target.side.value,
                    "joint_names": list(phase.target.commanded_joint_names),
                    "command_delta_rad": phase.target.command_delta_rad,
                }
                for phase in combined_phases
            ],
            "post_reset_recovery": {
                "side": recovery_side,
                "joint_name": recovery_joint_name,
                "command_delta_rad": recovery_delta_rad,
            },
        },
        "single_digit_motion_diagnostics": {
            "scripted_phases": single_digit_motion_diagnostics,
            "post_reset_recovery": {
                "commanded_digit": recovery_digit_partition.commanded_digit,
                "other_fingers_max_feedback_delta_rad": max_phase_delta(
                    "post_reset_recovery",
                    "post_reset_settle_b",
                    recovery_side,
                    recovery_other_digit_runtime_indices,
                ),
                "same_digit_uncommanded_max_feedback_delta_rad": max_phase_delta(
                    "post_reset_recovery",
                    "post_reset_settle_b",
                    recovery_side,
                    recovery_same_digit_uncommanded_runtime_indices,
                ),
                "same_digit_linkage_is_gate_check": False,
                "other_fingers_isolation_tolerance_rad": (MOTION_ISOLATION_TOLERANCE_RAD),
            },
        },
        "topology_reset": {
            "articulation_root_paths_before_reset": list(root_paths_before_reset),
            "articulation_root_paths_after_reset": list(root_paths_after_reset),
            "partitions_stable": partitions_after_reset == partitions_before_reset,
        },
        "external_collision_settling": {
            "fixed_workcell_collider_paths": external_fixed_collider_paths,
            "settling_policy_id": FULL_SCRIPTED_Q27_SETTLING_POLICY.policy_id,
            "settling_window_frames": (FULL_SCRIPTED_Q27_SETTLING_POLICY.window_frames),
            "settling_min_windows": (FULL_SCRIPTED_Q27_SETTLING_POLICY.minimum_windows),
            "settling_max_windows": (FULL_SCRIPTED_Q27_SETTLING_POLICY.maximum_windows),
            "settling_tolerance_rad": (FULL_SCRIPTED_Q27_SETTLING_POLICY.max_window_delta_rad),
            "settling_records": settling_records,
            "initial_max_feedback_delta_rad": max_all_sides_delta(
                "initial",
                "initial_settle_a",
            ),
            "post_reset_max_feedback_delta_rad": max_all_sides_delta(
                "post_reset_settle_b",
                "post_reset_settle_a",
            ),
            "reset_to_initial_max_feedback_delta_rad": max_all_sides_delta(
                "post_reset_settle_b",
                "initial",
            ),
            "deliberate_unknown_penetration_probe": False,
            "evidence_boundary": (
                "fixed external colliders retained; bounded q27 rest "
                "convergence before every scripted hand baseline and after "
                "reset; no deliberate unknown penetration/contact scenario "
                "introduced"
            ),
        },
        "glove_live": glove_live_report,
        "feedback": feedback,
        "checks": checks,
        "screenshot": {
            "path": (None if ARGS.screenshot is None else ARGS.screenshot.resolve().as_posix()),
            "camera_prim_path": SCREENSHOT_CAMERA_PRIM_PATH,
            "camera_eye_frame": OBLIQUE_CAMERA_EYE_FRAME,
            "camera_target_frame": OBLIQUE_CAMERA_TARGET_FRAME,
            "camera_eye": list(oblique_camera_eye),
            "camera_target": list(oblique_camera_target),
            **screenshot_runtime,
        },
        "top_screenshot": {
            "path": (
                None if ARGS.top_screenshot is None else ARGS.top_screenshot.resolve().as_posix()
            ),
            "camera_prim_path": SCREENSHOT_CAMERA_PRIM_PATH,
            "camera_eye_frame": TOP_CAMERA_EYE_FRAME,
            "camera_target_frame": TOP_CAMERA_TARGET_FRAME,
            "camera_eye": list(top_camera_eye),
            "camera_target": list(top_camera_target),
            **top_screenshot_runtime,
        },
        "interface_screenshot": {
            "path": (
                None
                if ARGS.interface_screenshot is None
                else ARGS.interface_screenshot.resolve().as_posix()
            ),
            "camera_prim_path": SCREENSHOT_CAMERA_PRIM_PATH,
            "camera_eye_frame": INTERFACE_CAMERA_EYE_FRAME,
            "camera_target_frame": INTERFACE_CAMERA_TARGET_FRAME,
            "camera_eye": list(interface_camera_eye),
            "camera_target": list(interface_camera_target),
            **interface_screenshot_runtime,
        },
        "passed": passed,
    }
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if ARGS.report is not None:
        ARGS.report.parent.mkdir(parents=True, exist_ok=True)
        ARGS.report.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return qualification_gate_exit_code(passed)


try:
    exit_code = main()
finally:
    simulation_app.close()
raise SystemExit(exit_code)
