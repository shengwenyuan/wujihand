#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Run the resolved NV-2 dual NERO + physical Hand 2 simulation Session.

This entry point never connects to ROS, CAN, NERO hardware, or Hand 2 hardware.
It resolves all five configuration layers before starting Isaac, authors one
q27 articulation per side, and performs bounded q7/q20 isolation phases
suitable for the NV-2 simulation Gate.  Explicit opt-in modes may either read
one Wuji Glove for one simulated Hand 2 or consume canonical Tracker samples
over loopback for the simulated right NERO; the two input tests are isolated.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from importlib.metadata import version
import json
from pathlib import Path
import sys
import time
from collections.abc import Callable
from typing import Any, cast

import numpy as np
import numpy.typing as npt


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.application.supervision import JointCommandSupervisor
from wujihand.application.qualification import (
    Hand2QualificationTarget,
    build_hand2_qualification_targets,
    partition_hand2_single_digit_indices,
    qualification_gate_exit_code,
)
from wujihand.application.teleoperation import (
    GloveHand2SimulationController,
    RelativeTrackerTranslationMapper,
    compose_q27_hand_target,
)
from wujihand.domain import HandSide
from wujihand.domain.pose import rotation_matrix_to_quaternion_wxyz
from wujihand.runtime import SessionResolver
from wujihand.specs import AttachmentSpec, PoseSpec


DEFAULT_SESSION = (
    ROOT / "configs/sessions/isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--gui", action="store_true")
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
            "Opt in to bounded canonical Tracker XYZ control of only the "
            "simulated right NERO."
        ),
    )
    parser.add_argument("--tracker-serial")
    parser.add_argument("--tracker-udp-port", type=int, default=49154)
    parser.add_argument("--tracker-frames", type=int, default=2400)
    parser.add_argument("--tracker-scale", type=float, default=0.25)
    parser.add_argument("--tracker-max-delta-m", type=float, default=0.08)
    parser.add_argument("--tracker-stale-s", type=float, default=0.25)
    parser.add_argument(
        "--tracker-auto-reference",
        action="store_true",
        help="Establish the reference without waiting for Enter (CI/HIL only).",
    )
    return parser.parse_args()


ARGS = parse_args()
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
if not 1 <= ARGS.tracker_udp_port <= 65535:
    raise SystemExit("--tracker-udp-port must be in [1, 65535]")
if ARGS.tracker_frames < 1:
    raise SystemExit("--tracker-frames must be positive")
if not 0.0 < ARGS.tracker_scale <= 1.0:
    raise SystemExit("--tracker-scale must be in (0, 1]")
if not 0.0 < ARGS.tracker_max_delta_m <= 0.15:
    raise SystemExit("--tracker-max-delta-m must be in (0, 0.15]")
if not 0.05 <= ARGS.tracker_stale_s <= 1.0:
    raise SystemExit("--tracker-stale-s must be in [0.05, 1.0]")

RESOLVED = SessionResolver(ROOT).resolve(
    ARGS.session,
    verify_artifacts=ARGS.verify_artifacts,
)
if RESOLVED.session.backend != "isaac" or RESOLVED.session.runtime_role != "simulation":
    raise SystemExit("NV-2 runner requires an Isaac simulation Session")
if RESOLVED.session.runtime.transport_contract is not None:
    raise SystemExit("NV-2 scripted runner must not declare a transport contract")

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
    NeroHand2AttachmentConfig,
    NeroHand2AttachmentHandles,
    NeroHand2DofPartition,
    author_nero_hand2_attachment,
    discover_nero_hand2_dofs,
    load_nero_dual_tabletop_qualification_profile,
    load_hand2_model_profile,
)
from wujihand.adapters.simulation.nero_model import (
    NERO_JOINT_NAMES,
    NeroModelProfile,
    load_nero_model_profile,
)
from wujihand.adapters.input import (
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
SETTLING_WINDOW_FRAMES = 60
REST_SETTLING_DELTA_TOLERANCE_RAD = 0.005
REST_SETTLING_MIN_WINDOWS = 2
REST_SETTLING_MAX_WINDOWS = 8
RESET_INITIAL_TOLERANCE_RAD = 0.08
TRACKER_REFERENCE_WAIT_S = 10.0
TRACKER_RESPONSE_MIN_RAD = 0.005
TRACKER_LEFT_FEEDBACK_TOLERANCE_RAD = 0.03
TRACKER_HAND_FEEDBACK_TOLERANCE_RAD = 0.10
TRACKER_STREAM_ID = "vive.right"
TRACKER_LOGICAL_ROLE = "operator_right"
TRACKER_FRAME = "vive_tracking"
# Workstation2 measured OpenVR standing frame:
# body right = Tracker -Z, body forward = Tracker -X, body up = Tracker +Y.
# Workcell frame: +X right, +Y table-inward, +Z up.
TRACKER_TO_WORKCELL = (
    (0.0, 0.0, -1.0),   # Workcell X = -Tracker Z
    (-1.0, 0.0, 0.0),   # Workcell Y = -Tracker X
    (0.0, 1.0, 0.0),    # Workcell Z =  Tracker Y
)
NERO_LULA_DESCRIPTION = (
    ROOT / "configs/profiles/agilex_nero_lula_kinematics_v1.yaml"
)
NERO_LULA_URDF = (
    ROOT / "third_party/src/agx_arm_urdf/nero/urdf/nero_description.urdf"
)
NERO_LULA_URDF_SHA256 = (
    "c297c4bd2caeff44c673ae69070fc80f950510c0cb33cfa8b81b5bc774e91278"
)
SCREENSHOT_CAMERA_PRIM_PATH = "/OmniverseKit_Persp"
OBLIQUE_CAMERA_EYE_FRAME = "simulation_nominal_camera_oblique_eye"
OBLIQUE_CAMERA_TARGET_FRAME = "simulation_nominal_camera_oblique_target"
TOP_CAMERA_EYE_FRAME = "simulation_nominal_camera_top_eye"
TOP_CAMERA_TARGET_FRAME = "simulation_nominal_camera_top_target"


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
if RESOLVED.session.runtime.compatibility_profile is None:
    raise RuntimeError(
        "NV-2 tabletop qualification requires a Session compatibility profile"
    )
TABLETOP_PROFILE_PATH = ROOT / RESOLVED.session.runtime.compatibility_profile
TABLETOP_PROFILE = load_nero_dual_tabletop_qualification_profile(
    TABLETOP_PROFILE_PATH
)


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
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    direction = matrix.TransformDir(Gf.Vec3d(*local_axis_xyz))
    result = np.asarray(tuple(direction), dtype=np.float64)
    norm = float(np.linalg.norm(result))
    if not np.isfinite(result).all() or not np.isclose(norm, 1.0, atol=1e-6):
        raise RuntimeError(
            f"axis measurement is not a finite unit vector for {prim_path}: "
            f"{result.tolist()}"
        )
    return result / norm


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
    camera_world_transform = UsdGeom.Xformable(
        camera_prim
    ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    capture = capture_viewport_to_file(viewport, file_path=str(path))
    captured = simulation_app.run_coroutine(capture.wait_for_result(completion_frames=30))
    omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
    if not captured or not path.is_file():
        raise RuntimeError("Isaac viewport capture did not complete")
    return {
        "active_viewport_camera_path": str(viewport.camera_path),
        "camera_prim_valid": True,
        "camera_world_transform_row_major": [
            [
                float(camera_world_transform[row][column])
                for column in range(4)
            ]
            for row in range(4)
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
    input_adapter = WujiGloveHandSkeletonAdapter(
        side,
        source_id=f"wuji_glove.{side_name}.isaac_live",
        calibration_id=ARGS.glove_calibration_id,
        transform_id="wuji_glove.hand_skeleton.v1",
        serial_number=ARGS.glove_serial,
        address=ARGS.glove_address,
        device_name=f"nv2_glove_{side_name}",
    )
    retargeter = WujiHand2RetargetAdapter(side)
    supervisor = JointCommandSupervisor(
        hand_profiles[side_name].layout,
        hand_targets[side_name].tolist(),
        stale_after_s=0.25,
        velocity_scale=0.20,
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
    last_retarget_model_id: str | None = None
    frame_period_ns = round(1_000_000_000 / PHYSICS_HZ)
    started_ns = time.monotonic_ns()
    last_tick_ns = started_ns
    deadline_ns = started_ns
    try:
        controller.start(now_ns=started_ns)
        for _ in range(ARGS.glove_frames):
            deadline_ns += frame_period_ns
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
                last_retarget_model_id = step.intent.retarget_model_id
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
            hand_targets[side_name] = decision.command.copy()
            apply_targets()
            world.step(render=ARGS.gui)
            selected_feedback = _positions(articulations[side_name])
            other_feedback = _positions(articulations[other_side])
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
            "minimum landmark confidence 0.90; [0.90,0.95) is accepted only "
            "through JointCommandSupervisor; >=0.95 is success"
        ),
        "selector": {
            "kind": selector_kind,
            "value": selector_value,
        },
        "simulation_frames": ARGS.glove_frames,
        "accepted_skeleton_frames": accepted_frames,
        "empty_polls": empty_polls,
        "rejected_skeleton_frames": rejected_frames,
        "degraded_intents": degraded_intents,
        "rejection_reasons": rejection_reasons,
        "supervision_reasons": supervision_reasons,
        "position_clamped_commands": clamped_commands,
        "rate_limited_commands": rate_limited_commands,
        "max_supervised_command_delta_rad": max_supervised_command_delta_rad,
        "selected_hand_max_feedback_delta_rad": (max_selected_hand_feedback_delta_rad),
        "selected_arm_max_feedback_delta_rad": (max_selected_arm_feedback_delta_rad),
        "last_retarget_model_id": last_retarget_model_id,
        "wall_duration_s": (finished_ns - started_ns) / 1_000_000_000,
        "other_side_max_feedback_delta_rad": max_other_side_feedback_delta_rad,
    }
    return report, feedback


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

    if sha256_file(NERO_LULA_URDF) != NERO_LULA_URDF_SHA256:
        raise RuntimeError("source-locked NERO URDF hash drifted")
    if not NERO_LULA_DESCRIPTION.is_file():
        raise RuntimeError(f"NERO Lula descriptor not found: {NERO_LULA_DESCRIPTION}")

    right_runtime = next(runtime for runtime in SIDES if runtime.side == "right")
    solver = LulaKinematicsSolver(
        str(NERO_LULA_DESCRIPTION),
        str(NERO_LULA_URDF),
    )
    if tuple(solver.get_joint_names()) != NERO_JOINT_NAMES:
        raise RuntimeError(
            "NERO Lula cspace differs from the canonical q7 layout: "
            f"{solver.get_joint_names()}"
        )
    if "link7" not in solver.get_all_frame_names():
        raise RuntimeError("NERO Lula model does not expose link7")
    solver.set_robot_base_pose(
        np.asarray(right_runtime.mount_pose.position_m, dtype=np.float64),
        np.asarray(right_runtime.mount_pose.quat_wxyz, dtype=np.float64),
    )

    initial_feedback = {
        side: _positions(articulations[side]) for side in ("left", "right")
    }
    initial_left_arm_command = arm_targets["left"].copy()
    initial_hand_commands = {
        side: hand_targets[side].copy() for side in ("left", "right")
    }
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
    reference_position_m, reference_rotation = solver.compute_forward_kinematics(
        "link7",
        initial_q7,
    )
    reference_position_m = np.asarray(reference_position_m, dtype=np.float64)
    reference_rotation = np.asarray(reference_rotation, dtype=np.float64)
    if (
        reference_position_m.shape != (3,)
        or not np.isfinite(reference_position_m).all()
        or reference_rotation.shape != (3, 3)
        or not np.isfinite(reference_rotation).all()
    ):
        raise RuntimeError("NERO Lula FK returned an invalid reference pose")
    reference_orientation_wxyz = rotation_matrix_to_quaternion_wxyz(
        reference_rotation
    )

    mapper = RelativeTrackerTranslationMapper(
        stream_id=TRACKER_STREAM_ID,
        device_serial=ARGS.tracker_serial,
        logical_role=TRACKER_LOGICAL_ROLE,
        tracking_frame=TRACKER_FRAME,
        tracker_to_world=TRACKER_TO_WORKCELL,
        scale=ARGS.tracker_scale,
        max_delta_m=ARGS.tracker_max_delta_m,
        stale_after_s=ARGS.tracker_stale_s,
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

    with UdpTrackingSampleReceiver(
        ARGS.tracker_udp_port,
        stream_id=TRACKER_STREAM_ID,
        device_serial=ARGS.tracker_serial,
        logical_role=TRACKER_LOGICAL_ROLE,
        tracking_frame=TRACKER_FRAME,
    ) as receiver:
        if not ARGS.tracker_auto_reference:
            print(
                "\n保持 Tracker 静止；确认 Isaac 场景就绪后按 Enter 建立 reference。",
                flush=True,
            )
            try:
                input()
            except EOFError as exc:
                raise RuntimeError(
                    "stdin closed before Tracker reference confirmation"
                ) from exc
        reference_requested_ns = time.monotonic_ns()
        reference_deadline_ns = reference_requested_ns + round(
            TRACKER_REFERENCE_WAIT_S * 1_000_000_000
        )
        reference_sample = None
        last_reference_error = "no canonical sample received"
        while time.monotonic_ns() < reference_deadline_ns:
            now_ns = time.monotonic_ns()
            candidate = receiver.receive_latest(now_ns=now_ns)
            if (
                candidate is not None
                and candidate.host_time_ns >= reference_requested_ns
            ):
                try:
                    mapper.arm(
                        candidate,
                        reference_position_m,
                        now_ns=now_ns,
                    )
                except ValueError as exc:
                    last_reference_error = str(exc)
                else:
                    reference_sample = candidate
                    break
            world.step(render=ARGS.gui)
            time.sleep(1.0 / PHYSICS_HZ)
        if reference_sample is None:
            raise RuntimeError(
                "no fresh actionable Tracker sample arrived within "
                f"{TRACKER_REFERENCE_WAIT_S:g}s: {last_reference_error}"
            )

        armed_ns = max(time.monotonic_ns(), reference_requested_ns + 1)
        supervisor.arm(armed_ns)
        print(
            "TRACKER_REFERENCE_READY "
            f"serial={reference_sample.device_serial} "
            f"position_m={list(reference_sample.position_m or ())} "
            "mapping='tracker[x,y,z] -> workcell[-z,-x,y]' "
            f"scale={ARGS.tracker_scale:g} clamp=±{ARGS.tracker_max_delta_m:g}m",
            flush=True,
        )
        print(
            "现在依次小幅移动 Tracker：左右、前后、上下；仅右 NERO 应响应。",
            flush=True,
        )

        frame_period_ns = round(1_000_000_000 / PHYSICS_HZ)
        next_deadline_ns = armed_ns
        last_tick_ns = armed_ns
        accepted_samples = 0
        clamped_samples = 0
        ik_successes = 0
        ik_failures = 0
        consecutive_ik_failures = 0
        completed_frames = 0
        termination_reason = "completed"
        max_world_delta_m = 0.0
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
                clamped_samples += int(mapping.clamped)
            if mapping.requires_reference or mapping.target_position_m is None:
                termination_reason = mapping.reason
                break
            assert mapping.input_host_time_ns is not None
            if mapping.world_delta_m is not None:
                max_world_delta_m = max(
                    max_world_delta_m,
                    float(np.linalg.norm(mapping.world_delta_m)),
                )

            solution, ik_success = solver.compute_inverse_kinematics(
                "link7",
                np.asarray(mapping.target_position_m, dtype=np.float64),
                reference_orientation_wxyz,
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
            completed_frames += 1
            if frame_index % 60 == 0:
                world_delta = (
                    None
                    if mapping.world_delta_m is None
                    else [round(value, 4) for value in mapping.world_delta_m]
                )
                print(
                    f"tracker_frame={frame_index:04d} "
                    f"world_delta_m={world_delta} "
                    f"ik={'ok' if ik_success else 'hold'} "
                    "q7="
                    f"{[round(float(value), 3) for value in arm_targets['right']]}",
                    flush=True,
                )
            last_tick_ns = tick_ns
            remaining_s = (
                next_deadline_ns - time.monotonic_ns()
            ) / 1_000_000_000
            if remaining_s > 0.0:
                time.sleep(remaining_s)

        completed = (
            termination_reason == "completed"
            and completed_frames == ARGS.tracker_frames
        )
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
        passed = bool(
            completed
            and accepted_samples > 0
            and ik_successes > 0
            and max_right_arm_feedback_delta_rad >= TRACKER_RESPONSE_MIN_RAD
            and right_hand_command_held
            and left_commands_held
            and max_right_hand_feedback_delta_rad
            <= TRACKER_HAND_FEEDBACK_TOLERANCE_RAD
            and max_left_feedback_delta_rad
            <= TRACKER_LEFT_FEEDBACK_TOLERANCE_RAD
        )
        report = {
            "schema": "wujihand.isaac_tracker_right_nero_smoke.v1",
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
                "tracking_frame": TRACKER_FRAME,
                "udp_endpoint": f"127.0.0.1:{ARGS.tracker_udp_port}",
                "accepted_samples": accepted_samples,
                "receiver_accepted_drains": receiver.accepted,
                "receiver_rejected_datagrams": receiver.rejected,
            },
            "mapping": {
                "mode": "relative_xyz_translation_only",
                "tracker_to_workcell": [
                    list(row) for row in TRACKER_TO_WORKCELL
                ],
                "scale": ARGS.tracker_scale,
                "max_delta_each_axis_m": ARGS.tracker_max_delta_m,
                "stale_after_s": ARGS.tracker_stale_s,
                "clamped_samples": clamped_samples,
                "max_world_delta_norm_m": max_world_delta_m,
                "fixed_link7_orientation_wxyz": (
                    reference_orientation_wxyz.tolist()
                ),
            },
            "kinematics": {
                "solver": "Isaac Sim 6.0.1 LulaKinematicsSolver",
                "descriptor": NERO_LULA_DESCRIPTION.relative_to(ROOT).as_posix(),
                "urdf": NERO_LULA_URDF.relative_to(ROOT).as_posix(),
                "urdf_sha256": NERO_LULA_URDF_SHA256,
                "end_effector_frame": "link7",
                "ik_successes": ik_successes,
                "ik_failures": ik_failures,
                "supervision_reasons": supervision_reasons,
            },
            "runtime": {
                "requested_frames": ARGS.tracker_frames,
                "completed_frames": completed_frames,
                "termination_reason": termination_reason,
                "last_mapping_reason": (
                    None if last_mapping is None else last_mapping.reason
                ),
            },
            "measurements": {
                "right_arm_max_feedback_delta_rad": (
                    max_right_arm_feedback_delta_rad
                ),
                "right_hand_max_feedback_delta_rad": (
                    max_right_hand_feedback_delta_rad
                ),
                "left_q27_max_feedback_delta_rad": (
                    max_left_feedback_delta_rad
                ),
                "right_hand_command_held": right_hand_command_held,
                "left_arm_and_hand_commands_held": left_commands_held,
            },
            "checks": {
                "bounded_run_completed": completed,
                "fresh_tracker_samples_received": accepted_samples > 0,
                "ik_succeeded": ik_successes > 0,
                "right_arm_responded": (
                    max_right_arm_feedback_delta_rad
                    >= TRACKER_RESPONSE_MIN_RAD
                ),
                "right_hand_command_held": right_hand_command_held,
                "right_hand_feedback_bounded": (
                    max_right_hand_feedback_delta_rad
                    <= TRACKER_HAND_FEEDBACK_TOLERANCE_RAD
                ),
                "left_commands_held": left_commands_held,
                "left_articulation_feedback_bounded": (
                    max_left_feedback_delta_rad
                    <= TRACKER_LEFT_FEEDBACK_TOLERANCE_RAD
                ),
            },
            "passed": passed,
        }
        encoded = (
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
            + "\n"
        )
        if ARGS.report is not None:
            ARGS.report.parent.mkdir(parents=True, exist_ok=True)
            ARGS.report.write_text(encoded, encoding="utf-8")
        print(encoded, end="")
        return qualification_gate_exit_code(passed)


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
                f"NV-2 nominal qualification requires fixed workcell entities: "
                f"{entity.entity_id}"
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
    articulations: dict[str, Articulation] = {}
    partitions: dict[str, NeroHand2DofPartition] = {}
    arm_profiles: dict[str, NeroModelProfile] = {}
    hand_profiles: dict[str, Hand2ModelProfile] = {}
    initial_arm_targets: dict[str, npt.NDArray[np.float64]] = {}
    for runtime in SIDES:
        add_reference_to_stage(str(runtime.arm_asset), runtime.arm_prim_path)
        arm_root = stage.GetPrimAtPath(runtime.arm_prim_path)
        _set_world_pose(arm_root, runtime.mount_pose)
        add_reference_to_stage(str(runtime.hand_asset), runtime.hand_prim_path)

        arm_articulation_root = _one_prim(
            stage,
            prefix=runtime.arm_prim_path,
            articulation_root=True,
        )
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
                raise RuntimeError(
                    f"{side} qualification q7 drive gains were not applied"
                )
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
            if (
                str(prim.GetPath()) == prefix
                or str(prim.GetPath()).startswith(prefix + "/")
            )
            and prim.HasAPI(UsdPhysics.CollisionAPI)
        ]
        if not matches:
            raise RuntimeError(
                f"fixed workcell entity has no authored external collider: {prefix}"
            )
        external_fixed_collider_paths.extend(matches)
    external_fixed_collider_paths = sorted(set(external_fixed_collider_paths))
    if not external_fixed_collider_paths:
        raise RuntimeError("NV-2 nominal workcell has no fixed external collider")

    arm_targets = {
        side: initial_arm_targets[side].copy() for side in ("left", "right")
    }
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
            side: _positions(articulations[side]).tolist()
            for side in ("left", "right")
        }

    def alias_feedback(source_key: str, alias_key: str) -> None:
        if alias_key in feedback:
            raise RuntimeError(f"duplicate feedback phase key: {alias_key}")
        feedback[alias_key] = {
            side: list(feedback[source_key][side])
            for side in ("left", "right")
        }

    def settle_until_stable(settling_id: str) -> tuple[str, str]:
        """Wait for two consecutive bounded q27 windows or fail closed."""

        if settling_id in settling_records:
            raise RuntimeError(f"duplicate settling phase: {settling_id}")
        window_keys: list[str] = []
        max_delta_history_rad: list[float] = []
        previous_key: str | None = None
        for window_number in range(1, REST_SETTLING_MAX_WINDOWS + 1):
            _step(world, SETTLING_WINDOW_FRAMES, render=ARGS.gui)
            current_key = f"{settling_id}_settle_{window_number:02d}"
            capture_feedback(current_key)
            window_keys.append(current_key)
            if previous_key is not None:
                max_delta_rad = max(
                    float(
                        np.max(
                            np.abs(
                                np.asarray(feedback[current_key][side])
                                - np.asarray(feedback[previous_key][side])
                            )
                        )
                    )
                    for side in ("left", "right")
                )
                max_delta_history_rad.append(max_delta_rad)
                if (
                    window_number >= REST_SETTLING_MIN_WINDOWS
                    and max_delta_rad <= REST_SETTLING_DELTA_TOLERANCE_RAD
                ):
                    settling_records[settling_id] = {
                        "converged": True,
                        "window_frames": SETTLING_WINDOW_FRAMES,
                        "windows_run": window_number,
                        "window_keys": list(window_keys),
                        "max_delta_history_rad": list(max_delta_history_rad),
                        "peak_max_delta_rad": max(max_delta_history_rad),
                        "final_max_delta_rad": max_delta_rad,
                        "tolerance_rad": REST_SETTLING_DELTA_TOLERANCE_RAD,
                        "measured_scope": "both q27 articulations",
                    }
                    return previous_key, current_key
            previous_key = current_key

        settling_records[settling_id] = {
            "converged": False,
            "window_frames": SETTLING_WINDOW_FRAMES,
            "windows_run": REST_SETTLING_MAX_WINDOWS,
            "window_keys": list(window_keys),
            "max_delta_history_rad": list(max_delta_history_rad),
            "peak_max_delta_rad": max(max_delta_history_rad),
            "final_max_delta_rad": max_delta_history_rad[-1],
            "tolerance_rad": REST_SETTLING_DELTA_TOLERANCE_RAD,
            "measured_scope": "both q27 articulations",
        }
        raise RuntimeError(
            f"{settling_id} did not settle within "
            f"{REST_SETTLING_MAX_WINDOWS} windows: "
            f"last max q27 delta={max_delta_history_rad[-1]:.6f} rad"
        )

    apply_targets()
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
            profile.layout.names.index(name)
            for name in target.commanded_joint_names
        )
        hand_targets[side] = profile.layout.validate_vector(target.q20_rad).copy()
        apply_targets()
        _step(world, ARGS.frames_per_phase, render=ARGS.gui)
        command_key = f"{target.phase_id}_command"
        capture_feedback(command_key)
        commanded_runtime_indices = tuple(
            partitions[side].hand_indices_q20[index]
            for index in commanded_profile_indices
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
    if ARGS.glove_live:
        glove_live_report, live_feedback = _run_glove_live(
            world,
            articulations=articulations,
            partitions=partitions,
            hand_profiles=hand_profiles,
            hand_targets=hand_targets,
            apply_targets=apply_targets,
        )
        feedback.update(live_feedback)

    partitions_before_reset = dict(partitions)
    world.reset()
    partitions_after_reset, root_paths_after_reset = validate_articulations()
    arm_drive_after_reset = apply_qualification_arm_drive_gains(
        partitions_after_reset
    )
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
    post_reset_previous_key, post_reset_final_key = settle_until_stable(
        "post_reset"
    )
    alias_feedback(post_reset_previous_key, "post_reset_settle_a")
    alias_feedback(post_reset_final_key, "post_reset_settle_b")

    recovery_side = "left"
    recovery_profile = hand_profiles[recovery_side]
    recovery_joint_name = "l_middle_finger_pip"
    recovery_profile_index = recovery_profile.layout.names.index(recovery_joint_name)
    recovery_runtime_index = partitions[recovery_side].hand_indices_q20[
        recovery_profile_index
    ]
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
        return max(
            max_phase_delta(current, previous, side)
            for side in ("left", "right")
        )

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
        "fixed_external_workcell_collider_present": bool(
            external_fixed_collider_paths
        ),
        "initial_two_window_settling_finite": bool(
            np.isfinite(arrays["initial_settle_a"]["left"]).all()
            and np.isfinite(arrays["initial_settle_a"]["right"]).all()
            and np.isfinite(arrays["initial"]["left"]).all()
            and np.isfinite(arrays["initial"]["right"]).all()
        ),
        "initial_two_window_settling_bounded": bool(
            max_all_sides_delta("initial", "initial_settle_a")
            <= REST_SETTLING_DELTA_TOLERANCE_RAD
        ),
        "post_reset_two_window_settling_finite": bool(
            np.isfinite(arrays["post_reset_settle_a"]["left"]).all()
            and np.isfinite(arrays["post_reset_settle_a"]["right"]).all()
            and np.isfinite(arrays["post_reset_settle_b"]["left"]).all()
            and np.isfinite(arrays["post_reset_settle_b"]["right"]).all()
        ),
        "post_reset_two_window_settling_bounded": bool(
            max_all_sides_delta("post_reset_settle_b", "post_reset_settle_a")
            <= REST_SETTLING_DELTA_TOLERANCE_RAD
        ),
        "post_command_reset_two_q27_revalidated": topology_stable_after_reset,
        "post_command_reset_returned_to_approved_initial": bool(
            max_all_sides_delta("post_reset_settle_b", "initial")
            <= RESET_INITIAL_TOLERANCE_RAD
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
            partition.hand_indices_q20[index]
            for index in digit_partition.other_digit_indices
        )
        same_digit_uncommanded_runtime_indices = tuple(
            partition.hand_indices_q20[index]
            for index in digit_partition.same_digit_uncommanded_indices
        )
        response = (
            arrays[phase.command_key][side][
                np.asarray(commanded_runtime, dtype=np.int64)
            ]
            - arrays[phase.baseline_key][side][
                np.asarray(commanded_runtime, dtype=np.int64)
            ]
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
            "same_digit_uncommanded_max_feedback_delta_rad": (
                same_digit_uncommanded_max_delta_rad
            ),
            "same_digit_linkage_is_gate_check": False,
            "other_fingers_isolation_tolerance_rad": (
                MOTION_ISOLATION_TOLERANCE_RAD
            ),
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

    if ARGS.glove_live:
        accounted_frames = (
            cast(int, glove_live_report["accepted_skeleton_frames"])
            + cast(int, glove_live_report["empty_polls"])
            + cast(int, glove_live_report["rejected_skeleton_frames"])
        )
        checks["glove_live_observation_received"] = bool(
            cast(int, glove_live_report["accepted_skeleton_frames"]) > 0
        )
        checks["glove_live_frames_accounted"] = bool(accounted_frames == ARGS.glove_frames)
        checks["glove_live_supervised_command_changed"] = bool(
            cast(float, glove_live_report["max_supervised_command_delta_rad"])
            >= LIVE_HAND_RESPONSE_MIN_RAD
        )
        checks["glove_live_selected_hand_responded"] = bool(
            cast(float, glove_live_report["selected_hand_max_feedback_delta_rad"])
            >= LIVE_HAND_RESPONSE_MIN_RAD
        )
        checks["glove_live_selected_arm_held"] = bool(
            cast(float, glove_live_report["selected_arm_max_feedback_delta_rad"])
            <= MOTION_ISOLATION_TOLERANCE_RAD
        )
        checks["glove_live_other_side_held"] = bool(
            cast(float, glove_live_report["other_side_max_feedback_delta_rad"])
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
                arm_values
                >= np.asarray(arm_profile.layout.lower)
                - FEEDBACK_LIMIT_TOLERANCE_RAD
            )
            and np.all(
                arm_values
                <= np.asarray(arm_profile.layout.upper)
                + FEEDBACK_LIMIT_TOLERANCE_RAD
            )
            and np.all(
                hand_values
                >= np.asarray(hand_profile.layout.lower)
                - FEEDBACK_LIMIT_TOLERANCE_RAD
            )
            and np.all(
                hand_values
                <= np.asarray(hand_profile.layout.upper)
                + FEEDBACK_LIMIT_TOLERANCE_RAD
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
        checks[f"{side}_post_reset_feedback_within_limits"] = (
            feedback_rows_within_limits(side, reset_rows)
        )
        arm_indices = np.asarray(partitions[side].arm_indices_q7)
        initial_error = np.max(
            np.abs(arrays["initial"][side][arm_indices] - initial_arm_targets[side])
        )
        post_reset_error = np.max(
            np.abs(
                arrays["post_reset_settle_b"][side][arm_indices]
                - initial_arm_targets[side]
            )
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
        flange_axis_world = _world_axis(
            stage,
            attachment.parent_link_path,
            geometry.flange_forearm_axis_local_xyz,
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
        attachment_axis_dot = float(np.dot(hand_axis_world, flange_axis_world))
        base_port_outward_dot = float(
            np.dot(
                port_axis_world,
                np.asarray(geometry.table_outward_axis_world_xyz),
            )
        )
        hand_world_inward_dot = float(
            np.dot(
                hand_axis_world,
                np.asarray(geometry.table_inward_axis_world_xyz),
            )
        )
        hand_world_vertical_abs = float(abs(hand_axis_world[2]))
        hand_palm_down_dot = float(
            np.dot(
                palm_normal_world,
                np.asarray(geometry.table_down_axis_world_xyz),
            )
        )
        geometry_measurements[side] = {
            "flange_forearm_axis_world_xyz": flange_axis_world.tolist(),
            "hand_longitudinal_axis_world_xyz": hand_axis_world.tolist(),
            "hand_palm_normal_axis_world_xyz": palm_normal_world.tolist(),
            "base_port_axis_world_xyz": port_axis_world.tolist(),
            "attachment_axis_dot": attachment_axis_dot,
            "base_port_outward_dot": base_port_outward_dot,
            "hand_world_inward_dot": hand_world_inward_dot,
            "hand_world_vertical_abs": hand_world_vertical_abs,
            "hand_palm_down_dot": hand_palm_down_dot,
        }
        checks[f"{side}_attachment_axis_aligned"] = bool(
            attachment_axis_dot >= thresholds.attachment_axis_min_dot
        )
        checks[f"{side}_base_port_axis_outward"] = bool(
            base_port_outward_dot >= thresholds.base_port_outward_min_dot
        )
        checks[f"{side}_hand_axis_points_table_inward"] = bool(
            hand_world_inward_dot >= thresholds.hand_world_inward_min_dot
        )
        checks[f"{side}_hand_axis_near_horizontal"] = bool(
            hand_world_vertical_abs <= thresholds.hand_world_vertical_abs_max
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
            }
            for runtime in SIDES
        },
        "tabletop_qualification_profile": {
            "path": RESOLVED.session.runtime.compatibility_profile,
            "profile_id": TABLETOP_PROFILE.profile_id,
            "status": TABLETOP_PROFILE.status,
            "sha256": sha256_file(TABLETOP_PROFILE_PATH),
            "initial_arm_q7_rad": {
                side: initial_arm_targets[side].tolist()
                for side in ("left", "right")
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
                "other_fingers_isolation_tolerance_rad": (
                    MOTION_ISOLATION_TOLERANCE_RAD
                ),
            },
        },
        "topology_reset": {
            "articulation_root_paths_before_reset": list(
                root_paths_before_reset
            ),
            "articulation_root_paths_after_reset": list(
                root_paths_after_reset
            ),
            "partitions_stable": partitions_after_reset
            == partitions_before_reset,
        },
        "external_collision_settling": {
            "fixed_workcell_collider_paths": external_fixed_collider_paths,
            "settling_window_frames": SETTLING_WINDOW_FRAMES,
            "settling_min_windows": REST_SETTLING_MIN_WINDOWS,
            "settling_max_windows": REST_SETTLING_MAX_WINDOWS,
            "settling_tolerance_rad": REST_SETTLING_DELTA_TOLERANCE_RAD,
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
            "path": (
                None
                if ARGS.screenshot is None
                else ARGS.screenshot.resolve().as_posix()
            ),
            "camera_prim_path": SCREENSHOT_CAMERA_PRIM_PATH,
            "camera_eye_frame": OBLIQUE_CAMERA_EYE_FRAME,
            "camera_target_frame": OBLIQUE_CAMERA_TARGET_FRAME,
            "camera_eye": list(oblique_camera_eye),
            "camera_target": list(oblique_camera_target),
            **screenshot_runtime,
        },
        "top_screenshot": {
            "path": (
                None
                if ARGS.top_screenshot is None
                else ARGS.top_screenshot.resolve().as_posix()
            ),
            "camera_prim_path": SCREENSHOT_CAMERA_PRIM_PATH,
            "camera_eye_frame": TOP_CAMERA_EYE_FRAME,
            "camera_target_frame": TOP_CAMERA_TARGET_FRAME,
            "camera_eye": list(top_camera_eye),
            "camera_target": list(top_camera_target),
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


try:
    exit_code = main()
finally:
    simulation_app.close()
raise SystemExit(exit_code)
