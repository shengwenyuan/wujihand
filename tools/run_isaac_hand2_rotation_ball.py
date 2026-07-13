#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Run the fixed-XYZ, three-axis Hand 2 wrist and dynamic-ball scene.

The scripted source is a deterministic physics qualification attempt.  The UDP
source consumes atomic ``wujihand.hand_command.v2`` packets from the MediaPipe
process.  Neither path teleports, attaches, or kinematically freezes the ball.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
import traceback
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true", help="Open the Isaac Sim GUI.")
    parser.add_argument("--frames", type=int, default=1560, help="Physics frames to run.")
    parser.add_argument("--command-source", choices=("scripted", "udp"), default="scripted")
    parser.add_argument("--udp-port", type=int, default=49152)
    parser.add_argument(
        "--asset",
        type=Path,
        default=ROOT / "third_party/src/wuji-description/hand2_beta/body/usd/right/wujihand.usd",
    )
    parser.add_argument(
        "--profile",
        type=Path,
        default=ROOT / "configs/profiles/hand2_right_v2026_6_27.yaml",
    )
    parser.add_argument(
        "--scene-profile",
        type=Path,
        default=ROOT / "configs/base/hand2_rotation_ball_v1.yaml",
    )
    parser.add_argument(
        "--validation-output-dir",
        type=Path,
        help="Write the physics report and optional screenshot here.",
    )
    parser.add_argument(
        "--require-grasp-success",
        action="store_true",
        help="Exit non-zero unless the configured physics lift/contact/hold criteria pass.",
    )
    parser.add_argument(
        "--skip-screenshot",
        action="store_true",
        help="Do not capture a viewport image with validation output.",
    )
    parser.add_argument("--contact-threshold-n", type=float, default=0.05)
    return parser.parse_args()


ARGS = parse_args()
if ARGS.frames < 1:
    raise SystemExit("--frames must be positive")
if not 1 <= ARGS.udp_port <= 65535:
    raise SystemExit("--udp-port must be in [1, 65535]")
if ARGS.contact_threshold_n < 0.0:
    raise SystemExit("--contact-threshold-n must be non-negative")
for required_path, label in (
    (ARGS.asset, "Hand 2 USD"),
    (ARGS.profile, "Hand 2 profile"),
    (ARGS.scene_profile, "rotation-ball scene profile"),
):
    if not required_path.is_file():
        raise SystemExit(f"{label} not found: {required_path}")
if ARGS.require_grasp_success and ARGS.command_source != "scripted":
    raise SystemExit("--require-grasp-success currently requires --command-source scripted")

from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {"headless": not ARGS.gui, "width": 960, "height": 720, "anti_aliasing": 0}
)

import numpy as np
from pxr import UsdLux

import omni.kit.renderer_capture
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.xforms import get_world_pose
from wujihand.adapters.simulation import (
    BallLiftCriteria,
    BallLiftEvaluator,
    BallLiftSample,
    Hand2BallConfig,
    Hand2RotationMountConfig,
    add_hand2_ball_scene,
    author_rotation_mount,
    contact_groups_from_force_matrix,
    discover_rotation_mount_dofs,
    load_hand2_model_profile,
    set_rotation_mount_target_quaternion,
)
from wujihand.adapters.transport import UdpHandCommandReceiver
from wujihand.application.supervision import JointCommandSupervisor, PoseSupervisor, SafetyState
from wujihand.domain import HAND2_RIGHT_LAYOUT, HAND2_RIGHT_REST, PoseIntent
from wujihand.domain.pose import (
    IDENTITY_QUATERNION_WXYZ,
    quaternion_geodesic_distance_rad,
    quaternion_wxyz_to_rotation_matrix,
)
from wujihand.runtime import load_rotation_ball_config
from wujihand.runtime.rotation_ball_script import scripted_rotation_ball_target


PHYSICS_HZ = 120
COMMAND_HZ = 60
COMMAND_DIVISOR = PHYSICS_HZ // COMMAND_HZ
FEEDBACK_LIMIT_TOLERANCE_RAD = 0.01
SCRIPTED_CALIBRATION_ID = "scripted-neutral-v1"


def articulation_dof_paths(hand: Articulation) -> list[str]:
    """Read the PhysX-created DOF paths and fail if the expected single view is absent."""

    paths_by_articulation = getattr(hand, "_dof_paths", None)
    if not isinstance(paths_by_articulation, list) or len(paths_by_articulation) != 1:
        raise RuntimeError("Isaac articulation did not expose one initialized DOF path list")
    paths = [str(path) for path in paths_by_articulation[0]]
    if len(paths) != len(hand.dof_names):
        raise RuntimeError("Isaac DOF path/name counts differ")
    return paths


def ball_in_palm_frame(
    ball_position_m: np.ndarray,
    palm_position_m: np.ndarray,
    palm_quaternion_wxyz: np.ndarray,
) -> tuple[float, float, float]:
    rotation = quaternion_wxyz_to_rotation_matrix(palm_quaternion_wxyz)
    relative = rotation.T @ (ball_position_m - palm_position_m)
    return float(relative[0]), float(relative[1]), float(relative[2])


def sha256_file(path: Path) -> str:
    """Hash the actual USD bytes used by this run."""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    validation_output = ARGS.validation_output_dir
    if validation_output is not None:
        validation_output.mkdir(parents=True, exist_ok=True)
        for stale_name in (
            "validation.json",
            "hand2_rotation_ball.png",
            "error.txt",
        ):
            (validation_output / stale_name).unlink(missing_ok=True)

    profile = load_hand2_model_profile(ARGS.profile)
    scene_config = load_rotation_ball_config(ARGS.scene_profile)
    if profile.layout != HAND2_RIGHT_LAYOUT:
        raise RuntimeError("Hand 2 profile differs from the pinned firmware layout")
    if not np.array_equal(profile.rest_position, HAND2_RIGHT_REST):
        raise RuntimeError("Hand 2 profile differs from the pinned rest position")
    provenance_keys = ("repository", "tag", "commit", "usd", "usd_sha256")
    if any(
        profile.provenance.get(key) != scene_config.provenance.get(key)
        for key in provenance_keys
    ):
        raise RuntimeError("model and scene profiles disagree on pinned USD provenance")
    asset_sha256 = sha256_file(ARGS.asset.resolve())
    if asset_sha256 != profile.provenance.get("usd_sha256"):
        raise RuntimeError("actual Hand 2 USD SHA-256 differs from the pinned profile")

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / PHYSICS_HZ,
        rendering_dt=1.0 / 30.0,
        backend="numpy",
    )
    world.scene.add_default_ground_plane()
    world.scene.add(
        FixedCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.asarray(scene_config.table.position_m, dtype=np.float64),
            scale=np.asarray(scene_config.table.size_m, dtype=np.float64),
            size=1.0,
            color=np.asarray(scene_config.table.color_rgb, dtype=np.float32),
        )
    )
    add_reference_to_stage(str(ARGS.asset.resolve()), "/World/Hand2")
    stage = world.scene.stage
    mount = author_rotation_mount(
        stage,
        Hand2RotationMountConfig(
            flange_position_m=scene_config.flange.position_m,
            flange_orientation_wxyz=scene_config.flange.neutral_quat_wxyz,
            roll_limit_rad=scene_config.wrist.roll_limit_rad,
            pitch_limit_rad=scene_config.wrist.pitch_limit_rad,
            drive_stiffness=scene_config.wrist.drive.stiffness,
            drive_damping=scene_config.wrist.drive.damping,
            drive_max_force=scene_config.wrist.drive.max_force,
        ),
    )
    ball_scene = add_hand2_ball_scene(
        world,
        Hand2BallConfig(
            center_xyz_m=scene_config.ball.center_m,
            table_top_z_m=scene_config.table.top_z_m,
            radius_m=scene_config.ball.radius_m,
            mass_kg=scene_config.ball.mass_kg,
            static_friction=scene_config.ball.static_friction,
            dynamic_friction=scene_config.ball.dynamic_friction,
            restitution=scene_config.ball.restitution,
            color_rgb=scene_config.ball.color_rgb,
        ),
    )
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(900.0)
    hand = world.scene.add(
        Articulation(mount.paths.articulation_root_path, name="hand2_right_rotation_mount")
    )
    world.reset()
    ball_scene.contact_view.initialize(world.physics_sim_view)
    ball_scene.hand_table_contact_view.initialize(world.physics_sim_view)

    camera_eye = np.asarray([0.46, -0.48, 0.78], dtype=np.float64)
    camera_target = np.asarray([0.08, 0.0, 0.44], dtype=np.float64)
    set_camera_view(
        eye=camera_eye,
        target=camera_target,
        camera_prim_path="/OmniverseKit_Persp",
    )
    world.step(render=True)

    backend_names = list(hand.dof_names)
    backend_paths = articulation_dof_paths(hand)
    partition = discover_rotation_mount_dofs(
        backend_names,
        backend_paths,
        profile.layout.names,
        mount.paths.rotation_joint_path,
    )
    if len(backend_names) != 23:
        raise RuntimeError(f"expected wrist3 + finger20, got {len(backend_names)} DOFs")

    usd_limits = np.asarray(hand.get_dof_limits(), dtype=np.float64)[0]
    finger_limits = usd_limits[np.asarray(partition.finger_indices_q20, dtype=np.int64)]
    expected_finger_limits = np.column_stack((profile.layout.lower, profile.layout.upper))
    if not np.allclose(finger_limits, expected_finger_limits, atol=1e-4):
        error = float(np.max(np.abs(finger_limits - expected_finger_limits)))
        raise RuntimeError(f"USD finger limits differ from profile: max_error={error}")
    expected_wrist_limits = np.asarray(
        [
            [-scene_config.wrist.roll_limit_rad, scene_config.wrist.roll_limit_rad],
            [-scene_config.wrist.pitch_limit_rad, scene_config.wrist.pitch_limit_rad],
        ],
        dtype=np.float64,
    )
    wrist_limited_indices = np.asarray(partition.wrist_indices_xyz[:2], dtype=np.int64)
    if not np.allclose(usd_limits[wrist_limited_indices], expected_wrist_limits, atol=1e-4):
        raise RuntimeError("runtime D6 roll/pitch limits differ from the scene profile")

    joint_supervisor = JointCommandSupervisor(
        profile.layout,
        profile.rest_position,
        stale_after_s=scene_config.wrist.stale_after_s,
        velocity_scale=0.20,
    )
    pose_supervisor = PoseSupervisor(
        degraded_after_s=scene_config.wrist.stale_after_s,
        disarm_after_s=scene_config.wrist.disarm_after_s,
        max_angular_speed_rad_s=scene_config.wrist.max_angular_velocity_rad_s,
        max_pitch_rad=scene_config.wrist.pitch_limit_rad,
        max_roll_rad=scene_config.wrist.roll_limit_rad,
        min_quality=scene_config.wrist.min_quality,
    )
    receiver = (
        UdpHandCommandReceiver(ARGS.udp_port) if ARGS.command_source == "udp" else None
    )
    if receiver is None:
        clock_origin_ns = 0
        joint_decision = joint_supervisor.arm(0)
        clutch = PoseIntent(
            quat_wxyz=IDENTITY_QUATERNION_WXYZ,
            frame_id="hand2_right_neutral",
            host_time_ns=0,
            quality=1.0,
            calibration_id=SCRIPTED_CALIBRATION_ID,
        )
        pose_decision = pose_supervisor.arm_with_clutch(clutch, now_ns=0)
    else:
        clock_origin_ns = time.monotonic_ns()
        joint_decision = joint_supervisor.arm(clock_origin_ns)
        pose_decision = pose_supervisor.disarm()

    evaluator = BallLiftEvaluator(
        BallLiftCriteria(
            table_top_z_m=scene_config.table.top_z_m,
            ball_radius_m=scene_config.ball.radius_m,
            min_bottom_clearance_m=scene_config.qualification.lift_height_m,
            min_hold_s=scene_config.qualification.hold_time_s,
            min_opposing_fingers=scene_config.qualification.required_opposing_finger_groups,
            max_palm_relative_slip_m=scene_config.qualification.max_hand_relative_slip_m,
        )
    )
    finger_indices = np.asarray(partition.finger_indices_q20, dtype=np.int64)
    hand.set_joint_position_targets(profile.rest_position.reshape(1, -1), joint_indices=finger_indices)
    last_rotation_target_deg = set_rotation_mount_target_quaternion(
        stage, mount, IDENTITY_QUATERNION_WXYZ
    )

    last_udp_q20: np.ndarray | None = None
    last_udp_time_ns: int | None = None
    latest_phase = "waiting_for_udp" if receiver is not None else "settle_home_open"
    accepted_pose_epochs = 1 if receiver is None else 0
    rejected_pose_epochs = 0
    tracking_ticks = 0
    degraded_ticks = 0
    disarmed_ticks = 0
    max_flange_translation_error_m = 0.0
    max_ball_center_z_m = -np.inf
    min_ball_center_z_m = np.inf
    peak_contact_force_n = 0.0
    peak_hand_table_contact_force_n = 0.0
    hand_table_contact_frames = 0
    hand_table_contact_matrix_shape: tuple[int, ...] | None = None
    maximum_finger_feedback_abs_rad = 0.0
    maximum_wrist_feedback_abs_rad = 0.0
    feedback_within_limits = True
    ever_grasp_passed = False
    grasp_pass_time_s: float | None = None
    best_hold_duration_s = 0.0
    maximum_qualified_palm_relative_slip_m: float | None = None
    grasp_pass_palm_relative_slip_m: float | None = None
    latest_contacts: frozenset[str] = frozenset()
    ever_contact_groups: set[str] = set()
    latest_lift_result = None
    loop_started = time.monotonic()

    for frame in range(ARGS.frames):
        if receiver is not None:
            deadline = loop_started + frame / PHYSICS_HZ
            time.sleep(max(deadline - time.monotonic(), 0.0))

        if frame % COMMAND_DIVISOR == 0:
            if receiver is None:
                now_ns = int((frame + COMMAND_DIVISOR) / PHYSICS_HZ * 1e9)
                target = scripted_rotation_ball_target(frame / PHYSICS_HZ, scene_config)
                latest_phase = target.phase
                input_time_ns = now_ns
                q20_intent: np.ndarray | None = target.q20
                pose_intent = PoseIntent(
                    quat_wxyz=tuple(float(value) for value in target.root_delta_quat_wxyz),
                    frame_id="hand2_right_neutral",
                    host_time_ns=input_time_ns,
                    quality=1.0,
                    calibration_id=SCRIPTED_CALIBRATION_ID,
                )
            else:
                packet = receiver.receive_latest()
                now_ns = time.monotonic_ns()
                if packet is not None:
                    if packet.quality >= scene_config.wrist.min_quality:
                        last_udp_q20 = packet.q20.copy()
                        last_udp_time_ns = packet.host_time_ns
                    else:
                        # v2 quality applies to the indivisible finger/root sample.
                        last_udp_q20 = None
                        last_udp_time_ns = None
                    pose_intent = PoseIntent(
                        quat_wxyz=tuple(
                            float(value) for value in packet.root_delta_quat_wxyz
                        ),
                        frame_id=packet.pose_frame,
                        host_time_ns=packet.host_time_ns,
                        quality=packet.quality,
                        calibration_id=packet.calibration_id,
                    )
                else:
                    pose_intent = None
                q20_intent = last_udp_q20
                input_time_ns = last_udp_time_ns

            joint_decision = joint_supervisor.step(
                q20_intent,
                now_ns=now_ns,
                input_time_ns=input_time_ns,
            )
            new_pose_epoch = (
                pose_intent is not None
                and pose_intent.calibration_id != pose_decision.calibration_id
            )
            if new_pose_epoch and pose_intent is not None:
                is_identity = quaternion_geodesic_distance_rad(
                    pose_intent.quat_wxyz,
                    IDENTITY_QUATERNION_WXYZ,
                ) <= 1e-6
                if is_identity:
                    try:
                        pose_decision = pose_supervisor.arm_with_clutch(
                            pose_intent, now_ns=now_ns
                        )
                    except ValueError:
                        rejected_pose_epochs += 1
                        pose_decision = pose_supervisor.disarm()
                    else:
                        accepted_pose_epochs += 1
                else:
                    rejected_pose_epochs += 1
                    pose_decision = pose_supervisor.disarm()
            else:
                pose_decision = pose_supervisor.step(pose_intent, now_ns=now_ns)

            if pose_decision.state is SafetyState.TRACKING:
                tracking_ticks += 1
            elif pose_decision.state is SafetyState.DEGRADED:
                degraded_ticks += 1
            else:
                disarmed_ticks += 1
            hand.set_joint_position_targets(
                joint_decision.command.reshape(1, -1),
                joint_indices=finger_indices,
            )
            last_rotation_target_deg = set_rotation_mount_target_quaternion(
                stage,
                mount,
                pose_decision.command_quat_wxyz,
                previous_yaw_target_deg=last_rotation_target_deg[2],
            )

        world.step(
            render=ARGS.gui or (validation_output is not None and frame % 4 == 0)
        )

        feedback = np.asarray(hand.get_joint_positions(), dtype=np.float64)[0]
        feedback_q20 = profile.backend_full_to_firmware(feedback, backend_names)
        feedback_wrist = feedback[np.asarray(partition.wrist_indices_xyz, dtype=np.int64)]
        maximum_finger_feedback_abs_rad = max(
            maximum_finger_feedback_abs_rad,
            float(np.max(np.abs(feedback_q20))),
        )
        maximum_wrist_feedback_abs_rad = max(
            maximum_wrist_feedback_abs_rad,
            float(np.max(np.abs(feedback_wrist))),
        )
        feedback_within_limits = feedback_within_limits and bool(
            np.all(feedback_q20 >= np.asarray(profile.layout.lower) - FEEDBACK_LIMIT_TOLERANCE_RAD)
            and np.all(
                feedback_q20 <= np.asarray(profile.layout.upper) + FEEDBACK_LIMIT_TOLERANCE_RAD
            )
        )

        ball_position, _ = ball_scene.ball.get_world_pose()
        ball_position = np.asarray(ball_position, dtype=np.float64)
        palm_position, palm_quaternion = get_world_pose(mount.paths.base_link_path)
        palm_position = np.asarray(palm_position, dtype=np.float64)
        palm_quaternion = np.asarray(palm_quaternion, dtype=np.float64)
        flange_error = float(
            np.linalg.norm(palm_position - np.asarray(scene_config.flange.position_m))
        )
        max_flange_translation_error_m = max(max_flange_translation_error_m, flange_error)
        max_ball_center_z_m = max(max_ball_center_z_m, float(ball_position[2]))
        min_ball_center_z_m = min(min_ball_center_z_m, float(ball_position[2]))

        force_matrix = np.asarray(
            ball_scene.contact_view.get_contact_force_matrix(dt=1.0 / PHYSICS_HZ),
            dtype=np.float64,
        )
        if force_matrix.size:
            peak_contact_force_n = max(
                peak_contact_force_n,
                float(np.max(np.linalg.norm(force_matrix, axis=-1))),
            )
        latest_contacts = contact_groups_from_force_matrix(
            force_matrix,
            ball_scene.filters,
            threshold_n=ARGS.contact_threshold_n,
        )
        hand_table_forces = np.asarray(
            ball_scene.hand_table_contact_view.get_contact_force_matrix(
                dt=1.0 / PHYSICS_HZ
            ),
            dtype=np.float64,
        )
        if (
            hand_table_forces.size == 0
            or hand_table_forces.ndim not in (2, 3)
            or hand_table_forces.shape[-1] != 3
            or (hand_table_forces.ndim == 3 and hand_table_forces.shape[-2] != 1)
            or not np.isfinite(hand_table_forces).all()
        ):
            raise RuntimeError(
                "hand/table contact view returned an empty, non-finite, or invalid force matrix: "
                f"shape={hand_table_forces.shape}"
            )
        current_hand_table_shape = tuple(int(value) for value in hand_table_forces.shape)
        if hand_table_contact_matrix_shape is None:
            hand_table_contact_matrix_shape = current_hand_table_shape
        elif current_hand_table_shape != hand_table_contact_matrix_shape:
            raise RuntimeError("hand/table contact force matrix shape changed during the run")
        hand_table_peak = float(
            np.max(np.linalg.norm(hand_table_forces.reshape(-1, 3), axis=1))
        )
        peak_hand_table_contact_force_n = max(
            peak_hand_table_contact_force_n,
            hand_table_peak,
        )
        if hand_table_peak >= ARGS.contact_threshold_n:
            hand_table_contact_frames += 1
        ever_contact_groups.update(latest_contacts)
        relative_position = ball_in_palm_frame(
            ball_position,
            palm_position,
            palm_quaternion,
        )
        latest_lift_result = evaluator.update(
            BallLiftSample(
                time_s=(frame + 1) / PHYSICS_HZ,
                ball_center_xyz_m=tuple(float(value) for value in ball_position),
                contact_groups=latest_contacts,
                ball_in_palm_xyz_m=relative_position,
            )
        )
        best_hold_duration_s = max(
            best_hold_duration_s,
            latest_lift_result.hold_duration_s,
        )
        if (
            latest_lift_result.qualified
            and latest_lift_result.palm_relative_slip_m is not None
        ):
            if maximum_qualified_palm_relative_slip_m is None:
                maximum_qualified_palm_relative_slip_m = (
                    latest_lift_result.palm_relative_slip_m
                )
            else:
                maximum_qualified_palm_relative_slip_m = max(
                    maximum_qualified_palm_relative_slip_m,
                    latest_lift_result.palm_relative_slip_m,
                )
        if latest_lift_result.passed and not ever_grasp_passed:
            ever_grasp_passed = True
            grasp_pass_time_s = (frame + 1) / PHYSICS_HZ
            grasp_pass_palm_relative_slip_m = (
                latest_lift_result.palm_relative_slip_m
            )

    if receiver is not None:
        receiver.close()

    screenshot_path: str | None = None
    screenshot_error: str | None = None
    if validation_output is not None and not ARGS.skip_screenshot:
        try:
            for _ in range(8):
                world.step(render=True)
            screenshot = validation_output / "hand2_rotation_ball.png"
            viewport = get_active_viewport()
            if viewport is None:
                raise RuntimeError("active Isaac viewport is unavailable")
            capture = capture_viewport_to_file(viewport, file_path=str(screenshot))
            captured = simulation_app.run_coroutine(
                capture.wait_for_result(completion_frames=30)
            )
            omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
            if not captured or not screenshot.is_file():
                raise RuntimeError("Isaac viewport capture did not complete")
            screenshot_path = str(screenshot)
        except Exception as exc:  # screenshot is evidence, not a physics control path
            screenshot_error = f"{type(exc).__name__}: {exc}"

    final_ball_position, _ = ball_scene.ball.get_world_pose()
    final_ball_position = np.asarray(final_ball_position, dtype=np.float64)
    final_palm_position, final_palm_quaternion = get_world_pose(mount.paths.base_link_path)
    final_palm_position = np.asarray(final_palm_position, dtype=np.float64)
    final_palm_quaternion = np.asarray(final_palm_quaternion, dtype=np.float64)
    final_feedback = np.asarray(hand.get_joint_positions(), dtype=np.float64)[0]
    structural_passed = bool(
        len(backend_names) == 23
        and feedback_within_limits
        and np.isfinite(final_ball_position).all()
        and max_flange_translation_error_m <= 0.001
    )
    movement_expected = ARGS.command_source == "scripted" and ARGS.frames >= 600
    movement_observed = bool(
        maximum_wrist_feedback_abs_rad > 0.30
        and maximum_finger_feedback_abs_rad > 0.20
    )
    hand_table_contact_free = hand_table_contact_frames == 0
    grasp_passed = ever_grasp_passed and hand_table_contact_free
    report = {
        "isaac_sim": "5.1.0",
        "asset": str(ARGS.asset.resolve()),
        "asset_sha256": asset_sha256,
        "profile": str(ARGS.profile.resolve()),
        "scene_profile": str(ARGS.scene_profile.resolve()),
        "profile_provenance": profile.provenance,
        "scene_provenance": dict(scene_config.provenance),
        "articulation_root": mount.paths.articulation_root_path,
        "disabled_upstream_root": mount.paths.disabled_root_joint_path,
        "rotation_joint": mount.paths.rotation_joint_path,
        "rotation_joint_frame_quat_wxyz": list(mount.joint_frame_quat_wxyz),
        "flange_home_position_m": list(scene_config.flange.position_m),
        "flange_home_quat_wxyz": list(scene_config.flange.neutral_quat_wxyz),
        "script_pregrasp_delta_pitch_rad": (
            scene_config.script.pregrasp_delta_pitch_rad
        ),
        "script_lifted_delta_pitch_rad": scene_config.script.lifted_delta_pitch_rad,
        "final_rotation_drive_target_rpy_deg": list(last_rotation_target_deg),
        "dof_count": len(backend_names),
        "backend_dof_names": backend_names,
        "backend_dof_paths": backend_paths,
        "wrist_indices_xyz": list(partition.wrist_indices_xyz),
        "finger_indices_q20": list(partition.finger_indices_q20),
        "limits_match": True,
        "frames": ARGS.frames,
        "physics_hz": PHYSICS_HZ,
        "command_hz": COMMAND_HZ,
        "command_source": ARGS.command_source,
        "udp_port": ARGS.udp_port if receiver is not None else None,
        "udp_packets_accepted": receiver.accepted if receiver is not None else None,
        "udp_packets_rejected": receiver.rejected if receiver is not None else None,
        "pose_epochs_accepted": accepted_pose_epochs,
        "pose_epochs_rejected": rejected_pose_epochs,
        "pose_tracking_ticks": tracking_ticks,
        "pose_degraded_ticks": degraded_ticks,
        "pose_disarmed_ticks": disarmed_ticks,
        "last_joint_state": joint_decision.state.value,
        "last_joint_reason": joint_decision.reason,
        "last_pose_state": pose_decision.state.value,
        "last_pose_reason": pose_decision.reason,
        "last_phase": latest_phase,
        "feedback_within_limits": feedback_within_limits,
        "feedback_limit_tolerance_rad": FEEDBACK_LIMIT_TOLERANCE_RAD,
        "maximum_wrist_feedback_abs_rad": maximum_wrist_feedback_abs_rad,
        "maximum_finger_feedback_abs_rad": maximum_finger_feedback_abs_rad,
        "movement_expected": movement_expected,
        "movement_observed": movement_observed,
        "flange_translation_limit_m": 0.001,
        "max_flange_translation_error_m": max_flange_translation_error_m,
        "ball_initial_center_m": list(scene_config.ball.center_m),
        "ball_radius_m": scene_config.ball.radius_m,
        "ball_final_center_m": np.asarray(final_ball_position).tolist(),
        "ball_final_in_palm_xyz_m": list(
            ball_in_palm_frame(
                final_ball_position,
                final_palm_position,
                final_palm_quaternion,
            )
        ),
        "ball_min_center_z_m": min_ball_center_z_m,
        "ball_max_center_z_m": max_ball_center_z_m,
        "peak_contact_force_n": peak_contact_force_n,
        "peak_hand_table_contact_force_n": peak_hand_table_contact_force_n,
        "hand_table_contact_frames": hand_table_contact_frames,
        "hand_table_contact_free": hand_table_contact_free,
        "hand_table_contact_matrix_shape": list(hand_table_contact_matrix_shape or ()),
        "ever_contact_groups": sorted(ever_contact_groups),
        "final_palm_world_xyz_m": final_palm_position.tolist(),
        "final_palm_world_quat_wxyz": final_palm_quaternion.tolist(),
        "final_wrist_feedback_rad": final_feedback[
            np.asarray(partition.wrist_indices_xyz, dtype=np.int64)
        ].tolist(),
        "final_finger_feedback_q20_rad": profile.backend_full_to_firmware(
            final_feedback,
            backend_names,
        ).tolist(),
        "latest_contact_groups": sorted(latest_contacts),
        "grasp_criteria": {
            "min_bottom_clearance_m": scene_config.qualification.lift_height_m,
            "min_hold_s": scene_config.qualification.hold_time_s,
            "min_opposing_fingers": scene_config.qualification.required_opposing_finger_groups,
            "max_palm_relative_slip_m": scene_config.qualification.max_hand_relative_slip_m,
        },
        "ball_lift_contact_passed": ever_grasp_passed,
        "grasp_passed": grasp_passed,
        "grasp_pass_time_s": grasp_pass_time_s,
        "best_hold_duration_s": best_hold_duration_s,
        "maximum_qualified_palm_relative_slip_m": (
            maximum_qualified_palm_relative_slip_m
        ),
        "grasp_pass_palm_relative_slip_m": grasp_pass_palm_relative_slip_m,
        "last_grasp_reasons": (
            None if latest_lift_result is None else list(latest_lift_result.reasons)
        ),
        "screenshot": screenshot_path,
        "screenshot_error": screenshot_error,
        "structural_passed": structural_passed,
        "grasp_success_required": ARGS.require_grasp_success,
    }
    if validation_output is not None:
        (validation_output / "validation.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(report, indent=2, sort_keys=True))

    if not structural_passed:
        return 1
    if movement_expected and not movement_observed:
        return 1
    if ARGS.require_grasp_success and not grasp_passed:
        return 1
    return 0


exit_code = 1
try:
    exit_code = main()
except Exception:
    error = traceback.format_exc()
    if ARGS.validation_output_dir is not None:
        ARGS.validation_output_dir.mkdir(parents=True, exist_ok=True)
        (ARGS.validation_output_dir / "error.txt").write_text(error, encoding="utf-8")
    print(error, file=sys.stderr, flush=True)
finally:
    simulation_app.close()
raise SystemExit(exit_code)
