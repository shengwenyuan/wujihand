#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Run one isolated SDK 8.3 -> supervisor -> Description 8.3 Isaac Hand2 chain."""

from __future__ import annotations

import argparse
from importlib import metadata
import json
from pathlib import Path
import sys
import time
import traceback


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.domain import HandSide
from wujihand.runtime import (
    detect_wuji_studio_processes,
    inspect_wuji_sdk_runtime,
    preflight_wuji_hand2_matched_chain,
    resolve_isaac_workcell_plan,
)
from wujihand.runtime.session_compat import (
    fixed_hand_workcell_runtime,
    resolve_isaac_hand_runtime,
)


DEFAULT_QUALIFICATION = (
    ROOT / "configs/qualifications/wuji_hand2_matched_chain_v2026_8_3_v1.yaml"
)
PHYSICS_HZ = 120
COMMAND_HZ = 60
COMMAND_DIVISOR = PHYSICS_HZ // COMMAND_HZ
LIVE_PHASES = (
    ("open_start", 4, "手掌完全张开并保持"),
    ("individual_fingers", 10, "依次屈伸拇指、食指、中指、无名指、小指，各一次"),
    ("fist_open", 8, "缓慢握拳再张开，共两次"),
    ("thumb_index", 5, "拇指与食指轻触、分开两次，最后保持轻触"),
    ("thumb_middle", 5, "拇指与中指轻触、分开两次，最后保持轻触"),
    ("thumb_ring", 5, "拇指与无名指轻触、分开两次，最后保持轻触"),
    ("thumb_pinky", 5, "拇指与小指轻触、分开两次，最后保持轻触"),
    ("open_end", 4, "恢复完全张开并保持"),
)
LIVE_FRAMES = sum(duration_s for _, duration_s, _ in LIVE_PHASES) * PHYSICS_HZ
STUB_FRAMES = 5 * 5 * PHYSICS_HZ
STUB_TRACKING_WARNING_RAD = 0.20
STUB_TRACKING_FAILURE_RAD = 0.50
TIP_INDICES = {"thumb": 4, "index": 8, "middle": 12, "ring": 16, "pinky": 20}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", required=True, choices=("left", "right"))
    parser.add_argument("--input", choices=("stub", "glove"), default="stub")
    parser.add_argument("--local-binding", type=Path, required=True)
    parser.add_argument("--qualification", type=Path, default=DEFAULT_QUALIFICATION)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--frames",
        type=int,
        default=None,
        help="Defaults to 3000 for stub and the complete 46-second script for Glove.",
    )
    parser.add_argument("--start-delay-s", type=int, default=5)
    parser.add_argument(
        "--self-collision",
        choices=("off", "on"),
        default="off",
        help="Explicit runtime Hand2 self-collision policy; live baseline must use off.",
    )
    parser.add_argument(
        "--table-collision",
        choices=("off", "on"),
        default="off",
        help="Explicit table-collision policy; isolated Glove qualification uses off.",
    )
    parser.add_argument(
        "--control-mode",
        choices=("dynamic", "kinematic-diagnostic"),
        default="dynamic",
        help="Kinematic mode is a stub-only q20/USD diagnostic and is never a live mode.",
    )
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    return parser.parse_args()


ARGS = parse_args()
if ARGS.frames is None:
    ARGS.frames = LIVE_FRAMES if ARGS.input == "glove" else STUB_FRAMES
if ARGS.frames < 10:
    raise SystemExit("--frames must be at least 10")
if ARGS.start_delay_s < 0:
    raise SystemExit("--start-delay-s must be non-negative")
if ARGS.input == "glove" and ARGS.control_mode != "dynamic":
    raise SystemExit("Glove input requires --control-mode dynamic")
if ARGS.output_dir.exists():
    raise SystemExit(f"output directory already exists: {ARGS.output_dir}")

import wuji_sdk


SIDE = HandSide(ARGS.side)
SDK_FACTS = inspect_wuji_sdk_runtime(
    wuji_sdk,
    distribution_version=metadata.version("wuji-sdk"),
)
PREFLIGHT = preflight_wuji_hand2_matched_chain(
    ROOT,
    qualification_path=ARGS.qualification,
    local_binding_path=ARGS.local_binding,
    side=SIDE,
    input_mode=ARGS.input,
    sdk_runtime=SDK_FACTS,
    user_manager=wuji_sdk.SdkManager.instance(),
    studio_processes=detect_wuji_studio_processes(),
    verify_artifacts=True,
)
SESSION_RUNTIME = resolve_isaac_hand_runtime(
    ROOT,
    session_path=ROOT / PREFLIGHT.session_path,
    runtime_roles={"qualification"},
)
if SESSION_RUNTIME.resolved.session_hash != PREFLIGHT.session_hash:
    raise SystemExit("preflight and Isaac resolver produced different Session hashes")
WORKCELL_RUNTIME = fixed_hand_workcell_runtime(SESSION_RUNTIME.resolved)
WORKCELL_PLAN = resolve_isaac_workcell_plan(ROOT, SESSION_RUNTIME.resolved.workcell)

ARGS.output_dir.mkdir(parents=True, exist_ok=False)
(ARGS.output_dir / "preflight.json").write_text(
    json.dumps(PREFLIGHT.to_mapping(), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
if ARGS.preflight_only:
    print(json.dumps(PREFLIGHT.to_mapping(), ensure_ascii=False, indent=2, sort_keys=True))
    raise SystemExit(0)

from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {"headless": not ARGS.gui, "width": 960, "height": 720, "anti_aliasing": 0}
)

import numpy as np
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.version import get_version as get_isaac_sim_version
from wujihand.adapters.input import (
    NoHandSkeletonFrameAvailable,
    WujiGloveHandSkeletonAdapter,
)
from wujihand.adapters.retargeting import WujiHand2RetargetAdapter
from wujihand.adapters.simulation import load_hand2_model_profile
from wujihand.adapters.storage import CanonicalHandObservationReplayAdapter
from wujihand.application.qualification import (
    WUJI_GLOVE_STUB_POSES,
    build_wuji_glove_stub_observations,
)
from wujihand.application.supervision import JointCommandSupervisor
from wujihand.application.teleoperation import GloveHand2SimulationController
from wujihand.domain import hand2_layout, hand2_rest
from wujihand.integrity import sha256_file
from wujihand.runtime.isaac_workcell import materialize_isaac_workcell


def _articulation_root(stage: object) -> str:
    roots = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith("/World/Hand2")
        and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(roots) != 1:
        raise RuntimeError(f"expected one Hand2 articulation root, found {roots}")
    return roots[0]


def _input_adapter() -> object:
    if ARGS.input == "stub":
        records = build_wuji_glove_stub_observations(
            SIDE,
            calibration_id=PREFLIGHT.calibration_id,
            frames_per_pose=_stub_frames_per_pose(),
        )
        return CanonicalHandObservationReplayAdapter(records)
    return WujiGloveHandSkeletonAdapter(
        SIDE,
        source_id=f"wuji_glove.{SIDE.value}.matched_chain_qualification",
        calibration_id=PREFLIGHT.calibration_id,
        transform_id="wuji_glove.hand_skeleton.v1",
        serial_number=PREFLIGHT.serial_number,
        device_name=f"matched_chain_{SIDE.value}_8_3",
    )


def _stub_frames_per_pose() -> int:
    command_ticks = (ARGS.frames + COMMAND_DIVISOR - 1) // COMMAND_DIVISOR
    return max(
        (command_ticks + len(WUJI_GLOVE_STUB_POSES) - 1)
        // len(WUJI_GLOVE_STUB_POSES),
        1,
    )


def _stub_phase(frame: int) -> str:
    command_index = frame // COMMAND_DIVISOR
    pose_index = min(
        command_index // _stub_frames_per_pose(),
        len(WUJI_GLOVE_STUB_POSES) - 1,
    )
    return WUJI_GLOVE_STUB_POSES[pose_index]


def _live_phase(frame: int) -> tuple[str, str]:
    elapsed_frames = 0
    for phase, duration_s, prompt in LIVE_PHASES:
        elapsed_frames += duration_s * PHYSICS_HZ
        if frame < elapsed_frames:
            return phase, prompt
    return LIVE_PHASES[-1][0], LIVE_PHASES[-1][2]


def _site_positions(stage: object, paths: list[str]) -> dict[str, list[float]]:
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    result: dict[str, list[float]] = {}
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        translation = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
        finger = prim.GetName().removeprefix(f"{SIDE.value[0]}_")
        finger = (
            finger.removesuffix("_finger_tip")
            if finger.endswith("_finger_tip")
            else finger.removesuffix("_tip")
        )
        result[finger] = [float(value) for value in translation]
    return result


def _thumb_distances(positions: dict[str, list[float]]) -> dict[str, float]:
    thumb = np.asarray(positions["thumb"], dtype=np.float64)
    return {
        finger: float(np.linalg.norm(np.asarray(positions[finger]) - thumb))
        for finger in ("index", "middle", "ring", "pinky")
    }


def _drive_inventory(stage: object) -> list[dict[str, object]]:
    inventory: list[dict[str, object]] = []
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith("/World/Hand2") or not prim.IsA(
            UsdPhysics.RevoluteJoint
        ):
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            continue
        inventory.append(
            {
                "joint": prim.GetName(),
                "path": str(prim.GetPath()),
                "stiffness": drive.GetStiffnessAttr().Get(),
                "damping": drive.GetDampingAttr().Get(),
                "max_force": drive.GetMaxForceAttr().Get(),
                "type": str(drive.GetTypeAttr().Get()),
            }
        )
    return sorted(inventory, key=lambda item: str(item["joint"]))


def main() -> int:
    profile = load_hand2_model_profile(SESSION_RUNTIME.profile_path)
    if profile.side != SIDE.value or profile.layout != hand2_layout(SIDE.value):
        raise RuntimeError("resolved Hand2 profile side/layout differs from preflight")
    if sha256_file(SESSION_RUNTIME.asset_path) != profile.provenance.get("usd_sha256"):
        raise RuntimeError("resolved Hand2 USD differs from the pinned profile")

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / PHYSICS_HZ,
        rendering_dt=1.0 / 30.0,
        backend="numpy",
    )
    workcell = materialize_isaac_workcell(world, WORKCELL_PLAN)
    add_reference_to_stage(str(SESSION_RUNTIME.asset_path.resolve()), "/World/Hand2")
    stage = world.scene.stage
    stage_prims = list(Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()))
    collision_paths = sorted(
        str(prim.GetPath())
        for prim in stage_prims
        if str(prim.GetPath()).startswith("/World/Hand2")
        and prim.HasAPI(UsdPhysics.CollisionAPI)
    )
    fingertip_names = {
        f"{SIDE.value[0]}_thumb_tip",
        f"{SIDE.value[0]}_index_finger_tip",
        f"{SIDE.value[0]}_middle_finger_tip",
        f"{SIDE.value[0]}_ring_finger_tip",
        f"{SIDE.value[0]}_pinky_tip",
    }
    fingertip_paths = sorted(
        str(prim.GetPath()) for prim in stage_prims if prim.GetName() in fingertip_names
    )
    root = _articulation_root(stage)
    self_collision_api = PhysxSchema.PhysxArticulationAPI(stage.GetPrimAtPath(root))
    self_collision_api.CreateEnabledSelfCollisionsAttr().Set(ARGS.self_collision == "on")
    self_collisions_enabled = bool(
        self_collision_api.GetEnabledSelfCollisionsAttr().Get()
    )
    if self_collisions_enabled is not (ARGS.self_collision == "on"):
        raise RuntimeError("Hand2 self-collision readback differs from the explicit CLI policy")
    if len(workcell.primitive_prim_paths) != 1:
        raise RuntimeError("matched-chain workcell must materialize exactly one table")
    table_path = workcell.primitive_prim_paths[0]
    table_collision_api = UsdPhysics.CollisionAPI(stage.GetPrimAtPath(table_path))
    if not table_collision_api:
        raise RuntimeError(f"workcell table has no CollisionAPI: {table_path}")
    table_collision_api.CreateCollisionEnabledAttr().Set(ARGS.table_collision == "on")
    table_collision_enabled = bool(table_collision_api.GetCollisionEnabledAttr().Get())
    if table_collision_enabled is not (ARGS.table_collision == "on"):
        raise RuntimeError("table-collision readback differs from the explicit CLI policy")
    hand = world.scene.add(Articulation(root, name=f"hand2_{SIDE.value}_matched_chain"))
    world.reset()
    hand.set_world_poses(
        positions=np.asarray([WORKCELL_RUNTIME.hand_mount.position_m], dtype=np.float32),
        orientations=np.asarray([WORKCELL_RUNTIME.hand_mount.quat_wxyz], dtype=np.float32),
    )
    set_camera_view(
        eye=np.asarray(WORKCELL_RUNTIME.camera_eye_m),
        target=np.asarray(WORKCELL_RUNTIME.camera_target_m),
        camera_prim_path="/OmniverseKit_Persp",
    )
    world.step(render=ARGS.gui)

    backend_names = tuple(hand.dof_names)
    if len(backend_names) != 20:
        raise RuntimeError(f"expected 20 Hand2 DOFs, found {len(backend_names)}")
    reorder = profile.layout.indices_for(backend_names)
    usd_limits = np.asarray(hand.get_dof_limits(), dtype=np.float64)[0]
    expected_limits = np.column_stack((profile.layout.lower, profile.layout.upper))[
        np.asarray(reorder)
    ]
    if not np.allclose(usd_limits, expected_limits, atol=1e-4):
        raise RuntimeError("Isaac Hand2 limits differ from the pinned q20 profile")
    drive_inventory = _drive_inventory(stage)

    controller = GloveHand2SimulationController(
        SIDE,
        _input_adapter(),
        WujiHand2RetargetAdapter(SIDE),
        JointCommandSupervisor(
            profile.layout,
            hand2_rest(SIDE.value),
            stale_after_s=0.25,
            velocity_scale=1.0,
        ),
    )
    if ARGS.input == "glove":
        for remaining in range(ARGS.start_delay_s, 0, -1):
            print(f"{remaining} 秒后连接 {SIDE.value} Glove，请保持手掌自然张开。", flush=True)
            time.sleep(1.0)

    synthetic_epoch_ns = 1_000_000_000
    start_ns = time.monotonic_ns() if ARGS.input == "glove" else synthetic_epoch_ns
    controller.start(now_ns=start_ns)
    accepted = 0
    empty_polls = 0
    rejected = 0
    degraded = 0
    clamped = 0
    rate_limited = 0
    tracking_ticks = 0
    feedback_peak_abs = 0.0
    feedback_within_limits = True
    model_ids: set[str] = set()
    config_ids: set[str] = set()
    trajectory: list[dict[str, object]] = []
    last_command = profile.rest_position.copy()
    live_deadline = time.monotonic()
    active_phase: str | None = None
    try:
        for frame in range(ARGS.frames):
            if ARGS.input == "glove":
                live_deadline += 1.0 / PHYSICS_HZ
                time.sleep(max(live_deadline - time.monotonic(), 0.0))
            if frame % COMMAND_DIVISOR == 0:
                phase, prompt = (
                    _live_phase(frame) if ARGS.input == "glove" else (_stub_phase(frame), "")
                )
                if phase != active_phase:
                    active_phase = phase
                    if ARGS.input == "glove":
                        print(f"\n[{phase}] {prompt}", flush=True)
                now_ns = (
                    time.monotonic_ns()
                    if ARGS.input == "glove"
                    else synthetic_epoch_ns + round((frame + 1) / PHYSICS_HZ * 1e9)
                )
                try:
                    step = controller.poll(now_ns=now_ns)
                except NoHandSkeletonFrameAvailable:
                    empty_polls += 1
                    step = controller.advance_without_observation(now_ns=now_ns)
                if step.intent is not None:
                    accepted += 1
                    model_ids.add(step.intent.retarget_model_id)
                    config_ids.add(step.intent.retarget_config_id)
                    degraded += int(step.intent.retarget_status.value == "degraded")
                    if step.intent.source_observation.calibration_id != PREFLIGHT.calibration_id:
                        raise RuntimeError("input calibration provenance changed after preflight")
                    observation = step.intent.source_observation
                    landmark_names = [
                        landmark.name for landmark in observation.landmarks
                    ]
                    landmark_positions = [
                        landmark.position_m for landmark in observation.landmarks
                    ]
                    landmark_confidences = [
                        landmark.confidence for landmark in observation.landmarks
                    ]
                    human_distances = {
                        finger: float(
                            np.linalg.norm(
                                np.asarray(landmark_positions[index], dtype=np.float64)
                                - np.asarray(
                                    landmark_positions[TIP_INDICES["thumb"]],
                                    dtype=np.float64,
                                )
                            )
                        )
                        for finger, index in TIP_INDICES.items()
                        if finger != "thumb"
                    }
                    minimum_confidence = min(
                        landmark_confidences
                    )
                    source_sequence = observation.sequence
                    source_id = observation.source_id
                    source_time_ns = observation.source_time_ns
                    receive_time_ns = observation.receive_time_ns
                    device_time_ns = observation.device_time_ns
                    device_clock_domain = observation.device_clock_domain
                    retarget_q20 = list(step.intent.q20_rad)
                else:
                    landmark_names = None
                    landmark_positions = None
                    landmark_confidences = None
                    human_distances = None
                    minimum_confidence = None
                    source_sequence = None
                    source_id = None
                    source_time_ns = None
                    receive_time_ns = None
                    device_time_ns = None
                    device_clock_domain = None
                    retarget_q20 = None
                if step.rejection_reason is not None:
                    rejected += 1
                clamped += int(step.decision.position_clamped)
                rate_limited += int(step.decision.rate_limited)
                tracking_ticks += int(step.decision.state.value == "tracking")
                last_command = step.decision.command.copy()
                backend_target = profile.firmware_to_backend(last_command, backend_names)
                hand.set_joint_position_targets(backend_target.reshape(1, -1))
                if ARGS.control_mode == "kinematic-diagnostic":
                    hand.set_joint_positions(backend_target.reshape(1, -1))
                    hand.set_joint_velocities(
                        np.zeros((1, len(backend_names)), dtype=np.float64)
                    )
                feedback_backend = np.asarray(hand.get_joint_positions(), dtype=np.float64)[0]
                feedback = profile.backend_to_firmware(feedback_backend, backend_names)
                feedback_peak_abs = max(feedback_peak_abs, float(np.abs(feedback).max()))
                feedback_within_limits = feedback_within_limits and bool(
                    np.all(feedback >= np.asarray(profile.layout.lower) - 0.01)
                    and np.all(feedback <= np.asarray(profile.layout.upper) + 0.01)
                )
                fingertip_positions = _site_positions(stage, fingertip_paths)
                trajectory.append(
                    {
                        "frame": frame,
                        "elapsed_s": frame / PHYSICS_HZ,
                        "phase": phase,
                        "intent": step.intent is not None,
                        "source_sequence": source_sequence,
                        "source_id": source_id,
                        "source_time_ns": source_time_ns,
                        "receive_time_ns": receive_time_ns,
                        "device_time_ns": device_time_ns,
                        "device_clock_domain": device_clock_domain,
                        "landmark_names": landmark_names,
                        "landmark_positions_m": landmark_positions,
                        "landmark_confidences": landmark_confidences,
                        "minimum_landmark_confidence": minimum_confidence,
                        "human_thumb_to_tip_distance_m": human_distances,
                        "retarget_status": (
                            None if step.intent is None else step.intent.retarget_status.value
                        ),
                        "supervision_state": step.decision.state.value,
                        "supervision_reason": step.decision.reason,
                        "rejection_reason": step.rejection_reason,
                        "retarget_q20_rad": retarget_q20,
                        "command_q20_rad": last_command.tolist(),
                        "feedback_q20_rad": feedback.tolist(),
                        "isaac_fingertip_site_position_m": fingertip_positions,
                        "isaac_thumb_to_tip_distance_m": _thumb_distances(
                            fingertip_positions
                        ),
                    }
                )
            world.step(render=ARGS.gui)
    finally:
        controller.close()

    expected_model_id = f"wuji_sdk.WujiHand2.{PREFLIGHT.sdk_version}"
    failure_reasons: list[str] = []
    if accepted == 0:
        failure_reasons.append("no_retarget_intent")
    if model_ids != {expected_model_id}:
        failure_reasons.append("retarget_model_identity_mismatch")
    if len(config_ids) != 1 or not next(iter(config_ids), "").startswith(
        f"wuji_sdk.builtin.WujiHand2.{SIDE.value}.{PREFLIGHT.sdk_version}."
    ):
        failure_reasons.append("retarget_config_identity_mismatch")
    if rejected:
        failure_reasons.append("retarget_rejection")
    if clamped:
        failure_reasons.append("position_clamp")
    if not feedback_within_limits:
        failure_reasons.append("feedback_outside_limits")
    if feedback_peak_abs <= 0.05:
        failure_reasons.append("isaac_movement_not_observed")
    if not collision_paths:
        failure_reasons.append("collision_inventory_empty")
    if len(fingertip_paths) != 5:
        failure_reasons.append("fingertip_site_count_mismatch")
    if {str(item["joint"]) for item in drive_inventory} != set(backend_names):
        failure_reasons.append("drive_inventory_mismatch")

    diagnostic_warnings: list[str] = []
    phase_summary: dict[str, dict[str, object]] = {}
    phase_definitions = (
        LIVE_PHASES
        if ARGS.input == "glove"
        else tuple((pose, 0, "") for pose in WUJI_GLOVE_STUB_POSES)
    )
    for phase, _, _ in phase_definitions:
        rows = [row for row in trajectory if row["phase"] == phase and row["intent"]]
        settled = rows[-min(COMMAND_HZ, len(rows)) :]
        phase_summary[phase] = {
            "accepted_intents": len(rows),
            "minimum_landmark_confidence": (
                None
                if not rows
                else min(float(row["minimum_landmark_confidence"]) for row in rows)
            ),
            "human_thumb_to_tip_min_m": {
                finger: min(
                    float(row["human_thumb_to_tip_distance_m"][finger]) for row in rows
                )
                for finger in ("index", "middle", "ring", "pinky")
            }
            if rows
            else None,
            "isaac_thumb_to_tip_min_m": {
                finger: min(
                    float(row["isaac_thumb_to_tip_distance_m"][finger]) for row in rows
                )
                for finger in ("index", "middle", "ring", "pinky")
            }
            if rows
            else None,
            "settled_feedback_max_abs_error_rad": (
                None
                if not settled
                else max(
                    max(
                        abs(command - feedback)
                        for command, feedback in zip(
                            row["command_q20_rad"],
                            row["feedback_q20_rad"],
                            strict=True,
                        )
                    )
                    for row in settled
                )
            ),
            "settled_supervision_max_abs_delta_rad": (
                None
                if not settled
                else max(
                    max(
                        abs(retarget - command)
                        for retarget, command in zip(
                            row["retarget_q20_rad"],
                            row["command_q20_rad"],
                            strict=True,
                        )
                    )
                    for row in settled
                )
            ),
        }
    if ARGS.input == "stub" and ARGS.control_mode == "dynamic":
        for phase, values in phase_summary.items():
            error = values["settled_feedback_max_abs_error_rad"]
            if error is None or float(error) > STUB_TRACKING_FAILURE_RAD:
                failure_reasons.append(
                    f"{phase}_settled_tracking_error_above_0_50_rad"
                )
            elif float(error) > STUB_TRACKING_WARNING_RAD:
                diagnostic_warnings.append(
                    f"{phase}_settled_tracking_error_above_0_20_rad"
                )

    report = {
        "schema": "wujihand.isaac_wuji_hand2_matched_chain_qualification.v1",
        "passed": not failure_reasons,
        "failure_reasons": failure_reasons,
        "diagnostic_warnings": diagnostic_warnings,
        "qualification_only": True,
        "dataset_eligible": False,
        "simulation_only": True,
        "hardware_reusable": False,
        "hand2_hardware_connected": False,
        "nero_connected": False,
        "glove_device_access_attempted": ARGS.input == "glove",
        "input_mode": ARGS.input,
        "control_mode": ARGS.control_mode,
        "self_collisions_enabled": self_collisions_enabled,
        "table_collision_enabled": table_collision_enabled,
        "side": SIDE.value,
        "isaac_sim": get_isaac_sim_version()[0],
        "preflight": PREFLIGHT.to_mapping(),
        "session_hash": SESSION_RUNTIME.resolved.session_hash,
        "workcell": workcell.to_mapping(),
        "asset": str(SESSION_RUNTIME.asset_path),
        "profile": str(SESSION_RUNTIME.profile_path),
        "articulation_root": root,
        "backend_dof_names": list(backend_names),
        "collision_prim_count": len(collision_paths),
        "collision_prim_paths": collision_paths,
        "drive_inventory": drive_inventory,
        "fingertip_site_paths": fingertip_paths,
        "frames": ARGS.frames,
        "physics_hz": PHYSICS_HZ,
        "command_hz": COMMAND_HZ,
        "accepted_intents": accepted,
        "empty_polls": empty_polls,
        "rejected_frames": rejected,
        "degraded_intents": degraded,
        "position_clamped_ticks": clamped,
        "rate_limited_ticks": rate_limited,
        "tracking_ticks": tracking_ticks,
        "retarget_model_ids": sorted(model_ids),
        "retarget_config_ids": sorted(config_ids),
        "phase_summary": phase_summary,
        "feedback_peak_abs_rad": feedback_peak_abs,
        "feedback_within_limits": feedback_within_limits,
        "final_command_q20_rad": last_command.tolist(),
        "beta_warning": (
            "Wuji Hand2 remains Beta1; this result does not validate physical soft-fingertip contact."
        ),
    }
    (ARGS.output_dir / "trajectory.jsonl").write_text(
        "".join(
            json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n"
            for record in trajectory
        ),
        encoding="utf-8",
    )
    (ARGS.output_dir / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if not failure_reasons else 1


exit_code = 1
try:
    exit_code = main()
except Exception:
    error = traceback.format_exc()
    (ARGS.output_dir / "error.txt").write_text(error, encoding="utf-8")
    print(error, file=sys.stderr, flush=True)
finally:
    simulation_app.close()
raise SystemExit(exit_code)
