#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Run a supervised Wuji Hand 2 table scene in a resolved Isaac Sim Session.

The scene accepts either loopback UDP teleoperation or a bounded scripted command.
Isaac Lab is not required. Validation artifacts are generated only when explicitly
requested, so normal live sessions do not retain an unbounded command log.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.runtime.session_compat import (
    ISAAC_FIXED_PREVIEW_SESSION,
    ISAAC_FIXED_TELEOP_SESSION,
    fixed_hand_workcell_runtime,
    resolve_isaac_hand_runtime,
)
from wujihand.integrity import sha256_file


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true", help="Open the Isaac Sim GUI.")
    parser.add_argument("--frames", type=int, default=600, help="Physics frames to run.")
    parser.add_argument("--command-source", choices=("scripted", "udp"), default="scripted")
    parser.add_argument("--udp-port", type=int, default=49152)
    parser.add_argument(
        "--session",
        type=Path,
        help="Five-layer Session; defaults by scripted/UDP command source.",
    )
    parser.add_argument(
        "--require-udp-loss-recovery",
        action="store_true",
        help="Require tracking, then packet loss, then a supervised return to rest.",
    )
    parser.add_argument(
        "--asset",
        type=Path,
        default=None,
        help="Explicit Hand 2 USD override.",
    )
    parser.add_argument(
        "--profile",
        type=Path,
        default=None,
        help="Explicit Hand 2 profile override.",
    )
    parser.add_argument(
        "--validation-output-dir",
        type=Path,
        help="Enable qualification checks and write their bounded artifacts to this path.",
    )
    return parser.parse_args()


ARGS = parse_args()
if ARGS.frames < 1:
    raise SystemExit("--frames must be positive")
default_session = (
    ISAAC_FIXED_TELEOP_SESSION
    if ARGS.command_source == "udp"
    else ISAAC_FIXED_PREVIEW_SESSION
)
SESSION_RUNTIME = resolve_isaac_hand_runtime(
    ROOT,
    session_path=ARGS.session or ROOT / default_session,
    runtime_roles=(
        {"teleop_consumer"} if ARGS.command_source == "udp" else {"simulation"}
    ),
    asset_override=ARGS.asset,
    profile_override=ARGS.profile,
)
ARGS.asset = SESSION_RUNTIME.asset_path
ARGS.profile = SESSION_RUNTIME.profile_path
WORKCELL_RUNTIME = fixed_hand_workcell_runtime(SESSION_RUNTIME.resolved)
if not ARGS.asset.is_file():
    raise SystemExit(f"Hand 2 USD not found: {ARGS.asset}")
if not ARGS.profile.is_file():
    raise SystemExit(f"Hand 2 profile not found: {ARGS.profile}")
if ARGS.require_udp_loss_recovery and ARGS.validation_output_dir is None:
    raise SystemExit("--require-udp-loss-recovery requires --validation-output-dir")

from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {"headless": not ARGS.gui, "width": 960, "height": 720, "anti_aliasing": 0}
)

import numpy as np
from pxr import UsdLux, UsdPhysics

import omni.kit.renderer_capture
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.version import get_version as get_isaac_sim_version
from wujihand.adapters.simulation import load_hand2_model_profile
from wujihand.adapters.transport import UdpJointCommandReceiver
from wujihand.application.supervision import JointCommandSupervisor
from wujihand.domain import HAND2_RIGHT_LAYOUT, HAND2_RIGHT_REST


PHYSICS_HZ = 120
COMMAND_HZ = 60
COMMAND_DIVISOR = PHYSICS_HZ // COMMAND_HZ
FEEDBACK_LIMIT_TOLERANCE_RAD = 0.01


def scripted_target(frame: int) -> np.ndarray:
    """Rest -> close -> hold -> rest, expressed in firmware q20 order."""

    close = np.array(
        [
            0.15,
            -0.35,
            0.75,
            0.75,
            0.65,
            0.00,
            1.10,
            0.85,
            0.72,
            0.00,
            1.20,
            0.90,
            0.72,
            0.00,
            1.20,
            0.90,
            0.68,
            0.00,
            1.10,
            0.85,
        ],
        dtype=np.float64,
    )
    if frame < 120:
        return HAND2_RIGHT_REST.copy()
    if frame < 300:
        alpha = min((frame - 120) / 120.0, 1.0)
        return close * alpha
    if frame < 360:
        return close
    alpha = max(1.0 - (frame - 360) / 120.0, 0.0)
    return close * alpha


def find_articulation_root(stage: object) -> str:
    roots = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    roots = [path for path in roots if path.startswith("/World/Hand2")]
    if len(roots) != 1:
        raise RuntimeError(f"expected one Hand 2 articulation root, found {roots}")
    return roots[0]


def main() -> int:
    validation_output = ARGS.validation_output_dir
    if validation_output is not None:
        validation_output.mkdir(parents=True, exist_ok=True)
        (validation_output / "error.txt").unlink(missing_ok=True)

    profile = load_hand2_model_profile(ARGS.profile)
    asset_sha256 = sha256_file(ARGS.asset)
    if asset_sha256 != profile.provenance.get("usd_sha256"):
        raise RuntimeError("Hand 2 USD SHA-256 differs from the pinned profile")
    if profile.layout != HAND2_RIGHT_LAYOUT:
        raise RuntimeError("Hand 2 profile differs from the pinned firmware layout")
    if not np.array_equal(profile.rest_position, HAND2_RIGHT_REST):
        raise RuntimeError("Hand 2 profile differs from the pinned rest position")

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / PHYSICS_HZ,
        rendering_dt=1.0 / 30.0,
        backend="numpy",
    )
    world.scene.add_default_ground_plane()
    table = world.scene.add(
        FixedCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.asarray(WORKCELL_RUNTIME.table.transform.position_m),
            scale=np.asarray(WORKCELL_RUNTIME.table.primitive.size_m),
            size=1.0,
            color=np.array([0.34, 0.20, 0.10]),
        )
    )
    add_reference_to_stage(str(ARGS.asset.resolve()), "/World/Hand2")
    stage = world.scene.stage
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(900.0)

    articulation_root = find_articulation_root(stage)
    hand = world.scene.add(Articulation(articulation_root, name="hand2_right"))
    world.reset()
    # Lay the hand above the table: local +X (palm normal) points toward world +Z.
    hand.set_world_poses(
        positions=np.asarray(
            [WORKCELL_RUNTIME.hand_mount.position_m], dtype=np.float32
        ),
        orientations=np.asarray(
            [WORKCELL_RUNTIME.hand_mount.quat_wxyz], dtype=np.float32
        ),
    )
    camera_eye = np.asarray(WORKCELL_RUNTIME.camera_eye_m)
    camera_target = np.asarray(WORKCELL_RUNTIME.camera_target_m)
    set_camera_view(
        eye=camera_eye,
        target=camera_target,
        camera_prim_path="/OmniverseKit_Persp",
    )
    world.step(render=True)

    backend_names = list(hand.dof_names)
    reorder = profile.layout.indices_for(backend_names)
    usd_limits = np.asarray(hand.get_dof_limits(), dtype=np.float64)[0]
    expected_limits = np.column_stack((profile.layout.lower, profile.layout.upper))[
        np.asarray(reorder)
    ]
    if not np.allclose(usd_limits, expected_limits, atol=1e-4):
        raise RuntimeError(
            f"USD limits differ from pinned profile: max_error={np.max(np.abs(usd_limits - expected_limits))}"
        )

    supervisor = JointCommandSupervisor(
        profile.layout,
        profile.rest_position,
        stale_after_s=0.25,
        velocity_scale=0.20,
    )
    receiver = UdpJointCommandReceiver(ARGS.udp_port) if ARGS.command_source == "udp" else None
    supervisor.arm(time.monotonic_ns() if receiver is not None else 0)
    last_decision = None
    command_log: list[dict[str, object]] | None = [] if validation_output is not None else None
    feedback_peak_abs = 0.0
    feedback_within_limits = True
    tracking_ticks = 0
    degraded_after_tracking_ticks = 0
    udp_target: np.ndarray | None = None
    udp_input_time_ns: int | None = None
    loop_started = time.monotonic()
    for frame in range(ARGS.frames):
        if receiver is not None:
            deadline = loop_started + frame / PHYSICS_HZ
            time.sleep(max(deadline - time.monotonic(), 0.0))
        if frame % COMMAND_DIVISOR == 0:
            feedback_backend = np.asarray(hand.get_joint_positions(), dtype=np.float64)[0]
            feedback_firmware = profile.backend_to_firmware(feedback_backend, backend_names)
            feedback_peak_abs = max(feedback_peak_abs, float(np.abs(feedback_firmware).max()))
            feedback_within_limits = feedback_within_limits and bool(
                np.all(
                    feedback_firmware
                    >= np.asarray(profile.layout.lower) - FEEDBACK_LIMIT_TOLERANCE_RAD
                )
                and np.all(
                    feedback_firmware
                    <= np.asarray(profile.layout.upper) + FEEDBACK_LIMIT_TOLERANCE_RAD
                )
            )
            if receiver is None:
                now_ns = int((frame + COMMAND_DIVISOR) / PHYSICS_HZ * 1e9)
                target = scripted_target(frame)
                input_time_ns = now_ns
                packet_sequence = None
            else:
                now_ns = time.monotonic_ns()
                packet = receiver.receive_latest()
                if packet is not None:
                    udp_target = packet.q20
                    udp_input_time_ns = packet.host_time_ns
                target = udp_target
                input_time_ns = udp_input_time_ns
                packet_sequence = None if packet is None else packet.sequence
            last_decision = supervisor.step(target, now_ns=now_ns, input_time_ns=input_time_ns)
            if last_decision.state.value == "tracking":
                tracking_ticks += 1
            elif tracking_ticks > 0:
                degraded_after_tracking_ticks += 1
            backend_q = profile.firmware_to_backend(last_decision.command, backend_names)
            hand.set_joint_position_targets(backend_q.reshape(1, -1))
            if command_log is not None:
                command_log.append(
                    {
                        "frame": frame,
                        "state": last_decision.state.value,
                        "reason": last_decision.reason,
                        "q20": last_decision.command.tolist(),
                        "feedback_q20": feedback_firmware.tolist(),
                        "packet_sequence": packet_sequence,
                    }
                )
        world.step(render=ARGS.gui or (validation_output is not None and frame % 4 == 0))

    actual_backend = np.asarray(hand.get_joint_positions(), dtype=np.float64)[0]
    actual_firmware = profile.backend_to_firmware(actual_backend, backend_names)
    final_target = last_decision.command if last_decision is not None else profile.rest_position
    final_error = np.abs(actual_firmware - final_target)
    finite = bool(np.isfinite(actual_firmware).all())

    if validation_output is None:
        summary = {
            "session": SESSION_RUNTIME.resolved.session.session_id,
            "session_hash": SESSION_RUNTIME.resolved.session_hash,
            "frames": ARGS.frames,
            "command_source": ARGS.command_source,
            "last_state": None if last_decision is None else last_decision.state.value,
            "last_reason": None if last_decision is None else last_decision.reason,
            "tracking_ticks": tracking_ticks,
            "degraded_after_tracking_ticks": degraded_after_tracking_ticks,
            "udp_packets_accepted": receiver.accepted if receiver is not None else None,
            "udp_packets_rejected": receiver.rejected if receiver is not None else None,
        }
        if receiver is not None:
            receiver.close()
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0

    # Render and capture one deterministic validation frame.
    for _ in range(8):
        world.step(render=True)
    table_position, _ = table.get_world_pose()
    hand_position, hand_orientation = hand.get_world_poses()
    screenshot = validation_output / "hand2_table.png"
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("active Isaac viewport is unavailable")
    capture = capture_viewport_to_file(viewport, file_path=str(screenshot))
    captured = simulation_app.run_coroutine(capture.wait_for_result(completion_frames=30))
    omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
    if not captured or not screenshot.is_file():
        raise RuntimeError("Isaac viewport capture did not complete")
    report = {
        "isaac_sim": get_isaac_sim_version()[0],
        "session": SESSION_RUNTIME.resolved.session.session_id,
        "session_hash": SESSION_RUNTIME.resolved.session_hash,
        "asset": str(ARGS.asset.resolve()),
        "asset_sha256": asset_sha256,
        "profile": str(ARGS.profile.resolve()),
        "profile_provenance": profile.provenance,
        "articulation_root": articulation_root,
        "dof_count": len(backend_names),
        "backend_dof_names": backend_names,
        "firmware_order": list(profile.layout.names),
        "backend_from_firmware_indices": list(reorder),
        "limits_match": True,
        "frames": ARGS.frames,
        "physics_hz": PHYSICS_HZ,
        "command_hz": COMMAND_HZ,
        "command_source": ARGS.command_source,
        "udp_port": ARGS.udp_port if receiver is not None else None,
        "udp_packets_accepted": receiver.accepted if receiver is not None else None,
        "udp_packets_rejected": receiver.rejected if receiver is not None else None,
        "actual_finite": finite,
        "feedback_peak_abs_rad": feedback_peak_abs,
        "feedback_within_limits": feedback_within_limits,
        "feedback_limit_tolerance_rad": FEEDBACK_LIMIT_TOLERANCE_RAD,
        "tracking_ticks": tracking_ticks,
        "degraded_after_tracking_ticks": degraded_after_tracking_ticks,
        "movement_observed": feedback_peak_abs > 0.20 if ARGS.frames >= 360 else None,
        "final_max_abs_error_rad": float(final_error.max()),
        "final_command_max_abs_rad": float(np.abs(final_target).max()),
        "camera_eye": camera_eye.tolist(),
        "camera_target": camera_target.tolist(),
        "table_position": np.asarray(table_position).tolist(),
        "hand_position": np.asarray(hand_position[0]).tolist(),
        "hand_orientation_wxyz": np.asarray(hand_orientation[0]).tolist(),
        "screenshot": str(screenshot),
    }
    (validation_output / "validation.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (validation_output / "commands.json").write_text(
        json.dumps(command_log, separators=(",", ":")) + "\n", encoding="utf-8"
    )
    if receiver is not None:
        receiver.close()
    print(json.dumps(report, indent=2, sort_keys=True))
    movement_ok = ARGS.frames < 360 or feedback_peak_abs > 0.20
    transport_ok = receiver is None or (receiver.accepted > 0 and tracking_ticks > 0)
    loss_recovery_ok = not ARGS.require_udp_loss_recovery or (
        receiver is not None
        and degraded_after_tracking_ticks > 0
        and last_decision is not None
        and last_decision.state.value == "degraded"
        and last_decision.reason == "stale_input_return_to_rest"
        and np.abs(final_target).max() < 0.02
    )
    if (
        not finite
        or not feedback_within_limits
        or not movement_ok
        or not transport_ok
        or not loss_recovery_ok
        or len(backend_names) != 20
        or final_error.max() > 0.20
    ):
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
