#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Run one isolated C0/C1 NERO—Hand 2 self-collision qualification phase."""

from __future__ import annotations

import argparse
from collections import defaultdict
from collections.abc import Mapping
import json
import math
from pathlib import Path
import sys
import traceback
from typing import Any, cast

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.simulation.nero_hand2_self_collision import (
    load_nero_hand2_self_collision_filter_profile,
    load_nero_hand2_self_collision_qualification_profile,
)
from wujihand.adapters.simulation.nero_link_geometry_alignment import (
    load_nero_link_geometry_alignment,
)
from wujihand.adapters.simulation.nero_tabletop import (
    load_nero_dual_tabletop_qualification_profile,
)
from wujihand.application.qualification.hand2_scripted import (
    build_hand2_qualification_targets,
)
from wujihand.domain.hand_teleoperation import HandSide
from wujihand.integrity import sha256_file
from wujihand.runtime import SessionResolver
from wujihand.runtime.isaac_dual_scene import (
    DualNeroHand2IsaacScene,
    resolve_dual_side_runtimes,
)


DEFAULT_SESSION = (
    ROOT / "configs/sessions/isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml"
)
DEFAULT_PROFILE = (
    ROOT / "configs/profiles/isaac_nero_hand2_self_collision_qualification_v1.yaml"
)
DEFAULT_FILTER_PROFILE = (
    ROOT / "configs/profiles/isaac_nero_hand2_self_collision_filtered_pairs_v1.yaml"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--filter-profile", type=Path, default=DEFAULT_FILTER_PROFILE)
    parser.add_argument(
        "--unfiltered",
        action="store_true",
        help="Retain every self-collision pair for the evidence-gathering baseline.",
    )
    parser.add_argument(
        "--enabled-sides",
        choices=("none", "left", "right", "both"),
        required=True,
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--wrist-rig-collision-mode",
        choices=("none", "mount", "all"),
        default="none",
    )
    parser.add_argument(
        "--mass-baseline-report",
        type=Path,
        help="Passing C1 report required by C2/C3 runtime inertial comparison.",
    )
    parser.add_argument("--export-stage", action="store_true")
    return parser.parse_args()


ARGS = _parse_args()

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": True})

import omni.physx
from isaacsim.core.prims import RigidPrim
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics


class ContactTracker:
    """Collect per-frame PhysX contact pairs without trusting aggregate forces."""

    def __init__(self, *, separation_epsilon_m: float) -> None:
        self.phase = "not_started"
        self.frame_index = -1
        self.separation_epsilon_m = separation_epsilon_m
        self._pairs: dict[tuple[str, str], dict[str, object]] = {}

    def set_frame(self, phase: str, frame_index: int) -> None:
        self.phase = phase
        self.frame_index = frame_index

    def callback(self, headers: object, data: object) -> None:
        for header in headers:  # type: ignore[union-attr]
            left = self._path(getattr(header, "collider0", 0))
            right = self._path(getattr(header, "collider1", 0))
            if left == "/" or right == "/":
                left = self._path(getattr(header, "actor0", 0))
                right = self._path(getattr(header, "actor1", 0))
            pair = tuple(sorted((left, right)))
            record = self._pairs.setdefault(
                cast(tuple[str, str], pair),
                {
                    "event_count": 0,
                    "minimum_separation_m": math.inf,
                    "event_phases": defaultdict(set),
                    "contact_phases": defaultdict(set),
                    "minimum_separation_by_phase_m": {},
                },
            )
            record["event_count"] = int(record["event_count"]) + 1
            cast(defaultdict[str, set[int]], record["event_phases"])[self.phase].add(
                self.frame_index
            )
            offset = int(getattr(header, "contact_data_offset", 0))
            count = int(getattr(header, "num_contact_data", 0))
            event_minimum = math.inf
            for index in range(offset, offset + count):
                separation = float(getattr(data[index], "separation"))  # type: ignore[index]
                event_minimum = min(event_minimum, separation)
                record["minimum_separation_m"] = min(
                    float(record["minimum_separation_m"]), separation
                )
            if event_minimum <= self.separation_epsilon_m:
                cast(defaultdict[str, set[int]], record["contact_phases"])[
                    self.phase
                ].add(self.frame_index)
            if not math.isinf(event_minimum):
                phase_minimum = cast(
                    dict[str, float], record["minimum_separation_by_phase_m"]
                )
                phase_minimum[self.phase] = min(
                    phase_minimum.get(self.phase, math.inf), event_minimum
                )

    @staticmethod
    def _path(encoded: object) -> str:
        try:
            return str(PhysicsSchemaTools.intToSdfPath(int(encoded)))
        except (TypeError, ValueError):
            return "/"

    def to_mapping(self) -> dict[str, object]:
        result: dict[str, object] = {}
        for pair, raw in sorted(self._pairs.items()):
            event_phases = cast(defaultdict[str, set[int]], raw["event_phases"])
            contact_phases = cast(defaultdict[str, set[int]], raw["contact_phases"])
            minimum = float(raw["minimum_separation_m"])
            result[" <-> ".join(pair)] = {
                "paths": list(pair),
                "event_count": raw["event_count"],
                "minimum_separation_m": None if math.isinf(minimum) else minimum,
                "phase_event_frames": {
                    phase: len(frames) for phase, frames in sorted(event_phases.items())
                },
                "phase_contact_frames": {
                    phase: len(frames) for phase, frames in sorted(contact_phases.items())
                },
                "phase_minimum_separation_m": dict(
                    sorted(
                        cast(
                            dict[str, float],
                            raw["minimum_separation_by_phase_m"],
                        ).items()
                    )
                ),
            }
        return result


def _enabled_sides(value: str) -> frozenset[str]:
    if value == "none":
        return frozenset()
    if value == "both":
        return frozenset({"left", "right"})
    return frozenset({value})


def _shared_alignment(project_root: Path, resolved: object, sides: object) -> object:
    references = {
        resolved.instance(runtime.arm_instance_id).binding.compatibility_profile  # type: ignore[union-attr]
        for runtime in sides  # type: ignore[union-attr]
    }
    if None in references or len(references) != 1:
        raise RuntimeError("both NERO bindings must share one alignment profile")
    return load_nero_link_geometry_alignment(project_root / cast(str, references.pop()))


def _world_transform(stage: object, path: str) -> dict[str, list[float]]:
    prim = stage.GetPrimAtPath(path)  # type: ignore[union-attr]
    transform = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    translation = transform.ExtractTranslation()
    quaternion = transform.ExtractRotationQuat()
    imaginary = quaternion.GetImaginary()
    return {
        "translation_m": [float(translation[index]) for index in range(3)],
        "quat_wxyz": [
            float(quaternion.GetReal()),
            *[float(imaginary[index]) for index in range(3)],
        ],
    }


def _topology(stage: object) -> dict[str, object]:
    roots: list[str] = []
    rigid_bodies: list[str] = []
    joints: list[str] = []
    for prim in stage.Traverse():  # type: ignore[union-attr]
        path = str(prim.GetPath())
        if not path.startswith("/World/Robots/"):
            continue
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            roots.append(path)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_bodies.append(path)
        if prim.IsA(UsdPhysics.Joint):
            joints.append(path)
    return {
        "articulation_roots": sorted(roots),
        "rigid_body_count": len(rigid_bodies),
        "joint_count": len(joints),
    }


def _author_contact_reports(stage: object, threshold_n: float) -> tuple[str, ...]:
    authored: list[str] = []
    for prim in stage.Traverse():  # type: ignore[union-attr]
        path = str(prim.GetPath())
        if not path.startswith("/World/Robots/") or not prim.HasAPI(
            UsdPhysics.RigidBodyAPI
        ):
            continue
        api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
        api.CreateThresholdAttr(threshold_n)
        authored.append(path)
    if not authored:
        raise RuntimeError("no robot rigid bodies accepted ContactReportAPI")
    return tuple(sorted(authored))


def _author_external_probes(
    scene: DualNeroHand2IsaacScene,
) -> dict[str, RigidPrim]:
    stage = scene.stage
    result: dict[str, RigidPrim] = {}
    for side in ("left", "right"):
        path = f"/World/Qualification/WristRigContactProbe{side.capitalize()}"
        sphere = UsdGeom.Sphere.Define(stage, path)
        sphere.CreateRadiusAttr(0.006)
        sphere.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.18, 0.08)])
        xformable = UsdGeom.Xformable(sphere.GetPrim())
        xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0.0, 0.0, -10.0)
        )
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        rigid = UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
        rigid.CreateKinematicEnabledAttr(True)
        result[side] = scene.world.scene.add(
            RigidPrim(path, name=f"wrist_rig_contact_probe_{side}")
        )
    return result


def _wrist_rig_physics_inventory(
    scene: DualNeroHand2IsaacScene,
) -> dict[str, object]:
    collision_paths: list[str] = []
    forbidden_paths: list[str] = []
    roots = tuple(handles.root_path for handles in scene.wrist_rigs)
    for prim in scene.stage.Traverse():
        path = str(prim.GetPath())
        if not any(path == root or path.startswith(root + "/") for root in roots):
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_paths.append(path)
        if (
            prim.HasAPI(UsdPhysics.RigidBodyAPI)
            or prim.HasAPI(UsdPhysics.MassAPI)
            or prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            or prim.IsA(UsdPhysics.Joint)
        ):
            forbidden_paths.append(path)
    return {
        "collision_paths": sorted(collision_paths),
        "forbidden_rigid_mass_joint_root_paths": sorted(forbidden_paths),
    }


def _run_external_probe_smoke(
    scene: DualNeroHand2IsaacScene,
    *,
    tracker: ContactTracker,
    probes: Mapping[str, RigidPrim],
    start_frame: int,
) -> int:
    for handles in scene.wrist_rigs:
        target_path = handles.mount_collision_paths[0]
        target = _world_transform(scene.stage, target_path)["translation_m"]
        probes[handles.side].set_world_poses(
            positions=np.asarray([target], dtype=np.float64),
            orientations=np.asarray([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
        )
    scene.world.play()
    frames = 12
    for offset in range(frames):
        tracker.set_frame("external_probe_smoke", start_frame + offset)
        scene.world.step(render=False)
    scene.world.pause()
    return frames


def _self_collision_readback(scene: DualNeroHand2IsaacScene) -> dict[str, bool]:
    return {
        side: bool(
            PhysxSchema.PhysxArticulationAPI(
                scene.stage.GetPrimAtPath(scene.authored[side].articulation_root_path)
            )
            .GetEnabledSelfCollisionsAttr()
            .Get()
        )
        for side in ("left", "right")
    }


def _runtime_body_properties(scene: DualNeroHand2IsaacScene) -> dict[str, object]:
    result: dict[str, object] = {}
    for side in ("left", "right"):
        articulation = scene.articulations[side]
        center_positions, center_orientations = articulation.get_body_coms()
        values = {
            "body_names": list(articulation.body_names),
            "masses_kg": np.asarray(
                articulation.get_body_masses(), dtype=np.float64
            ).tolist(),
            "centers_of_mass_m": np.asarray(
                center_positions, dtype=np.float64
            ).tolist(),
            "principal_axes_wxyz": np.asarray(
                center_orientations, dtype=np.float64
            ).tolist(),
            "inertias": np.asarray(
                articulation.get_body_inertias(), dtype=np.float64
            ).tolist(),
        }
        if not all(
            np.isfinite(np.asarray(value, dtype=np.float64)).all()
            for key, value in values.items()
            if key != "body_names"
        ):
            raise RuntimeError(f"{side} runtime body properties are non-finite")
        result[side] = values
    return result


def _mass_baseline_check(
    *,
    requested_sides: frozenset[str],
    session_hash: str,
    current: Mapping[str, object],
) -> tuple[bool, dict[str, object] | None]:
    if ARGS.wrist_rig_collision_mode == "none":
        if ARGS.mass_baseline_report is not None:
            raise ValueError("mass baseline is only valid when accessory collision is enabled")
        return True, None
    if ARGS.mass_baseline_report is None:
        raise ValueError("C2/C3 requires --mass-baseline-report from the matching C1 run")
    path = ARGS.mass_baseline_report.resolve()
    baseline = cast(
        Mapping[str, object], json.loads(path.read_text(encoding="utf-8"))
    )
    if (
        baseline.get("passed") is not True
        or baseline.get("gate") != "C1"
        or baseline.get("wrist_rig_collision_mode") != "none"
        or baseline.get("enabled_self_collision_sides") != sorted(requested_sides)
        or cast(Mapping[str, object], baseline.get("session"))["session_hash"]
        != session_hash
    ):
        raise RuntimeError("mass baseline report does not match this C2/C3 run")
    baseline_values = cast(Mapping[str, object], baseline["runtime_body_properties"])
    matches = all(
        cast(Mapping[str, object], baseline_values[side])["body_names"]
        == cast(Mapping[str, object], current[side])["body_names"]
        and all(
            np.allclose(
                np.asarray(
                    cast(Mapping[str, object], baseline_values[side])[field],
                    dtype=np.float64,
                ),
                np.asarray(
                    cast(Mapping[str, object], current[side])[field],
                    dtype=np.float64,
                ),
                rtol=0.0,
                atol=1e-12,
            )
            for field in (
                "masses_kg",
                "centers_of_mass_m",
                "principal_axes_wxyz",
                "inertias",
            )
        )
        for side in ("left", "right")
    )
    return matches, {"path": str(path), "sha256": sha256_file(path)}


def _transform_delta(
    start: Mapping[str, list[float]], end: Mapping[str, list[float]]
) -> tuple[float, float]:
    start_translation = np.asarray(start["translation_m"], dtype=np.float64)
    end_translation = np.asarray(end["translation_m"], dtype=np.float64)
    start_quaternion = np.asarray(start["quat_wxyz"], dtype=np.float64)
    end_quaternion = np.asarray(end["quat_wxyz"], dtype=np.float64)
    return (
        float(np.linalg.norm(end_translation - start_translation)),
        1.0 - abs(float(np.dot(start_quaternion, end_quaternion))),
    )


def _pair_side(path: str) -> str | None:
    if "Left" in path:
        return "left"
    if "Right" in path:
        return "right"
    return None


def main() -> int:
    project_root = ROOT
    output_dir = ARGS.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    profile = load_nero_hand2_self_collision_qualification_profile(ARGS.profile)
    resolved = SessionResolver(project_root).resolve(ARGS.session, verify_artifacts=True)
    sides = resolve_dual_side_runtimes(project_root, resolved)
    if resolved.session.runtime.compatibility_profile is None:
        raise RuntimeError("session must reference the tabletop qualification profile")
    tabletop_path = project_root / resolved.session.runtime.compatibility_profile
    tabletop = load_nero_dual_tabletop_qualification_profile(tabletop_path)
    requested_sides = _enabled_sides(ARGS.enabled_sides)
    if not requested_sides and ARGS.wrist_rig_collision_mode != "none":
        raise ValueError("C0 cannot enable wrist-rig collision")
    filter_profile = (
        load_nero_hand2_self_collision_filter_profile(ARGS.filter_profile)
        if requested_sides and not ARGS.unfiltered
        else None
    )
    scene = DualNeroHand2IsaacScene(
        project_root=project_root,
        resolved=resolved,
        sides=sides,
        alignment_profile=_shared_alignment(project_root, resolved, sides),
        qualification_profile=tabletop,
        physics_hz=profile.physics_hz,
        self_collision_sides=requested_sides,
        self_collision_filter_profile=filter_profile,
        wrist_rig_collision_mode=ARGS.wrist_rig_collision_mode,
    )
    probes = (
        _author_external_probes(scene)
        if ARGS.wrist_rig_collision_mode != "none"
        else None
    )
    contact_api_paths = _author_contact_reports(
        scene.stage, profile.thresholds.contact_report_threshold_n
    )
    topology_before = _topology(scene.stage)
    scene.world.reset()
    scene.partitions, root_paths_after_reset = scene.validate_articulations()
    scene.apply_arm_drive_gains(scene.partitions)
    readback = _self_collision_readback(scene)
    runtime_body_properties = _runtime_body_properties(scene)
    mass_baseline_matches, mass_baseline = _mass_baseline_check(
        requested_sides=requested_sides,
        session_hash=resolved.session_hash,
        current=runtime_body_properties,
    )
    expected_readback = {
        side: side in requested_sides for side in ("left", "right")
    }
    if readback != expected_readback:
        raise RuntimeError(
            f"self-collision readback mismatch: expected={expected_readback}, actual={readback}"
        )

    tracker = ContactTracker(
        separation_epsilon_m=profile.thresholds.contact_separation_epsilon_m
    )
    subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(
        tracker.callback
    )
    rest_targets = {
        side: scene.hand_profiles[side].rest_position.copy() for side in ("left", "right")
    }
    grasp_targets = {
        side: np.asarray(
            build_hand2_qualification_targets(
                HandSide(side),
                rest_targets[side],
                amplitude_rad=profile.hand_amplitude_rad,
            )[1].q20_rad,
            dtype=np.float64,
        )
        for side in ("left", "right")
    }
    phase_feedback: dict[str, dict[str, list[list[float]]]] = {}
    maximum_target_error = 0.0
    phase_maximum_target_error: dict[str, float] = {}
    phase_maximum_arm_error: dict[str, float] = {}
    phase_maximum_hand_error: dict[str, float] = {}
    finite = True
    global_frame = 0

    def step_phase(
        phase: str,
        frames: int,
        target_at: Any,
    ) -> None:
        nonlocal global_frame, maximum_target_error, finite
        samples = {"left": [], "right": []}
        phase_maximum_target_error[phase] = 0.0
        phase_maximum_arm_error[phase] = 0.0
        phase_maximum_hand_error[phase] = 0.0
        for frame in range(frames):
            for side in ("left", "right"):
                scene.hand_targets[side] = np.asarray(target_at(side, frame, frames)).copy()
            applied = scene.apply_targets()
            tracker.set_frame(phase, global_frame)
            scene.world.step(render=False)
            for side in ("left", "right"):
                feedback = np.asarray(scene.feedback_q27(side), dtype=np.float64)
                samples[side].append(feedback.tolist())
                finite &= bool(np.isfinite(feedback).all())
                error = float(np.max(np.abs(feedback - applied[side])))
                maximum_target_error = max(maximum_target_error, error)
                phase_maximum_target_error[phase] = max(
                    phase_maximum_target_error[phase], error
                )
                arm_indices = np.asarray(
                    scene.partitions[side].arm_indices_q7, dtype=np.int64
                )
                hand_indices = np.asarray(
                    scene.partitions[side].hand_indices_q20, dtype=np.int64
                )
                phase_maximum_arm_error[phase] = max(
                    phase_maximum_arm_error[phase],
                    float(np.max(np.abs(feedback[arm_indices] - applied[side][arm_indices]))),
                )
                phase_maximum_hand_error[phase] = max(
                    phase_maximum_hand_error[phase],
                    float(np.max(np.abs(feedback[hand_indices] - applied[side][hand_indices]))),
                )
            global_frame += 1
        phase_feedback[phase] = samples

    scene.world.play()

    def constant_rest(side: str, _frame: int, _frames: int) -> np.ndarray[Any, Any]:
        return rest_targets[side]

    def constant_grasp(side: str, _frame: int, _frames: int) -> np.ndarray[Any, Any]:
        return grasp_targets[side]

    step_phase("settle_rest", profile.phases.settle_rest, constant_rest)
    hand_base_after_settle = {
        side: _world_transform(scene.stage, scene.authored[side].config.child_base_link_path)
        for side in ("left", "right")
    }
    step_phase("observe_rest", profile.phases.observe_rest, constant_rest)

    def closing(side: str, frame: int, frames: int) -> np.ndarray[Any, Any]:
        alpha = 0.5 - 0.5 * math.cos(math.pi * (frame + 1) / frames)
        return rest_targets[side] + alpha * (grasp_targets[side] - rest_targets[side])

    def opening(side: str, frame: int, frames: int) -> np.ndarray[Any, Any]:
        alpha = 0.5 + 0.5 * math.cos(math.pi * (frame + 1) / frames)
        return rest_targets[side] + alpha * (grasp_targets[side] - rest_targets[side])

    step_phase("close_trajectory", profile.phases.close_trajectory, closing)
    step_phase("hold_grasp", profile.phases.hold_grasp, constant_grasp)
    step_phase("open_trajectory", profile.phases.open_trajectory, opening)
    step_phase("final_rest", profile.phases.final_rest, constant_rest)
    scene.world.pause()

    topology_after = _topology(scene.stage)
    _, root_paths_final = scene.validate_articulations()
    hand_base_final = {
        side: _world_transform(scene.stage, scene.authored[side].config.child_base_link_path)
        for side in ("left", "right")
    }
    if probes is not None:
        global_frame += _run_external_probe_smoke(
            scene,
            tracker=tracker,
            probes=probes,
            start_frame=global_frame,
        )
    del subscription
    hold_drift = {
        phase: max(
            float(
                np.max(
                    np.abs(
                        np.asarray(samples[side][-1]) - np.asarray(samples[side][0])
                    )
                )
            )
            for side in ("left", "right")
        )
        for phase, samples in phase_feedback.items()
        if phase in {"observe_rest", "hold_grasp", "final_rest"}
    }
    transform_drift = {
        side: dict(
            zip(
                ("translation_m", "rotation_one_minus_abs_dot"),
                _transform_delta(hand_base_after_settle[side], hand_base_final[side]),
                strict=True,
            )
        )
        for side in ("left", "right")
    }
    contacts = tracker.to_mapping()
    external_probe_contact_pairs = [
        pair_name
        for pair_name, raw_record in contacts.items()
        if "WristRigContactProbe" in pair_name
        and "D405WristRig" in pair_name
        and cast(Mapping[str, int], cast(Mapping[str, object], raw_record)["phase_contact_frames"])
        .get("external_probe_smoke", 0)
        > 0
    ]
    steady_phases = {"observe_rest", "hold_grasp", "final_rest"}
    maximum_steady_target_error = max(
        phase_maximum_target_error[phase] for phase in steady_phases
    )
    maximum_steady_arm_error = max(
        phase_maximum_arm_error[phase] for phase in steady_phases
    )
    maximum_steady_hand_error = max(
        phase_maximum_hand_error[phase] for phase in steady_phases
    )
    rest_phases = {"observe_rest", "final_rest"}
    unexplained_rest_pairs: list[str] = []
    deep_self_pairs: list[str] = []
    cross_side_frames = 0
    for pair_name, raw_record in contacts.items():
        record = cast(Mapping[str, object], raw_record)
        paths = cast(list[str], record["paths"])
        sides_for_pair = tuple(_pair_side(path) for path in paths)
        phase_frames = cast(Mapping[str, int], record["phase_contact_frames"])
        phase_minimum = cast(
            Mapping[str, float], record["phase_minimum_separation_m"]
        )
        if set(sides_for_pair) == {"left", "right"}:
            cross_side_frames += sum(phase_frames.values())
        same_robot_side = (
            sides_for_pair[0] is not None and sides_for_pair[0] == sides_for_pair[1]
        )
        if not same_robot_side:
            continue
        rest_frames = sum(phase_frames.get(phase, 0) for phase in rest_phases)
        rest_minimum = min(
            (phase_minimum[phase] for phase in rest_phases if phase in phase_minimum),
            default=math.inf,
        )
        primary_minimum = min(
            (
                value
                for phase, value in phase_minimum.items()
                if phase != "external_probe_smoke"
            ),
            default=math.inf,
        )
        rest_penetration = (
            0.0 if math.isinf(rest_minimum) else max(0.0, -rest_minimum)
        )
        primary_penetration = (
            0.0 if math.isinf(primary_minimum) else max(0.0, -primary_minimum)
        )
        if (
            rest_frames > profile.thresholds.maximum_unexplained_rest_contact_frames
            or rest_penetration
            > profile.thresholds.maximum_unexplained_rest_penetration_m
            and rest_frames > 0
        ):
            unexplained_rest_pairs.append(pair_name)
        if primary_penetration > profile.thresholds.maximum_any_self_penetration_m:
            deep_self_pairs.append(pair_name)

    expected_mount_collision_count = (
        14 if ARGS.wrist_rig_collision_mode in {"mount", "all"} else 0
    )
    expected_camera_collision_count = (
        1 if ARGS.wrist_rig_collision_mode == "all" else 0
    )
    wrist_rig_physics_inventory = _wrist_rig_physics_inventory(scene)
    checks = {
        "self_collision_readback_matches_requested_sides": readback == expected_readback,
        "two_q27_roots_preserved": (
            root_paths_after_reset == scene.root_paths_before_reset == root_paths_final
        ),
        "topology_preserved": topology_before == topology_after,
        "q27_feedback_finite": finite,
        "q7_target_error_bounded": (
            maximum_steady_arm_error
            <= profile.thresholds.maximum_arm_target_error_rad
        ),
        "q20_target_error_bounded": (
            maximum_steady_hand_error
            <= profile.thresholds.maximum_hand_target_error_rad
        ),
        "hold_drift_bounded": (
            max(hold_drift.values()) <= profile.thresholds.maximum_hold_drift_rad
        ),
        "hand_base_translation_stable": all(
            float(values["translation_m"])
            <= profile.thresholds.transform_translation_tolerance_m
            for values in transform_drift.values()
        ),
        "hand_base_rotation_stable": all(
            float(values["rotation_one_minus_abs_dot"])
            <= profile.thresholds.transform_rotation_tolerance
            for values in transform_drift.values()
        ),
        "no_unexplained_rest_self_contact": not unexplained_rest_pairs,
        "no_deep_self_penetration": not deep_self_pairs,
        "no_cross_side_contact": (
            cross_side_frames <= profile.thresholds.maximum_cross_side_contact_frames
        ),
        "contact_reporting_enabled": bool(contact_api_paths),
        "wrist_rig_inventory_matches_gate": (
            all(
                len(handles.mount_collision_paths) == expected_mount_collision_count
                and len(handles.camera_collision_paths)
                == expected_camera_collision_count
                for handles in scene.wrist_rigs
            )
            and (len(scene.wrist_rigs) == 2 or not scene.wrist_rig_runtimes)
        ),
        "hand_base_authored_mass_properties_unchanged": all(
            handles.hand_base_mass_before == handles.hand_base_mass_after
            for handles in scene.wrist_rigs
        ),
        "runtime_body_mass_properties_match_c1_baseline": mass_baseline_matches,
        "wrist_rig_adds_no_rigid_mass_joint_or_root": not cast(
            list[str],
            wrist_rig_physics_inventory["forbidden_rigid_mass_joint_root_paths"],
        ),
        "external_mount_contact_smoke_passes": (
            len(external_probe_contact_pairs) == len(scene.wrist_rigs)
            if ARGS.wrist_rig_collision_mode != "none"
            else True
        ),
        "filtered_pairs_match_profile": (
            len(scene.self_collision_filtered_pairs)
            == (
                0
                if filter_profile is None
                else sum(
                    side in requested_sides
                    for rule in filter_profile.filtered_pairs
                    for side in rule.sides
                )
            )
        ),
    }
    passed = all(checks.values())
    gate = (
        "C0"
        if not requested_sides
        else {"none": "C1", "mount": "C2", "all": "C3"}[
            ARGS.wrist_rig_collision_mode
        ]
    )
    report = {
        "schema": "wujihand.isaac_nero_hand2_self_collision_qualification.v1",
        "gate": gate,
        "phase": ARGS.enabled_sides,
        "passed": passed,
        "scope": "simulation_only collision qualification",
        "session": {
            "path": ARGS.session.resolve().relative_to(project_root).as_posix(),
            "session_hash": resolved.session_hash,
        },
        "profile": {
            "path": ARGS.profile.resolve().relative_to(project_root).as_posix(),
            "sha256": sha256_file(ARGS.profile),
            "profile_id": profile.profile_id,
        },
        "filtered_pair_profile": (
            None
            if filter_profile is None
            else {
                "path": ARGS.filter_profile.resolve()
                .relative_to(project_root)
                .as_posix(),
                "sha256": sha256_file(ARGS.filter_profile),
                "profile_id": filter_profile.profile_id,
            }
        ),
        "authored_filtered_pairs": [
            {
                "pair_id": pair_id,
                "first_rigid_body_path": first_path,
                "second_rigid_body_path": second_path,
            }
            for pair_id, first_path, second_path in scene.self_collision_filtered_pairs
        ],
        "enabled_self_collision_sides": sorted(requested_sides),
        "wrist_rig_collision_mode": ARGS.wrist_rig_collision_mode,
        "wrist_rig": [
            {
                "side": handles.side,
                "root_path": handles.root_path,
                "mount_visual_path": handles.mount_visual_path,
                "camera_visual_path": handles.camera_visual_path,
                "mount_collision_paths": list(handles.mount_collision_paths),
                "camera_collision_paths": list(handles.camera_collision_paths),
                "authored_mass_properties_before": {
                    "mass_kg": handles.hand_base_mass_before.mass_kg,
                    "center_of_mass_m": handles.hand_base_mass_before.center_of_mass_m,
                    "diagonal_inertia_kg_m2": (
                        handles.hand_base_mass_before.diagonal_inertia_kg_m2
                    ),
                    "principal_axes_wxyz": (
                        handles.hand_base_mass_before.principal_axes_wxyz
                    ),
                },
            }
            for handles in scene.wrist_rigs
        ],
        "runtime_body_properties": runtime_body_properties,
        "mass_baseline": mass_baseline,
        "wrist_rig_physics_inventory": wrist_rig_physics_inventory,
        "external_probe_contact_pairs": external_probe_contact_pairs,
        "self_collision_readback": readback,
        "physics": {
            "physics_hz": profile.physics_hz,
            "simulated_frames": global_frame,
            "simulated_duration_s": global_frame / profile.physics_hz,
        },
        "topology_before": topology_before,
        "topology_after": topology_after,
        "q27": {
            "dof_count_per_side": {side: 27 for side in ("left", "right")},
            "maximum_target_error_rad": maximum_target_error,
            "maximum_steady_target_error_rad": maximum_steady_target_error,
            "maximum_steady_arm_error_rad": maximum_steady_arm_error,
            "maximum_steady_hand_error_rad": maximum_steady_hand_error,
            "phase_maximum_target_error_rad": phase_maximum_target_error,
            "phase_maximum_arm_error_rad": phase_maximum_arm_error,
            "phase_maximum_hand_error_rad": phase_maximum_hand_error,
            "hold_drift_rad": hold_drift,
            "finite": finite,
        },
        "hand_base_after_settle": hand_base_after_settle,
        "hand_base_final": hand_base_final,
        "hand_base_transform_drift": transform_drift,
        "contact_report_api_paths": list(contact_api_paths),
        "contacts": contacts,
        "unexplained_rest_pairs": unexplained_rest_pairs,
        "deep_self_pairs": deep_self_pairs,
        "cross_side_contact_frames": cross_side_frames,
        "checks": checks,
    }
    report_path = output_dir / "report.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if ARGS.export_stage:
        scene.stage.Export(str(output_dir / "stage.usda"))
    print(
        f"SELF COLLISION QUALIFICATION {'PASS' if passed else 'FAIL'}: "
        f"gate={report['gate']} phase={ARGS.enabled_sides} report={report_path}",
        flush=True,
    )
    return 0 if passed else 2


exit_code = 1
try:
    exit_code = main()
except BaseException:  # Isaac fast shutdown can otherwise swallow the traceback.
    traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
finally:
    simulation_app.close()
raise SystemExit(exit_code)
