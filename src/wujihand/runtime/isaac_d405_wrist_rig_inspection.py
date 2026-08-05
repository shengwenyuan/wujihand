"""Preflight and settle the formal dual D405 wrist-rig Isaac inspector."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, cast

import numpy as np

from wujihand.adapters.simulation.nero_hand2_self_collision import (
    NeroHand2SelfCollisionFilterProfile,
    NeroHand2SelfCollisionQualificationProfile,
    load_nero_hand2_self_collision_filter_profile,
    load_nero_hand2_self_collision_qualification_profile,
)
from wujihand.adapters.simulation.nero_link_geometry_alignment import (
    NeroLinkGeometryAlignment,
    load_nero_link_geometry_alignment,
)
from wujihand.adapters.simulation.nero_tabletop import (
    NeroDualTabletopQualificationProfile,
    load_nero_dual_tabletop_qualification_profile,
)
from wujihand.application.qualification.hand2_scripted import (
    build_hand2_qualification_targets,
)
from wujihand.domain.hand_teleoperation import HandSide

from .isaac_d405_wrist_rig import (
    D405WristRigRuntime,
    resolve_d405_wrist_rig_runtimes,
)
from .isaac_dual_scene import (
    DualNeroHand2IsaacScene,
    DualSideRuntime,
    resolve_dual_side_runtimes,
)
from .session_resolver import ResolvedSession, SessionResolver


HandPose = Literal["rest", "grasp"]


@dataclass(frozen=True, slots=True)
class D405WristRigInspectionPlan:
    """All static-inspector inputs resolved before Isaac starts."""

    project_root: Path
    session_path: Path
    filter_profile_path: Path
    qualification_profile_path: Path
    resolved: ResolvedSession
    sides: tuple[DualSideRuntime, DualSideRuntime]
    wrist_rigs: tuple[D405WristRigRuntime, D405WristRigRuntime]
    alignment: NeroLinkGeometryAlignment
    tabletop: NeroDualTabletopQualificationProfile
    self_collision_filter: NeroHand2SelfCollisionFilterProfile
    self_collision_qualification: NeroHand2SelfCollisionQualificationProfile


@dataclass(slots=True)
class D405WristRigInspection:
    """Settled and paused inspection stage retained by the GUI lifecycle."""

    plan: D405WristRigInspectionPlan
    scene: DualNeroHand2IsaacScene
    hand_pose: HandPose
    settled_snapshot: dict[str, object]
    paused_snapshot: dict[str, object]
    final_target_error_rad: dict[str, float]

    def hand_base_point_world_m(
        self,
        side: str,
        local_point_m: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Transform a view point from the selected Hand 2 base to world."""

        from pxr import Gf, Usd, UsdGeom  # type: ignore[import-not-found]

        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        path = self.scene.authored[side].config.child_base_link_path
        prim = self.scene.stage.GetPrimAtPath(path)
        if not prim.IsValid() or not prim.IsA(UsdGeom.Xformable):
            raise RuntimeError(f"invalid Hand 2 inspection frame: {path}")
        transform = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
            prim
        )
        point = transform.Transform(Gf.Vec3d(*local_point_m))
        return cast(
            tuple[float, float, float],
            tuple(float(point[index]) for index in range(3)),
        )


def _project_file(project_root: Path, path: str | Path) -> Path:
    candidate = Path(path).expanduser()
    if not candidate.is_absolute():
        candidate = project_root / candidate
    resolved = candidate.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    return resolved


def preflight_d405_wrist_rig_inspection(
    *,
    project_root: str | Path,
    session_path: str | Path,
    filter_profile_path: str | Path,
    qualification_profile_path: str | Path,
    verify_artifacts: bool = True,
) -> D405WristRigInspectionPlan:
    """Resolve the device-free eight-instance scene and qualification profiles."""

    root = Path(project_root).expanduser().resolve()
    if not root.is_dir():
        raise FileNotFoundError(root)
    session = _project_file(root, session_path)
    filter_path = _project_file(root, filter_profile_path)
    qualification_path = _project_file(root, qualification_profile_path)
    resolved = SessionResolver(root).resolve(
        session,
        verify_artifacts=verify_artifacts,
    )
    if (
        resolved.session.backend != "isaac"
        or resolved.session.runtime_role != "simulation"
        or resolved.session.runtime.transport_contract is not None
    ):
        raise RuntimeError("D405 wrist-rig inspection requires a device-free Isaac Session")
    sides = resolve_dual_side_runtimes(root, resolved)
    wrist_rigs = resolve_d405_wrist_rig_runtimes(root, resolved)
    if len(wrist_rigs) != 2 or tuple(rig.side for rig in wrist_rigs) != (
        "left",
        "right",
    ):
        raise RuntimeError("inspection Session must contain left and right D405 wrist rigs")
    alignment_refs = {
        resolved.instance(side.arm_instance_id).binding.compatibility_profile
        for side in sides
    }
    if None in alignment_refs or len(alignment_refs) != 1:
        raise RuntimeError("both NERO bindings must share one geometry alignment profile")
    tabletop_ref = resolved.session.runtime.compatibility_profile
    if tabletop_ref is None:
        raise RuntimeError("inspection Session lacks a tabletop qualification profile")
    self_collision_qualification = (
        load_nero_hand2_self_collision_qualification_profile(qualification_path)
    )
    if self_collision_qualification.physics_hz != 120:
        raise RuntimeError("static inspector requires the qualified 120 Hz physics rate")
    return D405WristRigInspectionPlan(
        project_root=root,
        session_path=session,
        filter_profile_path=filter_path,
        qualification_profile_path=qualification_path,
        resolved=resolved,
        sides=sides,
        wrist_rigs=wrist_rigs,
        alignment=load_nero_link_geometry_alignment(
            _project_file(root, cast(str, alignment_refs.pop()))
        ),
        tabletop=load_nero_dual_tabletop_qualification_profile(
            _project_file(root, tabletop_ref)
        ),
        self_collision_filter=load_nero_hand2_self_collision_filter_profile(
            filter_path
        ),
        self_collision_qualification=self_collision_qualification,
    )


def inspection_state_snapshot(scene: DualNeroHand2IsaacScene) -> dict[str, object]:
    """Read the physics and attachment state protected by the pause Gate."""

    from pxr import Usd, UsdGeom

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    def transform(path: str) -> list[list[float]]:
        prim = scene.stage.GetPrimAtPath(path)
        if not prim.IsValid() or not prim.IsA(UsdGeom.Xformable):
            raise RuntimeError(f"inspection transform prim disappeared: {path}")
        matrix = cache.GetLocalToWorldTransform(prim)
        return [
            [float(matrix[row][column]) for column in range(4)] for row in range(4)
        ]

    return {
        "q27": {
            side: scene.feedback_q27(side).tolist() for side in ("left", "right")
        },
        "hand_base_world": {
            side: transform(scene.authored[side].config.child_base_link_path)
            for side in ("left", "right")
        },
        "wrist_rig_world": {
            handles.side: transform(handles.root_path) for handles in scene.wrist_rigs
        },
        "camera_world": {
            handles.side: transform(handles.camera_prim_path)
            for handles in scene.wrist_rigs
        },
    }


def materialize_d405_wrist_rig_inspection(
    plan: D405WristRigInspectionPlan,
    *,
    hand_pose: HandPose,
    render_settle_frames: int = 8,
) -> D405WristRigInspection:
    """Build, settle and pause the formal inspector without base-frame overrides."""

    if hand_pose not in {"rest", "grasp"}:
        raise ValueError("hand_pose must be rest or grasp")
    settle_frames = plan.self_collision_qualification.phases.settle_rest
    if render_settle_frames < 1 or render_settle_frames > settle_frames:
        raise ValueError("render_settle_frames must fit inside the settle phase")
    scene = DualNeroHand2IsaacScene(
        project_root=plan.project_root,
        resolved=plan.resolved,
        sides=plan.sides,
        alignment_profile=plan.alignment,
        qualification_profile=plan.tabletop,
        physics_hz=plan.self_collision_qualification.physics_hz,
        self_collision_sides=frozenset({"left", "right"}),
        self_collision_filter_profile=plan.self_collision_filter,
        wrist_rig_collision_mode="all",
    )
    for side in ("left", "right"):
        rest = scene.hand_profiles[side].rest_position.copy()
        if hand_pose == "rest":
            scene.hand_targets[side] = rest
        else:
            scene.hand_targets[side] = np.asarray(
                build_hand2_qualification_targets(
                    HandSide(side),
                    rest,
                    amplitude_rad=plan.self_collision_qualification.hand_amplitude_rad,
                )[1].q20_rad,
                dtype=np.float64,
            )
    scene.world.play()
    applied: dict[str, np.ndarray[Any, Any]] = {}
    for frame in range(settle_frames):
        applied = scene.apply_targets()
        scene.world.step(render=frame >= settle_frames - render_settle_frames)
    settled_snapshot = inspection_state_snapshot(scene)
    final_error = {
        side: float(np.max(np.abs(scene.feedback_q27(side) - applied[side])))
        for side in ("left", "right")
    }
    if any(
        error > plan.self_collision_qualification.thresholds.maximum_hand_target_error_rad
        for error in final_error.values()
    ):
        raise RuntimeError(f"selected Hand 2 pose did not settle: {final_error}")
    roots_before = scene.root_paths_before_reset
    _, roots_after = scene.validate_articulations()
    if roots_after != roots_before:
        raise RuntimeError("static inspection changed q27 articulation roots")
    scene.world.pause()
    paused_snapshot = inspection_state_snapshot(scene)
    if paused_snapshot != settled_snapshot or scene.world.is_playing():
        raise RuntimeError("world.pause changed the settled inspection state")
    return D405WristRigInspection(
        plan=plan,
        scene=scene,
        hand_pose=hand_pose,
        settled_snapshot=settled_snapshot,
        paused_snapshot=paused_snapshot,
        final_target_error_rad=final_error,
    )


__all__ = [
    "D405WristRigInspection",
    "D405WristRigInspectionPlan",
    "HandPose",
    "inspection_state_snapshot",
    "materialize_d405_wrist_rig_inspection",
    "preflight_d405_wrist_rig_inspection",
]
