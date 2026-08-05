"""Compose the visual wrist-mount overlay onto the qualified dual Isaac scene."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import cast

from wujihand.adapters.simulation.nero_hand2_gemini305_mount import (
    MOUNT_V1_CONFIG,
    NeroHand2Gemini305MountConfig,
    NeroHand2Gemini305OverlayHandles,
    StlMesh,
    author_inspection_lights,
    author_nero_hand2_gemini305_mount_overlay,
    load_stl_mesh_mm,
    sha256_file,
    stl_geometry_sha256,
)
from wujihand.adapters.simulation.nero_link_geometry_alignment import (
    NeroLinkGeometryAlignment,
    load_nero_link_geometry_alignment,
)
from wujihand.adapters.simulation.nero_tabletop import (
    NeroDualTabletopQualificationProfile,
    load_nero_dual_tabletop_qualification_profile,
)

from .isaac_dual_scene import (
    DualNeroHand2IsaacScene,
    DualSideRuntime,
    resolve_dual_side_runtimes,
)
from .session_resolver import ResolvedSession, SessionResolver


@dataclass(frozen=True, slots=True)
class IsaacWristMountInspectionPlan:
    """Fully resolved, device-free inputs validated before Isaac starts."""

    project_root: Path
    session_path: Path
    mount_stl_path: Path
    camera_stl_path: Path | None
    resolved: ResolvedSession
    sides: tuple[DualSideRuntime, DualSideRuntime]
    alignment: NeroLinkGeometryAlignment
    qualification: NeroDualTabletopQualificationProfile
    config: NeroHand2Gemini305MountConfig
    mount_mesh: StlMesh
    camera_mesh: StlMesh | None


@dataclass(slots=True)
class IsaacWristMountInspection:
    """Materialized scene and visual-overlay handles kept alive by the GUI tool."""

    plan: IsaacWristMountInspectionPlan
    scene: DualNeroHand2IsaacScene
    hand_base_path: str
    overlay: NeroHand2Gemini305OverlayHandles
    light_paths: tuple[str, ...]
    articulation_root_paths: tuple[str, ...]

    def hand_base_point_world_m(
        self,
        local_mm: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Transform one inspection camera point from Hand 2 local mm to world m."""

        from pxr import Gf, Usd, UsdGeom  # type: ignore[import-not-found]

        prim = self.scene.stage.GetPrimAtPath(self.hand_base_path)
        if not prim.IsValid() or not prim.IsA(UsdGeom.Xformable):
            raise RuntimeError(f"right Hand 2 base disappeared: {self.hand_base_path}")
        matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
        point = matrix.Transform(Gf.Vec3d(*(value * 0.001 for value in local_mm)))
        result = (float(point[0]), float(point[1]), float(point[2]))
        if not all(abs(value) < 1e6 for value in result):
            raise RuntimeError("non-finite Hand 2 camera point")
        return result

    @property
    def assembly_eye_world_m(self) -> tuple[float, float, float]:
        return self.hand_base_point_world_m(self.plan.config.assembly_eye_hand_base_mm)

    @property
    def assembly_target_world_m(self) -> tuple[float, float, float]:
        return self.hand_base_point_world_m(self.plan.config.assembly_target_hand_base_mm)


def _input_path(path: str | Path, *, base: Path | None = None) -> Path:
    candidate = Path(path).expanduser()
    if not candidate.is_absolute() and base is not None:
        candidate = base / candidate
    resolved = candidate.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    return resolved


def _shared_alignment(
    project_root: Path,
    resolved: ResolvedSession,
    sides: tuple[DualSideRuntime, DualSideRuntime],
) -> NeroLinkGeometryAlignment:
    refs = {resolved.instance(side.arm_instance_id).binding.compatibility_profile for side in sides}
    if None in refs or len(refs) != 1:
        raise RuntimeError("both NERO bindings must share one geometry alignment profile")
    path = _input_path(cast(str, next(iter(refs))), base=project_root)
    return load_nero_link_geometry_alignment(path)


def _qualification(
    project_root: Path,
    resolved: ResolvedSession,
) -> NeroDualTabletopQualificationProfile:
    ref = resolved.session.runtime.compatibility_profile
    if ref is None:
        raise RuntimeError("inspection Session has no tabletop qualification profile")
    return load_nero_dual_tabletop_qualification_profile(_input_path(ref, base=project_root))


def _verify_file_digest(path: Path, expected: str, *, label: str) -> None:
    actual = sha256_file(path)
    if actual != expected:
        raise RuntimeError(
            f"{label} SHA-256 differs: expected={expected}, actual={actual}, path={path}"
        )


def preflight_isaac_wrist_mount_inspection(
    *,
    project_root: str | Path,
    session_path: str | Path,
    mount_stl_path: str | Path,
    camera_stl_path: str | Path | None,
    config: NeroHand2Gemini305MountConfig = MOUNT_V1_CONFIG,
    verify_session_artifacts: bool = True,
    verify_mount_digest: bool = True,
    verify_camera_digest: bool = True,
) -> IsaacWristMountInspectionPlan:
    """Resolve all five-layer and mesh inputs before backend initialization."""

    root = Path(project_root).expanduser().resolve()
    if not root.is_dir():
        raise FileNotFoundError(root)
    session = _input_path(session_path, base=root)
    mount_stl = _input_path(mount_stl_path, base=root)
    camera_stl = None if camera_stl_path is None else _input_path(camera_stl_path, base=root)
    _input_path(config.scad_source, base=root)

    resolved = SessionResolver(root).resolve(
        session,
        verify_artifacts=verify_session_artifacts,
    )
    if resolved.session.backend != "isaac" or resolved.session.runtime_role != "simulation":
        raise RuntimeError("wrist-mount inspection requires an Isaac simulation Session")
    if resolved.session.runtime.transport_contract is not None:
        raise RuntimeError("wrist-mount inspection requires a device-free Session")
    sides = resolve_dual_side_runtimes(root, resolved)
    if {side.side for side in sides} != {"left", "right"}:
        raise RuntimeError("wrist-mount inspection requires the qualified dual scene")

    mount_mesh = load_stl_mesh_mm(mount_stl)
    if verify_mount_digest:
        actual_geometry_digest = stl_geometry_sha256(mount_mesh)
        if actual_geometry_digest != config.mount_geometry_sha256:
            raise RuntimeError(
                "mount STL geometry SHA-256 differs: "
                f"expected={config.mount_geometry_sha256}, "
                f"actual={actual_geometry_digest}, path={mount_stl}"
            )
    if camera_stl is not None and verify_camera_digest:
        _verify_file_digest(
            camera_stl,
            config.aligned_camera_mesh_sha256,
            label="aligned Gemini 305 STL",
        )
    return IsaacWristMountInspectionPlan(
        project_root=root,
        session_path=session,
        mount_stl_path=mount_stl,
        camera_stl_path=camera_stl,
        resolved=resolved,
        sides=sides,
        alignment=_shared_alignment(root, resolved, sides),
        qualification=_qualification(root, resolved),
        config=config,
        mount_mesh=mount_mesh,
        camera_mesh=None if camera_stl is None else load_stl_mesh_mm(camera_stl),
    )


def _physics_prims_below(stage: object, root_path: str) -> tuple[str, ...]:
    from pxr import UsdPhysics

    result: list[str] = []
    for prim in stage.Traverse():  # type: ignore[attr-defined]
        path = str(prim.GetPath())
        if path != root_path and not path.startswith(root_path + "/"):
            continue
        if (
            prim.IsA(UsdPhysics.Joint)
            or prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            or prim.HasAPI(UsdPhysics.CollisionAPI)
            or prim.HasAPI(UsdPhysics.MassAPI)
            or prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ):
            result.append(path)
    return tuple(result)


def materialize_isaac_wrist_mount_inspection(
    plan: IsaacWristMountInspectionPlan,
    *,
    physics_hz: int = 120,
    settle_frames: int = 240,
    render_settle_frames: int = 8,
    show_camera_visual_aids: bool | None = None,
    extra_lights: bool = True,
) -> IsaacWristMountInspection:
    """Build the qualified scene, add a visual overlay, settle, and pause it."""

    if physics_hz < 1 or settle_frames < 1 or render_settle_frames < 1:
        raise ValueError("physics_hz and settle frame counts must be positive")
    scene = DualNeroHand2IsaacScene(
        project_root=plan.project_root,
        resolved=plan.resolved,
        sides=plan.sides,
        alignment_profile=plan.alignment,
        qualification_profile=plan.qualification,
        physics_hz=physics_hz,
    )
    roots_before = scene.root_paths_before_reset
    hand_base_path = scene.authored[plan.config.side].config.child_base_link_path
    overlay = author_nero_hand2_gemini305_mount_overlay(
        scene.stage,
        hand_base_path=hand_base_path,
        config=plan.config,
        mount_mesh=plan.mount_mesh,
        camera_mesh=plan.camera_mesh,
        show_camera_visual_aids=(
            plan.camera_mesh is None if show_camera_visual_aids is None else show_camera_visual_aids
        ),
    )
    lights = (
        author_inspection_lights(scene.stage, overlay.overlay_root_path) if extra_lights else ()
    )
    for frame in range(settle_frames):
        scene.apply_targets()
        scene.world.step(render=frame >= settle_frames - render_settle_frames)
    _, roots_after = scene.validate_articulations()
    if roots_after != roots_before:
        raise RuntimeError(
            f"visual overlay changed articulation roots: before={roots_before}, after={roots_after}"
        )
    physics_prims = _physics_prims_below(scene.stage, overlay.overlay_root_path)
    if physics_prims:
        raise RuntimeError(f"inspection overlay must remain visual-only: {physics_prims}")
    scene.world.pause()
    return IsaacWristMountInspection(
        plan=plan,
        scene=scene,
        hand_base_path=hand_base_path,
        overlay=overlay,
        light_paths=lights,
        articulation_root_paths=roots_after,
    )


__all__ = [
    "IsaacWristMountInspection",
    "IsaacWristMountInspectionPlan",
    "materialize_isaac_wrist_mount_inspection",
    "preflight_isaac_wrist_mount_inspection",
]
