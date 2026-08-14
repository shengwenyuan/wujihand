"""Shared Isaac materialization for the dual NERO + Hand 2 workcell."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Protocol, cast

import numpy as np
import numpy.typing as npt

from wujihand.application.teleoperation import (
    compose_partitioned_q27_target,
)
from wujihand.adapters.simulation.hand2_model import (
    Hand2ModelProfile,
    load_hand2_model_profile,
)
from wujihand.adapters.simulation.nero_hand2_twin import (
    NeroHand2AttachmentConfig,
    NeroHand2AttachmentHandles,
    NeroHand2DofPartition,
    author_nero_hand2_attachment,
    discover_nero_hand2_dofs,
)
from wujihand.adapters.simulation.nero_hand2_self_collision import (
    NeroHand2SelfCollisionFilterProfile,
    author_isaac_self_collision_filters,
)
from wujihand.adapters.simulation.nero_link_geometry_alignment import (
    NeroLinkGeometryAlignment,
    NeroLinkGeometryAlignmentHandles,
    apply_isaac_nero_link_geometry_alignment,
)
from wujihand.adapters.simulation.nero_model import (
    NERO_JOINT_NAMES,
    NeroModelProfile,
    load_nero_model_profile,
)
from wujihand.adapters.simulation.q27_execution import (
    IsaacQ27ExecutionAdapter,
)
from wujihand.specs import AttachmentSpec, PoseSpec
from wujihand.dataset.profile import Q54JointProfile
from wujihand.domain.dataset_recording import (
    DynamicRigidBodyTruth,
    KinematicLinkTruth,
    SimulationFramePhase,
    SimulationStateFrame,
)

from .isaac_d405_wrist_rig import (
    WristRigCollisionMode,
    materialize_isaac_d405_wrist_rigs,
    resolve_d405_wrist_rig_runtimes,
)
from .session_resolver import ResolvedSession
from .isaac_workcell import (
    IsaacWorkcellMaterialization,
    materialize_isaac_workcell,
)
from .isaac_workcell_plan import (
    ResolvedIsaacWorkcellPlan,
    resolve_isaac_workcell_plan,
)


@dataclass(frozen=True, slots=True)
class ScenePose:
    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]


class NeroSimulationDriveGains(Protocol):
    @property
    def stiffness(self) -> float: ...

    @property
    def damping(self) -> float: ...


class NeroDualSimulationStartup(Protocol):
    @property
    def arm_drive_gains(self) -> NeroSimulationDriveGains: ...

    def initial_position(
        self,
        instance_id: str,
        group_id: str,
        layout_id: str,
    ) -> npt.NDArray[np.float64]: ...


@dataclass(frozen=True, slots=True)
class DualSideRuntime:
    side: str
    arm_instance_id: str
    hand_instance_id: str
    arm_asset: Path
    hand_asset: Path
    arm_profile: Path
    hand_profile: Path
    arm_prim_path: str
    hand_prim_path: str
    mount_pose: ScenePose
    attachment: AttachmentSpec


@dataclass(frozen=True, slots=True)
class SceneRigidBodySnapshot:
    prim_path: str
    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    linear_velocity_m_s: tuple[float, float, float] | None
    angular_velocity_deg_s: tuple[float, float, float] | None
    kinematic_enabled: bool


@dataclass(frozen=True, slots=True)
class SceneFixedBodySnapshot:
    prim_path: str
    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class SceneKinematicLinkSnapshot:
    side: str
    logical_link_id: str
    prim_path: str
    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class SceneDatasetStateSnapshot:
    """Exact simulator truth before frame metadata and digest materialization."""

    q54_rad: tuple[float, ...]
    qdot54_rad_s: tuple[float, ...]
    rigid_bodies: tuple[DynamicRigidBodyTruth, ...]
    kinematic_links: tuple[KinematicLinkTruth, ...]

    def to_frame(
        self,
        *,
        run_id: str,
        control_index: int,
        phase: SimulationFramePhase,
        simulation_time_s: float,
        physics_boundary_index: int,
    ) -> SimulationStateFrame:
        return SimulationStateFrame.create(
            run_id=run_id,
            episode_id=run_id,
            control_index=control_index,
            tick_id=control_index,
            phase=phase,
            simulation_time_s=simulation_time_s,
            physics_boundary_index=physics_boundary_index,
            q54_rad=self.q54_rad,
            qdot54_rad_s=self.qdot54_rad_s,
            rigid_bodies=self.rigid_bodies,
            kinematic_links=self.kinematic_links,
            expected_rigid_body_count=len(self.rigid_bodies),
            expected_kinematic_link_count=len(self.kinematic_links),
        )


def expand_dataset_state_for_operator_preview(
    *,
    dataset_snapshot: SceneDatasetStateSnapshot,
    link_snapshots: tuple[SceneKinematicLinkSnapshot, ...],
) -> SceneDatasetStateSnapshot:
    """Expand recorded truth with the full, unrecorded operator link inventory."""

    return SceneDatasetStateSnapshot(
        q54_rad=dataset_snapshot.q54_rad,
        qdot54_rad_s=dataset_snapshot.qdot54_rad_s,
        rigid_bodies=dataset_snapshot.rigid_bodies,
        kinematic_links=tuple(
            KinematicLinkTruth(
                side=item.side,
                logical_link_id=item.logical_link_id,
                prim_path=item.prim_path,
                position_m=item.position_m,
                quat_wxyz=item.quat_wxyz,
                valid=True,
            )
            for item in link_snapshots
        ),
    )


@dataclass(frozen=True, slots=True)
class SceneReplaySnapshot:
    """Post-physics visual state sufficient for paused camera replay."""

    q27_by_side: tuple[
        tuple[str, tuple[float, ...]],
        tuple[str, tuple[float, ...]],
    ]
    rigid_bodies: tuple[SceneRigidBodySnapshot, ...]


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
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ],
            [
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ],
            [
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def _compose(parent: ScenePose, child: PoseSpec | ScenePose) -> ScenePose:
    position = np.asarray(
        parent.position_m,
        dtype=np.float64,
    ) + _rotation_matrix(parent.quat_wxyz) @ np.asarray(
        child.position_m,
        dtype=np.float64,
    )
    return ScenePose(
        position_m=cast(
            tuple[float, float, float],
            tuple(position.tolist()),
        ),
        quat_wxyz=_quat_multiply(
            parent.quat_wxyz,
            child.quat_wxyz,
        ),
    )


def _workcell_frame_pose(
    resolved: ResolvedSession,
    frame_id: str,
    cache: dict[str, ScenePose],
) -> ScenePose:
    if frame_id in cache:
        return cache[frame_id]
    frame = next(item for item in resolved.workcell.frames if item.frame_id == frame_id)
    result = _compose(
        _workcell_frame_pose(resolved, frame.parent, cache),
        frame.transform,
    )
    cache[frame_id] = result
    return result


def workcell_pose(
    resolved: ResolvedSession,
    frame_id: str,
    local: PoseSpec,
) -> ScenePose:
    cache = {
        resolved.workcell.world_frame: ScenePose(
            position_m=(0.0, 0.0, 0.0),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        )
    }
    return _compose(
        _workcell_frame_pose(resolved, frame_id, cache),
        local,
    )


def workcell_frame_position(
    resolved: ResolvedSession,
    frame_id: str,
) -> tuple[float, float, float]:
    cache = {
        resolved.workcell.world_frame: ScenePose(
            position_m=(0.0, 0.0, 0.0),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        )
    }
    return _workcell_frame_pose(resolved, frame_id, cache).position_m


def resolve_dual_side_runtimes(
    project_root: Path,
    resolved: ResolvedSession,
) -> tuple[DualSideRuntime, DualSideRuntime]:
    def profile_path(instance_id: str, group_id: str) -> Path:
        instance = resolved.instance(instance_id)
        profile = instance.asset.control_group(group_id).joint_profile
        if profile is None:
            raise RuntimeError(f"{instance_id}/{group_id} has no joint profile")
        path = project_root / profile
        if not path.is_file():
            raise RuntimeError(f"joint profile not found: {path}")
        return path

    result: list[DualSideRuntime] = []
    for attachment in resolved.assembly.attachments:
        parent_spec = resolved.assembly.instance(attachment.parent.instance)
        child_spec = resolved.assembly.instance(attachment.child.instance)
        if (parent_spec.role, child_spec.role) != ("arm", "end_effector"):
            continue
        arm = resolved.instance(attachment.parent.instance)
        hand = resolved.instance(attachment.child.instance)
        side = hand.asset.side
        if side not in {"left", "right"}:
            raise RuntimeError("Hand 2 attachment must declare an explicit side")
        if arm.asset.product != "agilex_nero" or hand.asset.product != "wuji_hand_2":
            raise RuntimeError("dual scene attachment must connect NERO to Hand 2")
        if (
            resolved.assembly.instance(arm.instance_id).role != "arm"
            or resolved.assembly.instance(hand.instance_id).role != "end_effector"
        ):
            raise RuntimeError("dual scene attachment roles must be arm -> end_effector")
        if attachment.parent.frame != arm.asset.frame_name(
            "tool_flange"
        ) or attachment.child.frame != hand.asset.frame_name("base"):
            raise RuntimeError("dual scene must connect tool flange to hand base")
        if arm.binding.loader != "usd" or hand.binding.loader != "usd":
            raise RuntimeError("dual scene requires USD bindings")
        if arm.artifact is None or hand.artifact is None:
            raise RuntimeError("dual scene instances require source-locked USD artifacts")
        mount = resolved.workcell.mount(resolved.session.mount_for(arm.instance_id))
        title = side.capitalize()
        result.append(
            DualSideRuntime(
                side=side,
                arm_instance_id=arm.instance_id,
                hand_instance_id=hand.instance_id,
                arm_asset=arm.artifact.absolute_path,
                hand_asset=hand.artifact.absolute_path,
                arm_profile=profile_path(arm.instance_id, "arm_joints"),
                hand_profile=profile_path(
                    hand.instance_id,
                    "finger_joints",
                ),
                arm_prim_path=f"/World/Robots/Nero{title}",
                hand_prim_path=f"/World/Robots/Hand2{title}",
                mount_pose=workcell_pose(
                    resolved,
                    mount.frame,
                    mount.transform,
                ),
                attachment=attachment,
            )
        )
    if {item.side for item in result} != {"left", "right"} or len(result) != 2:
        raise RuntimeError("dual scene must contain one left and one right attachment")
    return cast(
        tuple[DualSideRuntime, DualSideRuntime],
        tuple(sorted(result, key=lambda item: item.side)),
    )


class DualNeroHand2IsaacScene:
    """Materialized scene plus the sole atomic q27 execution boundary."""

    def __init__(
        self,
        *,
        project_root: Path,
        resolved: ResolvedSession,
        sides: tuple[DualSideRuntime, DualSideRuntime],
        alignment_profile: NeroLinkGeometryAlignment | None,
        qualification_profile: NeroDualSimulationStartup,
        physics_hz: int,
        self_collision_sides: frozenset[str] = frozenset(),
        self_collision_filter_profile: NeroHand2SelfCollisionFilterProfile | None = None,
        wrist_rig_collision_mode: WristRigCollisionMode = "none",
        visual_replay_only: bool = False,
        workcell_plan: ResolvedIsaacWorkcellPlan | None = None,
    ) -> None:
        from isaacsim.core.api import World  # type: ignore[import-not-found]
        from isaacsim.core.prims import (  # type: ignore[import-not-found]
            Articulation,
            SingleRigidPrim,
        )
        from isaacsim.core.utils.stage import (  # type: ignore[import-not-found]
            add_reference_to_stage,
        )
        from pxr import (  # type: ignore[import-not-found]
            Gf,
            UsdGeom,
        )

        if not self_collision_sides <= {"left", "right"}:
            raise ValueError("self_collision_sides must contain only left/right")
        if type(visual_replay_only) is not bool:
            raise TypeError("visual_replay_only must be a bool")
        if visual_replay_only and self_collision_sides:
            raise ValueError("visual replay must not enable runtime self collisions")
        if visual_replay_only and self_collision_filter_profile is not None:
            raise ValueError("visual replay must not author runtime collision filters")
        self.visual_replay_only = visual_replay_only
        self.resolved = resolved
        self.sides = sides
        self.alignment_profile = alignment_profile
        self.qualification_profile = qualification_profile
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / physics_hz,
            rendering_dt=1.0 / 30.0,
            backend="numpy",
            device="cpu",
        )
        self.stage = self.world.scene.stage
        if workcell_plan is None:
            workcell_plan = resolve_isaac_workcell_plan(
                project_root,
                resolved.workcell,
                verify_content=True,
            )
        elif workcell_plan.workcell_id != resolved.workcell.workcell_id:
            raise ValueError(
                "resolved Workcell plan does not match the Session Workcell"
            )
        self.workcell_materialization: IsaacWorkcellMaterialization = materialize_isaac_workcell(
            self.world, workcell_plan
        )
        self.dynamic_workcell_prims = (
            {}
            if visual_replay_only
            else {
                path: self.world.scene.add(
                    SingleRigidPrim(
                        prim_path=path,
                        name=f"camera_replay_dynamic_{index}",
                    )
                )
                for index, path in enumerate(self.workcell_materialization.rigid_body_paths)
            }
        )
        UsdGeom.Xform.Define(self.stage, "/World/Robots")
        UsdGeom.Xform.Define(self.stage, "/World/Attachments")

        self.authored: dict[str, NeroHand2AttachmentHandles] = {}
        self.geometry_alignments: dict[
            str,
            NeroLinkGeometryAlignmentHandles,
        ] = {}
        self.articulations: dict[str, Any] = {}
        self.arm_profiles: dict[str, NeroModelProfile] = {}
        self.hand_profiles: dict[str, Hand2ModelProfile] = {}
        self.initial_arm_targets: dict[
            str,
            npt.NDArray[np.float64],
        ] = {}
        for runtime in sides:
            add_reference_to_stage(
                str(runtime.arm_asset),
                runtime.arm_prim_path,
            )
            arm_root = self.stage.GetPrimAtPath(runtime.arm_prim_path)
            xformable = UsdGeom.Xformable(arm_root)
            xformable.ClearXformOpOrder()
            matrix = Gf.Matrix4d(1.0)
            matrix.SetRotate(Gf.Quatd(*runtime.mount_pose.quat_wxyz))
            matrix.SetTranslateOnly(Gf.Vec3d(*runtime.mount_pose.position_m))
            xformable.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(matrix)
            arm_instance = resolved.instance(runtime.arm_instance_id)
            hand_instance = resolved.instance(runtime.hand_instance_id)
            parent_link = _one_prim(
                self.stage,
                prefix=runtime.arm_prim_path,
                name=arm_instance.binding.backend_frame(runtime.attachment.parent.frame),
                rigid_body=True,
            )
            if alignment_profile is not None:
                wrist_housing = _one_prim(
                    self.stage,
                    prefix=runtime.arm_prim_path,
                    name=arm_instance.binding.backend_frame(
                        arm_instance.asset.frame_name("wrist_housing")
                    ),
                    rigid_body=True,
                )
                self.geometry_alignments[runtime.side] = (
                    apply_isaac_nero_link_geometry_alignment(
                        self.stage,
                        link_path=str(wrist_housing.GetPath()),
                        profile=alignment_profile,
                    )
                )
            add_reference_to_stage(
                str(runtime.hand_asset),
                runtime.hand_prim_path,
            )
            arm_articulation_root = _one_prim(
                self.stage,
                prefix=runtime.arm_prim_path,
                articulation_root=True,
            )
            child_base = _one_prim(
                self.stage,
                prefix=runtime.hand_prim_path,
                name=hand_instance.binding.backend_frame(runtime.attachment.child.frame),
                rigid_body=True,
            )
            hand_root_joint = _one_prim(
                self.stage,
                prefix=runtime.hand_prim_path,
                articulation_root=True,
                fixed_joint=True,
            )
            self.authored[runtime.side] = author_nero_hand2_attachment(
                self.stage,
                NeroHand2AttachmentConfig(
                    side=runtime.side,
                    nero_prim_path=runtime.arm_prim_path,
                    hand_prim_path=runtime.hand_prim_path,
                    nero_articulation_root_path=str(arm_articulation_root.GetPath()),
                    parent_link_path=str(parent_link.GetPath()),
                    child_base_link_path=str(child_base.GetPath()),
                    hand_root_joint_path=str(hand_root_joint.GetPath()),
                    attachment_joint_path=(
                        f"/World/Attachments/{runtime.attachment.attachment_id}"
                    ),
                    position_m=runtime.attachment.transform.position_m,
                    quat_wxyz=runtime.attachment.transform.quat_wxyz,
                    enable_self_collisions=runtime.side in self_collision_sides,
                ),
            )
            if not visual_replay_only:
                self.articulations[runtime.side] = self.world.scene.add(
                    Articulation(
                        str(arm_articulation_root.GetPath()),
                        name=f"nero_hand2_{runtime.side}",
                    )
                )
            arm_profile = load_nero_model_profile(runtime.arm_profile)
            hand_profile = load_hand2_model_profile(runtime.hand_profile)
            self.arm_profiles[runtime.side] = arm_profile
            self.hand_profiles[runtime.side] = hand_profile
            self.initial_arm_targets[runtime.side] = arm_profile.layout.validate_vector(
                qualification_profile.initial_position(
                    runtime.arm_instance_id,
                    "arm_joints",
                    arm_profile.layout_id,
                )
            ).copy()

        self.wrist_rig_runtimes = resolve_d405_wrist_rig_runtimes(project_root, resolved)
        self.wrist_rigs = materialize_isaac_d405_wrist_rigs(
            self.stage,
            runtimes=self.wrist_rig_runtimes,
            hand_base_paths={
                side: handles.config.child_base_link_path for side, handles in self.authored.items()
            },
            collision_mode=wrist_rig_collision_mode,
        )
        self.self_collision_filtered_pairs = (
            author_isaac_self_collision_filters(
                self.stage,
                arm_prim_paths={runtime.side: runtime.arm_prim_path for runtime in sides},
                hand_prim_paths={runtime.side: runtime.hand_prim_path for runtime in sides},
                enabled_sides=self_collision_sides,
                profile=self_collision_filter_profile,
            )
            if self_collision_filter_profile is not None
            else ()
        )
        self.expected_root_paths = tuple(
            sorted(handle.articulation_root_path for handle in self.authored.values())
        )
        self.partitions: dict[str, NeroHand2DofPartition]
        self.dataset_dynamic_object_paths: dict[str, str]
        self.dataset_kinematic_link_paths: dict[tuple[str, str], str]
        self.dataset_kinematic_link_indices: dict[tuple[str, str], int]
        self.operator_preview_link_paths: dict[tuple[str, str], str]
        self.operator_preview_link_indices: dict[tuple[str, str], int]
        if visual_replay_only:
            self.root_paths_before_reset = self.expected_root_paths
            self.partitions = {}
            self.arm_drive_runtime = {}
            self.external_fixed_collider_paths = list(
                self.workcell_materialization.fixed_collider_paths
            )
            self.arm_targets = {}
            self.hand_targets = {}
            self.dataset_dynamic_object_paths = {}
            self.dataset_kinematic_link_paths = {}
            self.dataset_kinematic_link_indices = {}
            self.operator_preview_link_paths = {}
            self.operator_preview_link_indices = {}
            return
        self.world.reset()
        (
            self.partitions,
            self.root_paths_before_reset,
        ) = self.validate_articulations()
        self.arm_drive_runtime = self.apply_arm_drive_gains(self.partitions)
        self.q27_execution = IsaacQ27ExecutionAdapter(self.articulations)
        self.external_fixed_collider_paths = list(
            self.workcell_materialization.fixed_collider_paths
        )
        if not self.external_fixed_collider_paths:
            raise RuntimeError("dual scene has no fixed external collider")
        self.arm_targets = {
            side: self.initial_arm_targets[side].copy() for side in ("left", "right")
        }
        self.hand_targets = {
            side: self.hand_profiles[side].rest_position.copy() for side in ("left", "right")
        }
        self.dataset_dynamic_object_paths = {}
        self.dataset_kinematic_link_paths = {}
        self.dataset_kinematic_link_indices = {}
        self.operator_preview_link_paths = {}
        self.operator_preview_link_indices = {}
        if resolved.session.dataset_profile is not None:
            self._discover_dataset_truth_inventory()

    def feedback_q27(
        self,
        side: str,
    ) -> npt.NDArray[np.float64]:
        return self.q27_execution.read_feedback_q27(side)

    def feedback_qdot27(
        self,
        side: str,
    ) -> npt.NDArray[np.float64]:
        """Read backend joint velocity; never derive qdot from adjacent positions."""

        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        values = np.asarray(
            self.articulations[side].get_joint_velocities(),
            dtype=np.float64,
        )
        if values.shape == (1, 27):
            values = values[0]
        if values.shape != (27,) or not np.isfinite(values).all():
            raise RuntimeError(f"{side} qdot27 backend readback is invalid: {values.shape}")
        return values.copy()

    def runtime_joint_inventory(
        self,
        side: str,
    ) -> tuple[tuple[str, ...], tuple[tuple[float, float], ...]]:
        """Return exact runtime order and limits used by the q54 startup gate."""

        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        articulation = self.articulations[side]
        names = tuple(str(name) for name in articulation.dof_names)
        limits = np.asarray(articulation.get_dof_limits(), dtype=np.float64)
        if limits.shape == (1, 27, 2):
            limits = limits[0]
        if (
            len(names) != 27
            or len(set(names)) != 27
            or limits.shape != (27, 2)
            or not np.isfinite(limits).all()
        ):
            raise RuntimeError(f"{side} q27 runtime inventory is invalid")
        return names, tuple((float(row[0]), float(row[1])) for row in limits)

    def _discover_dataset_truth_inventory(self) -> None:
        """Freeze scene-declared dynamic objects and bilateral link truth."""

        dynamic = dict(
            self.workcell_materialization.dynamic_rigid_body_paths
        )
        if not dynamic:
            raise RuntimeError(
                "dataset task scene must declare at least one dynamic rigid body"
            )
        self.dataset_dynamic_object_paths = dynamic

        link_paths: dict[tuple[str, str], str] = {}
        fingertip_ids = (
            "thumb_tip",
            "index_finger_tip",
            "middle_finger_tip",
            "ring_finger_tip",
            "pinky_tip",
        )
        for runtime in self.sides:
            side = runtime.side
            link7 = _one_prim(
                self.stage,
                prefix=runtime.arm_prim_path,
                name="link7",
                rigid_body=True,
            )
            link_paths[(side, "arm_link7")] = str(link7.GetPath())
            link_paths[(side, "palm")] = self.authored[side].config.child_base_link_path
            prefix = "l" if side == "left" else "r"
            for logical_id in fingertip_ids:
                prim = _one_prim(
                    self.stage,
                    prefix=runtime.hand_prim_path,
                    name=f"{prefix}_{logical_id}",
                    rigid_body=True,
                )
                link_paths[(side, logical_id)] = str(prim.GetPath())
        if len(link_paths) != 14:
            raise RuntimeError("008 dataset kinematic inventory must contain 14 links")
        self.dataset_kinematic_link_paths = link_paths

        link_indices: dict[tuple[str, str], int] = {}
        for side in ("left", "right"):
            body_names = tuple(str(name) for name in self.articulations[side].body_names)
            if len(body_names) != len(set(body_names)):
                raise RuntimeError(f"{side} articulation link names are not unique")
            body_indices = {name: index for index, name in enumerate(body_names)}
            for identity, path in link_paths.items():
                if identity[0] != side:
                    continue
                body_name = Path(path).name
                if body_name not in body_indices:
                    raise RuntimeError(
                        f"dataset kinematic prim is not an articulation link: {path}"
                    )
                link_indices[identity] = body_indices[body_name]
        if set(link_indices) != set(link_paths):
            raise RuntimeError("dataset kinematic articulation-link inventory drifted")
        self.dataset_kinematic_link_indices = link_indices

        # The MCAP deliberately stores only the 14 privileged links above.  The
        # unrecorded operator-preview topic carries every articulation link so a
        # passive renderer can reproduce the complete robot pose without stepping
        # its own physics scene.
        from pxr import UsdPhysics

        preview_paths: dict[tuple[str, str], str] = {}
        preview_indices: dict[tuple[str, str], int] = {}
        for runtime in self.sides:
            side = runtime.side
            prefixes = (runtime.arm_prim_path + "/", runtime.hand_prim_path + "/")
            rigid_paths_by_name: dict[str, list[str]] = {}
            for prim in self.stage.Traverse():
                path = str(prim.GetPath())
                if not path.startswith(prefixes) or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    continue
                rigid_paths_by_name.setdefault(prim.GetName(), []).append(path)
            body_names = tuple(str(name) for name in self.articulations[side].body_names)
            for index, body_name in enumerate(body_names):
                candidates = rigid_paths_by_name.get(body_name, [])
                if len(candidates) != 1:
                    raise RuntimeError(
                        f"{side} preview link path is ambiguous for {body_name}: {candidates}"
                    )
                identity = (side, body_name)
                preview_paths[identity] = candidates[0]
                preview_indices[identity] = index
        if len(preview_paths) <= len(link_paths):
            raise RuntimeError("operator preview must contain more than the 14 MCAP links")
        if set(preview_paths) != set(preview_indices):
            raise RuntimeError("operator preview articulation-link inventory drifted")
        self.operator_preview_link_paths = preview_paths
        self.operator_preview_link_indices = preview_indices

    def kinematic_link_snapshots(self) -> tuple[SceneKinematicLinkSnapshot, ...]:
        """Read exact world poses for link7, palm and all declared fingertips."""

        return self._kinematic_link_snapshots(
            paths=self.dataset_kinematic_link_paths,
            indices=self.dataset_kinematic_link_indices,
        )

    def operator_preview_link_snapshots(self) -> tuple[SceneKinematicLinkSnapshot, ...]:
        """Read every articulation-link pose for the unrecorded GUI replica."""

        return self._kinematic_link_snapshots(
            paths=self.operator_preview_link_paths,
            indices=self.operator_preview_link_indices,
        )

    def _kinematic_link_snapshots(
        self,
        *,
        paths: Mapping[tuple[str, str], str],
        indices: Mapping[tuple[str, str], int],
    ) -> tuple[SceneKinematicLinkSnapshot, ...]:
        """Read one declared articulation-link inventory from the tensor backend."""

        transforms_by_side: dict[str, npt.NDArray[np.float64]] = {}
        for side in ("left", "right"):
            articulation = self.articulations[side]
            physics_view = getattr(articulation, "_physics_view", None)
            if physics_view is None:
                raise RuntimeError(f"{side} articulation physics view is unavailable")
            transforms = np.asarray(
                physics_view.get_link_transforms(),
                dtype=np.float64,
            )
            body_count = len(articulation.body_names)
            if transforms.shape != (1, body_count, 7) or not np.isfinite(transforms).all():
                raise RuntimeError(
                    f"{side} articulation link transforms are invalid: {transforms.shape}"
                )
            transforms_by_side[side] = transforms[0]

        result: list[SceneKinematicLinkSnapshot] = []
        for (side, logical_id), path in sorted(paths.items()):
            index = indices.get((side, logical_id))
            if index is None:
                raise RuntimeError(f"kinematic link index disappeared: {path}")
            transform = transforms_by_side[side][index]
            result.append(
                SceneKinematicLinkSnapshot(
                    side=side,
                    logical_link_id=logical_id,
                    prim_path=path,
                    position_m=(
                        float(transform[0]),
                        float(transform[1]),
                        float(transform[2]),
                    ),
                    quat_wxyz=(
                        float(transform[6]),
                        float(transform[3]),
                        float(transform[4]),
                        float(transform[5]),
                    ),
                )
            )
        return tuple(result)

    def dataset_state_snapshot(
        self,
        *,
        q54_profile: Q54JointProfile,
        q27_by_side: Mapping[str, npt.NDArray[np.float64]],
        qdot27_by_side: Mapping[str, npt.NDArray[np.float64]],
        link_snapshots: tuple[SceneKinematicLinkSnapshot, ...] | None = None,
    ) -> SceneDatasetStateSnapshot:
        """Read and freeze all variable simulator truth without hashing it."""

        if set(q27_by_side) != {"left", "right"} or set(qdot27_by_side) != {
            "left",
            "right",
        }:
            raise ValueError("dataset state must cover left and right q27/qdot27")
        bodies: list[DynamicRigidBodyTruth] = []
        snapshots = {item.prim_path: item for item in self.rigid_body_snapshots()}
        if set(snapshots) != set(self.dataset_dynamic_object_paths.values()):
            raise RuntimeError("dataset dynamic rigid-body inventory drifted")
        for logical_id, path in sorted(self.dataset_dynamic_object_paths.items()):
            item = snapshots[path]
            if item.linear_velocity_m_s is None or item.angular_velocity_deg_s is None:
                raise RuntimeError(f"{logical_id} pose/twist backend readback is incomplete")
            bodies.append(
                DynamicRigidBodyTruth(
                    logical_object_id=logical_id,
                    prim_path=path,
                    position_m=item.position_m,
                    quat_wxyz=item.quat_wxyz,
                    linear_velocity_m_s=item.linear_velocity_m_s,
                    angular_velocity_rad_s=cast(
                        tuple[float, float, float],
                        tuple(math.radians(value) for value in item.angular_velocity_deg_s),
                    ),
                    sleeping=None,
                    kinematic=item.kinematic_enabled,
                    valid=True,
                )
            )
        link_snapshots = link_snapshots or self.kinematic_link_snapshots()
        snapshots_by_path = {item.prim_path: item for item in link_snapshots}
        if not set(self.dataset_kinematic_link_paths.values()) <= set(snapshots_by_path):
            raise RuntimeError("dataset kinematic-link inventory drifted")
        links: list[KinematicLinkTruth] = []
        for (side, logical_link_id), path in sorted(
            self.dataset_kinematic_link_paths.items()
        ):
            item = snapshots_by_path[path]
            links.append(
                KinematicLinkTruth(
                    side=side,
                    logical_link_id=logical_link_id,
                    prim_path=item.prim_path,
                    position_m=item.position_m,
                    quat_wxyz=item.quat_wxyz,
                    valid=True,
                )
            )
        q54 = q54_profile.assemble_from_q27(
            left_q27_rad=tuple(float(value) for value in q27_by_side["left"]),
            right_q27_rad=tuple(float(value) for value in q27_by_side["right"]),
        )
        qdot54 = q54_profile.assemble_velocity_from_q27(
            left_qdot27_rad_s=tuple(float(value) for value in qdot27_by_side["left"]),
            right_qdot27_rad_s=tuple(float(value) for value in qdot27_by_side["right"]),
        )
        return SceneDatasetStateSnapshot(
            q54_rad=q54,
            qdot54_rad_s=qdot54,
            rigid_bodies=tuple(bodies),
            kinematic_links=tuple(links),
        )

    def operator_preview_state_snapshot(
        self,
        *,
        dataset_snapshot: SceneDatasetStateSnapshot,
        link_snapshots: tuple[SceneKinematicLinkSnapshot, ...] | None = None,
    ) -> SceneDatasetStateSnapshot:
        """Expand one dataset snapshot with all links for the unrecorded GUI topic."""

        return expand_dataset_state_for_operator_preview(
            dataset_snapshot=dataset_snapshot,
            link_snapshots=(
                link_snapshots
                if link_snapshots is not None
                else self.operator_preview_link_snapshots()
            ),
        )

    def create_dataset_state_frame(
        self,
        *,
        run_id: str,
        control_index: int,
        phase: SimulationFramePhase,
        simulation_time_s: float,
        physics_boundary_index: int,
        q54_profile: Q54JointProfile,
        q27_by_side: Mapping[str, npt.NDArray[np.float64]],
        qdot27_by_side: Mapping[str, npt.NDArray[np.float64]],
    ) -> SimulationStateFrame:
        """Close one independently reconstructable pre/post simulation state."""

        snapshot = self.dataset_state_snapshot(
            q54_profile=q54_profile,
            q27_by_side=q27_by_side,
            qdot27_by_side=qdot27_by_side,
        )
        return snapshot.to_frame(
            run_id=run_id,
            control_index=control_index,
            phase=phase,
            simulation_time_s=simulation_time_s,
            physics_boundary_index=physics_boundary_index,
        )

    def restore_dataset_state_frame(
        self,
        frame: SimulationStateFrame,
        *,
        q54_profile: Q54JointProfile,
    ) -> None:
        """Inject an immutable pre-action state without advancing physics."""

        if frame.phase is not SimulationFramePhase.PRE_ACTION:
            raise ValueError("offline renderer accepts pre_action frames only")
        self._inject_dataset_state_frame(frame, q54_profile=q54_profile)

    def _inject_dataset_state_frame(
        self,
        frame: SimulationStateFrame,
        *,
        q54_profile: Q54JointProfile,
    ) -> None:
        """Restore recorded articulation and object truth without stepping."""

        left_q27, right_q27 = q54_profile.decompose_to_q27(frame.q54_rad)
        left_qdot27, right_qdot27 = q54_profile.decompose_velocity_to_q27(frame.qdot54_rad_s)
        for side, positions, velocities in (
            ("left", left_q27, left_qdot27),
            ("right", right_q27, right_qdot27),
        ):
            self.articulations[side].set_joint_positions(
                np.asarray(positions, dtype=np.float64)[np.newaxis, :]
            )
            self.articulations[side].set_joint_velocities(
                np.asarray(velocities, dtype=np.float64)[np.newaxis, :]
            )
        bodies = {item.logical_object_id: item for item in frame.rigid_bodies}
        if set(bodies) != set(self.dataset_dynamic_object_paths):
            raise ValueError("offline render rigid-body inventory drifted")
        for logical_id, path in self.dataset_dynamic_object_paths.items():
            item = bodies[logical_id]
            if item.prim_path != path or not item.valid:
                raise ValueError(f"offline render state differs for {logical_id}")
            rigid = self.dynamic_workcell_prims[path]
            rigid.set_world_pose(
                position=np.asarray(item.position_m, dtype=np.float64),
                orientation=np.asarray(item.quat_wxyz, dtype=np.float64),
            )
            rigid.set_linear_velocity(np.asarray(item.linear_velocity_m_s, dtype=np.float64))
            rigid.set_angular_velocity(np.asarray(item.angular_velocity_rad_s, dtype=np.float64))

    def apply_targets(
        self,
    ) -> dict[str, npt.NDArray[np.float64]]:
        applied: dict[str, npt.NDArray[np.float64]] = {}
        for side in ("left", "right"):
            arm_profile = self.arm_profiles[side]
            hand_profile = self.hand_profiles[side]
            arm = self.arm_targets[side]
            hand = self.hand_targets[side]
            if not (
                np.all(arm >= np.asarray(arm_profile.layout.lower))
                and np.all(arm <= np.asarray(arm_profile.layout.upper))
                and np.all(hand >= np.asarray(hand_profile.layout.lower))
                and np.all(hand <= np.asarray(hand_profile.layout.upper))
            ):
                raise RuntimeError(f"{side} q7/q20 target exceeds limits")
            target = compose_partitioned_q27_target(
                side=side,
                arm_indices_q7=(self.partitions[side].arm_indices_q7),
                hand_indices_q20=(self.partitions[side].hand_indices_q20),
                arm_q7=arm,
                hand_q20=hand,
            )
            self.q27_execution.apply_target_q27(target)
            applied[side] = target.positions.copy()
        return applied

    def teleport_to_targets(self) -> dict[str, npt.NDArray[np.float64]]:
        """Initialize simulation state directly at the current q27 targets."""

        applied = self.apply_targets()
        for side, positions in applied.items():
            self.articulations[side].set_joint_positions(positions[np.newaxis, :])
            self.articulations[side].set_joint_velocities(
                np.zeros((1, 27), dtype=np.float64)
            )
        return applied

    def rigid_body_snapshots(
        self,
    ) -> tuple[SceneRigidBodySnapshot, ...]:
        """Read raw post-step state for the frozen Workcell inventory."""

        from pxr import Usd, UsdGeom, UsdPhysics

        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        result: list[SceneRigidBodySnapshot] = []
        for path in self.workcell_materialization.rigid_body_paths:
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"recorded rigid body disappeared: {path}")
            transform = cache.GetLocalToWorldTransform(prim)
            position = transform.ExtractTranslation()
            quaternion = transform.ExtractRotationQuat()
            imaginary = quaternion.GetImaginary()
            rigid_body = UsdPhysics.RigidBodyAPI(prim)
            linear_velocity = _optional_vec3(rigid_body.GetVelocityAttr().Get())
            angular_velocity = _optional_vec3(rigid_body.GetAngularVelocityAttr().Get())
            kinematic = rigid_body.GetKinematicEnabledAttr().Get()
            result.append(
                SceneRigidBodySnapshot(
                    prim_path=path,
                    position_m=(
                        float(position[0]),
                        float(position[1]),
                        float(position[2]),
                    ),
                    quat_wxyz=(
                        float(quaternion.GetReal()),
                        float(imaginary[0]),
                        float(imaginary[1]),
                        float(imaginary[2]),
                    ),
                    linear_velocity_m_s=linear_velocity,
                    angular_velocity_deg_s=angular_velocity,
                    kinematic_enabled=(False if kinematic is None else bool(kinematic)),
                )
            )
        return tuple(result)

    def fixed_body_snapshots(
        self,
    ) -> tuple[SceneFixedBodySnapshot, ...]:
        """Read one manifest-time pose for each immutable task fixture."""

        from pxr import Usd, UsdGeom

        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        result: list[SceneFixedBodySnapshot] = []
        for path in self.workcell_materialization.fixed_rigid_body_paths:
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"fixed task fixture disappeared: {path}")
            transform = cache.GetLocalToWorldTransform(prim)
            position = transform.ExtractTranslation()
            quaternion = transform.ExtractRotationQuat()
            imaginary = quaternion.GetImaginary()
            result.append(
                SceneFixedBodySnapshot(
                    prim_path=path,
                    position_m=(
                        float(position[0]),
                        float(position[1]),
                        float(position[2]),
                    ),
                    quat_wxyz=(
                        float(quaternion.GetReal()),
                        float(imaginary[0]),
                        float(imaginary[1]),
                        float(imaginary[2]),
                    ),
                )
            )
        return tuple(result)

    def camera_replay_snapshot(
        self,
        *,
        q27_by_side: Mapping[str, npt.NDArray[np.float64]],
    ) -> SceneReplaySnapshot:
        """Capture the exact paused-render state after a physics substep."""

        if set(q27_by_side) != {"left", "right"}:
            raise ValueError("camera replay q27 input must cover left and right")
        q27_values: dict[str, tuple[float, ...]] = {}
        for side in ("left", "right"):
            values = np.asarray(q27_by_side[side], dtype=np.float64)
            if values.shape != (27,) or not np.isfinite(values).all():
                raise ValueError(f"invalid {side} camera replay q27 input")
            q27_values[side] = tuple(float(value) for value in values)

        return SceneReplaySnapshot(
            q27_by_side=(
                ("left", q27_values["left"]),
                ("right", q27_values["right"]),
            ),
            rigid_bodies=self.rigid_body_snapshots(),
        )

    def restore_camera_replay_snapshot(self, snapshot: SceneReplaySnapshot) -> None:
        """Teleport one recorded visual state without advancing physics."""

        q27_by_side = dict(snapshot.q27_by_side)
        if set(q27_by_side) != {"left", "right"}:
            raise ValueError("camera replay q27 snapshot must cover left and right")
        for side in ("left", "right"):
            positions = np.asarray(q27_by_side[side], dtype=np.float64)
            if positions.shape != (27,) or not np.isfinite(positions).all():
                raise ValueError(f"invalid {side} camera replay q27")
            self.articulations[side].set_joint_positions(positions[np.newaxis, :])
            self.articulations[side].set_joint_velocities(np.zeros((1, 27), dtype=np.float64))

        rigid_by_path = {item.prim_path: item for item in snapshot.rigid_bodies}
        if set(rigid_by_path) != set(self.dynamic_workcell_prims):
            raise ValueError("camera replay rigid-body inventory drifted")
        for path, rigid_prim in self.dynamic_workcell_prims.items():
            item = rigid_by_path[path]
            rigid_prim.set_world_pose(
                position=np.asarray(item.position_m, dtype=np.float64),
                orientation=np.asarray(item.quat_wxyz, dtype=np.float64),
            )
            rigid_prim.set_linear_velocity(np.zeros(3, dtype=np.float64))
            rigid_prim.set_angular_velocity(np.zeros(3, dtype=np.float64))

    def validate_articulations(
        self,
    ) -> tuple[
        dict[str, NeroHand2DofPartition],
        tuple[str, ...],
    ]:
        from pxr import UsdPhysics

        root_paths = tuple(
            sorted(
                str(prim.GetPath())
                for prim in self.stage.Traverse()
                if str(prim.GetPath()).startswith("/World/Robots/")
                and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            )
        )
        if root_paths != self.expected_root_paths:
            raise RuntimeError(
                "dual scene must contain exactly two expected q27 roots: "
                f"expected={self.expected_root_paths}, actual={root_paths}"
            )
        result: dict[str, NeroHand2DofPartition] = {}
        for runtime in self.sides:
            articulation = self.articulations[runtime.side]
            partition = discover_nero_hand2_dofs(
                articulation.dof_names,
                _dof_paths(articulation),
                NERO_JOINT_NAMES,
                self.hand_profiles[runtime.side].layout.names,
                nero_prim_path=runtime.arm_prim_path,
                hand_prim_path=runtime.hand_prim_path,
            )
            result[runtime.side] = partition
            limits = np.asarray(
                articulation.get_dof_limits(),
                dtype=np.float64,
            )
            if limits.shape != (1, 27, 2):
                raise RuntimeError(f"{runtime.side} q27 limits have shape {limits.shape}")
            arm_limits = limits[
                0,
                np.asarray(partition.arm_indices_q7),
            ]
            hand_limits = limits[
                0,
                np.asarray(partition.hand_indices_q20),
            ]
            expected_arm = np.column_stack(
                (
                    self.arm_profiles[runtime.side].layout.lower,
                    self.arm_profiles[runtime.side].layout.upper,
                )
            )
            expected_hand = np.column_stack(
                (
                    self.hand_profiles[runtime.side].layout.lower,
                    self.hand_profiles[runtime.side].layout.upper,
                )
            )
            if not np.allclose(arm_limits, expected_arm, atol=1e-4):
                raise RuntimeError(f"{runtime.side} NERO q7 limits drifted")
            if not np.allclose(hand_limits, expected_hand, atol=1e-4):
                raise RuntimeError(f"{runtime.side} Hand 2 q20 limits drifted")
        return result, root_paths

    def apply_arm_drive_gains(
        self,
        partitions: dict[str, NeroHand2DofPartition],
    ) -> dict[str, dict[str, list[float]]]:
        configured = self.qualification_profile.arm_drive_gains
        result: dict[str, dict[str, list[float]]] = {}
        for side in ("left", "right"):
            indices = np.asarray(
                partitions[side].arm_indices_q7,
                dtype=np.int64,
            )
            kps = np.full(
                (1, len(indices)),
                configured.stiffness,
                dtype=np.float32,
            )
            kds = np.full(
                (1, len(indices)),
                configured.damping,
                dtype=np.float32,
            )
            articulation = self.articulations[side]
            articulation.set_gains(
                kps=kps,
                kds=kds,
                joint_indices=indices,
            )
            actual_kps, actual_kds = articulation.get_gains(joint_indices=indices)
            actual_kps = np.asarray(actual_kps, dtype=np.float64)
            actual_kds = np.asarray(actual_kds, dtype=np.float64)
            if (
                actual_kps.shape != kps.shape
                or actual_kds.shape != kds.shape
                or not np.allclose(actual_kps, kps)
                or not np.allclose(actual_kds, kds)
            ):
                raise RuntimeError(f"{side} q7 drive gains were not applied")
            result[side] = {
                "stiffness": actual_kps[0].tolist(),
                "damping": actual_kds[0].tolist(),
            }
        return result


def _one_prim(
    stage: Any,
    *,
    prefix: str,
    name: str | None = None,
    articulation_root: bool = False,
    rigid_body: bool = False,
    fixed_joint: bool = False,
) -> Any:
    from pxr import UsdPhysics

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
            f"expected one prim below {prefix} for name={name!r}; "
            f"found {[str(item.GetPath()) for item in matches]}"
        )
    return matches[0]


def _dof_paths(articulation: Any) -> tuple[str, ...]:
    paths = np.asarray(
        getattr(articulation, "_dof_paths", None),
        dtype=object,
    )
    if paths.ndim != 2 or paths.shape[0] != 1:
        raise RuntimeError(f"expected one articulation DOF-path row, got {paths.shape}")
    result = tuple(str(path) for path in paths[0])
    if len(result) != len(articulation.dof_names):
        raise RuntimeError("Isaac DOF path/name counts differ")
    return result


def _optional_vec3(
    value: object,
) -> tuple[float, float, float] | None:
    if value is None:
        return None
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise RuntimeError("rigid-body velocity must be a finite vec3")
    return (float(vector[0]), float(vector[1]), float(vector[2]))


__all__ = [
    "DualNeroHand2IsaacScene",
    "DualSideRuntime",
    "SceneDatasetStateSnapshot",
    "ScenePose",
    "SceneKinematicLinkSnapshot",
    "SceneReplaySnapshot",
    "SceneRigidBodySnapshot",
    "resolve_dual_side_runtimes",
    "workcell_frame_position",
    "workcell_pose",
]
