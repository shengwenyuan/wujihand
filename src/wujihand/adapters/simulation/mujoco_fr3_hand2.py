"""MuJoCo composition adapter for a pedestal-mounted FR3 v2 and Wuji Hand 2 right.

MuJoCo is an optional dependency and is imported only when this adapter is
constructed.  The two upstream MJCF files remain read-only: the combined model
is assembled in memory with ``MjSpec`` and is not serialized to a lossy XML.
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib
from pathlib import Path
from types import ModuleType
from typing import Any, Sequence

import numpy as np
import numpy.typing as npt

from wujihand.adapters.simulation.fr3_model import Fr3ModelProfile, load_fr3_model_profile
from wujihand.adapters.simulation.hand2_model import (
    Hand2ModelProfile,
    load_hand2_model_profile,
)
from wujihand.compat.mujoco_table import MujocoTableSceneConfig
from wujihand.domain.joints import FloatArray
from wujihand.integrity import sha256_file, sha256_tree
from wujihand.specs import GroupBindingSpec


FINGERTIP_SITE_NAMES = (
    "r_thumb_tip",
    "r_index_finger_tip",
    "r_middle_finger_tip",
    "r_ring_finger_tip",
    "r_pinky_tip",
)


def _require_mujoco() -> ModuleType:
    try:
        return importlib.import_module("mujoco")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "MuJoCo is not installed; install the project with the 'mujoco' extra"
        ) from exc

def _mesh_geom_world_vertices(
    mujoco: ModuleType, model: Any, data: Any, geom_name: str
) -> npt.NDArray[np.float64]:
    geom_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name))
    if geom_id < 0 or int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_MESH):
        raise RuntimeError(f"required mesh geom missing: {geom_name}")
    mesh_id = int(model.geom_dataid[geom_id])
    first_vertex = int(model.mesh_vertadr[mesh_id])
    vertex_count = int(model.mesh_vertnum[mesh_id])
    local_vertices = np.asarray(
        model.mesh_vert[first_vertex : first_vertex + vertex_count], dtype=np.float64
    )
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
    position = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
    return local_vertices @ rotation.T + position


@dataclass(frozen=True, slots=True)
class MujocoJointBinding:
    """Name-derived addresses for one independently controlled joint partition."""

    names: tuple[str, ...]
    joint_ids: tuple[int, ...]
    qpos_addresses: tuple[int, ...]
    dof_addresses: tuple[int, ...]
    actuator_ids: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class MujocoFr3Hand2State:
    arm_q7: FloatArray
    arm_dq7: FloatArray
    hand_q20: FloatArray
    hand_dq20: FloatArray
    flange_position_m: FloatArray
    flange_quat_wxyz: FloatArray
    palm_position_m: FloatArray
    palm_quat_wxyz: FloatArray
    fingertip_positions_m: FloatArray
    contact_count: int
    simulation_time_s: float


def _enum_value(mujoco: ModuleType, enum_name: str, member_name: str) -> int:
    return int(getattr(getattr(mujoco, enum_name), member_name))


def _apply_global_options(mujoco: ModuleType, spec: Any, config: MujocoTableSceneConfig) -> None:
    """Give both parent and child the same explicit scene-owned global option."""

    integrators = {
        "Euler": "mjINT_EULER",
        "RK4": "mjINT_RK4",
        "implicit": "mjINT_IMPLICIT",
        "implicitfast": "mjINT_IMPLICITFAST",
    }
    solvers = {"PGS": "mjSOL_PGS", "CG": "mjSOL_CG", "Newton": "mjSOL_NEWTON"}
    jacobians = {
        "auto": "mjJAC_AUTO",
        "dense": "mjJAC_DENSE",
        "sparse": "mjJAC_SPARSE",
    }
    option = spec.option
    option.timestep = config.physics.timestep_s
    option.integrator = _enum_value(
        mujoco, "mjtIntegrator", integrators[config.physics.integrator]
    )
    option.solver = _enum_value(mujoco, "mjtSolver", solvers[config.physics.solver])
    option.jacobian = _enum_value(mujoco, "mjtJacobian", jacobians[config.physics.jacobian])
    option.iterations = config.physics.iterations
    option.tolerance = config.physics.tolerance
    option.gravity = config.physics.gravity_m_s2


def _camera_xyaxes(eye: Sequence[float], target: Sequence[float]) -> list[float]:
    eye_array = np.asarray(eye, dtype=np.float64)
    target_array = np.asarray(target, dtype=np.float64)
    camera_z = eye_array - target_array
    camera_z /= np.linalg.norm(camera_z)
    up = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    camera_x = np.cross(up, camera_z)
    if np.linalg.norm(camera_x) < 1e-8:
        up = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
        camera_x = np.cross(up, camera_z)
    camera_x /= np.linalg.norm(camera_x)
    camera_y = np.cross(camera_z, camera_x)
    return [*camera_x.tolist(), *camera_y.tolist()]


def _pedestal_mesh_data(config: MujocoTableSceneConfig) -> tuple[list[float], list[int]]:
    """Return one closed convex frustum centered on its configured body position."""

    top_x, top_y = (value / 2.0 for value in config.arm_pedestal.top_size_m)
    bottom_x, bottom_y = (value / 2.0 for value in config.arm_pedestal.bottom_size_m)
    half_height = config.arm_pedestal.height_m / 2.0
    vertices = [
        -bottom_x,
        -bottom_y,
        -half_height,
        bottom_x,
        -bottom_y,
        -half_height,
        bottom_x,
        bottom_y,
        -half_height,
        -bottom_x,
        bottom_y,
        -half_height,
        -top_x,
        -top_y,
        half_height,
        top_x,
        -top_y,
        half_height,
        top_x,
        top_y,
        half_height,
        -top_x,
        top_y,
        half_height,
    ]
    faces = [
        0,
        2,
        1,
        0,
        3,
        2,
        4,
        5,
        6,
        4,
        6,
        7,
        0,
        1,
        5,
        0,
        5,
        4,
        1,
        2,
        6,
        1,
        6,
        5,
        2,
        3,
        7,
        2,
        7,
        6,
        3,
        0,
        4,
        3,
        4,
        7,
    ]
    return vertices, faces


def _author_environment(mujoco: ModuleType, spec: Any, config: MujocoTableSceneConfig) -> None:
    spec.visual.global_.offwidth = config.camera.width_px
    spec.visual.global_.offheight = config.camera.height_px
    spec.visual.headlight.active = 0
    world = spec.worldbody
    world.add_geom(
        name="floor",
        type=_enum_value(mujoco, "mjtGeom", "mjGEOM_PLANE"),
        pos=[0.0, 0.0, config.floor.z_m],
        size=[2.5, 2.5, 0.1],
        rgba=config.floor.color_rgba,
        friction=config.table.friction,
        contype=1,
        conaffinity=1,
    )
    half_size = np.asarray(config.table.size_m, dtype=np.float64) / 2.0
    world.add_geom(
        name="tabletop",
        type=_enum_value(mujoco, "mjtGeom", "mjGEOM_BOX"),
        pos=config.table.center_m,
        size=half_size.tolist(),
        rgba=config.table.color_rgba,
        friction=config.table.friction,
        contype=1,
        conaffinity=1,
    )
    table_bottom = config.table.center_m[2] - half_size[2]
    leg_height = table_bottom - config.floor.z_m
    leg_half_width = config.table.leg_width_m / 2.0
    leg_half_height = leg_height / 2.0
    x_offset = half_size[0] - config.table.leg_edge_inset_m - leg_half_width
    y_offset = half_size[1] - config.table.leg_edge_inset_m - leg_half_width
    for x_sign, y_sign, suffix in (
        (-1.0, -1.0, "x_min_y_min"),
        (-1.0, 1.0, "x_min_y_max"),
        (1.0, -1.0, "x_max_y_min"),
        (1.0, 1.0, "x_max_y_max"),
    ):
        world.add_geom(
            name=f"table_leg_{suffix}",
            type=_enum_value(mujoco, "mjtGeom", "mjGEOM_BOX"),
            pos=[
                config.table.center_m[0] + x_sign * x_offset,
                config.table.center_m[1] + y_sign * y_offset,
                config.floor.z_m + leg_half_height,
            ],
            size=[leg_half_width, leg_half_width, leg_half_height],
            rgba=config.table.color_rgba,
            friction=config.table.friction,
            contype=1,
            conaffinity=1,
        )
    pedestal_vertices, pedestal_faces = _pedestal_mesh_data(config)
    spec.add_mesh(
        name="arm_pedestal_frustum_mesh",
        uservert=pedestal_vertices,
        userface=pedestal_faces,
    )
    world.add_geom(
        name="arm_pedestal",
        type=_enum_value(mujoco, "mjtGeom", "mjGEOM_MESH"),
        pos=config.arm_pedestal.center_m,
        meshname="arm_pedestal_frustum_mesh",
        rgba=config.arm_pedestal.color_rgba,
        friction=config.arm_pedestal.friction,
        contype=1,
        conaffinity=1,
    )
    world.add_site(
        name="workspace_center",
        type=_enum_value(mujoco, "mjtGeom", "mjGEOM_SPHERE"),
        pos=config.workspace_center_m,
        size=[0.012],
        rgba=[1.0, 0.62, 0.08, 0.9],
        group=4,
    )
    world.add_camera(
        name="overview",
        pos=config.camera.eye_m,
        xyaxes=_camera_xyaxes(config.camera.eye_m, config.camera.target_m),
        fovy=config.camera.fovy_deg,
    )
    world.add_light(
        name="observation_light",
        type=_enum_value(mujoco, "mjtLightType", "mjLIGHT_DIRECTIONAL"),
        pos=[0.0, 0.0, 2.5],
        dir=config.observation_light.direction,
        diffuse=config.observation_light.diffuse_rgb,
        ambient=config.observation_light.ambient_rgb,
        specular=config.observation_light.specular_rgb,
        castshadow=int(config.observation_light.cast_shadow),
    )


def _resolve_asset(project_root: Path, relative_path: Path, label: str) -> Path:
    path = (project_root / relative_path).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"{label} not found: {path}")
    return path


def _resolve_asset_directory(project_root: Path, relative_path: Path, label: str) -> Path:
    path = (project_root / relative_path).resolve()
    if not path.is_dir():
        raise FileNotFoundError(f"{label} not found: {path}")
    return path


def build_mujoco_fr3_hand2_model(
    config: MujocoTableSceneConfig,
    arm_profile: Fr3ModelProfile,
    hand_profile: Hand2ModelProfile,
    *,
    project_root: str | Path,
) -> Any:
    """Verify the pinned assets, compose them in memory, and compile one model."""

    mujoco = _require_mujoco()
    root = Path(project_root).resolve()
    arm_path = _resolve_asset(root, config.assets.arm_mjcf, "FR3 v2 MJCF")
    hand_path = _resolve_asset(root, config.assets.hand_mjcf, "Wuji Hand 2 MJCF")
    arm_asset_dir = _resolve_asset_directory(
        root, config.assets.arm_asset_dir, "FR3 v2 mesh directory"
    )
    hand_asset_dir = _resolve_asset_directory(
        root, config.assets.hand_asset_dir, "Wuji Hand 2 mesh directory"
    )
    for label, path, expected in (
        ("FR3 v2 MJCF", arm_path, config.assets.arm_mjcf_sha256),
        ("Wuji Hand 2 MJCF", hand_path, config.assets.hand_mjcf_sha256),
    ):
        actual = sha256_file(path)
        if actual != expected:
            raise RuntimeError(f"{label} SHA-256 mismatch: expected {expected}, got {actual}")
    for label, directory, expected in (
        ("FR3 v2 mesh tree", arm_asset_dir, config.assets.arm_asset_tree_sha256),
        ("Wuji Hand 2 mesh tree", hand_asset_dir, config.assets.hand_asset_tree_sha256),
    ):
        actual = sha256_tree(directory)
        if actual != expected:
            raise RuntimeError(f"{label} SHA-256 mismatch: expected {expected}, got {actual}")
    if arm_profile.provenance.get("mjcf_sha256") != config.assets.arm_mjcf_sha256:
        raise RuntimeError("arm profile and scene disagree on FR3 v2 MJCF provenance")
    if hand_profile.provenance.get("mjcf_sha256") != config.assets.hand_mjcf_sha256:
        raise RuntimeError("hand profile and scene disagree on Hand 2 MJCF provenance")
    if arm_profile.provenance.get("asset_tree_sha256") != config.assets.arm_asset_tree_sha256:
        raise RuntimeError("arm profile and scene disagree on FR3 v2 mesh provenance")
    if (
        hand_profile.provenance.get("mjcf_asset_tree_sha256")
        != config.assets.hand_asset_tree_sha256
    ):
        raise RuntimeError("hand profile and scene disagree on Hand 2 mesh provenance")
    provenance_checks = (
        ("arm_repository", arm_profile.provenance, "repository"),
        ("arm_commit", arm_profile.provenance, "commit"),
        ("hand_repository", hand_profile.provenance, "repository"),
        ("hand_tag", hand_profile.provenance, "tag"),
        ("hand_commit", hand_profile.provenance, "commit"),
    )
    for scene_key, profile_provenance, profile_key in provenance_checks:
        if config.provenance.get(scene_key) != profile_provenance.get(profile_key):
            raise RuntimeError(f"scene and model profile disagree on provenance: {scene_key}")

    arm_spec = mujoco.MjSpec.from_file(str(arm_path))
    hand_spec = mujoco.MjSpec.from_file(str(hand_path))
    arm_spec.modelname = config.name
    arm_spec.copy_during_attach = True
    _apply_global_options(mujoco, arm_spec, config)
    _apply_global_options(mujoco, hand_spec, config)
    base = arm_spec.body(arm_profile.base_body_name)
    if base is None:
        raise RuntimeError(f"FR3 base body missing: {arm_profile.base_body_name}")
    base.pos = config.arm_mount.position_m
    base.quat = config.arm_mount.quat_wxyz
    flange = arm_spec.body(config.hand_attachment.parent_body)
    if flange is None:
        raise RuntimeError(f"FR3 flange body missing: {config.hand_attachment.parent_body}")
    mount = flange.add_frame(
        name="fr3_flange_to_hand2",
        pos=config.hand_attachment.position_m,
        quat=config.hand_attachment.quat_wxyz,
    )
    # Empty prefix is intentional: current upstream names are disjoint and the
    # Hand 2 canonical q20 contract must remain exact.  We never call to_xml()
    # because attached specs with separate asset roots do not round-trip safely.
    arm_spec.attach(hand_spec, frame=mount, prefix="", suffix="")
    _author_environment(mujoco, arm_spec, config)
    return arm_spec.compile()


def _binding_for_names(mujoco: ModuleType, model: Any, names: Sequence[str]) -> MujocoJointBinding:
    joint_ids: list[int] = []
    qpos_addresses: list[int] = []
    dof_addresses: list[int] = []
    actuator_ids: list[int] = []
    for name in names:
        joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
        if joint_id < 0:
            raise RuntimeError(f"joint missing from compiled MuJoCo model: {name}")
        if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_HINGE):
            raise RuntimeError(f"joint is not a one-DoF hinge: {name}")
        matches = [
            actuator_id
            for actuator_id in range(model.nu)
            if int(model.actuator_trntype[actuator_id]) == int(mujoco.mjtTrn.mjTRN_JOINT)
            and int(model.actuator_trnid[actuator_id, 0]) == joint_id
        ]
        if len(matches) != 1:
            raise RuntimeError(f"joint {name} must have exactly one direct actuator, got {matches}")
        joint_ids.append(joint_id)
        qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
        dof_addresses.append(int(model.jnt_dofadr[joint_id]))
        actuator_ids.append(matches[0])
    return MujocoJointBinding(
        names=tuple(names),
        joint_ids=tuple(joint_ids),
        qpos_addresses=tuple(qpos_addresses),
        dof_addresses=tuple(dof_addresses),
        actuator_ids=tuple(actuator_ids),
    )


def _validate_partition_ranges(
    model: Any,
    binding: MujocoJointBinding,
    lower: Sequence[float],
    upper: Sequence[float],
    label: str,
) -> None:
    actual = np.asarray(model.jnt_range[np.asarray(binding.joint_ids)], dtype=np.float64)
    expected = np.column_stack((lower, upper))
    if not np.allclose(actual, expected, rtol=0.0, atol=1e-9):
        raise RuntimeError(f"compiled {label} joint ranges differ from the pinned profile")
    ctrl = np.asarray(model.actuator_ctrlrange[np.asarray(binding.actuator_ids)], dtype=np.float64)
    if not np.allclose(ctrl, expected, rtol=0.0, atol=1e-9):
        raise RuntimeError(f"compiled {label} actuator ranges differ from the pinned profile")


class MujocoFr3Hand2:
    """Strict 7+20 position-control facade over the combined MuJoCo model."""

    def __init__(
        self,
        model: Any,
        config: MujocoTableSceneConfig,
        arm_profile: Fr3ModelProfile,
        hand_profile: Hand2ModelProfile,
        *,
        arm_contract: GroupBindingSpec | None = None,
        hand_contract: GroupBindingSpec | None = None,
    ) -> None:
        self._mujoco = _require_mujoco()
        self.model = model
        self.data = self._mujoco.MjData(model)
        self.config = config
        self.arm_profile = arm_profile
        self.hand_profile = hand_profile
        self.arm_contract = arm_contract
        self.hand_contract = hand_contract
        self.arm = _binding_for_names(self._mujoco, model, arm_profile.names)
        self.hand = _binding_for_names(self._mujoco, model, hand_profile.layout.names)
        self._validate_contract()
        self._flange_body_id = self._body_id(config.hand_attachment.parent_body)
        self._palm_body_id = self._body_id(config.hand_attachment.child_body)
        self._fingertip_site_ids = tuple(self._site_id(name) for name in FINGERTIP_SITE_NAMES)
        self.reset()

    @classmethod
    def from_config(
        cls,
        config: MujocoTableSceneConfig,
        *,
        project_root: str | Path,
        arm_contract: GroupBindingSpec | None = None,
        hand_contract: GroupBindingSpec | None = None,
    ) -> MujocoFr3Hand2:
        root = Path(project_root).resolve()
        arm_profile_path = _resolve_asset(root, config.assets.arm_profile, "FR3 profile")
        hand_profile_path = _resolve_asset(root, config.assets.hand_profile, "Hand 2 profile")
        arm_profile = load_fr3_model_profile(arm_profile_path)
        hand_profile = load_hand2_model_profile(hand_profile_path)
        model = build_mujoco_fr3_hand2_model(
            config, arm_profile, hand_profile, project_root=root
        )
        return cls(
            model,
            config,
            arm_profile,
            hand_profile,
            arm_contract=arm_contract,
            hand_contract=hand_contract,
        )

    def _body_id(self, name: str) -> int:
        body_id = int(
            self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_BODY, name)
        )
        if body_id < 0:
            raise RuntimeError(f"body missing from compiled model: {name}")
        return body_id

    def _site_id(self, name: str) -> int:
        site_id = int(
            self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_SITE, name)
        )
        if site_id < 0:
            raise RuntimeError(f"site missing from compiled model: {name}")
        return site_id

    def _validate_contract(self) -> None:
        expected_names = (*self.arm_profile.names, *self.hand_profile.layout.names)
        actual_names = tuple(
            self._mujoco.mj_id2name(self.model, self._mujoco.mjtObj.mjOBJ_JOINT, index)
            for index in range(self.model.njnt)
        )
        expected_dofs = len(expected_names)
        if (self.model.nq, self.model.nv, self.model.nu) != (
            expected_dofs,
            expected_dofs,
            expected_dofs,
        ):
            raise RuntimeError(
                "combined model qpos, velocity, and actuator counts must match "
                "the resolved arm + hand profiles"
            )
        if actual_names != expected_names:
            raise RuntimeError("compiled joint order differs from arm_q7 + canonical hand_q20")
        self._validate_binding_contract(
            self.arm,
            self.arm_contract,
            self.arm_profile.names,
            "FR3 v2",
        )
        self._validate_binding_contract(
            self.hand,
            self.hand_contract,
            self.hand_profile.layout.names,
            "Hand 2",
        )
        _validate_partition_ranges(
            self.model,
            self.arm,
            self.arm_profile.lower,
            self.arm_profile.upper,
            "FR3 v2",
        )
        _validate_partition_ranges(
            self.model,
            self.hand,
            self.hand_profile.layout.lower,
            self.hand_profile.layout.upper,
            "Hand 2",
        )
        base_id = self._body_id(self.arm_profile.base_body_name)
        if not np.allclose(
            self.model.body_pos[base_id], self.config.arm_mount.position_m, atol=1e-12
        ) or not np.allclose(
            self.model.body_quat[base_id], self.config.arm_mount.quat_wxyz, atol=1e-12
        ):
            raise RuntimeError("compiled FR3 base pose differs from the pedestal mount config")
        flange_id = self._body_id(self.config.hand_attachment.parent_body)
        palm_id = self._body_id(self.config.hand_attachment.child_body)
        if int(self.model.body_parentid[palm_id]) != flange_id:
            raise RuntimeError("Hand 2 root is not rigidly parented by the FR3 flange body")
        if not np.allclose(
            self.model.body_pos[palm_id], self.config.hand_attachment.position_m, atol=1e-12
        ) or not np.allclose(
            self.model.body_quat[palm_id], self.config.hand_attachment.quat_wxyz, atol=1e-12
        ):
            raise RuntimeError("compiled flange-to-hand transform differs from scene config")
        if not np.isclose(self.model.opt.timestep, self.config.physics.timestep_s):
            raise RuntimeError("compiled MuJoCo timestep differs from scene config")
        expected_home = np.concatenate(
            (self.arm_profile.home_position, self.hand_profile.rest_position)
        )
        if self.model.nkey != 1 or not np.allclose(
            self.model.key_qpos[0], expected_home, rtol=0.0, atol=1e-9
        ):
            raise RuntimeError(
                "upstream home keyframe did not extend to the resolved combined home"
            )
        self._mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self._mujoco.mj_forward(self.model, self.data)
        base_vertices = _mesh_geom_world_vertices(
            self._mujoco, self.model, self.data, "fr3v2_link0_collision"
        )
        pedestal_top_center = np.asarray(self.config.arm_pedestal.center_xy_m)
        pedestal_top_half_size = np.asarray(self.config.arm_pedestal.top_size_m) / 2.0
        if np.any(
            base_vertices[:, :2].min(axis=0)
            < pedestal_top_center - pedestal_top_half_size - 1e-9
        ) or np.any(
            base_vertices[:, :2].max(axis=0)
            > pedestal_top_center + pedestal_top_half_size + 1e-9
        ):
            raise RuntimeError("FR3 base footprint extends outside the pedestal top")
        joint2_z_m = float(self.data.xanchor[self.arm.joint_ids[1], 2])
        actual_clearance_m = joint2_z_m - self.config.table.top_z_m
        if not np.isclose(
            actual_clearance_m,
            self.config.arm_mount.joint2_clearance_above_table_m,
            rtol=0.0,
            atol=1e-9,
        ):
            raise RuntimeError(
                "compiled FR3 joint2 height differs from the configured tabletop clearance"
            )

    def _validate_binding_contract(
        self,
        binding: MujocoJointBinding,
        contract: GroupBindingSpec | None,
        profile_names: Sequence[str],
        label: str,
    ) -> None:
        if contract is None:
            return
        if contract.joints != tuple(profile_names):
            raise RuntimeError(
                f"{label} Backend Binding joint order differs from its model profile"
            )
        actuator_names = tuple(
            self._mujoco.mj_id2name(
                self.model,
                self._mujoco.mjtObj.mjOBJ_ACTUATOR,
                actuator_id,
            )
            for actuator_id in binding.actuator_ids
        )
        if actuator_names != contract.actuators:
            raise RuntimeError(
                f"{label} compiled actuator names differ from its Backend Binding"
            )

    @staticmethod
    def _strict_position(
        values: Sequence[float] | npt.NDArray[np.floating[Any]],
        lower: Sequence[float],
        upper: Sequence[float],
        label: str,
    ) -> FloatArray:
        array = np.asarray(values, dtype=np.float64)
        if array.shape != (len(lower),):
            raise ValueError(f"{label} must have shape {(len(lower),)}, got {array.shape}")
        if not np.isfinite(array).all():
            raise ValueError(f"{label} contains NaN or infinity")
        if np.any(array < np.asarray(lower)) or np.any(array > np.asarray(upper)):
            raise ValueError(f"{label} exceeds its pinned joint range")
        return array

    def reset(self) -> MujocoFr3Hand2State:
        """Reset deterministically to the explicit FR3 home + Hand 2 q20 rest."""

        self._mujoco.mj_resetData(self.model, self.data)
        arm = self.arm_profile.validate_position(self.arm_profile.home_position)
        hand = self._strict_position(
            self.hand_profile.rest_position,
            self.hand_profile.layout.lower,
            self.hand_profile.layout.upper,
            "hand_q20",
        )
        self.data.qpos[np.asarray(self.arm.qpos_addresses)] = arm
        self.data.qpos[np.asarray(self.hand.qpos_addresses)] = hand
        self.data.ctrl[np.asarray(self.arm.actuator_ids)] = arm
        self.data.ctrl[np.asarray(self.hand.actuator_ids)] = hand
        self._mujoco.mj_forward(self.model, self.data)
        self._assert_finite()
        return self.observe()

    def set_joint_targets(
        self,
        arm_q7: Sequence[float] | npt.NDArray[np.floating[Any]],
        hand_q20: Sequence[float] | npt.NDArray[np.floating[Any]],
    ) -> None:
        """Set two named target partitions; range violations fail closed."""

        arm = self.arm_profile.validate_position(arm_q7)
        hand = self._strict_position(
            hand_q20,
            self.hand_profile.layout.lower,
            self.hand_profile.layout.upper,
            "hand_q20",
        )
        self.data.ctrl[np.asarray(self.arm.actuator_ids)] = arm
        self.data.ctrl[np.asarray(self.hand.actuator_ids)] = hand

    def step(self, count: int = 1) -> MujocoFr3Hand2State:
        """Advance ``count`` 100 Hz control ticks using configured physics substeps."""

        if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
            raise ValueError("count must be a positive integer")
        for _ in range(count * self.config.control.physics_substeps):
            self._mujoco.mj_step(self.model, self.data)
        self._assert_finite()
        return self.observe()

    def _assert_finite(self) -> None:
        if not (
            np.isfinite(self.data.qpos).all()
            and np.isfinite(self.data.qvel).all()
            and np.isfinite(self.data.ctrl).all()
            and np.isfinite(self.data.time)
        ):
            raise RuntimeError("MuJoCo state became non-finite")

    def observe(self) -> MujocoFr3Hand2State:
        return MujocoFr3Hand2State(
            arm_q7=np.asarray(self.data.qpos[np.asarray(self.arm.qpos_addresses)]).copy(),
            arm_dq7=np.asarray(self.data.qvel[np.asarray(self.arm.dof_addresses)]).copy(),
            hand_q20=np.asarray(self.data.qpos[np.asarray(self.hand.qpos_addresses)]).copy(),
            hand_dq20=np.asarray(self.data.qvel[np.asarray(self.hand.dof_addresses)]).copy(),
            flange_position_m=np.asarray(self.data.xpos[self._flange_body_id]).copy(),
            flange_quat_wxyz=np.asarray(self.data.xquat[self._flange_body_id]).copy(),
            palm_position_m=np.asarray(self.data.xpos[self._palm_body_id]).copy(),
            palm_quat_wxyz=np.asarray(self.data.xquat[self._palm_body_id]).copy(),
            fingertip_positions_m=np.asarray(
                self.data.site_xpos[np.asarray(self._fingertip_site_ids)]
            ).copy(),
            contact_count=int(self.data.ncon),
            simulation_time_s=float(self.data.time),
        )

    def render(
        self, *, width: int | None = None, height: int | None = None
    ) -> np.ndarray[Any, np.dtype[np.uint8]]:
        """Render the named overview camera; the caller chooses the GL backend."""

        width = self.config.camera.width_px if width is None else width
        height = self.config.camera.height_px if height is None else height
        if width <= 0 or height <= 0:
            raise ValueError("render dimensions must be positive")
        renderer = self._mujoco.Renderer(self.model, height=height, width=width)
        try:
            renderer.update_scene(self.data, camera="overview")
            return np.asarray(renderer.render()).copy()
        finally:
            renderer.close()


__all__ = [
    "FINGERTIP_SITE_NAMES",
    "MujocoFr3Hand2",
    "MujocoFr3Hand2State",
    "MujocoJointBinding",
    "build_mujoco_fr3_hand2_model",
    "sha256_file",
    "sha256_tree",
]
