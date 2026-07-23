from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from wujihand.adapters.simulation import FINGERTIP_SITE_NAMES, MujocoFr3Hand2
from wujihand.domain import HAND2_RIGHT_LAYOUT
from wujihand.runtime import load_mujoco_table_scene_config


mujoco = pytest.importorskip("mujoco")

ROOT = Path(__file__).parents[2]
SCENE = ROOT / "configs/base/mujoco_fr3v2_hand2_right_table_v1.yaml"
ARM_MJCF = ROOT / "third_party/src/mujoco_menagerie/franka_fr3_v2/fr3v2.xml"
HAND_MJCF = ROOT / "third_party/src/wuji-description/hand2_beta/body/mjcf/right.xml"

pytestmark = [pytest.mark.requires_mujoco, pytest.mark.requires_upstream_asset]


@pytest.fixture(scope="module")
def environment() -> MujocoFr3Hand2:
    if not ARM_MJCF.is_file() or not HAND_MJCF.is_file():
        pytest.skip("restore pinned MJCF assets from third_party/sources.lock.yaml")
    config = load_mujoco_table_scene_config(SCENE)
    return MujocoFr3Hand2.from_config(config, project_root=ROOT)


def test_combined_model_is_strict_arm7_plus_canonical_hand20(
    environment: MujocoFr3Hand2,
) -> None:
    model = environment.model

    assert (model.nq, model.nv, model.nu, model.njnt) == (27, 27, 27, 27)
    assert model.nexclude == 32
    assert environment.arm.names == tuple(f"fr3v2_joint{index}" for index in range(1, 8))
    assert environment.hand.names == HAND2_RIGHT_LAYOUT.names
    assert len(set((*environment.arm.actuator_ids, *environment.hand.actuator_ids))) == 27
    assert all(
        int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_HINGE)
        for joint_id in (*environment.arm.joint_ids, *environment.hand.joint_ids)
    )


def test_attachment_and_global_physics_match_scene_config(
    environment: MujocoFr3Hand2,
) -> None:
    model = environment.model
    config = environment.config
    flange_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "fr3v2_link8")
    palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "r_base_link")

    assert model.body_parentid[palm_id] == flange_id
    np.testing.assert_allclose(model.body_pos[palm_id], config.hand_attachment.position_m)
    np.testing.assert_allclose(model.body_quat[palm_id], config.hand_attachment.quat_wxyz)
    assert model.opt.timestep == pytest.approx(0.002)
    assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    assert model.opt.solver == mujoco.mjtSolver.mjSOL_NEWTON
    assert model.opt.jacobian == mujoco.mjtJacobian.mjJAC_SPARSE
    assert model.opt.iterations == 100
    environment.reset()
    joint2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "fr3v2_joint2")
    joint2_clearance_m = (
        environment.data.xanchor[joint2_id, 2] - environment.config.table.top_z_m
    )
    assert joint2_clearance_m == pytest.approx(
        environment.config.arm_mount.joint2_clearance_above_table_m
    )


def test_environment_and_fingertip_kinematic_markers_exist(
    environment: MujocoFr3Hand2,
) -> None:
    model = environment.model
    geom_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        for index in range(model.ngeom)
    }
    site_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, index)
        for index in range(model.nsite)
    }

    assert {
        "floor",
        "tabletop",
        "table_leg_x_min_y_max",
        "table_leg_x_max_y_min",
        "arm_pedestal",
    } <= geom_names
    assert set(FINGERTIP_SITE_NAMES) <= site_names
    assert "workspace_center" in site_names

    pedestal_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "arm_pedestal")
    assert model.geom_type[pedestal_id] == mujoco.mjtGeom.mjGEOM_MESH
    mesh_id = int(model.geom_dataid[pedestal_id])
    first_vertex = int(model.mesh_vertadr[mesh_id])
    vertex_count = int(model.mesh_vertnum[mesh_id])
    assert vertex_count == 8
    assert model.mesh_facenum[mesh_id] == 12
    local_vertices = model.mesh_vert[first_vertex : first_vertex + vertex_count]
    rotation = environment.data.geom_xmat[pedestal_id].reshape(3, 3)
    world_vertices = environment.data.geom_xpos[pedestal_id] + local_vertices @ rotation.T
    bottom_half_size = np.asarray(environment.config.arm_pedestal.bottom_size_m) / 2.0
    np.testing.assert_allclose(
        world_vertices[:, :2].min(axis=0),
        np.asarray(environment.config.arm_pedestal.center_xy_m) - bottom_half_size,
        atol=2e-8,
    )
    np.testing.assert_allclose(
        world_vertices[:, :2].max(axis=0),
        np.asarray(environment.config.arm_pedestal.center_xy_m) + bottom_half_size,
        atol=2e-8,
    )
    np.testing.assert_allclose(
        world_vertices[:, 2].min(), environment.config.floor.z_m, atol=2e-8
    )
    np.testing.assert_allclose(
        world_vertices[:, 2].max(), environment.config.arm_pedestal.top_z_m, atol=2e-8
    )
    top_vertices = world_vertices[
        np.isclose(world_vertices[:, 2], environment.config.arm_pedestal.top_z_m)
    ]
    np.testing.assert_allclose(
        np.ptp(top_vertices[:, :2], axis=0),
        environment.config.arm_pedestal.top_size_m,
        atol=2e-8,
    )


def test_observation_light_is_single_directional_and_shadow_free(
    environment: MujocoFr3Hand2,
) -> None:
    model = environment.model
    config = environment.config.observation_light

    assert model.nlight == 1
    assert model.vis.headlight.active == 0
    light_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_LIGHT, "observation_light"
    )
    assert light_id == 0
    assert model.light_type[light_id] == mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    assert not model.light_castshadow[light_id]
    np.testing.assert_allclose(model.light_dir[light_id], config.direction)
    np.testing.assert_allclose(model.light_ambient[light_id], config.ambient_rgb)
    np.testing.assert_allclose(model.light_diffuse[light_id], config.diffuse_rgb)
    np.testing.assert_allclose(model.light_specular[light_id], config.specular_rgb)


def test_compiled_contract_rejects_pedestal_top_smaller_than_fr3_base() -> None:
    config = load_mujoco_table_scene_config(SCENE)
    undersized = replace(
        config,
        arm_pedestal=replace(config.arm_pedestal, top_size_m=(0.10, 0.10)),
    )

    with pytest.raises(RuntimeError, match="base footprint extends outside"):
        MujocoFr3Hand2.from_config(undersized, project_root=ROOT)


def test_target_writes_are_partitioned_by_name_derived_actuator_ids(
    environment: MujocoFr3Hand2,
) -> None:
    environment.reset()
    arm_target = environment.arm_profile.home_position.copy()
    arm_target[0] += 0.05
    hand_target = environment.hand_profile.rest_position.copy()
    hand_target[4] += 0.10

    environment.set_joint_targets(arm_target, hand_target)

    np.testing.assert_allclose(
        environment.data.ctrl[np.asarray(environment.arm.actuator_ids)], arm_target
    )
    np.testing.assert_allclose(
        environment.data.ctrl[np.asarray(environment.hand.actuator_ids)], hand_target
    )
    with pytest.raises(ValueError, match="joint range"):
        environment.set_joint_targets([0.0] * 7, hand_target)
    with pytest.raises(ValueError, match="hand_q20"):
        environment.set_joint_targets(arm_target, [0.0] * 19)
