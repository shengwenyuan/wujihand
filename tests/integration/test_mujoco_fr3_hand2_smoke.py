from __future__ import annotations

from pathlib import Path
import json
import subprocess
import sys

import numpy as np
import pytest

from wujihand.adapters.simulation import MujocoFr3Hand2
from wujihand.runtime import load_mujoco_table_scene_config


mujoco = pytest.importorskip("mujoco")

ROOT = Path(__file__).parents[2]
SCENE = ROOT / "configs/base/mujoco_fr3v2_hand2_right_table_v2026_6_27_v1.yaml"
ARM_MJCF = ROOT / "third_party/src/mujoco_menagerie/franka_fr3_v2/fr3v2.xml"
HAND_MJCF = (
    ROOT
    / "third_party/src/wuji-description/v2026.6.27/hand2_beta/body/mjcf/right.xml"
)

pytestmark = [pytest.mark.requires_mujoco, pytest.mark.requires_upstream_asset]


@pytest.fixture(scope="module")
def environment() -> MujocoFr3Hand2:
    if not ARM_MJCF.is_file() or not HAND_MJCF.is_file():
        pytest.skip("restore pinned MJCF assets from third_party/sources.lock.yaml")
    config = load_mujoco_table_scene_config(SCENE)
    return MujocoFr3Hand2.from_config(config, project_root=ROOT)


def test_reset_is_deterministic_and_starts_clear_of_table(
    environment: MujocoFr3Hand2,
) -> None:
    first = environment.reset()
    environment.step(25)
    second = environment.reset()

    np.testing.assert_array_equal(second.arm_q7, first.arm_q7)
    np.testing.assert_array_equal(second.hand_q20, first.hand_q20)
    np.testing.assert_array_equal(second.flange_position_m, first.flange_position_m)
    np.testing.assert_array_equal(second.palm_position_m, first.palm_position_m)
    np.testing.assert_array_equal(second.fingertip_positions_m, first.fingertip_positions_m)
    assert first.contact_count == 0
    assert np.min(first.fingertip_positions_m[:, 2]) > environment.config.table.top_z_m


def test_home_hold_is_finite_for_ten_seconds_and_fixed_mount_does_not_drift(
    environment: MujocoFr3Hand2,
) -> None:
    initial = environment.reset()
    relative_position = initial.palm_position_m - initial.flange_position_m
    max_relative_drift_m = 0.0

    for _ in range(1000):
        state = environment.step()
        max_relative_drift_m = max(
            max_relative_drift_m,
            float(
                np.linalg.norm(
                    (state.palm_position_m - state.flange_position_m) - relative_position
                )
            ),
        )

    assert state.simulation_time_s == pytest.approx(10.0)
    assert np.isfinite(
        np.concatenate(
            (
                state.arm_q7,
                state.arm_dq7,
                state.hand_q20,
                state.hand_dq20,
                state.fingertip_positions_m.ravel(),
            )
        )
    ).all()
    assert max_relative_drift_m < 1e-12
    assert np.max(np.abs(state.arm_dq7)) < 1e-9
    assert np.max(np.abs(state.hand_dq20)) < 1e-9
    assert state.contact_count == 0


def test_arm_and_hand_each_respond_to_small_position_steps(
    environment: MujocoFr3Hand2,
) -> None:
    initial = environment.reset()
    arm_target = environment.arm_profile.home_position.copy()
    arm_target[0] += 0.05
    environment.set_joint_targets(arm_target, environment.hand_profile.rest_position)
    arm_state = environment.step(50)
    assert arm_state.arm_q7[0] - initial.arm_q7[0] > 0.03

    initial = environment.reset()
    hand_target = environment.hand_profile.rest_position.copy()
    hand_target[4] += 0.10
    environment.set_joint_targets(environment.arm_profile.home_position, hand_target)
    hand_state = environment.step(50)
    assert hand_state.hand_q20[4] - initial.hand_q20[4] > 0.05


def test_smooth_reach_to_table_contact_remains_finite(
    environment: MujocoFr3Hand2,
) -> None:
    environment.reset()
    home = environment.arm_profile.home_position.copy()
    # Deterministic, seed-derived reach pose used only as a rigid-contact probe.
    contact_q7 = np.asarray(
        [
            0.07365736,
            0.13810905,
            -0.05131673,
            -1.72600727,
            -0.15936100,
            1.75886180,
            -0.64464427,
        ]
    )
    tabletop_id = mujoco.mj_name2id(
        environment.model, mujoco.mjtObj.mjOBJ_GEOM, "tabletop"
    )
    contact_observed = False
    hand_table_contact_observed = False
    non_hand_table_contact_observed = False
    minimum_distance_m = 0.0

    for index in range(300):
        phase = (index + 1) / 300.0
        blend = phase * phase * (3.0 - 2.0 * phase)
        environment.set_joint_targets(
            home + blend * (contact_q7 - home), environment.hand_profile.rest_position
        )
        state = environment.step()
        for contact_index in range(environment.data.ncon):
            contact = environment.data.contact[contact_index]
            if tabletop_id in (int(contact.geom1), int(contact.geom2)):
                contact_observed = True
                minimum_distance_m = min(minimum_distance_m, float(contact.dist))
                other_geom = (
                    int(contact.geom2) if int(contact.geom1) == tabletop_id else int(contact.geom1)
                )
                other_body = int(environment.model.geom_bodyid[other_geom])
                other_body_name = mujoco.mj_id2name(
                    environment.model, mujoco.mjtObj.mjOBJ_BODY, other_body
                )
                is_hand_body = bool(other_body_name and other_body_name.startswith("r_"))
                hand_table_contact_observed |= is_hand_body
                non_hand_table_contact_observed |= not is_hand_body
    for _ in range(300):
        state = environment.step()
        for contact_index in range(environment.data.ncon):
            contact = environment.data.contact[contact_index]
            if tabletop_id in (int(contact.geom1), int(contact.geom2)):
                contact_observed = True
                minimum_distance_m = min(minimum_distance_m, float(contact.dist))
                other_geom = (
                    int(contact.geom2) if int(contact.geom1) == tabletop_id else int(contact.geom1)
                )
                other_body = int(environment.model.geom_bodyid[other_geom])
                other_body_name = mujoco.mj_id2name(
                    environment.model, mujoco.mjtObj.mjOBJ_BODY, other_body
                )
                is_hand_body = bool(other_body_name and other_body_name.startswith("r_"))
                hand_table_contact_observed |= is_hand_body
                non_hand_table_contact_observed |= not is_hand_body

    assert contact_observed
    assert hand_table_contact_observed
    assert not non_hand_table_contact_observed
    assert minimum_distance_m > -0.002
    final_hand_table_contact = False
    for contact_index in range(environment.data.ncon):
        contact = environment.data.contact[contact_index]
        if tabletop_id not in (int(contact.geom1), int(contact.geom2)):
            continue
        other_geom = (
            int(contact.geom2) if int(contact.geom1) == tabletop_id else int(contact.geom1)
        )
        other_body = int(environment.model.geom_bodyid[other_geom])
        other_body_name = mujoco.mj_id2name(
            environment.model, mujoco.mjtObj.mjOBJ_BODY, other_body
        )
        final_hand_table_contact |= bool(
            other_body_name and other_body_name.startswith("r_")
        )
    assert final_hand_table_contact
    assert np.isfinite(
        np.concatenate((state.arm_q7, state.arm_dq7, state.hand_q20, state.hand_dq20))
    ).all()
    assert np.max(np.abs(np.concatenate((state.arm_dq7, state.hand_dq20)))) < 0.01


def test_headless_runner_emits_reproducible_report() -> None:
    if not ARM_MJCF.is_file() or not HAND_MJCF.is_file():
        pytest.skip("restore pinned MJCF assets from third_party/sources.lock.yaml")
    completed = subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools/run_mujoco_fr3_hand2_table.py"),
            "--duration-s",
            "0.1",
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    report = json.loads(completed.stdout)

    assert report["mujoco_version"] == "3.10.0"
    assert report["dimensions"] == {"nq": 27, "nu": 27, "nv": 27}
    assert report["finite"] is True
    assert report["scene_geometry"]["table_size_m"] == [1.6, 1.0, 0.06]
    assert report["scene_geometry"]["pedestal_top_z_m"] == pytest.approx(0.47)
    assert report["scene_geometry"]["joint2_clearance_above_table_m"] == pytest.approx(
        0.053
    )
    assert (
        report["assets"]["arm_asset_tree_sha256"]
        == "0ee1f659bb749fb88c1c2cca93215ec42460de9626eab31018bbc7bf1c88d7bf"
    )
    assert (
        report["assets"]["hand_asset_tree_sha256"]
        == "4f1a7e96cafb13403ed82c5ef2f18d52a40afb49776ce56ee8f2224280ffcc13"
    )
    assert report["attachment"]["relative_position_drift_m"] < 1e-12
