from __future__ import annotations

from pathlib import Path

import pytest

from wujihand.dataset import (
    Q54JointProfile,
    load_q54_joint_profile,
    parse_dataset_truth_inventories,
    validate_q54_runtime_inventory,
    validate_state_truth_inventory,
)
from wujihand.domain.dataset_recording import (
    DynamicRigidBodyTruth,
    KinematicLinkTruth,
    SimulationFramePhase,
    SimulationStateFrame,
)


ROOT = Path(__file__).parents[2]
PROFILE = "configs/profiles/isaac_nero_hand2_q54_dataset_v1.yaml"
LINK_IDS = (
    "arm_link7",
    "palm",
    "thumb_tip",
    "index_finger_tip",
    "middle_finger_tip",
    "ring_finger_tip",
    "pinky_tip",
)


def _runtime_inventory() -> tuple[Q54JointProfile, dict[str, object]]:
    profile = load_q54_joint_profile(ROOT, PROFILE)
    runtime_names: dict[str, tuple[str, ...]] = {}
    runtime_limits: list[list[float]] = []
    for side in ("left", "right"):
        joints = tuple(item for item in profile.joints if item.side == side)
        runtime_names[side] = tuple(
            next(
                item.source_joint_name
                for item in joints
                if item.source_index_q27 == index
            )
            for index in range(27)
        )
    for item in profile.joints:
        runtime_limits.append(
            [
                item.lower_rad - item.zero_offset_rad,
                item.upper_rad - item.zero_offset_rad,
            ]
            if item.sign == 1
            else [
                item.zero_offset_rad - item.upper_rad,
                item.zero_offset_rad - item.lower_rad,
            ]
        )
    inventory: dict[str, object] = {
        "schema": "wujihand.q54_runtime_inventory.v1",
        "profile_id": profile.profile_id,
        "profile_sha256": profile.file_sha256,
        "canonical_names": list(profile.canonical_names),
        "left_runtime_names": list(runtime_names["left"]),
        "right_runtime_names": list(runtime_names["right"]),
        "canonical_source_indices": [item.source_index_q27 for item in profile.joints],
        "runtime_limits_rad": runtime_limits,
    }
    return profile, inventory


def _manifest_inventories() -> dict[str, object]:
    return {
        "dynamic_object_inventory": {"banana": "/World/Workcell/banana"},
        "kinematic_link_inventory": [
            {
                "side": side,
                "logical_link_id": logical_id,
                "prim_path": f"/World/Robot/{side}/{logical_id}",
            }
            for side in ("left", "right")
            for logical_id in LINK_IDS
        ],
    }


def _state(*, banana_kinematic: bool = False, missing_link: bool = False) -> SimulationStateFrame:
    manifest = _manifest_inventories()
    raw_links = manifest["kinematic_link_inventory"]
    assert isinstance(raw_links, list)
    selected = raw_links[:-1] if missing_link else raw_links
    links = tuple(
        KinematicLinkTruth(
            side=str(item["side"]),
            logical_link_id=str(item["logical_link_id"]),
            prim_path=str(item["prim_path"]),
            position_m=(0.0, 0.0, 0.0),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            valid=True,
        )
        for item in selected
    )
    return SimulationStateFrame.create(
        run_id="episode-001",
        episode_id="episode-001",
        control_index=0,
        tick_id=0,
        phase=SimulationFramePhase.PRE_ACTION,
        simulation_time_s=0.0,
        physics_boundary_index=0,
        q54_rad=(0.0,) * 54,
        qdot54_rad_s=(0.0,) * 54,
        rigid_bodies=(
            DynamicRigidBodyTruth(
                logical_object_id="banana",
                prim_path="/World/Workcell/banana",
                position_m=(0.0, 0.0, 0.0),
                quat_wxyz=(1.0, 0.0, 0.0, 0.0),
                linear_velocity_m_s=(0.0, 0.0, 0.0),
                angular_velocity_rad_s=(0.0, 0.0, 0.0),
                sleeping=False,
                kinematic=banana_kinematic,
                valid=True,
            ),
        ),
        kinematic_links=links,
        expected_rigid_body_count=1,
        expected_kinematic_link_count=len(links),
    )


def test_q54_runtime_inventory_closes_all_names_indices_and_limits() -> None:
    profile, inventory = _runtime_inventory()

    assert validate_q54_runtime_inventory(inventory, profile=profile) == profile.canonical_names

    assert isinstance(inventory, dict)
    limits = inventory["runtime_limits_rad"]
    assert isinstance(limits, list)
    assert isinstance(limits[8], list)
    limits[8][1] += 0.01
    with pytest.raises(ValueError, match=r"runtime limits\[8\]"):
        validate_q54_runtime_inventory(inventory, profile=profile)


def test_truth_inventory_requires_banana_and_all_fourteen_links() -> None:
    manifest = _manifest_inventories()

    objects, links = parse_dataset_truth_inventories(manifest)

    assert objects == {"banana": "/World/Workcell/banana"}
    assert len(links) == 14

    raw_links = manifest["kinematic_link_inventory"]
    assert isinstance(raw_links, list)
    manifest["kinematic_link_inventory"] = raw_links[:-1]
    with pytest.raises(ValueError, match="14 unique bilateral"):
        parse_dataset_truth_inventories(manifest)


def test_each_state_must_match_manifest_and_dynamic_banana_semantics() -> None:
    objects, links = parse_dataset_truth_inventories(_manifest_inventories())

    validate_state_truth_inventory(
        _state(),
        run_id="episode-001",
        objects=objects,
        links=links,
    )

    with pytest.raises(ValueError, match="expected count differs"):
        validate_state_truth_inventory(
            _state(missing_link=True),
            run_id="episode-001",
            objects=objects,
            links=links,
        )
    with pytest.raises(ValueError, match="valid and non-kinematic"):
        validate_state_truth_inventory(
            _state(banana_kinematic=True),
            run_id="episode-001",
            objects=objects,
            links=links,
        )
