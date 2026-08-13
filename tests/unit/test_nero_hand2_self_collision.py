from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from wujihand.adapters.simulation.nero_hand2_self_collision import (
    SELF_COLLISION_FILTER_PROFILE_ID,
    SELF_COLLISION_QUALIFICATION_PROFILE_ID,
    load_nero_hand2_self_collision_filter_profile,
    load_nero_hand2_self_collision_contact_target_profile,
    load_nero_hand2_self_collision_q7_sweep_profile,
    load_nero_hand2_self_collision_qualification_profile,
)


ROOT = Path(__file__).parents[2]
PROFILE = ROOT / "configs/profiles/isaac_nero_hand2_self_collision_qualification_v1.yaml"
FILTER_PROFILE = ROOT / "configs/profiles/isaac_nero_hand2_self_collision_filtered_pairs_v1.yaml"
V8_3_CONTACT_TARGET_PROFILE = (
    ROOT / "configs/profiles/isaac_nero_hand2_self_collision_contact_target_v2026_8_3_v1.yaml"
)
GRIPPER_FLANGE_PROFILE = (
    ROOT
    / "configs/profiles/isaac_nero_hand2_self_collision_qualification_gripper_flange_v2026_8_3_v1.yaml"
)
GRIPPER_FLANGE_Q7_SWEEP = (
    ROOT
    / "configs/profiles/isaac_nero_hand2_self_collision_q7_sweep_gripper_flange_v2026_8_3_v1.yaml"
)
COLLISION_PROXY_PROFILE = (
    ROOT / "configs/profiles/"
    "isaac_nero_hand2_self_collision_qualification_gripper_flange_collision_proxy_v1.yaml"
)
COLLISION_PROXY_FILTER = (
    ROOT / "configs/profiles/"
    "isaac_nero_hand2_self_collision_filtered_pairs_gripper_flange_collision_proxy_v1.yaml"
)


def test_self_collision_profile_freezes_staged_c1_thresholds() -> None:
    profile = load_nero_hand2_self_collision_qualification_profile(PROFILE)

    assert profile.profile_id == SELF_COLLISION_QUALIFICATION_PROFILE_ID
    assert profile.physics_hz == 120
    assert profile.phases.settle_rest == 240
    assert profile.phases.close_trajectory == profile.phases.open_trajectory == 240
    assert profile.thresholds.maximum_unexplained_rest_contact_frames == 3
    assert profile.thresholds.maximum_any_self_penetration_m == pytest.approx(0.002)
    assert dict(profile.collision_mesh_contract)["accessory_collision"] == "disabled"


def test_self_collision_profile_rejects_unknown_fields_and_negative_thresholds(
    tmp_path: Path,
) -> None:
    document = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    document["unexpected"] = True
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="unexpected"):
        load_nero_hand2_self_collision_qualification_profile(invalid)

    document.pop("unexpected")
    document["thresholds"]["maximum_hold_drift_rad"] = -1.0
    invalid.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="maximum_hold_drift_rad"):
        load_nero_hand2_self_collision_qualification_profile(invalid)


def test_self_collision_filter_freezes_only_observed_nero_pair() -> None:
    profile = load_nero_hand2_self_collision_filter_profile(FILTER_PROFILE)

    assert profile.profile_id == SELF_COLLISION_FILTER_PROFILE_ID
    assert len(profile.filtered_pairs) == 1
    pair = profile.filtered_pairs[0]
    assert pair.pair_id == "nero_link5_link7_intrinsic_mesh_overlap_v1"
    assert pair.sides == ("left", "right")
    assert (pair.first_rigid_body_name, pair.second_rigid_body_name) == (
        "link5",
        "link7",
    )
    assert dict(profile.source_contract)["nero_source"].endswith(
        "f6642ce0d7872c686f29c99e9e10cd23d1d49313"
    )
    assert pair.first_instance == pair.second_instance == "arm"


def test_v8_3_contact_target_profile_pins_two_finite_q20_vectors() -> None:
    profile = load_nero_hand2_self_collision_contact_target_profile(V8_3_CONTACT_TARGET_PROFILE)

    assert profile.profile_id.endswith("v2026_8_3_v1")
    assert profile.hand2_source.endswith("8271644a78d69ed9a4adcf9165d882c64ad33dfa")
    assert len(profile.target("left")) == len(profile.target("right")) == 20


def test_gripper_flange_qualification_profile_pins_new_candidate_assets() -> None:
    profile = load_nero_hand2_self_collision_qualification_profile(GRIPPER_FLANGE_PROFILE)
    contract = dict(profile.collision_mesh_contract)

    assert profile.profile_id.endswith("gripper_flange_v2026_8_3_v1")
    assert contract["nero_isaac_asset"].endswith(
        "eaa29a46124c9697a53f8765acd3400b4e8d56c5624748f4ad615f083d679e0d"
    )
    assert contract["hand2_isaac_asset"].endswith(
        "0b6b26fd744b17c33f5750c8502a620cc6e34d6c53d3b76c8701f09702465c8e"
    )


def test_gripper_flange_q7_sweep_covers_wrist_axes_and_45_degree_j7() -> None:
    profile = load_nero_hand2_self_collision_q7_sweep_profile(GRIPPER_FLANGE_Q7_SWEEP)
    waypoints = {waypoint.name: dict(waypoint.overrides_rad) for waypoint in profile.waypoints}

    assert set().union(*(values.keys() for values in waypoints.values())) == {
        "joint4",
        "joint5",
        "joint6",
        "joint7",
    }
    assert waypoints["j7_positive_45_deg"]["joint7"] == pytest.approx(0.7853981633974483)
    assert waypoints["j7_negative_45_deg"]["joint7"] == pytest.approx(-0.7853981633974483)


def test_collision_proxy_profiles_keep_only_the_intrinsic_link5_link7_filter() -> None:
    qualification = load_nero_hand2_self_collision_qualification_profile(COLLISION_PROXY_PROFILE)
    profile = load_nero_hand2_self_collision_filter_profile(COLLISION_PROXY_FILTER)

    assert qualification.profile_id.endswith("gripper_flange_collision_proxy_v1")
    assert dict(qualification.collision_mesh_contract)["nero_isaac_asset"].endswith(
        "a077dc4e47033784326e9701f106f0a190bd1bc17b1bd081df2a3a3ac83286b6"
    )
    assert profile.profile_id.endswith("gripper_flange_collision_proxy_v1")
    assert len(profile.filtered_pairs) == 1
    pair = profile.filtered_pairs[0]
    assert pair.first_instance == pair.second_instance == "arm"
    assert (pair.first_rigid_body_name, pair.second_rigid_body_name) == (
        "link5",
        "link7",
    )


def test_self_collision_filter_rejects_unexplained_and_duplicate_rules(
    tmp_path: Path,
) -> None:
    document = yaml.safe_load(FILTER_PROFILE.read_text(encoding="utf-8"))
    document["filtered_pairs"][0]["evidence"] = {}
    invalid = tmp_path / "invalid-filter.yaml"
    invalid.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="evidence"):
        load_nero_hand2_self_collision_filter_profile(invalid)

    document = yaml.safe_load(FILTER_PROFILE.read_text(encoding="utf-8"))
    document["filtered_pairs"].append(document["filtered_pairs"][0].copy())
    invalid.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="unique"):
        load_nero_hand2_self_collision_filter_profile(invalid)
