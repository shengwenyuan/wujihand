from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from wujihand.adapters.simulation.nero_hand2_self_collision import (
    SELF_COLLISION_FILTER_PROFILE_ID,
    SELF_COLLISION_QUALIFICATION_PROFILE_ID,
    load_nero_hand2_self_collision_filter_profile,
    load_nero_hand2_self_collision_qualification_profile,
)


ROOT = Path(__file__).parents[2]
PROFILE = ROOT / "configs/profiles/isaac_nero_hand2_self_collision_qualification_v1.yaml"
FILTER_PROFILE = (
    ROOT / "configs/profiles/isaac_nero_hand2_self_collision_filtered_pairs_v1.yaml"
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
