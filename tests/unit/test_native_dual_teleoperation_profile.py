from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import pytest
import yaml

from wujihand.runtime import ConfigRepository
from wujihand.specs import (
    NATIVE_DUAL_TELEOPERATION_PROFILE_ID,
    NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT,
    NativeDualTeleoperationProfile,
)


ROOT = Path(__file__).parents[2]
PROFILE = (
    ROOT
    / "configs/profiles/"
    "isaac_nero_hand2_native_dual_teleoperation_v1.yaml"
)


def profile_mapping() -> dict[str, Any]:
    value = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def test_profile_loads_all_live_policy_without_duplicating_mapping() -> None:
    profile = ConfigRepository(
        ROOT
    ).load_native_dual_teleoperation_profile(PROFILE)

    assert profile.profile_id == NATIVE_DUAL_TELEOPERATION_PROFILE_ID
    assert (
        profile.transport_contract
        == NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT
    )
    assert profile.base_qualification.expected_id == (
        "isaac_nero_dual_tabletop_qualification_v1"
    )
    assert profile.physics_hz == 120
    assert profile.tracker.stable_after_s == pytest.approx(0.25)
    assert profile.tracker.max_consecutive_ik_failures == 5
    assert profile.kinematics.end_effector_frame == "link7"
    assert profile.kinematics.position_tolerance_m == pytest.approx(0.002)
    assert profile.arm_supervision.velocity_scale == pytest.approx(0.20)
    assert profile.glove.minimum_landmark_confidence == 0.0
    assert profile.glove.success_landmark_confidence == pytest.approx(0.90)
    assert profile.hand_supervision.velocity_scale == 1.0
    assert "translation_scale" not in profile.to_mapping()
    assert "max_translation_delta_m" not in profile.to_mapping()


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (
            lambda value: value.update({"unexpected": True}),
            "keys differ",
        ),
        (
            lambda value: value.update({"physics_hz": 0}),
            "integer",
        ),
        (
            lambda value: value["tracker"].update(
                {"max_consecutive_ik_failures": 0}
            ),
            "integer",
        ),
        (
            lambda value: value["kinematics"].update(
                {"position_tolerance_m": 0.0}
            ),
            "positive",
        ),
        (
            lambda value: value["arm_supervision"].update(
                {"velocity_scale": 1.1}
            ),
            "at most 1",
        ),
        (
            lambda value: value["glove"].update(
                {
                    "minimum_landmark_confidence": 0.95,
                    "success_landmark_confidence": 0.90,
                }
            ),
            "must not exceed",
        ),
        (
            lambda value: value.update(
                {"transport_contract": "wrong.contract.v1"}
            ),
            "transport_contract",
        ),
    ),
)
def test_profile_rejects_policy_drift(
    mutation: Any,
    message: str,
) -> None:
    value = deepcopy(profile_mapping())
    mutation(value)

    with pytest.raises(ValueError, match=message):
        NativeDualTeleoperationProfile.from_mapping(value)
