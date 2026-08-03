from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import pytest
import yaml

from wujihand.runtime import ConfigRepository, SessionResolver
from wujihand.specs import (
    DUAL_TELEOPERATION_CONTRACT,
    DUAL_TELEOPERATION_PROFILE_ID,
    DualTeleoperationProfile,
)


ROOT = Path(__file__).parents[2]
PROFILE = (
    ROOT
    / "configs/profiles/isaac_nero_hand2_dual_teleoperation_v1.yaml"
)
SESSION = ROOT / "configs/sessions/isaac_nero_dual_hand2_teleop_v1.yaml"


def profile_mapping() -> dict[str, Any]:
    value = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def test_transport_neutral_profile_preserves_validated_policy() -> None:
    repository = ConfigRepository(ROOT)
    profile = repository.load_dual_teleoperation_profile(PROFILE)

    assert profile.profile_id == DUAL_TELEOPERATION_PROFILE_ID
    assert profile.transport_contract == DUAL_TELEOPERATION_CONTRACT
    assert profile.physics_hz == 120
    assert profile.tracker.max_consecutive_ik_failures == 5
    assert profile.kinematics.end_effector_frame == "link7"
    assert profile.arm_supervision.velocity_scale == pytest.approx(0.20)
    assert profile.glove.minimum_landmark_confidence == 0.0
    assert profile.glove.success_landmark_confidence == pytest.approx(0.60)
    assert profile.hand_supervision.velocity_scale == 1.0


def test_transport_neutral_session_resolves_all_five_layers() -> None:
    resolved = SessionResolver(ROOT).resolve(SESSION)

    assert resolved.session.session_id == "isaac_nero_dual_hand2_teleop_v1"
    assert (
        resolved.session.runtime.transport_contract
        == DUAL_TELEOPERATION_CONTRACT
    )
    assert len(resolved.instances) == 4


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (
            lambda value: value.update(
                {"transport_contract": "wujihand.native_dual_teleoperation.v1"}
            ),
            "transport_contract",
        ),
        (
            lambda value: value["tracker"].update(
                {"max_consecutive_ik_failures": 0}
            ),
            "integer",
        ),
        (
            lambda value: value["glove"].update(
                {
                    "minimum_landmark_confidence": 0.95,
                    "success_landmark_confidence": 0.60,
                }
            ),
            "must not exceed",
        ),
    ),
)
def test_transport_neutral_profile_rejects_policy_drift(
    mutation: Any,
    message: str,
) -> None:
    value = deepcopy(profile_mapping())
    mutation(value)

    with pytest.raises(ValueError, match=message):
        DualTeleoperationProfile.from_mapping(value)
