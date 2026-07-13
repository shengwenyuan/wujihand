from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from wujihand.adapters.simulation import load_hand2_model_profile
from wujihand.domain import HAND2_RIGHT_LAYOUT


ROOT = Path(__file__).parents[2]
PROFILE = ROOT / "configs/profiles/hand2_right_v2026_6_27.yaml"
URDF = ROOT / "third_party/src/wuji-description/hand2_beta/body/urdf/right.urdf"


def test_profile_matches_pinned_domain_contract() -> None:
    profile = load_hand2_model_profile(PROFILE)
    assert profile.layout == HAND2_RIGHT_LAYOUT
    assert profile.layout.size == 20
    assert profile.provenance["tag"] == "v2026.6.27"


@pytest.mark.requires_upstream_asset
def test_profile_matches_pinned_urdf_joint_order_and_limits() -> None:
    if not URDF.is_file():
        pytest.skip("restore wuji-description from third_party/sources.lock.yaml")

    profile = load_hand2_model_profile(PROFILE)
    root = ET.parse(URDF).getroot()
    joints = [joint for joint in root.findall("joint") if joint.attrib["type"] != "fixed"]
    assert [joint.attrib["name"] for joint in joints] == list(profile.layout.names)
    np.testing.assert_allclose(
        [float(joint.find("limit").attrib["lower"]) for joint in joints], profile.layout.lower
    )
    np.testing.assert_allclose(
        [float(joint.find("limit").attrib["upper"]) for joint in joints], profile.layout.upper
    )


def test_backend_mapping_matches_observed_isaac_dof_order() -> None:
    profile = load_hand2_model_profile(PROFILE)
    backend_names = [
        "r_index_finger_mcp_flex",
        "r_middle_finger_mcp_flex",
        "r_pinky_mcp_flex",
        "r_ring_finger_mcp_flex",
        "r_thumb_cmc_flex",
        "r_index_finger_mcp_abd",
        "r_middle_finger_mcp_abd",
        "r_pinky_mcp_abd",
        "r_ring_finger_mcp_abd",
        "r_thumb_cmc_abd",
        "r_index_finger_pip",
        "r_middle_finger_pip",
        "r_pinky_pip",
        "r_ring_finger_pip",
        "r_thumb_mcp",
        "r_index_finger_dip",
        "r_middle_finger_dip",
        "r_pinky_dip",
        "r_ring_finger_dip",
        "r_thumb_ip",
    ]
    expected_indices = [4, 8, 16, 12, 0, 5, 9, 17, 13, 1, 6, 10, 18, 14, 2, 7, 11, 19, 15, 3]
    q20 = np.arange(20, dtype=np.float64)
    backend = profile.firmware_to_backend(q20, backend_names)

    np.testing.assert_array_equal(backend, q20[expected_indices])
    np.testing.assert_array_equal(profile.backend_to_firmware(backend, backend_names), q20)
