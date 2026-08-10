from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree

import pytest
import yaml

from wujihand.runtime import SessionResolver


ROOT = Path(__file__).parents[2]
MATRIX = (
    ("left", "v2026_6_27", "wuji-description-v2026-6-27", "l_base_link"),
    ("right", "v2026_6_27", "wuji-description-v2026-6-27", "r_base_link"),
    ("left", "v2026_8_3", "wuji-description-v2026-8-3", "l_wrist"),
    ("right", "v2026_8_3", "wuji-description-v2026-8-3", "r_wrist"),
)
EXPECTED_FIXED_MOUNTS = {
    ("left", "v2026_6_27"): (
        (-0.1, 0.0, 0.43),
        (0.70710678, 0.0, -0.70710678, 0.0),
    ),
    ("right", "v2026_6_27"): (
        (-0.1, 0.0, 0.43),
        (0.70710678, 0.0, -0.70710678, 0.0),
    ),
    ("left", "v2026_8_3"): (
        (-0.1285, -0.003, 0.43030016),
        (0.5, -0.5, 0.5, -0.5),
    ),
    ("right", "v2026_8_3"): (
        (-0.12849985, 0.00300004, 0.43025016),
        (0.5, 0.5, 0.5, 0.5),
    ),
}


@pytest.mark.parametrize(("side", "token", "source", "root"), MATRIX)
@pytest.mark.parametrize("kind", ("fixed", "collision"))
def test_hand2_qualification_sessions_close_one_explicit_description_version(
    side: str,
    token: str,
    source: str,
    root: str,
    kind: str,
) -> None:
    path = (
        ROOT
        / f"configs/sessions/isaac_hand2_{side}_{kind}_qualification_{token}_v1.yaml"
    )
    resolved = SessionResolver(ROOT).resolve(path)
    hand = resolved.instance("hand")

    assert [record.name for record in resolved.source_records] == [source]
    assert hand.binding.root == root
    assert hand.binding.backend_frame(hand.asset.frame_name("base")) == root
    assert token in hand.asset_path
    assert token in hand.binding_path
    assert token in resolved.workcell_path
    assert side in resolved.workcell_path
    assert hand.binding.compatibility_profile == hand.asset.canonical_profile

    position, orientation = EXPECTED_FIXED_MOUNTS[(side, token)]
    mount = resolved.workcell.mount("hand_fixed_mount")
    assert mount.transform.position_m == pytest.approx(position)
    assert mount.transform.quat_wxyz == pytest.approx(orientation)


@pytest.mark.parametrize("side", ("left", "right"))
def test_old_and_new_profiles_preserve_q20_names_limits_and_units(side: str) -> None:
    profiles = []
    for token in ("v2026_6_27", "v2026_8_3"):
        path = ROOT / f"configs/profiles/hand2_{side}_{token}.yaml"
        profile = yaml.safe_load(path.read_text(encoding="utf-8"))
        assert profile["units"] == {"position": "rad", "velocity": "rad_s"}
        profiles.append(profile)

    assert profiles[0]["joints"] == profiles[1]["joints"]
    assert profiles[0]["rest_position"] == profiles[1]["rest_position"]
    assert profiles[0]["derived_from"]["tag"] == "v2026.6.27"
    assert profiles[1]["derived_from"]["tag"] == "v2026.8.3"
    assert profiles[1]["derived_from"]["hand2_model_revision"] == "v2026.7.23"


@pytest.mark.requires_upstream_asset
@pytest.mark.parametrize("side", ("left", "right"))
def test_old_and_new_urdf_roots_change_but_q20_contract_does_not(side: str) -> None:
    prefix = side[0]
    paths = (
        ROOT
        / f"third_party/src/wuji-description/v2026.6.27/hand2_beta/body/urdf/{side}.urdf",
        ROOT
        / (
            "third_party/src/wuji-description/v2026.8.3/"
            f"hand2/hand2_beta1/body/urdf/{side}.urdf"
        ),
    )
    if not all(path.is_file() for path in paths):
        pytest.skip("restore both Wuji Description releases from the source lock")

    summaries = []
    for path in paths:
        robot = ElementTree.parse(path).getroot()
        links = {link.attrib["name"] for link in robot.findall("link")}
        joints = robot.findall("joint")
        children = {
            child.attrib["link"]
            for joint in joints
            if (child := joint.find("child")) is not None
        }
        movable = [joint for joint in joints if joint.attrib.get("type") != "fixed"]
        summaries.append(
            {
                "roots": links - children,
                "names": [joint.attrib["name"] for joint in movable],
                "limits": [joint.find("limit").attrib for joint in movable],  # type: ignore[union-attr]
                "axes": [joint.find("axis").attrib["xyz"] for joint in movable],  # type: ignore[union-attr]
            }
        )

    assert summaries[0]["roots"] == {f"{prefix}_base_link"}
    assert summaries[1]["roots"] == {f"{prefix}_wrist"}
    assert summaries[0]["names"] == summaries[1]["names"]
    assert summaries[0]["limits"] == summaries[1]["limits"]
    assert summaries[0]["axes"] != summaries[1]["axes"]
