from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from wujihand.adapters.simulation.nero_gripper_flange_import import (
    build_nero_gripper_flange_urdf,
    load_nero_gripper_flange_facts,
    load_nero_gripper_flange_import_profile,
)


ROOT = Path(__file__).parents[2]
PROFILE = ROOT / "configs/profiles/agilex_nero_gripper_flange_isaac_6_0_1_import_v1.yaml"
SOURCE = ROOT / "third_party/src/agx_arm_urdf_nero_gripper_flange"


def test_flange_profile_keeps_the_base_recipe_and_parallel_output() -> None:
    profile = load_nero_gripper_flange_import_profile(PROFILE)

    assert profile.source_commit == "f6642ce0d7872c686f29c99e9e10cd23d1d49313"
    assert profile.base_recipe == "configs/profiles/agilex_nero_isaac_6_0_1_import_v1.yaml"
    assert profile.output_root.endswith("agilex_nero_gripper_flange_v1")
    assert profile.robot_name == "nero_description"
    assert profile.flange_post_rotation_rpy_rad == pytest.approx((0.0, 0.0, 0.0))


@pytest.mark.requires_upstream_asset
def test_pinned_flange_xacro_has_the_official_fixed_transform() -> None:
    xacro = SOURCE / "nero/urdf/nero_with_gripper_flange_description.xacro"
    if not xacro.is_file():
        pytest.skip("restore the pinned NERO gripper-flange source")
    facts = load_nero_gripper_flange_facts(xacro)

    assert facts.parent_link == "link7"
    assert facts.position_m == (0.031, 0.0, -0.0235)
    assert facts.mass_kg == pytest.approx(0.04771096)
    assert facts.visual_mesh_uri.endswith("gripper_flange.dae")
    assert facts.collision_mesh_uri.endswith("gripper_flange.stl")


@pytest.mark.requires_upstream_asset
def test_expanded_urdf_adds_the_clocked_fixed_flange_without_changing_nero(
    tmp_path: Path,
) -> None:
    xacro = SOURCE / "nero/urdf/nero_with_gripper_flange_description.xacro"
    base = ROOT / "third_party/src/agx_arm_urdf/nero/urdf/nero_description.urdf"
    if not xacro.is_file() or not base.is_file():
        pytest.skip("restore the pinned NERO sources")
    output = tmp_path / "nero_description.urdf"
    effective = build_nero_gripper_flange_urdf(
        base_urdf=base,
        flange_xacro=xacro,
        output=output,
        flange_package_name="agilex_nero_gripper_flange",
        flange_post_rotation_rpy_rad=(0.0, 0.0, 0.0),
    )

    root = ET.parse(output).getroot()
    assert len(root.findall("link")) == 10
    assert len(root.findall("joint")) == 9
    assert root.find("link[@name='gripper_flange']") is not None
    joint = root.find("joint[@name='gripper_flange_joint']")
    assert joint is not None and joint.attrib["type"] == "fixed"
    filenames = [
        mesh.attrib["filename"]
        for mesh in root.findall("link[@name='gripper_flange']/*/geometry/mesh")
    ]
    assert all(value.startswith("package://agilex_nero_gripper_flange/") for value in filenames)
    link7 = root.find("link[@name='link7']")
    assert link7 is not None
    for path in ("visual/origin", "collision/origin", "inertial/origin"):
        origin = link7.find(path)
        assert origin is not None
        source_origin = ET.parse(base).getroot().find(f"link[@name='link7']/{path}")
        assert source_origin is not None and origin.attrib == source_origin.attrib
    assert tuple(
        float(value)
        for value in link7.find("inertial/origin").attrib["xyz"].split()  # type: ignore[union-attr]
    ) == pytest.approx((-0.00014, -0.0001, -0.00275))
    flange_origin = joint.find("origin")
    assert flange_origin is not None
    assert tuple(float(value) for value in flange_origin.attrib["xyz"].split()) == pytest.approx(
        (0.031, 0.0, -0.0235)
    )
    assert effective.position_m == pytest.approx((0.031, 0.0, -0.0235))
    assert effective.quaternion_wxyz == pytest.approx(
        (0.5, -0.5, 0.5, -0.5), abs=5e-6
    )
    source_joint7 = ET.parse(base).getroot().find("joint[@name='joint7']/origin")
    expanded_joint7 = root.find("joint[@name='joint7']/origin")
    assert source_joint7 is not None and expanded_joint7 is not None
    assert expanded_joint7.attrib == source_joint7.attrib
