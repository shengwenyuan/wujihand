from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from wujihand.adapters.simulation.nero_model import (
    NERO_CHILD_LINKS,
    NERO_JOINT_NAMES,
    load_nero_model_profile,
)
from wujihand.adapters.simulation.nero_urdf_import import (
    NERO_MESH_URI_PREFIX,
    NERO_SOURCE_URDF_SHA256,
)
from wujihand.integrity import sha256_file


ROOT = Path(__file__).parents[2]
SOURCE_ROOT = ROOT / "third_party/src/agx_arm_urdf"
URDF = SOURCE_ROOT / "nero/urdf/nero_description.urdf"
PROFILE = ROOT / "configs/profiles/agilex_nero_q7_provisional_v1.yaml"


def _restored_urdf() -> ET.Element:
    if not URDF.is_file():
        pytest.skip("restore the pinned AgileX NERO URDF and meshes")
    return ET.parse(URDF).getroot()


def test_pinned_nero_urdf_matches_the_canonical_q7_profile() -> None:
    robot = _restored_urdf()
    profile = load_nero_model_profile(PROFILE)
    revolute = {
        joint.attrib["name"]: joint
        for joint in robot.findall("joint")
        if joint.attrib["type"] == "revolute"
    }

    assert sha256_file(URDF) == NERO_SOURCE_URDF_SHA256
    assert tuple(revolute) == NERO_JOINT_NAMES
    world_joint = robot.find("joint[@name='world_to_base_link']")
    assert world_joint is not None
    assert world_joint.attrib["type"] == "fixed"
    for index, name in enumerate(NERO_JOINT_NAMES):
        joint = revolute[name]
        axis = joint.find("axis")
        child = joint.find("child")
        limit = joint.find("limit")
        assert axis is not None
        assert child is not None
        assert limit is not None
        assert (
            tuple(float(value) for value in axis.attrib["xyz"].split()) == (profile.axes_xyz[index])
        )
        assert child.attrib["link"] == NERO_CHILD_LINKS[index]
        assert float(limit.attrib["lower"]) == pytest.approx(profile.layout.lower[index])
        assert float(limit.attrib["upper"]) == pytest.approx(profile.layout.upper[index])
        assert float(limit.attrib["velocity"]) == pytest.approx(profile.urdf_velocity_rad_s[index])


def test_pinned_nero_urdf_has_all_sixteen_source_meshes() -> None:
    robot = _restored_urdf()
    mesh_uris = tuple(mesh.attrib["filename"] for mesh in robot.findall(".//mesh"))

    assert len(mesh_uris) == len(set(mesh_uris)) == 16
    assert sum("/dae/" in uri for uri in mesh_uris) == 8
    assert sum(uri.endswith(".stl") for uri in mesh_uris) == 8
    for uri in mesh_uris:
        assert uri.startswith(NERO_MESH_URI_PREFIX)
        path = SOURCE_ROOT / uri.removeprefix(NERO_MESH_URI_PREFIX)
        assert path.is_file(), path
