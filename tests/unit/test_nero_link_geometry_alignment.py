from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import pytest
import yaml

from wujihand.adapters.simulation.nero_link_geometry_alignment import (
    NERO_LINK_GEOMETRY_ALIGNMENT_ID,
    load_nero_link_geometry_alignment,
)


ROOT = Path(__file__).parents[2]
PROFILE = (
    ROOT
    / "configs/profiles/agilex_nero_7f_link6_geometry_alignment_v1.yaml"
)


def _profile_mapping() -> dict[str, Any]:
    value = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def _write_profile(tmp_path: Path, value: dict[str, Any]) -> Path:
    path = tmp_path / "link6-geometry-alignment.yaml"
    path.write_text(
        yaml.safe_dump(value, sort_keys=False),
        encoding="utf-8",
    )
    return path


def test_profile_maps_link6_cylinder_axis_to_forearm_without_joint_changes() -> None:
    profile = load_nero_link_geometry_alignment(PROFILE)

    assert profile.alignment_id == NERO_LINK_GEOMETRY_ALIGNMENT_ID
    assert profile.link_name == "link6"
    assert profile.source_cylinder_axis_local_xyz == (0.0, 1.0, 0.0)
    assert profile.corrected_cylinder_axis_local_xyz == (1.0, 0.0, 0.0)
    assert profile.geometry_post_rotation_quat_wxyz == pytest.approx(
        (2.0**-0.5, 0.0, 0.0, -(2.0**-0.5))
    )
    assert profile.visual_child_name == "link6"
    assert profile.collision_child_name == "link6_1"
    assert (
        "link6_geometry_alignment_preserves_all_joint_frames_and_hand_world_pose"
        in profile.assumptions
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (
            lambda value: value["correction"].update(
                {"corrected_cylinder_axis_local_xyz": [0.0, 0.0, 1.0]}
            ),
            "corrected cylinder axis",
        ),
        (
            lambda value: value["correction"].update(
                {"corrected_center_of_mass_m": [0.0, 0.0, 0.0]}
            ),
            "corrected center of mass",
        ),
        (
            lambda value: value["correction"].update(
                {
                    "corrected_principal_axes_quat_wxyz": [
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                    ]
                }
            ),
            "corrected principal axes",
        ),
        (
            lambda value: value.update({"unexpected": True}),
            "keys differ",
        ),
    ),
)
def test_profile_rejects_geometry_and_schema_drift(
    tmp_path: Path,
    mutation: Any,
    message: str,
) -> None:
    value = deepcopy(_profile_mapping())
    mutation(value)

    with pytest.raises(ValueError, match=message):
        load_nero_link_geometry_alignment(_write_profile(tmp_path, value))
