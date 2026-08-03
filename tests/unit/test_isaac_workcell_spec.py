from __future__ import annotations

from copy import deepcopy
from typing import Any

import pytest

from wujihand.specs import IsaacStaticUsdWorkcellProfile


def _profile() -> dict[str, Any]:
    content = {
        "source": "scene-library",
        "source_revision": f"commit:{'a' * 40}",
        "path": "assets/scenes/demo.usda",
        "expected_sha256": "b" * 64,
    }
    return {
        "schema": "wujihand.isaac_static_usd_workcell.v1",
        "profile_id": "demo_scene_v1",
        "import_id": "demo_scene",
        "scene": content,
        "composition": "reference",
        "frame": "scene_anchor",
        "transform": {
            "position_m": [0.0, 0.0, 0.0],
            "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
        },
        "policies": {
            "ground": "preserve",
            "physics_scene": "project",
            "camera": "project",
            "collision": "preserve",
            "fixed_rigid_body_paths": ["table", "fixtures/bowl"],
        },
        "lighting": {
            "mode": "selected_hdr",
            "content": {
                **content,
                "path": "assets/backgrounds/studio.hdr",
                "expected_sha256": "c" * 64,
            },
            "intensity": 900.0,
            "exposure": 0.0,
            "visible_in_primary_ray": False,
            "background_color_rgb": [0.12, 0.12, 0.12],
        },
        "expectations": {
            "default_prim": "world",
            "meters_per_unit": 1.0,
            "up_axis": "Z",
            "min_colliders": 1,
        },
    }


def test_profile_round_trips_strict_source_locked_scene_and_hdr() -> None:
    profile = IsaacStaticUsdWorkcellProfile.from_mapping(_profile())

    assert profile.profile_id == "demo_scene_v1"
    assert profile.scene.artifact.path == "assets/scenes/demo.usda"
    assert profile.lighting.content is not None
    assert profile.lighting.visible_in_primary_ray is False
    assert profile.lighting.background_color_rgb == (0.12, 0.12, 0.12)
    assert profile.policies.fixed_rigid_body_paths == (
        "table",
        "fixtures/bowl",
    )
    assert profile.to_mapping() == _profile()


def test_profile_keeps_v1_collision_policy_backward_compatible() -> None:
    legacy = _profile()
    del legacy["policies"]["fixed_rigid_body_paths"]

    profile = IsaacStaticUsdWorkcellProfile.from_mapping(legacy)

    assert profile.policies.fixed_rigid_body_paths == ()


def test_profile_rejects_unpinned_or_conflicting_policy() -> None:
    invalid_digest = deepcopy(_profile())
    invalid_digest["scene"]["expected_sha256"] = "latest"
    with pytest.raises(ValueError, match="lowercase SHA-256"):
        IsaacStaticUsdWorkcellProfile.from_mapping(invalid_digest)

    conflicting = deepcopy(_profile())
    conflicting["policies"]["collision"] = "replace"
    with pytest.raises(ValueError, match="must be 'preserve'"):
        IsaacStaticUsdWorkcellProfile.from_mapping(conflicting)

    invalid_background = deepcopy(_profile())
    invalid_background["lighting"]["background_color_rgb"] = [0.0, 2.0, 0.0]
    with pytest.raises(ValueError, match=r"must be in \[0, 1\]"):
        IsaacStaticUsdWorkcellProfile.from_mapping(invalid_background)

    invalid_fixed_path = deepcopy(_profile())
    invalid_fixed_path["policies"]["fixed_rigid_body_paths"] = [
        "../table"
    ]
    with pytest.raises(ValueError, match="relative USD prim path"):
        IsaacStaticUsdWorkcellProfile.from_mapping(invalid_fixed_path)
