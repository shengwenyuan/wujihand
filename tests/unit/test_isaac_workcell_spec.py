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
    assert profile.to_mapping() == _profile()


def test_profile_rejects_unpinned_or_conflicting_policy() -> None:
    invalid_digest = deepcopy(_profile())
    invalid_digest["scene"]["expected_sha256"] = "latest"
    with pytest.raises(ValueError, match="lowercase SHA-256"):
        IsaacStaticUsdWorkcellProfile.from_mapping(invalid_digest)

    conflicting = deepcopy(_profile())
    conflicting["policies"]["collision"] = "replace"
    with pytest.raises(ValueError, match="must be 'preserve'"):
        IsaacStaticUsdWorkcellProfile.from_mapping(conflicting)
