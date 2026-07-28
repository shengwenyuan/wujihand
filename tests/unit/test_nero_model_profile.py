from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml

from wujihand.adapters.simulation.nero_model import (
    NERO_CHILD_LINKS,
    NERO_JOINT_NAMES,
    NERO_LAYOUT_ID,
    NERO_PROFILE_STATUS,
    load_nero_model_profile,
)


ROOT = Path(__file__).parents[2]
PROFILE = ROOT / "configs/profiles/agilex_nero_q7_provisional_v1.yaml"


def _profile_mapping() -> dict[str, Any]:
    value = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def _write_profile(tmp_path: Path, value: dict[str, Any]) -> Path:
    path = tmp_path / "nero.yaml"
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    return path


def test_nero_profile_has_pinned_q7_frames_limits_and_provenance() -> None:
    profile = load_nero_model_profile(PROFILE)

    assert profile.profile_id == "agilex_nero_q7_provisional_v1"
    assert profile.status == NERO_PROFILE_STATUS
    assert profile.layout_id == NERO_LAYOUT_ID
    assert profile.layout.names == NERO_JOINT_NAMES
    assert profile.child_links == NERO_CHILD_LINKS
    assert profile.base_frame == "base_link"
    assert profile.tool_flange_frame == "link7"
    assert profile.axes_xyz == ((0.0, 0.0, 1.0),) * 7
    assert np.array_equal(profile.home_position, np.zeros(7))
    assert profile.layout.lower[1] == pytest.approx(-1.74)
    assert profile.layout.upper[1] == pytest.approx(1.74)
    assert profile.layout.velocity[:3] == pytest.approx((np.pi,) * 3)
    assert profile.layout.velocity[3:] == pytest.approx((1.25 * np.pi,) * 4)
    assert profile.urdf_velocity_rad_s == (5.0,) * 7
    assert profile.provenance["urdf"] == {
        "commit": "f6642ce0d7872c686f29c99e9e10cd23d1d49313",
        "license": "MIT",
        "path": "nero/urdf/nero_description.urdf",
        "repository": "https://github.com/agilexrobotics/agx_arm_urdf.git",
        "sha256": "c297c4bd2caeff44c673ae69070fc80f950510c0cb33cfa8b81b5bc774e91278",
    }
    assert dict(profile.limit_policy)["hardware_gate"] == (
        "read_back_both_devices_before_real_motion"
    )


def test_nero_profile_maps_canonical_and_backend_orders() -> None:
    profile = load_nero_model_profile(PROFILE)
    backend_names = tuple(reversed(NERO_JOINT_NAMES))
    canonical = np.arange(7, dtype=np.float64) * 0.1

    backend = profile.canonical_to_backend(canonical, backend_names)
    restored = profile.backend_to_canonical(backend, backend_names)

    assert np.array_equal(backend, canonical[::-1])
    assert np.array_equal(restored, canonical)


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (lambda value: value.update({"status": "hardware_approved"}), "status"),
        (
            lambda value: value["joints"][1].update({"name": "joint3"}),
            "ordered",
        ),
        (
            lambda value: value["joints"][0].update(
                {"axis_xyz": [0.0, 0.0, 2.0]}
            ),
            "unit vector",
        ),
        (
            lambda value: value["joints"][0].update({"velocity": 6.0}),
            "must not exceed",
        ),
        (
            lambda value: value.update(
                {"home_position": [9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
            ),
            "within joint limits",
        ),
    ),
)
def test_nero_profile_rejects_unapproved_or_inconsistent_facts(
    tmp_path: Path,
    mutation: Any,
    message: str,
) -> None:
    value = deepcopy(_profile_mapping())
    mutation(value)

    with pytest.raises(ValueError, match=message):
        load_nero_model_profile(_write_profile(tmp_path, value))
