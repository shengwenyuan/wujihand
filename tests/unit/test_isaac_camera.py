from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml

from wujihand.adapters.simulation.isaac_camera import (
    CANONICAL_QUIET_NAN_U32,
    IsaacCameraApiReadback,
    assert_profile_matches_readback,
    depth_to_32fc1,
    derive_pinhole_calibration,
    rgba_to_rgb8,
)
from wujihand.runtime.config_repository import ConfigRepository
from wujihand.specs import (
    SIMULATION_ONLY_CAMERA_WARNING,
    SYNTHETIC_WIDE_ANGLE_CLASSIFICATION,
    IsaacCameraProfile,
)


ROOT = Path(__file__).parents[2]
PROFILE = "configs/profiles/isaac_d405_synthetic_wide_angle_140_v1.yaml"


def _mapping() -> dict[str, Any]:
    value = yaml.safe_load((ROOT / PROFILE).read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def _readback(profile: IsaacCameraProfile) -> IsaacCameraApiReadback:
    optics = profile.optics
    return IsaacCameraApiReadback(
        width_px=profile.capture.width_px,
        height_px=profile.capture.height_px,
        projection=optics.projection,
        focal_length_mm=optics.focal_length_mm,
        horizontal_aperture_mm=optics.horizontal_aperture_mm,
        vertical_aperture_mm=optics.vertical_aperture_mm,
        horizontal_aperture_offset_mm=optics.horizontal_aperture_offset_mm,
        vertical_aperture_offset_mm=optics.vertical_aperture_offset_mm,
        clipping_range_m=optics.clipping_range_m,
    )


def test_d405_synthetic_camera_profile_round_trips_strictly() -> None:
    profile = ConfigRepository(ROOT).load_isaac_camera_profile(PROFILE)

    assert profile.profile_id == "isaac_d405_synthetic_wide_angle_140_v1"
    assert profile.simulation_only is True
    assert profile.warning == SIMULATION_ONLY_CAMERA_WARNING
    assert (
        profile.projection_classification
        == SYNTHETIC_WIDE_ANGLE_CLASSIFICATION
    )
    assert (profile.capture.width_px, profile.capture.height_px) == (640, 480)
    assert profile.capture.rate_hz == 30.0
    assert profile.optics.horizontal_fov_deg == 140.0
    assert profile.rgb.source_shape == (480, 640, 4)
    assert profile.depth.source_shape == (480, 640)
    assert profile.depth.source_no_hit_encoding == "positive_infinity"
    assert IsaacCameraProfile.from_mapping(profile.to_mapping()) == profile


@pytest.mark.parametrize(
    ("path", "value", "message"),
    (
        (("simulation_only",), False, "simulation_only must be true"),
        (("warning",), "physical calibration", "simulation-only warning"),
        (
            ("projection_classification",),
            "d405_calibrated",
            "synthetic 140-degree lens",
        ),
        (("optics", "horizontal_fov_deg"), 87.0, "must be 140"),
        (("optics", "distortion_coefficients"), [0, 0, 1, 0, 0], "zero plumb_bob"),
        (("payloads", "rgb", "source_shape"), [480, 640, 3], "differs from capture"),
    ),
)
def test_d405_synthetic_camera_profile_rejects_boundary_drift(
    path: tuple[str, ...],
    value: object,
    message: str,
) -> None:
    mapping = deepcopy(_mapping())
    target: dict[str, Any] = mapping
    for key in path[:-1]:
        nested = target[key]
        assert isinstance(nested, dict)
        target = nested
    target[path[-1]] = value

    with pytest.raises(ValueError, match=message):
        IsaacCameraProfile.from_mapping(mapping)


def test_pinhole_calibration_is_derived_from_api_readback() -> None:
    profile = ConfigRepository(ROOT).load_isaac_camera_profile(PROFILE)
    readback = _readback(profile)

    assert_profile_matches_readback(profile, readback)
    calibration = derive_pinhole_calibration(readback)

    assert calibration.horizontal_fov_deg == pytest.approx(140.0)
    assert calibration.vertical_fov_deg == pytest.approx(128.22599124075742)
    assert calibration.k_row_major == pytest.approx(
        (
            116.47047496518479,
            0.0,
            320.0,
            0.0,
            116.47047496518479,
            240.0,
            0.0,
            0.0,
            1.0,
        )
    )
    assert calibration.p_row_major == pytest.approx(
        (
            116.47047496518479,
            0.0,
            320.0,
            0.0,
            0.0,
            116.47047496518479,
            240.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        )
    )


def test_aperture_offsets_have_frozen_usd_to_ros_raster_signs() -> None:
    profile = ConfigRepository(ROOT).load_isaac_camera_profile(PROFILE)
    readback = _readback(profile)
    shifted = IsaacCameraApiReadback(
        width_px=readback.width_px,
        height_px=readback.height_px,
        projection=readback.projection,
        focal_length_mm=readback.focal_length_mm,
        horizontal_aperture_mm=readback.horizontal_aperture_mm,
        vertical_aperture_mm=readback.vertical_aperture_mm,
        horizontal_aperture_offset_mm=1.0,
        vertical_aperture_offset_mm=1.0,
        clipping_range_m=readback.clipping_range_m,
    )

    calibration = derive_pinhole_calibration(shifted)
    assert calibration.k_row_major[2] < 320.0
    assert calibration.k_row_major[5] > 240.0


def test_payload_conversion_is_shape_dtype_and_channel_strict() -> None:
    profile = ConfigRepository(ROOT).load_isaac_camera_profile(PROFILE)
    rgba = np.zeros(profile.rgb.source_shape, dtype=np.uint8)
    rgba[0, 0] = [11, 22, 33, 44]

    rgb = rgba_to_rgb8(rgba, profile)

    assert rgb.shape == (480, 640, 3)
    assert rgb.dtype == np.uint8
    assert rgb.flags.c_contiguous
    assert rgb[0, 0].tolist() == [11, 22, 33]
    with pytest.raises(ValueError, match="uint8 RGBA"):
        rgba_to_rgb8(rgba[..., :3], profile)


def test_depth_conversion_preserves_positive_values_and_canonicalizes_invalid() -> None:
    profile = ConfigRepository(ROOT).load_isaac_camera_profile(PROFILE)
    depth = np.full(profile.depth.source_shape, 1.25, dtype=np.float32)
    depth[0, :5] = [np.inf, -np.inf, np.nan, 0.0, -1.0]

    converted = depth_to_32fc1(depth, profile)

    assert converted.flags.c_contiguous
    assert converted[1, 1] == np.float32(1.25)
    assert np.all(
        converted.view(np.uint32)[0, :5] == CANONICAL_QUIET_NAN_U32
    )
    assert math.isinf(float(depth[0, 0]))
    with pytest.raises(ValueError, match="float32 optical-Z"):
        depth_to_32fc1(depth.astype(np.float64), profile)
