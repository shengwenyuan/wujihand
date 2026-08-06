from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from wujihand.adapters.simulation.isaac_camera import IsaacCameraApiReadback
from wujihand.dataset.camera import (
    OFFLINE_CAPTURE_PHASE,
    assert_dataset_projection_matches_readback,
    load_dataset_camera_projections,
)
from wujihand.dataset.profile import load_mini_dataset_profile


ROOT = Path(__file__).parents[2]
DATASET_PROFILE = "configs/profiles/isaac_nero_hand2_triview_q54_mini_dataset_v1.yaml"


def _readback(projection_index: int) -> IsaacCameraApiReadback:
    profile = load_mini_dataset_profile(ROOT, DATASET_PROFILE)
    projection = load_dataset_camera_projections(ROOT, profile)[projection_index]
    optics = projection.optics
    return IsaacCameraApiReadback(
        width_px=projection.width_px,
        height_px=projection.height_px,
        projection=optics.projection,
        focal_length_mm=optics.focal_length_mm,
        horizontal_aperture_mm=optics.horizontal_aperture_mm,
        vertical_aperture_mm=optics.vertical_aperture_mm,
        horizontal_aperture_offset_mm=optics.horizontal_aperture_offset_mm,
        vertical_aperture_offset_mm=optics.vertical_aperture_offset_mm,
        clipping_range_m=optics.clipping_range_m,
    )


def test_dataset_camera_resolver_freezes_scene_and_dual_wrist_rgb() -> None:
    profile = load_mini_dataset_profile(ROOT, DATASET_PROFILE)

    projections = load_dataset_camera_projections(ROOT, profile)

    assert tuple(item.logical_id for item in projections) == (
        "scene_rgb",
        "left_wrist_rgb",
        "right_wrist_rgb",
    )
    assert tuple(item.optics.horizontal_fov_deg for item in projections) == (
        90.0,
        140.0,
        140.0,
    )
    assert all(item.capture_phase == OFFLINE_CAPTURE_PHASE for item in projections)
    assert all(item.source_shape == (480, 640, 4) for item in projections)
    assert all(item.output_encoding == "rgb8" for item in projections)
    assert all(item.simulation_only for item in projections)
    assert not any(item.physical_calibration_compatible for item in projections)


@pytest.mark.parametrize("projection_index", [0, 1, 2])
def test_dataset_camera_api_readback_closes_projection(projection_index: int) -> None:
    profile = load_mini_dataset_profile(ROOT, DATASET_PROFILE)
    projection = load_dataset_camera_projections(ROOT, profile)[projection_index]

    calibration = assert_dataset_projection_matches_readback(
        projection,
        _readback(projection_index),
    )

    assert calibration.horizontal_fov_deg == pytest.approx(
        projection.optics.horizontal_fov_deg
    )
    assert len(calibration.k_row_major) == 9
    assert len(calibration.p_row_major) == 12


def test_dataset_camera_api_readback_rejects_active_lens_drift() -> None:
    profile = load_mini_dataset_profile(ROOT, DATASET_PROFILE)
    projection = load_dataset_camera_projections(ROOT, profile)[1]
    readback = replace(_readback(1), horizontal_aperture_mm=20.0)

    with pytest.raises(ValueError, match="optics readback differs"):
        assert_dataset_projection_matches_readback(projection, readback)
