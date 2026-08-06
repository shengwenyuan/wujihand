from __future__ import annotations

from pathlib import Path

import numpy as np

from wujihand.adapters.simulation.isaac_camera import (
    IsaacCameraApiReadback,
    derive_pinhole_calibration,
)
from wujihand.dataset.camera import (
    DatasetCameraRuntimeInventory,
    load_dataset_camera_projections,
)
from wujihand.dataset.profile import MiniDatasetProfile, load_mini_dataset_profile
from wujihand.dataset.rendering import encode_rgb8_png
from wujihand.dataset.vision import DatasetVisionProvenance


ROOT = Path(__file__).parents[2]
DATASET_PROFILE = "configs/profiles/isaac_nero_hand2_triview_q54_mini_dataset_v1.yaml"
IDENTITY = (
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
)


def dataset_profile_and_camera_inventories() -> tuple[
    MiniDatasetProfile,
    tuple[DatasetCameraRuntimeInventory, ...],
]:
    profile = load_mini_dataset_profile(ROOT, DATASET_PROFILE)
    inventories = []
    for projection in load_dataset_camera_projections(ROOT, profile):
        optics = projection.optics
        readback = IsaacCameraApiReadback(
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
        inventories.append(
            DatasetCameraRuntimeInventory(
                camera_id=projection.logical_id,
                carrier_identity=projection.carrier_identity,
                profile_id=projection.profile_id,
                profile_path=projection.profile_path,
                profile_sha256=projection.profile_sha256,
                warning=projection.warning,
                camera_prim_path=f"/World/Test/{projection.logical_id}",
                render_product_path=f"/Render/Test/{projection.logical_id}",
                parent_prim_path="/World",
                parent_frame_id=f"{projection.logical_id}_parent",
                camera_frame_id=f"{projection.logical_id}_usd",
                optical_frame_id=f"{projection.logical_id}_optical",
                parent_from_camera_optical_row_major=IDENTITY,
                mount_visual_sha256=(
                    None if projection.logical_id == "scene_rgb" else "a" * 64
                ),
                camera_visual_sha256=(
                    None if projection.logical_id == "scene_rgb" else "b" * 64
                ),
                generation_report_sha256=(
                    None if projection.logical_id == "scene_rgb" else "c" * 64
                ),
                readback=readback,
                calibration=derive_pinhole_calibration(readback),
            )
        )
    return profile, tuple(inventories)


def rgb_png(value: int) -> bytes:
    return encode_rgb8_png(np.full((480, 640, 3), value, dtype=np.uint8))


def vision_provenance(
    profile: MiniDatasetProfile,
    inventories: tuple[DatasetCameraRuntimeInventory, ...],
    *,
    renderer_identity: str = "offline-fixed-state-v1",
) -> DatasetVisionProvenance:
    return DatasetVisionProvenance.create(
        collection_id="mini-v1",
        dataset_profile_sha256=profile.file_sha256,
        deployment_sha256="1" * 64,
        session_sha256="2" * 64,
        assembly_sha256="3" * 64,
        workcell_sha256="4" * 64,
        renderer_identity=renderer_identity,
        renderer_backend="RayTracedLighting",
        lighting_identity="session_workcell_authored_lighting",
        color_space="isaac_rgb_annotator_srgb",
        motion_blur_enabled=False,
        camera_profile_sha256_by_id={
            item.camera_id: item.profile_sha256 for item in inventories
        },
    )
