"""Pure conversion boundary for the synthetic Isaac wrist cameras."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from wujihand.specs import IsaacCameraProfile


CANONICAL_QUIET_NAN_U32 = np.uint32(0x7FC00000)


@dataclass(frozen=True, slots=True)
class IsaacCameraApiReadback:
    """Static values read from the active Isaac 6.0.1 camera/render product."""

    width_px: int
    height_px: int
    projection: str
    focal_length_mm: float
    horizontal_aperture_mm: float
    vertical_aperture_mm: float
    horizontal_aperture_offset_mm: float
    vertical_aperture_offset_mm: float
    clipping_range_m: tuple[float, float]


@dataclass(frozen=True, slots=True)
class PinholeCalibration:
    """Derived ROS optical-frame calibration using row-major matrices."""

    horizontal_fov_deg: float
    vertical_fov_deg: float
    k_row_major: tuple[float, ...]
    p_row_major: tuple[float, ...]


def derive_pinhole_calibration(
    readback: IsaacCameraApiReadback,
) -> PinholeCalibration:
    """Derive exact-simulation K/P from Isaac API readback.

    The versioned convention maps the USD aperture window to ROS raster pixels:
    image extents are ``[0,width] x [0,height]``; positive horizontal aperture
    offset moves ``cx`` left, while positive USD-up vertical offset moves ROS-down
    ``cy`` down. This convention must be render-checked before CameraInfo release.
    """

    if readback.projection != "perspective":
        raise ValueError("camera readback projection must be perspective")
    if readback.width_px <= 0 or readback.height_px <= 0:
        raise ValueError("camera readback resolution must be positive")
    values = (
        readback.focal_length_mm,
        readback.horizontal_aperture_mm,
        readback.vertical_aperture_mm,
        readback.horizontal_aperture_offset_mm,
        readback.vertical_aperture_offset_mm,
        *readback.clipping_range_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("camera readback optics must be finite")
    if readback.focal_length_mm <= 0.0:
        raise ValueError("camera readback focal length must be positive")
    if readback.horizontal_aperture_mm <= 0.0 or readback.vertical_aperture_mm <= 0.0:
        raise ValueError("camera readback apertures must be positive")
    near_m, far_m = readback.clipping_range_m
    if near_m <= 0.0 or far_m <= near_m:
        raise ValueError("camera readback clipping range must be positive and increasing")

    width = float(readback.width_px)
    height = float(readback.height_px)
    fx = width * readback.focal_length_mm / readback.horizontal_aperture_mm
    fy = height * readback.focal_length_mm / readback.vertical_aperture_mm
    cx = width * (
        0.5
        - readback.horizontal_aperture_offset_mm
        / readback.horizontal_aperture_mm
    )
    cy = height * (
        0.5
        + readback.vertical_aperture_offset_mm
        / readback.vertical_aperture_mm
    )
    horizontal_fov_deg = math.degrees(
        2.0
        * math.atan(readback.horizontal_aperture_mm / (2.0 * readback.focal_length_mm))
    )
    vertical_fov_deg = math.degrees(
        2.0
        * math.atan(readback.vertical_aperture_mm / (2.0 * readback.focal_length_mm))
    )
    return PinholeCalibration(
        horizontal_fov_deg=horizontal_fov_deg,
        vertical_fov_deg=vertical_fov_deg,
        k_row_major=(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0),
        p_row_major=(fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0),
    )


def assert_profile_matches_readback(
    profile: IsaacCameraProfile,
    readback: IsaacCameraApiReadback,
    *,
    absolute_tolerance: float = 1e-5,
) -> None:
    """Fail closed when authored profile facts differ from active API readback."""

    if (readback.width_px, readback.height_px) != (
        profile.capture.width_px,
        profile.capture.height_px,
    ):
        raise ValueError("camera resolution readback differs from profile")
    if readback.projection != profile.optics.projection:
        raise ValueError("camera projection readback differs from profile")
    expected = (
        profile.optics.focal_length_mm,
        profile.optics.horizontal_aperture_mm,
        profile.optics.vertical_aperture_mm,
        profile.optics.horizontal_aperture_offset_mm,
        profile.optics.vertical_aperture_offset_mm,
        *profile.optics.clipping_range_m,
    )
    actual = (
        readback.focal_length_mm,
        readback.horizontal_aperture_mm,
        readback.vertical_aperture_mm,
        readback.horizontal_aperture_offset_mm,
        readback.vertical_aperture_offset_mm,
        *readback.clipping_range_m,
    )
    if not np.allclose(actual, expected, rtol=0.0, atol=absolute_tolerance):
        raise ValueError("camera optics readback differs from profile")
    calibration = derive_pinhole_calibration(readback)
    if not math.isclose(
        calibration.horizontal_fov_deg,
        profile.optics.horizontal_fov_deg,
        rel_tol=0.0,
        abs_tol=absolute_tolerance,
    ):
        raise ValueError("derived horizontal FOV differs from profile")


def rgba_to_rgb8(
    rgba: NDArray[np.generic],
    profile: IsaacCameraProfile,
) -> NDArray[np.uint8]:
    """Validate Isaac's RGBA contract and return contiguous ROS ``rgb8``."""

    if rgba.dtype != np.uint8 or rgba.shape != profile.rgb.source_shape:
        raise ValueError(
            "RGB payload must match the profile's uint8 RGBA source contract"
        )
    return np.ascontiguousarray(rgba[..., :3])


def depth_to_32fc1(
    depth: NDArray[np.generic],
    profile: IsaacCameraProfile,
) -> NDArray[np.float32]:
    """Validate optical-Z depth and canonicalize all invalid samples to qNaN."""

    if depth.dtype != np.float32 or depth.shape != profile.depth.source_shape:
        raise ValueError(
            "depth payload must match the profile's float32 optical-Z source contract"
        )
    output = np.ascontiguousarray(depth).copy()
    invalid = np.logical_or(~np.isfinite(output), output <= 0.0)
    output.view(np.uint32)[invalid] = CANONICAL_QUIET_NAN_U32
    return output


__all__ = [
    "CANONICAL_QUIET_NAN_U32",
    "IsaacCameraApiReadback",
    "PinholeCalibration",
    "assert_profile_matches_readback",
    "depth_to_32fc1",
    "derive_pinhole_calibration",
    "rgba_to_rgb8",
]
