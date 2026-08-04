"""Strict configuration for one synthetic Isaac camera render product."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Self, cast

from .common import (
    finite_number,
    finite_vector,
    positive_number,
    require_exact_mapping,
    require_sequence,
    require_string,
    validate_identifier,
)


ISAAC_CAMERA_PROFILE_SCHEMA = "wujihand.isaac_camera_profile.v1"
SIMULATION_ONLY_CAMERA_WARNING = (
    "SIMULATION ONLY: synthetic 140-degree HFOV; not a physical RealSense "
    "D405 specification or calibration."
)
SYNTHETIC_WIDE_ANGLE_CLASSIFICATION = (
    "synthetic_wide_angle_140_simulation_only"
)


def _boolean(value: object, *, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{field} must be a boolean")
    return value


def _positive_integer(value: object, *, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{field} must be a positive integer")
    return value


def _non_negative_integer(value: object, *, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    return value


def _string_tuple(value: object, *, field: str) -> tuple[str, ...]:
    items = require_sequence(value, field=field)
    return tuple(
        require_string(item, field=f"{field}[{index}]")
        for index, item in enumerate(items)
    )


@dataclass(frozen=True, slots=True)
class IsaacCameraCaptureSpec:
    width_px: int
    height_px: int
    rate_hz: float
    warmup_frames: int

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {"width_px", "height_px", "rate_hz", "warmup_frames"}
            ),
            field=field,
        )
        return cls(
            width_px=_positive_integer(
                data["width_px"], field=f"{field}.width_px"
            ),
            height_px=_positive_integer(
                data["height_px"], field=f"{field}.height_px"
            ),
            rate_hz=positive_number(data["rate_hz"], field=f"{field}.rate_hz"),
            warmup_frames=_non_negative_integer(
                data["warmup_frames"], field=f"{field}.warmup_frames"
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "width_px": self.width_px,
            "height_px": self.height_px,
            "rate_hz": self.rate_hz,
            "warmup_frames": self.warmup_frames,
        }


@dataclass(frozen=True, slots=True)
class IsaacCameraOpticsSpec:
    projection: str
    lens_model: str
    horizontal_fov_deg: float
    focal_length_mm: float
    horizontal_aperture_mm: float
    vertical_aperture_mm: float
    horizontal_aperture_offset_mm: float
    vertical_aperture_offset_mm: float
    clipping_range_m: tuple[float, float]
    pixel_geometry_convention: str
    distortion_model: str
    distortion_coefficients: tuple[float, float, float, float, float]

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "projection",
                    "lens_model",
                    "horizontal_fov_deg",
                    "focal_length_mm",
                    "horizontal_aperture_mm",
                    "vertical_aperture_mm",
                    "horizontal_aperture_offset_mm",
                    "vertical_aperture_offset_mm",
                    "clipping_range_m",
                    "pixel_geometry_convention",
                    "distortion_model",
                    "distortion_coefficients",
                }
            ),
            field=field,
        )
        clipping_range = finite_vector(
            data["clipping_range_m"], size=2, field=f"{field}.clipping_range_m"
        )
        if clipping_range[0] <= 0.0 or clipping_range[1] <= clipping_range[0]:
            raise ValueError(
                f"{field}.clipping_range_m must be positive and increasing"
            )
        distortion = finite_vector(
            data["distortion_coefficients"],
            size=5,
            field=f"{field}.distortion_coefficients",
        )
        return cls(
            projection=require_string(
                data["projection"], field=f"{field}.projection"
            ),
            lens_model=require_string(
                data["lens_model"], field=f"{field}.lens_model"
            ),
            horizontal_fov_deg=positive_number(
                data["horizontal_fov_deg"],
                field=f"{field}.horizontal_fov_deg",
            ),
            focal_length_mm=positive_number(
                data["focal_length_mm"], field=f"{field}.focal_length_mm"
            ),
            horizontal_aperture_mm=positive_number(
                data["horizontal_aperture_mm"],
                field=f"{field}.horizontal_aperture_mm",
            ),
            vertical_aperture_mm=positive_number(
                data["vertical_aperture_mm"],
                field=f"{field}.vertical_aperture_mm",
            ),
            horizontal_aperture_offset_mm=finite_number(
                data["horizontal_aperture_offset_mm"],
                field=f"{field}.horizontal_aperture_offset_mm",
            ),
            vertical_aperture_offset_mm=finite_number(
                data["vertical_aperture_offset_mm"],
                field=f"{field}.vertical_aperture_offset_mm",
            ),
            clipping_range_m=cast(tuple[float, float], clipping_range),
            pixel_geometry_convention=require_string(
                data["pixel_geometry_convention"],
                field=f"{field}.pixel_geometry_convention",
            ),
            distortion_model=require_string(
                data["distortion_model"], field=f"{field}.distortion_model"
            ),
            distortion_coefficients=cast(
                tuple[float, float, float, float, float], distortion
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "projection": self.projection,
            "lens_model": self.lens_model,
            "horizontal_fov_deg": self.horizontal_fov_deg,
            "focal_length_mm": self.focal_length_mm,
            "horizontal_aperture_mm": self.horizontal_aperture_mm,
            "vertical_aperture_mm": self.vertical_aperture_mm,
            "horizontal_aperture_offset_mm": self.horizontal_aperture_offset_mm,
            "vertical_aperture_offset_mm": self.vertical_aperture_offset_mm,
            "clipping_range_m": list(self.clipping_range_m),
            "pixel_geometry_convention": self.pixel_geometry_convention,
            "distortion_model": self.distortion_model,
            "distortion_coefficients": list(self.distortion_coefficients),
        }


@dataclass(frozen=True, slots=True)
class IsaacCameraPayloadSpec:
    annotator: str
    source_shape: tuple[int, ...]
    source_dtype: str
    source_encoding: str
    source_no_hit_encoding: str
    output_encoding: str
    invalid_value_policy: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "annotator",
                    "source_shape",
                    "source_dtype",
                    "source_encoding",
                    "source_no_hit_encoding",
                    "output_encoding",
                    "invalid_value_policy",
                }
            ),
            field=field,
        )
        raw_shape = require_sequence(data["source_shape"], field=f"{field}.source_shape")
        shape = tuple(
            _positive_integer(item, field=f"{field}.source_shape[{index}]")
            for index, item in enumerate(raw_shape)
        )
        if not shape:
            raise ValueError(f"{field}.source_shape must not be empty")
        return cls(
            annotator=require_string(data["annotator"], field=f"{field}.annotator"),
            source_shape=shape,
            source_dtype=require_string(
                data["source_dtype"], field=f"{field}.source_dtype"
            ),
            source_encoding=require_string(
                data["source_encoding"], field=f"{field}.source_encoding"
            ),
            source_no_hit_encoding=require_string(
                data["source_no_hit_encoding"],
                field=f"{field}.source_no_hit_encoding",
            ),
            output_encoding=require_string(
                data["output_encoding"], field=f"{field}.output_encoding"
            ),
            invalid_value_policy=require_string(
                data["invalid_value_policy"],
                field=f"{field}.invalid_value_policy",
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "annotator": self.annotator,
            "source_shape": list(self.source_shape),
            "source_dtype": self.source_dtype,
            "source_encoding": self.source_encoding,
            "source_no_hit_encoding": self.source_no_hit_encoding,
            "output_encoding": self.output_encoding,
            "invalid_value_policy": self.invalid_value_policy,
        }


@dataclass(frozen=True, slots=True)
class IsaacCameraScheduleSpec:
    physics_substeps_per_capture: int
    control_ticks_per_capture: int
    capture_phase: str
    completed_frame_identity: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "physics_substeps_per_capture",
                    "control_ticks_per_capture",
                    "capture_phase",
                    "completed_frame_identity",
                }
            ),
            field=field,
        )
        return cls(
            physics_substeps_per_capture=_positive_integer(
                data["physics_substeps_per_capture"],
                field=f"{field}.physics_substeps_per_capture",
            ),
            control_ticks_per_capture=_positive_integer(
                data["control_ticks_per_capture"],
                field=f"{field}.control_ticks_per_capture",
            ),
            capture_phase=require_string(
                data["capture_phase"], field=f"{field}.capture_phase"
            ),
            completed_frame_identity=require_string(
                data["completed_frame_identity"],
                field=f"{field}.completed_frame_identity",
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "physics_substeps_per_capture": self.physics_substeps_per_capture,
            "control_ticks_per_capture": self.control_ticks_per_capture,
            "capture_phase": self.capture_phase,
            "completed_frame_identity": self.completed_frame_identity,
        }


@dataclass(frozen=True, slots=True)
class IsaacCameraProfile:
    """Isaac-only synthetic camera facts; never a physical camera calibration."""

    profile_id: str
    simulation_only: bool
    warning: str
    projection_classification: str
    capture: IsaacCameraCaptureSpec
    optics: IsaacCameraOpticsSpec
    rgb: IsaacCameraPayloadSpec
    depth: IsaacCameraPayloadSpec
    schedule: IsaacCameraScheduleSpec
    assumptions: tuple[str, ...]

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "Isaac camera profile") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "schema",
                    "profile_id",
                    "simulation_only",
                    "warning",
                    "projection_classification",
                    "capture",
                    "optics",
                    "payloads",
                    "schedule",
                    "assumptions",
                }
            ),
            field=field,
        )
        if data["schema"] != ISAAC_CAMERA_PROFILE_SCHEMA:
            raise ValueError(
                f"{field}.schema must be {ISAAC_CAMERA_PROFILE_SCHEMA!r}"
            )
        payloads = require_exact_mapping(
            data["payloads"],
            expected=frozenset({"rgb", "depth"}),
            field=f"{field}.payloads",
        )
        profile = cls(
            profile_id=validate_identifier(
                data["profile_id"], field=f"{field}.profile_id"
            ),
            simulation_only=_boolean(
                data["simulation_only"], field=f"{field}.simulation_only"
            ),
            warning=require_string(data["warning"], field=f"{field}.warning"),
            projection_classification=require_string(
                data["projection_classification"],
                field=f"{field}.projection_classification",
            ),
            capture=IsaacCameraCaptureSpec.from_mapping(
                data["capture"], field=f"{field}.capture"
            ),
            optics=IsaacCameraOpticsSpec.from_mapping(
                data["optics"], field=f"{field}.optics"
            ),
            rgb=IsaacCameraPayloadSpec.from_mapping(
                payloads["rgb"], field=f"{field}.payloads.rgb"
            ),
            depth=IsaacCameraPayloadSpec.from_mapping(
                payloads["depth"], field=f"{field}.payloads.depth"
            ),
            schedule=IsaacCameraScheduleSpec.from_mapping(
                data["schedule"], field=f"{field}.schedule"
            ),
            assumptions=_string_tuple(
                data["assumptions"], field=f"{field}.assumptions"
            ),
        )
        profile._validate_contract(field=field)
        return profile

    def _validate_contract(self, *, field: str) -> None:
        if not self.simulation_only:
            raise ValueError(f"{field}.simulation_only must be true")
        if self.warning != SIMULATION_ONLY_CAMERA_WARNING:
            raise ValueError(f"{field}.warning must retain the simulation-only warning")
        if self.projection_classification != SYNTHETIC_WIDE_ANGLE_CLASSIFICATION:
            raise ValueError(
                f"{field}.projection_classification must identify the synthetic 140-degree lens"
            )
        if self.optics.projection != "perspective" or self.optics.lens_model != "pinhole":
            raise ValueError(f"{field}.optics must use perspective pinhole projection")
        if self.optics.horizontal_fov_deg != 140.0:
            raise ValueError(f"{field}.optics.horizontal_fov_deg must be 140")
        if self.optics.pixel_geometry_convention != "usd_aperture_to_ros_raster_v1":
            raise ValueError(f"{field}.optics pixel geometry convention is unsupported")
        if self.optics.distortion_model != "plumb_bob" or any(
            self.optics.distortion_coefficients
        ):
            raise ValueError(f"{field}.optics must declare zero plumb_bob distortion")
        expected_rgb_shape = (self.capture.height_px, self.capture.width_px, 4)
        expected_depth_shape = (self.capture.height_px, self.capture.width_px)
        if self.rgb.source_shape != expected_rgb_shape:
            raise ValueError(f"{field}.payloads.rgb source shape differs from capture")
        if self.depth.source_shape != expected_depth_shape:
            raise ValueError(f"{field}.payloads.depth source shape differs from capture")
        if self.rgb.source_no_hit_encoding != "none":
            raise ValueError(f"{field}.payloads.rgb must not declare a no-hit encoding")
        if self.depth.source_no_hit_encoding != "positive_infinity":
            raise ValueError(
                f"{field}.payloads.depth no-hit encoding must match the Isaac API spike"
            )

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": ISAAC_CAMERA_PROFILE_SCHEMA,
            "profile_id": self.profile_id,
            "simulation_only": self.simulation_only,
            "warning": self.warning,
            "projection_classification": self.projection_classification,
            "capture": self.capture.to_mapping(),
            "optics": self.optics.to_mapping(),
            "payloads": {
                "rgb": self.rgb.to_mapping(),
                "depth": self.depth.to_mapping(),
            },
            "schedule": self.schedule.to_mapping(),
            "assumptions": list(self.assumptions),
        }


__all__ = [
    "ISAAC_CAMERA_PROFILE_SCHEMA",
    "SIMULATION_ONLY_CAMERA_WARNING",
    "SYNTHETIC_WIDE_ANGLE_CLASSIFICATION",
    "IsaacCameraCaptureSpec",
    "IsaacCameraOpticsSpec",
    "IsaacCameraPayloadSpec",
    "IsaacCameraProfile",
    "IsaacCameraScheduleSpec",
]
