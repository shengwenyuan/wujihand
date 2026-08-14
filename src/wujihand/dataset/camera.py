"""Frozen synthetic RGB camera profiles and runtime readback closure for 008."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Final, cast

from wujihand.adapters.simulation.isaac_camera import (
    IsaacCameraApiReadback,
    PinholeCalibration,
    derive_pinhole_calibration,
)
from wujihand.integrity import sha256_file
from wujihand.runtime.yaml_loader import load_yaml_strict
from wujihand.specs import IsaacCameraOpticsSpec, IsaacCameraProfile

from .profile import DatasetCameraRole, MiniDatasetProfile


DATASET_SCENE_CAMERA_PROFILE_SCHEMA: Final = "wujihand.dataset_camera_profile.v1"
DATASET_CAMERA_RUNTIME_INVENTORY_SCHEMA: Final = (
    "wujihand.dataset_camera_runtime_inventory.v2"
)
OFFLINE_CAPTURE_PHASE: Final = "offline_fixed_state_pre_action"


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(Mapping[str, object], value)


def _exact(
    value: object,
    *,
    keys: frozenset[str],
    field: str,
) -> Mapping[str, object]:
    result = _mapping(value, field=field)
    actual = frozenset(result)
    if actual != keys:
        raise ValueError(
            f"{field} keys differ: "
            f"missing={sorted(keys - actual)}, unexpected={sorted(actual - keys)}"
        )
    return result


def _string(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{field} must be a non-blank trimmed string")
    return value


def _positive_integer(value: object, *, field: str) -> int:
    if type(value) is not int or value <= 0:
        raise ValueError(f"{field} must be a positive integer")
    return value


def _positive_number(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a positive finite number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{field} must be a positive finite number")
    return result


def _finite_number(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be a finite number")
    return result


def _sequence(value: object, *, field: str) -> Sequence[object]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{field} must be a sequence")
    return cast(Sequence[object], value)


def _finite_tuple(value: object, *, size: int, field: str) -> tuple[float, ...]:
    result = tuple(
        _finite_number(item, field=f"{field}[{index}]")
        for index, item in enumerate(_sequence(value, field=field))
    )
    if len(result) != size:
        raise ValueError(f"{field} must contain exactly {size} values")
    return result


def _optional_digest(value: object, *, field: str) -> str | None:
    if value is None:
        return None
    result = _string(value, field=field)
    if len(result) != 64 or any(char not in "0123456789abcdef" for char in result):
        raise ValueError(f"{field} must be a lowercase SHA-256 or null")
    return result


def _rigid_matrix(value: object, *, field: str) -> tuple[float, ...]:
    result = _finite_tuple(value, size=16, field=field)
    if any(
        not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-8)
        for actual, expected in zip(result[12:16], (0.0, 0.0, 0.0, 1.0), strict=True)
    ):
        raise ValueError(f"{field} must be an affine SE(3) matrix")
    rotation = (
        (result[0], result[1], result[2]),
        (result[4], result[5], result[6]),
        (result[8], result[9], result[10]),
    )
    for row in rotation:
        if not math.isclose(
            sum(item * item for item in row),
            1.0,
            rel_tol=0.0,
            abs_tol=1e-6,
        ):
            raise ValueError(f"{field} rotation must be orthonormal")
    for first, second in ((0, 1), (0, 2), (1, 2)):
        dot = sum(rotation[first][index] * rotation[second][index] for index in range(3))
        if not math.isclose(dot, 0.0, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError(f"{field} rotation must be orthonormal")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1]
        * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2]
        * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(f"{field} rotation determinant must be +1")
    return result


@dataclass(frozen=True, slots=True)
class DatasetRgbCameraProjection:
    """One pinned RGB-only synthetic projection; never a physical calibration."""

    logical_id: str
    feature_key: str
    carrier_identity: str
    profile_id: str
    profile_path: str
    profile_sha256: str
    simulation_only: bool
    physical_calibration_compatible: bool
    warning: str
    projection_classification: str
    width_px: int
    height_px: int
    rate_hz: float
    warmup_frames: int
    optics: IsaacCameraOpticsSpec
    source_shape: tuple[int, int, int]
    source_dtype: str
    source_encoding: str
    output_encoding: str
    capture_phase: str
    completed_frame_identity_contract: str

    def __post_init__(self) -> None:
        if not self.simulation_only or self.physical_calibration_compatible:
            raise ValueError("dataset cameras must remain simulation-only and non-physical")
        if "SIMULATION ONLY" not in self.warning:
            raise ValueError("dataset camera warning must retain SIMULATION ONLY")
        if (self.width_px, self.height_px, self.rate_hz) != (640, 480, 30.0):
            raise ValueError("dataset camera projection must be 640x480 at 30 Hz")
        if self.source_shape != (480, 640, 4):
            raise ValueError("dataset camera RGB source must be 480x640 RGBA")
        if (
            self.source_dtype,
            self.source_encoding,
            self.output_encoding,
            self.capture_phase,
        ) != ("uint8", "RGBA", "rgb8", OFFLINE_CAPTURE_PHASE):
            raise ValueError("dataset camera RGB/capture contract differs")
        if self.optics.projection != "perspective" or self.optics.lens_model != "pinhole":
            raise ValueError("dataset camera must use perspective pinhole projection")
        if self.optics.pixel_geometry_convention != "usd_aperture_to_ros_raster_v1":
            raise ValueError("dataset camera raster convention differs")
        if self.optics.distortion_model != "plumb_bob" or any(
            self.optics.distortion_coefficients
        ):
            raise ValueError("dataset camera must declare zero plumb_bob distortion")
        expected_hfov = 90.0 if self.logical_id == "scene_rgb" else 140.0
        if not math.isclose(
            self.optics.horizontal_fov_deg,
            expected_hfov,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError(f"{self.logical_id} synthetic horizontal FOV differs")


@dataclass(frozen=True, slots=True)
class DatasetCameraRuntimeInventory:
    """Active Camera/RenderProduct values read back after Isaac materialization."""

    camera_id: str
    carrier_identity: str
    profile_id: str
    profile_path: str
    profile_sha256: str
    warning: str
    camera_prim_path: str
    render_product_path: str
    parent_prim_path: str
    parent_frame_id: str
    camera_frame_id: str
    optical_frame_id: str
    parent_from_camera_optical_row_major: tuple[float, ...]
    mount_visual_sha256: str | None
    camera_visual_sha256: str | None
    generation_report_sha256: str | None
    readback: IsaacCameraApiReadback
    calibration: PinholeCalibration

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str,
    ) -> DatasetCameraRuntimeInventory:
        data = _exact(
            value,
            keys=frozenset(
                {
                    "schema",
                    "camera_id",
                    "carrier_identity",
                    "profile_id",
                    "profile_path",
                    "profile_sha256",
                    "simulation_only",
                    "physical_calibration_compatible",
                    "warning",
                    "camera_prim_path",
                    "render_product_path",
                    "parent_prim_path",
                    "parent_frame_id",
                    "camera_frame_id",
                    "optical_frame_id",
                    "matrix_convention",
                    "parent_from_camera_optical_row_major",
                    "static_artifact_hashes",
                    "api_readback",
                    "derived_calibration",
                }
            ),
            field=field,
        )
        if data["schema"] != DATASET_CAMERA_RUNTIME_INVENTORY_SCHEMA:
            raise ValueError(f"{field}.schema differs")
        if data["simulation_only"] is not True or data[
            "physical_calibration_compatible"
        ] is not False:
            raise ValueError(f"{field} physical/simulation classification differs")
        warning = _string(data["warning"], field=f"{field}.warning")
        if "SIMULATION ONLY" not in warning:
            raise ValueError(f"{field}.warning must retain SIMULATION ONLY")
        digest = _string(data["profile_sha256"], field=f"{field}.profile_sha256")
        if len(digest) != 64 or any(char not in "0123456789abcdef" for char in digest):
            raise ValueError(f"{field}.profile_sha256 must be a lowercase SHA-256")
        if data["matrix_convention"] != "row_major_T_parent_from_ros_optical_v1":
            raise ValueError(f"{field}.matrix_convention differs")
        parent_from_camera = _rigid_matrix(
            data["parent_from_camera_optical_row_major"],
            field=f"{field}.parent_from_camera_optical_row_major",
        )
        static_hashes = _exact(
            data["static_artifact_hashes"],
            keys=frozenset(
                {
                    "mount_visual_sha256",
                    "camera_visual_sha256",
                    "generation_report_sha256",
                }
            ),
            field=f"{field}.static_artifact_hashes",
        )
        readback_data = _exact(
            data["api_readback"],
            keys=frozenset(
                {
                    "width_px",
                    "height_px",
                    "projection",
                    "focal_length_mm",
                    "horizontal_aperture_mm",
                    "vertical_aperture_mm",
                    "horizontal_aperture_offset_mm",
                    "vertical_aperture_offset_mm",
                    "clipping_range_m",
                }
            ),
            field=f"{field}.api_readback",
        )
        clipping = tuple(
            _positive_number(item, field=f"{field}.api_readback.clipping_range_m[{index}]")
            for index, item in enumerate(
                _sequence(
                    readback_data["clipping_range_m"],
                    field=f"{field}.api_readback.clipping_range_m",
                )
            )
        )
        if len(clipping) != 2 or clipping[1] <= clipping[0]:
            raise ValueError(f"{field}.api_readback.clipping_range_m differs")
        readback = IsaacCameraApiReadback(
            width_px=_positive_integer(
                readback_data["width_px"], field=f"{field}.api_readback.width_px"
            ),
            height_px=_positive_integer(
                readback_data["height_px"], field=f"{field}.api_readback.height_px"
            ),
            projection=_string(
                readback_data["projection"], field=f"{field}.api_readback.projection"
            ),
            focal_length_mm=_positive_number(
                readback_data["focal_length_mm"],
                field=f"{field}.api_readback.focal_length_mm",
            ),
            horizontal_aperture_mm=_positive_number(
                readback_data["horizontal_aperture_mm"],
                field=f"{field}.api_readback.horizontal_aperture_mm",
            ),
            vertical_aperture_mm=_positive_number(
                readback_data["vertical_aperture_mm"],
                field=f"{field}.api_readback.vertical_aperture_mm",
            ),
            horizontal_aperture_offset_mm=_finite_number(
                readback_data["horizontal_aperture_offset_mm"],
                field=f"{field}.api_readback.horizontal_aperture_offset_mm",
            ),
            vertical_aperture_offset_mm=_finite_number(
                readback_data["vertical_aperture_offset_mm"],
                field=f"{field}.api_readback.vertical_aperture_offset_mm",
            ),
            clipping_range_m=clipping,
        )
        calibration_data = _exact(
            data["derived_calibration"],
            keys=frozenset(
                {
                    "horizontal_fov_deg",
                    "vertical_fov_deg",
                    "k_row_major",
                    "p_row_major",
                    "r_row_major",
                    "distortion_model",
                    "distortion_coefficients",
                    "pixel_geometry_convention",
                }
            ),
            field=f"{field}.derived_calibration",
        )
        expected = derive_pinhole_calibration(readback)
        k = _finite_tuple(
            calibration_data["k_row_major"],
            size=9,
            field=f"{field}.derived_calibration.k_row_major",
        )
        p = _finite_tuple(
            calibration_data["p_row_major"],
            size=12,
            field=f"{field}.derived_calibration.p_row_major",
        )
        distortion = _finite_tuple(
            calibration_data["distortion_coefficients"],
            size=5,
            field=f"{field}.derived_calibration.distortion_coefficients",
        )
        rectification = _finite_tuple(
            calibration_data["r_row_major"],
            size=9,
            field=f"{field}.derived_calibration.r_row_major",
        )
        reported = (
            _positive_number(
                calibration_data["horizontal_fov_deg"],
                field=f"{field}.derived_calibration.horizontal_fov_deg",
            ),
            _positive_number(
                calibration_data["vertical_fov_deg"],
                field=f"{field}.derived_calibration.vertical_fov_deg",
            ),
            *k,
            *p,
        )
        wanted = (
            expected.horizontal_fov_deg,
            expected.vertical_fov_deg,
            *expected.k_row_major,
            *expected.p_row_major,
        )
        if (
            calibration_data["distortion_model"] != "plumb_bob"
            or any(distortion)
            or rectification != (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
            or calibration_data["pixel_geometry_convention"]
            != "usd_aperture_to_ros_raster_v1"
            or any(
                not math.isclose(actual, target, rel_tol=0.0, abs_tol=1e-6)
                for actual, target in zip(reported, wanted, strict=True)
            )
        ):
            raise ValueError(f"{field}.derived_calibration differs from API readback")
        return cls(
            camera_id=_string(data["camera_id"], field=f"{field}.camera_id"),
            carrier_identity=_string(
                data["carrier_identity"], field=f"{field}.carrier_identity"
            ),
            profile_id=_string(data["profile_id"], field=f"{field}.profile_id"),
            profile_path=_string(data["profile_path"], field=f"{field}.profile_path"),
            profile_sha256=digest,
            warning=warning,
            camera_prim_path=_string(
                data["camera_prim_path"], field=f"{field}.camera_prim_path"
            ),
            render_product_path=_string(
                data["render_product_path"], field=f"{field}.render_product_path"
            ),
            parent_prim_path=_string(
                data["parent_prim_path"], field=f"{field}.parent_prim_path"
            ),
            parent_frame_id=_string(
                data["parent_frame_id"], field=f"{field}.parent_frame_id"
            ),
            camera_frame_id=_string(
                data["camera_frame_id"], field=f"{field}.camera_frame_id"
            ),
            optical_frame_id=_string(
                data["optical_frame_id"], field=f"{field}.optical_frame_id"
            ),
            parent_from_camera_optical_row_major=parent_from_camera,
            mount_visual_sha256=_optional_digest(
                static_hashes["mount_visual_sha256"],
                field=f"{field}.static_artifact_hashes.mount_visual_sha256",
            ),
            camera_visual_sha256=_optional_digest(
                static_hashes["camera_visual_sha256"],
                field=f"{field}.static_artifact_hashes.camera_visual_sha256",
            ),
            generation_report_sha256=_optional_digest(
                static_hashes["generation_report_sha256"],
                field=f"{field}.static_artifact_hashes.generation_report_sha256",
            ),
            readback=readback,
            calibration=PinholeCalibration(
                horizontal_fov_deg=reported[0],
                vertical_fov_deg=reported[1],
                k_row_major=k,
                p_row_major=p,
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": DATASET_CAMERA_RUNTIME_INVENTORY_SCHEMA,
            "camera_id": self.camera_id,
            "carrier_identity": self.carrier_identity,
            "profile_id": self.profile_id,
            "profile_path": self.profile_path,
            "profile_sha256": self.profile_sha256,
            "simulation_only": True,
            "physical_calibration_compatible": False,
            "warning": self.warning,
            "camera_prim_path": self.camera_prim_path,
            "render_product_path": self.render_product_path,
            "parent_prim_path": self.parent_prim_path,
            "parent_frame_id": self.parent_frame_id,
            "camera_frame_id": self.camera_frame_id,
            "optical_frame_id": self.optical_frame_id,
            "matrix_convention": "row_major_T_parent_from_ros_optical_v1",
            "parent_from_camera_optical_row_major": list(
                self.parent_from_camera_optical_row_major
            ),
            "static_artifact_hashes": {
                "mount_visual_sha256": self.mount_visual_sha256,
                "camera_visual_sha256": self.camera_visual_sha256,
                "generation_report_sha256": self.generation_report_sha256,
            },
            "api_readback": {
                "width_px": self.readback.width_px,
                "height_px": self.readback.height_px,
                "projection": self.readback.projection,
                "focal_length_mm": self.readback.focal_length_mm,
                "horizontal_aperture_mm": self.readback.horizontal_aperture_mm,
                "vertical_aperture_mm": self.readback.vertical_aperture_mm,
                "horizontal_aperture_offset_mm": (
                    self.readback.horizontal_aperture_offset_mm
                ),
                "vertical_aperture_offset_mm": self.readback.vertical_aperture_offset_mm,
                "clipping_range_m": list(self.readback.clipping_range_m),
            },
            "derived_calibration": {
                "horizontal_fov_deg": self.calibration.horizontal_fov_deg,
                "vertical_fov_deg": self.calibration.vertical_fov_deg,
                "k_row_major": list(self.calibration.k_row_major),
                "p_row_major": list(self.calibration.p_row_major),
                "r_row_major": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                "distortion_model": "plumb_bob",
                "distortion_coefficients": [0.0] * 5,
                "pixel_geometry_convention": "usd_aperture_to_ros_raster_v1",
            },
        }


def _load_scene_projection(
    role: DatasetCameraRole,
    document: object,
) -> DatasetRgbCameraProjection:
    field = f"camera profile {role.logical_id}"
    data = _exact(
        document,
        keys=frozenset(
            {
                "schema",
                "profile_id",
                "status",
                "logical_role",
                "carrier_identity",
                "simulation_only",
                "physical_calibration_compatible",
                "warning",
                "projection_classification",
                "capture",
                "optics",
                "payload",
                "schedule",
                "provenance",
            }
        ),
        field=field,
    )
    if data["schema"] != DATASET_SCENE_CAMERA_PROFILE_SCHEMA:
        raise ValueError(f"{field}.schema differs")
    if data["logical_role"] != role.logical_id:
        raise ValueError(f"{field}.logical_role differs")
    if data["carrier_identity"] != "intel_realsense_d435i":
        raise ValueError(f"{field}.carrier_identity differs")
    if data["simulation_only"] is not True or data["physical_calibration_compatible"] is not False:
        raise ValueError(f"{field} physical/simulation classification differs")
    capture = _exact(
        data["capture"],
        keys=frozenset({"width_px", "height_px", "rate_hz", "warmup_frames"}),
        field=f"{field}.capture",
    )
    payload = _exact(
        data["payload"],
        keys=frozenset(
            {
                "annotator",
                "source_shape",
                "source_dtype",
                "source_encoding",
                "output_encoding",
            }
        ),
        field=f"{field}.payload",
    )
    schedule = _exact(
        data["schedule"],
        keys=frozenset(
            {"capture_phase", "control_ticks_per_capture", "completed_frame_identity"}
        ),
        field=f"{field}.schedule",
    )
    if payload["annotator"] != "rgb":
        raise ValueError(f"{field} RGB schedule differs")
    shape = tuple(
        _positive_integer(item, field=f"{field}.payload.source_shape[{index}]")
        for index, item in enumerate(
            _sequence(payload["source_shape"], field=f"{field}.payload.source_shape")
        )
    )
    if len(shape) != 3:
        raise ValueError(f"{field}.payload.source_shape must have three dimensions")
    return DatasetRgbCameraProjection(
        logical_id=role.logical_id,
        feature_key=role.feature_key,
        carrier_identity=role.carrier_identity,
        profile_id=_string(data["profile_id"], field=f"{field}.profile_id"),
        profile_path=role.profile.path,
        profile_sha256=role.profile.sha256,
        simulation_only=True,
        physical_calibration_compatible=False,
        warning=_string(data["warning"], field=f"{field}.warning"),
        projection_classification=_string(
            data["projection_classification"],
            field=f"{field}.projection_classification",
        ),
        width_px=_positive_integer(capture["width_px"], field=f"{field}.capture.width_px"),
        height_px=_positive_integer(
            capture["height_px"], field=f"{field}.capture.height_px"
        ),
        rate_hz=_positive_number(capture["rate_hz"], field=f"{field}.capture.rate_hz"),
        warmup_frames=_positive_integer(
            capture["warmup_frames"], field=f"{field}.capture.warmup_frames"
        ),
        optics=IsaacCameraOpticsSpec.from_mapping(
            data["optics"], field=f"{field}.optics"
        ),
        source_shape=shape,
        source_dtype=_string(payload["source_dtype"], field=f"{field}.payload.source_dtype"),
        source_encoding=_string(
            payload["source_encoding"], field=f"{field}.payload.source_encoding"
        ),
        output_encoding=_string(
            payload["output_encoding"], field=f"{field}.payload.output_encoding"
        ),
        capture_phase=_string(
            schedule["capture_phase"], field=f"{field}.schedule.capture_phase"
        ),
        completed_frame_identity_contract=_string(
            schedule["completed_frame_identity"],
            field=f"{field}.schedule.completed_frame_identity",
        ),
    )


def _load_wrist_projection(
    role: DatasetCameraRole,
    document: object,
) -> DatasetRgbCameraProjection:
    profile = IsaacCameraProfile.from_mapping(
        document,
        field=f"camera profile {role.logical_id}",
    )
    rgb = profile.rgb
    if (
        rgb.annotator,
        rgb.source_dtype,
        rgb.source_encoding,
        rgb.output_encoding,
    ) != ("rgb", "uint8", "RGBA", "rgb8"):
        raise ValueError(f"camera profile {role.logical_id} RGB contract differs")
    if profile.capture.warmup_frames <= 0:
        raise ValueError(f"camera profile {role.logical_id} requires positive warm-up")
    # The D405 source profile also serves the 007 online RGB+depth path.  The
    # 008 camera-set whitelist deliberately consumes only its RGB projection
    # and fixes capture to offline pre-action rendering; this is not a launch
    # option and must never be interpreted as physical D405 calibration.
    return DatasetRgbCameraProjection(
        logical_id=role.logical_id,
        feature_key=role.feature_key,
        carrier_identity=role.carrier_identity,
        profile_id=profile.profile_id,
        profile_path=role.profile.path,
        profile_sha256=role.profile.sha256,
        simulation_only=profile.simulation_only,
        physical_calibration_compatible=False,
        warning=profile.warning,
        projection_classification=profile.projection_classification,
        width_px=profile.capture.width_px,
        height_px=profile.capture.height_px,
        rate_hz=profile.capture.rate_hz,
        warmup_frames=profile.capture.warmup_frames,
        optics=profile.optics,
        source_shape=cast(tuple[int, int, int], profile.rgb.source_shape),
        source_dtype=profile.rgb.source_dtype,
        source_encoding=profile.rgb.source_encoding,
        output_encoding=profile.rgb.output_encoding,
        capture_phase=OFFLINE_CAPTURE_PHASE,
        completed_frame_identity_contract=(
            "completed_render_product_reference_time_v1"
        ),
    )


def load_dataset_camera_projections(
    project_root: str | Path,
    dataset_profile: MiniDatasetProfile,
) -> tuple[DatasetRgbCameraProjection, ...]:
    """Resolve all three hash-pinned profiles without any runtime camera option."""

    root = Path(project_root).resolve()
    if not root.is_dir():
        raise ValueError("project root must be a directory")
    result: list[DatasetRgbCameraProjection] = []
    for role in dataset_profile.cameras:
        path = (root / role.profile.path).resolve()
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise ValueError("dataset camera profile escapes project root") from exc
        if not path.is_file() or sha256_file(path) != role.profile.sha256:
            raise ValueError(f"dataset camera profile hash differs: {role.logical_id}")
        document = load_yaml_strict(path.read_text(encoding="utf-8"))
        mapping = _mapping(document, field=f"camera profile {role.logical_id}")
        if mapping.get("profile_id") != role.profile.expected_id:
            raise ValueError(f"dataset camera profile ID differs: {role.logical_id}")
        projection = (
            _load_scene_projection(role, document)
            if mapping.get("schema") == DATASET_SCENE_CAMERA_PROFILE_SCHEMA
            else _load_wrist_projection(role, document)
        )
        schedule = _mapping(
            mapping.get("schedule"),
            field=f"camera profile {role.logical_id}.schedule",
        )
        expected_control_ticks = dataset_profile.control_hz // int(projection.rate_hz)
        if (
            dataset_profile.control_hz % int(projection.rate_hz) != 0
            or schedule.get("control_ticks_per_capture") != expected_control_ticks
        ):
            raise ValueError(f"dataset camera control schedule differs: {role.logical_id}")
        if "physics_substeps_per_capture" in schedule:
            expected_physics_steps = dataset_profile.physics_hz // int(projection.rate_hz)
            if (
                dataset_profile.physics_hz % int(projection.rate_hz) != 0
                or schedule["physics_substeps_per_capture"] != expected_physics_steps
            ):
                raise ValueError(f"dataset camera physics schedule differs: {role.logical_id}")
        result.append(projection)
    expected = tuple(camera.logical_id for camera in dataset_profile.cameras)
    if tuple(item.logical_id for item in result) != expected:
        raise ValueError("resolved camera projection order differs")
    return tuple(result)


def assert_dataset_projection_matches_readback(
    projection: DatasetRgbCameraProjection,
    readback: IsaacCameraApiReadback,
    *,
    absolute_tolerance: float = 1e-4,
) -> PinholeCalibration:
    """Fail closed on active Camera/RenderProduct drift and return derived K/P."""

    if (readback.width_px, readback.height_px) != (
        projection.width_px,
        projection.height_px,
    ):
        raise ValueError("dataset camera resolution readback differs")
    if readback.projection != projection.optics.projection:
        raise ValueError("dataset camera projection readback differs")
    actual = (
        readback.focal_length_mm,
        readback.horizontal_aperture_mm,
        readback.vertical_aperture_mm,
        readback.horizontal_aperture_offset_mm,
        readback.vertical_aperture_offset_mm,
        *readback.clipping_range_m,
    )
    expected = (
        projection.optics.focal_length_mm,
        projection.optics.horizontal_aperture_mm,
        projection.optics.vertical_aperture_mm,
        projection.optics.horizontal_aperture_offset_mm,
        projection.optics.vertical_aperture_offset_mm,
        *projection.optics.clipping_range_m,
    )
    if any(
        not math.isclose(value, wanted, rel_tol=0.0, abs_tol=absolute_tolerance)
        for value, wanted in zip(actual, expected, strict=True)
    ):
        raise ValueError("dataset camera optics readback differs")
    calibration = derive_pinhole_calibration(readback)
    if not math.isclose(
        calibration.horizontal_fov_deg,
        projection.optics.horizontal_fov_deg,
        rel_tol=0.0,
        abs_tol=absolute_tolerance,
    ):
        raise ValueError("dataset camera derived horizontal FOV differs")
    return calibration


__all__ = [
    "DATASET_SCENE_CAMERA_PROFILE_SCHEMA",
    "OFFLINE_CAPTURE_PHASE",
    "DatasetCameraRuntimeInventory",
    "DatasetRgbCameraProjection",
    "assert_dataset_projection_matches_readback",
    "load_dataset_camera_projections",
]
