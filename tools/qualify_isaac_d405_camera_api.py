#!/usr/bin/env python3
"""Qualify the Isaac 6.0.1 API contract for the synthetic D405 wrist camera."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import traceback
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = Path(
    "configs/profiles/isaac_d405_synthetic_wide_angle_140_v1.yaml"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--marker-pixel-tolerance", type=float, default=2.0)
    parser.add_argument("--marker-depth-tolerance-m", type=float, default=0.03)
    return parser


def _as_numpy(value: object) -> np.ndarray:
    payload = value
    if isinstance(payload, dict) and "data" in payload:
        payload = payload["data"]
    if hasattr(payload, "numpy"):
        payload = payload.numpy()  # type: ignore[union-attr]
    return np.asarray(payload).copy()


def _sha256_array(value: np.ndarray) -> str:
    contiguous = np.ascontiguousarray(value)
    return hashlib.sha256(contiguous.view(np.uint8)).hexdigest()


def _scalar(value: object) -> float:
    return float(_as_numpy(value).reshape(-1)[0])


def _jsonable(value: object) -> object:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _marker_centroid(
    rgba: np.ndarray,
    *,
    dominant_channel: int,
) -> tuple[float, float]:
    color = rgba[..., :3].astype(np.int16)
    other_channels = [index for index in range(3) if index != dominant_channel]
    mask = np.logical_and(
        color[..., dominant_channel] > 80,
        color[..., dominant_channel]
        > np.max(color[..., other_channels], axis=-1) + 30,
    )
    rows, columns = np.nonzero(mask)
    if columns.size < 4:
        maxima = np.max(rgba[..., :3], axis=(0, 1)).tolist()
        center = rgba[rgba.shape[0] // 2, rgba.shape[1] // 2].tolist()
        raise RuntimeError(
            "rendered marker color mask is empty: "
            f"rgb_max={maxima}, center_rgba={center}"
        )
    return float(columns.mean()), float(rows.mean())


def _depth_invalid_mask(depth: np.ndarray) -> np.ndarray:
    return np.logical_or.reduce(
        (
            ~np.isfinite(depth),
            depth <= 0.0,
            depth == np.finfo(np.float32).max,
        )
    )


def _no_hit_summary(depth: np.ndarray) -> dict[str, int]:
    return {
        "positive_infinity": int(np.count_nonzero(np.isposinf(depth))),
        "negative_infinity": int(np.count_nonzero(np.isneginf(depth))),
        "nan": int(np.count_nonzero(np.isnan(depth))),
        "float32_max": int(
            np.count_nonzero(depth == np.finfo(np.float32).max)
        ),
        "nonpositive_finite": int(
            np.count_nonzero(np.logical_and(np.isfinite(depth), depth <= 0.0))
        ),
    }


def main() -> int:
    args = build_parser().parse_args()
    if args.marker_pixel_tolerance <= 0.0:
        raise ValueError("--marker-pixel-tolerance must be positive")
    if args.marker_depth_tolerance_m <= 0.0:
        raise ValueError("--marker-depth-tolerance-m must be positive")

    project_root = args.project_root.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(project_root / "src"))

    from wujihand.adapters.simulation.isaac_camera import (
        IsaacCameraApiReadback,
        assert_profile_matches_readback,
        derive_pinhole_calibration,
    )
    from wujihand.runtime.config_repository import ConfigRepository

    repository = ConfigRepository(project_root)
    profile = repository.load_isaac_camera_profile(args.profile)

    from isaacsim import SimulationApp  # type: ignore[import-not-found]

    simulation_app = SimulationApp(
        {"headless": True, "anti_aliasing": 0, "width": 640, "height": 480}
    )
    writer = None
    phase = "importing Isaac APIs"
    try:
        import carb  # type: ignore[import-not-found]
        import isaacsim.core.experimental.utils.app as app_utils  # type: ignore[import-not-found]
        import omni.replicator.core as rep  # type: ignore[import-not-found]
        import omni.timeline  # type: ignore[import-not-found]
        import omni.usd  # type: ignore[import-not-found]
        from isaacsim.sensors.experimental.rtx import (  # type: ignore[import-not-found]
            CameraSensor,
            RtxCamera,
        )
        from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade  # type: ignore[import-not-found]

        phase = "creating qualification stage"
        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        if stage is None:
            raise RuntimeError("Isaac failed to create a USD stage")
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.Xform.Define(stage, "/World")
        carb.settings.get_settings().set(
            "/rtx/hydra/supportMultiTickRate", True
        )

        marker = UsdGeom.Cube.Define(stage, "/World/Marker")
        marker.GetSizeAttr().Set(0.1)
        marker_translation = marker.AddTranslateOp()
        material = UsdShade.Material.Define(stage, "/World/MarkerMaterial")
        shader = UsdShade.Shader.Define(stage, "/World/MarkerMaterial/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        diffuse = shader.CreateInput(
            "diffuseColor", Sdf.ValueTypeNames.Color3f
        )
        emissive = shader.CreateInput(
            "emissiveColor", Sdf.ValueTypeNames.Color3f
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        UsdShade.MaterialBindingAPI(marker.GetPrim()).Bind(material)
        dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
        dome.CreateIntensityAttr(500.0)
        marker_translation.Set(Gf.Vec3d(0.25, 0.0, -1.0))
        diffuse.Set(Gf.Vec3f(1.0, 0.0, 0.0))
        emissive.Set(Gf.Vec3f(1.0, 0.0, 0.0))

        phase = "creating RTX camera and render product"
        rtx_camera = RtxCamera(
            "/World/Camera", tick_rate=profile.capture.rate_hz
        )
        camera = rtx_camera.camera
        sensor = CameraSensor(
            rtx_camera,
            resolution=(profile.capture.height_px, profile.capture.width_px),
            annotators=None,
        )

        # SIMULATION ONLY: synthetic 140-degree HFOV; not a physical RealSense
        # D405 specification or calibration.
        camera.set_focal_lengths([profile.optics.focal_length_mm])
        camera.set_apertures(
            horizontal_apertures=[profile.optics.horizontal_aperture_mm],
            vertical_apertures=[profile.optics.vertical_aperture_mm],
        )
        camera.set_aperture_offsets(
            horizontal_offsets=[profile.optics.horizontal_aperture_offset_mm],
            vertical_offsets=[profile.optics.vertical_aperture_offset_mm],
        )
        camera.set_projections([profile.optics.projection])
        camera.set_clipping_ranges(
            near_distances=[profile.optics.clipping_range_m[0]],
            far_distances=[profile.optics.clipping_range_m[1]],
        )

        phase = "reading back camera optics"
        horizontal_apertures, vertical_apertures = camera.get_apertures()
        horizontal_offsets, vertical_offsets = camera.get_aperture_offsets()
        near_distances, far_distances = camera.get_clipping_ranges()
        render_resolution = tuple(
            int(item) for item in sensor.render_product.GetResolutionAttr().Get()
        )
        readback = IsaacCameraApiReadback(
            width_px=render_resolution[0],
            height_px=render_resolution[1],
            projection=camera.get_projections()[0],
            focal_length_mm=_scalar(camera.get_focal_lengths()),
            horizontal_aperture_mm=_scalar(horizontal_apertures),
            vertical_aperture_mm=_scalar(vertical_apertures),
            horizontal_aperture_offset_mm=_scalar(horizontal_offsets),
            vertical_aperture_offset_mm=_scalar(vertical_offsets),
            clipping_range_m=(
                _scalar(near_distances),
                _scalar(far_distances),
            ),
        )
        assert_profile_matches_readback(profile, readback, absolute_tolerance=1e-4)
        calibration = derive_pinhole_calibration(readback)

        class SynchronousCameraProbeWriter(rep.Writer):
            """Collect images and Replicator's shared completed-frame identity."""

            def __init__(self) -> None:
                self.version = "1.0.0"
                self.annotators = ["rgb", profile.depth.annotator]
                self.records: list[dict[str, Any]] = []

            def write(self, data: dict[str, Any]) -> None:
                self.records.append(data)

            def write_metadata(self) -> None:
                pass

        phase = "attaching synchronized writer"
        writer = SynchronousCameraProbeWriter()
        render_product_path = str(sensor.render_product.GetPath())
        writer.attach(render_product_path)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        phase = "warming up render product"
        warmup_start = len(writer.records)
        for _ in range(profile.capture.warmup_frames * 4):
            app_utils.update_app()
            if len(writer.records) - warmup_start >= profile.capture.warmup_frames:
                break
        if len(writer.records) - warmup_start < profile.capture.warmup_frames:
            raise RuntimeError("CameraSensor did not produce all configured warm-up frames")
        writer.records.clear()

        states = (
            {
                "name": "red_near_right",
                "translation_usd_m": (0.25, 0.0, -1.0),
                "color": (1.0, 0.0, 0.0),
                "dominant_channel": 0,
            },
            {
                "name": "red_far_left",
                "translation_usd_m": (-0.25, 0.0, -2.0),
                "color": (1.0, 0.0, 0.0),
                "dominant_channel": 0,
            },
        )
        captures: list[dict[str, object]] = []
        for capture_index, state in enumerate(states):
            phase = f"capturing state {capture_index}"
            translation = state["translation_usd_m"]
            color = state["color"]
            assert isinstance(translation, tuple)
            assert isinstance(color, tuple)
            marker_translation.Set(Gf.Vec3d(*translation))
            diffuse.Set(Gf.Vec3f(*color))
            emissive.Set(Gf.Vec3f(*color))
            fx = calibration.k_row_major[0]
            fy = calibration.k_row_major[4]
            cx = calibration.k_row_major[2]
            cy = calibration.k_row_major[5]
            x_ros = float(translation[0])
            y_ros = -float(translation[1])
            z_ros = -float(translation[2])
            expected_u = fx * x_ros / z_ros + cx
            expected_v = fy * y_ros / z_ros + cy
            expected_front_depth_m = z_ros - 0.05

            before_records = len(writer.records)
            update_count = 0
            candidate_cursor = before_records
            discarded_completed_frames: list[dict[str, object]] = []
            selected: tuple[
                dict[str, Any],
                np.ndarray,
                np.ndarray,
                tuple[int, int],
                float,
                float,
                float,
                float,
            ] | None = None
            for update_count in range(1, 33):
                app_utils.update_app()
                for candidate in writer.records[candidate_cursor:]:
                    candidate_rgba = _as_numpy(candidate["rgb"])
                    candidate_depth = _as_numpy(candidate[profile.depth.annotator])
                    if candidate_rgba.ndim == 1:
                        candidate_rgba = candidate_rgba.reshape(profile.rgb.source_shape)
                    if candidate_depth.ndim == 1:
                        candidate_depth = candidate_depth.reshape(
                            profile.depth.source_shape
                        )
                    candidate_reference = candidate.get("reference_time")
                    if (
                        not isinstance(candidate_reference, tuple)
                        or len(candidate_reference) != 2
                        or int(candidate_reference[1]) == 0
                    ):
                        raise RuntimeError(
                            "writer record has no valid completed-frame identity"
                        )
                    try:
                        candidate_u, candidate_v = _marker_centroid(
                            candidate_rgba,
                            dominant_channel=int(state["dominant_channel"]),
                        )
                        candidate_pixel_error = float(
                            np.hypot(
                                candidate_u - expected_u,
                                candidate_v - expected_v,
                            )
                        )
                        sample_u = int(round(candidate_u))
                        sample_v = int(round(candidate_v))
                        candidate_marker_depth_m = float(
                            candidate_depth[sample_v, sample_u]
                        )
                        candidate_depth_error_m = abs(
                            candidate_marker_depth_m - expected_front_depth_m
                        )
                        candidate_error: str | None = None
                    except RuntimeError as exc:
                        candidate_u = candidate_v = float("nan")
                        candidate_pixel_error = float("inf")
                        candidate_marker_depth_m = float("nan")
                        candidate_depth_error_m = float("inf")
                        candidate_error = str(exc)
                    if (
                        candidate_pixel_error <= args.marker_pixel_tolerance
                        and candidate_depth_error_m
                        <= args.marker_depth_tolerance_m
                    ):
                        selected = (
                            candidate,
                            candidate_rgba,
                            candidate_depth,
                            candidate_reference,
                            candidate_u,
                            candidate_v,
                            candidate_pixel_error,
                            candidate_marker_depth_m,
                        )
                        break
                    discarded_completed_frames.append(
                        {
                            "reference_time": candidate_reference,
                            "marker_centroid_uv_px": (candidate_u, candidate_v),
                            "marker_pixel_error_px": candidate_pixel_error,
                            "marker_depth_m": candidate_marker_depth_m,
                            "marker_depth_error_m": candidate_depth_error_m,
                            "error": candidate_error,
                        }
                    )
                candidate_cursor = len(writer.records)
                if selected is not None:
                    break
            if selected is None:
                raise RuntimeError(
                    "no completed writer frame matched the authored marker state: "
                    f"before={before_records}, after={len(writer.records)}, "
                    f"timeline_playing={timeline.is_playing()}, "
                    f"discarded={discarded_completed_frames}"
                )
            (
                record,
                writer_rgba,
                writer_depth,
                reference_time,
                centroid_u,
                centroid_v,
                pixel_error,
                marker_depth_m,
            ) = selected

            expected_rgba = profile.rgb.source_shape
            expected_depth = profile.depth.source_shape
            if writer_rgba.shape != expected_rgba or writer_rgba.dtype != np.uint8:
                raise RuntimeError(
                    f"writer RGBA contract mismatch: {writer_rgba.shape}/{writer_rgba.dtype}"
                )
            if writer_depth.shape != expected_depth or writer_depth.dtype != np.float32:
                raise RuntimeError(
                    f"writer depth contract mismatch: {writer_depth.shape}/{writer_depth.dtype}"
                )
            writer_rgba_hash = _sha256_array(writer_rgba)
            writer_depth_hash = _sha256_array(writer_depth)
            writer_no_hit_summary = _no_hit_summary(writer_depth)
            writer_depth_invalid = _depth_invalid_mask(writer_depth)

            writer_no_hit_count = int(np.count_nonzero(writer_depth_invalid))
            if writer_no_hit_count == 0:
                raise RuntimeError("unable to identify Isaac depth no-hit encoding")
            observed_no_hit_encodings = {
                key for key, count in writer_no_hit_summary.items() if count > 0
            }
            if observed_no_hit_encodings != {
                profile.depth.source_no_hit_encoding
            }:
                raise RuntimeError(
                    "depth no-hit encoding differs from the versioned profile: "
                    f"observed={sorted(observed_no_hit_encodings)}"
                )
            swh_frame_number = _jsonable(record.get("swhFrameNumber"))
            if not isinstance(swh_frame_number, int):
                raise RuntimeError("writer record has no integer swhFrameNumber")

            from PIL import Image  # type: ignore[import-not-found]

            Image.fromarray(writer_rgba, mode="RGBA").save(
                output_dir / f"capture_{capture_index:02d}_{state['name']}_rgba.png"
            )
            np.save(
                output_dir / f"capture_{capture_index:02d}_{state['name']}_depth.npy",
                writer_depth,
                allow_pickle=False,
            )
            captures.append(
                {
                    "camera_frame_index": capture_index,
                    "app_updates_until_completed_frame": update_count,
                    "discarded_stale_completed_frames": discarded_completed_frames,
                    "state": state,
                    "reference_time": reference_time,
                    "swh_frame_number": swh_frame_number,
                    "writer_keys": sorted(record),
                    "rgba_shape": writer_rgba.shape,
                    "rgba_dtype": str(writer_rgba.dtype),
                    "rgba_sha256": writer_rgba_hash,
                    "alpha_values": np.unique(writer_rgba[..., 3]),
                    "depth_shape": writer_depth.shape,
                    "depth_dtype": str(writer_depth.dtype),
                    "depth_sha256": writer_depth_hash,
                    "no_hit_source_encoding": writer_no_hit_summary,
                    "no_hit_count": writer_no_hit_count,
                    "marker_centroid_uv_px": (centroid_u, centroid_v),
                    "expected_marker_uv_px": (expected_u, expected_v),
                    "marker_pixel_error_px": pixel_error,
                    "marker_depth_m": marker_depth_m,
                    "expected_marker_front_depth_m": expected_front_depth_m,
                }
            )

        reference_times = [tuple(item["reference_time"]) for item in captures]
        if len(set(reference_times)) != len(reference_times):
            raise RuntimeError("completed-frame reference times are not unique")
        swh_frame_numbers = [int(item["swh_frame_number"]) for item in captures]
        report = {
            "status": "pass",
            "isaac_distribution_version": "6.0.1.0",
            "profile_path": repository.project_relative(
                args.profile, field="camera profile"
            ),
            "profile": profile.to_mapping(),
            "render_product_path": render_product_path,
            "completed_frame_identity": {
                "source": "Replicator Writer reference_time",
                "synchronization": (
                    "CameraSensor 30 Hz tick driven by committed timeline app updates; "
                    "one completed writer callback bundles rgb and "
                    "distance_to_image_plane before the app update returns"
                ),
                "reference_times": reference_times,
                "swh_frame_numbers_observed": swh_frame_numbers,
                "swh_frame_number_usable_as_identity": False,
            },
            "api_readback": {
                "width_px": readback.width_px,
                "height_px": readback.height_px,
                "projection": readback.projection,
                "focal_length_mm": readback.focal_length_mm,
                "horizontal_aperture_mm": readback.horizontal_aperture_mm,
                "vertical_aperture_mm": readback.vertical_aperture_mm,
                "horizontal_aperture_offset_mm": readback.horizontal_aperture_offset_mm,
                "vertical_aperture_offset_mm": readback.vertical_aperture_offset_mm,
                "clipping_range_m": readback.clipping_range_m,
            },
            "derived_calibration": {
                "horizontal_fov_deg": calibration.horizontal_fov_deg,
                "vertical_fov_deg": calibration.vertical_fov_deg,
                "k_row_major": calibration.k_row_major,
                "p_row_major": calibration.p_row_major,
                "pixel_geometry_convention": profile.optics.pixel_geometry_convention,
            },
            "captures": captures,
        }
        report_path = output_dir / "report.json"
        report_path.write_text(
            json.dumps(_jsonable(report), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"D405 CAMERA API QUALIFICATION PASS: report={report_path}", flush=True)
        return 0
    except BaseException as exc:
        failure_path = output_dir / "failure.json"
        failure_path.write_text(
            json.dumps(
                {
                    "status": "fail",
                    "phase": phase,
                    "exception_type": type(exc).__name__,
                    "message": str(exc),
                    "traceback": traceback.format_exc(),
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        print(
            f"D405 CAMERA API QUALIFICATION FAIL: phase={phase} report={failure_path}",
            file=sys.stderr,
            flush=True,
        )
        if writer is not None:
            try:
                writer.detach()
            except Exception as detach_exc:
                print(
                    f"writer detach warning: {detach_exc}",
                    file=sys.stderr,
                    flush=True,
                )
            writer = None
        simulation_app.close(exit_code=1)
        return 1
    finally:
        if writer is not None:
            try:
                writer.detach()
            except Exception as exc:
                print(f"writer detach warning: {exc}", file=sys.stderr, flush=True)
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
