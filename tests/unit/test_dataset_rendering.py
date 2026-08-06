from __future__ import annotations

import json
from pathlib import Path
import struct
import zlib

import numpy as np
import pytest

from test_dataset_release import _episode
from wujihand.adapters.simulation.isaac_camera import (
    IsaacCameraApiReadback,
    derive_pinhole_calibration,
)
from wujihand.dataset.alignment import build_exact_30hz_alignment
from wujihand.dataset.artifacts import write_alignment_artifact
from wujihand.dataset.camera import (
    DatasetCameraRuntimeInventory,
    load_dataset_camera_projections,
)
from wujihand.dataset.normalized import write_normalized_episode_artifact
from wujihand.dataset.profile import MiniDatasetProfile, load_mini_dataset_profile
from wujihand.dataset.release import validate_episode_release
from wujihand.dataset.release_artifact import write_release_decision_artifact
from wujihand.dataset.rendering import CompletedRgbRender, render_exact_triview
from wujihand.domain.dataset_recording import SimulationStateFrame
from wujihand.runtime.isaac_dataset_rgb_renderer import (
    _RawRgbFrame,
    _ReplayClock,
    _deduplicate_reference_records,
)


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


def _retime(frame: SimulationStateFrame, simulation_time_s: float) -> SimulationStateFrame:
    return SimulationStateFrame.create(
        run_id=frame.run_id,
        episode_id=frame.episode_id,
        control_index=frame.control_index,
        tick_id=frame.tick_id,
        phase=frame.phase,
        simulation_time_s=simulation_time_s,
        physics_boundary_index=frame.physics_boundary_index,
        q54_rad=frame.q54_rad,
        qdot54_rad_s=frame.qdot54_rad_s,
        rigid_bodies=frame.rigid_bodies,
        kinematic_links=frame.kinematic_links,
        expected_rigid_body_count=frame.expected_rigid_body_count,
        expected_kinematic_link_count=frame.expected_kinematic_link_count,
    )


def _chunk(kind: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + kind
        + payload
        + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
    )


def _rgb_png(value: int) -> bytes:
    row = bytes((value, 20, 30)) * 640
    raw = b"".join(b"\x00" + row for _ in range(480))
    return (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", struct.pack(">IIBBBBB", 640, 480, 8, 2, 0, 0, 0))
        + _chunk(b"IDAT", zlib.compress(raw, level=1))
        + _chunk(b"IEND", b"")
    )


class _Backend:
    renderer_identity = "fixture-fixed-state-renderer-v1"
    renderer_backend = "RayTracedLighting"
    lighting_identity = "session_workcell_authored_lighting"
    color_space = "isaac_rgb_annotator_srgb"
    motion_blur_enabled = False

    def __init__(self, profile: MiniDatasetProfile, *, advance: bool = False) -> None:
        projections = load_dataset_camera_projections(ROOT, profile)
        inventories = []
        for projection in projections:
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
        self.camera_runtime_inventories = tuple(inventories)
        self.camera_hashes = {item.camera_id: item.profile_sha256 for item in inventories}
        self.current: SimulationStateFrame | None = None
        self.time = 9.0
        self.advance = advance
        self.injected_frame_indices: list[int] = []

    @property
    def simulation_time_s(self) -> float:
        return self.time

    def inject_pre_action_state(
        self,
        frame: SimulationStateFrame,
        *,
        dataset_frame_index: int,
    ) -> str:
        self.current = frame
        self.time = dataset_frame_index / 30.0
        self.injected_frame_indices.append(dataset_frame_index)
        return frame.payload_digest_sha256

    def render_rgb(self, *, camera_id: str, dataset_frame_index: int) -> CompletedRgbRender:
        assert self.current is not None
        if self.advance:
            self.time += 1.0 / 120.0
        return CompletedRgbRender(
            camera_id=camera_id,
            payload_png=_rgb_png(40 + dataset_frame_index),
            completed_frame_identity=f"frame-{dataset_frame_index}-{camera_id}",
            camera_profile_sha256=self.camera_hashes[camera_id],
            parent_frame_id=f"{camera_id}_parent",
            world_from_parent_row_major=IDENTITY,
            world_from_camera_optical_row_major=IDENTITY,
        )


def _prepared_run(tmp_path: Path) -> tuple[Path, MiniDatasetProfile]:
    facts, q54 = _episode()
    run_root = tmp_path / facts.run_id
    run_root.mkdir()
    profile = load_mini_dataset_profile(ROOT, DATASET_PROFILE)
    (run_root / "manifest.json").write_text(
        json.dumps(
            {
                "deployment": {
                    "deployment_hash": "1" * 64,
                    "session_hash": "2" * 64,
                    "assembly_sha256": "3" * 64,
                    "workcell_sha256": "4" * 64,
                },
                "dataset": {"profile_sha256": profile.file_sha256},
            }
        )
        + "\n",
        encoding="utf-8",
    )
    write_normalized_episode_artifact(run_root, facts)
    write_release_decision_artifact(run_root, validate_episode_release(facts, q54))
    alignment = build_exact_30hz_alignment(tick.transition for tick in facts.ticks)
    write_alignment_artifact(run_root, alignment)
    return run_root, profile


def test_fixed_state_renderer_publishes_exact_three_camera_grid(tmp_path: Path) -> None:
    run_root, profile = _prepared_run(tmp_path)

    artifact = render_exact_triview(
        run_root,
        dataset_profile=profile,
        backend=_Backend(profile),
    )

    assert artifact.frame_count == 2
    assert len(artifact.frames) == 6
    assert artifact.frame(1, "right_wrist_rgb").source_control_index == 2
    assert artifact.payload(artifact.frame(0, "scene_rgb")).read_bytes().startswith(b"\x89PNG")


def test_fixed_state_renderer_rejects_simulation_advance_atomically(tmp_path: Path) -> None:
    run_root, profile = _prepared_run(tmp_path)

    with pytest.raises(ValueError, match="advanced simulation time"):
        render_exact_triview(
            run_root,
            dataset_profile=profile,
            backend=_Backend(profile, advance=True),
        )

    assert not (run_root / "derived" / "vision").exists()
    assert not tuple((run_root / "derived").glob(".vision-*"))


def test_replay_clock_uses_frame_index_and_fixed_source_origin() -> None:
    facts, _ = _episode()
    first = _retime(facts.ticks[0].pre_action_frame, 3.58333352)
    second = _retime(facts.ticks[2].pre_action_frame, 3.61666685)
    clock = _ReplayClock(physics_hz=120.0, policy_fps=30.0)

    assert clock.observe(first, dataset_frame_index=0) == 0.0
    assert clock.observe(second, dataset_frame_index=1) == pytest.approx(1.0 / 30.0)
    assert clock.source_physics_grid_origin == 430


def test_replay_clock_rejects_source_time_outside_five_microseconds() -> None:
    facts, _ = _episode()
    frame = _retime(facts.ticks[0].pre_action_frame, 5.1e-6)

    with pytest.raises(RuntimeError, match="120 Hz source grid"):
        _ReplayClock(physics_hz=120.0, policy_fps=30.0).observe(
            frame,
            dataset_frame_index=0,
        )


def test_rgb_callback_deduplication_is_reference_and_payload_exact() -> None:
    payload = np.zeros((480, 640, 4), dtype=np.uint8)
    first = _RawRgbFrame(reference_time=(7, 30), rgba=payload)
    duplicate = _RawRgbFrame(reference_time=(7, 30), rgba=payload.copy())

    selected = _deduplicate_reference_records(
        (first, duplicate),
        camera_id="scene_rgb",
    )

    assert selected.reference_time == (7, 30)
    conflicting = payload.copy()
    conflicting[0, 0, 0] = 1
    with pytest.raises(RuntimeError, match="conflicting payloads"):
        _deduplicate_reference_records(
            (first, _RawRgbFrame(reference_time=(7, 30), rgba=conflicting)),
            camera_id="scene_rgb",
        )
    with pytest.raises(RuntimeError, match="multiple RGB references"):
        _deduplicate_reference_records(
            (first, _RawRgbFrame(reference_time=(8, 30), rgba=payload)),
            camera_id="scene_rgb",
        )
    newer = _RawRgbFrame(reference_time=(8, 30), rgba=payload)
    assert _deduplicate_reference_records(
        (first, duplicate, newer),
        camera_id="scene_rgb",
        after=(7, 30),
    ).reference_time == (8, 30)
