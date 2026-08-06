from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct
import zlib

import numpy as np
import pytest

from dataset_camera_fixture import vision_provenance
from wujihand.adapters.simulation.isaac_camera import (
    IsaacCameraApiReadback,
    derive_pinhole_calibration,
)
from wujihand.dataset.camera import (
    DatasetCameraRuntimeInventory,
    load_dataset_camera_projections,
)
from wujihand.dataset.profile import load_mini_dataset_profile
from wujihand.dataset.rendering import encode_rgb8_png
from wujihand.dataset.vision import (
    CAMERA_IDS,
    VISION_ARTIFACT_SCHEMA,
    VISION_FRAME_SCHEMA,
    load_vision_artifact,
)


ROOT = Path(__file__).parents[2]
DATASET_PROFILE = "configs/profiles/isaac_nero_hand2_triview_q54_mini_dataset_v1.yaml"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _rewrite_checksums(root: Path) -> None:
    material = sorted(
        path
        for path in root.rglob("*")
        if path.is_file() and path.name != "checksums.sha256"
    )
    (root / "checksums.sha256").write_text(
        "".join(
            f"{_sha256(path)}  {path.relative_to(root).as_posix()}\n"
            for path in material
        ),
        encoding="utf-8",
    )


def _chunk(kind: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + kind
        + payload
        + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
    )


def _short_decoded_png() -> bytes:
    return (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", struct.pack(">IIBBBBB", 640, 480, 8, 2, 0, 0, 0))
        + _chunk(b"IDAT", zlib.compress(b"\x00short"))
        + _chunk(b"IEND", b"")
    )


def _vision(tmp_path: Path) -> tuple[Path, str]:
    root = tmp_path / "vision"
    root.mkdir()
    alignment_digest = "b" * 64
    dataset_profile = load_mini_dataset_profile(ROOT, DATASET_PROFILE)
    projections = load_dataset_camera_projections(ROOT, dataset_profile)
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
                parent_from_camera_optical_row_major=(
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
                ),
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
    profile_hashes = {item.camera_id: item.profile_sha256 for item in inventories}
    provenance = vision_provenance(dataset_profile, tuple(inventories))
    records = []
    for frame_index in range(2):
        state_digest = hashlib.sha256(f"state-{frame_index}".encode()).hexdigest()
        for camera_id in CAMERA_IDS:
            relative = Path(camera_id) / f"{frame_index:06d}.png"
            payload = root / relative
            payload.parent.mkdir(exist_ok=True)
            payload.write_bytes(
                encode_rgb8_png(
                    np.full((480, 640, 3), 20 + frame_index, dtype=np.uint8)
                )
            )
            records.append(
                {
                    "schema": VISION_FRAME_SCHEMA,
                    "run_id": "episode-001",
                    "collection_id": provenance.collection_id,
                    "provenance_sha256": provenance.digest_sha256,
                    "camera_id": camera_id,
                    "dataset_frame_index": frame_index,
                    "source_control_index": 10 + 2 * frame_index,
                    "source_tick_id": 10 + 2 * frame_index,
                    "phase": "pre_action",
                    "simulation_time_s": frame_index / 30.0,
                    "source_state_digest": state_digest,
                    "payload_path": relative.as_posix(),
                    "payload_sha256": _sha256(payload),
                    "width_px": 640,
                    "height_px": 480,
                    "encoding": "rgb8",
                    "camera_profile_sha256": profile_hashes[camera_id],
                    "completed_frame_identity": f"render-{frame_index}-{camera_id}",
                    "parent_frame_id": f"{camera_id}_parent",
                    "world_from_parent_row_major": [
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
                    ],
                    "world_from_camera_optical_row_major": [
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
                    ],
                }
            )
    (root / "frame_index.jsonl").write_text(
        "".join(json.dumps(item, sort_keys=True) + "\n" for item in records),
        encoding="utf-8",
    )
    (root / "manifest.json").write_text(
        json.dumps(
            {
                "schema": VISION_ARTIFACT_SCHEMA,
                "run_id": "episode-001",
                "alignment_digest_sha256": alignment_digest,
                "camera_ids": list(CAMERA_IDS),
                "frame_count": 2,
                "renderer_identity": "offline-fixed-state-v1",
                "provenance": provenance.to_mapping(),
                "camera_runtime_inventories": [
                    item.to_mapping() for item in inventories
                ],
            }
        )
        + "\n",
        encoding="utf-8",
    )
    _rewrite_checksums(root)
    return root, alignment_digest


def test_vision_artifact_requires_exact_three_camera_source_state_closure(
    tmp_path: Path,
) -> None:
    root, alignment_digest = _vision(tmp_path)

    artifact = load_vision_artifact(
        root,
        expected_run_id="episode-001",
        expected_alignment_digest=alignment_digest,
    )

    assert artifact.frame_count == 2
    assert len(artifact.frames) == 6
    frame = artifact.frame(1, "right_wrist_rgb")
    assert frame.source_control_index == 12
    assert artifact.payload(frame).read_bytes().startswith(b"\x89PNG")


def test_vision_artifact_rejects_payload_tampering(tmp_path: Path) -> None:
    root, alignment_digest = _vision(tmp_path)
    (root / "scene_rgb" / "000000.png").write_bytes(b"tampered")

    with pytest.raises(ValueError, match="checksum differs"):
        load_vision_artifact(
            root,
            expected_run_id="episode-001",
            expected_alignment_digest=alignment_digest,
        )


def test_vision_artifact_rejects_alignment_mismatch(tmp_path: Path) -> None:
    root, _ = _vision(tmp_path)

    with pytest.raises(ValueError, match="alignment digests differ"):
        load_vision_artifact(
            root,
            expected_run_id="episode-001",
            expected_alignment_digest="d" * 64,
        )


def test_vision_artifact_rejects_non_rigid_camera_transform(tmp_path: Path) -> None:
    root, alignment_digest = _vision(tmp_path)
    index_path = root / "frame_index.jsonl"
    rows = [json.loads(line) for line in index_path.read_text(encoding="utf-8").splitlines()]
    rows[0]["world_from_camera_optical_row_major"][0] = -1.0
    index_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    _rewrite_checksums(root)

    with pytest.raises(ValueError, match="determinant must be \\+1"):
        load_vision_artifact(
            root,
            expected_run_id="episode-001",
            expected_alignment_digest=alignment_digest,
        )


def test_vision_artifact_rejects_parent_static_world_extrinsic_mismatch(
    tmp_path: Path,
) -> None:
    root, alignment_digest = _vision(tmp_path)
    index_path = root / "frame_index.jsonl"
    rows = [json.loads(line) for line in index_path.read_text(encoding="utf-8").splitlines()]
    rows[0]["world_from_parent_row_major"][3] = 0.1
    index_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    _rewrite_checksums(root)

    with pytest.raises(ValueError, match="extrinsic closure differs"):
        load_vision_artifact(
            root,
            expected_run_id="episode-001",
            expected_alignment_digest=alignment_digest,
        )


def test_vision_artifact_rejects_png_that_cannot_decode_to_full_rgb_raster(
    tmp_path: Path,
) -> None:
    root, alignment_digest = _vision(tmp_path)
    payload_path = root / "scene_rgb" / "000000.png"
    payload_path.write_bytes(_short_decoded_png())
    index_path = root / "frame_index.jsonl"
    rows = [json.loads(line) for line in index_path.read_text(encoding="utf-8").splitlines()]
    rows[0]["payload_sha256"] = _sha256(payload_path)
    index_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    _rewrite_checksums(root)

    with pytest.raises(ValueError, match="decoded raster is invalid"):
        load_vision_artifact(
            root,
            expected_run_id="episode-001",
            expected_alignment_digest=alignment_digest,
        )


def test_vision_artifact_rejects_black_frame_and_stale_payload_across_states(
    tmp_path: Path,
) -> None:
    root, alignment_digest = _vision(tmp_path)
    index_path = root / "frame_index.jsonl"
    rows = [json.loads(line) for line in index_path.read_text(encoding="utf-8").splitlines()]
    black = root / "scene_rgb" / "000000.png"
    black.write_bytes(encode_rgb8_png(np.zeros((480, 640, 3), dtype=np.uint8)))
    rows[0]["payload_sha256"] = _sha256(black)
    index_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    _rewrite_checksums(root)

    with pytest.raises(ValueError, match="all black or all white"):
        load_vision_artifact(
            root,
            expected_run_id="episode-001",
            expected_alignment_digest=alignment_digest,
        )

    second_root = tmp_path / "second"
    second_root.mkdir()
    root, alignment_digest = _vision(second_root)
    index_path = root / "frame_index.jsonl"
    rows = [json.loads(line) for line in index_path.read_text(encoding="utf-8").splitlines()]
    first = root / rows[0]["payload_path"]
    second = root / rows[3]["payload_path"]
    second.write_bytes(first.read_bytes())
    rows[3]["payload_sha256"] = _sha256(second)
    index_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    _rewrite_checksums(root)

    with pytest.raises(ValueError, match="repeated across distinct source states"):
        load_vision_artifact(
            root,
            expected_run_id="episode-001",
            expected_alignment_digest=alignment_digest,
        )


def test_vision_artifact_rejects_reordered_index_rows(tmp_path: Path) -> None:
    root, alignment_digest = _vision(tmp_path)
    path = root / "frame_index.jsonl"
    rows = path.read_text(encoding="utf-8").splitlines()
    rows[0], rows[1] = rows[1], rows[0]
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    _rewrite_checksums(root)

    with pytest.raises(ValueError, match="rows are reordered"):
        load_vision_artifact(
            root,
            expected_run_id="episode-001",
            expected_alignment_digest=alignment_digest,
        )
