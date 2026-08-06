"""Backend-neutral orchestration for exact pre-action three-view rendering."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import struct
from typing import Protocol, cast
import zlib

import numpy as np
from numpy.typing import NDArray

from wujihand.domain.dataset_recording import SimulationFramePhase, SimulationStateFrame

from .artifacts import load_alignment_artifact
from .camera import DatasetCameraRuntimeInventory
from .normalized import load_normalized_episode_artifact
from .profile import MiniDatasetProfile
from .release_artifact import load_release_decision_artifact
from .vision import (
    CAMERA_IDS,
    DatasetVisionProvenance,
    VisionArtifact,
    VisionArtifactBuilder,
    VisionFrameRecord,
)


@dataclass(frozen=True, slots=True)
class CompletedRgbRender:
    camera_id: str
    payload_png: bytes
    completed_frame_identity: str
    camera_profile_sha256: str
    parent_frame_id: str
    world_from_parent_row_major: tuple[float, ...]
    world_from_camera_optical_row_major: tuple[float, ...]


class FixedStateRgbBackend(Protocol):
    @property
    def renderer_identity(self) -> str: ...

    @property
    def renderer_backend(self) -> str: ...

    @property
    def lighting_identity(self) -> str: ...

    @property
    def color_space(self) -> str: ...

    @property
    def motion_blur_enabled(self) -> bool: ...

    @property
    def simulation_time_s(self) -> float: ...

    @property
    def camera_runtime_inventories(self) -> tuple[DatasetCameraRuntimeInventory, ...]: ...

    def inject_pre_action_state(
        self,
        frame: SimulationStateFrame,
        *,
        dataset_frame_index: int,
    ) -> str: ...

    def render_rgb(
        self,
        *,
        camera_id: str,
        dataset_frame_index: int,
    ) -> CompletedRgbRender: ...


def encode_rgb8_png(rgb: NDArray[np.generic]) -> bytes:
    """Encode one canonical, non-interlaced RGB8 frame without optional codecs."""

    if rgb.dtype != np.uint8 or rgb.shape != (480, 640, 3):
        raise ValueError("RGB frame must be uint8 with shape 480x640x3")
    contiguous = np.ascontiguousarray(rgb)

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    scanlines = b"".join(
        b"\x00" + contiguous[row_index].tobytes(order="C")
        for row_index in range(contiguous.shape[0])
    )
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", 640, 480, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(scanlines, level=6))
        + chunk(b"IEND", b"")
    )


def _mapping(value: object, *, field: str) -> dict[str, object]:
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(dict[str, object], value)


def _sha256_value(value: object, *, field: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(char not in "0123456789abcdef" for char in value)
    ):
        raise ValueError(f"{field} must be a lowercase SHA-256")
    return value


def _vision_provenance(
    root: Path,
    *,
    dataset_profile: MiniDatasetProfile,
    renderer_identity: str,
    renderer_backend: str,
    lighting_identity: str,
    color_space: str,
    motion_blur_enabled: bool,
    camera_hashes: dict[str, str],
    collection_id: str,
) -> DatasetVisionProvenance:
    try:
        document = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError("raw run manifest is invalid") from exc
    manifest = _mapping(document, field="raw manifest")
    deployment = _mapping(manifest.get("deployment"), field="raw deployment")
    dataset = _mapping(manifest.get("dataset"), field="raw dataset")
    profile_hash = _sha256_value(
        dataset.get("profile_sha256"),
        field="raw dataset profile hash",
    )
    if profile_hash != dataset_profile.file_sha256:
        raise ValueError("raw and renderer dataset profile hashes differ")
    return DatasetVisionProvenance.create(
        collection_id=collection_id,
        dataset_profile_sha256=profile_hash,
        deployment_sha256=_sha256_value(
            deployment.get("deployment_hash"),
            field="raw deployment hash",
        ),
        session_sha256=_sha256_value(
            deployment.get("session_hash"),
            field="raw session hash",
        ),
        assembly_sha256=_sha256_value(
            deployment.get("assembly_sha256"),
            field="raw assembly hash",
        ),
        workcell_sha256=_sha256_value(
            deployment.get("workcell_sha256"),
            field="raw workcell hash",
        ),
        renderer_identity=renderer_identity,
        renderer_backend=renderer_backend,
        lighting_identity=lighting_identity,
        color_space=color_space,
        motion_blur_enabled=motion_blur_enabled,
        camera_profile_sha256_by_id=camera_hashes,
    )


def render_exact_triview(
    run_root: str | Path,
    *,
    dataset_profile: MiniDatasetProfile,
    backend: FixedStateRgbBackend,
) -> VisionArtifact:
    """Render all alignment anchors without nearest joins or physics stepping."""

    raw_root = Path(run_root)
    if raw_root.is_symlink():
        raise ValueError("render run root must not be a symbolic link")
    root = raw_root.resolve()
    if not root.is_dir():
        raise ValueError("render run root must be a directory")
    run_id = root.name
    release = load_release_decision_artifact(
        root / "derived" / "release",
        expected_run_id=run_id,
    )
    if not release.decision.passed:
        raise ValueError("offline rendering requires a passing release decision")
    normalized = load_normalized_episode_artifact(
        root / "derived" / "normalized",
        expected_run_id=run_id,
    )
    alignment = load_alignment_artifact(
        root / "derived" / "alignment",
        expected_run_id=run_id,
    )
    if not alignment.frames:
        raise ValueError("offline rendering requires at least one alignment frame")
    if tuple(camera.logical_id for camera in dataset_profile.cameras) != CAMERA_IDS:
        raise ValueError("dataset profile camera order differs from the vision contract")
    camera_hashes = {camera.logical_id: camera.profile.sha256 for camera in dataset_profile.cameras}
    runtime_inventories = backend.camera_runtime_inventories
    if tuple(item.camera_id for item in runtime_inventories) != CAMERA_IDS or any(
        item.profile_sha256 != camera_hashes[item.camera_id] for item in runtime_inventories
    ):
        raise ValueError("renderer camera runtime inventories differ from the dataset profile")
    inventory_by_camera = {item.camera_id: item for item in runtime_inventories}
    collection_ids = {item.collection_id for item in normalized.facts.boundaries}
    if len(collection_ids) != 1:
        raise ValueError("normalized episode does not contain one collection identity")
    provenance = _vision_provenance(
        root,
        dataset_profile=dataset_profile,
        renderer_identity=backend.renderer_identity,
        renderer_backend=backend.renderer_backend,
        lighting_identity=backend.lighting_identity,
        color_space=backend.color_space,
        motion_blur_enabled=backend.motion_blur_enabled,
        camera_hashes=camera_hashes,
        collection_id=next(iter(collection_ids)),
    )
    pre_frames = {
        tick.transition.control_index: tick.pre_action_frame for tick in normalized.facts.ticks
    }
    builder = VisionArtifactBuilder(
        root,
        run_id=run_id,
        alignment_digest_sha256=alignment.digest_sha256,
        frame_count=len(alignment.frames),
        renderer_identity=backend.renderer_identity,
        provenance=provenance,
        camera_runtime_inventories=runtime_inventories,
    )
    completed_identities: set[str] = set()
    try:
        for alignment_frame in alignment.frames:
            try:
                state = pre_frames[alignment_frame.source_control_index]
            except KeyError as exc:
                raise ValueError("alignment source pre-action state is missing") from exc
            if (
                state.phase is not SimulationFramePhase.PRE_ACTION
                or state.tick_id != alignment_frame.source_tick_id
                or state.payload_digest_sha256 != alignment_frame.source_state_digest
                or not math.isclose(
                    state.simulation_time_s,
                    alignment_frame.simulation_time_s,
                    rel_tol=0.0,
                    abs_tol=1e-9,
                )
            ):
                raise ValueError("alignment and pre-action source state differ")
            injected_digest = backend.inject_pre_action_state(
                state,
                dataset_frame_index=alignment_frame.dataset_frame_index,
            )
            if injected_digest != state.payload_digest_sha256:
                raise ValueError("renderer did not acknowledge the exact source state digest")
            fixed_time = backend.simulation_time_s
            if not math.isfinite(fixed_time) or fixed_time < 0.0:
                raise ValueError("renderer simulation time is invalid")
            for camera_id in CAMERA_IDS:
                rendered = backend.render_rgb(
                    camera_id=camera_id,
                    dataset_frame_index=alignment_frame.dataset_frame_index,
                )
                if rendered.camera_id != camera_id:
                    raise ValueError("renderer returned the wrong logical camera")
                if rendered.camera_profile_sha256 != camera_hashes[camera_id]:
                    raise ValueError("renderer camera profile hash differs")
                inventory = inventory_by_camera[camera_id]
                if rendered.parent_frame_id != inventory.parent_frame_id:
                    raise ValueError("renderer camera parent frame differs")
                if rendered.completed_frame_identity in completed_identities:
                    raise ValueError("renderer completed-frame identity is duplicated")
                completed_identities.add(rendered.completed_frame_identity)
                if not math.isclose(
                    backend.simulation_time_s,
                    fixed_time,
                    rel_tol=0.0,
                    abs_tol=1e-12,
                ):
                    raise ValueError("RGB rendering advanced simulation time")
                payload_sha256 = hashlib.sha256(rendered.payload_png).hexdigest()
                record = VisionFrameRecord(
                    run_id=run_id,
                    collection_id=provenance.collection_id,
                    provenance_sha256=provenance.digest_sha256,
                    camera_id=camera_id,
                    dataset_frame_index=alignment_frame.dataset_frame_index,
                    source_control_index=alignment_frame.source_control_index,
                    source_tick_id=alignment_frame.source_tick_id,
                    phase="pre_action",
                    simulation_time_s=state.simulation_time_s,
                    source_state_digest=state.payload_digest_sha256,
                    payload_path=(f"{camera_id}/{alignment_frame.dataset_frame_index:06d}.png"),
                    payload_sha256=payload_sha256,
                    width_px=640,
                    height_px=480,
                    encoding="rgb8",
                    camera_profile_sha256=rendered.camera_profile_sha256,
                    completed_frame_identity=rendered.completed_frame_identity,
                    parent_frame_id=rendered.parent_frame_id,
                    world_from_parent_row_major=rendered.world_from_parent_row_major,
                    world_from_camera_optical_row_major=(
                        rendered.world_from_camera_optical_row_major
                    ),
                )
                builder.add_rgb_png(record, rendered.payload_png)
        return builder.publish()
    except BaseException:
        builder.abort()
        raise


__all__ = [
    "CompletedRgbRender",
    "FixedStateRgbBackend",
    "encode_rgb8_png",
    "render_exact_triview",
]
