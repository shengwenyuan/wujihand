"""Exact policy-facing episode bundle assembled from immutable derived artifacts."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Final

from .alignment import ExactAlignment
from .artifacts import load_alignment_artifact
from .episode import DatasetEpisodeAnnotation, load_episode_annotation
from .vision import CAMERA_IDS, VisionArtifact, load_vision_artifact


POLICY_IMAGE_KEYS: Final = (
    "observation.images.scene_rgb",
    "observation.images.left_wrist_rgb",
    "observation.images.right_wrist_rgb",
)


@dataclass(frozen=True, slots=True)
class PolicyFrame:
    frame_index: int
    timestamp_s: float
    observation_q54_rad: tuple[float, ...]
    action_q54_rad: tuple[float, ...]
    source_control_index: int
    source_tick_id: int
    simulation_time_s: float
    source_state_digest: str
    image_paths: tuple[Path, Path, Path]
    temporal_continuity: bool = True
    missing_control_periods_before: int = 0
    temporal_segment_index: int = 0

    def __post_init__(self) -> None:
        if type(self.temporal_continuity) is not bool:
            raise ValueError("policy temporal continuity must be boolean")
        if (
            type(self.missing_control_periods_before) is not int
            or self.missing_control_periods_before < 0
            or type(self.temporal_segment_index) is not int
            or self.temporal_segment_index < 0
        ):
            raise ValueError("policy gap mask fields must be non-negative integers")
        if self.temporal_continuity != (
            self.missing_control_periods_before == 0
        ):
            raise ValueError("policy temporal continuity and missing mask differ")


@dataclass(frozen=True, slots=True)
class PolicyEpisode:
    run_id: str
    root: Path
    annotation: DatasetEpisodeAnnotation
    alignment: ExactAlignment
    vision: VisionArtifact
    frames: tuple[PolicyFrame, ...]

    @property
    def task(self) -> str:
        return self.annotation.task


def _validate_exact_vision_join(
    alignment: ExactAlignment,
    vision: VisionArtifact,
) -> tuple[PolicyFrame, ...]:
    if alignment.run_id != vision.run_id:
        raise ValueError("alignment and vision run IDs differ")
    if len(alignment.frames) != vision.frame_count:
        raise ValueError("alignment and vision frame counts differ")
    completed_identities: set[str] = set()
    payloads: set[Path] = set()
    camera_profile_by_id: dict[str, str] = {}
    result: list[PolicyFrame] = []
    for source in alignment.frames:
        records = tuple(
            vision.frame(source.dataset_frame_index, camera_id) for camera_id in CAMERA_IDS
        )
        for record in records:
            if (
                record.source_control_index != source.source_control_index
                or record.source_tick_id != source.source_tick_id
                or record.source_state_digest != source.source_state_digest
                or not math.isclose(
                    record.simulation_time_s,
                    source.simulation_time_s,
                    rel_tol=0.0,
                    abs_tol=1e-9,
                )
            ):
                raise ValueError("vision record does not exactly match its alignment source")
            previous_profile = camera_profile_by_id.setdefault(
                record.camera_id,
                record.camera_profile_sha256,
            )
            if previous_profile != record.camera_profile_sha256:
                raise ValueError("camera profile changed within one episode")
            if record.completed_frame_identity in completed_identities:
                raise ValueError("completed render frame identity is duplicated")
            completed_identities.add(record.completed_frame_identity)
        image_paths = tuple(vision.payload(record) for record in records)
        if any(path in payloads for path in image_paths):
            raise ValueError("vision payload is reused by multiple policy frames")
        payloads.update(image_paths)
        result.append(
            PolicyFrame(
                frame_index=source.dataset_frame_index,
                timestamp_s=source.timestamp_s,
                observation_q54_rad=source.observation_q54_rad,
                action_q54_rad=source.action_q54_rad,
                source_control_index=source.source_control_index,
                source_tick_id=source.source_tick_id,
                simulation_time_s=source.simulation_time_s,
                source_state_digest=source.source_state_digest,
                temporal_continuity=source.temporal_continuity,
                missing_control_periods_before=(
                    source.missing_control_periods_before
                ),
                temporal_segment_index=source.temporal_segment_index,
                image_paths=(image_paths[0], image_paths[1], image_paths[2]),
            )
        )
    return tuple(result)


def load_policy_episode(run_root: str | Path) -> PolicyEpisode:
    raw_root = Path(run_root)
    if raw_root.is_symlink():
        raise ValueError("policy episode run root must not be a symbolic link")
    root = raw_root.resolve()
    if not root.is_dir():
        raise ValueError("policy episode run root must be a directory")
    annotation = load_episode_annotation(root, expected_run_id=root.name)
    alignment = load_alignment_artifact(
        root / "derived" / "alignment",
        expected_run_id=root.name,
    )
    vision = load_vision_artifact(
        root / "derived" / "vision",
        expected_run_id=root.name,
        expected_alignment_digest=alignment.digest_sha256,
    )
    frames = _validate_exact_vision_join(alignment, vision)
    return PolicyEpisode(
        run_id=root.name,
        root=root,
        annotation=annotation,
        alignment=alignment,
        vision=vision,
        frames=frames,
    )


__all__ = [
    "POLICY_IMAGE_KEYS",
    "PolicyEpisode",
    "PolicyFrame",
    "load_policy_episode",
]
