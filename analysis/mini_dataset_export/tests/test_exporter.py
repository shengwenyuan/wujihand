from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import numpy as np
import pytest
from wujihand.dataset.alignment import AlignmentFrame, ExactAlignment
from wujihand.dataset.episode import DatasetEpisodeAnnotation
from wujihand.dataset.domain_randomization import NOMINAL_VISUAL_DOMAIN_VARIANT
from wujihand.dataset.policy import PolicyEpisode, PolicyFrame
from wujihand.dataset.profile import load_q54_joint_profile
from wujihand.dataset.vision import (
    CAMERA_IDS,
    DatasetVisionProvenance,
    VisionArtifact,
    VisionFrameRecord,
)

from wujihand_mini_dataset_export.exporter import (
    POLICY_FEATURE_KEYS,
    export_collection,
    lerobot_feature_contract,
)

PROJECT_ROOT = Path(__file__).resolve().parents[3]
Q54_PROFILE = "configs/profiles/isaac_nero_hand2_q54_dataset_v1.yaml"
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
AUTO_FEATURES = {
    "timestamp": {"dtype": "float32", "shape": (1,), "names": None},
    "frame_index": {"dtype": "int64", "shape": (1,), "names": None},
    "episode_index": {"dtype": "int64", "shape": (1,), "names": None},
    "index": {"dtype": "int64", "shape": (1,), "names": None},
    "task_index": {"dtype": "int64", "shape": (1,), "names": None},
}


class FakeDataset:
    def __init__(
        self,
        root: Path,
        features: dict[str, dict[str, object]],
        *,
        fps: int,
    ) -> None:
        root.mkdir()
        (root / "meta").mkdir()
        self.root = root
        self.features = {**features, **AUTO_FEATURES}
        self.fps = fps
        self.num_episodes = 0
        self._buffer: list[dict[str, object]] = []
        self._episodes: list[list[dict[str, object]]] = []

    def add_frame(self, frame: dict[str, object]) -> None:
        self._buffer.append(dict(frame))

    def save_episode(
        self,
        episode_data: dict[str, object] | None = None,
        parallel_encoding: bool = True,
    ) -> None:
        assert episode_data is None
        assert not parallel_encoding
        self._episodes.append(self._buffer)
        self._buffer = []
        self.num_episodes += 1

    def finalize(self) -> None:
        (self.root / "meta" / "info.json").write_text("{}\n", encoding="utf-8")
        data = self.root / "data" / "chunk-000"
        data.mkdir(parents=True, exist_ok=True)
        (data / "file-000.parquet").write_bytes(b"fake-parquet")
        videos = self.root / "videos"
        videos.mkdir(exist_ok=True)
        (videos / "fake.mp4").write_bytes(b"fake-video")

    def __len__(self) -> int:
        return sum(len(episode) for episode in self._episodes)

    def __getitem__(self, index: int) -> dict[str, object]:
        cursor = 0
        for episode_index, episode in enumerate(self._episodes):
            if index < cursor + len(episode):
                frame_index = index - cursor
                frame = episode[frame_index]
                result = {
                    key: np.transpose(cast(np.ndarray[Any, Any], value), (2, 0, 1))
                    if key in POLICY_FEATURE_KEYS and key.startswith("observation.images.")
                    else value
                    for key, value in frame.items()
                }
                result.update(
                    {
                        "timestamp": np.asarray(frame_index / self.fps, dtype=np.float32),
                        "frame_index": np.asarray(frame_index, dtype=np.int64),
                        "episode_index": np.asarray(episode_index, dtype=np.int64),
                        "index": np.asarray(index, dtype=np.int64),
                        "task_index": np.asarray(episode_index, dtype=np.int64),
                    }
                )
                return result
            cursor += len(episode)
        raise IndexError(index)


@dataclass
class FakeBackend:
    dataset: FakeDataset | None = None

    def create(
        self,
        *,
        repo_id: str,
        root: Path,
        fps: int,
        robot_type: str,
        features: dict[str, dict[str, object]],
    ) -> FakeDataset:
        assert repo_id == "local/wujihand-mini"
        assert robot_type == "agile_nero_dual_wuji_hand2_simulation"
        self.dataset = FakeDataset(root, features, fps=fps)
        return self.dataset

    def load(self, *, repo_id: str, root: Path) -> FakeDataset:
        assert repo_id == "local/wujihand-mini"
        assert self.dataset is not None and self.dataset.root == root
        return self.dataset


def _episode(tmp_path: Path, run_id: str, *, offset: int) -> PolicyEpisode:
    root = tmp_path / run_id
    vision_root = root / "derived" / "vision"
    vision_root.mkdir(parents=True)
    paths = []
    records = []
    provenance = DatasetVisionProvenance.create(
        collection_id="mini-v1",
        dataset_profile_sha256="e" * 64,
        deployment_sha256="1" * 64,
        session_sha256="2" * 64,
        assembly_sha256="3" * 64,
        workcell_sha256="4" * 64,
        renderer_identity="offline-fixed-state-v1",
        renderer_backend="RayTracedLighting",
        lighting_identity="session_workcell_authored_lighting",
        color_space="isaac_rgb_annotator_srgb",
        motion_blur_enabled=False,
        camera_profile_sha256_by_id={camera_id: "c" * 64 for camera_id in CAMERA_IDS},
    )
    for camera_id in CAMERA_IDS:
        relative = Path(camera_id) / "000000.png"
        payload = vision_root / relative
        payload.parent.mkdir()
        payload.write_bytes(camera_id.encode())
        paths.append(payload)
        records.append(
            VisionFrameRecord(
                run_id=run_id,
                collection_id=provenance.collection_id,
                provenance_sha256=provenance.digest_sha256,
                camera_id=camera_id,
                dataset_frame_index=0,
                source_control_index=offset,
                source_tick_id=offset,
                phase="pre_action",
                simulation_time_s=0.0,
                source_state_digest="a" * 64,
                payload_path=relative.as_posix(),
                payload_sha256="b" * 64,
                width_px=640,
                height_px=480,
                encoding="rgb8",
                camera_profile_sha256="c" * 64,
                completed_frame_identity=f"render-{run_id}-{camera_id}",
                parent_frame_id=f"{camera_id}_parent",
                world_from_parent_row_major=IDENTITY,
                world_from_camera_optical_row_major=IDENTITY,
            )
        )
    alignment_frame = AlignmentFrame(
        dataset_frame_index=0,
        source_control_index=offset,
        source_tick_id=offset,
        timestamp_s=0.0,
        simulation_time_s=0.0,
        observation_q54_rad=(float(offset),) * 54,
        action_q54_rad=(float(offset) + 0.25,) * 54,
        source_state_digest="a" * 64,
    )
    alignment = ExactAlignment(
        run_id=run_id,
        source_first_control_index=offset,
        source_last_control_index=offset,
        source_transition_count=1,
        frames=(alignment_frame,),
        digest_sha256="d" * 64,
    )
    vision = VisionArtifact(
        root=vision_root,
        run_id=run_id,
        alignment_digest_sha256=alignment.digest_sha256,
        frame_count=1,
        renderer_identity="offline-fixed-state-v1",
        provenance=provenance,
        camera_runtime_inventories=(),
        frames=tuple(records),
    )
    frame = PolicyFrame(
        frame_index=0,
        timestamp_s=0.0,
        observation_q54_rad=alignment_frame.observation_q54_rad,
        action_q54_rad=alignment_frame.action_q54_rad,
        source_control_index=offset,
        source_tick_id=offset,
        simulation_time_s=0.0,
        source_state_digest="a" * 64,
        image_paths=(paths[0], paths[1], paths[2]),
    )
    return PolicyEpisode(
        run_id=run_id,
        source_run_id=run_id,
        root=root,
        annotation=DatasetEpisodeAnnotation(
            run_id=run_id,
            task="Move both hands toward the tabletop objects.",
            operator_note="",
        ),
        alignment=alignment,
        vision=vision,
        frames=(frame,),
        quality_grade="C",
        release_decision_sha256="e" * 64,
        visual_domain_variant=NOMINAL_VISUAL_DOMAIN_VARIANT,
        visual_domain_variant_profile_sha256="f" * 64,
    )


def _rgb(_: Path) -> np.ndarray[Any, np.dtype[np.uint8]]:
    return np.full((480, 640, 3), 127, dtype=np.uint8)


def test_export_collection_finalizes_reopens_and_publishes_sidecars(tmp_path: Path) -> None:
    q54 = load_q54_joint_profile(PROJECT_ROOT, Q54_PROFILE)
    episodes = (
        _episode(tmp_path, "episode-001", offset=10),
        _episode(tmp_path, "episode-002", offset=20),
    )
    backend = FakeBackend()
    destination = tmp_path / "dataset-revision"

    result = export_collection(
        episodes,
        q54,
        destination,
        repo_id="local/wujihand-mini",
        dataset_factory=backend.create,
        dataset_loader=backend.load,
        image_loader=_rgb,
    )

    assert result.episode_count == 2
    assert result.frame_count == 2
    assert destination.is_dir()
    manifest = json.loads(
        (destination / "meta" / "wujihand_export_manifest.json").read_text()
    )
    assert manifest["episode_order"] == ["episode-001", "episode-002"]
    assert manifest["episode_quality_grades"] == {
        "episode-001": "C",
        "episode-002": "C",
    }
    assert manifest["valid_transition_count"] == 0
    assert manifest["success_semantics"] == "not_recorded_not_evaluated"
    source_rows = (
        destination / "meta" / "wujihand_frame_source.jsonl"
    ).read_text().splitlines()
    assert len(source_rows) == 2
    source = json.loads(source_rows[0])
    assert source["temporal_continuity"] is True
    assert source["transition_from_previous_allowed"] is True
    assert source["missing_control_periods_before"] == 0
    assert source["temporal_segment_index"] == 0
    assert "observation.images.left_wrist_rgb" in manifest["policy_features"]
    assert (destination / "wujihand_checksums.sha256").is_file()
    assert not tuple(tmp_path.glob(".dataset-revision.tmp-*"))


def test_export_refuses_overwrite_without_calling_backend(tmp_path: Path) -> None:
    q54 = load_q54_joint_profile(PROJECT_ROOT, Q54_PROFILE)
    episode = _episode(tmp_path, "episode-001", offset=10)
    destination = tmp_path / "dataset-revision"
    destination.mkdir()
    backend = FakeBackend()

    with pytest.raises(FileExistsError, match="already exists"):
        export_collection(
            (episode,),
            q54,
            destination,
            repo_id="local/wujihand-mini",
            dataset_factory=backend.create,
            dataset_loader=backend.load,
            image_loader=_rgb,
        )

    assert backend.dataset is None


def test_feature_contract_keeps_all_q54_names_and_only_policy_fields() -> None:
    q54 = load_q54_joint_profile(PROJECT_ROOT, Q54_PROFILE)

    features = lerobot_feature_contract(q54.canonical_names)

    assert frozenset(features) == POLICY_FEATURE_KEYS
    assert features["observation.state"]["names"] == list(q54.canonical_names)
    assert features["action"]["shape"] == (54,)


def test_pinned_lerobot_real_round_trip_when_dependency_is_installed(
    tmp_path: Path,
) -> None:
    pytest.importorskip("lerobot")
    image_module = pytest.importorskip("PIL.Image")
    q54 = load_q54_joint_profile(PROJECT_ROOT, Q54_PROFILE)
    episodes = (
        _episode(tmp_path, "episode-001", offset=10),
        _episode(tmp_path, "episode-002", offset=20),
    )
    for episode_index, episode in enumerate(episodes):
        for camera_index, path in enumerate(episode.frames[0].image_paths):
            value = 32 + episode_index * 60 + camera_index * 20
            image_module.fromarray(
                np.full((480, 640, 3), value, dtype=np.uint8),
                mode="RGB",
            ).save(path, format="PNG")

    result = export_collection(
        episodes,
        q54,
        tmp_path / "real-lerobot-revision",
        repo_id="local/wujihand-mini-golden",
    )

    assert result.episode_count == 2
    assert result.frame_count == 2
    assert (result.root / "meta" / "info.json").is_file()
