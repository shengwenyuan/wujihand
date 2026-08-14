"""Atomic LeRobot v3 export with a strict policy-feature whitelist and round trip."""

from __future__ import annotations

import fcntl
import hashlib
import json
import math
import os
import shutil
import sys
from collections.abc import Callable, Iterator, Mapping, Sequence
from contextlib import contextmanager, suppress
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Final, Protocol, cast
from uuid import uuid4

import numpy as np
from numpy.typing import NDArray
from wujihand.dataset.policy import POLICY_IMAGE_KEYS, PolicyEpisode
from wujihand.dataset.profile import Q54JointProfile

LEROBOT_COMMIT: Final = "7e241bd630a3719a56157a497ce5d08f244784f1"
LEROBOT_TAG: Final = "v0.6.1"
EXPORTER_VERSION: Final = "0.3.0"
EXPORT_MANIFEST_SCHEMA: Final = "wujihand.lerobot_export_manifest.v1"
Q54_SIDECAR_SCHEMA: Final = "wujihand.lerobot_q54_sidecar.v1"
SOURCE_MAP_SCHEMA: Final = "wujihand.lerobot_frame_source.v3"
VALID_TRANSITION_SCHEMA: Final = "wujihand.lerobot_valid_transition.v1"
LEROBOT_VIDEO_SEEK_TOLERANCE_S: Final = 1e-4
AUTO_FEATURE_KEYS: Final = frozenset(
    {"timestamp", "frame_index", "episode_index", "index", "task_index"}
)
POLICY_FEATURE_KEYS: Final = frozenset(
    {"observation.state", "action", *POLICY_IMAGE_KEYS}
)


class DatasetHandle(Protocol):
    features: Mapping[str, Mapping[str, object]]
    fps: int
    num_episodes: int

    def add_frame(self, frame: dict[str, object]) -> None: ...

    def save_episode(
        self,
        episode_data: dict[str, object] | None = None,
        parallel_encoding: bool = True,
    ) -> None: ...

    def finalize(self) -> None: ...

    def __len__(self) -> int: ...

    def __getitem__(self, index: int) -> dict[str, object]: ...


class DatasetFactory(Protocol):
    def __call__(
        self,
        *,
        repo_id: str,
        root: Path,
        fps: int,
        robot_type: str,
        features: dict[str, dict[str, object]],
    ) -> DatasetHandle: ...


class DatasetLoader(Protocol):
    def __call__(self, *, repo_id: str, root: Path) -> DatasetHandle: ...


ImageLoader = Callable[[Path], NDArray[np.uint8]]


@dataclass(frozen=True, slots=True)
class ExportResult:
    root: Path
    repo_id: str
    episode_count: int
    frame_count: int
    manifest_sha256: str
    checksums_sha256: str


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _json_bytes(value: object) -> bytes:
    return (
        json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + "\n"
    ).encode("utf-8")


@contextmanager
def _isolated_huggingface_cache(cache_root: Path) -> Iterator[None]:
    """Keep local reopen caches outside the user home and outside the artifact."""

    keys = ("HF_HOME", "HF_DATASETS_CACHE", "HF_HUB_CACHE")
    previous_environment = {key: os.environ.get(key) for key in keys}
    values = {
        "HF_HOME": cache_root.as_posix(),
        "HF_DATASETS_CACHE": (cache_root / "datasets").as_posix(),
        "HF_HUB_CACHE": (cache_root / "hub").as_posix(),
    }
    os.environ.update(values)
    datasets_module = sys.modules.get("datasets")
    config = None if datasets_module is None else getattr(datasets_module, "config", None)
    previous_dataset_cache = (
        None if config is None else getattr(config, "HF_DATASETS_CACHE", None)
    )
    if config is not None:
        config.HF_DATASETS_CACHE = Path(values["HF_DATASETS_CACHE"])
    try:
        yield
    finally:
        if config is not None:
            config.HF_DATASETS_CACHE = previous_dataset_cache
        for key, previous in previous_environment.items():
            if previous is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = previous
        shutil.rmtree(cache_root, ignore_errors=True)


def lerobot_feature_contract(q54_names: Sequence[str]) -> dict[str, dict[str, object]]:
    names = tuple(q54_names)
    if len(names) != 54 or len(set(names)) != 54:
        raise ValueError("LeRobot q54 names must contain 54 unique entries")
    if any(not name or "/" in name for name in names):
        raise ValueError("LeRobot q54 names must be non-empty and must not contain '/'")
    features: dict[str, dict[str, object]] = {
        "observation.state": {
            "dtype": "float32",
            "shape": (54,),
            "names": list(names),
        },
        "action": {
            "dtype": "float32",
            "shape": (54,),
            "names": list(names),
        },
    }
    for key in POLICY_IMAGE_KEYS:
        features[key] = {
            "dtype": "video",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channels"],
            "info": {"is_depth_map": False},
        }
    return features


def _default_image_loader(path: Path) -> NDArray[np.uint8]:
    from PIL import Image

    with Image.open(path) as image:
        image.load()
        if image.format != "PNG" or image.mode != "RGB" or image.size != (640, 480):
            raise ValueError(f"policy RGB must be an unmodified 640x480 RGB PNG: {path}")
        array = np.asarray(image, dtype=np.uint8).copy()
    if array.shape != (480, 640, 3):
        raise ValueError(f"decoded policy RGB shape differs: {path}")
    if bool(np.all(array == 0)) or bool(np.all(array == 255)):
        raise ValueError(f"policy RGB is all black or all white: {path}")
    return array


def _default_factory(
    *,
    repo_id: str,
    root: Path,
    fps: int,
    robot_type: str,
    features: dict[str, dict[str, object]],
) -> DatasetHandle:
    from lerobot.configs import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    encoder = RGBEncoderConfig(
        vcodec="libsvtav1",
        pix_fmt="yuv420p",
        g=2,
        crf=30,
        preset=12,
        fast_decode=0,
        video_backend="pyav",
        extra_options={},
    )
    return cast(
        DatasetHandle,
        LeRobotDataset.create(
            repo_id=repo_id,
            root=root,
            fps=fps,
            robot_type=robot_type,
            features=features,
            use_videos=True,
            # LeRobot queries video PTS using float32 tensors.  The ULP grows
            # with a long concatenated video, so a fixed 1 us tolerance rejects
            # the exact intended frame after roughly 32 s.  100 us covers that
            # representation error while remaining far below one 30 Hz frame.
            tolerance_s=LEROBOT_VIDEO_SEEK_TOLERANCE_S,
            image_writer_processes=0,
            image_writer_threads=0,
            video_backend="pyav",
            batch_encoding_size=1,
            rgb_encoder=encoder,
            metadata_buffer_size=10,
            streaming_encoding=False,
            encoder_threads=4,
        ),
    )


def _default_loader(*, repo_id: str, root: Path) -> DatasetHandle:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    return cast(
        DatasetHandle,
        LeRobotDataset(
            repo_id=repo_id,
            root=root,
            tolerance_s=LEROBOT_VIDEO_SEEK_TOLERANCE_S,
            download_videos=True,
            video_backend="pyav",
            return_uint8=True,
        ),
    )


def _to_numpy(value: object) -> NDArray[Any]:
    candidate = value
    detach = getattr(candidate, "detach", None)
    if callable(detach):
        candidate = detach()
    cpu = getattr(candidate, "cpu", None)
    if callable(cpu):
        candidate = cpu()
    to_numpy = getattr(candidate, "numpy", None)
    if callable(to_numpy):
        candidate = to_numpy()
    return np.asarray(candidate)


def _scalar(value: object, *, field: str) -> float:
    array = _to_numpy(value)
    if array.size != 1:
        raise ValueError(f"round-trip {field} must be scalar")
    result = float(array.reshape(-1)[0])
    if not math.isfinite(result):
        raise ValueError(f"round-trip {field} must be finite")
    return result


def _validate_features(
    actual: Mapping[str, Mapping[str, object]],
    expected: Mapping[str, Mapping[str, object]],
) -> None:
    if frozenset(actual) != POLICY_FEATURE_KEYS | AUTO_FEATURE_KEYS:
        raise ValueError("round-trip LeRobot feature whitelist differs")
    for key, contract in expected.items():
        observed = actual[key]
        for field in ("dtype", "shape", "names"):
            expected_value = contract[field]
            observed_value = observed.get(field)
            if field == "shape":
                expected_value = tuple(cast(Sequence[int], expected_value))
                observed_value = tuple(cast(Sequence[int], observed_value))
            if observed_value != expected_value:
                raise ValueError(f"round-trip LeRobot feature differs: {key}.{field}")


def _expected_rows(
    episodes: Sequence[PolicyEpisode],
) -> tuple[tuple[int, int, PolicyEpisode, object], ...]:
    rows: list[tuple[int, int, PolicyEpisode, object]] = []
    global_index = 0
    for episode_index, episode in enumerate(episodes):
        for frame in episode.frames:
            rows.append((global_index, episode_index, episode, frame))
            global_index += 1
    return tuple(rows)


def _validate_round_trip(
    dataset: DatasetHandle,
    episodes: Sequence[PolicyEpisode],
    features: Mapping[str, Mapping[str, object]],
) -> None:
    rows = _expected_rows(episodes)
    if dataset.fps != 30 or dataset.num_episodes != len(episodes) or len(dataset) != len(rows):
        raise ValueError("round-trip LeRobot episode/frame/fps closure differs")
    _validate_features(dataset.features, features)
    for global_index, episode_index, episode, frame_value in rows:
        frame = cast(Any, frame_value)
        item = dataset[global_index]
        if int(_scalar(item["episode_index"], field="episode_index")) != episode_index:
            raise ValueError("round-trip episode index differs")
        if int(_scalar(item["frame_index"], field="frame_index")) != frame.frame_index:
            raise ValueError("round-trip frame index differs")
        observed_timestamp = np.float32(_scalar(item["timestamp"], field="timestamp"))
        expected_timestamp = np.float32(frame.frame_index / 30.0)
        if observed_timestamp.view(np.uint32) != expected_timestamp.view(np.uint32):
            raise ValueError("round-trip timestamp differs")
        if item.get("task") != episode.task:
            raise ValueError("round-trip task differs")
        for key, expected_vector in (
            ("observation.state", frame.observation_q54_rad),
            ("action", frame.action_q54_rad),
        ):
            actual_vector = _to_numpy(item[key])
            if actual_vector.shape != (54,) or actual_vector.dtype != np.dtype("float32"):
                raise ValueError(f"round-trip {key} shape or dtype differs")
            if not np.allclose(
                actual_vector,
                np.asarray(expected_vector, dtype=np.float32),
                rtol=0.0,
                atol=1e-6,
            ):
                raise ValueError(f"round-trip {key} values differ")
        for key in POLICY_IMAGE_KEYS:
            image = _to_numpy(item[key])
            if image.shape != (3, 480, 640) or image.dtype != np.dtype("uint8"):
                raise ValueError(f"round-trip {key} decode shape or dtype differs")
    probe_indices = tuple(dict.fromkeys((0, len(rows) // 2, len(rows) - 1)))
    for index in probe_indices:
        if not isinstance(dataset[index], dict):
            raise TypeError("round-trip random access did not return a frame mapping")


def _write_sidecars(
    root: Path,
    *,
    repo_id: str,
    robot_type: str,
    episodes: Sequence[PolicyEpisode],
    q54_profile: Q54JointProfile,
    features: Mapping[str, Mapping[str, object]],
) -> Path:
    meta = root / "meta"
    if not meta.is_dir() or meta.is_symlink():
        raise ValueError("LeRobot meta directory is missing or unsafe")
    q54_path = meta / "wujihand_q54.json"
    q54_path.write_bytes(
        _json_bytes(
            {
                "schema": Q54_SIDECAR_SCHEMA,
                "profile": asdict(q54_profile),
                "hardware_boundary": (
                    "simulation profile; real NERO limits/mapping require device readback"
                ),
            }
        )
    )
    source_path = meta / "wujihand_frame_source.jsonl"
    source_lines: list[bytes] = []
    transition_lines: list[bytes] = []
    global_index = 0
    for episode_index, episode in enumerate(episodes):
        for frame in episode.frames:
            camera_records = {
                camera_id: episode.vision.frame(frame.frame_index, camera_id)
                for camera_id in ("scene_rgb", "left_wrist_rgb", "right_wrist_rgb")
            }
            row = {
                "schema": SOURCE_MAP_SCHEMA,
                "dataset_global_index": global_index,
                "dataset_episode_index": episode_index,
                "dataset_frame_index": frame.frame_index,
                "run_id": episode.run_id,
                "source_run_id": episode.source_run_id,
                "visual_domain_variant": episode.visual_domain_variant.to_mapping(),
                "visual_domain_variant_sha256": (
                    episode.visual_domain_variant.digest_sha256
                ),
                "visual_domain_variant_profile_sha256": (
                    episode.visual_domain_variant_profile_sha256
                ),
                "collection_id": episode.vision.provenance.collection_id,
                "source_control_index": frame.source_control_index,
                "source_tick_id": frame.source_tick_id,
                "source_simulation_time_s": frame.simulation_time_s,
                "source_state_digest": frame.source_state_digest,
                "temporal_continuity": frame.temporal_continuity,
                "transition_from_previous_allowed": frame.temporal_continuity,
                "gap_before_row": frame.gap_before_row,
                "transition_valid": frame.transition_valid,
                "missing_control_periods_before": (
                    frame.missing_control_periods_before
                ),
                "temporal_segment_index": frame.temporal_segment_index,
                "alignment_digest_sha256": episode.alignment.digest_sha256,
                "vision_provenance_sha256": episode.vision.provenance.digest_sha256,
                "renderer_configuration_sha256": (
                    episode.vision.provenance.renderer_configuration_sha256
                ),
                "vision_payload_sha256": {
                    camera_id: record.payload_sha256
                    for camera_id, record in camera_records.items()
                },
                "vision_completed_frame_identity": {
                    camera_id: record.completed_frame_identity
                    for camera_id, record in camera_records.items()
                },
                "vision_camera_pose": {
                    camera_id: {
                        "parent_frame_id": record.parent_frame_id,
                        "world_from_parent_row_major": list(
                            record.world_from_parent_row_major
                        ),
                        "world_from_camera_optical_row_major": list(
                            record.world_from_camera_optical_row_major
                        ),
                    }
                    for camera_id, record in camera_records.items()
                },
            }
            source_lines.append(
                json.dumps(
                    row,
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                ).encode("utf-8")
                + b"\n"
            )
            if frame.transition_valid:
                transition_lines.append(
                    json.dumps(
                        {
                            "schema": VALID_TRANSITION_SCHEMA,
                            "dataset_global_index": global_index,
                            "dataset_episode_index": episode_index,
                            "dataset_frame_index": frame.frame_index,
                            "run_id": episode.run_id,
                            "source_control_index": frame.source_control_index,
                        },
                        sort_keys=True,
                        separators=(",", ":"),
                        allow_nan=False,
                    ).encode("utf-8")
                    + b"\n"
                )
            global_index += 1
    source_path.write_bytes(b"".join(source_lines))
    (meta / "wujihand_valid_transitions.jsonl").write_bytes(
        b"".join(transition_lines)
    )
    manifest = {
        "schema": EXPORT_MANIFEST_SCHEMA,
        "exporter_version": EXPORTER_VERSION,
        "lerobot": {"tag": LEROBOT_TAG, "commit": LEROBOT_COMMIT},
        "repo_id": repo_id,
        "robot_type": robot_type,
        "fps": 30,
        "episode_count": len(episodes),
        "frame_count": global_index,
        "episode_order": [episode.run_id for episode in episodes],
        "source_episode_order": [episode.source_run_id for episode in episodes],
        "episode_quality_grades": {
            episode.run_id: episode.quality_grade for episode in episodes
        },
        "source_release_decision_sha256": {
            episode.run_id: episode.release_decision_sha256 for episode in episodes
        },
        "visual_domain_variants": {
            episode.run_id: {
                "profile_sha256": episode.visual_domain_variant_profile_sha256,
                "variant": episode.visual_domain_variant.to_mapping(),
                "variant_sha256": episode.visual_domain_variant.digest_sha256,
            }
            for episode in episodes
        },
        "valid_transition_count": sum(
            frame.transition_valid for episode in episodes for frame in episode.frames
        ),
        "policy_features": features,
        "q54_profile_id": q54_profile.profile_id,
        "q54_profile_sha256": q54_profile.file_sha256,
        "source_alignment_digests": {
            episode.run_id: episode.alignment.digest_sha256 for episode in episodes
        },
        "source_vision_provenance_digests": {
            episode.run_id: episode.vision.provenance.digest_sha256
            for episode in episodes
        },
        "source_vision_provenance": {
            episode.run_id: episode.vision.provenance.to_mapping()
            for episode in episodes
        },
        "rgb_video_encoding": {
            "backend": "pyav",
            "codec": "libsvtav1",
            "pixel_format": "yuv420p",
            "gop": 2,
            "crf": 30,
            "preset": 12,
            "encoder_threads": 4,
            "seek_tolerance_s": LEROBOT_VIDEO_SEEK_TOLERANCE_S,
        },
        "success_semantics": "not_recorded_not_evaluated",
    }
    manifest_path = meta / "wujihand_export_manifest.json"
    manifest_path.write_bytes(_json_bytes(manifest))
    (root / "README.md").write_text(
        "# WujiHand q54 tri-view mini simulation dataset\n\n"
        "This candidate dataset contains absolute q54 joint targets and pre-action q54 "
        "observations at 30 Hz, mapped one-to-one from immutable "
        "30 Hz ROS 2/Isaac control facts. It includes scene RGB plus left/right wrist RGB.\n\n"
        "The wrist views use a synthetic 140-degree pinhole projection for simulation "
        "composition. They are not physical RealSense D405 calibration or specifications.\n\n"
        "No reward, success predicate or task-completion label is present. Raw q21, Tracker, "
        "qdot, post-state, object/link/contact and timing facts remain in source sidecars and "
        "are not policy observations. The source sidecar carries explicit missed-period and "
        "temporal-continuity masks. The valid-transition sidecar is the only training transition "
        "index; sequence consumers must never bridge a false continuity boundary. Episode "
        "quality grades A/B/C/D are metadata and C/D are not integrity rejection states. "
        "NERO real-hardware limits and mappings remain unverified "
        "until device readback.\n",
        encoding="utf-8",
    )
    return manifest_path


def _write_checksums(root: Path) -> Path:
    paths = tuple(
        sorted(
            (
                path
                for path in root.rglob("*")
                if path.is_file() and path.name != "wujihand_checksums.sha256"
            ),
            key=lambda path: path.relative_to(root).as_posix(),
        )
    )
    if not paths or any(path.is_symlink() for path in paths):
        raise ValueError("LeRobot artifact is empty or contains symbolic-link files")
    checksum_path = root / "wujihand_checksums.sha256"
    checksum_path.write_text(
        "".join(
            f"{_sha256(path)}  {path.relative_to(root).as_posix()}\n" for path in paths
        ),
        encoding="utf-8",
    )
    return checksum_path


def _publish(temporary: Path, destination: Path) -> None:
    lock = destination.parent / ".wujihand-mini-dataset-export.lock"
    with lock.open("a+b") as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        try:
            if destination.exists() or destination.is_symlink():
                raise FileExistsError("LeRobot dataset revision already exists")
            os.rename(temporary, destination)
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def export_collection(
    episodes: Sequence[PolicyEpisode],
    q54_profile: Q54JointProfile,
    destination: str | Path,
    *,
    repo_id: str,
    robot_type: str = "agile_nero_dual_wuji_hand2_simulation",
    dataset_factory: DatasetFactory = _default_factory,
    dataset_loader: DatasetLoader = _default_loader,
    image_loader: ImageLoader = _default_image_loader,
) -> ExportResult:
    """Export accepted episode bundles without mutating any source artifact."""

    rows = tuple(episodes)
    if not rows or len(rows) > 18:
        raise ValueError("LeRobot export requires between 1 and 18 accepted episodes")
    if len({episode.run_id for episode in rows}) != len(rows):
        raise ValueError("LeRobot export episode IDs must be unique")
    if any(not episode.frames for episode in rows):
        raise ValueError("LeRobot export refuses an empty episode")
    if not repo_id or repo_id != repo_id.strip() or any(char.isspace() for char in repo_id):
        raise ValueError("LeRobot repo_id must be a trimmed non-whitespace identifier")
    output = Path(destination)
    if output.is_symlink() or output.exists():
        raise FileExistsError("LeRobot dataset revision already exists")
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.parent.is_symlink():
        raise ValueError("LeRobot destination parent must not be a symbolic link")
    features = lerobot_feature_contract(q54_profile.canonical_names)
    temporary = output.parent / f".{output.name}.tmp-{uuid4().hex}"
    cache_root = output.parent / f".{output.name}.hf-cache-{uuid4().hex}"
    writer: DatasetHandle | None = None
    try:
        with _isolated_huggingface_cache(cache_root):
            writer = dataset_factory(
                repo_id=repo_id,
                root=temporary,
                fps=30,
                robot_type=robot_type,
                features=features,
            )
            for episode in rows:
                for frame in episode.frames:
                    payload: dict[str, object] = {
                        "observation.state": np.asarray(
                            frame.observation_q54_rad,
                            dtype=np.float32,
                        ),
                        "action": np.asarray(frame.action_q54_rad, dtype=np.float32),
                        "task": episode.task,
                    }
                    for key, path in zip(POLICY_IMAGE_KEYS, frame.image_paths, strict=True):
                        payload[key] = image_loader(path)
                    if frozenset(payload) != POLICY_FEATURE_KEYS | {"task"}:
                        raise ValueError("policy payload whitelist differs")
                    writer.add_frame(payload)
                # Mini collections favor deterministic, sandbox-safe sequential camera
                # encoding over a three-process pool; this does not change frame semantics.
                writer.save_episode(parallel_encoding=False)
            writer.finalize()
            reopened = dataset_loader(repo_id=repo_id, root=temporary)
            _validate_round_trip(reopened, rows, features)
        manifest_path = _write_sidecars(
            temporary,
            repo_id=repo_id,
            robot_type=robot_type,
            episodes=rows,
            q54_profile=q54_profile,
            features=features,
        )
        checksum_path = _write_checksums(temporary)
        manifest_sha256 = _sha256(manifest_path)
        checksums_sha256 = _sha256(checksum_path)
        _publish(temporary, output)
    except BaseException:
        if writer is not None:
            with suppress(Exception):
                writer.finalize()
        shutil.rmtree(temporary, ignore_errors=True)
        shutil.rmtree(cache_root, ignore_errors=True)
        raise
    return ExportResult(
        root=output.resolve(),
        repo_id=repo_id,
        episode_count=len(rows),
        frame_count=sum(len(episode.frames) for episode in rows),
        manifest_sha256=manifest_sha256,
        checksums_sha256=checksums_sha256,
    )


__all__ = [
    "EXPORTER_VERSION",
    "EXPORT_MANIFEST_SCHEMA",
    "LEROBOT_COMMIT",
    "ExportResult",
    "export_collection",
    "lerobot_feature_contract",
]
