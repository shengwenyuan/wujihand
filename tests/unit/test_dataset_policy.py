from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from dataset_camera_fixture import (
    dataset_profile_and_camera_inventories,
    rgb_png,
    vision_provenance,
)
from wujihand.dataset.alignment import RawTransition, build_exact_30hz_alignment
from wujihand.dataset.artifacts import write_alignment_artifact
from wujihand.dataset.episode import (
    DatasetEpisodeAnnotation,
    load_episode_annotation,
    write_episode_annotation,
)
from wujihand.dataset.policy import load_policy_episode
from wujihand.dataset.release import ReleaseDecision
from wujihand.dataset.release_artifact import write_release_decision_artifact
from wujihand.dataset.vision import (
    CAMERA_IDS,
    VISION_ARTIFACT_SCHEMA,
    VISION_FRAME_SCHEMA,
)


IDENTITY = [
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
]


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _episode(
    tmp_path: Path,
    source_rows: tuple[RawTransition, ...] | None = None,
) -> Path:
    run = tmp_path / "episode-001"
    run.mkdir()
    rows = (
        source_rows
        if source_rows is not None
        else tuple(
            RawTransition(
                run_id=run.name,
                control_index=index + 10,
                tick_id=index + 10,
                simulation_time_before_s=index / 30.0,
                simulation_time_after_s=(index + 1) / 30.0,
                pre_feedback_q54_rad=(float(index),) * 54,
                applied_target_q54_rad=(float(index) + 0.25,) * 54,
                post_feedback_q54_rad=(float(index + 1),) * 54,
                pre_action_state_digest=hashlib.sha256(
                    f"state-{index}".encode()
                ).hexdigest(),
            )
            for index in range(3)
        )
    )
    alignment = build_exact_30hz_alignment(rows)
    write_alignment_artifact(run, alignment)
    write_episode_annotation(
        run,
        DatasetEpisodeAnnotation(
            run_id=run.name,
            task="Move the left hand toward the banana and attempt to grasp it.",
            operator_note="diagnostic",
        ),
    )

    vision = run / "derived" / "vision"
    vision.mkdir()
    profile, inventories = dataset_profile_and_camera_inventories()
    provenance = vision_provenance(profile, inventories)
    profile_hashes = {item.camera_id: item.profile_sha256 for item in inventories}
    records = []
    for frame in alignment.frames:
        for camera_id in CAMERA_IDS:
            relative = Path(camera_id) / f"{frame.dataset_frame_index:06d}.png"
            payload = vision / relative
            payload.parent.mkdir(exist_ok=True)
            payload.write_bytes(rgb_png(20 + frame.dataset_frame_index))
            records.append(
                {
                    "schema": VISION_FRAME_SCHEMA,
                    "run_id": run.name,
                    "collection_id": provenance.collection_id,
                    "provenance_sha256": provenance.digest_sha256,
                    "camera_id": camera_id,
                    "dataset_frame_index": frame.dataset_frame_index,
                    "source_control_index": frame.source_control_index,
                    "source_tick_id": frame.source_tick_id,
                    "phase": "pre_action",
                    "simulation_time_s": frame.simulation_time_s,
                    "source_state_digest": frame.source_state_digest,
                    "payload_path": relative.as_posix(),
                    "payload_sha256": _sha256(payload),
                    "width_px": 640,
                    "height_px": 480,
                    "encoding": "rgb8",
                    "camera_profile_sha256": profile_hashes[camera_id],
                    "completed_frame_identity": (
                        f"render-{frame.dataset_frame_index}-{camera_id}"
                    ),
                    "parent_frame_id": f"{camera_id}_parent",
                    "world_from_parent_row_major": IDENTITY,
                    "world_from_camera_optical_row_major": IDENTITY,
                }
            )
    (vision / "frame_index.jsonl").write_text(
        "".join(json.dumps(record, sort_keys=True) + "\n" for record in records),
        encoding="utf-8",
    )
    (vision / "manifest.json").write_text(
        json.dumps(
            {
                "schema": VISION_ARTIFACT_SCHEMA,
                "run_id": run.name,
                "alignment_digest_sha256": alignment.digest_sha256,
                "camera_ids": list(CAMERA_IDS),
                "frame_count": len(alignment.frames),
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
    material = sorted(path for path in vision.rglob("*") if path.is_file())
    (vision / "checksums.sha256").write_text(
        "".join(
            f"{_sha256(path)}  {path.relative_to(vision).as_posix()}\n"
            for path in material
        ),
        encoding="utf-8",
    )
    return run


def test_policy_episode_closes_annotation_alignment_and_three_camera_payloads(
    tmp_path: Path,
) -> None:
    run = _episode(tmp_path)
    write_release_decision_artifact(
        run,
        ReleaseDecision(run_id=run.name, passed=True, gates=()),
    )

    episode = load_policy_episode(run)

    assert episode.run_id == "episode-001"
    assert len(episode.frames) == 3
    assert episode.frames[1].source_control_index == 11
    assert len(episode.frames[0].image_paths) == 3
    assert episode.task.startswith("Move the left hand")
    assert episode.quality_grade == "A"


def test_policy_episode_rejects_non_exact_vision_source_join(tmp_path: Path) -> None:
    run = _episode(tmp_path)
    write_release_decision_artifact(
        run,
        ReleaseDecision(run_id=run.name, passed=True, gates=()),
    )
    path = run / "derived" / "vision" / "frame_index.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    for row in rows[:3]:
        row["source_control_index"] += 2
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    vision = path.parent
    material = sorted(
        item
        for item in vision.rglob("*")
        if item.is_file() and item.name != "checksums.sha256"
    )
    (vision / "checksums.sha256").write_text(
        "".join(
            f"{_sha256(item)}  {item.relative_to(vision).as_posix()}\n"
            for item in material
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="exactly match"):
        load_policy_episode(run)


def test_episode_annotation_is_idempotent_but_not_mutable(tmp_path: Path) -> None:
    run = tmp_path / "episode-001"
    run.mkdir()
    annotation = DatasetEpisodeAnnotation(
        run_id=run.name,
        task="Move both hands toward the tabletop objects.",
        operator_note="",
    )

    first = write_episode_annotation(run, annotation)
    second = write_episode_annotation(run, annotation)

    assert first == second
    assert load_episode_annotation(run) == annotation
    with pytest.raises(FileExistsError, match="different"):
        write_episode_annotation(
            run,
            DatasetEpisodeAnnotation(
                run_id=run.name,
                task="A different task.",
                operator_note="",
            ),
        )
