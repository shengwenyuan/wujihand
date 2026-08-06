"""Fail-closed loading of a separately rendered three-camera vision artifact."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import hashlib
import json
import math
from numbers import Real
import os
from pathlib import Path
import shutil
import tempfile
from typing import Final, cast
import zlib

from wujihand.domain.recording import validate_recording_token, validate_run_id

from .camera import DatasetCameraRuntimeInventory


VISION_ARTIFACT_SCHEMA: Final = "wujihand.dataset_vision_artifact.v1"
VISION_FRAME_SCHEMA: Final = "wujihand.dataset_vision_frame.v1"
VISION_PROVENANCE_SCHEMA: Final = "wujihand.dataset_vision_provenance.v1"
CAMERA_IDS: Final = ("scene_rgb", "left_wrist_rgb", "right_wrist_rgb")
DATASET_RENDERER_BACKEND: Final = "RayTracedLighting"
DATASET_LIGHTING_IDENTITY: Final = "session_workcell_authored_lighting"
DATASET_COLOR_SPACE: Final = "isaac_rgb_annotator_srgb"
_PNG_SIGNATURE: Final = b"\x89PNG\r\n\x1a\n"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_checksums(root: Path) -> dict[str, str]:
    path = root / "checksums.sha256"
    if path.is_symlink():
        raise ValueError("vision checksums must not be a symbolic link")
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ValueError("vision checksums cannot be read") from exc
    checksums: dict[str, str] = {}
    for line_number, line in enumerate(lines, start=1):
        parts = line.split(maxsplit=1)
        if len(parts) != 2:
            raise ValueError(f"vision checksum line {line_number} is invalid")
        digest = _digest(parts[0], field=f"checksums[{line_number}]")
        relative_text = parts[1].lstrip("*")
        relative = _safe_relative(relative_text, field=f"checksums[{line_number}].path")
        if relative.as_posix() in checksums:
            raise ValueError("vision checksums contain a duplicate path")
        target = _safe_payload(root, relative)
        if not target.is_file() or _sha256(target) != digest:
            raise ValueError(f"vision artifact checksum differs: {relative}")
        checksums[relative.as_posix()] = digest
    if not checksums:
        raise ValueError("vision checksums must not be empty")
    return checksums


def _digest(value: object, *, field: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(char not in "0123456789abcdef" for char in value)
    ):
        raise ValueError(f"{field} must be a lowercase SHA-256")
    return value


def _safe_relative(value: object, *, field: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a project-relative path")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or value.startswith("~") or "\\" in value:
        raise ValueError(f"{field} must be a safe relative path")
    return path


def _safe_payload(root: Path, relative: Path) -> Path:
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise ValueError(f"vision payload path contains a symbolic link: {relative}")
    resolved = current.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"vision payload escapes artifact root: {relative}") from exc
    return resolved


def validate_rgb8_png(payload: bytes, *, field: str = "rgb_payload") -> bytes:
    if len(payload) < 33 or payload[:8] != _PNG_SIGNATURE:
        raise ValueError(f"{field} must be a PNG payload")
    if int.from_bytes(payload[8:12], "big") != 13 or payload[12:16] != b"IHDR":
        raise ValueError(f"{field} must start with a PNG IHDR chunk")
    width = int.from_bytes(payload[16:20], "big")
    height = int.from_bytes(payload[20:24], "big")
    bit_depth = payload[24]
    color_type = payload[25]
    compression, filtering, interlace = payload[26:29]
    if (
        (width, height) != (640, 480)
        or bit_depth != 8
        or color_type != 2
        or compression != 0
        or filtering != 0
        or interlace != 0
    ):
        raise ValueError(f"{field} must be non-interlaced 640x480 RGB8 PNG")
    offset = 8
    chunk_types: list[bytes] = []
    compressed = bytearray()
    while offset < len(payload):
        if offset + 12 > len(payload):
            raise ValueError(f"{field} PNG chunk header is truncated")
        length = int.from_bytes(payload[offset : offset + 4], "big")
        kind = payload[offset + 4 : offset + 8]
        end = offset + 12 + length
        if end > len(payload):
            raise ValueError(f"{field} PNG chunk payload is truncated")
        data = payload[offset + 8 : offset + 8 + length]
        expected_crc = int.from_bytes(payload[offset + 8 + length : end], "big")
        if zlib.crc32(kind + data) & 0xFFFFFFFF != expected_crc:
            raise ValueError(f"{field} PNG chunk CRC differs")
        if kind not in {b"IHDR", b"IDAT", b"IEND"}:
            raise ValueError(f"{field} PNG must use the canonical RGB truth chunks")
        chunk_types.append(kind)
        if kind == b"IDAT":
            compressed.extend(data)
        if kind == b"IEND":
            if data or end != len(payload):
                raise ValueError(f"{field} PNG IEND or trailing bytes are invalid")
            break
        offset = end
    if (
        not chunk_types
        or chunk_types[0] != b"IHDR"
        or chunk_types[-1] != b"IEND"
        or chunk_types.count(b"IHDR") != 1
        or chunk_types.count(b"IEND") != 1
        or b"IDAT" not in chunk_types
    ):
        raise ValueError(f"{field} PNG chunk order is incomplete")
    try:
        scanlines = zlib.decompress(bytes(compressed))
    except zlib.error as exc:
        raise ValueError(f"{field} PNG IDAT cannot be decoded") from exc
    stride = 1 + 640 * 3
    if len(scanlines) != stride * 480:
        raise ValueError(f"{field} PNG decoded raster is invalid")
    if any(scanlines[row * stride] != 0 for row in range(480)):
        raise ValueError(f"{field} PNG must use canonical unfiltered RGB scanlines")
    pixels = b"".join(
        scanlines[row * stride + 1 : (row + 1) * stride] for row in range(480)
    )
    if not pixels or all(value == 0 for value in pixels) or all(
        value == 255 for value in pixels
    ):
        raise ValueError(f"{field} PNG must not be all black or all white")
    return pixels


def _matrix4(value: object, *, field: str) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{field} must contain 16 finite values")
    raw = tuple(value)
    if len(raw) != 16 or any(isinstance(item, bool) or not isinstance(item, Real) for item in raw):
        raise ValueError(f"{field} must contain 16 finite values")
    result = tuple(float(item) for item in raw)
    if any(not math.isfinite(item) for item in result):
        raise ValueError(f"{field} must contain 16 finite values")
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
        if not math.isclose(sum(item * item for item in row), 1.0, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError(f"{field} rotation must be orthonormal")
    for first, second in ((0, 1), (0, 2), (1, 2)):
        dot = sum(rotation[first][index] * rotation[second][index] for index in range(3))
        if not math.isclose(dot, 0.0, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError(f"{field} rotation must be orthonormal")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(f"{field} rotation determinant must be +1")
    return result


def _matrix_product(left: tuple[float, ...], right: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(
        sum(left[row * 4 + inner] * right[inner * 4 + column] for inner in range(4))
        for row in range(4)
        for column in range(4)
    )


def _json_digest(value: object) -> str:
    return hashlib.sha256(
        json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()


@dataclass(frozen=True, slots=True)
class DatasetVisionProvenance:
    """Artifact-wide scene and renderer closure referenced by every frame row."""

    collection_id: str
    dataset_profile_sha256: str
    deployment_sha256: str
    session_sha256: str
    assembly_sha256: str
    workcell_sha256: str
    renderer_identity: str
    renderer_backend: str
    lighting_identity: str
    color_space: str
    motion_blur_enabled: bool
    renderer_configuration_sha256: str

    def __post_init__(self) -> None:
        if (
            self.renderer_backend != DATASET_RENDERER_BACKEND
            or self.lighting_identity != DATASET_LIGHTING_IDENTITY
            or self.color_space != DATASET_COLOR_SPACE
            or self.motion_blur_enabled
        ):
            raise ValueError("vision renderer/color/lighting/motion-blur contract differs")

    @classmethod
    def create(
        cls,
        *,
        collection_id: str,
        dataset_profile_sha256: str,
        deployment_sha256: str,
        session_sha256: str,
        assembly_sha256: str,
        workcell_sha256: str,
        renderer_identity: str,
        renderer_backend: str,
        lighting_identity: str,
        color_space: str,
        motion_blur_enabled: bool,
        camera_profile_sha256_by_id: Mapping[str, str],
    ) -> DatasetVisionProvenance:
        renderer = validate_recording_token(
            renderer_identity,
            field="renderer_identity",
        )
        backend = validate_recording_token(
            renderer_backend,
            field="renderer_backend",
        )
        lighting = validate_recording_token(
            lighting_identity,
            field="lighting_identity",
        )
        color = validate_recording_token(color_space, field="color_space")
        if type(motion_blur_enabled) is not bool:
            raise ValueError("motion_blur_enabled must be boolean")
        if tuple(camera_profile_sha256_by_id) != CAMERA_IDS:
            raise ValueError("vision provenance camera profile order differs")
        hashes = {
            "dataset_profile_sha256": _digest(
                dataset_profile_sha256,
                field="dataset_profile_sha256",
            ),
            "deployment_sha256": _digest(
                deployment_sha256,
                field="deployment_sha256",
            ),
            "session_sha256": _digest(session_sha256, field="session_sha256"),
            "assembly_sha256": _digest(assembly_sha256, field="assembly_sha256"),
            "workcell_sha256": _digest(workcell_sha256, field="workcell_sha256"),
        }
        camera_hashes = {
            camera_id: _digest(
                camera_profile_sha256_by_id[camera_id],
                field=f"camera_profile_sha256_by_id.{camera_id}",
            )
            for camera_id in CAMERA_IDS
        }
        configuration = _json_digest(
            {
                "renderer_identity": renderer,
                "renderer_backend": backend,
                "lighting_identity": lighting,
                "color_space": color,
                "motion_blur_enabled": motion_blur_enabled,
                "dataset_profile_sha256": hashes["dataset_profile_sha256"],
                "camera_profile_sha256_by_id": camera_hashes,
            }
        )
        return cls(
            collection_id=validate_recording_token(
                collection_id,
                field="collection_id",
            ),
            renderer_identity=renderer,
            renderer_backend=backend,
            lighting_identity=lighting,
            color_space=color,
            motion_blur_enabled=motion_blur_enabled,
            renderer_configuration_sha256=configuration,
            **hashes,
        )

    @property
    def digest_sha256(self) -> str:
        return _json_digest(self._payload_mapping())

    def _payload_mapping(self) -> dict[str, object]:
        return {
            "schema": VISION_PROVENANCE_SCHEMA,
            "collection_id": self.collection_id,
            "dataset_profile_sha256": self.dataset_profile_sha256,
            "deployment_sha256": self.deployment_sha256,
            "session_sha256": self.session_sha256,
            "assembly_sha256": self.assembly_sha256,
            "workcell_sha256": self.workcell_sha256,
            "renderer_identity": self.renderer_identity,
            "renderer_backend": self.renderer_backend,
            "lighting_identity": self.lighting_identity,
            "color_space": self.color_space,
            "motion_blur_enabled": self.motion_blur_enabled,
            "renderer_configuration_sha256": self.renderer_configuration_sha256,
        }

    def to_mapping(self) -> dict[str, object]:
        return {**self._payload_mapping(), "digest_sha256": self.digest_sha256}

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> DatasetVisionProvenance:
        if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
            raise ValueError(f"{field} must be a string-keyed mapping")
        data = cast(Mapping[str, object], value)
        keys = {
            "schema",
            "collection_id",
            "dataset_profile_sha256",
            "deployment_sha256",
            "session_sha256",
            "assembly_sha256",
            "workcell_sha256",
            "renderer_identity",
            "renderer_backend",
            "lighting_identity",
            "color_space",
            "motion_blur_enabled",
            "renderer_configuration_sha256",
            "digest_sha256",
        }
        if set(data) != keys or data["schema"] != VISION_PROVENANCE_SCHEMA:
            raise ValueError(f"{field} schema or keys differ")
        if type(data["motion_blur_enabled"]) is not bool:
            raise ValueError(f"{field}.motion_blur_enabled must be boolean")
        provenance = cls(
            collection_id=validate_recording_token(
                data["collection_id"],
                field=f"{field}.collection_id",
            ),
            dataset_profile_sha256=_digest(
                data["dataset_profile_sha256"],
                field=f"{field}.dataset_profile_sha256",
            ),
            deployment_sha256=_digest(
                data["deployment_sha256"],
                field=f"{field}.deployment_sha256",
            ),
            session_sha256=_digest(
                data["session_sha256"],
                field=f"{field}.session_sha256",
            ),
            assembly_sha256=_digest(
                data["assembly_sha256"],
                field=f"{field}.assembly_sha256",
            ),
            workcell_sha256=_digest(
                data["workcell_sha256"],
                field=f"{field}.workcell_sha256",
            ),
            renderer_identity=validate_recording_token(
                data["renderer_identity"],
                field=f"{field}.renderer_identity",
            ),
            renderer_backend=validate_recording_token(
                data["renderer_backend"],
                field=f"{field}.renderer_backend",
            ),
            lighting_identity=validate_recording_token(
                data["lighting_identity"],
                field=f"{field}.lighting_identity",
            ),
            color_space=validate_recording_token(
                data["color_space"],
                field=f"{field}.color_space",
            ),
            motion_blur_enabled=data["motion_blur_enabled"],
            renderer_configuration_sha256=_digest(
                data["renderer_configuration_sha256"],
                field=f"{field}.renderer_configuration_sha256",
            ),
        )
        if _digest(data["digest_sha256"], field=f"{field}.digest_sha256") != (
            provenance.digest_sha256
        ):
            raise ValueError(f"{field} digest differs")
        return provenance

    def validate_camera_configuration(
        self,
        camera_profile_sha256_by_id: Mapping[str, str],
    ) -> None:
        expected = DatasetVisionProvenance.create(
            collection_id=self.collection_id,
            dataset_profile_sha256=self.dataset_profile_sha256,
            deployment_sha256=self.deployment_sha256,
            session_sha256=self.session_sha256,
            assembly_sha256=self.assembly_sha256,
            workcell_sha256=self.workcell_sha256,
            renderer_identity=self.renderer_identity,
            renderer_backend=self.renderer_backend,
            lighting_identity=self.lighting_identity,
            color_space=self.color_space,
            motion_blur_enabled=self.motion_blur_enabled,
            camera_profile_sha256_by_id=camera_profile_sha256_by_id,
        )
        if self.renderer_configuration_sha256 != expected.renderer_configuration_sha256:
            raise ValueError("vision renderer configuration digest differs")


@dataclass(frozen=True, slots=True)
class VisionFrameRecord:
    run_id: str
    collection_id: str
    provenance_sha256: str
    camera_id: str
    dataset_frame_index: int
    source_control_index: int
    source_tick_id: int
    phase: str
    simulation_time_s: float
    source_state_digest: str
    payload_path: str
    payload_sha256: str
    width_px: int
    height_px: int
    encoding: str
    camera_profile_sha256: str
    completed_frame_identity: str
    parent_frame_id: str
    world_from_parent_row_major: tuple[float, ...]
    world_from_camera_optical_row_major: tuple[float, ...]

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> VisionFrameRecord:
        if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
            raise ValueError(f"{field} must be a string-keyed mapping")
        data = cast(Mapping[str, object], value)
        expected = frozenset(
            {
                "schema",
                "run_id",
                "collection_id",
                "provenance_sha256",
                "camera_id",
                "dataset_frame_index",
                "source_control_index",
                "source_tick_id",
                "phase",
                "simulation_time_s",
                "source_state_digest",
                "payload_path",
                "payload_sha256",
                "width_px",
                "height_px",
                "encoding",
                "camera_profile_sha256",
                "completed_frame_identity",
                "parent_frame_id",
                "world_from_parent_row_major",
                "world_from_camera_optical_row_major",
            }
        )
        if frozenset(data) != expected or data["schema"] != VISION_FRAME_SCHEMA:
            raise ValueError(f"{field} schema or keys differ")
        run_id = validate_run_id(data["run_id"])
        collection_id = validate_recording_token(
            data["collection_id"],
            field=f"{field}.collection_id",
        )
        camera_id = validate_recording_token(data["camera_id"], field=f"{field}.camera_id")
        if camera_id not in CAMERA_IDS:
            raise ValueError(f"{field}.camera_id is not a dataset camera")
        for key in (
            "dataset_frame_index",
            "source_control_index",
            "source_tick_id",
            "width_px",
            "height_px",
        ):
            if type(data[key]) is not int or cast(int, data[key]) < 0:
                raise ValueError(f"{field}.{key} must be a non-negative integer")
        simulation_time = data["simulation_time_s"]
        if isinstance(simulation_time, bool) or not isinstance(simulation_time, (int, float)):
            raise ValueError(f"{field}.simulation_time_s must be finite")
        simulation_time_s = float(simulation_time)
        if not math.isfinite(simulation_time_s) or simulation_time_s < 0.0:
            raise ValueError(f"{field}.simulation_time_s must be finite and non-negative")
        if data["phase"] != "pre_action":
            raise ValueError(f"{field}.phase must be pre_action")
        if data["encoding"] != "rgb8":
            raise ValueError(f"{field}.encoding must be rgb8")
        if (data["width_px"], data["height_px"]) != (640, 480):
            raise ValueError(f"{field} must be 640x480")
        completed = validate_recording_token(
            data["completed_frame_identity"],
            field=f"{field}.completed_frame_identity",
        )
        return cls(
            run_id=run_id,
            collection_id=collection_id,
            provenance_sha256=_digest(
                data["provenance_sha256"],
                field=f"{field}.provenance_sha256",
            ),
            camera_id=camera_id,
            dataset_frame_index=cast(int, data["dataset_frame_index"]),
            source_control_index=cast(int, data["source_control_index"]),
            source_tick_id=cast(int, data["source_tick_id"]),
            phase="pre_action",
            simulation_time_s=simulation_time_s,
            source_state_digest=_digest(
                data["source_state_digest"],
                field=f"{field}.source_state_digest",
            ),
            payload_path=_safe_relative(
                data["payload_path"],
                field=f"{field}.payload_path",
            ).as_posix(),
            payload_sha256=_digest(
                data["payload_sha256"],
                field=f"{field}.payload_sha256",
            ),
            width_px=cast(int, data["width_px"]),
            height_px=cast(int, data["height_px"]),
            encoding="rgb8",
            camera_profile_sha256=_digest(
                data["camera_profile_sha256"],
                field=f"{field}.camera_profile_sha256",
            ),
            completed_frame_identity=completed,
            parent_frame_id=validate_recording_token(
                data["parent_frame_id"],
                field=f"{field}.parent_frame_id",
            ),
            world_from_parent_row_major=_matrix4(
                data["world_from_parent_row_major"],
                field=f"{field}.world_from_parent_row_major",
            ),
            world_from_camera_optical_row_major=_matrix4(
                data["world_from_camera_optical_row_major"],
                field=f"{field}.world_from_camera_optical_row_major",
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": VISION_FRAME_SCHEMA,
            "run_id": self.run_id,
            "collection_id": self.collection_id,
            "provenance_sha256": self.provenance_sha256,
            "camera_id": self.camera_id,
            "dataset_frame_index": self.dataset_frame_index,
            "source_control_index": self.source_control_index,
            "source_tick_id": self.source_tick_id,
            "phase": self.phase,
            "simulation_time_s": self.simulation_time_s,
            "source_state_digest": self.source_state_digest,
            "payload_path": self.payload_path,
            "payload_sha256": self.payload_sha256,
            "width_px": self.width_px,
            "height_px": self.height_px,
            "encoding": self.encoding,
            "camera_profile_sha256": self.camera_profile_sha256,
            "completed_frame_identity": self.completed_frame_identity,
            "parent_frame_id": self.parent_frame_id,
            "world_from_parent_row_major": list(self.world_from_parent_row_major),
            "world_from_camera_optical_row_major": list(self.world_from_camera_optical_row_major),
        }


@dataclass(frozen=True, slots=True)
class VisionArtifact:
    root: Path
    run_id: str
    alignment_digest_sha256: str
    frame_count: int
    renderer_identity: str
    provenance: DatasetVisionProvenance
    camera_runtime_inventories: tuple[DatasetCameraRuntimeInventory, ...]
    frames: tuple[VisionFrameRecord, ...]

    def frame(self, frame_index: int, camera_id: str) -> VisionFrameRecord:
        matches = tuple(
            item
            for item in self.frames
            if item.dataset_frame_index == frame_index and item.camera_id == camera_id
        )
        if len(matches) != 1:
            raise KeyError(f"vision frame is not unique: {frame_index}/{camera_id}")
        return matches[0]

    def payload(self, record: VisionFrameRecord) -> Path:
        return _safe_payload(self.root, Path(record.payload_path))


class VisionArtifactBuilder:
    """Stream lossless PNG frames into a temporary artifact, then publish atomically."""

    def __init__(
        self,
        run_root: str | Path,
        *,
        run_id: str,
        alignment_digest_sha256: str,
        frame_count: int,
        renderer_identity: str,
        provenance: DatasetVisionProvenance,
        camera_runtime_inventories: tuple[DatasetCameraRuntimeInventory, ...],
    ) -> None:
        identifier = validate_run_id(run_id)
        raw_run = Path(run_root)
        if raw_run.is_symlink():
            raise ValueError("vision run root must not be a symbolic link")
        root = raw_run.resolve()
        if not root.is_dir() or root.name != identifier:
            raise ValueError("vision run root must exist and its name must equal run_id")
        if type(frame_count) is not int or frame_count <= 0:
            raise ValueError("vision frame_count must be a positive integer")
        self.run_root = root
        self.run_id = identifier
        self.alignment_digest_sha256 = _digest(
            alignment_digest_sha256,
            field="alignment_digest_sha256",
        )
        self.frame_count = frame_count
        self.renderer_identity = validate_recording_token(
            renderer_identity,
            field="renderer_identity",
        )
        if provenance.renderer_identity != self.renderer_identity:
            raise ValueError("vision provenance renderer identity differs")
        self.provenance = provenance
        if tuple(item.camera_id for item in camera_runtime_inventories) != CAMERA_IDS:
            raise ValueError("vision camera runtime inventory order differs")
        provenance.validate_camera_configuration(
            {item.camera_id: item.profile_sha256 for item in camera_runtime_inventories}
        )
        self.camera_runtime_inventories = camera_runtime_inventories
        self.derived = root / "derived"
        self.derived.mkdir(exist_ok=True)
        if self.derived.is_symlink():
            raise ValueError("vision derived root must not be a symbolic link")
        self.destination = self.derived / "vision"
        if self.destination.exists() or self.destination.is_symlink():
            raise FileExistsError("vision artifact already exists")
        self.temporary = Path(tempfile.mkdtemp(prefix=".vision-", dir=self.derived))
        self._records: dict[tuple[int, str], VisionFrameRecord] = {}
        self._published = False

    def add_rgb_png(self, record: VisionFrameRecord, payload: bytes) -> Path:
        if self._published or not self.temporary.is_dir():
            raise RuntimeError("vision artifact builder is no longer writable")
        if record.run_id != self.run_id:
            raise ValueError("vision record and builder run IDs differ")
        if (
            record.collection_id != self.provenance.collection_id
            or record.provenance_sha256 != self.provenance.digest_sha256
        ):
            raise ValueError("vision record and artifact provenance differ")
        if not 0 <= record.dataset_frame_index < self.frame_count:
            raise ValueError("vision record frame index is outside the artifact")
        expected_path = (
            Path(record.camera_id) / f"{record.dataset_frame_index:06d}.png"
        ).as_posix()
        if record.payload_path != expected_path:
            raise ValueError("vision payload path differs from the canonical layout")
        key = (record.dataset_frame_index, record.camera_id)
        if key in self._records:
            raise ValueError("vision frame/camera record is duplicated")
        validate_rgb8_png(payload, field=record.payload_path)
        if hashlib.sha256(payload).hexdigest() != record.payload_sha256:
            raise ValueError("vision payload digest differs from the frame record")
        previous = self._records.get((record.dataset_frame_index - 1, record.camera_id))
        if (
            previous is not None
            and previous.payload_sha256 == record.payload_sha256
            and previous.source_state_digest != record.source_state_digest
        ):
            raise ValueError("vision payload repeated across distinct source states")
        destination = self.temporary / record.payload_path
        destination.parent.mkdir(exist_ok=True)
        destination.write_bytes(payload)
        self._records[key] = record
        return destination

    def publish(self) -> VisionArtifact:
        if self._published:
            raise RuntimeError("vision artifact has already been published")
        expected_keys = {
            (frame_index, camera_id)
            for frame_index in range(self.frame_count)
            for camera_id in CAMERA_IDS
        }
        if set(self._records) != expected_keys:
            raise ValueError("vision builder does not contain the complete three-camera grid")
        records = tuple(
            self._records[(frame_index, camera_id)]
            for frame_index in range(self.frame_count)
            for camera_id in CAMERA_IDS
        )
        (self.temporary / "frame_index.jsonl").write_text(
            "".join(
                json.dumps(
                    record.to_mapping(),
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                )
                + "\n"
                for record in records
            ),
            encoding="utf-8",
        )
        (self.temporary / "manifest.json").write_text(
            json.dumps(
                {
                    "schema": VISION_ARTIFACT_SCHEMA,
                    "run_id": self.run_id,
                    "alignment_digest_sha256": self.alignment_digest_sha256,
                    "camera_ids": list(CAMERA_IDS),
                    "frame_count": self.frame_count,
                    "renderer_identity": self.renderer_identity,
                    "provenance": self.provenance.to_mapping(),
                    "camera_runtime_inventories": [
                        item.to_mapping() for item in self.camera_runtime_inventories
                    ],
                },
                sort_keys=True,
                indent=2,
                allow_nan=False,
            )
            + "\n",
            encoding="utf-8",
        )
        material = tuple(
            sorted(
                (path for path in self.temporary.rglob("*") if path.is_file()),
                key=lambda path: path.relative_to(self.temporary).as_posix(),
            )
        )
        (self.temporary / "checksums.sha256").write_text(
            "".join(
                f"{_sha256(path)}  {path.relative_to(self.temporary).as_posix()}\n"
                for path in material
            ),
            encoding="utf-8",
        )
        load_vision_artifact(
            self.temporary,
            expected_run_id=self.run_id,
            expected_alignment_digest=self.alignment_digest_sha256,
        )
        if self.destination.exists() or self.destination.is_symlink():
            raise FileExistsError("vision artifact appeared during publication")
        os.rename(self.temporary, self.destination)
        self._published = True
        return load_vision_artifact(
            self.destination,
            expected_run_id=self.run_id,
            expected_alignment_digest=self.alignment_digest_sha256,
        )

    def abort(self) -> None:
        if not self._published:
            shutil.rmtree(self.temporary, ignore_errors=True)


def load_vision_artifact(
    vision_root: str | Path,
    *,
    expected_run_id: str,
    expected_alignment_digest: str,
) -> VisionArtifact:
    raw_root = Path(vision_root)
    if raw_root.is_symlink():
        raise ValueError("vision root must not be a symbolic link")
    root = raw_root.resolve()
    if not root.is_dir():
        raise ValueError("vision root must be a directory")
    checksums = _load_checksums(root)
    if not {"manifest.json", "frame_index.jsonl"}.issubset(checksums):
        raise ValueError("vision checksums omit manifest or frame index")
    try:
        manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError("vision manifest is not valid JSON") from exc
    manifest_keys = {
        "schema",
        "run_id",
        "alignment_digest_sha256",
        "camera_ids",
        "frame_count",
        "renderer_identity",
        "provenance",
        "camera_runtime_inventories",
    }
    if (
        not isinstance(manifest, dict)
        or set(manifest) != manifest_keys
        or manifest.get("schema") != VISION_ARTIFACT_SCHEMA
    ):
        raise ValueError("vision manifest schema is invalid")
    run_id = validate_run_id(manifest.get("run_id"))
    if run_id != expected_run_id:
        raise ValueError("vision and expected run IDs differ")
    alignment_digest = _digest(
        manifest.get("alignment_digest_sha256"),
        field="alignment_digest_sha256",
    )
    if alignment_digest != expected_alignment_digest:
        raise ValueError("vision and alignment digests differ")
    if manifest.get("camera_ids") != list(CAMERA_IDS):
        raise ValueError("vision manifest must contain the frozen three-camera order")
    frame_count = manifest.get("frame_count")
    if type(frame_count) is not int or frame_count <= 0:
        raise ValueError("vision manifest frame_count must be positive")
    renderer = validate_recording_token(
        manifest.get("renderer_identity"),
        field="renderer_identity",
    )
    provenance = DatasetVisionProvenance.from_mapping(
        manifest.get("provenance"),
        field="provenance",
    )
    if provenance.renderer_identity != renderer:
        raise ValueError("vision manifest renderer and provenance differ")
    raw_inventories = manifest.get("camera_runtime_inventories")
    if not isinstance(raw_inventories, list):
        raise ValueError("vision manifest camera runtime inventories are invalid")
    camera_runtime_inventories = tuple(
        DatasetCameraRuntimeInventory.from_mapping(
            item,
            field=f"camera_runtime_inventories[{index}]",
        )
        for index, item in enumerate(raw_inventories)
    )
    if tuple(item.camera_id for item in camera_runtime_inventories) != CAMERA_IDS:
        raise ValueError("vision camera runtime inventory order differs")
    if len({item.camera_prim_path for item in camera_runtime_inventories}) != len(CAMERA_IDS):
        raise ValueError("vision Camera prim inventory is not unique")
    if len({item.render_product_path for item in camera_runtime_inventories}) != len(CAMERA_IDS):
        raise ValueError("vision RenderProduct inventory is not unique")
    if len({item.camera_frame_id for item in camera_runtime_inventories}) != len(CAMERA_IDS):
        raise ValueError("vision camera-frame inventory is not unique")
    if len({item.optical_frame_id for item in camera_runtime_inventories}) != len(CAMERA_IDS):
        raise ValueError("vision optical-frame inventory is not unique")
    for item in camera_runtime_inventories:
        artifact_hashes = (
            item.mount_visual_sha256,
            item.camera_visual_sha256,
            item.generation_report_sha256,
        )
        if item.camera_id == "scene_rgb":
            if any(value is not None for value in artifact_hashes):
                raise ValueError("scene camera must remain a Workcell-owned logical carrier")
        elif any(value is None for value in artifact_hashes):
            raise ValueError("wrist camera static artifact provenance is incomplete")
    inventory_by_camera = {
        item.camera_id: item for item in camera_runtime_inventories
    }
    provenance.validate_camera_configuration(
        {item.camera_id: item.profile_sha256 for item in camera_runtime_inventories}
    )

    records: list[VisionFrameRecord] = []
    for line_number, line in enumerate(
        (root / "frame_index.jsonl").read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        if not line:
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"vision frame index line {line_number} is invalid") from exc
        records.append(VisionFrameRecord.from_mapping(value, field=f"frames[{line_number}]"))
    expected_record_count = frame_count * len(CAMERA_IDS)
    if len(records) != expected_record_count:
        raise ValueError("vision frame index does not contain three records per frame")
    expected_order = tuple(
        (frame_index, camera_id)
        for frame_index in range(frame_count)
        for camera_id in CAMERA_IDS
    )
    if tuple((item.dataset_frame_index, item.camera_id) for item in records) != expected_order:
        raise ValueError("vision frame index rows are reordered")

    grouped: dict[int, list[VisionFrameRecord]] = defaultdict(list)
    completed_identities: set[str] = set()
    previous_by_camera: dict[str, VisionFrameRecord] = {}
    for record in records:
        if record.run_id != run_id:
            raise ValueError("vision frame run ID differs")
        if (
            record.collection_id != provenance.collection_id
            or record.provenance_sha256 != provenance.digest_sha256
        ):
            raise ValueError("vision frame provenance differs from the manifest")
        if (
            record.camera_profile_sha256
            != inventory_by_camera[record.camera_id].profile_sha256
        ):
            raise ValueError("vision frame camera profile differs from runtime inventory")
        inventory = inventory_by_camera[record.camera_id]
        if record.parent_frame_id != inventory.parent_frame_id:
            raise ValueError("vision frame parent identity differs from runtime inventory")
        expected_world_from_camera = _matrix_product(
            record.world_from_parent_row_major,
            inventory.parent_from_camera_optical_row_major,
        )
        if any(
            not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-6)
            for actual, expected in zip(
                record.world_from_camera_optical_row_major,
                expected_world_from_camera,
                strict=True,
            )
        ):
            raise ValueError("vision frame parent/static/world extrinsic closure differs")
        if record.completed_frame_identity in completed_identities:
            raise ValueError("vision completed-frame identity is duplicated")
        completed_identities.add(record.completed_frame_identity)
        previous = previous_by_camera.get(record.camera_id)
        if (
            previous is not None
            and previous.payload_sha256 == record.payload_sha256
            and previous.source_state_digest != record.source_state_digest
        ):
            raise ValueError("vision payload repeated across distinct source states")
        previous_by_camera[record.camera_id] = record
        grouped[record.dataset_frame_index].append(record)
        relative = _safe_relative(record.payload_path, field="payload_path")
        payload = _safe_payload(root, relative)
        if not payload.is_file():
            raise ValueError(f"vision payload is missing or a symlink: {relative}")
        if _sha256(payload) != record.payload_sha256:
            raise ValueError(f"vision payload checksum differs: {relative}")
        validate_rgb8_png(payload.read_bytes(), field=record.payload_path)
    if tuple(sorted(grouped)) != tuple(range(frame_count)):
        raise ValueError("vision frame indices must be contiguous from zero")
    for index, group in grouped.items():
        ordered = sorted(group, key=lambda item: CAMERA_IDS.index(item.camera_id))
        if tuple(item.camera_id for item in ordered) != CAMERA_IDS:
            raise ValueError(f"vision frame {index} camera set differs")
        common = {
            (
                item.source_control_index,
                item.source_tick_id,
                item.simulation_time_s,
                item.source_state_digest,
            )
            for item in ordered
        }
        if len(common) != 1:
            raise ValueError(f"vision frame {index} cameras do not share one source state")
    expected_material = {
        "manifest.json",
        "frame_index.jsonl",
        *(record.payload_path for record in records),
    }
    if set(checksums) != expected_material:
        raise ValueError("vision checksum inventory differs from manifest and frame index")

    return VisionArtifact(
        root=root,
        run_id=run_id,
        alignment_digest_sha256=alignment_digest,
        frame_count=frame_count,
        renderer_identity=renderer,
        provenance=provenance,
        camera_runtime_inventories=camera_runtime_inventories,
        frames=tuple(
            sorted(
                records,
                key=lambda item: (
                    item.dataset_frame_index,
                    CAMERA_IDS.index(item.camera_id),
                ),
            )
        ),
    )


__all__ = [
    "CAMERA_IDS",
    "DATASET_COLOR_SPACE",
    "DATASET_LIGHTING_IDENTITY",
    "DATASET_RENDERER_BACKEND",
    "VISION_ARTIFACT_SCHEMA",
    "VISION_FRAME_SCHEMA",
    "VISION_PROVENANCE_SCHEMA",
    "DatasetVisionProvenance",
    "VisionArtifact",
    "VisionArtifactBuilder",
    "VisionFrameRecord",
    "load_vision_artifact",
    "validate_rgb8_png",
]
