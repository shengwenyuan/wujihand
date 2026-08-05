"""Completed-frame capture for the dual synthetic D405 wrist cameras.

This adapter is intentionally simulation-only.  It wraps the Camera prims
authored by :mod:`isaac_d405_wrist_rig`, joins Replicator's rational completed-
frame time to a bounded USD pose history, and returns transport-neutral frames.
It contains no physical RealSense discovery or calibration path.
"""

from __future__ import annotations

from collections import OrderedDict, deque
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
from threading import Condition, Thread
import time
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray

from wujihand.adapters.simulation.isaac_camera import (
    IsaacCameraApiReadback,
    PinholeCalibration,
    assert_profile_matches_readback,
    depth_to_32fc1,
    derive_pinhole_calibration,
    rgba_to_rgb8,
)
from wujihand.integrity import sha256_file
from wujihand.specs import IsaacCameraProfile

from .isaac_d405_wrist_rig import (
    D405WristRigHandles,
    D405WristRigRuntime,
    RigidTransform,
)


SIMULATION_CAMERA_FRAME_SCHEMA = "wujihand.simulation_camera_frame_truth.v2"
SIMULATION_CAMERA_CAPTURE_ADAPTER = "isaac_d405_capture_phase_pose_history_v5"
SIMULATION_TIME_STAMP_RULE = "capture_phase_rational_grid_round_half_up_to_nanoseconds_v2"
POSE_HISTORY_JOIN_TOLERANCE_NS = 1_000
WORLD_FRAME_ID = "world"
_SIDES = ("left", "right")
_POSE_HISTORY_CAPACITY = 64
_WRITER_QUEUE_CAPACITY = 16
_USD_CAMERA_FROM_ROS_OPTICAL = np.diag((1.0, -1.0, -1.0)).astype(np.float64)


Matrix4 = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]


def _is_capture_phase(
    profile: IsaacCameraProfile,
    *,
    control_tick_id: int,
    physics_substep_ordinal: int,
) -> bool:
    return (
        control_tick_id + 1
    ) % profile.schedule.control_ticks_per_capture == 0 and physics_substep_ordinal == 1


@dataclass(frozen=True, slots=True)
class SimulationCameraStaticInventory:
    """Manifest-time facts read from one active RTX render product."""

    side: str
    camera_prim_path: str
    render_product_path: str
    world_frame_id: str
    hand_base_frame_id: str
    optical_frame_id: str
    profile_path: str
    profile_sha256: str
    profile: IsaacCameraProfile
    readback: IsaacCameraApiReadback
    calibration: PinholeCalibration
    hand_base_from_camera_optical: Matrix4
    mount_visual_sha256: str
    camera_visual_sha256: str
    generation_report_sha256: str

    def to_mapping(self) -> dict[str, object]:
        profile = self.profile
        return {
            "side": self.side,
            "simulation_only": True,
            "warning": profile.warning,
            "projection_classification": profile.projection_classification,
            "camera_prim_path": self.camera_prim_path,
            "render_product_path": self.render_product_path,
            "parent_frame_id": self.hand_base_frame_id,
            "world_frame_id": self.world_frame_id,
            "optical_frame_id": self.optical_frame_id,
            "profile_path": self.profile_path,
            "profile_sha256": self.profile_sha256,
            "profile": profile.to_mapping(),
            "api_readback": {
                "width_px": self.readback.width_px,
                "height_px": self.readback.height_px,
                "projection": self.readback.projection,
                "focal_length_mm": self.readback.focal_length_mm,
                "horizontal_aperture_mm": self.readback.horizontal_aperture_mm,
                "vertical_aperture_mm": self.readback.vertical_aperture_mm,
                "horizontal_aperture_offset_mm": (self.readback.horizontal_aperture_offset_mm),
                "vertical_aperture_offset_mm": (self.readback.vertical_aperture_offset_mm),
                "clipping_range_m": list(self.readback.clipping_range_m),
            },
            "derived_calibration": {
                "horizontal_fov_deg": self.calibration.horizontal_fov_deg,
                "vertical_fov_deg": self.calibration.vertical_fov_deg,
                "k_row_major": list(self.calibration.k_row_major),
                "d": list(profile.optics.distortion_coefficients),
                "r_row_major": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                "p_row_major": list(self.calibration.p_row_major),
                "distortion_model": profile.optics.distortion_model,
                "distortion_source": "simulator_authored_zero",
                "pixel_geometry_convention": (profile.optics.pixel_geometry_convention),
            },
            "hand_base_from_camera_optical_row_major": _flatten_matrix(
                self.hand_base_from_camera_optical
            ),
            "payload_conversion": {
                "converter": "wujihand.adapters.simulation.isaac_camera.v1",
                "rgb": profile.rgb.to_mapping(),
                "depth": profile.depth.to_mapping(),
                "shared_completed_capture": True,
            },
            "source_hashes": {
                "mount_visual_sha256": self.mount_visual_sha256,
                "camera_visual_sha256": self.camera_visual_sha256,
                "generation_report_sha256": self.generation_report_sha256,
            },
        }


@dataclass(frozen=True, slots=True)
class SimulationCameraFrame:
    """One synchronized RGB/depth/truth transaction ready for ROS conversion."""

    run_id: str
    side: str
    camera_frame_index: int
    stamp_ns: int
    optical_frame_id: str
    hand_base_frame_id: str
    control_tick_id: int
    physics_substep_index: int
    capture_sim_time_s: float
    host_capture_start_ns: int
    host_capture_end_ns: int
    world_from_hand_base: Matrix4
    world_from_camera_optical: Matrix4
    hand_base_from_camera_optical: Matrix4
    reference_time_numerator: int
    reference_time_denominator: int
    rgb: NDArray[np.uint8]
    depth: NDArray[np.float32]

    @property
    def completed_frame_identity(self) -> str:
        return f"{self.reference_time_numerator}/{self.reference_time_denominator}"


@dataclass(frozen=True, slots=True)
class _PoseSample:
    stamp_ns: int
    simulation_time_s: float
    control_tick_id: int
    physics_substep_index: int
    physics_substep_ordinal: int
    world_from_hand_base: Matrix4
    world_from_camera_optical: Matrix4


@dataclass(frozen=True, slots=True)
class _RawCompletedFrame:
    reference_time: tuple[int, int]
    callback_start_ns: int
    callback_end_ns: int
    rgba: NDArray[np.generic]
    depth: NDArray[np.generic]


@dataclass(frozen=True, slots=True)
class _PendingCompletedFrame:
    reference_time: tuple[int, int]
    callback_start_ns: int
    rgba: object
    depth: object


class _CompletedFrameSink:
    """Own GPU payloads quickly, then perform blocking host copies off-thread."""

    def __init__(
        self,
        profile: IsaacCameraProfile,
        *,
        clone_payload: Callable[[object], object],
        payload_to_numpy: Callable[[object], NDArray[np.generic]] = lambda value: _as_numpy(value),
    ) -> None:
        self._profile = profile
        self._clone_payload = clone_payload
        self._payload_to_numpy = payload_to_numpy
        self._condition = Condition()
        self._pending: deque[_PendingCompletedFrame] = deque()
        self._records: deque[_RawCompletedFrame] = deque()
        self._overflow_count = 0
        self._processing_count = 0
        self._failure: BaseException | None = None
        self._closing = False
        self._worker = Thread(
            target=self._run,
            name="synthetic-d405-host-copy",
            daemon=True,
        )
        self._worker.start()

    def write(self, data: Mapping[str, object]) -> None:
        started_ns = time.monotonic_ns()
        reference = _reference_time(data.get("reference_time"))
        rgba = self._clone_payload(data.get("rgb"))
        depth = self._clone_payload(data.get(self._profile.depth.annotator))
        pending = _PendingCompletedFrame(
            reference_time=reference,
            callback_start_ns=started_ns,
            rgba=rgba,
            depth=depth,
        )
        with self._condition:
            self._raise_if_failed_locked()
            if self._closing:
                raise RuntimeError("synthetic D405 writer callback arrived after close")
            queue_size = len(self._pending) + len(self._records) + self._processing_count
            if queue_size >= _WRITER_QUEUE_CAPACITY:
                self._overflow_count += 1
                return
            self._pending.append(pending)
            self._condition.notify_all()

    def _run(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: bool(self._pending) or self._closing)
                if not self._pending:
                    return
                pending = self._pending.popleft()
                self._processing_count += 1
            try:
                rgba = self._payload_to_numpy(pending.rgba)
                depth = self._payload_to_numpy(pending.depth)
                if rgba.ndim == 1:
                    rgba = rgba.reshape(self._profile.rgb.source_shape)
                if depth.ndim == 1:
                    depth = depth.reshape(self._profile.depth.source_shape)
                record = _RawCompletedFrame(
                    reference_time=pending.reference_time,
                    callback_start_ns=pending.callback_start_ns,
                    callback_end_ns=time.monotonic_ns(),
                    rgba=rgba,
                    depth=depth,
                )
            except BaseException as exc:
                with self._condition:
                    self._failure = exc
                    self._pending.clear()
                    self._processing_count -= 1
                    self._condition.notify_all()
                return
            with self._condition:
                self._records.append(record)
                self._processing_count -= 1
                self._condition.notify_all()

    def pop_all(
        self,
        *,
        wait_for_pending: bool = False,
        timeout_s: float = 5.0,
    ) -> tuple[_RawCompletedFrame, ...]:
        with self._condition:
            if wait_for_pending:
                idle = self._condition.wait_for(
                    lambda: (
                        (not self._pending and self._processing_count == 0)
                        or self._failure is not None
                    ),
                    timeout=timeout_s,
                )
                if not idle:
                    raise TimeoutError("synthetic D405 host-copy worker did not become idle")
            self._raise_if_failed_locked()
            if self._overflow_count:
                raise RuntimeError("synthetic D405 writer queue overflowed; refusing a gapped run")
            result = tuple(self._records)
            self._records.clear()
            return result

    def clear(self) -> None:
        self.pop_all(wait_for_pending=True)
        with self._condition:
            self._records.clear()
            self._overflow_count = 0

    def close(self, *, timeout_s: float = 10.0) -> None:
        with self._condition:
            self._closing = True
            self._condition.notify_all()
        self._worker.join(timeout=timeout_s)
        if self._worker.is_alive():
            raise TimeoutError("synthetic D405 host-copy worker did not stop")
        with self._condition:
            self._raise_if_failed_locked()
            if self._overflow_count:
                raise RuntimeError("synthetic D405 writer queue overflowed; refusing a gapped run")

    def _raise_if_failed_locked(self) -> None:
        if self._failure is not None:
            raise RuntimeError("synthetic D405 host-copy worker failed") from self._failure


@dataclass(slots=True)
class _CameraPipeline:
    runtime: D405WristRigRuntime
    handles: D405WristRigHandles
    hand_base_path: str
    rtx_camera: Any
    sensor: Any
    writer: Any
    sink: _CompletedFrameSink
    inventory: SimulationCameraStaticInventory


class DualD405CameraCapture:
    """Own both active render products only while ``record=true``."""

    def __init__(
        self,
        *,
        project_root: Path,
        scene: Any,
        run_id: str,
    ) -> None:
        if not run_id or run_id != run_id.strip():
            raise ValueError("camera capture run_id must be non-blank")
        if len(scene.wrist_rig_runtimes) != 2 or len(scene.wrist_rigs) != 2:
            raise RuntimeError("recorded D405 capture requires two materialized wrist rigs")

        import carb  # type: ignore[import-not-found]
        import omni.replicator.core as rep  # type: ignore[import-not-found]
        import warp as wp  # type: ignore[import-not-found]
        from isaacsim.sensors.experimental.rtx import (  # type: ignore[import-not-found]
            CameraSensor,
            RtxCamera,
        )

        carb.settings.get_settings().set("/rtx/hydra/supportMultiTickRate", True)
        self._project_root = project_root.resolve()
        self._scene = scene
        self._run_id = run_id
        self._pipelines: dict[str, _CameraPipeline] = {}
        self._history: dict[str, OrderedDict[int, _PoseSample]] = {
            side: OrderedDict() for side in _SIDES
        }
        self._frame_indices = {side: 0 for side in _SIDES}
        self._last_reference: dict[str, tuple[int, int] | None] = {side: None for side in _SIDES}
        self._first_stamp_ns: dict[str, int | None] = {side: None for side in _SIDES}
        self._last_stamp_ns: dict[str, int | None] = {side: None for side in _SIDES}
        self._warmup_frames = {side: 0 for side in _SIDES}
        self._post_activation_stale_frames = {side: 0 for side in _SIDES}
        self._activation_stamp_ns: int | None = None
        self._warmup_updates = 0
        self._shutdown_drain_app_updates = 0
        self._in_flight_drain_count = 0
        self._replay_priming_updates = 0
        self._replay_priming_discarded_frames = {side: 0 for side in _SIDES}
        self._closed = False

        runtimes = {runtime.side: runtime for runtime in scene.wrist_rig_runtimes}
        handles = {item.side: item for item in scene.wrist_rigs}
        try:
            for side in _SIDES:
                runtime = runtimes[side]
                handle = handles[side]
                camera_prim = scene.stage.GetPrimAtPath(handle.camera_prim_path)
                if not camera_prim.IsValid() or camera_prim.GetTypeName() != "Camera":
                    raise RuntimeError(
                        f"synthetic D405 Camera prim is invalid: {handle.camera_prim_path}"
                    )
                # RtxCamera only validates existing prims.  Apply its sensor API
                # explicitly in record=true mode; record=false never reaches here.
                if not camera_prim.ApplyAPI("OmniSensorAPI"):
                    raise RuntimeError("failed to apply OmniSensorAPI to D405 Camera prim")
                from pxr import UsdGeom  # type: ignore[import-not-found]

                authored_local_transform = UsdGeom.Xformable(camera_prim).GetLocalTransformation()
                rtx_camera = RtxCamera(
                    handle.camera_prim_path,
                    tick_rate=runtime.camera_profile.capture.rate_hz,
                    reset_xform_op_properties=False,
                )
                sensor = CameraSensor(
                    rtx_camera,
                    resolution=(
                        runtime.camera_profile.capture.height_px,
                        runtime.camera_profile.capture.width_px,
                    ),
                    annotators=None,
                )
                profile = runtime.camera_profile
                # CameraSensor constructs an experimental Camera wrapper that
                # normalizes the existing xform and lens fields.  Restore the
                # authoritative mounted USD contract before the first capture.
                _restore_camera_prim_contract(
                    camera_prim=camera_prim,
                    authored_local_transform=authored_local_transform,
                    profile=profile,
                )

                def clone_payload(value: object) -> object:
                    payload = _annotator_payload(value)
                    if isinstance(payload, np.ndarray):
                        return payload.copy()
                    try:
                        return wp.clone(payload)
                    except (AttributeError, TypeError) as exc:
                        raise RuntimeError(
                            "synthetic D405 annotator did not return an owned-copyable payload"
                        ) from exc

                sink = _CompletedFrameSink(
                    runtime.camera_profile,
                    clone_payload=clone_payload,
                )
                rgb_annotator = rep.AnnotatorRegistry.get_annotator(
                    "rgb",
                    device="cuda",
                    do_array_copy=False,
                )
                depth_annotator = rep.AnnotatorRegistry.get_annotator(
                    profile.depth.annotator,
                    device="cuda",
                    do_array_copy=False,
                )

                class SynchronizedCameraWriter(rep.Writer):  # type: ignore[misc]
                    """Bundle RGB, depth and rational reference time once."""

                    def __init__(self, target: _CompletedFrameSink) -> None:
                        self.version = "1.0.0"
                        self.annotators = [
                            rgb_annotator,
                            depth_annotator,
                        ]
                        self._target = target

                    def write(self, data: dict[str, Any]) -> None:
                        self._target.write(data)

                    def write_metadata(self) -> None:
                        pass

                writer = SynchronizedCameraWriter(sink)
                render_product_path = str(sensor.render_product.GetPath())
                writer.attach(render_product_path)
                hand_base_path = scene.authored[side].config.child_base_link_path
                inventory = _static_inventory(
                    project_root=self._project_root,
                    side=side,
                    runtime=runtime,
                    handles=handle,
                    stage=self._scene.stage,
                    sensor=sensor,
                )
                self._pipelines[side] = _CameraPipeline(
                    runtime=runtime,
                    handles=handle,
                    hand_base_path=hand_base_path,
                    rtx_camera=rtx_camera,
                    sensor=sensor,
                    writer=writer,
                    sink=sink,
                    inventory=inventory,
                )
        except BaseException:
            self.close()
            raise

    @property
    def inventories(self) -> tuple[SimulationCameraStaticInventory, ...]:
        return tuple(self._pipelines[side].inventory for side in _SIDES)

    @property
    def active(self) -> bool:
        return self._activation_stamp_ns is not None and not self._closed

    @property
    def capture_counts(self) -> dict[str, int]:
        return dict(self._frame_indices)

    def warm_up(
        self,
        *,
        update_app: Callable[[], None],
        simulation_time_s: Callable[[], float],
    ) -> dict[str, object]:
        """Discard configured warm-up frames and stop on a sensor tick boundary."""

        if self._activation_stamp_ns is not None:
            raise RuntimeError("camera capture cannot warm up after activation")
        maximum_updates = (
            max(
                pipeline.runtime.camera_profile.capture.warmup_frames
                for pipeline in self._pipelines.values()
            )
            * 16
        )
        latest: dict[str, tuple[int, int] | None] = {side: None for side in _SIDES}
        final_simulation_stamp_ns = simulation_seconds_to_stamp_ns(simulation_time_s())
        for update_index in range(1, maximum_updates + 1):
            update_app()
            for side, pipeline in self._pipelines.items():
                records = pipeline.sink.pop_all(wait_for_pending=True)
                self._warmup_frames[side] += len(records)
                if records:
                    latest[side] = records[-1].reference_time
            ready = all(
                self._warmup_frames[side]
                >= self._pipelines[side].runtime.camera_profile.capture.warmup_frames
                for side in _SIDES
            )
            if not ready or latest["left"] != latest["right"] or latest["left"] is None:
                continue
            current_stamp_ns = simulation_seconds_to_stamp_ns(simulation_time_s())
            final_simulation_stamp_ns = current_stamp_ns
            reference_stamp_ns = reference_time_to_stamp_ns(latest["left"])
            if reference_stamp_ns > current_stamp_ns:
                raise RuntimeError("D405 completed-frame reference is ahead of simulation time")
            self._warmup_updates = update_index
            for pipeline in self._pipelines.values():
                pipeline.sink.clear()
            return {
                "discarded_frames": dict(self._warmup_frames),
                "app_updates": update_index,
                "last_reference_time": list(latest["left"]),
                "simulation_stamp_ns": current_stamp_ns,
                "frames_in_flight_lag_ns": current_stamp_ns - reference_stamp_ns,
                "completed_before_update_return": True,
            }
        raise RuntimeError(
            "dual D405 warm-up did not finish on a shared completed-frame boundary: "
            f"discarded={self._warmup_frames!r}, latest={latest!r}, "
            f"simulation_stamp_ns={final_simulation_stamp_ns}"
        )

    def activate(self, *, simulation_time_s: float) -> None:
        """Reset public indices immediately before the recorded control loop."""

        if self._closed:
            raise RuntimeError("camera capture is closed")
        if self._warmup_updates == 0:
            raise RuntimeError("camera capture must complete warm-up before activation")
        if self._activation_stamp_ns is not None:
            raise RuntimeError("camera capture is already active")
        self._activation_stamp_ns = simulation_seconds_to_stamp_ns(simulation_time_s)
        for pipeline in self._pipelines.values():
            pipeline.sink.clear()

    def prime_after_timeline_reset(
        self,
        *,
        render_update: Callable[[], None],
        simulation_time_s: float,
    ) -> dict[str, object]:
        """Discard one render after replay reset, before scheduled capture starts."""

        if not self.active:
            raise RuntimeError("camera capture must be active before replay priming")
        if self._replay_priming_updates != 0:
            raise RuntimeError("camera replay priming may run only once")
        if any(self._frame_indices.values()):
            raise RuntimeError("camera replay priming must precede public frames")
        expected_stamp_ns = simulation_seconds_to_stamp_ns(simulation_time_s)
        for pipeline in self._pipelines.values():
            pipeline.sink.clear()
        render_update()
        references: dict[str, tuple[tuple[int, int], ...]] = {}
        for side, pipeline in self._pipelines.items():
            records = pipeline.sink.pop_all(wait_for_pending=True)
            references[side] = tuple(record.reference_time for record in records)
            self._replay_priming_discarded_frames[side] = len(records)
            for record in records:
                if reference_time_to_stamp_ns(record.reference_time) != expected_stamp_ns:
                    raise RuntimeError(
                        "D405 replay priming returned an unexpected simulation stamp: "
                        f"side={side!r}, reference={record.reference_time!r}, "
                        f"expected_stamp_ns={expected_stamp_ns}"
                    )
        self._replay_priming_updates = 1
        return {
            "render_updates": self._replay_priming_updates,
            "discarded_frames": dict(self._replay_priming_discarded_frames),
            "references": references,
            "simulation_stamp_ns": expected_stamp_ns,
        }

    def observe_completed_substep(
        self,
        *,
        control_tick_id: int,
        physics_substep_index: int,
        physics_substep_ordinal: int,
        simulation_time_s: float,
    ) -> tuple[SimulationCameraFrame, ...]:
        """Save exact capture-phase poses, then join callbacks completed by this step."""

        if not self.active:
            raise RuntimeError("camera capture is not active")
        if physics_substep_ordinal not in (0, 1):
            raise ValueError("physics_substep_ordinal must be zero or one")
        for side, pipeline in self._pipelines.items():
            profile = pipeline.runtime.camera_profile
            if not _is_capture_phase(
                profile,
                control_tick_id=control_tick_id,
                physics_substep_ordinal=physics_substep_ordinal,
            ):
                continue
            assert self._activation_stamp_ns is not None
            stamp_ns = scheduled_capture_stamp_ns(
                activation_stamp_ns=self._activation_stamp_ns,
                capture_rate_hz=profile.capture.rate_hz,
                control_tick_id=control_tick_id,
            )
            sample = _pose_sample(
                self._scene.stage,
                pipeline=pipeline,
                stamp_ns=stamp_ns,
                simulation_time_s=simulation_time_s,
                control_tick_id=control_tick_id,
                physics_substep_index=physics_substep_index,
                physics_substep_ordinal=physics_substep_ordinal,
            )
            history = self._history[side]
            history[stamp_ns] = sample
            while len(history) > _POSE_HISTORY_CAPACITY:
                history.popitem(last=False)
        return self._drain_available()

    def _drain_available(
        self,
        *,
        wait_for_pending: bool = False,
    ) -> tuple[SimulationCameraFrame, ...]:
        frames: list[SimulationCameraFrame] = []
        assert self._activation_stamp_ns is not None
        for side, pipeline in self._pipelines.items():
            profile = pipeline.runtime.camera_profile
            for raw in pipeline.sink.pop_all(wait_for_pending=wait_for_pending):
                reference_stamp_ns = reference_time_to_stamp_ns(raw.reference_time)
                if reference_stamp_ns <= self._activation_stamp_ns:
                    self._post_activation_stale_frames[side] += 1
                    continue
                sample = _nearest_pose_sample(
                    self._history[side],
                    reference_stamp_ns=reference_stamp_ns,
                )
                if sample is None:
                    raise RuntimeError(
                        "D405 completed-frame reference_time has no bounded pose sample: "
                        f"side={side!r}, reference={raw.reference_time!r}, "
                        f"stamp_ns={reference_stamp_ns}, "
                        f"history={tuple(self._history[side])!r}"
                    )
                if not _is_capture_phase(
                    profile,
                    control_tick_id=sample.control_tick_id,
                    physics_substep_ordinal=sample.physics_substep_ordinal,
                ):
                    raise RuntimeError(
                        "D405 capture drifted from the completed fourth physics substep"
                    )
                previous = self._last_reference[side]
                if previous is not None and _reference_fraction(raw.reference_time) <= (
                    _reference_fraction(previous)
                ):
                    raise RuntimeError("D405 completed-frame identity is not monotonic")
                rgb = rgba_to_rgb8(raw.rgba, profile)
                depth = depth_to_32fc1(raw.depth, profile)
                frame_index = self._frame_indices[side]
                frame = SimulationCameraFrame(
                    run_id=self._run_id,
                    side=side,
                    camera_frame_index=frame_index,
                    stamp_ns=sample.stamp_ns,
                    optical_frame_id=pipeline.inventory.optical_frame_id,
                    hand_base_frame_id=pipeline.inventory.hand_base_frame_id,
                    control_tick_id=sample.control_tick_id,
                    physics_substep_index=sample.physics_substep_index,
                    capture_sim_time_s=_reference_fraction(raw.reference_time),
                    host_capture_start_ns=raw.callback_start_ns,
                    host_capture_end_ns=raw.callback_end_ns,
                    world_from_hand_base=sample.world_from_hand_base,
                    world_from_camera_optical=sample.world_from_camera_optical,
                    hand_base_from_camera_optical=(
                        pipeline.inventory.hand_base_from_camera_optical
                    ),
                    reference_time_numerator=raw.reference_time[0],
                    reference_time_denominator=raw.reference_time[1],
                    rgb=rgb,
                    depth=depth,
                )
                _validate_frame_closure(frame)
                frames.append(frame)
                self._frame_indices[side] += 1
                self._last_reference[side] = raw.reference_time
                if self._first_stamp_ns[side] is None:
                    self._first_stamp_ns[side] = sample.stamp_ns
                self._last_stamp_ns[side] = sample.stamp_ns
        return tuple(sorted(frames, key=lambda item: (item.stamp_ns, item.side)))

    def drain_completed(self) -> tuple[SimulationCameraFrame, ...]:
        """Drain callbacks already completed without advancing the timeline."""

        if not self.active:
            return ()
        return self._drain_available()

    def stop_and_drain(
        self,
        *,
        update_app: Callable[[], None],
        expected_frames_per_side: int,
        maximum_updates: int = 64,
    ) -> tuple[SimulationCameraFrame, ...]:
        """Drain bounded RTX work after the caller pauses simulation time."""

        if not self.active:
            return ()
        if expected_frames_per_side < 0:
            raise ValueError("expected camera frame count must be non-negative")
        if maximum_updates <= 0:
            raise ValueError("maximum shutdown drain updates must be positive")
        frames = list(self._drain_available(wait_for_pending=True))
        for update_index in range(maximum_updates + 1):
            counts = tuple(self._frame_indices[side] for side in _SIDES)
            if any(count > expected_frames_per_side for count in counts):
                raise RuntimeError("D405 shutdown drain exceeded the 30 Hz schedule")
            if all(count == expected_frames_per_side for count in counts):
                self._shutdown_drain_app_updates = update_index
                self._in_flight_drain_count = len(frames)
                return tuple(sorted(frames, key=lambda item: (item.stamp_ns, item.side)))
            if update_index == maximum_updates:
                break
            update_app()
            frames.extend(self._drain_available(wait_for_pending=True))
        raise RuntimeError(
            "D405 in-flight drain did not reach the 30 Hz schedule: "
            f"expected={expected_frames_per_side}, actual={self._frame_indices!r}"
        )

    def receipt(self, *, publish_counts: Mapping[str, int]) -> dict[str, object]:
        """Return capture facts using caller-owned ROS publish counters."""

        if set(publish_counts) != set(_SIDES):
            raise ValueError("D405 publish counts must cover exactly left and right")
        for side in _SIDES:
            published = publish_counts[side]
            if isinstance(published, bool) or not isinstance(published, int) or published < 0:
                raise ValueError(f"{side} D405 publish count must be a non-negative integer")
            if published > self._frame_indices[side]:
                raise ValueError(f"{side} D405 publish count exceeds captured frames")
        return {
            "adapter": SIMULATION_CAMERA_CAPTURE_ADAPTER,
            "stamp_rule": SIMULATION_TIME_STAMP_RULE,
            "warmup_frames_discarded": dict(self._warmup_frames),
            "post_activation_stale_frames_discarded": dict(self._post_activation_stale_frames),
            "warmup_app_updates": self._warmup_updates,
            "replay_priming": {
                "render_updates": self._replay_priming_updates,
                "discarded_frames": dict(self._replay_priming_discarded_frames),
            },
            "sides": {
                side: {
                    "capture_count": self._frame_indices[side],
                    "publish_count": publish_counts[side],
                    "first_stamp_ns": self._first_stamp_ns[side],
                    "last_stamp_ns": self._last_stamp_ns[side],
                    "last_camera_frame_index": (
                        None if self._frame_indices[side] == 0 else self._frame_indices[side] - 1
                    ),
                }
                for side in _SIDES
            },
            "in_flight_drain_count": self._in_flight_drain_count,
            "shutdown_drain_app_updates": self._shutdown_drain_app_updates,
            "closed": self._closed,
        }

    def close(self) -> None:
        if self._closed:
            return
        errors: list[BaseException] = []
        for pipeline in self._pipelines.values():
            try:
                pipeline.writer.detach()
            except BaseException as exc:  # preserve all detach attempts
                errors.append(exc)
        for pipeline in self._pipelines.values():
            try:
                pipeline.sink.close()
            except BaseException as exc:  # preserve all worker-stop attempts
                errors.append(exc)
        self._closed = True
        if errors:
            raise RuntimeError(
                f"failed to close {len(errors)} synthetic D405 capture component(s)"
            ) from errors[0]


def simulation_seconds_to_stamp_ns(value: float) -> int:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("simulation time must be finite and non-negative")
    return int(math.floor(value * 1_000_000_000.0 + 0.5))


def reference_time_to_stamp_ns(reference: tuple[int, int]) -> int:
    numerator, denominator = reference
    if numerator < 0 or denominator <= 0:
        raise ValueError("completed-frame reference_time must be non-negative")
    scaled = numerator * 1_000_000_000
    quotient, remainder = divmod(scaled, denominator)
    return quotient + int(remainder * 2 >= denominator)


def scheduled_capture_stamp_ns(
    *,
    activation_stamp_ns: int,
    capture_rate_hz: float,
    control_tick_id: int,
) -> int:
    """Map a completed 30 Hz control phase onto an exact rational time grid."""

    if activation_stamp_ns < 0:
        raise ValueError("activation stamp must be non-negative")
    if not math.isfinite(capture_rate_hz) or not capture_rate_hz.is_integer():
        raise ValueError("capture rate must be a positive integer-valued frequency")
    rate_hz = int(capture_rate_hz)
    if rate_hz <= 0:
        raise ValueError("capture rate must be a positive integer-valued frequency")
    if control_tick_id < 0 or control_tick_id % 2 != 1:
        raise ValueError("D405 capture control tick must be a non-negative odd index")
    activation_grid_index = (activation_stamp_ns * rate_hz + 500_000_000) // 1_000_000_000
    capture_ordinal = (control_tick_id + 1) // 2
    scaled = (activation_grid_index + capture_ordinal) * 1_000_000_000
    quotient, remainder = divmod(scaled, rate_hz)
    return quotient + int(remainder * 2 >= rate_hz)


def nearest_pose_stamp_ns(
    history_stamps_ns: Sequence[int],
    *,
    reference_stamp_ns: int,
    tolerance_ns: int = POSE_HISTORY_JOIN_TOLERANCE_NS,
) -> int | None:
    """Select a deterministic pose stamp within the frozen rounding tolerance."""

    if reference_stamp_ns < 0:
        raise ValueError("reference pose stamp must be non-negative")
    if tolerance_ns < 0:
        raise ValueError("pose join tolerance must be non-negative")
    if not history_stamps_ns:
        return None
    nearest = min(
        history_stamps_ns,
        key=lambda stamp_ns: (abs(stamp_ns - reference_stamp_ns), stamp_ns),
    )
    if abs(nearest - reference_stamp_ns) > tolerance_ns:
        return None
    return nearest


def _nearest_pose_sample(
    history: Mapping[int, _PoseSample],
    *,
    reference_stamp_ns: int,
) -> _PoseSample | None:
    stamp_ns = nearest_pose_stamp_ns(
        tuple(history),
        reference_stamp_ns=reference_stamp_ns,
    )
    if stamp_ns is None:
        return None
    return history[stamp_ns]


def _reference_fraction(reference: tuple[int, int]) -> float:
    return reference[0] / reference[1]


def _reference_time(value: object) -> tuple[int, int]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 2:
        raise RuntimeError("writer record has no rational completed-frame identity")
    numerator, denominator = value
    if (
        isinstance(numerator, bool)
        or isinstance(denominator, bool)
        or not isinstance(numerator, (int, np.integer))
        or not isinstance(denominator, (int, np.integer))
    ):
        raise RuntimeError("writer reference_time values must be integers")
    result = (int(numerator), int(denominator))
    reference_time_to_stamp_ns(result)
    return result


def _as_numpy(value: object) -> NDArray[np.generic]:
    payload = _annotator_payload(value)
    if hasattr(payload, "numpy"):
        payload = payload.numpy()
    return np.asarray(payload).copy()


def _annotator_payload(value: object) -> object:
    if value is None:
        raise RuntimeError("writer record is missing an annotator payload")
    if isinstance(value, Mapping) and "data" in value:
        return value["data"]
    return value


def _restore_camera_prim_contract(
    *,
    camera_prim: Any,
    authored_local_transform: Any,
    profile: IsaacCameraProfile,
) -> None:
    """Restore the mounted USD contract after CameraSensor wrapper creation."""

    from pxr import Gf, UsdGeom

    xformable = UsdGeom.Xformable(camera_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(authored_local_transform)
    camera = UsdGeom.Camera(camera_prim)
    optics = profile.optics
    camera.CreateProjectionAttr(optics.projection)
    camera.CreateFocalLengthAttr(optics.focal_length_mm)
    camera.CreateHorizontalApertureAttr(optics.horizontal_aperture_mm)
    camera.CreateVerticalApertureAttr(optics.vertical_aperture_mm)
    camera.CreateHorizontalApertureOffsetAttr(optics.horizontal_aperture_offset_mm)
    camera.CreateVerticalApertureOffsetAttr(optics.vertical_aperture_offset_mm)
    camera.CreateClippingRangeAttr(Gf.Vec2f(*optics.clipping_range_m))


def _static_inventory(
    *,
    project_root: Path,
    side: str,
    runtime: D405WristRigRuntime,
    handles: D405WristRigHandles,
    stage: Any,
    sensor: Any,
) -> SimulationCameraStaticInventory:
    from pxr import UsdGeom

    camera = UsdGeom.Camera(stage.GetPrimAtPath(handles.camera_prim_path))
    clipping_range = camera.GetClippingRangeAttr().Get()
    width_px, height_px = (int(value) for value in sensor.render_product.GetResolutionAttr().Get())
    readback = IsaacCameraApiReadback(
        width_px=width_px,
        height_px=height_px,
        projection=str(camera.GetProjectionAttr().Get()),
        focal_length_mm=float(camera.GetFocalLengthAttr().Get()),
        horizontal_aperture_mm=float(camera.GetHorizontalApertureAttr().Get()),
        vertical_aperture_mm=float(camera.GetVerticalApertureAttr().Get()),
        horizontal_aperture_offset_mm=float(camera.GetHorizontalApertureOffsetAttr().Get()),
        vertical_aperture_offset_mm=float(camera.GetVerticalApertureOffsetAttr().Get()),
        clipping_range_m=(float(clipping_range[0]), float(clipping_range[1])),
    )
    assert_profile_matches_readback(
        runtime.camera_profile,
        readback,
        absolute_tolerance=1e-4,
    )
    relative_profile = runtime.camera_profile_path.resolve().relative_to(project_root)
    return SimulationCameraStaticInventory(
        side=side,
        camera_prim_path=handles.camera_prim_path,
        render_product_path=str(sensor.render_product.GetPath()),
        world_frame_id=WORLD_FRAME_ID,
        hand_base_frame_id=f"wujihand_{side}_hand_base",
        optical_frame_id=f"wujihand_{side}_wrist_camera_optical",
        profile_path=relative_profile.as_posix(),
        profile_sha256=sha256_file(runtime.camera_profile_path),
        profile=runtime.camera_profile,
        readback=readback,
        calibration=derive_pinhole_calibration(readback),
        hand_base_from_camera_optical=_transform_matrix(runtime.optical_in_hand),
        mount_visual_sha256=runtime.mount_visual_sha256,
        camera_visual_sha256=runtime.camera_visual_sha256,
        generation_report_sha256=runtime.generation_report_sha256,
    )


def _pose_sample(
    stage: Any,
    *,
    pipeline: _CameraPipeline,
    stamp_ns: int,
    simulation_time_s: float,
    control_tick_id: int,
    physics_substep_index: int,
    physics_substep_ordinal: int,
) -> _PoseSample:
    hand = _prim_world_matrix(stage, pipeline.hand_base_path)
    static = _matrix_array(pipeline.inventory.hand_base_from_camera_optical)
    optical = hand @ static
    authored_camera = _prim_world_matrix(stage, pipeline.handles.camera_prim_path)
    authored_optical = authored_camera.copy()
    authored_optical[:3, :3] = authored_camera[:3, :3] @ _USD_CAMERA_FROM_ROS_OPTICAL
    if not np.allclose(authored_optical, optical, rtol=0.0, atol=1e-7):
        maximum_error = float(np.max(np.abs(authored_optical - optical)))
        raise RuntimeError(
            "authored USD Camera and ROS optical transforms diverged: "
            f"maximum_error={maximum_error!r}, "
            f"expected={optical.tolist()!r}, authored={authored_optical.tolist()!r}"
        )
    return _PoseSample(
        stamp_ns=stamp_ns,
        simulation_time_s=simulation_time_s,
        control_tick_id=control_tick_id,
        physics_substep_index=physics_substep_index,
        physics_substep_ordinal=physics_substep_ordinal,
        world_from_hand_base=_matrix_tuple(hand),
        world_from_camera_optical=_matrix_tuple(optical),
    )


def _prim_world_matrix(stage: Any, path: str) -> NDArray[np.float64]:
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"camera pose prim disappeared: {path}")
    transform = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    translation = transform.ExtractTranslation()
    quaternion = transform.ExtractRotationQuat()
    imaginary = quaternion.GetImaginary()
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = _rotation_matrix(
        (
            float(quaternion.GetReal()),
            float(imaginary[0]),
            float(imaginary[1]),
            float(imaginary[2]),
        )
    )
    result[:3, 3] = tuple(float(translation[index]) for index in range(3))
    _validate_rigid_matrix(result)
    return result


def _transform_matrix(transform: RigidTransform) -> Matrix4:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(transform.rotation, dtype=np.float64)
    result[:3, 3] = np.asarray(transform.translation_m, dtype=np.float64)
    _validate_rigid_matrix(result)
    return _matrix_tuple(result)


def _rotation_matrix(
    quaternion_wxyz: tuple[float, float, float, float],
) -> NDArray[np.float64]:
    values = np.asarray(quaternion_wxyz, dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise RuntimeError("USD pose quaternion is not normalized")
    w, x, y, z = values / norm
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


def _validate_frame_closure(frame: SimulationCameraFrame) -> None:
    world_hand = _matrix_array(frame.world_from_hand_base)
    hand_optical = _matrix_array(frame.hand_base_from_camera_optical)
    world_optical = _matrix_array(frame.world_from_camera_optical)
    for value in (world_hand, hand_optical, world_optical):
        _validate_rigid_matrix(value)
        inverse = np.linalg.inv(value)
        if not np.allclose(value @ inverse, np.eye(4), rtol=0.0, atol=1e-9):
            raise RuntimeError("camera frame transform inverse closure failed")
    if not np.allclose(
        world_hand @ hand_optical,
        world_optical,
        rtol=0.0,
        atol=1e-8,
    ):
        raise RuntimeError("camera frame static/dynamic extrinsic closure failed")


def _validate_rigid_matrix(value: NDArray[np.float64]) -> None:
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise RuntimeError("camera transform must be finite 4x4")
    rotation = value[:3, :3]
    if (
        not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0.0, atol=1e-8)
        or not math.isclose(float(np.linalg.det(rotation)), 1.0, rel_tol=0.0, abs_tol=1e-8)
        or not np.array_equal(value[3], np.asarray((0.0, 0.0, 0.0, 1.0)))
    ):
        raise RuntimeError("camera transform must be a proper rigid transform")


def _matrix_tuple(value: NDArray[np.float64]) -> Matrix4:
    return cast(
        Matrix4,
        tuple(
            cast(tuple[float, float, float, float], tuple(float(item) for item in row))
            for row in value
        ),
    )


def _matrix_array(value: Matrix4) -> NDArray[np.float64]:
    return np.asarray(value, dtype=np.float64)


def _flatten_matrix(value: Matrix4) -> list[float]:
    return [item for row in value for item in row]


__all__ = [
    "SIMULATION_CAMERA_CAPTURE_ADAPTER",
    "SIMULATION_CAMERA_FRAME_SCHEMA",
    "SIMULATION_TIME_STAMP_RULE",
    "DualD405CameraCapture",
    "SimulationCameraFrame",
    "SimulationCameraStaticInventory",
    "reference_time_to_stamp_ns",
    "scheduled_capture_stamp_ns",
    "simulation_seconds_to_stamp_ns",
]
