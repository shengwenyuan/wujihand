"""Isaac backend for exact fixed-state, pre-action three-view RGB rendering.

The two wrist projections intentionally reuse the synthetic 140-degree profile
from the 007 mount work.  That projection is simulation-only: this module has
no physical D405 calibration fallback and exposes no runtime lens option.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from fractions import Fraction
import math
from numbers import Integral
from pathlib import Path
from threading import Condition
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray

from wujihand.adapters.simulation.isaac_camera import IsaacCameraApiReadback
from wujihand.dataset.camera import (
    DatasetCameraRuntimeInventory,
    DatasetRgbCameraProjection,
    assert_dataset_projection_matches_readback,
    load_dataset_camera_projections,
)
from wujihand.dataset.domain_randomization import (
    NOMINAL_VISUAL_DOMAIN_VARIANT,
    VisualDomainVariant,
)
from wujihand.dataset.profile import MiniDatasetProfile
from wujihand.dataset.rendering import CompletedRgbRender, encode_rgb8_png
from wujihand.domain.dataset_recording import SimulationStateFrame

from .isaac_dual_scene import (
    DualNeroHand2IsaacScene,
    workcell_frame_position,
)


SCENE_CAMERA_PRIM_PATH = "/World/DatasetCameras/SceneD435iRgb"
SCENE_CAMERA_EYE_FRAME = "simulation_nominal_camera_oblique_eye"
SCENE_CAMERA_TARGET_FRAME = "simulation_nominal_camera_oblique_target"
ISAAC_DATASET_RENDERER_IDENTITY = "isaac-6.0.1-fixed-state-triview-rgb-v2"
_CAMERA_IDS = ("scene_rgb", "left_wrist_rgb", "right_wrist_rgb")
_USD_CAMERA_FROM_ROS_OPTICAL = np.diag((1.0, -1.0, -1.0)).astype(np.float64)
_SOURCE_TIME_TOLERANCE_S = 5e-6
_KINEMATIC_LINK_POSITION_LIMIT_M = 2e-5


@dataclass(frozen=True, slots=True)
class _RawRgbFrame:
    reference_time: tuple[int, int]
    rgba: NDArray[np.generic]


@dataclass(slots=True)
class _ReplayClock:
    """Map recorded 120 Hz truth onto an independent 30 Hz render timeline."""

    physics_hz: float
    policy_fps: float
    source_physics_grid_origin: int | None = None
    last_source_physics_boundary_index: int | None = None
    last_dataset_frame_index: int | None = None

    def observe(
        self,
        frame: SimulationStateFrame,
        *,
        dataset_frame_index: int,
    ) -> float:
        if type(dataset_frame_index) is not int or dataset_frame_index < 0:
            raise ValueError("dataset_frame_index must be non-negative")
        if self.last_dataset_frame_index is not None and (
            dataset_frame_index <= self.last_dataset_frame_index
        ):
            raise RuntimeError("replay dataset frame index is not strictly increasing")
        source_grid_index = round(frame.simulation_time_s * self.physics_hz)
        source_grid_time_s = source_grid_index / self.physics_hz
        if abs(source_grid_time_s - frame.simulation_time_s) > _SOURCE_TIME_TOLERANCE_S:
            raise RuntimeError("recorded state time differs from the 120 Hz source grid")
        if self.last_source_physics_boundary_index is not None and (
            frame.physics_boundary_index <= self.last_source_physics_boundary_index
        ):
            raise RuntimeError("source physics boundary index is not strictly increasing")
        source_origin = source_grid_index - frame.physics_boundary_index
        if self.source_physics_grid_origin is None:
            self.source_physics_grid_origin = source_origin
        elif source_origin != self.source_physics_grid_origin:
            raise RuntimeError("recorded state time and physics boundary origin differ")
        self.last_source_physics_boundary_index = frame.physics_boundary_index
        self.last_dataset_frame_index = dataset_frame_index
        return dataset_frame_index / self.policy_fps


class _RgbSink:
    """Own one completed RGB callback before the next render submission."""

    def __init__(self, projection: DatasetRgbCameraProjection) -> None:
        self._projection = projection
        self._condition = Condition()
        self._records: list[_RawRgbFrame] = []
        self._failure: BaseException | None = None
        self._closed = False

    def write(self, data: Mapping[str, object]) -> None:
        try:
            reference = _reference_time(data.get("reference_time"))
            rgba = _payload_numpy(data.get("rgb"))
            if rgba.ndim == 1:
                rgba = rgba.reshape(self._projection.source_shape)
            if rgba.dtype != np.uint8 or rgba.shape != self._projection.source_shape:
                raise RuntimeError("Isaac RGB callback differs from the pinned RGBA contract")
            record = _RawRgbFrame(reference_time=reference, rgba=rgba)
        except BaseException as exc:
            with self._condition:
                self._failure = exc
                self._condition.notify_all()
            return
        with self._condition:
            if self._closed:
                self._failure = RuntimeError("RGB callback arrived after renderer close")
            else:
                self._records.append(record)
            self._condition.notify_all()

    def clear(self) -> None:
        with self._condition:
            self._raise_if_failed()
            self._records.clear()

    def has_record(self) -> bool:
        with self._condition:
            self._raise_if_failed()
            return bool(self._records)

    def wait(self, timeout_s: float) -> None:
        with self._condition:
            self._condition.wait_for(
                lambda: bool(self._records) or self._failure is not None,
                timeout=timeout_s,
            )
            self._raise_if_failed()

    def wait_for_reference_after(
        self,
        reference_time: tuple[int, int] | None,
        timeout_s: float,
    ) -> None:
        with self._condition:
            completed = self._condition.wait_for(
                lambda: (
                    self._failure is not None
                    or any(
                        reference_time is None
                        or _reference_is_after(record.reference_time, reference_time)
                        for record in self._records
                    )
                ),
                timeout=timeout_s,
            )
            self._raise_if_failed()
            if not completed:
                raise TimeoutError("RGB callback reference did not advance")

    def pop_all(self) -> tuple[_RawRgbFrame, ...]:
        with self._condition:
            self._raise_if_failed()
            records = tuple(self._records)
            self._records.clear()
            return records

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._raise_if_failed()
            if self._records:
                raise RuntimeError("unconsumed RGB callbacks remain at renderer close")

    def _raise_if_failed(self) -> None:
        if self._failure is not None:
            raise RuntimeError("Isaac RGB writer callback failed") from self._failure


@dataclass(slots=True)
class _CameraPipeline:
    projection: DatasetRgbCameraProjection
    camera_prim_path: str
    render_product_path: str
    render_product: Any
    writer: Any
    sink: _RgbSink
    inventory: DatasetCameraRuntimeInventory


class IsaacFixedStateRgbBackend:
    """Render three RGB images at fixed render times without taking physics steps."""

    renderer_backend = "RayTracedLighting"
    lighting_identity = "session_workcell_authored_lighting"
    color_space = "isaac_rgb_annotator_srgb"
    motion_blur_enabled = False

    def __init__(
        self,
        *,
        project_root: Path,
        scene: DualNeroHand2IsaacScene,
        dataset_profile: MiniDatasetProfile,
        warmup_update_app: Callable[[], None],
        visual_domain_variant: VisualDomainVariant = NOMINAL_VISUAL_DOMAIN_VARIANT,
        visual_domain_variant_profile_sha256: str = "0" * 64,
        callback_timeout_s: float = 10.0,
    ) -> None:
        if not math.isfinite(callback_timeout_s) or callback_timeout_s <= 0.0:
            raise ValueError("callback_timeout_s must be positive and finite")
        import carb  # type: ignore[import-not-found]
        import omni.replicator.core as rep  # type: ignore[import-not-found]
        import omni.timeline  # type: ignore[import-not-found]
        from isaacsim.core.utils.viewports import (  # type: ignore[import-not-found]
            set_camera_view,
        )
        from pxr import UsdGeom, UsdRender  # type: ignore[import-not-found]

        root = project_root.resolve()
        if not root.is_dir():
            raise ValueError("project root must be a directory")
        if scene.resolved.session.dataset_profile is None:
            raise ValueError("fixed-state RGB backend requires a dataset Session")
        self._project_root = root
        self._scene = scene
        self._dataset_profile = dataset_profile
        self.visual_domain_variant = visual_domain_variant
        if len(visual_domain_variant_profile_sha256) != 64:
            raise ValueError("visual domain variant profile hash differs")
        self.visual_domain_variant_profile_sha256 = visual_domain_variant_profile_sha256
        self.renderer_identity = (
            f"{ISAAC_DATASET_RENDERER_IDENTITY}-{visual_domain_variant.variant_id}-"
            f"{visual_domain_variant.digest_sha256[:12]}"
        )
        self._render_update_app: Callable[[], None] = lambda: rep.orchestrator.step(
            rt_subframes=1,
            pause_timeline=True,
            delta_time=0.0,
            wait_for_render=True,
        )
        self._callback_timeout_s = callback_timeout_s
        self._pipelines: dict[str, _CameraPipeline] = {}
        self._current_state: SimulationStateFrame | None = None
        self._current_frame_index: int | None = None
        self._rendered_current_state = False
        self._cache: dict[str, CompletedRgbRender] = {}
        self._submission_index = 0
        self._last_reference_time: tuple[int, int] | None = None
        self._last_closure_metrics: dict[str, float] | None = None
        self._replay_clock = _ReplayClock(
            physics_hz=dataset_profile.physics_hz,
            policy_fps=dataset_profile.policy_fps,
        )
        self._closed = False

        settings = carb.settings.get_settings()
        settings.set("/rtx/hydra/supportMultiTickRate", True)
        settings.set("/rtx/post/motionblur/enabled", False)
        render_period = Fraction(1.0 / dataset_profile.policy_fps).limit_denominator(1_000_000_000)
        settings.set_int(
            "/app/settings/fabricDefaultSimPeriodNumerator",
            render_period.numerator,
        )
        settings.set_int(
            "/app/settings/fabricDefaultSimPeriodDenominator",
            render_period.denominator,
        )
        if settings.get_as_bool("/rtx/post/motionblur/enabled"):
            raise RuntimeError("offline dataset renderer could not disable motion blur")
        from pxr import UsdLux  # type: ignore[attr-defined]

        dome = UsdLux.DomeLight.Get(scene.stage, "/World/Lighting/Environment")
        if not dome:
            raise RuntimeError("offline dataset renderer cannot resolve the environment light")
        intensity_attr = dome.GetIntensityAttr()
        exposure_attr = dome.GetExposureAttr()
        authored_intensity = intensity_attr.Get()
        authored_exposure = exposure_attr.Get()
        if not isinstance(authored_intensity, (int, float)) or not isinstance(
            authored_exposure,
            (int, float),
        ):
            raise RuntimeError("offline dataset authored lighting values are unavailable")
        intensity_attr.Set(
            float(authored_intensity) * visual_domain_variant.lighting_intensity_scale
        )
        exposure_attr.Set(float(authored_exposure) + visual_domain_variant.exposure_offset)
        if visual_domain_variant.background_color_rgb is not None:
            settings.set_float_array(
                "/rtx/background/source/color",
                list(visual_domain_variant.background_color_rgb),
            )
        import isaacsim.core.experimental.utils.prim as prim_utils  # type: ignore[import-not-found]
        import isaacsim.core.experimental.utils.stage as stage_utils  # type: ignore[import-not-found]
        from pxr import Sdf

        fabric_stage = stage_utils.get_current_stage(backend="fabric")
        external_time_prim = fabric_stage.GetPrimAtPath("/ExternalSimulationTime")
        if not external_time_prim:
            external_time_prim = fabric_stage.DefinePrim("/ExternalSimulationTime", "")
        if external_time_prim.HasAttribute("omni:time"):
            external_time_attr = external_time_prim.GetAttribute("omni:time")
        else:
            external_time_attr = prim_utils.create_prim_attribute(
                external_time_prim,
                name="omni:time",
                type_name=Sdf.ValueTypeNames.Double,
            )
        external_time_attr.Set(0.0)
        self._external_simulation_time_attr = external_time_attr
        timeline = omni.timeline.get_timeline_interface()
        # Pause (never stop) preserves articulation PhysX views while allowing
        # exact render-time seek.  Standard Replicator render products, unlike
        # live CameraSensor tick gates, can capture repeatedly at delta_time=0.
        timeline.pause()
        for _ in range(4):
            if not timeline.is_playing():
                break
            warmup_update_app()
        if timeline.is_playing():
            raise RuntimeError("offline renderer could not pause the Isaac timeline")
        timeline.set_current_time(0.0)
        for _ in range(4):
            if math.isclose(
                float(timeline.get_current_time()),
                0.0,
                rel_tol=0.0,
                abs_tol=1e-9,
            ):
                break
            warmup_update_app()
        if not math.isclose(
            float(timeline.get_current_time()),
            0.0,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise RuntimeError("offline renderer could not seek initialization to zero")
        set_current_time = getattr(timeline, "set_current_time", None)
        if not callable(set_current_time):
            raise RuntimeError("Isaac timeline does not expose fixed-time seek")
        set_end_time = getattr(timeline, "set_end_time", None)
        if not callable(set_end_time):
            raise RuntimeError("Isaac timeline does not expose a writable replay range")
        commit_timeline = getattr(timeline, "commit", None)
        if not callable(commit_timeline):
            raise RuntimeError("Isaac timeline does not expose buffered-state commit")
        self._timeline = timeline
        self._set_timeline_time: Callable[[float], None] = set_current_time
        self._set_timeline_end_time: Callable[[float], None] = set_end_time
        self._commit_timeline: Callable[[], None] = commit_timeline

        projections = load_dataset_camera_projections(root, dataset_profile)
        if tuple(item.logical_id for item in projections) != _CAMERA_IDS:
            raise RuntimeError("dataset camera projection order differs")

        UsdGeom.Xform.Define(scene.stage, "/World/DatasetCameras")
        if scene.stage.GetPrimAtPath(SCENE_CAMERA_PRIM_PATH).IsValid():
            raise RuntimeError("dataset scene Camera prim already exists")
        UsdGeom.Camera.Define(scene.stage, SCENE_CAMERA_PRIM_PATH)
        eye = np.asarray(
            workcell_frame_position(scene.resolved, SCENE_CAMERA_EYE_FRAME),
            dtype=np.float64,
        )
        target = np.asarray(
            workcell_frame_position(scene.resolved, SCENE_CAMERA_TARGET_FRAME),
            dtype=np.float64,
        )
        if (
            not np.isfinite(eye).all()
            or not np.isfinite(target).all()
            or np.linalg.norm(target - eye) <= 1e-6
        ):
            raise RuntimeError("dataset scene camera eye/target is invalid")
        set_camera_view(
            eye=eye,
            target=target,
            camera_prim_path=SCENE_CAMERA_PRIM_PATH,
        )

        wrist_paths = {item.side: item.camera_prim_path for item in scene.wrist_rigs}
        if set(wrist_paths) != {"left", "right"}:
            raise RuntimeError("dataset renderer requires exactly two wrist Camera prims")
        wrist_runtimes = {item.side: item for item in scene.wrist_rig_runtimes}
        if set(wrist_runtimes) != {"left", "right"}:
            raise RuntimeError("dataset renderer requires two wrist rig runtime contracts")
        camera_paths = {
            "scene_rgb": SCENE_CAMERA_PRIM_PATH,
            "left_wrist_rgb": wrist_paths["left"],
            "right_wrist_rgb": wrist_paths["right"],
        }
        try:
            for projection in projections:
                camera_path = camera_paths[projection.logical_id]
                camera_prim = scene.stage.GetPrimAtPath(camera_path)
                if not camera_prim.IsValid() or camera_prim.GetTypeName() != "Camera":
                    raise RuntimeError(f"dataset Camera prim is invalid: {camera_path}")
                _author_optics(camera_prim, projection)
                render_product = rep.create.render_product(
                    camera_path,
                    resolution=(projection.width_px, projection.height_px),
                    force_new=True,
                    name=f"Dataset_{projection.logical_id}",
                )
                render_product_path = (
                    render_product
                    if isinstance(render_product, str)
                    else getattr(render_product, "path", None)
                )
                if not isinstance(render_product_path, str) or not render_product_path:
                    raise RuntimeError("Replicator render product has no stable USD path")
                render_product_schema = UsdRender.Product.Get(
                    scene.stage,
                    render_product_path,
                )
                if not render_product_schema.GetPrim().IsValid():
                    raise RuntimeError("Replicator render product USD prim is invalid")
                readback = _camera_readback(
                    camera_prim=camera_prim,
                    render_product=render_product_schema,
                )
                calibration = assert_dataset_projection_matches_readback(
                    projection,
                    readback,
                )
                side = (
                    projection.logical_id.removesuffix("_wrist_rgb")
                    if projection.logical_id != "scene_rgb"
                    else None
                )
                if side is None:
                    parent_prim_path = "/World"
                    parent_frame_id = "world"
                    camera_frame_id = "wujihand_scene_camera_usd"
                    optical_frame_id = "wujihand_scene_camera_optical"
                    parent_from_camera_optical = _world_from_camera_optical(
                        scene.stage,
                        camera_path,
                    )
                    static_hashes: tuple[str | None, str | None, str | None] = (
                        None,
                        None,
                        None,
                    )
                else:
                    runtime = wrist_runtimes[side]
                    parent_prim_path = scene.authored[side].config.child_base_link_path
                    parent_frame_id = f"wujihand_{side}_hand_base"
                    camera_frame_id = f"wujihand_{side}_wrist_camera_usd"
                    optical_frame_id = f"wujihand_{side}_wrist_camera_optical"
                    parent_from_camera_optical = _rigid_transform_row_major(runtime.optical_in_hand)
                    static_hashes = (
                        runtime.mount_visual_sha256,
                        runtime.camera_visual_sha256,
                        runtime.generation_report_sha256,
                    )
                sink = _RgbSink(projection)
                annotator = rep.AnnotatorRegistry.get_annotator(
                    "rgb",
                    device="cuda",
                    do_array_copy=False,
                )

                class CompletedRgbWriter(rep.Writer):  # type: ignore[misc]
                    def __init__(self, target_sink: _RgbSink, rgb_annotator: Any) -> None:
                        self.version = "1.0.0"
                        self.annotators = [rgb_annotator]
                        self._target_sink = target_sink

                    def write(self, data: dict[str, Any]) -> None:
                        self._target_sink.write(data)

                    def write_metadata(self) -> None:
                        pass

                writer = CompletedRgbWriter(sink, annotator)
                writer.attach(render_product_path)
                inventory = DatasetCameraRuntimeInventory(
                    camera_id=projection.logical_id,
                    carrier_identity=projection.carrier_identity,
                    profile_id=projection.profile_id,
                    profile_path=projection.profile_path,
                    profile_sha256=projection.profile_sha256,
                    warning=projection.warning,
                    camera_prim_path=camera_path,
                    render_product_path=render_product_path,
                    parent_prim_path=parent_prim_path,
                    parent_frame_id=parent_frame_id,
                    camera_frame_id=camera_frame_id,
                    optical_frame_id=optical_frame_id,
                    parent_from_camera_optical_row_major=parent_from_camera_optical,
                    mount_visual_sha256=static_hashes[0],
                    camera_visual_sha256=static_hashes[1],
                    generation_report_sha256=static_hashes[2],
                    readback=readback,
                    calibration=calibration,
                )
                _assert_camera_extrinsic_closure(scene.stage, inventory)
                self._pipelines[projection.logical_id] = _CameraPipeline(
                    projection=projection,
                    camera_prim_path=camera_path,
                    render_product_path=render_product_path,
                    render_product=render_product,
                    writer=writer,
                    sink=sink,
                    inventory=inventory,
                )
            if tuple(self._pipelines) != _CAMERA_IDS:
                raise RuntimeError("dataset renderer camera pipeline inventory differs")
            self._warm_up()
            # Camera/timeline initialization is outside the replay truth
            # boundary.  From this anchor onward, injection and every render
            # callback must preserve the physics step index exactly.
            self._physics_step_anchor = _physics_step_index(scene.world)
        except BaseException:
            # Never replace the construction failure with best-effort writer
            # cleanup diagnostics; the caller needs the first causal error.
            try:
                self.close()
            except BaseException:
                pass
            raise

    @property
    def simulation_time_s(self) -> float:
        value = float(self._timeline.get_current_time())
        if not math.isfinite(value) or value < 0.0:
            raise RuntimeError("Isaac renderer simulation time is invalid")
        return value

    @property
    def inventories(self) -> tuple[DatasetCameraRuntimeInventory, ...]:
        return tuple(self._pipelines[camera_id].inventory for camera_id in _CAMERA_IDS)

    @property
    def camera_runtime_inventories(self) -> tuple[DatasetCameraRuntimeInventory, ...]:
        return self.inventories

    @property
    def source_physics_grid_origin(self) -> int:
        value = self._replay_clock.source_physics_grid_origin
        if value is None:
            raise RuntimeError("source physics grid origin is unavailable before injection")
        return value

    @property
    def completed_reference_time(self) -> tuple[int, int]:
        value = self._last_reference_time
        if value is None:
            raise RuntimeError("completed reference is unavailable before rendering")
        return value

    @property
    def closure_metrics(self) -> Mapping[str, float]:
        value = self._last_closure_metrics
        if value is None:
            raise RuntimeError("closure metrics are unavailable before rendering")
        return dict(value)

    def inject_pre_action_state(
        self,
        frame: SimulationStateFrame,
        *,
        dataset_frame_index: int,
    ) -> str:
        if self._closed:
            raise RuntimeError("fixed-state renderer is closed")
        if self._cache:
            raise RuntimeError("previous three-camera render batch was not fully consumed")
        if self._current_state is not None and not self._rendered_current_state:
            raise RuntimeError("previous injected state was never rendered")
        # Drop every callback from the preceding transaction before changing
        # either the replay clock or scene truth.
        for pipeline in self._pipelines.values():
            pipeline.sink.clear()
        replay_time_s = self._replay_clock.observe(
            frame,
            dataset_frame_index=dataset_frame_index,
        )
        self._seek_without_physics_to(replay_time_s)
        self._scene.restore_dataset_state_frame(
            frame,
            q54_profile=self._dataset_profile.q54,
        )
        actual_q54 = self._dataset_profile.q54.assemble_from_q27(
            left_q27_rad=tuple(float(value) for value in self._scene.feedback_q27("left")),
            right_q27_rad=tuple(float(value) for value in self._scene.feedback_q27("right")),
        )
        actual_qdot54 = self._dataset_profile.q54.assemble_velocity_from_q27(
            left_qdot27_rad_s=tuple(float(value) for value in self._scene.feedback_qdot27("left")),
            right_qdot27_rad_s=tuple(
                float(value) for value in self._scene.feedback_qdot27("right")
            ),
        )
        if not np.allclose(actual_q54, frame.q54_rad, rtol=0.0, atol=2e-5) or not np.allclose(
            actual_qdot54,
            frame.qdot54_rad_s,
            rtol=0.0,
            atol=2e-5,
        ):
            raise RuntimeError("Isaac articulation state injection did not close")
        self._current_state = frame
        self._current_frame_index = None
        self._rendered_current_state = False
        return frame.payload_digest_sha256

    def render_rgb(
        self,
        *,
        camera_id: str,
        dataset_frame_index: int,
    ) -> CompletedRgbRender:
        if self._closed:
            raise RuntimeError("fixed-state renderer is closed")
        if camera_id not in self._pipelines:
            raise ValueError("unknown dataset camera ID")
        if type(dataset_frame_index) is not int or dataset_frame_index < 0:
            raise ValueError("dataset_frame_index must be non-negative")
        if self._current_state is None:
            raise RuntimeError("pre-action state must be injected before rendering")
        if not self._cache:
            if self._rendered_current_state:
                raise RuntimeError("one injected state may be rendered only once")
            self._render_current_batch(dataset_frame_index)
        if self._current_frame_index != dataset_frame_index:
            raise RuntimeError("camera request crossed fixed-state render batches")
        try:
            return self._cache.pop(camera_id)
        except KeyError as exc:
            raise RuntimeError("dataset camera was requested twice for one frame") from exc

    def _warm_up(self) -> None:
        warmup_count = max(
            pipeline.projection.warmup_frames for pipeline in self._pipelines.values()
        )
        if warmup_count <= 0:
            raise RuntimeError("dataset renderer requires an explicit warm-up")
        counts = {camera_id: 0 for camera_id in _CAMERA_IDS}
        maximum_steps = warmup_count * 4
        for _ in range(maximum_steps):
            self._render_update_app()
            for camera_id, pipeline in self._pipelines.items():
                if not pipeline.sink.has_record():
                    pipeline.sink.wait(0.05)
                counts[camera_id] += len(pipeline.sink.pop_all())
            if all(value >= warmup_count for value in counts.values()):
                break
        else:
            raise RuntimeError(
                f"dataset camera warm-up did not close: expected={warmup_count}, actual={counts!r}"
            )
        # Synchronous zero-delta warm-up stays at time zero.  Discard every
        # completed frame before accepting dataset_frame_index=0.
        for pipeline in self._pipelines.values():
            pipeline.sink.clear()
        if not math.isclose(self.simulation_time_s, 0.0, rel_tol=0.0, abs_tol=1e-9):
            raise RuntimeError("dataset camera warm-up reset did not return to simulation zero")

    def _seek_without_physics_to(self, replay_time_s: float) -> None:
        """Seek the independent render clock without taking a physics step."""

        if self._timeline.is_playing():
            raise RuntimeError("offline renderer requires a paused Isaac timeline")
        if self.simulation_time_s > replay_time_s + _SOURCE_TIME_TOLERANCE_S:
            raise RuntimeError("offline renderer received non-monotonic replay time")
        # Kit clamps seeks to the authored timeline range.  Grow that range
        # from the clean 30 Hz replay clock instead of the MCAP absolute time.
        self._set_timeline_end_time(replay_time_s + 1.0 / self._dataset_profile.policy_fps)
        self._set_timeline_time(replay_time_s)
        self._commit_timeline()
        self._external_simulation_time_attr.Set(replay_time_s)
        external_time_readback = float(self._external_simulation_time_attr.Get())
        if not math.isclose(
            external_time_readback,
            replay_time_s,
            rel_tol=0.0,
            abs_tol=_SOURCE_TIME_TOLERANCE_S,
        ):
            raise RuntimeError("offline renderer Fabric replay time differs")
        if not math.isclose(
            self.simulation_time_s,
            replay_time_s,
            rel_tol=0.0,
            abs_tol=_SOURCE_TIME_TOLERANCE_S,
        ):
            raise RuntimeError(
                "offline renderer could not reach the independent replay time: "
                f"target={replay_time_s:.12f}, actual={self.simulation_time_s:.12f}, "
                f"tolerance={_SOURCE_TIME_TOLERANCE_S:.9g}"
            )
        self._assert_physics_step_unchanged()

    def _assert_physics_step_unchanged(self) -> None:
        if _physics_step_index(self._scene.world) != self._physics_step_anchor:
            raise RuntimeError("offline renderer advanced the Isaac physics step index")

    def _render_current_batch(self, dataset_frame_index: int) -> None:
        state = self._current_state
        if state is None:
            raise RuntimeError("fixed-state render has no source state")
        before = self.simulation_time_s
        raw = self._submit_and_collect()
        after = self.simulation_time_s
        if not math.isclose(before, after, rel_tol=0.0, abs_tol=1e-12):
            raise RuntimeError("fixed-state render advanced Isaac simulation time")
        references = {record.reference_time for record in raw.values()}
        if len(references) != 1:
            raise RuntimeError("three Camera callbacks do not share one reference time")
        reference_time = next(iter(references))
        if self._last_reference_time is not None and (
            reference_time[0] * self._last_reference_time[1]
            <= self._last_reference_time[0] * reference_time[1]
        ):
            raise RuntimeError(
                "completed-frame reference time is not strictly monotonic: "
                f"previous={self._last_reference_time!r}, current={reference_time!r}, "
                f"replay_time_s={self.simulation_time_s:.12f}"
            )
        self._last_reference_time = reference_time
        self._assert_injected_state_closure(state)
        self._submission_index += 1
        poses = {
            camera_id: _world_from_camera_optical(
                self._scene.stage,
                pipeline.camera_prim_path,
            )
            for camera_id, pipeline in self._pipelines.items()
        }
        parent_poses = {
            camera_id: _world_from_prim(
                self._scene.stage,
                pipeline.inventory.parent_prim_path,
            )
            for camera_id, pipeline in self._pipelines.items()
        }
        for pipeline in self._pipelines.values():
            _assert_camera_extrinsic_closure(self._scene.stage, pipeline.inventory)
        for camera_id in _CAMERA_IDS:
            record = raw[camera_id]
            projection = self._pipelines[camera_id].projection
            rgb = np.ascontiguousarray(record.rgba[..., :3])
            numerator, denominator = record.reference_time
            identity = (
                f"rt-{numerator}-{denominator}-s{self._submission_index}-"
                f"f{dataset_frame_index}-{camera_id}"
            )
            self._cache[camera_id] = CompletedRgbRender(
                camera_id=camera_id,
                payload_png=encode_rgb8_png(rgb),
                completed_frame_identity=identity,
                camera_profile_sha256=projection.profile_sha256,
                parent_frame_id=self._pipelines[camera_id].inventory.parent_frame_id,
                world_from_parent_row_major=parent_poses[camera_id],
                world_from_camera_optical_row_major=poses[camera_id],
            )
        self._current_frame_index = dataset_frame_index
        self._rendered_current_state = True

    def _submit_and_collect(self) -> dict[str, _RawRgbFrame]:
        """Perform one paused kinematic-update/render/callback transaction."""

        before = self.simulation_time_s
        deadline = time.monotonic() + self._callback_timeout_s
        physics_sim_view = self._scene.world.physics_sim_view
        update_kinematics = getattr(
            physics_sim_view,
            "update_articulations_kinematic",
            None,
        )
        if not callable(update_kinematics):
            raise RuntimeError("Isaac physics view lacks articulation kinematic update")
        if update_kinematics() is False:
            raise RuntimeError("Isaac articulation kinematic update failed")
        # Tensor setters update PhysX immediately, while the renderer and the
        # recorded link-truth reader consume USD/Fabric transforms.  Write the
        # new kinematics back without simulating or issuing a render update.
        from omni.physx import get_physx_interface  # type: ignore[import-not-found]

        get_physx_interface().update_transformations(False, True, True)
        self._assert_physics_step_unchanged()
        # Replicator owns the RGB writers, so its zero-delta submission is the
        # single render-only update for this state.  No synchronization frame
        # is rendered or discarded before it.
        self._render_update_app()
        self._assert_physics_step_unchanged()
        if not math.isclose(self.simulation_time_s, before, rel_tol=0.0, abs_tol=1e-12):
            raise RuntimeError("callback service advanced Isaac simulation time")
        selected: dict[str, _RawRgbFrame] = {}
        for camera_id, pipeline in self._pipelines.items():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError("three-camera completed-frame callbacks timed out")
            pipeline.sink.wait_for_reference_after(
                self._last_reference_time,
                remaining,
            )
            records = pipeline.sink.pop_all()
            selected[camera_id] = _deduplicate_reference_records(
                records,
                camera_id=camera_id,
                after=self._last_reference_time,
            )
        if not math.isclose(
            self.simulation_time_s,
            before,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise RuntimeError("render submission advanced Isaac simulation time")
        return selected

    def _assert_injected_state_closure(self, expected: SimulationStateFrame) -> None:
        q27 = {side: self._scene.feedback_q27(side) for side in ("left", "right")}
        qdot27 = {side: self._scene.feedback_qdot27(side) for side in ("left", "right")}
        actual = self._scene.create_dataset_state_frame(
            run_id=expected.run_id,
            control_index=expected.control_index,
            phase=expected.phase,
            simulation_time_s=expected.simulation_time_s,
            physics_boundary_index=expected.physics_boundary_index,
            q54_profile=self._dataset_profile.q54,
            q27_by_side=q27,
            qdot27_by_side=qdot27,
        )
        q54_error = float(
            np.max(
                np.abs(
                    np.asarray(actual.q54_rad, dtype=np.float64)
                    - np.asarray(expected.q54_rad, dtype=np.float64)
                )
            )
        )
        qdot54_error = float(
            np.max(
                np.abs(
                    np.asarray(actual.qdot54_rad_s, dtype=np.float64)
                    - np.asarray(expected.qdot54_rad_s, dtype=np.float64)
                )
            )
        )
        if q54_error > 2e-5:
            raise RuntimeError("rendered q54 differs from the injected state")
        if qdot54_error > 2e-5:
            raise RuntimeError("rendered qdot54 differs from the injected state")
        actual_bodies = {
            (item.logical_object_id, item.prim_path): item for item in actual.rigid_bodies
        }
        expected_bodies = {
            (item.logical_object_id, item.prim_path): item for item in expected.rigid_bodies
        }
        rigid_position_error = math.inf
        rigid_linear_velocity_error = math.inf
        rigid_angular_velocity_error = math.inf
        if set(actual_bodies) == set(expected_bodies) and len(actual_bodies) == len(
            actual.rigid_bodies
        ):
            rigid_position_error = 0.0
            rigid_linear_velocity_error = 0.0
            rigid_angular_velocity_error = 0.0
            for identity, wanted in expected_bodies.items():
                current = actual_bodies[identity]
                rigid_position_error = max(
                    rigid_position_error,
                    _maximum_abs_error(current.position_m, wanted.position_m),
                )
                rigid = self._scene.dynamic_workcell_prims[wanted.prim_path]
                rigid_linear_velocity_error = max(
                    rigid_linear_velocity_error,
                    _maximum_abs_error(
                        rigid.get_linear_velocity(),
                        wanted.linear_velocity_m_s,
                    ),
                )
                rigid_angular_velocity_error = max(
                    rigid_angular_velocity_error,
                    _maximum_abs_error(
                        rigid.get_angular_velocity(),
                        wanted.angular_velocity_rad_s,
                    ),
                )
        actual_links = {
            (item.side, item.logical_link_id, item.prim_path): item
            for item in actual.kinematic_links
        }
        expected_links = {
            (item.side, item.logical_link_id, item.prim_path): item
            for item in expected.kinematic_links
        }
        link_position_error = math.inf
        if set(actual_links) == set(expected_links) and len(actual_links) == len(
            actual.kinematic_links
        ):
            link_position_error = max(
                (
                    float(
                        np.max(
                            np.abs(
                                np.asarray(actual_links[key].position_m, dtype=np.float64)
                                - np.asarray(
                                    expected_links[key].position_m,
                                    dtype=np.float64,
                                )
                            )
                        )
                    )
                    for key in actual_links
                ),
                default=0.0,
            )
        self._last_closure_metrics = {
            "q54_max_abs_error_rad": q54_error,
            "qdot54_max_abs_error_rad_s": qdot54_error,
            "rigid_body_position_max_abs_error_m": rigid_position_error,
            "rigid_body_linear_velocity_max_abs_error_m_s": (rigid_linear_velocity_error),
            "rigid_body_angular_velocity_max_abs_error_rad_s": (rigid_angular_velocity_error),
            "kinematic_link_position_max_abs_error_m": link_position_error,
        }
        _assert_truth_items_close(
            actual.rigid_bodies,
            expected.rigid_bodies,
            key=lambda item: (item.logical_object_id, item.prim_path),
            vectors=lambda item: (item.position_m,),
            quaternions=lambda item: (item.quat_wxyz,),
            field="dynamic rigid body",
        )
        if rigid_linear_velocity_error > 2e-5 or rigid_angular_velocity_error > 2e-5:
            raise RuntimeError(
                "rendered dynamic rigid body PhysX velocity truth differs: "
                f"linear_error={rigid_linear_velocity_error:.9g}, "
                f"angular_error={rigid_angular_velocity_error:.9g}, required=<=2e-05"
            )
        _assert_truth_items_close(
            actual.kinematic_links,
            expected.kinematic_links,
            key=lambda item: (item.side, item.logical_link_id, item.prim_path),
            vectors=lambda item: (item.position_m,),
            quaternions=lambda item: (item.quat_wxyz,),
            field="kinematic link",
            vector_atol=_KINEMATIC_LINK_POSITION_LIMIT_M,
            strict_vector_limit=True,
        )

    def close(self) -> None:
        if self._closed:
            return
        errors: list[BaseException] = []
        for pipeline in self._pipelines.values():
            try:
                pipeline.writer.detach()
            except BaseException as exc:
                errors.append(exc)
        try:
            self._timeline.stop()
        except BaseException as exc:
            errors.append(exc)
        for pipeline in self._pipelines.values():
            try:
                pipeline.sink.close()
            except BaseException as exc:
                errors.append(exc)
        self._closed = True
        if errors:
            raise RuntimeError(
                f"failed to close {len(errors)} dataset renderer component(s)"
            ) from errors[0]

    def __enter__(self) -> IsaacFixedStateRgbBackend:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def _physics_step_index(world: object) -> int:
    value = getattr(world, "current_time_step_index", None)
    if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
        raise RuntimeError("Isaac World does not expose a valid physics step index")
    return int(value)


def _reference_time(value: object) -> tuple[int, int]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 2:
        raise RuntimeError("RGB writer callback lacks rational reference_time")
    numerator, denominator = value
    if (
        isinstance(numerator, bool)
        or isinstance(denominator, bool)
        or not isinstance(numerator, (int, np.integer))
        or not isinstance(denominator, (int, np.integer))
    ):
        raise RuntimeError("RGB writer reference_time must contain integers")
    result = (int(numerator), int(denominator))
    if result[0] < 0 or result[1] <= 0:
        raise RuntimeError("RGB writer reference_time is invalid")
    return result


def _deduplicate_reference_records(
    records: Sequence[_RawRgbFrame],
    *,
    camera_id: str,
    after: tuple[int, int] | None = None,
) -> _RawRgbFrame:
    """Accept repeated delivery of one identical completed-frame reference."""

    if not records:
        raise RuntimeError(f"fixed-state submission produced no RGB frame: {camera_id!r}")
    by_reference: dict[tuple[int, int], _RawRgbFrame] = {}
    for record in records:
        previous = by_reference.get(record.reference_time)
        if previous is not None and not np.array_equal(previous.rgba, record.rgba):
            raise RuntimeError(
                "repeated RGB callback reference has conflicting payloads: "
                f"camera={camera_id!r}, reference={record.reference_time!r}"
            )
        by_reference[record.reference_time] = record
    if after is not None:
        older = tuple(
            reference
            for reference in by_reference
            if reference != after and not _reference_is_after(reference, after)
        )
        if older:
            raise RuntimeError(
                "RGB callback reference moved backwards: "
                f"camera={camera_id!r}, previous={after!r}, older={older!r}"
            )
        by_reference.pop(after, None)
        if not by_reference:
            raise RuntimeError(
                "RGB callback reference did not advance after duplicate removal: "
                f"camera={camera_id!r}, previous={after!r}"
            )
    if len(by_reference) != 1:
        raise RuntimeError(
            "one fixed-state submission produced multiple RGB references: "
            f"camera={camera_id!r}, references={sorted(by_reference)!r}"
        )
    return next(iter(by_reference.values()))


def _reference_is_after(
    current: tuple[int, int],
    previous: tuple[int, int],
) -> bool:
    return current[0] * previous[1] > previous[0] * current[1]


def _payload_numpy(value: object) -> NDArray[np.generic]:
    if value is None:
        raise RuntimeError("RGB writer callback has no payload")
    payload = value.get("data") if isinstance(value, Mapping) and "data" in value else value
    if hasattr(payload, "numpy"):
        payload = payload.numpy()
    return np.asarray(payload).copy()


def _author_optics(camera_prim: Any, projection: DatasetRgbCameraProjection) -> None:
    from pxr import Gf, UsdGeom

    camera = UsdGeom.Camera(camera_prim)
    optics = projection.optics
    camera.CreateProjectionAttr(optics.projection)
    camera.CreateFocalLengthAttr(optics.focal_length_mm)
    camera.CreateHorizontalApertureAttr(optics.horizontal_aperture_mm)
    camera.CreateVerticalApertureAttr(optics.vertical_aperture_mm)
    camera.CreateHorizontalApertureOffsetAttr(optics.horizontal_aperture_offset_mm)
    camera.CreateVerticalApertureOffsetAttr(optics.vertical_aperture_offset_mm)
    camera.CreateClippingRangeAttr(Gf.Vec2f(*optics.clipping_range_m))


def _camera_readback(*, camera_prim: Any, render_product: Any) -> IsaacCameraApiReadback:
    from pxr import UsdGeom

    camera = UsdGeom.Camera(camera_prim)
    clipping = camera.GetClippingRangeAttr().Get()
    width_px, height_px = (int(value) for value in render_product.GetResolutionAttr().Get())
    return IsaacCameraApiReadback(
        width_px=width_px,
        height_px=height_px,
        projection=str(camera.GetProjectionAttr().Get()),
        focal_length_mm=float(camera.GetFocalLengthAttr().Get()),
        horizontal_aperture_mm=float(camera.GetHorizontalApertureAttr().Get()),
        vertical_aperture_mm=float(camera.GetVerticalApertureAttr().Get()),
        horizontal_aperture_offset_mm=float(camera.GetHorizontalApertureOffsetAttr().Get()),
        vertical_aperture_offset_mm=float(camera.GetVerticalApertureOffsetAttr().Get()),
        clipping_range_m=(float(clipping[0]), float(clipping[1])),
    )


def _world_from_camera_optical(stage: Any, camera_path: str) -> tuple[float, ...]:
    matrix = _world_matrix(stage, camera_path)
    matrix[:3, :3] = matrix[:3, :3] @ _USD_CAMERA_FROM_ROS_OPTICAL
    _validate_rigid_matrix(matrix, field="dataset camera optical pose")
    return _flatten_matrix(matrix)


def _world_from_prim(stage: Any, prim_path: str) -> tuple[float, ...]:
    return _flatten_matrix(_world_matrix(stage, prim_path))


def _world_matrix(stage: Any, prim_path: str) -> NDArray[np.float64]:
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"dataset transform prim disappeared: {prim_path}")
    transform = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    translation = transform.ExtractTranslation()
    quaternion = transform.ExtractRotationQuat()
    imaginary = quaternion.GetImaginary()
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = _rotation_matrix(
        (
            float(quaternion.GetReal()),
            float(imaginary[0]),
            float(imaginary[1]),
            float(imaginary[2]),
        )
    )
    matrix[:3, 3] = tuple(float(translation[index]) for index in range(3))
    _validate_rigid_matrix(matrix, field=f"dataset transform {prim_path}")
    return matrix


def _rigid_transform_row_major(transform: Any) -> tuple[float, ...]:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.asarray(transform.rotation, dtype=np.float64)
    matrix[:3, 3] = np.asarray(transform.translation_m, dtype=np.float64)
    _validate_rigid_matrix(matrix, field="dataset static camera extrinsic")
    return _flatten_matrix(matrix)


def _flatten_matrix(matrix: NDArray[np.float64]) -> tuple[float, ...]:
    return tuple(float(value) for value in matrix.reshape(-1))


def _validate_rigid_matrix(matrix: NDArray[np.float64], *, field: str) -> None:
    if (
        matrix.shape != (4, 4)
        or not np.isfinite(matrix).all()
        or not np.allclose(
            matrix[:3, :3].T @ matrix[:3, :3],
            np.eye(3),
            rtol=0.0,
            atol=1e-8,
        )
        or not math.isclose(
            float(np.linalg.det(matrix[:3, :3])),
            1.0,
            rel_tol=0.0,
            abs_tol=1e-8,
        )
        or not np.array_equal(matrix[3], np.asarray((0.0, 0.0, 0.0, 1.0)))
    ):
        raise RuntimeError(f"{field} is not SE(3)")


def _assert_camera_extrinsic_closure(
    stage: Any,
    inventory: DatasetCameraRuntimeInventory,
) -> None:
    world_from_parent = np.asarray(
        _world_from_prim(stage, inventory.parent_prim_path),
        dtype=np.float64,
    ).reshape(4, 4)
    parent_from_camera = np.asarray(
        inventory.parent_from_camera_optical_row_major,
        dtype=np.float64,
    ).reshape(4, 4)
    world_from_camera = np.asarray(
        _world_from_camera_optical(stage, inventory.camera_prim_path),
        dtype=np.float64,
    ).reshape(4, 4)
    if not np.allclose(
        world_from_parent @ parent_from_camera,
        world_from_camera,
        rtol=0.0,
        atol=1e-7,
    ):
        raise RuntimeError("dataset camera parent/static/world extrinsic closure differs")


def _rotation_matrix(quaternion_wxyz: tuple[float, float, float, float]) -> NDArray[np.float64]:
    values = np.asarray(quaternion_wxyz, dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise RuntimeError("dataset camera quaternion is not normalized")
    w, x, y, z = values / norm
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


def _quaternion_close(left: Sequence[float], right: Sequence[float]) -> bool:
    lhs = np.asarray(left, dtype=np.float64)
    rhs = np.asarray(right, dtype=np.float64)
    if lhs.shape != (4,) or rhs.shape != (4,):
        return False
    lhs /= np.linalg.norm(lhs)
    rhs /= np.linalg.norm(rhs)
    return math.isclose(abs(float(np.dot(lhs, rhs))), 1.0, rel_tol=0.0, abs_tol=2e-5)


def _maximum_abs_error(left: Sequence[float], right: Sequence[float]) -> float:
    lhs = np.asarray(left, dtype=np.float64)
    rhs = np.asarray(right, dtype=np.float64)
    if lhs.shape != rhs.shape or lhs.size == 0:
        return math.inf
    return float(np.max(np.abs(lhs - rhs)))


def _assert_truth_items_close(
    actual_items: Sequence[Any],
    expected_items: Sequence[Any],
    *,
    key: Callable[[Any], tuple[object, ...]],
    vectors: Callable[[Any], Sequence[Sequence[float]]],
    quaternions: Callable[[Any], Sequence[Sequence[float]]],
    field: str,
    vector_atol: float = 2e-5,
    strict_vector_limit: bool = False,
) -> None:
    actual = {key(item): item for item in actual_items}
    expected = {key(item): item for item in expected_items}
    if set(actual) != set(expected) or len(actual) != len(actual_items):
        raise RuntimeError(f"rendered {field} inventory differs")
    for identity, wanted in expected.items():
        current = actual[identity]
        actual_vectors = vectors(current)
        expected_vectors = vectors(wanted)
        if len(actual_vectors) != len(expected_vectors):
            raise RuntimeError(f"rendered {field} vector inventory differs")
        maximum_vector_error = max(
            (
                float(
                    np.max(
                        np.abs(
                            np.asarray(lhs, dtype=np.float64) - np.asarray(rhs, dtype=np.float64)
                        )
                    )
                )
                for lhs, rhs in zip(actual_vectors, expected_vectors, strict=True)
            ),
            default=0.0,
        )
        vector_passed = (
            maximum_vector_error < vector_atol
            if strict_vector_limit
            else maximum_vector_error <= vector_atol
        )
        if not vector_passed:
            relation = "<" if strict_vector_limit else "<="
            raise RuntimeError(
                f"rendered {field} vector truth differs: "
                f"max_error={maximum_vector_error:.9g}, required={relation}{vector_atol:.9g}"
            )
        actual_quaternions = quaternions(current)
        expected_quaternions = quaternions(wanted)
        if len(actual_quaternions) != len(expected_quaternions) or any(
            not _quaternion_close(lhs, rhs)
            for lhs, rhs in zip(actual_quaternions, expected_quaternions, strict=True)
        ):
            raise RuntimeError(f"rendered {field} quaternion truth differs")


__all__ = [
    "ISAAC_DATASET_RENDERER_IDENTITY",
    "SCENE_CAMERA_PRIM_PATH",
    "IsaacFixedStateRgbBackend",
]
