"""Read-only OpenVR adapter for one serial-addressed tracked rigid body.

The optional OpenVR dependency is imported only when the runtime is first
opened.  No SDK object or ephemeral tracked-device index crosses this adapter
boundary: callers receive only canonical domain and port values plus a
JSON-safe copy of the latest raw record.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from copy import deepcopy
from dataclasses import dataclass
import importlib
import math
import re
import time
from types import ModuleType
from typing import Protocol, TypeAlias, cast

import numpy as np
import numpy.typing as npt

from wujihand.domain import (
    HOST_MONOTONIC_CLOCK_DOMAIN,
    ClutchEdge,
    ClutchEvent,
    TrackedRigidBodySample,
    TrackingState,
)
from wujihand.domain.pose import (
    align_quaternion_hemisphere,
    rotation_matrix_to_quaternion_wxyz,
    validate_host_time_ns,
)
from wujihand.ports import TrackerInventoryItem, TrackingPoll


JsonScalar: TypeAlias = str | int | float | bool | None
JsonValue: TypeAlias = JsonScalar | list["JsonValue"] | dict[str, "JsonValue"]
RawOpenVrRecord: TypeAlias = dict[str, JsonValue]

_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/+-]{0,127}$")
_ROTATION_ATOL = 1.0e-4


class _OpenVrMatrix34(Protocol):
    m: Sequence[Sequence[float]]


class _OpenVrTrackedPose(Protocol):
    mDeviceToAbsoluteTracking: _OpenVrMatrix34
    eTrackingResult: int
    bPoseIsValid: bool
    bDeviceIsConnected: bool


class _OpenVrControllerState(Protocol):
    ulButtonPressed: int


class _OpenVrSystem(Protocol):
    def getTrackedDeviceClass(self, device_index: int) -> int: ...

    def isTrackedDeviceConnected(self, device_index: int) -> bool: ...

    def getStringTrackedDeviceProperty(self, device_index: int, prop: int) -> str: ...

    def getDeviceToAbsoluteTrackingPose(
        self,
        origin: int,
        predicted_seconds_to_photons_from_now: float,
        tracked_device_pose_array: object,
    ) -> Sequence[_OpenVrTrackedPose]: ...

    def getControllerState(
        self,
        controller_device_index: int,
    ) -> tuple[bool, _OpenVrControllerState]: ...


class _OpenVrModule(Protocol):
    VRApplication_Background: int
    TrackingUniverseStanding: int
    k_unMaxTrackedDeviceCount: int

    TrackedDeviceClass_Invalid: int
    TrackedDeviceClass_HMD: int
    TrackedDeviceClass_Controller: int
    TrackedDeviceClass_GenericTracker: int
    TrackedDeviceClass_TrackingReference: int
    TrackedDeviceClass_DisplayRedirect: int

    Prop_SerialNumber_String: int
    Prop_ModelNumber_String: int
    Prop_ManufacturerName_String: int

    TrackingResult_Uninitialized: int
    TrackingResult_Calibrating_InProgress: int
    TrackingResult_Calibrating_OutOfRange: int
    TrackingResult_Running_OK: int
    TrackingResult_Running_OutOfRange: int
    TrackingResult_Fallback_RotationOnly: int

    def init(self, application_type: int) -> _OpenVrSystem: ...

    def shutdown(self) -> None: ...


def matrix34_to_pose_m_wxyz(
    matrix_3x4: Sequence[Sequence[float]] | npt.NDArray[np.floating],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Convert a device-to-tracking 3x4 transform to metres and active ``wxyz``.

    OpenVR matrices are float32, so a small orthonormality tolerance is allowed
    before projecting the rotation to the nearest proper orthogonal matrix.
    Reflections, scaled matrices, malformed input, and non-finite values are
    rejected.
    """

    try:
        matrix = np.asarray(matrix_3x4, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("matrix_3x4 must be a numeric 3x4 matrix") from exc
    if matrix.shape != (3, 4):
        raise ValueError(f"matrix_3x4 must have shape (3, 4), got {matrix.shape}")
    if not np.isfinite(matrix).all():
        raise ValueError("matrix_3x4 contains NaN or infinity")

    rotation = matrix[:, :3]
    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3),
        rtol=0.0,
        atol=_ROTATION_ATOL,
    ):
        raise ValueError("matrix_3x4 rotation is not orthonormal")
    determinant = float(np.linalg.det(rotation))
    if not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=_ROTATION_ATOL):
        raise ValueError("matrix_3x4 rotation must be right-handed")

    left, _, right_transpose = np.linalg.svd(rotation)
    projected_rotation = left @ right_transpose
    if float(np.linalg.det(projected_rotation)) <= 0.0:
        raise ValueError("matrix_3x4 rotation projection is not right-handed")
    quaternion = rotation_matrix_to_quaternion_wxyz(projected_rotation)
    return (
        (
            float(matrix[0, 3]),
            float(matrix[1, 3]),
            float(matrix[2, 3]),
        ),
        (
            float(quaternion[0]),
            float(quaternion[1]),
            float(quaternion[2]),
            float(quaternion[3]),
        ),
    )


def _load_openvr_runtime() -> _OpenVrModule:
    try:
        module: ModuleType = importlib.import_module("openvr")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "OpenVR support is unavailable; install the project tracking extra"
        ) from exc
    return cast(_OpenVrModule, module)


def _identifier(value: object, *, field: str) -> str:
    if type(value) is not str or _IDENTIFIER.fullmatch(value) is None:
        raise ValueError(f"{field} must be a bounded transport-safe identifier")
    return value


def _inventory_text(value: str | None) -> str:
    if value is None:
        return "unknown"
    cleaned = "".join(character for character in value.strip() if character.isprintable())
    return cleaned[:128] if cleaned else "unknown"


def _matrix_rows(matrix: _OpenVrMatrix34) -> list[list[float]]:
    rows = [[float(matrix.m[row][column]) for column in range(4)] for row in range(3)]
    numeric = np.asarray(rows, dtype=np.float64)
    if numeric.shape != (3, 4) or not np.isfinite(numeric).all():
        raise ValueError("OpenVR returned a malformed or non-finite 3x4 matrix")
    return rows


class OpenVrTrackerAdapter:
    """Normalize one OpenVR generic tracker selected by stable serial.

    ``inventory`` may be used without configuring a serial.  ``start`` and
    ``poll`` require one.  Every poll resolves the serial against the current
    runtime inventory, deliberately avoiding persistence of OpenVR's transient
    tracked-device index.
    """

    def __init__(
        self,
        tracker_serial: str | None,
        stream_id: str,
        logical_role: str,
        *,
        producer_instance: str,
        transport_epoch: int,
        tracking_setup_revision: str,
        tracking_frame: str = "vive_tracking",
        clutch_button_id: int | None = None,
        clutch_input_id: str = "tracker_clutch",
        clock_domain: str = HOST_MONOTONIC_CLOCK_DOMAIN,
    ) -> None:
        self.tracker_serial = (
            None
            if tracker_serial is None
            else _identifier(tracker_serial, field="tracker_serial")
        )
        self.stream_id = _identifier(stream_id, field="stream_id")
        self.logical_role = _identifier(logical_role, field="logical_role")
        self.producer_instance = _identifier(
            producer_instance,
            field="producer_instance",
        )
        if type(transport_epoch) is not int or transport_epoch < 0:
            raise ValueError("transport_epoch must be a non-negative integer")
        self.transport_epoch = transport_epoch
        self.tracking_setup_revision = _identifier(
            tracking_setup_revision,
            field="tracking_setup_revision",
        )
        self.tracking_frame = _identifier(tracking_frame, field="tracking_frame")
        self.clutch_input_id = _identifier(clutch_input_id, field="clutch_input_id")
        if clock_domain != HOST_MONOTONIC_CLOCK_DOMAIN:
            raise ValueError(
                f"clock_domain must be {HOST_MONOTONIC_CLOCK_DOMAIN!r}"
            )
        self.clock_domain = clock_domain
        if clutch_button_id is not None and (
            type(clutch_button_id) is not int or not 0 <= clutch_button_id < 64
        ):
            raise ValueError("clutch_button_id must be an integer in [0, 63] or None")
        self.clutch_button_id = clutch_button_id

        self._openvr: _OpenVrModule | None = None
        self._system: _OpenVrSystem | None = None
        self._selected: TrackerInventoryItem | None = None
        self._started = False
        self._sample_sequence = 0
        self._clutch_sequence = 0
        self._last_host_time_ns = -1
        self._previous_quaternion: npt.NDArray[np.float64] | None = None
        self._button_pressed: bool | None = None
        self._last_raw_record: RawOpenVrRecord | None = None

    @property
    def last_raw_record(self) -> Mapping[str, JsonValue] | None:
        """Return an isolated JSON-safe copy of the latest OpenVR observation."""

        if self._last_raw_record is None:
            return None
        return deepcopy(self._last_raw_record)

    def inventory(self) -> tuple[TrackerInventoryItem, ...]:
        """List stable identities for all known non-invalid OpenVR devices."""

        module, system = self._ensure_runtime()
        items: list[TrackerInventoryItem] = []
        for device_index in range(module.k_unMaxTrackedDeviceCount):
            device_class = int(system.getTrackedDeviceClass(device_index))
            if device_class == module.TrackedDeviceClass_Invalid:
                continue
            serial = self._string_property(
                device_index,
                module.Prop_SerialNumber_String,
            )
            if serial is None or not serial.strip():
                continue
            items.append(
                TrackerInventoryItem(
                    serial=serial.strip(),
                    device_class=self._device_class_name(device_class),
                    model=_inventory_text(
                        self._string_property(
                            device_index,
                            module.Prop_ModelNumber_String,
                        )
                    ),
                    manufacturer=_inventory_text(
                        self._string_property(
                            device_index,
                            module.Prop_ManufacturerName_String,
                        )
                    ),
                    connected=bool(system.isTrackedDeviceConnected(device_index)),
                )
            )
        return tuple(
            sorted(
                items,
                key=lambda item: (
                    item.serial,
                    not item.connected,
                    item.device_class,
                    item.model,
                ),
            )
        )

    def start(self) -> TrackerInventoryItem:
        """Resolve the configured serial and reset stream-local state."""

        if self.tracker_serial is None:
            raise ValueError("tracker_serial is required to start tracking")
        matches = [
            item for item in self.inventory() if item.serial == self.tracker_serial
        ]
        selected = self._select_unique_inventory_item(matches)
        if selected.device_class != "generic_tracker":
            raise RuntimeError(
                f"serial {self.tracker_serial!r} is {selected.device_class}, "
                "not an OpenVR generic tracker"
            )

        self._selected = selected
        self._started = True
        self._sample_sequence = 0
        self._clutch_sequence = 0
        self._last_host_time_ns = -1
        self._previous_quaternion = None
        self._button_pressed = None
        self._last_raw_record = None
        return selected

    def poll(self, *, host_time_ns: int | None = None) -> TrackingPoll:
        """Acquire one normalized sample and any clutch edge observed with it."""

        module, system = self._require_started()
        timestamp = (
            time.monotonic_ns()
            if host_time_ns is None
            else validate_host_time_ns(host_time_ns)
        )
        if timestamp <= self._last_host_time_ns:
            raise ValueError("host_time_ns must increase strictly between polls")
        poses = system.getDeviceToAbsoluteTrackingPose(
            module.TrackingUniverseStanding,
            0.0,
            (),
        )
        return self._poll_snapshot(timestamp=timestamp, poses=poses)

    def _poll_snapshot(
        self,
        *,
        timestamp: int,
        poses: Sequence[_OpenVrTrackedPose],
    ) -> TrackingPoll:
        """Normalize this stream from one owner-acquired OpenVR pose array."""

        if timestamp <= self._last_host_time_ns:
            raise ValueError("host_time_ns must increase strictly between polls")
        resolved = self._resolve_serial()
        if resolved is None:
            self._previous_quaternion = None
            self._button_pressed = None
            self._record_raw_loss(timestamp, tracking_result="device_not_found")
            return self._finish_poll(
                timestamp,
                sample=self._invalid_sample(
                    timestamp,
                    connected=False,
                    tracking_state=TrackingState.LOST,
                ),
                clutch_events=(),
            )

        device_index, device_class = resolved
        if device_index >= len(poses):
            raise RuntimeError("OpenVR pose array does not contain the resolved device")
        pose = poses[device_index]
        connected = bool(pose.bDeviceIsConnected)
        raw_pose_valid = bool(pose.bPoseIsValid)
        tracking_result = int(pose.eTrackingResult)
        matrix_rows: list[list[float]] | None
        try:
            matrix_rows = _matrix_rows(pose.mDeviceToAbsoluteTracking)
        except (IndexError, TypeError, ValueError, OverflowError):
            matrix_rows = None
        self._last_raw_record = {
            "host_time_ns": timestamp,
            "serial": cast(str, self.tracker_serial),
            "device_class": self._device_class_name(device_class),
            "connected": connected,
            "pose_valid": raw_pose_valid,
            "tracking_result": tracking_result,
            "matrix_3x4": cast(JsonValue, matrix_rows),
        }

        tracking_state = self._tracking_state(
            tracking_result,
            connected=connected,
            pose_valid=raw_pose_valid,
        )
        if tracking_state is TrackingState.RUNNING and matrix_rows is None:
            tracking_state = TrackingState.LOST
        sample: TrackedRigidBodySample
        if tracking_state is TrackingState.RUNNING and matrix_rows is not None:
            try:
                position, quaternion_tuple = matrix34_to_pose_m_wxyz(matrix_rows)
            except ValueError:
                tracking_state = TrackingState.LOST
            else:
                quaternion = np.asarray(quaternion_tuple, dtype=np.float64)
                if self._previous_quaternion is not None:
                    quaternion = align_quaternion_hemisphere(
                        quaternion,
                        self._previous_quaternion,
                    )
                self._previous_quaternion = quaternion.copy()
                sample = TrackedRigidBodySample(
                    stream_id=self.stream_id,
                    device_serial=cast(str, self.tracker_serial),
                    logical_role=self.logical_role,
                    producer_instance=self.producer_instance,
                    transport_epoch=self.transport_epoch,
                    tracking_setup_revision=self.tracking_setup_revision,
                    sequence=self._sample_sequence,
                    tracking_frame=self.tracking_frame,
                    position_m=position,
                    quat_wxyz=cast(
                        tuple[float, float, float, float],
                        tuple(float(value) for value in quaternion),
                    ),
                    connected=True,
                    pose_valid=True,
                    tracking_state=TrackingState.RUNNING,
                    quality=1.0,
                    host_time_ns=timestamp,
                    device_time_ns=None,
                    clock_domain=self.clock_domain,
                )
                events = self._clutch_events(device_index, timestamp, connected=True)
                return self._finish_poll(
                    timestamp,
                    sample=sample,
                    clutch_events=events,
                )

        self._previous_quaternion = None
        sample = self._invalid_sample(
            timestamp,
            connected=connected,
            tracking_state=tracking_state,
        )
        events = self._clutch_events(device_index, timestamp, connected=connected)
        return self._finish_poll(
            timestamp,
            sample=sample,
            clutch_events=events,
        )

    def close(self) -> None:
        """Release the OpenVR runtime; repeated calls are harmless."""

        module = self._openvr
        self._detach_runtime()
        if module is not None:
            module.shutdown()

    def __enter__(self) -> OpenVrTrackerAdapter:
        if self.tracker_serial is None:
            self._ensure_runtime()
        else:
            self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _ensure_runtime(self) -> tuple[_OpenVrModule, _OpenVrSystem]:
        if self._openvr is not None and self._system is not None:
            return self._openvr, self._system
        module = _load_openvr_runtime()
        try:
            system = module.init(module.VRApplication_Background)
        except Exception:
            self._openvr = None
            self._system = None
            raise
        self._openvr = module
        self._system = system
        return module, system

    def _attach_runtime(
        self,
        module: _OpenVrModule,
        system: _OpenVrSystem,
    ) -> None:
        if self._openvr is not None or self._system is not None:
            raise RuntimeError("OpenVR runtime is already attached")
        self._openvr = module
        self._system = system

    def _detach_runtime(self) -> None:
        self._openvr = None
        self._system = None
        self._selected = None
        self._started = False
        self._sample_sequence = 0
        self._clutch_sequence = 0
        self._last_host_time_ns = -1
        self._previous_quaternion = None
        self._button_pressed = None
        self._last_raw_record = None

    def _require_started(self) -> tuple[_OpenVrModule, _OpenVrSystem]:
        if (
            not self._started
            or self.tracker_serial is None
            or self._openvr is None
            or self._system is None
        ):
            raise RuntimeError("start() must succeed before poll()")
        return self._openvr, self._system

    def _string_property(self, device_index: int, prop: int) -> str | None:
        assert self._system is not None
        try:
            value = self._system.getStringTrackedDeviceProperty(device_index, prop)
        except Exception:
            return None
        return value if isinstance(value, str) else None

    def _device_class_name(self, device_class: int) -> str:
        assert self._openvr is not None
        names = {
            self._openvr.TrackedDeviceClass_HMD: "hmd",
            self._openvr.TrackedDeviceClass_Controller: "controller",
            self._openvr.TrackedDeviceClass_GenericTracker: "generic_tracker",
            self._openvr.TrackedDeviceClass_TrackingReference: "tracking_reference",
            self._openvr.TrackedDeviceClass_DisplayRedirect: "display_redirect",
        }
        return names.get(device_class, f"openvr_class_{device_class}")

    def _select_unique_inventory_item(
        self,
        matches: Sequence[TrackerInventoryItem],
    ) -> TrackerInventoryItem:
        connected = [item for item in matches if item.connected]
        candidates = connected if connected else list(matches)
        if not candidates:
            raise LookupError(f"OpenVR tracker serial {self.tracker_serial!r} was not found")
        if len(candidates) != 1:
            raise RuntimeError(
                f"OpenVR tracker serial {self.tracker_serial!r} is not unique"
            )
        return candidates[0]

    def _resolve_serial(self) -> tuple[int, int] | None:
        assert self.tracker_serial is not None
        assert self._openvr is not None
        assert self._system is not None
        matches: list[tuple[int, int, bool]] = []
        for device_index in range(self._openvr.k_unMaxTrackedDeviceCount):
            device_class = int(self._system.getTrackedDeviceClass(device_index))
            if device_class == self._openvr.TrackedDeviceClass_Invalid:
                continue
            serial = self._string_property(
                device_index,
                self._openvr.Prop_SerialNumber_String,
            )
            if serial == self.tracker_serial:
                matches.append(
                    (
                        device_index,
                        device_class,
                        bool(self._system.isTrackedDeviceConnected(device_index)),
                    )
                )
        connected = [match for match in matches if match[2]]
        candidates = connected if connected else matches
        if not candidates:
            return None
        if len(candidates) != 1:
            raise RuntimeError(
                f"OpenVR tracker serial {self.tracker_serial!r} resolved ambiguously"
            )
        device_index, device_class, _ = candidates[0]
        return device_index, device_class

    def _tracking_state(
        self,
        tracking_result: int,
        *,
        connected: bool,
        pose_valid: bool,
    ) -> TrackingState:
        assert self._openvr is not None
        if not connected:
            return TrackingState.LOST
        if tracking_result == self._openvr.TrackingResult_Uninitialized:
            return TrackingState.UNINITIALIZED
        if tracking_result in {
            self._openvr.TrackingResult_Calibrating_InProgress,
            self._openvr.TrackingResult_Calibrating_OutOfRange,
        }:
            return TrackingState.CALIBRATING
        if tracking_result == self._openvr.TrackingResult_Running_OutOfRange:
            return TrackingState.OUT_OF_RANGE
        if tracking_result == self._openvr.TrackingResult_Fallback_RotationOnly:
            return TrackingState.ROTATION_ONLY
        if (
            tracking_result == self._openvr.TrackingResult_Running_OK
            and pose_valid
        ):
            return TrackingState.RUNNING
        return TrackingState.LOST

    def _invalid_sample(
        self,
        timestamp: int,
        *,
        connected: bool,
        tracking_state: TrackingState,
    ) -> TrackedRigidBodySample:
        assert self.tracker_serial is not None
        return TrackedRigidBodySample(
            stream_id=self.stream_id,
            device_serial=self.tracker_serial,
            logical_role=self.logical_role,
            producer_instance=self.producer_instance,
            transport_epoch=self.transport_epoch,
            tracking_setup_revision=self.tracking_setup_revision,
            sequence=self._sample_sequence,
            tracking_frame=self.tracking_frame,
            position_m=None,
            quat_wxyz=None,
            connected=connected,
            pose_valid=False,
            tracking_state=tracking_state,
            quality=None,
            host_time_ns=timestamp,
            device_time_ns=None,
            clock_domain=self.clock_domain,
        )

    def _clutch_events(
        self,
        device_index: int,
        timestamp: int,
        *,
        connected: bool,
    ) -> tuple[ClutchEvent, ...]:
        if self.clutch_button_id is None or not connected:
            self._button_pressed = None
            return ()
        assert self._system is not None
        state_valid, controller_state = self._system.getControllerState(device_index)
        if not state_valid:
            self._button_pressed = None
            return ()
        pressed = bool(int(controller_state.ulButtonPressed) & (1 << self.clutch_button_id))
        previous = self._button_pressed
        self._button_pressed = pressed
        if previous is None or previous == pressed:
            return ()

        edge = ClutchEdge.PRESSED if pressed else ClutchEdge.RELEASED
        assert self.tracker_serial is not None
        event = ClutchEvent(
            stream_id=self.stream_id,
            device_serial=self.tracker_serial,
            logical_role=self.logical_role,
            producer_instance=self.producer_instance,
            transport_epoch=self.transport_epoch,
            tracking_setup_revision=self.tracking_setup_revision,
            input_id=self.clutch_input_id,
            edge=edge,
            sequence=self._clutch_sequence,
            host_time_ns=timestamp,
            clock_domain=self.clock_domain,
            epoch_request=edge is ClutchEdge.PRESSED,
        )
        self._clutch_sequence += 1
        return (event,)

    def _record_raw_loss(
        self,
        timestamp: int,
        *,
        tracking_result: str,
    ) -> None:
        assert self.tracker_serial is not None
        device_class = (
            "generic_tracker"
            if self._selected is None
            else self._selected.device_class
        )
        self._last_raw_record = {
            "host_time_ns": timestamp,
            "serial": self.tracker_serial,
            "device_class": device_class,
            "connected": False,
            "pose_valid": False,
            "tracking_result": tracking_result,
            "matrix_3x4": None,
        }

    def _finish_poll(
        self,
        timestamp: int,
        *,
        sample: TrackedRigidBodySample,
        clutch_events: tuple[ClutchEvent, ...],
    ) -> TrackingPoll:
        self._last_host_time_ns = timestamp
        self._sample_sequence += 1
        return TrackingPoll(sample=sample, clutch_events=clutch_events)


@dataclass(frozen=True, slots=True)
class OpenVrTrackerStreamConfig:
    """One serial-addressed canonical stream owned by a shared runtime."""

    tracker_serial: str
    stream_id: str
    logical_role: str
    tracking_frame: str = "vive_tracking"
    clutch_button_id: int | None = None
    clutch_input_id: str = "tracker_clutch"

    def __post_init__(self) -> None:
        for field in (
            "tracker_serial",
            "stream_id",
            "logical_role",
            "tracking_frame",
            "clutch_input_id",
        ):
            object.__setattr__(
                self,
                field,
                _identifier(getattr(self, field), field=field),
            )
        if self.clutch_button_id is not None and (
            type(self.clutch_button_id) is not int
            or not 0 <= self.clutch_button_id < 64
        ):
            raise ValueError(
                "clutch_button_id must be an integer in [0, 63] or None"
            )


class OpenVrMultiTrackerAdapter:
    """Own one OpenVR runtime and normalize all configured streams per snapshot."""

    def __init__(
        self,
        streams: Sequence[OpenVrTrackerStreamConfig],
        *,
        producer_instance: str,
        transport_epoch: int,
        tracking_setup_revision: str,
        clock_domain: str = HOST_MONOTONIC_CLOCK_DOMAIN,
    ) -> None:
        try:
            configs = tuple(streams)
        except TypeError as exc:
            raise ValueError("streams must be a sequence") from exc
        if not configs or any(
            type(config) is not OpenVrTrackerStreamConfig for config in configs
        ):
            raise ValueError(
                "streams must contain at least one OpenVrTrackerStreamConfig"
            )
        for field in ("tracker_serial", "stream_id", "logical_role"):
            values = tuple(getattr(config, field) for config in configs)
            if len(set(values)) != len(values):
                raise ValueError(f"streams must have unique {field} values")
        self.streams = configs
        self.producer_instance = _identifier(
            producer_instance,
            field="producer_instance",
        )
        if type(transport_epoch) is not int or transport_epoch < 0:
            raise ValueError("transport_epoch must be a non-negative integer")
        self.transport_epoch = transport_epoch
        self.tracking_setup_revision = _identifier(
            tracking_setup_revision,
            field="tracking_setup_revision",
        )
        if clock_domain != HOST_MONOTONIC_CLOCK_DOMAIN:
            raise ValueError(
                f"clock_domain must be {HOST_MONOTONIC_CLOCK_DOMAIN!r}"
            )
        self.clock_domain = clock_domain
        self._channels = tuple(
            OpenVrTrackerAdapter(
                config.tracker_serial,
                config.stream_id,
                config.logical_role,
                producer_instance=self.producer_instance,
                transport_epoch=self.transport_epoch,
                tracking_setup_revision=self.tracking_setup_revision,
                tracking_frame=config.tracking_frame,
                clutch_button_id=config.clutch_button_id,
                clutch_input_id=config.clutch_input_id,
                clock_domain=clock_domain,
            )
            for config in configs
        )
        self._openvr: _OpenVrModule | None = None
        self._system: _OpenVrSystem | None = None
        self._started = False
        self._last_host_time_ns = -1

    @property
    def last_raw_records(self) -> Mapping[str, Mapping[str, JsonValue] | None]:
        """Return isolated raw records keyed by canonical stream ID."""

        return {
            channel.stream_id: channel.last_raw_record
            for channel in self._channels
        }

    def inventory(self) -> tuple[TrackerInventoryItem, ...]:
        """List stable identities from the one owned OpenVR runtime."""

        self._ensure_runtime()
        return self._channels[0].inventory()

    def start(self) -> tuple[TrackerInventoryItem, ...]:
        """Resolve every configured serial before allowing any stream to poll."""

        if self._started:
            raise RuntimeError("OpenVR multi-tracker adapter is already started")
        self._ensure_runtime()
        try:
            selected = tuple(channel.start() for channel in self._channels)
        except Exception:
            self.close()
            raise
        self._started = True
        self._last_host_time_ns = -1
        return selected

    def poll(
        self,
        *,
        host_time_ns: int | None = None,
    ) -> tuple[TrackingPoll, ...]:
        """Read one pose array and normalize every stream at the same timestamp."""

        module, system = self._require_started()
        timestamp = (
            time.monotonic_ns()
            if host_time_ns is None
            else validate_host_time_ns(host_time_ns)
        )
        if timestamp <= self._last_host_time_ns:
            raise ValueError("host_time_ns must increase strictly between polls")
        poses = system.getDeviceToAbsoluteTrackingPose(
            module.TrackingUniverseStanding,
            0.0,
            (),
        )
        for channel in self._channels:
            resolved = channel._resolve_serial()
            if resolved is not None and resolved[0] >= len(poses):
                raise RuntimeError(
                    "OpenVR pose array does not contain a resolved device"
                )
        polls = tuple(
            channel._poll_snapshot(timestamp=timestamp, poses=poses)
            for channel in self._channels
        )
        self._last_host_time_ns = timestamp
        return polls

    def close(self) -> None:
        """Release all stream state and shut down the shared runtime once."""

        module = self._openvr
        self._openvr = None
        self._system = None
        self._started = False
        self._last_host_time_ns = -1
        for channel in self._channels:
            channel._detach_runtime()
        if module is not None:
            module.shutdown()

    def __enter__(self) -> OpenVrMultiTrackerAdapter:
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _ensure_runtime(self) -> tuple[_OpenVrModule, _OpenVrSystem]:
        if self._openvr is not None and self._system is not None:
            return self._openvr, self._system
        module = _load_openvr_runtime()
        try:
            system = module.init(module.VRApplication_Background)
            for channel in self._channels:
                channel._attach_runtime(module, system)
        except Exception:
            for channel in self._channels:
                channel._detach_runtime()
            module.shutdown()
            raise
        self._openvr = module
        self._system = system
        return module, system

    def _require_started(self) -> tuple[_OpenVrModule, _OpenVrSystem]:
        if not self._started or self._openvr is None or self._system is None:
            raise RuntimeError("start() must succeed before poll()")
        return self._openvr, self._system


__all__ = [
    "JsonValue",
    "OpenVrMultiTrackerAdapter",
    "OpenVrTrackerAdapter",
    "OpenVrTrackerStreamConfig",
    "RawOpenVrRecord",
    "matrix34_to_pose_m_wxyz",
]
