"""Wuji Glove ``hand_skeleton`` input normalized to the hand domain contract.

The SDK is imported only when an adapter-owned connection is started.  SDK
device, subscription, frame, and joint objects remain inside this adapter.
The SDK frame timestamp is a device clock value and is therefore never
presented as a host-comparable source timestamp.
"""

from __future__ import annotations

from collections.abc import Sequence
import importlib
import re
import time
from types import ModuleType
from typing import Protocol, cast

from wujihand.domain import (
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    CanonicalHandObservation,
    HandLandmark,
    HandSide,
    MediaPipeHandLandmark,
)
from wujihand.domain.pose import validate_host_time_ns
from wujihand.ports import NoHandObservationAvailable


_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/+-]{0,127}$")
_DEVICE_NAME = re.compile(r"^[A-Za-z0-9_-]{1,64}$")


class NoHandSkeletonFrameAvailable(NoHandObservationAvailable):
    """Raised when the SDK's synchronous non-blocking subscription has no frame."""


class _FrameHeader(Protocol):
    seq: int
    timestamp_us: int
    frame_id: str


class _SkeletonPose(Protocol):
    position: Sequence[float]


class _SkeletonJoint(Protocol):
    name: str
    pose: _SkeletonPose
    confidence: float


class _HandSkeleton(Protocol):
    header: _FrameHeader
    joints: Sequence[_SkeletonJoint]


class _HandSkeletonSubscription(Protocol):
    def recv(self) -> _HandSkeleton | None: ...

    def close(self) -> None: ...


class _HandSkeletonResource(Protocol):
    def subscribe(self) -> _HandSkeletonSubscription: ...


class _HandSideResource(Protocol):
    def get(self) -> object: ...


class _WujiGlove(Protocol):
    def hand_skeleton(self) -> _HandSkeletonResource: ...

    def hand_side(self) -> _HandSideResource: ...


class _SdkManager(Protocol):
    def connect(self, *, device_name: str, **selection: object) -> _WujiGlove: ...

    def disconnect(self, *, device_name: str) -> None: ...


class _SdkManagerType(Protocol):
    @staticmethod
    def instance() -> _SdkManager: ...


class _HandednessType(Protocol):
    Left: object
    Right: object


class _WujiSdkModule(Protocol):
    SdkManager: _SdkManagerType
    Handedness: _HandednessType


def _load_wuji_sdk() -> _WujiSdkModule:
    try:
        module: ModuleType = importlib.import_module("wuji_sdk")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Wuji Glove support is unavailable; install wuji-sdk on supported Linux"
        ) from exc
    return cast(_WujiSdkModule, module)


def _identifier(value: object, *, field: str) -> str:
    if type(value) is not str or _IDENTIFIER.fullmatch(value) is None:
        raise ValueError(f"{field} must be a bounded transport-safe identifier")
    return value


def _optional_selector(value: object, *, field: str) -> str | None:
    if value is None:
        return None
    if type(value) is not str:
        raise ValueError(f"{field} must be a non-empty string or None")
    selected = value.strip()
    if not selected or len(selected) > 256 or not selected.isprintable():
        raise ValueError(f"{field} must be a non-empty printable string or None")
    return selected


def _non_negative_int(value: object, *, field: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    return value


def _reported_side_matches(
    reported_side: object,
    configured_side: HandSide,
    *,
    expected_sdk_handedness: object | None,
) -> bool:
    if expected_sdk_handedness is not None and reported_side == expected_sdk_handedness:
        return True
    if type(reported_side) is str:
        return reported_side == configured_side.value
    return str(reported_side) == configured_side.value


class WujiGloveHandSkeletonAdapter:
    """Read one configured Wuji Glove's canonical 21-landmark skeleton stream.

    Production callers normally provide only stable selection/configuration
    values and let the adapter own ``SdkManager.connect``/``disconnect``.
    ``manager``, ``sdk_module``, and ``glove`` are dependency-injection seams
    for qualification tests and composition roots that already own a device.
    """

    def __init__(
        self,
        side: HandSide,
        source_id: str,
        calibration_id: str,
        transform_id: str,
        *,
        serial_number: str | None = None,
        address: str | None = None,
        device_name: str | None = None,
        device_clock_domain: str = "wuji_glove_device_clock",
        manager: _SdkManager | None = None,
        sdk_module: _WujiSdkModule | None = None,
        glove: _WujiGlove | None = None,
    ) -> None:
        if type(side) is not HandSide:
            raise ValueError("side must be a HandSide")
        self.side = side
        self.source_id = _identifier(source_id, field="source_id")
        self.calibration_id = _identifier(calibration_id, field="calibration_id")
        self.transform_id = _identifier(transform_id, field="transform_id")
        self.device_clock_domain = _identifier(
            device_clock_domain,
            field="device_clock_domain",
        )

        self.serial_number = _optional_selector(
            serial_number,
            field="serial_number",
        )
        self.address = _optional_selector(address, field="address")
        if self.serial_number is not None and self.address is not None:
            raise ValueError("serial_number and address are mutually exclusive")

        resolved_device_name = f"wuji_glove_{side.value}" if device_name is None else device_name
        if (
            type(resolved_device_name) is not str
            or _DEVICE_NAME.fullmatch(resolved_device_name) is None
        ):
            raise ValueError("device_name must contain only letters, digits, '_' or '-'")
        self.device_name = resolved_device_name

        if glove is not None and (manager is not None or sdk_module is not None):
            raise ValueError("glove injection cannot be combined with manager/sdk_module")
        self._provided_glove = glove
        self._provided_manager = manager
        self._provided_sdk_module = sdk_module

        self._manager: _SdkManager | None = None
        self._glove: _WujiGlove | None = None
        self._subscription: _HandSkeletonSubscription | None = None
        self._owns_connection = False
        self._last_sequence = -1
        self._last_receive_time_ns = -1
        self._last_device_time_ns = -1

    def start(self) -> None:
        """Connect if needed, subscribe, and begin a fresh stream epoch."""

        if self._subscription is not None:
            raise RuntimeError("Wuji Glove hand_skeleton adapter is already started")

        manager: _SdkManager | None = None
        owns_connection = False
        expected_sdk_handedness: object | None = None
        if self._provided_glove is not None:
            glove = self._provided_glove
        else:
            module = (
                _load_wuji_sdk() if self._provided_sdk_module is None else self._provided_sdk_module
            )
            manager = (
                module.SdkManager.instance()
                if self._provided_manager is None
                else self._provided_manager
            )
            selection: dict[str, object]
            if self.serial_number is not None:
                selection = {"sn": self.serial_number}
            elif self.address is not None:
                selection = {"address": self.address}
            else:
                handedness = (
                    module.Handedness.Left
                    if self.side is HandSide.LEFT
                    else module.Handedness.Right
                )
                expected_sdk_handedness = handedness
                selection = {"handedness": handedness}
            glove = manager.connect(device_name=self.device_name, **selection)
            owns_connection = True

        try:
            reported_side = glove.hand_side().get()
            if not _reported_side_matches(
                reported_side,
                self.side,
                expected_sdk_handedness=expected_sdk_handedness,
            ):
                raise RuntimeError(
                    "connected Wuji Glove side does not match configuration: "
                    f"reported={reported_side!r} configured={self.side.value!r}"
                )
            subscription = glove.hand_skeleton().subscribe()
        except Exception:
            if owns_connection and manager is not None:
                manager.disconnect(device_name=self.device_name)
            raise

        self._manager = manager
        self._glove = glove
        self._subscription = subscription
        self._owns_connection = owns_connection
        self._last_sequence = -1
        self._last_receive_time_ns = -1
        self._last_device_time_ns = -1

    def poll(
        self,
        *,
        receive_time_ns: int | None = None,
    ) -> CanonicalHandObservation:
        """Return the next available canonical skeleton without blocking."""

        subscription = self._subscription
        if subscription is None:
            raise RuntimeError("start() must succeed before poll()")
        timestamp = (
            time.monotonic_ns()
            if receive_time_ns is None
            else validate_host_time_ns(receive_time_ns)
        )
        if timestamp <= self._last_receive_time_ns:
            raise ValueError("receive_time_ns must increase strictly between successful polls")

        latest_frame = subscription.recv()
        if latest_frame is None:
            raise NoHandSkeletonFrameAvailable(
                "no Wuji Glove hand_skeleton frame is currently available"
            )
        latest_sequence, latest_device_time_ns = self._frame_order_values(
            latest_frame,
            previous_sequence=self._last_sequence,
            previous_device_time_ns=self._last_device_time_ns,
        )
        while True:
            frame = subscription.recv()
            if frame is None:
                break
            latest_sequence, latest_device_time_ns = self._frame_order_values(
                frame,
                previous_sequence=latest_sequence,
                previous_device_time_ns=latest_device_time_ns,
            )
            latest_frame = frame

        observation, device_time_ns = self._normalize_frame(latest_frame, timestamp)
        self._last_sequence = observation.sequence
        self._last_receive_time_ns = timestamp
        self._last_device_time_ns = device_time_ns
        return observation

    def close(self) -> None:
        """Close the subscription and any connection owned by this adapter."""

        subscription = self._subscription
        manager = self._manager
        owns_connection = self._owns_connection

        self._subscription = None
        self._manager = None
        self._glove = None
        self._owns_connection = False
        self._last_sequence = -1
        self._last_receive_time_ns = -1
        self._last_device_time_ns = -1

        first_error: Exception | None = None
        if subscription is not None:
            try:
                subscription.close()
            except Exception as exc:  # pragma: no cover - defensive SDK cleanup
                first_error = exc
        if owns_connection and manager is not None:
            try:
                manager.disconnect(device_name=self.device_name)
            except Exception as exc:  # pragma: no cover - defensive SDK cleanup
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error

    def __enter__(self) -> WujiGloveHandSkeletonAdapter:
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _normalize_frame(
        self,
        frame: _HandSkeleton,
        receive_time_ns: int,
    ) -> tuple[CanonicalHandObservation, int]:
        header = frame.header
        sequence, device_time_ns = self._frame_order_values(
            frame,
            previous_sequence=self._last_sequence,
            previous_device_time_ns=self._last_device_time_ns,
        )

        expected_frame_id = "l_wrist" if self.side is HandSide.LEFT else "r_wrist"
        if header.frame_id != expected_frame_id:
            raise ValueError(
                f"hand_skeleton frame_id must be {expected_frame_id!r} for side {self.side.value!r}"
            )

        try:
            sdk_joints = tuple(frame.joints)
        except TypeError as exc:
            raise ValueError("hand_skeleton joints must be an iterable") from exc
        by_name: dict[MediaPipeHandLandmark, HandLandmark] = {}
        for sdk_joint in sdk_joints:
            try:
                name = MediaPipeHandLandmark(sdk_joint.name)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"unknown MediaPipe hand landmark name {sdk_joint.name!r}"
                ) from exc
            if name in by_name:
                raise ValueError(f"duplicate MediaPipe hand landmark {name.value!r}")
            try:
                position = tuple(sdk_joint.pose.position)
            except TypeError as exc:
                raise ValueError(f"landmark {name.value!r} position must be a 3-vector") from exc
            by_name[name] = HandLandmark(
                name=name,
                position_m=cast(tuple[float, float, float], position),
                confidence=sdk_joint.confidence,
            )

        expected_names = set(MEDIAPIPE_HAND_LANDMARK_NAMES)
        actual_names = set(by_name)
        if actual_names != expected_names:
            missing = sorted(name.value for name in expected_names - actual_names)
            raise ValueError(
                "hand_skeleton must contain each canonical MediaPipe landmark "
                f"exactly once; missing={missing}"
            )
        landmarks = tuple(by_name[name] for name in MEDIAPIPE_HAND_LANDMARK_NAMES)
        return (
            CanonicalHandObservation(
                side=self.side,
                sequence=sequence,
                source_id=self.source_id,
                calibration_id=self.calibration_id,
                transform_id=self.transform_id,
                source_time_ns=None,
                receive_time_ns=receive_time_ns,
                device_time_ns=device_time_ns,
                device_clock_domain=self.device_clock_domain,
                frame_id=expected_frame_id,
                landmarks=landmarks,
            ),
            device_time_ns,
        )

    @staticmethod
    def _frame_order_values(
        frame: _HandSkeleton,
        *,
        previous_sequence: int,
        previous_device_time_ns: int,
    ) -> tuple[int, int]:
        sequence = _non_negative_int(frame.header.seq, field="header.seq")
        if sequence <= previous_sequence:
            raise ValueError("Wuji Glove header.seq must increase strictly")
        timestamp_us = _non_negative_int(
            frame.header.timestamp_us,
            field="header.timestamp_us",
        )
        device_time_ns = timestamp_us * 1_000
        if device_time_ns <= previous_device_time_ns:
            raise ValueError("Wuji Glove device timestamp must increase strictly")
        return sequence, device_time_ns


__all__ = [
    "NoHandSkeletonFrameAvailable",
    "WujiGloveHandSkeletonAdapter",
]
