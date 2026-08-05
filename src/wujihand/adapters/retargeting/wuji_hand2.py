"""Wuji SDK Hand 2 retargeting behind the canonical hand port.

Only complete, fresh, side-matched MediaPipe ``(21, 3)`` metre observations
reach ``RetargetSession.step``.  Human-model ``hand_joint_angles`` are not an
accepted input and SDK objects never escape this adapter.
"""

from __future__ import annotations

from collections.abc import Callable
import importlib
from importlib import metadata
import math
from numbers import Real
import re
import time
from types import ModuleType
from typing import Protocol, cast

import numpy as np
import numpy.typing as npt

from wujihand.domain import (
    HAND2_LAYOUT_IDS,
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    CanonicalHandObservation,
    HandIntent,
    HandSide,
    RetargetStatus,
)
from wujihand.domain.pose import validate_host_time_ns


_SDK_VERSION = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.+-]{0,63}$")


class _RetargetSession(Protocol):
    def step(
        self,
        keypoints: npt.NDArray[np.float32],
    ) -> npt.NDArray[np.float32]: ...

    def reset(self) -> None: ...


class _RetargetSessionType(Protocol):
    @staticmethod
    def for_hand(hand_model: object, side: object) -> _RetargetSession: ...


class _HandModelType(Protocol):
    WujiHand2: object


class _HandednessType(Protocol):
    Left: object
    Right: object


class _WujiRetargetModule(Protocol):
    RetargetSession: _RetargetSessionType
    HandModel: _HandModelType
    Handedness: _HandednessType


SessionFactory = Callable[[HandSide], _RetargetSession]


def _sdk_version_identifier(value: object) -> str:
    if type(value) is not str or _SDK_VERSION.fullmatch(value) is None:
        raise ValueError("sdk_version must be a bounded version identifier")
    return value


def _load_wuji_retarget_runtime(
    side: HandSide,
) -> tuple[_RetargetSession, str]:
    try:
        module: ModuleType = importlib.import_module("wuji_sdk")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Wuji Hand 2 retargeting is unavailable; install wuji-sdk and numpy on supported Linux"
        ) from exc
    sdk = cast(_WujiRetargetModule, module)

    version_value = getattr(module, "__version__", None)
    if type(version_value) is not str:
        try:
            version_value = metadata.version("wuji-sdk")
        except metadata.PackageNotFoundError as exc:
            raise RuntimeError("cannot determine the installed wuji-sdk version") from exc
    version = _sdk_version_identifier(version_value)

    handedness = sdk.Handedness.Left if side is HandSide.LEFT else sdk.Handedness.Right
    session = sdk.RetargetSession.for_hand(
        sdk.HandModel.WujiHand2,
        side=handedness,
    )
    return session, version


def _confidence(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{field} must be a finite number in [0, 1]")
    result = float(value)
    if not math.isfinite(result) or not 0.0 <= result <= 1.0:
        raise ValueError(f"{field} must be a finite number in [0, 1]")
    return result


def _positive_seconds(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{field} must be a finite positive number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{field} must be a finite positive number")
    return result


def _warmup_keypoints(side: HandSide) -> npt.NDArray[np.float32]:
    """Return a deterministic complete skeleton used only to warm the SDK."""

    indices = np.arange(len(MEDIAPIPE_HAND_LANDMARK_NAMES), dtype=np.float32)
    side_sign = -1.0 if side is HandSide.LEFT else 1.0
    return np.ascontiguousarray(
        np.column_stack(
            (
                side_sign * indices / 1_000.0,
                indices / 500.0,
                indices / 750.0,
            )
        ),
        dtype=np.float32,
    )


def _validated_sdk_q20(value: object) -> npt.NDArray[np.float64]:
    try:
        q20 = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("Wuji SDK returned a non-numeric q20 result") from exc
    if q20.shape != (20,):
        raise ValueError(
            f"Wuji SDK q20 result must have shape (20,), got {q20.shape}"
        )
    if not np.isfinite(q20).all():
        raise ValueError("Wuji SDK q20 result contains NaN or infinity")
    return q20


class WujiHand2RetargetAdapter:
    """Retarget canonical landmarks into one explicit Hand 2 q20 intent.

    Complete, finite skeletons are admitted by default even when individual
    landmark confidence is low.  Confidence remains provenance and determines
    whether the resulting intent is ``SUCCESS`` or ``DEGRADED``; callers may
    opt into a non-zero hard floor for stricter deployments.
    """

    def __init__(
        self,
        side: HandSide,
        *,
        max_observation_age_s: float = 0.25,
        minimum_landmark_confidence: float = 0.0,
        success_landmark_confidence: float = 0.6,
        session_factory: SessionFactory | None = None,
        sdk_version: str | None = None,
    ) -> None:
        if type(side) is not HandSide:
            raise ValueError("side must be a HandSide")
        self.side = side
        self.max_observation_age_ns = int(
            _positive_seconds(
                max_observation_age_s,
                field="max_observation_age_s",
            )
            * 1_000_000_000
        )
        minimum = _confidence(
            minimum_landmark_confidence,
            field="minimum_landmark_confidence",
        )
        success = _confidence(
            success_landmark_confidence,
            field="success_landmark_confidence",
        )
        if minimum > success:
            raise ValueError(
                "minimum_landmark_confidence must not exceed success_landmark_confidence"
            )
        self.minimum_landmark_confidence = minimum
        self.success_landmark_confidence = success

        if session_factory is not None and sdk_version is None:
            raise ValueError("sdk_version is required with an injected session_factory")
        self._session_factory = session_factory
        self._sdk_version = None if sdk_version is None else _sdk_version_identifier(sdk_version)
        self._session: _RetargetSession | None = None
        self._session_prepared = False
        self._closed = False
        self._source_key: tuple[str, str, str, str] | None = None
        self._last_observation_sequence = -1
        self._last_receive_time_ns = -1

    def retarget(
        self,
        observation: CanonicalHandObservation,
        *,
        sequence: int,
        produced_time_ns: int | None = None,
    ) -> HandIntent:
        """Run one validated SDK solve and preserve its complete provenance."""

        if self._closed:
            raise RuntimeError("retarget adapter is closed")
        if type(observation) is not CanonicalHandObservation:
            raise ValueError("observation must be a CanonicalHandObservation")
        if observation.side is not self.side:
            raise ValueError(f"observation side must be {self.side.value!r} for this retargeter")
        if type(sequence) is not int or sequence < 0:
            raise ValueError("sequence must be a non-negative integer")

        produced = (
            time.monotonic_ns()
            if produced_time_ns is None
            else validate_host_time_ns(produced_time_ns)
        )
        if produced < observation.receive_time_ns:
            raise ValueError("produced_time_ns must not precede observation.receive_time_ns")
        source_reference_ns = (
            observation.receive_time_ns
            if observation.source_time_ns is None
            else observation.source_time_ns
        )
        source_age_ns = produced - source_reference_ns
        if source_age_ns > self.max_observation_age_ns:
            raise ValueError(
                "canonical hand observation is stale: "
                f"age_ns={source_age_ns} limit_ns={self.max_observation_age_ns}"
            )

        source_key = (
            observation.source_id,
            observation.calibration_id,
            observation.transform_id,
            observation.frame_id,
        )
        if self._source_key is not None and source_key != self._source_key:
            raise RuntimeError(
                "hand observation source/calibration changed; call reset() "
                "before retargeting the new stream"
            )
        if observation.sequence <= self._last_observation_sequence:
            raise ValueError("observation sequence must increase strictly")
        if observation.receive_time_ns <= self._last_receive_time_ns:
            raise ValueError("observation receive_time_ns must increase strictly")

        names = tuple(landmark.name for landmark in observation.landmarks)
        if names != MEDIAPIPE_HAND_LANDMARK_NAMES:
            raise ValueError("observation must use canonical MediaPipe landmark order")
        if any(landmark.position_m is None for landmark in observation.landmarks):
            raise ValueError("all 21 canonical hand landmarks must be present")

        minimum_confidence = min(landmark.confidence for landmark in observation.landmarks)
        if minimum_confidence < self.minimum_landmark_confidence:
            raise ValueError(
                "hand landmark confidence is below the retargeting floor: "
                f"minimum={minimum_confidence:.3f} "
                f"floor={self.minimum_landmark_confidence:.3f}"
            )

        keypoints = np.ascontiguousarray(
            [
                cast(tuple[float, float, float], landmark.position_m)
                for landmark in observation.landmarks
            ],
            dtype=np.float32,
        )
        if keypoints.shape != (21, 3):
            raise ValueError(f"canonical keypoints must have shape (21, 3), got {keypoints.shape}")

        session, sdk_version = self._ensure_session()
        try:
            raw_q20 = session.step(keypoints)
        except Exception as exc:
            raise RuntimeError("Wuji SDK Hand 2 retargeting step failed") from exc
        q20 = _validated_sdk_q20(raw_q20)
        self._session_prepared = True

        status = (
            RetargetStatus.SUCCESS
            if minimum_confidence >= self.success_landmark_confidence
            else RetargetStatus.DEGRADED
        )
        intent = HandIntent(
            side=self.side,
            sequence=sequence,
            source_observation=observation,
            q20_rad=tuple(float(value) for value in q20),
            layout_id=HAND2_LAYOUT_IDS[self.side.value],
            produced_time_ns=produced,
            retarget_status=status,
            retarget_confidence=minimum_confidence,
            retarget_model_id=f"wuji_sdk.WujiHand2.{sdk_version}",
            retarget_config_id=(
                f"wuji_sdk.builtin.WujiHand2.{self.side.value}.{sdk_version}."
                f"confidence_floor_{self.minimum_landmark_confidence:.3f}."
                f"success_{self.success_landmark_confidence:.3f}"
            ),
        )
        self._source_key = source_key
        self._last_observation_sequence = observation.sequence
        self._last_receive_time_ns = observation.receive_time_ns
        return intent

    def reset(self) -> None:
        """Prepare the SDK session and admit a new source epoch."""

        if self._closed:
            raise RuntimeError("retarget adapter is closed")
        session, _ = self._ensure_session()
        if not self._session_prepared:
            try:
                warmup_q20 = session.step(_warmup_keypoints(self.side))
            except Exception as exc:
                raise RuntimeError(
                    "Wuji SDK Hand 2 retargeting warm-up failed"
                ) from exc
            _validated_sdk_q20(warmup_q20)
            self._session_prepared = True
        session.reset()
        self._source_key = None
        self._last_observation_sequence = -1
        self._last_receive_time_ns = -1

    def close(self) -> None:
        """Release an optional SDK session resource and make closure terminal."""

        if self._closed:
            return
        self._closed = True
        session = self._session
        self._session = None
        self._session_prepared = False
        self._source_key = None
        self._last_observation_sequence = -1
        self._last_receive_time_ns = -1
        if session is not None:
            close = getattr(session, "close", None)
            if callable(close):
                close()

    def __enter__(self) -> WujiHand2RetargetAdapter:
        if self._closed:
            raise RuntimeError("retarget adapter is closed")
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _ensure_session(self) -> tuple[_RetargetSession, str]:
        if self._session is not None:
            assert self._sdk_version is not None
            return self._session, self._sdk_version
        if self._session_factory is None:
            session, sdk_version = _load_wuji_retarget_runtime(self.side)
        else:
            session = self._session_factory(self.side)
            assert self._sdk_version is not None
            sdk_version = self._sdk_version
        self._session = session
        self._sdk_version = sdk_version
        return session, sdk_version


__all__ = ["SessionFactory", "WujiHand2RetargetAdapter"]
