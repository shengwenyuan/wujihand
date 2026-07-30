from __future__ import annotations

import numpy as np
import pytest

from wujihand.application.supervision import JointCommandSupervisor
from wujihand.application.teleoperation import (
    GloveHand2ControllerSet,
    GloveHand2SimulationController,
)
from wujihand.domain import (
    HAND2_LAYOUT_IDS,
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    CanonicalHandObservation,
    HandIntent,
    HandLandmark,
    HandSide,
    RetargetStatus,
    hand2_layout,
)
from wujihand.ports import NoHandObservationAvailable


def observation(
    side: HandSide,
    *,
    sequence: int,
    receive_time_ns: int,
) -> CanonicalHandObservation:
    return CanonicalHandObservation(
        side=side,
        sequence=sequence,
        source_id=f"wuji_glove.{side.value}.fixture",
        calibration_id=f"glove_{side.value}_fixture_v1",
        transform_id="wuji_glove.hand_skeleton.v1",
        source_time_ns=None,
        receive_time_ns=receive_time_ns,
        device_time_ns=sequence,
        device_clock_domain="fixture_device_clock",
        frame_id="l_wrist" if side is HandSide.LEFT else "r_wrist",
        landmarks=tuple(
            HandLandmark(
                name=name,
                position_m=(
                    index / 100.0,
                    index / 200.0,
                    index / 400.0,
                ),
                confidence=0.96,
            )
            for index, name in enumerate(
                MEDIAPIPE_HAND_LANDMARK_NAMES
            )
        ),
    )


class FakeInput:
    def __init__(
        self,
        side: HandSide,
        events: list[str],
        *,
        start_error: Exception | None = None,
    ) -> None:
        self.side = side
        self.events = events
        self.start_error = start_error
        self.empty = False
        self.poll_error: Exception | None = None
        self.sequence = 0

    def start(self) -> None:
        self.events.append(f"start:{self.side.value}")
        if self.start_error is not None:
            raise self.start_error

    def poll(
        self,
        *,
        receive_time_ns: int | None = None,
    ) -> CanonicalHandObservation:
        assert receive_time_ns is not None
        if self.poll_error is not None:
            raise self.poll_error
        if self.empty:
            raise NoHandObservationAvailable(self.side.value)
        result = observation(
            self.side,
            sequence=self.sequence,
            receive_time_ns=receive_time_ns,
        )
        self.sequence += 1
        return result

    def close(self) -> None:
        self.events.append(f"close:{self.side.value}")


class FakeRetargeter:
    def __init__(self, side: HandSide) -> None:
        self.side = side

    def retarget(
        self,
        source: CanonicalHandObservation,
        *,
        sequence: int,
        produced_time_ns: int | None = None,
    ) -> HandIntent:
        assert produced_time_ns is not None
        return HandIntent(
            side=self.side,
            sequence=sequence,
            source_observation=source,
            q20_rad=(0.2,) * 20,
            layout_id=HAND2_LAYOUT_IDS[self.side.value],
            produced_time_ns=produced_time_ns,
            retarget_status=RetargetStatus.SUCCESS,
            retarget_confidence=0.96,
            retarget_model_id="fixture.WujiHand2.1",
            retarget_config_id=(
                f"fixture.WujiHand2.{self.side.value}.1"
            ),
        )

    def reset(self) -> None:
        return None


def hand_controller(
    side: HandSide,
    events: list[str],
    *,
    start_error: Exception | None = None,
) -> tuple[GloveHand2SimulationController, FakeInput]:
    source = FakeInput(side, events, start_error=start_error)
    controller = GloveHand2SimulationController(
        side,
        source,
        FakeRetargeter(side),
        JointCommandSupervisor(
            hand2_layout(side.value),
            (0.0,) * 20,
            stale_after_s=0.25,
            velocity_scale=0.2,
        ),
    )
    return controller, source


def test_empty_set_has_an_explicit_noop_lifecycle() -> None:
    subject = GloveHand2ControllerSet({})

    assert subject.sides == ()
    with pytest.raises(RuntimeError, match=r"start\(\)"):
        subject.step(now_ns=0)

    subject.start(now_ns=0)
    assert subject.step(now_ns=1) == ()
    with pytest.raises(RuntimeError, match="already started"):
        subject.start(now_ns=2)

    subject.close()
    subject.close()


def test_dual_set_starts_left_then_right_and_closes_in_reverse() -> None:
    events: list[str] = []
    left, _ = hand_controller(HandSide.LEFT, events)
    right, _ = hand_controller(HandSide.RIGHT, events)
    subject = GloveHand2ControllerSet(
        {
            HandSide.RIGHT: right,
            HandSide.LEFT: left,
        }
    )

    assert subject.sides == (HandSide.LEFT, HandSide.RIGHT)
    subject.start(now_ns=0)
    decisions = subject.step(now_ns=100_000_000)
    subject.close()
    subject.close()

    assert tuple(decision.side for decision in decisions) == (
        HandSide.LEFT,
        HandSide.RIGHT,
    )
    assert all(decision.step.intent is not None for decision in decisions)
    assert events == [
        "start:left",
        "start:right",
        "close:right",
        "close:left",
    ]


def test_missing_left_frame_does_not_suppress_right_intent() -> None:
    events: list[str] = []
    left, left_input = hand_controller(HandSide.LEFT, events)
    right, _ = hand_controller(HandSide.RIGHT, events)
    subject = GloveHand2ControllerSet(
        {
            HandSide.LEFT: left,
            HandSide.RIGHT: right,
        }
    )
    subject.start(now_ns=0)
    left_input.empty = True

    left_step, right_step = subject.step(now_ns=100_000_000)

    assert left_step.step.intent is None
    np.testing.assert_array_equal(
        left_step.step.decision.command,
        np.zeros(20),
    )
    assert right_step.step.intent is not None
    assert np.all(right_step.step.decision.command > 0.0)
    subject.close()


def test_invalid_left_frame_is_rejected_without_suppressing_right() -> None:
    events: list[str] = []
    left, left_input = hand_controller(HandSide.LEFT, events)
    right, _ = hand_controller(HandSide.RIGHT, events)
    subject = GloveHand2ControllerSet(
        {
            HandSide.LEFT: left,
            HandSide.RIGHT: right,
        }
    )
    subject.start(now_ns=0)
    left_input.poll_error = ValueError("malformed skeleton")

    left_step, right_step = subject.step(now_ns=100_000_000)

    assert left_step.step.intent is None
    assert left_step.step.rejection_reason == "input_rejected:ValueError"
    assert right_step.step.intent is not None
    subject.close()


def test_second_start_failure_rolls_back_first_controller() -> None:
    events: list[str] = []
    left, _ = hand_controller(HandSide.LEFT, events)
    right, _ = hand_controller(
        HandSide.RIGHT,
        events,
        start_error=RuntimeError("right connect failed"),
    )
    subject = GloveHand2ControllerSet(
        {
            HandSide.LEFT: left,
            HandSide.RIGHT: right,
        }
    )

    with pytest.raises(RuntimeError, match="right connect failed"):
        subject.start(now_ns=0)

    assert events == ["start:left", "start:right", "close:left"]


def test_set_rejects_side_mismatch_and_requires_start() -> None:
    events: list[str] = []
    left, _ = hand_controller(HandSide.LEFT, events)
    with pytest.raises(ValueError, match="key and configured side"):
        GloveHand2ControllerSet({HandSide.RIGHT: left})

    subject = GloveHand2ControllerSet({HandSide.LEFT: left})
    with pytest.raises(RuntimeError, match=r"start\(\)"):
        subject.step(now_ns=1)
