from __future__ import annotations

import numpy as np
import pytest

from wujihand.adapters.input import NoHandSkeletonFrameAvailable
from wujihand.application.supervision import JointCommandSupervisor, SafetyState
from wujihand.application.teleoperation import (
    GloveHand2SimulationController,
    compose_q27_hand_target,
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


def _observation(
    side: HandSide,
    *,
    sequence: int,
    receive_time_ns: int,
) -> CanonicalHandObservation:
    return CanonicalHandObservation(
        side=side,
        sequence=sequence,
        source_id=f"wuji_glove.{side.value}.fixture",
        calibration_id="fixture.calibration",
        transform_id="wuji_glove.hand_skeleton.v1",
        source_time_ns=None,
        receive_time_ns=receive_time_ns,
        device_time_ns=sequence + 1,
        device_clock_domain="fixture_device_clock",
        frame_id="l_wrist" if side is HandSide.LEFT else "r_wrist",
        landmarks=tuple(
            HandLandmark(
                name=name,
                position_m=(index / 100.0, index / 200.0, index / 400.0),
                confidence=0.96,
            )
            for index, name in enumerate(MEDIAPIPE_HAND_LANDMARK_NAMES)
        ),
    )


class _Input:
    def __init__(self, side: HandSide) -> None:
        self.side = side
        self.started = False
        self.closed = False
        self.sequence = 0
        self.poll_error: Exception | None = None

    def start(self) -> None:
        self.started = True

    def poll(
        self,
        *,
        receive_time_ns: int | None = None,
    ) -> CanonicalHandObservation:
        assert self.started
        assert receive_time_ns is not None
        if self.poll_error is not None:
            raise self.poll_error
        result = _observation(
            self.side,
            sequence=self.sequence,
            receive_time_ns=receive_time_ns,
        )
        self.sequence += 1
        return result

    def close(self) -> None:
        self.closed = True
        self.started = False


class _Retargeter:
    def __init__(self, side: HandSide, q20: tuple[float, ...]) -> None:
        self.side = side
        self.q20 = q20
        self.sequences: list[int] = []
        self.reset_calls = 0
        self.close_calls = 0
        self.status = RetargetStatus.SUCCESS
        self.failure: Exception | None = None
        self.corrupt_layout = False

    def retarget(
        self,
        observation: CanonicalHandObservation,
        *,
        sequence: int,
        produced_time_ns: int | None = None,
    ) -> HandIntent:
        assert produced_time_ns is not None
        self.sequences.append(sequence)
        if self.failure is not None:
            raise self.failure
        intent = HandIntent(
            side=self.side,
            sequence=sequence,
            source_observation=observation,
            q20_rad=self.q20,
            layout_id=HAND2_LAYOUT_IDS[self.side.value],
            produced_time_ns=produced_time_ns,
            retarget_status=self.status,
            retarget_confidence=0.96,
            retarget_model_id="fixture.WujiHand2.1",
            retarget_config_id=f"fixture.WujiHand2.{self.side.value}.1",
        )
        if self.corrupt_layout:
            object.__setattr__(intent, "layout_id", "wrong.layout")
        return intent

    def reset(self) -> None:
        self.reset_calls += 1

    def close(self) -> None:
        self.close_calls += 1


def _controller(
    side: HandSide = HandSide.RIGHT,
) -> tuple[GloveHand2SimulationController, _Input, _Retargeter]:
    input_port = _Input(side)
    retargeter = _Retargeter(side, (0.2,) * 20)
    supervisor = JointCommandSupervisor(
        hand2_layout(side.value),
        (0.0,) * 20,
        stale_after_s=0.25,
        velocity_scale=0.2,
    )
    return (
        GloveHand2SimulationController(
            side,
            input_port,
            retargeter,
            supervisor,
        ),
        input_port,
        retargeter,
    )


@pytest.mark.parametrize("side", tuple(HandSide))
def test_controller_composes_ports_and_preserves_intent_sequence(
    side: HandSide,
) -> None:
    controller, input_port, retargeter = _controller(side)
    armed = controller.start(now_ns=0)
    first = controller.poll(now_ns=100_000_000)
    second = controller.poll(now_ns=200_000_000)

    assert armed.state is SafetyState.TRACKING
    assert input_port.started
    assert retargeter.reset_calls == 1
    assert retargeter.sequences == [0, 1]
    assert first.intent is not None
    assert first.intent.side is side
    assert first.decision.state is SafetyState.TRACKING
    assert second.intent is not None
    np.testing.assert_allclose(second.decision.command, 0.2)

    controller.close()
    controller.close()
    assert input_port.closed
    assert retargeter.close_calls == 1


def test_no_new_observation_holds_last_intent_then_returns_toward_rest() -> None:
    controller, _, _ = _controller()
    controller.start(now_ns=0)
    tracked = controller.poll(now_ns=100_000_000)
    held = controller.advance_without_observation(now_ns=200_000_000)
    stale = controller.advance_without_observation(now_ns=400_000_001)

    assert tracked.intent is not None
    assert held.intent is None
    assert held.decision.state is SafetyState.TRACKING
    assert stale.decision.state is SafetyState.DEGRADED
    assert stale.decision.reason == "stale_input_return_to_rest"
    assert np.all(stale.decision.command <= held.decision.command)
    controller.close()


def test_controller_rejects_wrong_side_before_retargeting() -> None:
    controller, input_port, retargeter = _controller(HandSide.RIGHT)
    input_port.side = HandSide.LEFT
    controller.start(now_ns=0)

    step = controller.poll(now_ns=100_000_000)

    assert step.intent is None
    assert step.rejection_reason == "retarget_rejected:RuntimeError"
    np.testing.assert_array_equal(step.decision.command, np.zeros(20))
    assert retargeter.sequences == []
    controller.close()


@pytest.mark.parametrize(
    "failure",
    (
        ValueError("low confidence or non-finite q20"),
        RuntimeError("SDK solve failed"),
    ),
)
def test_retarget_failure_is_rejected_without_advancing_intent_sequence(
    failure: Exception,
) -> None:
    controller, _, retargeter = _controller()
    retargeter.failure = failure
    controller.start(now_ns=0)

    rejected = controller.poll(now_ns=100_000_000)
    retargeter.failure = None
    accepted = controller.poll(now_ns=200_000_000)

    assert rejected.intent is None
    assert rejected.rejection_reason == f"retarget_rejected:{type(failure).__name__}"
    np.testing.assert_array_equal(rejected.decision.command, np.zeros(20))
    assert accepted.intent is not None
    assert accepted.intent.sequence == 0
    assert retargeter.sequences == [0, 0]
    controller.close()


def test_wrong_layout_is_rejected_before_supervision_execution_boundary() -> None:
    controller, _, retargeter = _controller()
    retargeter.corrupt_layout = True
    controller.start(now_ns=0)

    step = controller.poll(now_ns=100_000_000)

    assert step.intent is None
    assert step.rejection_reason == "retarget_rejected:RuntimeError"
    np.testing.assert_array_equal(step.decision.command, np.zeros(20))
    controller.close()


def test_degraded_intent_above_floor_is_executed_only_through_supervision() -> None:
    controller, _, retargeter = _controller()
    retargeter.status = RetargetStatus.DEGRADED
    controller.start(now_ns=0)

    step = controller.poll(now_ns=100_000_000)

    assert step.intent is not None
    assert step.intent.retarget_status is RetargetStatus.DEGRADED
    assert step.decision.state is SafetyState.TRACKING
    assert np.all(step.decision.command > 0.0)
    assert np.all(step.decision.command <= 0.2)
    controller.close()


def test_no_frame_produces_no_intent_and_uses_supervised_hold_path() -> None:
    controller, input_port, retargeter = _controller()
    controller.start(now_ns=0)
    input_port.poll_error = NoHandSkeletonFrameAvailable("empty")

    with pytest.raises(NoHandSkeletonFrameAvailable):
        controller.poll(now_ns=100_000_000)
    step = controller.advance_without_observation(now_ns=100_000_000)

    assert step.intent is None
    assert step.rejection_reason is None
    assert retargeter.sequences == []
    np.testing.assert_array_equal(step.decision.command, np.zeros(20))
    controller.close()


def test_compose_q27_replaces_only_declared_hand_partition() -> None:
    baseline = np.linspace(-0.13, 0.13, 27)
    indices = tuple(range(0, 27, 2)) + tuple(range(1, 12, 2))
    assert len(indices) == 20
    command = np.linspace(0.0, 0.19, 20)

    result = compose_q27_hand_target(baseline, indices, command)

    np.testing.assert_allclose(result[np.asarray(indices)], command)
    arm_indices = tuple(index for index in range(27) if index not in indices)
    np.testing.assert_allclose(
        result[np.asarray(arm_indices)],
        baseline[np.asarray(arm_indices)],
    )
    np.testing.assert_allclose(baseline, np.linspace(-0.13, 0.13, 27))


@pytest.mark.parametrize(
    ("baseline", "indices", "command", "message"),
    (
        (np.zeros(26), tuple(range(20)), np.zeros(20), "baseline_q27"),
        (np.zeros(27), (0,) * 20, np.zeros(20), "hand_indices_q20"),
        (np.zeros(27), tuple(range(20)), np.zeros(19), "command_q20"),
        (
            np.zeros(27),
            tuple(range(20)),
            np.full(20, np.nan),
            "command_q20",
        ),
    ),
)
def test_compose_q27_rejects_malformed_partitions(
    baseline: object,
    indices: object,
    command: object,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        compose_q27_hand_target(baseline, indices, command)
