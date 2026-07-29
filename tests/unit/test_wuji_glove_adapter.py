from __future__ import annotations

import ast
from dataclasses import dataclass
from pathlib import Path
import pytest

from wujihand.adapters.input.wuji_glove import (
    NoHandSkeletonFrameAvailable,
    WujiGloveHandSkeletonAdapter,
)
from wujihand.domain import MEDIAPIPE_HAND_LANDMARK_NAMES, HandSide
from wujihand.ports import HandObservationInputPort


ROOT = Path(__file__).parents[2]


@dataclass
class _Header:
    seq: int
    timestamp_us: int
    frame_id: str


@dataclass
class _Pose:
    position: list[float]


@dataclass
class _Joint:
    name: str
    pose: _Pose
    confidence: float


@dataclass
class _Skeleton:
    header: _Header
    joints: list[_Joint]


class _Subscription:
    def __init__(self, frames: list[_Skeleton | None]) -> None:
        self.frames = frames
        self.close_calls = 0

    def recv(self) -> _Skeleton | None:
        return None if not self.frames else self.frames.pop(0)

    def close(self) -> None:
        self.close_calls += 1


class _Resource:
    def __init__(self, subscription: _Subscription) -> None:
        self.subscription = subscription
        self.subscribe_calls = 0

    def subscribe(self) -> _Subscription:
        self.subscribe_calls += 1
        return self.subscription


class _SideResource:
    def __init__(self, side: object) -> None:
        self.side = side

    def get(self) -> object:
        return self.side


class _Glove:
    def __init__(self, subscription: _Subscription, *, side: object = "left") -> None:
        self.resource = _Resource(subscription)
        self.side_resource = _SideResource(side)

    def hand_skeleton(self) -> _Resource:
        return self.resource

    def hand_side(self) -> _SideResource:
        return self.side_resource


class _Manager:
    def __init__(self, glove: _Glove) -> None:
        self.glove = glove
        self.connect_calls: list[dict[str, object]] = []
        self.disconnect_calls: list[str] = []

    def connect(self, *, device_name: str, **selection: object) -> _Glove:
        self.connect_calls.append({"device_name": device_name, **selection})
        return self.glove

    def disconnect(self, *, device_name: str) -> None:
        self.disconnect_calls.append(device_name)


class _DualManager:
    def __init__(self, left: _Glove, right: _Glove) -> None:
        self.gloves = {
            _Handedness.Left: left,
            _Handedness.Right: right,
        }
        self.connect_calls: list[dict[str, object]] = []
        self.disconnect_calls: list[str] = []

    def connect(self, *, device_name: str, **selection: object) -> _Glove:
        self.connect_calls.append({"device_name": device_name, **selection})
        return self.gloves[selection["handedness"]]

    def disconnect(self, *, device_name: str) -> None:
        self.disconnect_calls.append(device_name)


class _Handedness:
    class _Value:
        def __init__(self, value: str) -> None:
            self.value = value

        def __str__(self) -> str:
            return self.value

    Left = _Value("left")
    Right = _Value("right")


class _UnusedManagerType:
    @staticmethod
    def instance() -> _Manager:
        raise AssertionError("the injected manager must be used")


class _SdkModule:
    SdkManager = _UnusedManagerType
    Handedness = _Handedness


def _frame(
    *,
    side: HandSide = HandSide.LEFT,
    sequence: int = 7,
    timestamp_us: int = 2_000,
    confidence: float = 0.96,
    reverse: bool = False,
) -> _Skeleton:
    joints = [
        _Joint(
            name=name.value,
            pose=_Pose([index / 100.0, index / 200.0, index / 400.0]),
            confidence=confidence,
        )
        for index, name in enumerate(MEDIAPIPE_HAND_LANDMARK_NAMES)
    ]
    if reverse:
        joints.reverse()
    return _Skeleton(
        header=_Header(
            seq=sequence,
            timestamp_us=timestamp_us,
            frame_id="l_wrist" if side is HandSide.LEFT else "r_wrist",
        ),
        joints=joints,
    )


def _adapter(
    manager: _Manager,
    *,
    side: HandSide = HandSide.LEFT,
) -> WujiGloveHandSkeletonAdapter:
    return WujiGloveHandSkeletonAdapter(
        side,
        f"wuji_glove.{side.value}.SN_TEST",
        "wuji_sdk.default_user.builtin",
        "wuji_glove.hand_skeleton.v1",
        manager=manager,  # type: ignore[arg-type]
        sdk_module=_SdkModule(),  # type: ignore[arg-type]
    )


def test_wuji_sdk_is_not_imported_at_adapter_module_import_time() -> None:
    paths = (
        ROOT / "src/wujihand/adapters/input/wuji_glove.py",
        ROOT / "src/wujihand/adapters/retargeting/wuji_hand2.py",
    )
    for path in paths:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        imports = {
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        }
        imports.update(
            node.module or "" for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
        )
        assert "wuji_sdk" not in imports


def test_adapter_owns_side_selected_connection_and_normalizes_named_frame() -> None:
    subscription = _Subscription([_frame(reverse=True), None])
    glove = _Glove(subscription)
    manager = _Manager(glove)
    adapter = _adapter(manager)

    assert isinstance(adapter, HandObservationInputPort)
    adapter.start()
    observation = adapter.poll(receive_time_ns=3_000_000)

    assert manager.connect_calls == [
        {"device_name": "wuji_glove_left", "handedness": _Handedness.Left}
    ]
    assert glove.resource.subscribe_calls == 1
    assert observation.side is HandSide.LEFT
    assert observation.sequence == 7
    assert observation.source_time_ns is None
    assert observation.receive_time_ns == 3_000_000
    assert observation.device_time_ns == 2_000_000
    assert observation.device_clock_domain == "wuji_glove_device_clock"
    assert observation.frame_id == "l_wrist"
    assert tuple(item.name for item in observation.landmarks) == (MEDIAPIPE_HAND_LANDMARK_NAMES)
    assert observation.landmarks[8].position_m == pytest.approx((0.08, 0.04, 0.02))
    assert observation.landmarks[8].confidence == pytest.approx(0.96)

    adapter.close()
    adapter.close()
    assert subscription.close_calls == 1
    assert manager.disconnect_calls == ["wuji_glove_left"]


def test_two_adapters_share_one_manager_with_distinct_device_names() -> None:
    left_subscription = _Subscription(
        [_frame(side=HandSide.LEFT), None]
    )
    right_subscription = _Subscription(
        [_frame(side=HandSide.RIGHT), None]
    )
    manager = _DualManager(
        _Glove(left_subscription, side=_Handedness.Left),
        _Glove(right_subscription, side=_Handedness.Right),
    )
    left = _adapter(manager, side=HandSide.LEFT)  # type: ignore[arg-type]
    right = _adapter(manager, side=HandSide.RIGHT)  # type: ignore[arg-type]

    left.start()
    right.start()
    left_observation = left.poll(receive_time_ns=3_000_000)
    right_observation = right.poll(receive_time_ns=3_000_000)
    right.close()
    left.close()

    assert left_observation.side is HandSide.LEFT
    assert right_observation.side is HandSide.RIGHT
    assert manager.connect_calls == [
        {
            "device_name": "wuji_glove_left",
            "handedness": _Handedness.Left,
        },
        {
            "device_name": "wuji_glove_right",
            "handedness": _Handedness.Right,
        },
    ]
    assert manager.disconnect_calls == [
        "wuji_glove_right",
        "wuji_glove_left",
    ]


def test_official_handedness_object_is_not_mistaken_for_a_wrong_side() -> None:
    assert str(_Handedness.Left) == "left"
    subscription = _Subscription([_frame(), None])
    manager = _Manager(_Glove(subscription, side=_Handedness.Left))
    adapter = _adapter(manager)

    adapter.start()
    observation = adapter.poll(receive_time_ns=3_000_000)

    assert observation.side is HandSide.LEFT
    adapter.close()


def test_nonblocking_empty_poll_does_not_consume_receive_timestamp() -> None:
    subscription = _Subscription([None, _frame(), None])
    manager = _Manager(_Glove(subscription))
    adapter = _adapter(manager)
    adapter.start()

    with pytest.raises(NoHandSkeletonFrameAvailable, match="no Wuji Glove"):
        adapter.poll(receive_time_ns=3_000_000)
    observation = adapter.poll(receive_time_ns=3_000_000)

    assert observation.sequence == 7
    adapter.close()


def test_poll_drains_backlog_and_returns_only_the_latest_monotonic_frame() -> None:
    subscription = _Subscription(
        [
            _frame(sequence=7, timestamp_us=2_000),
            _frame(sequence=9, timestamp_us=2_010),
            _frame(sequence=15, timestamp_us=2_020),
            None,
        ]
    )
    manager = _Manager(_Glove(subscription))
    adapter = _adapter(manager)
    adapter.start()

    observation = adapter.poll(receive_time_ns=3_000_000)

    assert observation.sequence == 15
    assert observation.device_time_ns == 2_020_000
    adapter.close()


def test_adapter_rejects_wrong_side_missing_duplicate_and_stale_frames() -> None:
    wrong_side = _frame(side=HandSide.RIGHT)
    missing = _frame()
    missing.joints.pop()
    duplicate = _frame()
    duplicate.joints[-1] = duplicate.joints[0]

    for invalid, message in (
        (wrong_side, "frame_id"),
        (missing, "missing="),
        (duplicate, "duplicate"),
    ):
        subscription = _Subscription([invalid, None])
        adapter = _adapter(_Manager(_Glove(subscription)))
        adapter.start()
        with pytest.raises(ValueError, match=message):
            adapter.poll(receive_time_ns=3_000_000)
        adapter.close()

    subscription = _Subscription(
        [
            _frame(sequence=8, timestamp_us=2_001),
            None,
            _frame(sequence=8, timestamp_us=2_002),
            None,
        ]
    )
    adapter = _adapter(_Manager(_Glove(subscription)))
    adapter.start()
    adapter.poll(receive_time_ns=3_000_000)
    with pytest.raises(ValueError, match="header.seq"):
        adapter.poll(receive_time_ns=3_000_001)
    adapter.close()


def test_connected_glove_side_is_checked_even_for_explicit_serial_selection() -> None:
    subscription = _Subscription([_frame(side=HandSide.RIGHT), None])
    manager = _Manager(_Glove(subscription, side="right"))
    adapter = WujiGloveHandSkeletonAdapter(
        HandSide.LEFT,
        "wuji_glove.left.expected",
        "fixture.calibration",
        "fixture.transform",
        serial_number="WG1JA00TEST",
        manager=manager,  # type: ignore[arg-type]
        sdk_module=_SdkModule(),  # type: ignore[arg-type]
    )

    with pytest.raises(RuntimeError, match="does not match"):
        adapter.start()

    assert manager.connect_calls == [{"device_name": "wuji_glove_left", "sn": "WG1JA00TEST"}]
    assert manager.disconnect_calls == ["wuji_glove_left"]
    assert subscription.close_calls == 0


def test_injected_glove_is_not_disconnected_and_start_is_explicit() -> None:
    subscription = _Subscription([_frame(), None])
    glove = _Glove(subscription)
    adapter = WujiGloveHandSkeletonAdapter(
        HandSide.LEFT,
        "wuji_glove.left.fixture",
        "fixture.calibration",
        "fixture.transform",
        glove=glove,  # type: ignore[arg-type]
    )

    with pytest.raises(RuntimeError, match=r"start\(\)"):
        adapter.poll(receive_time_ns=3_000_000)
    adapter.start()
    with pytest.raises(RuntimeError, match="already started"):
        adapter.start()
    adapter.poll(receive_time_ns=3_000_000)
    adapter.close()

    assert subscription.close_calls == 1
