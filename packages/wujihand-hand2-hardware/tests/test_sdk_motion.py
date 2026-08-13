from types import SimpleNamespace

from wujihand_hand2_hardware.sdk_motion import WujiSdkMotionSession
from wujihand_hand2_hardware.types import JointCommandValue


class _Resource:
    def __init__(self, value: object) -> None:
        self.value = value

    def get(self) -> object:
        return self.value


class _Publisher:
    def __init__(self) -> None:
        self.frames: list[list[object]] = []
        self.closed = False

    def send(self, frame: list[object]) -> None:
        self.frames.append(frame)

    def close(self) -> None:
        self.closed = True


class _CommandResource:
    def __init__(self, publisher: _Publisher) -> None:
        self.publisher = publisher

    def publish(self) -> _Publisher:
        return self.publisher


class _Hand:
    def __init__(self) -> None:
        self.publisher = _Publisher()
        self.calls: list[tuple[str, object]] = []

    def effort_limit(self) -> _Resource:
        return _Resource([1.0] * 20)

    def mit_params(self) -> _Resource:
        return _Resource([SimpleNamespace(kp=1.0, kd=0.05)] * 20)

    def joint_command(self) -> _CommandResource:
        return _CommandResource(self.publisher)

    def enable(self, *, joints: list[int]) -> None:
        self.calls.append(("enable", joints))

    def disable(self, *, joints: list[int] | None = None) -> None:
        self.calls.append(("disable", joints))

    def emergency_stop(self) -> None:
        self.calls.append(("emergency_stop", None))


class _JointCommand:
    def __init__(self, position: float, velocity: float, effort: float) -> None:
        self.position = position
        self.velocity = velocity
        self.effort = effort


def session() -> tuple[WujiSdkMotionSession, _Hand]:
    value = object.__new__(WujiSdkMotionSession)
    hand = _Hand()
    value._hand = hand
    value._sdk = SimpleNamespace(JointCommand=_JointCommand)
    value._command_publisher = None
    value._may_be_enabled = False
    return value, hand


def test_sdk_motion_adapter_reads_existing_parameters_and_sends_q20() -> None:
    value, hand = session()
    readback = value.control_readback()
    assert readback.effort_limits_a == (1.0,) * 20
    assert readback.mit_parameters[4] is not None
    assert readback.mit_parameters[4].kp == 1.0

    value.open_command_stream()
    value.send_command(tuple(JointCommandValue(0.01 * index) for index in range(20)))
    assert len(hand.publisher.frames) == 1
    assert len(hand.publisher.frames[0]) == 20
    assert hand.publisher.frames[0][4].position == 0.04


def test_sdk_motion_adapter_masks_enable_and_escalation_actions() -> None:
    value, hand = session()
    mask = tuple(1 if index == 4 else 0 for index in range(20))
    value.enable_selected(mask)
    value.disable_selected(mask)
    value.emergency_stop_all()

    assert hand.calls == [
        ("enable", list(mask)),
        ("disable", list(mask)),
        ("emergency_stop", None),
    ]
