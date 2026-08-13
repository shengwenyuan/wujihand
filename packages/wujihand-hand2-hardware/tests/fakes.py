from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import replace

from wujihand_hand2_hardware.mapping import Q20_LABELS, Q20_NIDS
from wujihand_hand2_hardware.sdk_client import ReadOnlySession
from wujihand_hand2_hardware.types import (
    CommunicationNode,
    CommunicationSnapshot,
    ControlReadback,
    DeviceIdentity,
    DeviceTarget,
    DiagnosticsFrame,
    FingerCommunication,
    FrameHeader,
    JointCommandValue,
    JointDiagnostics,
    JointState,
    MitParameters,
    Side,
    StateFrame,
    TransportDiagnostics,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeSession:
    def __init__(
        self,
        *,
        error_code: int = 0,
        missing_nid: int | None = None,
        sequence_step: int = 1,
        timeout_delta: int = 0,
        temperature_step_c: float = 0.0,
        initial_response_rate_zero_frames: int = 0,
        initial_transport_uninitialized_frames: int = 0,
        response_rate_pct: float = 100.0,
    ) -> None:
        self._error_code = error_code
        self._nids = tuple(nid for nid in Q20_NIDS if nid != missing_nid)
        self._sequence_step = sequence_step
        self._timeout_delta = timeout_delta
        self._temperature_step_c = temperature_step_c
        self._initial_response_rate_zero_frames = initial_response_rate_zero_frames
        self._initial_transport_uninitialized_frames = initial_transport_uninitialized_frames
        self._response_rate_pct = response_rate_pct
        self._state_sequence = 0
        self._diagnostics_sequence = 0
        self._communication_calls = 0
        self.closed = False

    def identity(self) -> DeviceIdentity:
        return DeviceIdentity(
            serial="TEST-RIGHT",
            address="192.0.2.11:7447",
            side=Side.RIGHT,
            firmware="2.2.3",
            hardware="0.2.0",
            sdk="2026.8.3",
            online_joints=20,
            device_type="WujiHand2",
        )

    def joint_labels(self) -> tuple[str, ...]:
        return Q20_LABELS

    def poll_state(self) -> StateFrame:
        self._state_sequence += self._sequence_step
        return StateFrame(
            FrameHeader(self._state_sequence, self._state_sequence * 1000),
            tuple(JointState(nid, 0.0, 0.0, 0.0) for nid in self._nids),
        )

    def poll_diagnostics(self) -> DiagnosticsFrame:
        self._diagnostics_sequence += self._sequence_step
        temperature = 50.0 + self._diagnostics_sequence * self._temperature_step_c
        response_rate = (
            0.0
            if self._diagnostics_sequence <= self._initial_response_rate_zero_frames
            else self._response_rate_pct
        )
        joints = tuple(
            JointDiagnostics(
                nid=nid,
                status="Ready",
                current_a=0.0,
                bus_voltage_v=12.3,
                mcu_temperature_c=temperature,
                error_code=self._error_code,
                response_rate_pct=response_rate,
                timeout_total=0,
                position_limit_active=False,
                velocity_limit_active=False,
                current_limit_active=False,
            )
            for nid in self._nids
        )
        return DiagnosticsFrame(
            FrameHeader(self._diagnostics_sequence, self._diagnostics_sequence * 1000),
            joints,
            TransportDiagnostics(
                age_ms=(
                    65535
                    if self._diagnostics_sequence <= self._initial_transport_uninitialized_frames
                    else self._diagnostics_sequence % 1000
                ),
                e2e_received=100,
                e2e_lost=0,
                e2e_reordered=0,
                e2e_duplicates=0,
                e2e_window_loss_x100=0,
                rpc_total=1,
                rpc_retries=0,
                rpc_timeouts=0,
                comm_get_failures=0,
                sdk_dropped=0,
            ),
        )

    def communication(self) -> CommunicationSnapshot:
        self._communication_calls += 1
        total = (self._communication_calls - 1) * 10
        fingers = []
        for finger in range(5):
            nodes = []
            for slot in range(5):
                motor = slot < 4
                nodes.append(
                    CommunicationNode(
                        slot=slot,
                        node_type=0 if motor else 1,
                        online=motor,
                        request_total=total,
                        response_total=total if motor else 0,
                        timeout_total=self._timeout_delta
                        if total and motor and finger == 0 and slot == 0
                        else 0,
                        response_rate_pct=100.0 if motor else 0.0,
                        age_ms=1 if motor else 2**32 - 1,
                    )
                )
            fingers.append(FingerCommunication(finger, 0, 0, 0, 0.0, tuple(nodes)))
        return CommunicationSnapshot(tuple(fingers))

    def describe_error(self, code: int) -> str | None:
        return "BusFrameLossHigh" if code == 6 else None

    def close(self) -> None:
        self.closed = True


class FakeClient:
    def __init__(self, session: FakeSession) -> None:
        self.session = session

    @contextmanager
    def open(self, target: DeviceTarget) -> Iterator[ReadOnlySession]:
        del target
        try:
            yield self.session
        finally:
            self.session.close()


class FakeMotionSession(FakeSession):
    def __init__(
        self,
        *,
        stale_state_after_enable: bool = False,
        preflight_response_rate_pct: float = 100.0,
        response_rate_after_enable_pct: float = 100.0,
        temperature_c: float = 50.0,
        send_failure_at: int | None = None,
        interrupt_at: int | None = None,
        disable_fails: bool = False,
    ) -> None:
        super().__init__(response_rate_pct=preflight_response_rate_pct)
        self.positions = [0.0] * 20
        self.enabled_mask = [0] * 20
        self.commands: list[tuple[JointCommandValue, ...]] = []
        self.call_order: list[str] = []
        self.enabled_masks: list[tuple[int, ...]] = []
        self.emergency_stopped = False
        self.command_stream_open = False
        self._stale_state_after_enable = stale_state_after_enable
        self._response_rate_after_enable_pct = response_rate_after_enable_pct
        self._temperature_c = temperature_c
        self._send_failure_at = send_failure_at
        self._interrupt_at = interrupt_at
        self._disable_fails = disable_fails

    def poll_state(self) -> StateFrame | None:
        if self._stale_state_after_enable and any(self.enabled_mask):
            return None
        frame = super().poll_state()
        joints = tuple(
            replace(joint, position_rad=self.positions[index])
            for index, joint in enumerate(frame.joints)
        )
        return replace(frame, joints=joints)

    def poll_diagnostics(self) -> DiagnosticsFrame:
        frame = super().poll_diagnostics()
        joints = tuple(
            replace(
                joint,
                status="Enabled" if self.enabled_mask[index] else "Ready",
                response_rate_pct=(
                    self._response_rate_after_enable_pct
                    if any(self.enabled_mask)
                    else joint.response_rate_pct
                ),
                mcu_temperature_c=self._temperature_c,
            )
            for index, joint in enumerate(frame.joints)
        )
        return replace(frame, joints=joints)

    def control_readback(self) -> ControlReadback:
        return ControlReadback(
            effort_limits_a=(1.0,) * 20,
            mit_parameters=(MitParameters(1.0, 0.05),) * 20,
        )

    def open_command_stream(self) -> None:
        self.call_order.append("open_command_stream")
        self.command_stream_open = True

    def send_command(self, command: tuple[JointCommandValue, ...]) -> None:
        self.call_order.append("send_command")
        if not self.command_stream_open:
            raise RuntimeError("command stream closed")
        if len(command) != 20:
            raise ValueError("expected 20 commands")
        if self._send_failure_at is not None and len(self.commands) + 1 == self._send_failure_at:
            raise RuntimeError("synthetic send failure")
        if self._interrupt_at is not None and len(self.commands) + 1 == self._interrupt_at:
            raise KeyboardInterrupt
        self.commands.append(command)
        for index, enabled in enumerate(self.enabled_mask):
            if enabled:
                self.positions[index] = command[index].position_rad

    def enable_selected(self, mask: tuple[int, ...]) -> None:
        self.call_order.append("enable_selected")
        self.enabled_masks.append(mask)
        self.enabled_mask = list(mask)

    def disable_selected(self, mask: tuple[int, ...] | None = None) -> None:
        self.call_order.append("disable_selected_all" if mask is None else "disable_selected")
        if self._disable_fails:
            raise RuntimeError("synthetic disable failure")
        if mask is None:
            self.enabled_mask = [0] * 20
        else:
            for index, selected in enumerate(mask):
                if selected:
                    self.enabled_mask[index] = 0

    def emergency_stop_all(self) -> None:
        self.call_order.append("emergency_stop_all")
        self.emergency_stopped = True
        self.enabled_mask = [0] * 20

    def close_command_stream(self) -> None:
        self.call_order.append("close_command_stream")
        self.command_stream_open = False


class FakeMotionClient:
    def __init__(self, session: FakeMotionSession) -> None:
        self.session = session

    @contextmanager
    def open(self, target: DeviceTarget) -> Iterator[FakeMotionSession]:
        del target
        self.session.call_order.append("client_open")
        try:
            yield self.session
        finally:
            self.session.close()


def target() -> DeviceTarget:
    return DeviceTarget(
        serial="TEST-RIGHT",
        address="192.0.2.11:7447",
        side=Side.RIGHT,
        expected_firmware="2.2.3",
        expected_hardware="0.2.0",
    )
