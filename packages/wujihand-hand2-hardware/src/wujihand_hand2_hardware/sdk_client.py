from __future__ import annotations

import importlib
import importlib.metadata
from collections.abc import Iterator
from contextlib import contextmanager
from types import ModuleType
from typing import Any, Protocol

from .types import (
    CommunicationNode,
    CommunicationSnapshot,
    DeviceIdentity,
    DeviceTarget,
    DiagnosticsFrame,
    FingerCommunication,
    FrameHeader,
    JointDiagnostics,
    JointState,
    Side,
    StateFrame,
    TransportDiagnostics,
)


class ReadOnlySession(Protocol):
    def identity(self) -> DeviceIdentity: ...

    def joint_labels(self) -> tuple[str, ...]: ...

    def poll_state(self) -> StateFrame | None: ...

    def poll_diagnostics(self) -> DiagnosticsFrame | None: ...

    def communication(self) -> CommunicationSnapshot: ...

    def describe_error(self, code: int) -> str | None: ...

    def close(self) -> None: ...


class ReadOnlyClient(Protocol):
    @contextmanager
    def open(self, target: DeviceTarget) -> Iterator[ReadOnlySession]: ...


def _enum_text(value: object) -> str:
    name = getattr(value, "name", None)
    return str(name if name is not None else value).rsplit(".", maxsplit=1)[-1]


class WujiSdkReadOnlyClient:
    """Narrow adapter over the Wuji SDK 8.3 read-only resource surface."""

    @contextmanager
    def open(self, target: DeviceTarget) -> Iterator[ReadOnlySession]:
        session = _WujiSdkReadOnlySession(target)
        try:
            yield session
        finally:
            session.close()


class _WujiSdkReadOnlySession:
    def __init__(self, target: DeviceTarget) -> None:
        self._target = target
        self._sdk: ModuleType = importlib.import_module("wuji_sdk")
        self._manager: Any = self._sdk.SdkManager.instance()
        self._hand: Any = None
        self._state_subscription: Any = None
        self._diagnostics_subscription: Any = None
        self._discovered_address = ""
        self._device_type = ""
        try:
            self._discover()
            options = self._sdk.ConnectOptions(
                timeout_ms=1000,
                retry_count=3,
                enable_bridge=False,
                auto_time_sync_interval_ms=None,
            )
            self._hand = self._manager.connect(
                sn=target.serial,
                device_name="wujihand_hand2_hardware_readonly",
                options=options,
            )
            if type(self._hand).__name__ != "WujiHand2":
                raise RuntimeError(f"unexpected SDK handle type: {type(self._hand).__name__}")
            self._state_subscription = self._hand.joint_states().subscribe()
            self._diagnostics_subscription = self._hand.joint_diagnostics().subscribe()
        except Exception:
            self.close()
            raise

    def _discover(self) -> None:
        matches = [
            device for device in self._manager.scan() if str(device.sn) == self._target.serial
        ]
        if len(matches) != 1:
            raise RuntimeError(
                f"expected one discovered device for {self._target.serial}, got {len(matches)}"
            )
        device = matches[0]
        self._discovered_address = str(device.address)
        self._device_type = _enum_text(device.device_type)
        if self._discovered_address != self._target.address:
            raise RuntimeError(
                "discovered address mismatch: "
                f"expected={self._target.address}, observed={self._discovered_address}"
            )
        if self._device_type != "WujiHand2":
            raise RuntimeError(f"unexpected discovered device type: {self._device_type}")

    def identity(self) -> DeviceIdentity:
        info = self._hand.info
        hardware = self._hand.hw_version().get()
        return DeviceIdentity(
            serial=str(self._hand.serial_number),
            address=self._discovered_address,
            side=Side(str(self._hand.handedness().get()).lower()),
            firmware=str(info.firmware_version),
            hardware=f"{hardware.major}.{hardware.minor}.{hardware.patch}",
            sdk=importlib.metadata.version("wuji-sdk"),
            online_joints=int(self._hand.online_joints_count().get()),
            device_type=self._device_type,
        )

    def joint_labels(self) -> tuple[str, ...]:
        return tuple(str(joint.label) for joint in self._hand.joints())

    def poll_state(self) -> StateFrame | None:
        frame = self._state_subscription.recv()
        if frame is None:
            return None
        return StateFrame(
            header=FrameHeader(int(frame.header.seq), int(frame.header.timestamp_us)),
            joints=tuple(
                JointState(
                    nid=int(joint.nid),
                    position_rad=float(joint.position),
                    velocity_rad_s=float(joint.velocity),
                    effort_a=float(joint.effort),
                )
                for joint in frame.joints
            ),
        )

    def poll_diagnostics(self) -> DiagnosticsFrame | None:
        frame = self._diagnostics_subscription.recv()
        if frame is None:
            return None
        comm = frame.comm
        return DiagnosticsFrame(
            header=FrameHeader(int(frame.header.seq), int(frame.header.timestamp_us)),
            joints=tuple(
                JointDiagnostics(
                    nid=int(joint.nid),
                    status=str(joint.status_word.ext_state_name),
                    current_a=float(joint.current),
                    bus_voltage_v=float(joint.vbus_v_fb),
                    mcu_temperature_c=float(joint.mcu_temp_c_fb),
                    error_code=int(joint.error_code_current),
                    response_rate_pct=float(joint.comm_response_rate_pct),
                    timeout_total=int(joint.comm_timeout_total),
                    position_limit_active=bool(joint.status_word.position_limit_active),
                    velocity_limit_active=bool(joint.status_word.velocity_limit_active),
                    current_limit_active=bool(joint.status_word.current_limit_active),
                )
                for joint in frame.joints
            ),
            transport=TransportDiagnostics(
                age_ms=int(comm.age_ms),
                e2e_received=int(comm.e2e_received),
                e2e_lost=int(comm.e2e_lost),
                e2e_reordered=int(comm.e2e_reordered),
                e2e_duplicates=int(comm.e2e_duplicates),
                e2e_window_loss_x100=int(comm.e2e_window_loss_x100),
                rpc_total=int(comm.rpc_total),
                rpc_retries=int(comm.rpc_retries),
                rpc_timeouts=int(comm.rpc_timeouts),
                comm_get_failures=int(comm.comm_get_failures),
                sdk_dropped=int(comm.sdk_dropped),
            ),
        )

    def communication(self) -> CommunicationSnapshot:
        fingers: list[FingerCommunication] = []
        for finger_index, finger in enumerate(self._hand.comm_diag().get().fingers):
            nodes = tuple(
                CommunicationNode(
                    slot=slot,
                    node_type=int(node.node_type),
                    online=bool(node.online),
                    request_total=int(node.request_total),
                    response_total=int(node.response_ok_total),
                    timeout_total=int(node.timeout_total),
                    response_rate_pct=float(node.response_rate_pct),
                    age_ms=int(node.ms_since_last_response),
                )
                for slot, node in enumerate(finger.nodes)
            )
            fingers.append(
                FingerCommunication(
                    finger_index=finger_index,
                    crc_errors=int(finger.crc_error_total),
                    format_errors=int(finger.frame_format_error_total),
                    uart_errors=int(finger.uart_hw_error_total),
                    error_per_second=float(finger.error_per_sec),
                    nodes=nodes,
                )
            )
        return CommunicationSnapshot(tuple(fingers))

    def describe_error(self, code: int) -> str | None:
        description = self._sdk.WujiHand2.describe_error(code)
        if description is None:
            return None
        return str(description.get("name"))

    def _connected_hand(self) -> Any:
        if self._hand is None:
            raise RuntimeError("Wuji Hand2 session is disconnected")
        return self._hand

    def _sdk_module(self) -> ModuleType:
        return self._sdk

    def close(self) -> None:
        if self._state_subscription is not None:
            self._state_subscription.close()
            self._state_subscription = None
        if self._diagnostics_subscription is not None:
            self._diagnostics_subscription.close()
            self._diagnostics_subscription = None
        if self._hand is not None:
            self._hand.disconnect()
            self._hand = None
