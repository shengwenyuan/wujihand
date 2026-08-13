from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from typing import Any

from .sdk_client import _WujiSdkReadOnlySession
from .types import (
    ControlReadback,
    DeviceTarget,
    JointCommandValue,
    MitParameters,
)


class WujiSdkMotionClient:
    """SDK 8.3 write adapter; the executor is its only package caller."""

    @contextmanager
    def open(self, target: DeviceTarget) -> Iterator[WujiSdkMotionSession]:
        session = WujiSdkMotionSession(target)
        try:
            yield session
        finally:
            session.close()


class WujiSdkMotionSession(_WujiSdkReadOnlySession):
    def __init__(self, target: DeviceTarget) -> None:
        self._command_publisher: Any = None
        self._may_be_enabled = False
        super().__init__(target)

    def control_readback(self) -> ControlReadback:
        hand = self._connected_hand()
        efforts = tuple(
            None if value is None else float(value) for value in hand.effort_limit().get()
        )
        params = tuple(
            None if value is None else MitParameters(kp=float(value.kp), kd=float(value.kd))
            for value in hand.mit_params().get()
        )
        return ControlReadback(effort_limits_a=efforts, mit_parameters=params)

    def open_command_stream(self) -> None:
        if self._command_publisher is not None:
            raise RuntimeError("command stream is already open")
        self._command_publisher = self._connected_hand().joint_command().publish()

    def send_command(self, command: tuple[JointCommandValue, ...]) -> None:
        if self._command_publisher is None:
            raise RuntimeError("command stream is not open")
        sdk = self._sdk_module()
        self._command_publisher.send(
            [
                sdk.JointCommand(
                    value.position_rad,
                    value.velocity_rad_s,
                    value.effort_a,
                )
                for value in command
            ]
        )

    def enable_selected(self, mask: tuple[int, ...]) -> None:
        self._connected_hand().enable(joints=list(mask))
        self._may_be_enabled = True

    def disable_selected(self, mask: tuple[int, ...] | None = None) -> None:
        hand = self._connected_hand()
        if mask is None:
            hand.disable()
        else:
            hand.disable(joints=list(mask))
        self._may_be_enabled = False

    def emergency_stop_all(self) -> None:
        self._connected_hand().emergency_stop()
        self._may_be_enabled = False

    def close_command_stream(self) -> None:
        if self._command_publisher is not None:
            self._command_publisher.close()
            self._command_publisher = None

    def close(self) -> None:
        try:
            if self._may_be_enabled:
                try:
                    self.disable_selected()
                except Exception:  # noqa: BLE001 - disable failure must escalate to e-stop.
                    self.emergency_stop_all()
            self.close_command_stream()
        finally:
            super().close()
