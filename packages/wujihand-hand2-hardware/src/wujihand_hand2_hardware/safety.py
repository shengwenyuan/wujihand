from __future__ import annotations

from dataclasses import dataclass

from .types import DeviceIdentity, SafetyState


@dataclass(slots=True)
class ReadOnlyLifecycle:
    """The H1/H2 lifecycle deliberately has no arm or enable transition."""

    state: SafetyState = SafetyState.DISCONNECTED
    identity: DeviceIdentity | None = None
    fault_reason: str | None = None

    def connected(self, identity: DeviceIdentity) -> None:
        if self.state is not SafetyState.DISCONNECTED:
            raise RuntimeError(f"cannot connect from {self.state.value}")
        self.identity = identity
        self.state = SafetyState.READ_ONLY

    def fault(self, reason: str) -> None:
        self.fault_reason = reason
        self.state = SafetyState.FAULTED

    def disconnected(self) -> None:
        self.identity = None
        self.state = SafetyState.DISCONNECTED


@dataclass(slots=True)
class MotionLifecycle:
    state: SafetyState = SafetyState.DISCONNECTED
    identity: DeviceIdentity | None = None
    fault_reason: str | None = None

    def connected(self, identity: DeviceIdentity) -> None:
        if self.state is not SafetyState.DISCONNECTED:
            raise RuntimeError(f"cannot connect from {self.state.value}")
        self.identity = identity
        self.state = SafetyState.READ_ONLY

    def armed(self) -> None:
        if self.state is not SafetyState.READ_ONLY:
            raise RuntimeError(f"cannot arm from {self.state.value}")
        self.state = SafetyState.ARMED

    def enabled(self) -> None:
        if self.state is not SafetyState.ARMED:
            raise RuntimeError(f"cannot enable from {self.state.value}")
        self.state = SafetyState.ENABLED

    def disabled(self) -> None:
        if self.state not in {SafetyState.ARMED, SafetyState.ENABLED}:
            raise RuntimeError(f"cannot disable from {self.state.value}")
        self.state = SafetyState.READ_ONLY

    def fault(self, reason: str) -> None:
        self.fault_reason = reason
        self.state = SafetyState.FAULTED

    def estopped(self, reason: str) -> None:
        self.fault_reason = reason
        self.state = SafetyState.ESTOPPED

    def disconnected(self) -> None:
        if self.state is SafetyState.ENABLED:
            raise RuntimeError("cannot disconnect while enabled")
        self.identity = None
        self.state = SafetyState.DISCONNECTED
