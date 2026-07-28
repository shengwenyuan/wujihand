from __future__ import annotations

import ast
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import cast

import numpy as np
import pytest

import wujihand.adapters.input.openvr_tracker as openvr_adapter
from wujihand.adapters.input.openvr_tracker import OpenVrTrackerAdapter
from wujihand.domain import ClutchEdge, TrackingState
from wujihand.domain.pose import quaternion_wxyz_to_rotation_matrix
from wujihand.ports import TrackingInputPort


ROOT = Path(__file__).parents[2]
SERIAL = "LHR-TEST-TRACKER"


@dataclass
class _FakeMatrix:
    m: tuple[tuple[float, float, float, float], ...]


@dataclass
class _FakePose:
    mDeviceToAbsoluteTracking: _FakeMatrix
    eTrackingResult: int = 200
    bPoseIsValid: bool = True
    bDeviceIsConnected: bool = True


@dataclass
class _FakeControllerState:
    ulButtonPressed: int = 0


class _FakeSystem:
    def __init__(self, module: _FakeOpenVr) -> None:
        self.module = module
        self.devices: dict[int, dict[str, object]] = {
            0: {
                "serial": "HMD-TEST",
                "class": module.TrackedDeviceClass_HMD,
                "model": "VIVE HMD",
                "manufacturer": "HTC",
                "connected": True,
            },
            2: {
                "serial": SERIAL,
                "class": module.TrackedDeviceClass_GenericTracker,
                "model": "VIVE Tracker",
                "manufacturer": "HTC",
                "connected": True,
            },
            5: {
                "serial": "BASE-TEST",
                "class": module.TrackedDeviceClass_TrackingReference,
                "model": "Base Station 2.0",
                "manufacturer": "Valve",
                "connected": True,
            },
        }
        self.poses = [_identity_pose(connected=False) for _ in range(8)]
        self.poses[2] = _identity_pose()
        self.controller_states: dict[int, _FakeControllerState] = {
            2: _FakeControllerState()
        }

    def getTrackedDeviceClass(self, device_index: int) -> int:
        device = self.devices.get(device_index)
        return (
            self.module.TrackedDeviceClass_Invalid
            if device is None
            else cast(int, device["class"])
        )

    def isTrackedDeviceConnected(self, device_index: int) -> bool:
        device = self.devices.get(device_index)
        return bool(device is not None and device["connected"])

    def getStringTrackedDeviceProperty(self, device_index: int, prop: int) -> str:
        device = self.devices[device_index]
        fields = {
            self.module.Prop_SerialNumber_String: "serial",
            self.module.Prop_ModelNumber_String: "model",
            self.module.Prop_ManufacturerName_String: "manufacturer",
        }
        return cast(str, device[fields[prop]])

    def getDeviceToAbsoluteTrackingPose(
        self,
        origin: int,
        predicted_seconds_to_photons_from_now: float,
        tracked_device_pose_array: object,
    ) -> list[_FakePose]:
        assert origin == self.module.TrackingUniverseStanding
        assert predicted_seconds_to_photons_from_now == 0.0
        assert tracked_device_pose_array == ()
        return self.poses

    def getControllerState(
        self,
        controller_device_index: int,
    ) -> tuple[bool, _FakeControllerState]:
        state = self.controller_states.get(controller_device_index)
        return state is not None, state or _FakeControllerState()

    def move_tracker(self, old_index: int, new_index: int) -> None:
        self.devices[new_index] = self.devices.pop(old_index)
        while len(self.poses) <= new_index:
            self.poses.append(_identity_pose(connected=False))
        self.poses[new_index] = self.poses[old_index]
        self.poses[old_index] = _identity_pose(connected=False)
        state = self.controller_states.pop(old_index, _FakeControllerState())
        self.controller_states[new_index] = state


class _FakeOpenVr:
    VRApplication_Background = 3
    TrackingUniverseStanding = 1
    k_unMaxTrackedDeviceCount = 16

    TrackedDeviceClass_Invalid = 0
    TrackedDeviceClass_HMD = 1
    TrackedDeviceClass_Controller = 2
    TrackedDeviceClass_GenericTracker = 3
    TrackedDeviceClass_TrackingReference = 4
    TrackedDeviceClass_DisplayRedirect = 5

    Prop_SerialNumber_String = 1002
    Prop_ModelNumber_String = 1001
    Prop_ManufacturerName_String = 1005

    TrackingResult_Uninitialized = 1
    TrackingResult_Calibrating_InProgress = 100
    TrackingResult_Calibrating_OutOfRange = 101
    TrackingResult_Running_OK = 200
    TrackingResult_Running_OutOfRange = 201
    TrackingResult_Fallback_RotationOnly = 300

    def __init__(self) -> None:
        self.system = _FakeSystem(self)
        self.init_calls = 0
        self.shutdown_calls = 0

    def init(self, application_type: int) -> _FakeSystem:
        assert application_type == self.VRApplication_Background
        self.init_calls += 1
        return self.system

    def shutdown(self) -> None:
        self.shutdown_calls += 1


def _z_pose(
    angle_degrees: float,
    *,
    connected: bool = True,
    valid: bool = True,
    tracking_result: int = 200,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> _FakePose:
    angle = math.radians(angle_degrees)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return _FakePose(
        mDeviceToAbsoluteTracking=_FakeMatrix(
            (
                (cosine, -sine, 0.0, position[0]),
                (sine, cosine, 0.0, position[1]),
                (0.0, 0.0, 1.0, position[2]),
            )
        ),
        eTrackingResult=tracking_result,
        bPoseIsValid=valid,
        bDeviceIsConnected=connected,
    )


def _identity_pose(*, connected: bool = True) -> _FakePose:
    return _z_pose(0.0, connected=connected, valid=connected)


def _quaternion_pose(
    quaternion_wxyz: tuple[float, float, float, float],
) -> _FakePose:
    normalized = np.asarray(quaternion_wxyz, dtype=np.float64)
    normalized /= np.linalg.norm(normalized)
    rotation = quaternion_wxyz_to_rotation_matrix(normalized)
    return _FakePose(
        mDeviceToAbsoluteTracking=_FakeMatrix(
            cast(
                tuple[tuple[float, float, float, float], ...],
                tuple(
                    (
                        float(rotation[row, 0]),
                        float(rotation[row, 1]),
                        float(rotation[row, 2]),
                        0.0,
                    )
                    for row in range(3)
                ),
            )
        )
    )


@pytest.fixture
def fake_openvr(monkeypatch: pytest.MonkeyPatch) -> _FakeOpenVr:
    runtime = _FakeOpenVr()
    monkeypatch.setattr(openvr_adapter, "_load_openvr_runtime", lambda: runtime)
    return runtime


def _adapter(*, clutch_button_id: int | None = 2) -> OpenVrTrackerAdapter:
    return OpenVrTrackerAdapter(
        SERIAL,
        "vive.right",
        "operator_right",
        clutch_button_id=clutch_button_id,
        clutch_input_id="grip",
    )


def test_openvr_is_not_imported_at_module_import_time() -> None:
    path = ROOT / "src/wujihand/adapters/input/openvr_tracker.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    imports = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    }
    imports.update(
        node.module or ""
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
    )

    assert "openvr" not in imports


def test_inventory_without_serial_exposes_no_transient_device_index(
    fake_openvr: _FakeOpenVr,
) -> None:
    adapter = OpenVrTrackerAdapter(None, "inventory", "qualification")

    inventory = adapter.inventory()

    assert [item.serial for item in inventory] == [
        "BASE-TEST",
        "HMD-TEST",
        SERIAL,
    ]
    assert next(item for item in inventory if item.serial == SERIAL).device_class == (
        "generic_tracker"
    )
    assert all(not hasattr(item, "device_index") for item in inventory)
    with pytest.raises(ValueError, match="tracker_serial"):
        adapter.start()
    adapter.close()
    assert fake_openvr.shutdown_calls == 1


def test_adapter_satisfies_port_and_re_resolves_serial_after_index_change(
    fake_openvr: _FakeOpenVr,
) -> None:
    adapter = _adapter(clutch_button_id=None)
    selected = adapter.start()
    fake_openvr.system.poses[2] = _z_pose(0.0, position=(1.0, 2.0, 3.0))

    first = adapter.poll(host_time_ns=10)
    fake_openvr.system.move_tracker(2, 7)
    fake_openvr.system.poses[7] = _z_pose(90.0, position=(4.0, 5.0, 6.0))
    second = adapter.poll(host_time_ns=20)

    assert isinstance(adapter, TrackingInputPort)
    assert selected.serial == SERIAL
    assert first.sample.sequence == 0
    assert first.sample.position_m == pytest.approx((1.0, 2.0, 3.0))
    assert second.sample.sequence == 1
    assert second.sample.position_m == pytest.approx((4.0, 5.0, 6.0))
    assert second.sample.quat_wxyz == pytest.approx(
        (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    )


def test_invalid_pose_never_reuses_last_pose_and_resets_hemisphere(
    fake_openvr: _FakeOpenVr,
) -> None:
    adapter = _adapter(clutch_button_id=None)
    adapter.start()
    fake_openvr.system.poses[2] = _quaternion_pose((0.0, 0.711, -0.703, 0.0))

    before_loss = adapter.poll(host_time_ns=100).sample
    fake_openvr.system.poses[2] = _z_pose(
        0.0,
        valid=False,
        tracking_result=fake_openvr.TrackingResult_Running_OutOfRange,
    )
    lost = adapter.poll(host_time_ns=200).sample
    fake_openvr.system.poses[2] = _quaternion_pose((0.0, 0.703, -0.711, 0.0))
    reacquired = adapter.poll(host_time_ns=300).sample

    assert before_loss.pose_valid
    assert lost.tracking_state is TrackingState.OUT_OF_RANGE
    assert lost.connected
    assert not lost.pose_valid
    assert lost.position_m is None
    assert lost.quat_wxyz is None
    assert reacquired.pose_valid
    assert reacquired.quat_wxyz is not None
    assert reacquired.quat_wxyz[1] < 0.0
    assert reacquired.quat_wxyz[2] > 0.0
    assert before_loss.quat_wxyz is not None
    assert (
        sum(
            left * right
            for left, right in zip(before_loss.quat_wxyz, reacquired.quat_wxyz)
        )
        < 0.0
    )


def test_quaternion_hemisphere_is_continuous_without_loss(
    fake_openvr: _FakeOpenVr,
) -> None:
    adapter = _adapter(clutch_button_id=None)
    adapter.start()
    fake_openvr.system.poses[2] = _quaternion_pose((0.0, 0.711, -0.703, 0.0))
    first = adapter.poll(host_time_ns=100).sample
    fake_openvr.system.poses[2] = _quaternion_pose((0.0, 0.703, -0.711, 0.0))
    second = adapter.poll(host_time_ns=200).sample

    assert first.quat_wxyz is not None
    assert second.quat_wxyz is not None
    dot = sum(left * right for left, right in zip(first.quat_wxyz, second.quat_wxyz))
    assert dot > 0.0
    assert second.quat_wxyz[1] > 0.0
    assert second.quat_wxyz[2] < 0.0


def test_button_state_edges_become_canonical_clutch_events(
    fake_openvr: _FakeOpenVr,
) -> None:
    adapter = _adapter()
    adapter.start()

    baseline = adapter.poll(host_time_ns=10)
    fake_openvr.system.controller_states[2].ulButtonPressed = 1 << 2
    pressed = adapter.poll(host_time_ns=20)
    held = adapter.poll(host_time_ns=30)
    fake_openvr.system.controller_states[2].ulButtonPressed = 0
    released = adapter.poll(host_time_ns=40)

    assert baseline.clutch_events == ()
    assert len(pressed.clutch_events) == 1
    assert pressed.clutch_events[0].edge is ClutchEdge.PRESSED
    assert pressed.clutch_events[0].sequence == 0
    assert pressed.clutch_events[0].epoch_request
    assert held.clutch_events == ()
    assert released.clutch_events[0].edge is ClutchEdge.RELEASED
    assert released.clutch_events[0].sequence == 1
    assert not released.clutch_events[0].epoch_request


def test_device_disappearance_is_explicit_loss_and_raw_record_is_json_safe(
    fake_openvr: _FakeOpenVr,
) -> None:
    adapter = _adapter(clutch_button_id=None)
    adapter.start()
    adapter.poll(host_time_ns=10)
    valid_raw = adapter.last_raw_record
    assert valid_raw is not None
    assert isinstance(valid_raw["matrix_3x4"], list)
    json.dumps(valid_raw, allow_nan=False)
    _assert_json_types(valid_raw)
    fake_openvr.system.devices.pop(2)

    poll = adapter.poll(host_time_ns=20)
    raw = adapter.last_raw_record

    assert poll.sample.tracking_state is TrackingState.LOST
    assert not poll.sample.connected
    assert not poll.sample.pose_valid
    assert poll.sample.position_m is None
    assert poll.sample.quat_wxyz is None
    assert raw is not None
    assert set(raw) == {
        "host_time_ns",
        "serial",
        "device_class",
        "connected",
        "pose_valid",
        "tracking_result",
        "matrix_3x4",
    }
    assert "device_index" not in raw
    json.dumps(raw, allow_nan=False)
    _assert_json_types(raw)

    mutable_copy = cast(dict[str, object], raw)
    mutable_copy["serial"] = "changed"
    assert adapter.last_raw_record is not None
    assert adapter.last_raw_record["serial"] == SERIAL


def test_sdk_valid_flag_with_nonfinite_matrix_fails_closed(
    fake_openvr: _FakeOpenVr,
) -> None:
    adapter = _adapter(clutch_button_id=None)
    adapter.start()
    fake_openvr.system.poses[2] = _FakePose(
        mDeviceToAbsoluteTracking=_FakeMatrix(
            (
                (1.0, 0.0, 0.0, math.nan),
                (0.0, 1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
            )
        )
    )

    poll = adapter.poll(host_time_ns=10)
    raw = adapter.last_raw_record

    assert poll.sample.tracking_state is TrackingState.LOST
    assert poll.sample.position_m is None
    assert poll.sample.quat_wxyz is None
    assert raw is not None
    assert raw["pose_valid"] is True
    assert raw["matrix_3x4"] is None
    json.dumps(raw, allow_nan=False)


def test_nonincreasing_host_timestamp_is_rejected(
    fake_openvr: _FakeOpenVr,
) -> None:
    adapter = _adapter(clutch_button_id=None)
    adapter.start()
    adapter.poll(host_time_ns=10)

    with pytest.raises(ValueError, match="increase strictly"):
        adapter.poll(host_time_ns=10)


def test_close_is_idempotent(fake_openvr: _FakeOpenVr) -> None:
    adapter = _adapter()
    adapter.start()

    adapter.close()
    adapter.close()

    assert fake_openvr.shutdown_calls == 1
    with pytest.raises(RuntimeError, match=r"start\(\)"):
        adapter.poll()


def _assert_json_types(value: object) -> None:
    assert value is None or type(value) in {str, int, float, bool, list, dict}
    if isinstance(value, list):
        for item in value:
            _assert_json_types(item)
    elif isinstance(value, dict):
        assert all(type(key) is str for key in value)
        for item in value.values():
            _assert_json_types(item)
