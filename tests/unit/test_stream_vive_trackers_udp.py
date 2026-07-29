from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from typing import Any

import pytest

from wujihand.adapters.storage import decode_tracking_lifecycle_event_json
from wujihand.domain import (
    TrackedRigidBodySample,
    TrackingLifecycleKind,
    TrackingState,
)
from wujihand.ports import TrackerInventoryItem, TrackingPoll


ROOT = Path(__file__).parents[2]
SPEC = importlib.util.spec_from_file_location(
    "stream_vive_trackers_udp",
    ROOT / "tools/stream_vive_trackers_udp.py",
)
assert SPEC is not None and SPEC.loader is not None
cli = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = cli
SPEC.loader.exec_module(cli)


class _Clock:
    def __init__(self) -> None:
        self.now_ns = 0

    def __call__(self) -> int:
        self.now_ns += 20_000_000
        return self.now_ns

    def sleep(self, duration_s: float) -> None:
        self.now_ns += round(duration_s * 1_000_000_000)


class _Owner:
    instance: _Owner | None = None

    def __init__(
        self,
        streams: object,
        *,
        producer_instance: str,
        transport_epoch: int,
        tracking_setup_revision: str,
    ) -> None:
        self.streams = tuple(streams)  # type: ignore[arg-type]
        self.producer_instance = producer_instance
        self.transport_epoch = transport_epoch
        self.tracking_setup_revision = tracking_setup_revision
        self.sequence = 0
        self.closed = False
        type(self).instance = self

    def start(self) -> tuple[TrackerInventoryItem, ...]:
        return tuple(
            TrackerInventoryItem(
                serial=stream.tracker_serial,
                device_class="generic_tracker",
                model="VIVE Tracker 3.0",
                manufacturer="HTC",
                connected=True,
            )
            for stream in self.streams
        )

    def poll(self, *, host_time_ns: int | None = None) -> tuple[TrackingPoll, ...]:
        assert host_time_ns is not None
        result = tuple(
            TrackingPoll(
                sample=TrackedRigidBodySample(
                    stream_id=stream.stream_id,
                    device_serial=stream.tracker_serial,
                    logical_role=stream.logical_role,
                    producer_instance=self.producer_instance,
                    transport_epoch=self.transport_epoch,
                    tracking_setup_revision=self.tracking_setup_revision,
                    sequence=self.sequence,
                    tracking_frame=stream.tracking_frame,
                    position_m=(float(index), 0.0, 0.0),
                    quat_wxyz=(1.0, 0.0, 0.0, 0.0),
                    connected=True,
                    pose_valid=True,
                    tracking_state=TrackingState.RUNNING,
                    quality=1.0,
                    host_time_ns=host_time_ns,
                    device_time_ns=None,
                )
            )
            for index, stream in enumerate(self.streams)
        )
        self.sequence += 1
        return result

    def close(self) -> None:
        self.closed = True


class _Sender:
    instances: list[_Sender] = []

    def __init__(self, port: int) -> None:
        self.port = port
        self.samples: list[TrackedRigidBodySample] = []
        self.closed = False
        self.instances.append(self)

    def send(self, sample: TrackedRigidBodySample) -> None:
        self.samples.append(sample)

    def close(self) -> None:
        self.closed = True


def _args(*extra: str) -> Any:
    return cli.build_parser().parse_args(
        [
            "--left-serial",
            "LHR-LEFT",
            "--left-udp-port",
            "49154",
            "--right-serial",
            "LHR-RIGHT",
            "--right-udp-port",
            "49155",
            "--producer-instance",
            "openvr_dual_fixture",
            "--transport-epoch",
            "8",
            "--tracking-setup-revision",
            "standing_fixture_v1",
            "--duration-s",
            "0.12",
            *extra,
        ]
    )


def test_dual_producer_uses_one_owner_and_emits_lifecycle(
    capsys: pytest.CaptureFixture[str],
) -> None:
    _Sender.instances.clear()
    clock = _Clock()

    result = cli.run(
        _args(),
        adapter_factory=_Owner,
        sender_factory=_Sender,
        monotonic_ns=clock,
        sleeper=clock.sleep,
    )

    assert result == 0
    owner = _Owner.instance
    assert owner is not None and owner.closed
    assert [stream.stream_id for stream in owner.streams] == [
        "vive.left",
        "vive.right",
    ]
    assert len(_Sender.instances) == 2
    assert all(sender.closed and sender.samples for sender in _Sender.instances)
    assert {
        sample.host_time_ns
        for sender in _Sender.instances
        for sample in sender.samples
    } == {80_000_000, 120_000_000}

    stdout = capsys.readouterr().out.splitlines()
    events = tuple(decode_tracking_lifecycle_event_json(line) for line in stdout)
    assert [event.kind for event in events] == [
        TrackingLifecycleKind.STARTED,
        TrackingLifecycleKind.STOPPED,
    ]
    assert events[0].new_transport_epoch == 8
    assert events[1].old_transport_epoch == 8


def test_dual_producer_rejects_duplicate_identity_or_endpoint() -> None:
    duplicate_serial = _args()
    duplicate_serial.right_serial = duplicate_serial.left_serial
    with pytest.raises(ValueError, match="serials must differ"):
        cli.run(duplicate_serial)

    duplicate_port = _args()
    duplicate_port.right_udp_port = duplicate_port.left_udp_port
    with pytest.raises(ValueError, match="ports must differ"):
        cli.run(duplicate_port)
