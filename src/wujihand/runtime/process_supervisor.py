"""Managed process composition for runtime DeploymentSpec components."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
import selectors
import subprocess
from typing import Callable
from urllib.parse import urlparse
import uuid

from wujihand.adapters.storage import decode_tracking_lifecycle_event_json
from wujihand.domain import (
    TrackingLifecycleEvent,
    TrackingLifecycleKind,
)

from .deployment_resolver import ResolvedDeployment


OPENVR_PRODUCER_COMPONENT = "openvr_dual_tracker_producer"


@dataclass(frozen=True, slots=True)
class OpenVrStreamLaunch:
    """One side-specific Tracker stream passed to the managed producer."""

    side: str
    source_id: str
    stream_id: str
    logical_role: str
    device_serial: str
    udp_port: int


@dataclass(frozen=True, slots=True)
class OpenVrProducerLaunch:
    """Fully resolved argv for one trusted OpenVR producer component."""

    process_id: str
    executable: str
    environment_id: str
    tool_path: str
    project_root: str
    producer_instance: str
    transport_epoch: int
    previous_transport_epoch: int | None
    tracking_setup_revision: str
    tracking_frame: str
    poll_hz: float
    streams: tuple[OpenVrStreamLaunch, ...]

    @property
    def command(self) -> tuple[str, ...]:
        command = [
            self.executable,
            self.tool_path,
            "--producer-instance",
            self.producer_instance,
            "--transport-epoch",
            str(self.transport_epoch),
            "--tracking-setup-revision",
            self.tracking_setup_revision,
            "--tracking-frame",
            self.tracking_frame,
            "--poll-hz",
            f"{self.poll_hz:g}",
        ]
        if self.previous_transport_epoch is not None:
            command.extend(
                (
                    "--previous-transport-epoch",
                    str(self.previous_transport_epoch),
                )
            )
        for stream in self.streams:
            command.extend(
                (
                    f"--{stream.side}-serial",
                    stream.device_serial,
                    f"--{stream.side}-udp-port",
                    str(stream.udp_port),
                )
            )
        return tuple(command)

    @property
    def expected_kind(self) -> TrackingLifecycleKind:
        return (
            TrackingLifecycleKind.STARTED
            if self.previous_transport_epoch is None
            else TrackingLifecycleKind.REBOUND
        )

    def next_epoch(self) -> OpenVrProducerLaunch:
        return replace(
            self,
            transport_epoch=self.transport_epoch + 1,
            previous_transport_epoch=self.transport_epoch,
        )


def build_openvr_producer_launch(
    resolved: ResolvedDeployment,
    project_root: str | Path,
    *,
    producer_instance: str | None = None,
    transport_epoch: int = 0,
    previous_transport_epoch: int | None = None,
    poll_hz: float = 90.0,
) -> OpenVrProducerLaunch:
    """Compile one trusted component registry entry from a resolved deployment."""

    if type(transport_epoch) is not int or transport_epoch < 0:
        raise ValueError("transport_epoch must be a non-negative integer")
    if previous_transport_epoch is not None and (
        type(previous_transport_epoch) is not int
        or previous_transport_epoch < 0
        or previous_transport_epoch == transport_epoch
    ):
        raise ValueError(
            "previous_transport_epoch must be non-negative and differ from current"
        )
    if not 1.0 <= poll_hz <= 500.0:
        raise ValueError("poll_hz must be in [1, 500]")

    candidates = tuple(
        process
        for process in resolved.processes
        if process.process.component_id == OPENVR_PRODUCER_COMPONENT
    )
    if len(candidates) != 1:
        raise ValueError(
            "deployment must resolve exactly one OpenVR producer component"
        )
    process = candidates[0]
    if process.process.lifecycle != "managed" or process.local_binding is None:
        raise ValueError("OpenVR producer must be a locally bound managed process")

    source_rows: list[OpenVrStreamLaunch] = []
    for source in resolved.sources:
        spec = source.source
        if (
            spec.process_id != process.process.process_id
            or spec.kind != "vive_tracker"
        ):
            continue
        local = source.local_binding
        if local is None:
            raise ValueError("live Tracker source requires a local binding")
        expected_role = f"operator_{spec.side}"
        if spec.logical_role != expected_role:
            raise ValueError(
                f"Tracker role must be {expected_role!r} for side {spec.side!r}"
            )
        source_rows.append(
            OpenVrStreamLaunch(
                side=spec.side,
                source_id=spec.source_id,
                stream_id=f"vive.{spec.side}",
                logical_role=spec.logical_role,
                device_serial=local.device_identity,
                udp_port=_loopback_udp_port(local.endpoint),
            )
        )
    streams = tuple(sorted(source_rows, key=lambda item: item.side))
    if not streams:
        raise ValueError("OpenVR producer has no bound Tracker streams")
    if len({stream.device_serial for stream in streams}) != len(streams):
        raise ValueError("Tracker serial bindings must be unique")
    if len({stream.udp_port for stream in streams}) != len(streams):
        raise ValueError("Tracker UDP endpoints must be unique")

    root = Path(project_root).resolve()
    tool = root / "tools/stream_vive_trackers_udp.py"
    instance = (
        f"openvr_{uuid.uuid4().hex}"
        if producer_instance is None
        else producer_instance
    )
    return OpenVrProducerLaunch(
        process_id=process.process.process_id,
        executable=process.local_binding.executable,
        environment_id=process.local_binding.environment_id,
        tool_path=tool.as_posix(),
        project_root=root.as_posix(),
        producer_instance=instance,
        transport_epoch=transport_epoch,
        previous_transport_epoch=previous_transport_epoch,
        tracking_setup_revision=(
            resolved.deployment.tracking_setup.setup_revision
        ),
        tracking_frame=resolved.deployment.tracking_setup.tracking_frame,
        poll_hz=poll_hz,
        streams=streams,
    )


class ManagedOpenVrProducer:
    """Start, monitor, restart and stop the one owned OpenVR producer."""

    def __init__(
        self,
        launch: OpenVrProducerLaunch,
        *,
        popen_factory: Callable[..., subprocess.Popen[str]] = subprocess.Popen,
    ) -> None:
        self.launch = launch
        self._popen_factory = popen_factory
        self._process: subprocess.Popen[str] | None = None
        self._last_lifecycle: TrackingLifecycleEvent | None = None

    @property
    def running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    @property
    def last_lifecycle(self) -> TrackingLifecycleEvent | None:
        return self._last_lifecycle

    def start(self, *, timeout_s: float = 15.0) -> TrackingLifecycleEvent:
        if self._process is not None:
            raise RuntimeError("OpenVR producer is already started")
        executable = Path(self.launch.executable)
        tool = Path(self.launch.tool_path)
        if not executable.is_file():
            raise FileNotFoundError(
                f"OpenVR Python executable not found: {executable}"
            )
        if not tool.is_file():
            raise FileNotFoundError(f"OpenVR producer tool not found: {tool}")
        process = self._popen_factory(
            self.launch.command,
            cwd=self.launch.project_root,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=None,
            text=True,
            bufsize=1,
            close_fds=True,
        )
        self._process = process
        try:
            event = self._read_lifecycle(timeout_s=timeout_s)
            _validate_lifecycle(event, self.launch)
        except Exception:
            self._terminate_process()
            self._process = None
            raise
        self._last_lifecycle = event
        return event

    def ensure_running(self) -> None:
        process = self._process
        if process is None:
            raise RuntimeError("OpenVR producer has not been started")
        return_code = process.poll()
        if return_code is not None:
            raise RuntimeError(
                f"OpenVR producer exited unexpectedly with code {return_code}"
            )

    def restart(self, *, timeout_s: float = 15.0) -> TrackingLifecycleEvent:
        if self._process is not None:
            self.stop(timeout_s=timeout_s)
        self.launch = self.launch.next_epoch()
        return self.start(timeout_s=timeout_s)

    def stop(self, *, timeout_s: float = 10.0) -> TrackingLifecycleEvent | None:
        process = self._process
        if process is None:
            return None
        if process.poll() is None:
            process.terminate()
        try:
            stdout, _ = process.communicate(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            stdout, _ = process.communicate(timeout=timeout_s)
        self._process = None
        events = tuple(
            decode_tracking_lifecycle_event_json(line)
            for line in stdout.splitlines()
            if line.strip()
        )
        stopped = next(
            (
                event
                for event in reversed(events)
                if event.kind is TrackingLifecycleKind.STOPPED
            ),
            None,
        )
        if stopped is not None:
            _validate_stopped_lifecycle(stopped, self.launch)
            self._last_lifecycle = stopped
        return stopped

    def close(self) -> None:
        self.stop()

    def __enter__(self) -> ManagedOpenVrProducer:
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _read_lifecycle(self, *, timeout_s: float) -> TrackingLifecycleEvent:
        process = self._process
        if process is None or process.stdout is None:
            raise RuntimeError("OpenVR producer stdout is unavailable")
        if not 0.1 <= timeout_s <= 120.0:
            raise ValueError("timeout_s must be in [0.1, 120]")
        selector = selectors.DefaultSelector()
        try:
            selector.register(process.stdout, selectors.EVENT_READ)
            if not selector.select(timeout_s):
                raise TimeoutError(
                    "OpenVR producer did not emit a lifecycle event in time"
                )
            line = process.stdout.readline()
        finally:
            selector.close()
        if not line:
            return_code = process.poll()
            raise RuntimeError(
                "OpenVR producer exited before its lifecycle event "
                f"(code={return_code})"
            )
        return decode_tracking_lifecycle_event_json(line)

    def _terminate_process(self) -> None:
        process = self._process
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)


def _loopback_udp_port(endpoint: str) -> int:
    parsed = urlparse(endpoint)
    try:
        port = parsed.port
    except ValueError as exc:
        raise ValueError(f"invalid Tracker UDP endpoint: {endpoint!r}") from exc
    if (
        parsed.scheme != "udp"
        or parsed.hostname != "127.0.0.1"
        or port is None
        or parsed.username is not None
        or parsed.password is not None
        or parsed.path not in {"", "/"}
        or parsed.params
        or parsed.query
        or parsed.fragment
    ):
        raise ValueError(
            "Tracker endpoint must be udp://127.0.0.1:<port>"
        )
    return port


def _validate_lifecycle(
    event: TrackingLifecycleEvent,
    launch: OpenVrProducerLaunch,
) -> None:
    expected_streams = tuple(stream.stream_id for stream in launch.streams)
    if (
        event.kind is not launch.expected_kind
        or event.producer_instance != launch.producer_instance
        or event.tracking_setup_revision != launch.tracking_setup_revision
        or event.stream_ids != expected_streams
        or event.new_transport_epoch != launch.transport_epoch
        or event.old_transport_epoch != launch.previous_transport_epoch
    ):
        raise RuntimeError("OpenVR producer lifecycle does not match launch")


def _validate_stopped_lifecycle(
    event: TrackingLifecycleEvent,
    launch: OpenVrProducerLaunch,
) -> None:
    expected_streams = tuple(stream.stream_id for stream in launch.streams)
    if (
        event.producer_instance != launch.producer_instance
        or event.tracking_setup_revision != launch.tracking_setup_revision
        or event.stream_ids != expected_streams
        or event.old_transport_epoch != launch.transport_epoch
        or event.new_transport_epoch is not None
    ):
        raise RuntimeError("OpenVR producer stop lifecycle does not match launch")


__all__ = [
    "OPENVR_PRODUCER_COMPONENT",
    "ManagedOpenVrProducer",
    "OpenVrProducerLaunch",
    "OpenVrStreamLaunch",
    "build_openvr_producer_launch",
]
