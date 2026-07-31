"""Lifecycle source owning one OpenVR runtime for both Tracker routes."""

from __future__ import annotations

import time
import uuid

import rclpy  # type: ignore[import-not-found]
from rclpy.executors import (  # type: ignore[import-not-found]
    ExternalShutdownException,
    SingleThreadedExecutor,
)
from rclpy.lifecycle import (  # type: ignore[import-not-found]
    LifecycleNode,
    TransitionCallbackReturn,
)
from rclpy.lifecycle.publisher import (  # type: ignore[import-not-found]
    LifecyclePublisher,
)
from rclpy.timer import Timer  # type: ignore[import-not-found]
from lifecycle_msgs.msg import State  # type: ignore[import-not-found]
from wujihand_interfaces.msg import (  # type: ignore[import-not-found]
    TrackedRigidBodySample as TrackedRigidBodySampleMessage,
    TrackingLifecycleEvent as TrackingLifecycleEventMessage,
)

from wujihand.adapters.input import OpenVrMultiTrackerAdapter
from wujihand.domain import TrackingLifecycleEvent, TrackingLifecycleKind
from wujihand.runtime import ResolvedRosDeployment

from ..conversion import (
    lifecycle_event_to_message,
    tracked_sample_to_message,
)
from ..qos import qos_profile
from ..source_config import vive_stream_configs
from ._configuration import declare_and_resolve


class ViveSourceNode(LifecycleNode):  # type: ignore[misc]
    def __init__(self) -> None:
        super().__init__("vive_source")
        self._producer_instance = f"vive-source-{uuid.uuid4().hex}"
        self._activation_epoch = 0
        self._lifecycle_sequence = 0
        self._resolved: ResolvedRosDeployment | None = None
        self._adapter: OpenVrMultiTrackerAdapter | None = None
        self._timer: Timer | None = None
        self._sample_publishers: dict[
            str,
            LifecyclePublisher[TrackedRigidBodySampleMessage],
        ] = {}
        self._lifecycle_publisher: (
            LifecyclePublisher[TrackingLifecycleEventMessage] | None
        ) = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        del state
        try:
            resolved = declare_and_resolve(self)
            if not any(
                process.process_id == "vive_source"
                for process in resolved.deployment.processes
            ):
                raise ValueError(
                    "Deployment does not contain vive_source"
                )
            sample_qos = qos_profile(
                resolved.qos_profile.policy("tracker_sample")
            )
            for config in vive_stream_configs(resolved):
                side = resolved.deployment.source(config.stream_id).side
                self._sample_publishers[config.stream_id] = (
                    self.create_lifecycle_publisher(
                        TrackedRigidBodySampleMessage,
                        f"input/tracker/{side}/sample",
                        sample_qos,
                    )
                )
            self._lifecycle_publisher = (
                self.create_lifecycle_publisher(
                    TrackingLifecycleEventMessage,
                    "input/tracker/lifecycle",
                    qos_profile(
                        resolved.qos_profile.policy(
                            "tracking_lifecycle"
                        )
                    ),
                )
            )
            self._resolved = resolved
        except Exception as exc:
            self.get_logger().error(f"configure failed: {exc}")
            self._destroy_publishers()
            self._resolved = None
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        resolved = self._resolved
        if resolved is None:
            return TransitionCallbackReturn.FAILURE
        self._activation_epoch += 1
        adapter = OpenVrMultiTrackerAdapter(
            vive_stream_configs(resolved),
            producer_instance=self._producer_instance,
            transport_epoch=self._activation_epoch,
            tracking_setup_revision=(
                resolved.deployment.tracking_setup.setup_revision
            ),
        )
        try:
            adapter.start()
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                adapter.close()
                return result
            self._adapter = adapter
            self._publish_lifecycle(
                kind=TrackingLifecycleKind.STARTED,
                reason="activated",
                old_epoch=None,
                new_epoch=self._activation_epoch,
            )
            self._timer = self.create_timer(
                1.0 / resolved.control_profile.physics_hz,
                self._poll,
            )
        except Exception as exc:
            adapter.close()
            self.get_logger().error(f"activate failed: {exc}")
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._stop_timer()
        if self._adapter is not None:
            self._publish_lifecycle(
                kind=TrackingLifecycleKind.STOPPED,
                reason="deactivated",
                old_epoch=self._activation_epoch,
                new_epoch=None,
            )
            self._adapter.close()
            self._adapter = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        del state
        self._stop_timer()
        if self._adapter is not None:
            self._adapter.close()
            self._adapter = None
        self._destroy_publishers()
        self._resolved = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        del state
        self._stop_timer()
        if self._adapter is not None:
            self._adapter.close()
            self._adapter = None
        return TransitionCallbackReturn.SUCCESS

    def _poll(self) -> None:
        adapter = self._adapter
        if adapter is None:
            return
        try:
            polls = adapter.poll(host_time_ns=time.monotonic_ns())
            for poll in polls:
                publisher = self._sample_publishers[poll.sample.stream_id]
                publisher.publish(tracked_sample_to_message(poll.sample))
        except Exception as exc:
            self.get_logger().error(f"OpenVR poll failed: {exc}")

    def _publish_lifecycle(
        self,
        *,
        kind: TrackingLifecycleKind,
        reason: str,
        old_epoch: int | None,
        new_epoch: int | None,
    ) -> None:
        resolved = self._resolved
        publisher = self._lifecycle_publisher
        if resolved is None or publisher is None:
            return
        event = TrackingLifecycleEvent(
            producer_instance=self._producer_instance,
            tracking_setup_revision=(
                resolved.deployment.tracking_setup.setup_revision
            ),
            stream_ids=tuple(
                config.stream_id
                for config in vive_stream_configs(resolved)
            ),
            kind=kind,
            reason=reason,
            sequence=self._lifecycle_sequence,
            old_transport_epoch=old_epoch,
            new_transport_epoch=new_epoch,
            host_time_ns=time.monotonic_ns(),
        )
        self._lifecycle_sequence += 1
        publisher.publish(lifecycle_event_to_message(event))

    def _stop_timer(self) -> None:
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None

    def _destroy_publishers(self) -> None:
        for publisher in self._sample_publishers.values():
            self.destroy_lifecycle_publisher(publisher)
        self._sample_publishers.clear()
        if self._lifecycle_publisher is not None:
            self.destroy_lifecycle_publisher(
                self._lifecycle_publisher
            )
            self._lifecycle_publisher = None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ViveSourceNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
