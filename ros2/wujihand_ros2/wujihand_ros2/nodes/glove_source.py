"""Lifecycle source owning the configured left/right Wuji Gloves."""

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
    HandObservationEnvelope as HandObservationEnvelopeMessage,
)

from wujihand.adapters.input import (
    NoHandSkeletonFrameAvailable,
    WujiGloveHandSkeletonAdapter,
)
from wujihand.domain import HandSide
from wujihand.runtime import ResolvedRosDeployment

from ..conversion import (
    HandObservationTransportEnvelope,
    hand_envelope_to_message,
)
from ..qos import qos_profile
from ..source_config import glove_source_configs
from ._configuration import declare_and_resolve


class GloveSourceNode(LifecycleNode):  # type: ignore[misc]
    def __init__(self) -> None:
        super().__init__("glove_source")
        self._producer_instance = f"glove-source-{uuid.uuid4().hex}"
        self._activation_epoch = 0
        self._resolved: ResolvedRosDeployment | None = None
        self._adapters: dict[
            HandSide,
            WujiGloveHandSkeletonAdapter,
        ] = {}
        self._observation_publishers: dict[
            HandSide,
            LifecyclePublisher[HandObservationEnvelopeMessage],
        ] = {}
        self._timer: Timer | None = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        del state
        try:
            resolved = declare_and_resolve(self)
            configs = glove_source_configs(resolved)
            if not configs:
                raise ValueError("Deployment has no Glove routes")
            observation_qos = qos_profile(
                resolved.qos_profile.policy("glove_observation")
            )
            for config in configs:
                self._observation_publishers[config.side] = (
                    self.create_lifecycle_publisher(
                        HandObservationEnvelopeMessage,
                        (
                            "input/glove/"
                            f"{config.side.value}/observation"
                        ),
                        observation_qos,
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
        adapters = {
            config.side: WujiGloveHandSkeletonAdapter(
                config.side,
                source_id=config.source_id,
                calibration_id=config.calibration_id,
                transform_id=config.transform_id,
                serial_number=config.serial_number,
                device_name=config.device_name,
            )
            for config in glove_source_configs(resolved)
        }
        started: list[WujiGloveHandSkeletonAdapter] = []
        try:
            for side in (HandSide.LEFT, HandSide.RIGHT):
                if side in adapters:
                    adapters[side].start()
                    started.append(adapters[side])
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError("publisher activation failed")
            self._adapters = adapters
            self._timer = self.create_timer(
                1.0 / resolved.control_profile.physics_hz,
                self._poll,
            )
        except Exception as exc:
            for adapter in reversed(started):
                adapter.close()
            self.get_logger().error(f"activate failed: {exc}")
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._stop_timer()
        self._close_adapters()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        del state
        self._stop_timer()
        self._close_adapters()
        self._destroy_publishers()
        self._resolved = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        del state
        self._stop_timer()
        self._close_adapters()
        return TransitionCallbackReturn.SUCCESS

    def _poll(self) -> None:
        receive_time_ns = time.monotonic_ns()
        for side, adapter in self._adapters.items():
            try:
                observation = adapter.poll(
                    receive_time_ns=receive_time_ns
                )
            except NoHandSkeletonFrameAvailable:
                continue
            except Exception as exc:
                self.get_logger().error(
                    f"{side.value} Glove poll failed: {exc}"
                )
                continue
            envelope = HandObservationTransportEnvelope(
                producer_instance=self._producer_instance,
                transport_epoch=self._activation_epoch,
                observation=observation,
            )
            self._observation_publishers[side].publish(
                hand_envelope_to_message(envelope)
            )

    def _stop_timer(self) -> None:
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None

    def _close_adapters(self) -> None:
        first_error: Exception | None = None
        for side in (HandSide.RIGHT, HandSide.LEFT):
            adapter = self._adapters.get(side)
            if adapter is None:
                continue
            try:
                adapter.close()
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        self._adapters.clear()
        if first_error is not None:
            self.get_logger().error(
                f"Glove cleanup failed: {first_error}"
            )

    def _destroy_publishers(self) -> None:
        for publisher in self._observation_publishers.values():
            self.destroy_lifecycle_publisher(publisher)
        self._observation_publishers.clear()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GloveSourceNode()
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
