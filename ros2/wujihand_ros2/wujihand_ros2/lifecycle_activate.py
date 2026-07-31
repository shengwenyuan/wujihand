"""Configure and activate a bounded list of lifecycle nodes, then exit."""

from __future__ import annotations

import argparse

from lifecycle_msgs.msg import Transition  # type: ignore[import-not-found]
from lifecycle_msgs.srv import ChangeState  # type: ignore[import-not-found]
import rclpy  # type: ignore[import-not-found]
from rclpy.node import Node  # type: ignore[import-not-found]


def _transition(
    node: Node,
    target: str,
    transition_id: int,
) -> None:
    client = node.create_client(ChangeState, f"{target}/change_state")
    if not client.wait_for_service(timeout_sec=30.0):
        raise RuntimeError(
            f"lifecycle service did not appear for {target}"
        )
    request = ChangeState.Request()
    request.transition.id = transition_id
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    if not future.done() or future.result() is None:
        raise RuntimeError(
            f"lifecycle transition timed out for {target}"
        )
    if not future.result().success:
        raise RuntimeError(
            f"lifecycle transition failed for {target}"
        )
    node.destroy_client(client)


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("nodes", nargs="+")
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Node("lifecycle_activate")
    try:
        for target in parsed.nodes:
            _transition(
                node,
                target,
                Transition.TRANSITION_CONFIGURE,
            )
        for target in parsed.nodes:
            _transition(
                node,
                target,
                Transition.TRANSITION_ACTIVATE,
            )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
