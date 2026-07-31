"""Shared strict configuration entry for ROS nodes."""

from __future__ import annotations

from pathlib import Path

from rclpy.node import Node  # type: ignore[import-not-found]

from wujihand.runtime import ResolvedRosDeployment, RosDeploymentResolver


def declare_and_resolve(node: Node) -> ResolvedRosDeployment:
    node.declare_parameter("project_root", "")
    node.declare_parameter("deployment_path", "")
    node.declare_parameter("local_runtime_binding_path", "")
    project_root = str(node.get_parameter("project_root").value)
    deployment_path = str(node.get_parameter("deployment_path").value)
    local_path = str(
        node.get_parameter("local_runtime_binding_path").value
    )
    if not deployment_path or not local_path:
        raise ValueError(
            "deployment_path and local_runtime_binding_path are required"
        )
    root = Path.cwd() if not project_root else Path(project_root)
    return RosDeploymentResolver(root).resolve(
        deployment_path,
        local_binding=local_path,
        verify_artifacts=False,
    )


__all__ = ["declare_and_resolve"]
