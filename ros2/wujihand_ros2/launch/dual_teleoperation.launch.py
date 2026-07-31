"""Launch the three-process NV-5 ROS 2 teleoperation graph."""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration

from wujihand.runtime import RosDeploymentResolver


def _processes(context: object) -> list[object]:
    project_root = Path(
        LaunchConfiguration("project_root").perform(context)
    ).resolve()
    deployment_path = LaunchConfiguration("deployment").perform(context)
    local_path = LaunchConfiguration(
        "local_runtime_binding"
    ).perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    frames = int(LaunchConfiguration("frames").perform(context))
    record = (
        LaunchConfiguration("record").perform(context).lower() == "true"
    )
    if not gui and frames < 1:
        raise ValueError("headless launch requires frames > 0")
    resolved = RosDeploymentResolver(project_root).resolve(
        deployment_path,
        local_binding=local_path,
        verify_artifacts=False,
    )
    runtime_environment = {
        "ROS_DOMAIN_ID": str(resolved.local_binding.ros_domain_id),
        "RMW_IMPLEMENTATION": (
            resolved.local_binding.rmw_implementation
        ),
    }
    if resolved.local_binding.dds_profile is not None:
        runtime_environment["FASTRTPS_DEFAULT_PROFILES_FILE"] = (
            resolved.local_binding.dds_profile
        )
    namespace = f"/{resolved.deployment.root_namespace}"
    common_ros_args = [
        "--ros-args",
        "-r",
        f"__ns:={namespace}",
        "-p",
        f"project_root:={project_root}",
        "-p",
        f"deployment_path:={deployment_path}",
        "-p",
        f"local_runtime_binding_path:={local_path}",
    ]
    actions: list[object] = []
    source_modules = {
        "vive_source": "wujihand_ros2.nodes.vive_source",
        "glove_source": "wujihand_ros2.nodes.glove_source",
    }
    deployment_processes = {
        process.process_id for process in resolved.deployment.processes
    }
    lifecycle_nodes = []
    for process_id, module in source_modules.items():
        if process_id not in deployment_processes:
            continue
        process = resolved.local_binding.process(process_id)
        node_name = next(
            binding.node_name
            for binding in resolved.deployment.node_bindings
            if binding.process_id == process_id
        )
        lifecycle_nodes.append(f"{namespace}/{node_name}")
        actions.append(
            ExecuteProcess(
                cmd=[
                    process.executable,
                    "-m",
                    module,
                    *common_ros_args,
                ],
                name=process_id,
                output="screen",
                sigterm_timeout="10",
                additional_env=runtime_environment,
            )
        )
    consumer = resolved.local_binding.process("isaac_consumer")
    consumer_command = [
        consumer.executable,
        str(project_root / "tools/run_isaac_nero_hand2_ros.py"),
        "--deployment",
        deployment_path,
        "--local-runtime-binding",
        local_path,
        "--gui" if gui else "--no-gui",
    ]
    if frames:
        consumer_command.extend(["--frames", str(frames)])
    actions.append(
        ExecuteProcess(
            cmd=consumer_command,
            name="isaac_consumer",
            output="screen",
            sigterm_timeout="20",
            additional_env=runtime_environment,
        )
    )
    actions.append(
        ExecuteProcess(
            cmd=[
                "/usr/bin/python3",
                "-m",
                "wujihand_ros2.lifecycle_activate",
                *lifecycle_nodes,
            ],
            name="lifecycle_activate",
            output="screen",
            additional_env=runtime_environment,
        )
    )
    if record:
        topics = [
            f"{namespace}/input/tracker/left/sample",
            f"{namespace}/input/tracker/right/sample",
            f"{namespace}/input/tracker/lifecycle",
            f"{namespace}/input/glove/left/observation",
            f"{namespace}/input/glove/right/observation",
        ]
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "--storage",
                    "mcap",
                    "--output",
                    str(project_root / "artifacts/runs/nv5/rosbag"),
                    "--qos-profile-overrides-path",
                    str(
                        project_root
                        / "configs/profiles/"
                        "ros2_jazzy_dual_teleoperation_rosbag_qos_v1.yaml"
                    ),
                    *topics,
                ],
                name="rosbag2",
                output="screen",
                additional_env=runtime_environment,
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_root",
                default_value=str(Path.cwd()),
            ),
            DeclareLaunchArgument(
                "deployment",
                default_value=(
                    "configs/deployments/"
                    "isaac_nero_hand2_ros_dual_live_v2.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "local_runtime_binding",
                default_value=(
                    "configs/local/workstation2_nv5_ros_v2.yaml"
                ),
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("frames", default_value="0"),
            DeclareLaunchArgument("record", default_value="false"),
            OpaqueFunction(function=_processes),
        ]
    )
