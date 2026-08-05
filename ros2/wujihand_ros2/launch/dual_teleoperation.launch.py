"""Launch the three-process NV-5 ROS 2 teleoperation graph."""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from wujihand.runtime import (
    RosDeploymentResolver,
    new_run_id,
    parse_cpu_affinity,
    run_root,
)
from wujihand_ros2.recording import recording_topics, source_topics


def _processes(context: object) -> list[object]:
    project_root = Path(LaunchConfiguration("project_root").perform(context)).resolve()
    deployment_path = LaunchConfiguration("deployment").perform(context)
    local_path = LaunchConfiguration("local_runtime_binding").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    frames = int(LaunchConfiguration("frames").perform(context))
    record = LaunchConfiguration("record").perform(context).lower() == "true"
    isaac_cpu_affinity = (
        LaunchConfiguration("isaac_cpu_affinity", default="").perform(context).strip()
    )
    if isaac_cpu_affinity:
        parse_cpu_affinity(isaac_cpu_affinity)
    requested_run_id = (
        LaunchConfiguration(
            "run_id",
            default="",
        )
        .perform(context)
        .strip()
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
        "RMW_IMPLEMENTATION": (resolved.local_binding.rmw_implementation),
    }
    if resolved.local_binding.dds_profile is not None:
        runtime_environment["FASTRTPS_DEFAULT_PROFILES_FILE"] = resolved.local_binding.dds_profile
    namespace = f"/{resolved.deployment.root_namespace}"
    current_run_id = requested_run_id or new_run_id()
    current_run_root = run_root(
        project_root,
        resolved.deployment.report_root,
        current_run_id,
    )
    if record and current_run_root.exists():
        raise FileExistsError(f"recording run directory already exists: {current_run_root}")
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
    deployment_processes = {process.process_id for process in resolved.deployment.processes}
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
    if isaac_cpu_affinity:
        consumer_command.extend(["--cpu-affinity", isaac_cpu_affinity])
    if frames:
        consumer_command.extend(["--frames", str(frames)])
    if record:
        consumer_command.extend(
            [
                "--recording-enabled",
                "--run-id",
                current_run_id,
                "--run-root",
                str(current_run_root),
            ]
        )
    consumer_action = ExecuteProcess(
        cmd=consumer_command,
        name="isaac_consumer",
        output="screen",
        sigterm_timeout="20",
        additional_env=runtime_environment,
    )
    actions.append(consumer_action)
    recorder_action = None
    if record:
        recorder_topics = recording_topics(
            namespace,
            resolved.route_plan,
            include_synthetic_d405=True,
        )
        recorder_command = [
            "/usr/bin/python3",
            str(project_root / "tools/run_rosbag_recording.py"),
            "--run-root",
            str(current_run_root),
            "--run-id",
            current_run_id,
            "--qos-profile",
            str(
                project_root / "configs/profiles/"
                "ros2_jazzy_dual_teleoperation_d405_rosbag_qos_v1.yaml"
            ),
        ]
        for topic in recorder_topics:
            recorder_command.extend(["--topic", topic])
        recorder_action = ExecuteProcess(
            cmd=recorder_command,
            name="rosbag2",
            output="screen",
            sigterm_timeout="30",
            additional_env=runtime_environment,
        )
        actions.append(recorder_action)
    lifecycle_command = [
        "/usr/bin/python3",
        "-m",
        "wujihand_ros2.lifecycle_activate",
    ]
    if record:
        for topic in source_topics(namespace, resolved.route_plan):
            lifecycle_command.extend(["--wait-for-subscriber-topic", topic])
        lifecycle_command.extend(["--minimum-subscribers", "2"])
    lifecycle_command.extend(lifecycle_nodes)
    actions.append(
        ExecuteProcess(
            cmd=lifecycle_command,
            name="lifecycle_activate",
            output="screen",
            additional_env=runtime_environment,
        )
    )
    if record:
        assert recorder_action is not None
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=consumer_action,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(reason=("Isaac consumer closed; finalize recording"))
                        )
                    ],
                )
            )
        )
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=recorder_action,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason=("Recorder closed; stop the recorded run")))
                    ],
                )
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
                default_value=("configs/deployments/isaac_nero_hand2_ros_dual_live_v2.yaml"),
            ),
            DeclareLaunchArgument(
                "local_runtime_binding",
                default_value=("configs/local/workstation2_nv5_ros_v2.yaml"),
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("frames", default_value="0"),
            DeclareLaunchArgument("record", default_value="false"),
            DeclareLaunchArgument("run_id", default_value=""),
            DeclareLaunchArgument("isaac_cpu_affinity", default_value=""),
            OpaqueFunction(function=_processes),
        ]
    )
