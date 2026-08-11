"""Launch the three-process NV-5 ROS 2 teleoperation graph."""

from __future__ import annotations

import os
import subprocess
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
from wujihand_ros2.recording import recording_topics, source_topics

from wujihand.runtime import (
    RosDeploymentResolver,
    new_run_id,
    parse_cpu_affinity,
    run_root,
)


DATASET_PREVIEW_CPU_AFFINITY = "16-27"
DATASET_AUXILIARY_CPU_AFFINITY = "28-31"
DATASET_QUALIFICATION_CONTROL_FRAMES = 1080
DATASET_QUALIFICATION_SOURCE_FRAMES = 2400
DATASET_QUALIFICATION_PROFILE = "dataset_preview_e2e_aba_v1"


def _taskset_command(command: list[str], *, cpu_affinity: str) -> list[str]:
    return ["/usr/bin/taskset", "--cpu-list", cpu_affinity, *command]


def _processes(context: object) -> list[object]:
    project_root = Path(LaunchConfiguration("project_root").perform(context)).resolve()
    deployment_path = LaunchConfiguration("deployment").perform(context)
    local_path = LaunchConfiguration("local_runtime_binding").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    frames = int(LaunchConfiguration("frames").perform(context))
    record = LaunchConfiguration("record").perform(context).lower() == "true"
    record_qualification = (
        LaunchConfiguration("record_qualification").perform(context).lower() == "true"
    )
    qualification_fixture = (
        LaunchConfiguration("qualification_fixture").perform(context).lower() == "true"
    )
    matched_chain_path = LaunchConfiguration("matched_chain_binding").perform(context).strip()
    record_chain_qualification_path = LaunchConfiguration(
        "record_chain_qualification"
    ).perform(context)
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
    dataset_mode = resolved.session.session.dataset_profile is not None
    hand_revisions = {
        instance.asset.revision
        for instance in resolved.session.instances
        if instance.asset.product == "wuji_hand_2"
    }
    requires_matched_chain = hand_revisions == {"beta1_description_v2026_8_3"}
    if requires_matched_chain and not matched_chain_path:
        raise ValueError("Description 8.3 launch requires matched_chain_binding")
    if matched_chain_path and not requires_matched_chain:
        raise ValueError("matched_chain_binding is valid only for the Description 8.3 entry")
    if record_qualification and (not record or qualification_fixture):
        raise ValueError(
            "record_qualification requires live record:=true without qualification_fixture"
        )
    if requires_matched_chain and record and not (
        record_qualification or qualification_fixture
    ):
        raise ValueError(
            "the Description 8.3 entry remains qualification-only; "
            "set record_qualification:=true"
        )
    split_dataset_preview = bool(record and dataset_mode and gui)
    if qualification_fixture and not split_dataset_preview:
        raise ValueError(
            "dataset preview qualification requires gui:=true record:=true dataset mode"
        )
    if qualification_fixture:
        frames = DATASET_QUALIFICATION_CONTROL_FRAMES
    if split_dataset_preview and isaac_cpu_affinity != "0-15":
        raise ValueError(
            "Workstation2 dataset GUI recording requires isaac_cpu_affinity:=0-15; "
            "the passive preview owns 16-31"
        )
    runtime_environment = {
        "ROS_DOMAIN_ID": str(resolved.local_binding.ros_domain_id),
        "RMW_IMPLEMENTATION": (resolved.local_binding.rmw_implementation),
    }
    if resolved.local_binding.dds_profile is not None:
        runtime_environment["FASTRTPS_DEFAULT_PROFILES_FILE"] = resolved.local_binding.dds_profile
    namespace = f"/{resolved.deployment.root_namespace}"
    current_run_id = requested_run_id or new_run_id()
    current_run_root = (
        project_root / "artifacts/diagnostics/dataset-preview-qualification" / current_run_id
        if qualification_fixture
        else run_root(
            project_root,
            resolved.deployment.report_root,
            current_run_id,
        )
    )
    if record and current_run_root.exists():
        raise FileExistsError(f"recording run directory already exists: {current_run_root}")
    chain_preflight_path = None
    sdk_runtime_environment = runtime_environment
    if requires_matched_chain:
        from wujihand.runtime.wuji_hand2_matched_chain import (
            load_matched_chain_local_binding,
        )

        matched_local = load_matched_chain_local_binding(matched_chain_path)
        inherited_pythonpath = os.environ.get("PYTHONPATH", "")
        sdk_pythonpath = str(matched_local.sdk_module_root)
        if inherited_pythonpath:
            sdk_pythonpath = f"{sdk_pythonpath}:{inherited_pythonpath}"
        sdk_runtime_environment = {
            **runtime_environment,
            "PYTHONPATH": sdk_pythonpath,
        }
        chain_preflight_path = (
            current_run_root / "preflight" / "wuji_hand2_record_chain.json"
        )
        preflight_command = [
            str(matched_local.interpreter),
            str(project_root / "tools/preflight_wuji_hand2_record_chain.py"),
            "--qualification",
            record_chain_qualification_path,
            "--deployment",
            deployment_path,
            "--local-runtime-binding",
            local_path,
            "--matched-chain-binding",
            matched_chain_path,
            "--input",
            "stub" if qualification_fixture else "glove",
            "--output",
            str(chain_preflight_path),
        ]
        completed = subprocess.run(
            preflight_command,
            check=False,
            capture_output=True,
            text=True,
            env={**os.environ, **sdk_runtime_environment},
        )
        if completed.returncode != 0:
            detail = (completed.stdout + completed.stderr).strip()
            raise RuntimeError(f"Hand2 8.3 record-chain preflight failed: {detail}")
        print(completed.stdout, end="", flush=True)
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
        if qualification_fixture:
            continue
        if process_id not in deployment_processes:
            continue
        process = resolved.local_binding.process(process_id)
        node_name = next(
            binding.node_name
            for binding in resolved.deployment.node_bindings
            if binding.process_id == process_id
        )
        lifecycle_nodes.append(f"{namespace}/{node_name}")
        source_command = [
            process.executable,
            "-m",
            module,
            *common_ros_args,
        ]
        if split_dataset_preview:
            source_command = _taskset_command(
                source_command,
                cpu_affinity=DATASET_AUXILIARY_CPU_AFFINITY,
            )
        actions.append(
            ExecuteProcess(
                cmd=source_command,
                name=process_id,
                output="screen",
                sigterm_timeout="10",
                additional_env=(
                    sdk_runtime_environment
                    if process_id == "glove_source"
                    else runtime_environment
                ),
            )
        )
    consumer = resolved.local_binding.process("isaac_consumer")
    consumer_node_name = next(
        binding.node_name
        for binding in resolved.deployment.node_bindings
        if binding.process_id == "isaac_consumer"
    )
    consumer_command = [
        consumer.executable,
        str(project_root / "tools/run_isaac_nero_hand2_ros.py"),
        "--deployment",
        deployment_path,
        "--local-runtime-binding",
        local_path,
        "--gui" if gui and not split_dataset_preview else "--no-gui",
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
        if split_dataset_preview:
            consumer_command.append("--external-preview-required")
        if qualification_fixture:
            consumer_command.extend(["--dataset-source-mode", "synthetic_fixture"])
        elif record_qualification:
            consumer_command.extend(["--dataset-source-mode", "live_qualification"])
    if chain_preflight_path is not None:
        consumer_command.extend(["--chain-preflight", str(chain_preflight_path)])
    consumer_action = ExecuteProcess(
        cmd=consumer_command,
        name="isaac_consumer",
        output="screen",
        sigterm_timeout="20",
        additional_env=sdk_runtime_environment,
    )
    actions.append(consumer_action)
    if split_dataset_preview:
        # Dataset RGB remains offline.  This second Isaac process is a passive
        # latest-state operator view, isolated from the 120/60 Hz owner.
        preview_command = [
            consumer.executable,
            str(project_root / "tools/run_isaac_dataset_live_preview.py"),
            "--deployment",
            deployment_path,
            "--local-runtime-binding",
            local_path,
            "--run-id",
            current_run_id,
            "--run-root",
            str(current_run_root),
            "--cpu-affinity",
            DATASET_PREVIEW_CPU_AFFINITY,
            "--wait-for-node",
            f"{namespace}/{consumer_node_name}",
        ]
        if chain_preflight_path is not None:
            preview_command.extend(
                ["--chain-preflight", str(chain_preflight_path)]
            )
        actions.append(
            ExecuteProcess(
                cmd=preview_command,
                name="dataset_live_preview",
                output="screen",
                sigterm_timeout="12",
                additional_env=runtime_environment,
            )
        )
    recorder_action = None
    if record:
        recorder_topics = recording_topics(
            namespace,
            resolved.route_plan,
            include_synthetic_d405=not dataset_mode,
            include_dataset_facts=dataset_mode,
        )
        rosbag_qos_name = (
            "ros2_jazzy_dual_teleoperation_dataset_rosbag_qos_v1.yaml"
            if dataset_mode
            else "ros2_jazzy_dual_teleoperation_d405_rosbag_qos_v1.yaml"
        )
        recorder_command = [
            "/usr/bin/python3",
            str(project_root / "tools/run_rosbag_recording.py"),
            "--run-root",
            str(current_run_root),
            "--run-id",
            current_run_id,
            "--qos-profile",
            str(project_root / "configs/profiles" / rosbag_qos_name),
        ]
        for topic in recorder_topics:
            recorder_command.extend(["--topic", topic])
        if split_dataset_preview:
            recorder_command = _taskset_command(
                recorder_command,
                cpu_affinity=DATASET_AUXILIARY_CPU_AFFINITY,
            )
        recorder_action = ExecuteProcess(
            cmd=recorder_command,
            name="rosbag2",
            output="screen",
            sigterm_timeout="30",
            additional_env=runtime_environment,
        )
        actions.append(recorder_action)
    if qualification_fixture:
        actions.append(
            ExecuteProcess(
                cmd=_taskset_command(
                    [
                        "/usr/bin/python3",
                        str(project_root / "ros2/wujihand_ros2/test/fixture_sources.py"),
                        "--deployment",
                        deployment_path,
                        "--local-runtime-binding",
                        local_path,
                        "--profile",
                        DATASET_QUALIFICATION_PROFILE,
                        "--run-id",
                        current_run_id,
                        "--frames",
                        str(DATASET_QUALIFICATION_SOURCE_FRAMES),
                        "--minimum-subscribers",
                        "2",
                        "--receipt",
                        str(current_run_root / "fixture" / "receipt.json"),
                    ],
                    cpu_affinity=DATASET_AUXILIARY_CPU_AFFINITY,
                ),
                name="dataset_preview_fixture",
                output="screen",
                sigterm_timeout="10",
                additional_env=runtime_environment,
            )
        )
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
    if not qualification_fixture:
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
            DeclareLaunchArgument("record_qualification", default_value="false"),
            DeclareLaunchArgument("run_id", default_value=""),
            DeclareLaunchArgument("isaac_cpu_affinity", default_value=""),
            DeclareLaunchArgument("qualification_fixture", default_value="false"),
            DeclareLaunchArgument("matched_chain_binding", default_value=""),
            DeclareLaunchArgument(
                "record_chain_qualification",
                default_value=(
                    "configs/qualifications/"
                    "isaac_nero_hand2_record_chain_v2026_8_3_v1.yaml"
                ),
            ),
            OpaqueFunction(function=_processes),
        ]
    )
