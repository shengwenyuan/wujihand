from __future__ import annotations

import ast
from pathlib import Path

from wujihand.dataset.camera import load_dataset_camera_projections
from wujihand.dataset.profile import load_mini_dataset_profile
from wujihand.runtime import RosDeploymentResolver
from wujihand_ros2.recording import recording_topics


ROOT = Path(__file__).parents[2]
DEPLOYMENT = "configs/deployments/isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v3.yaml"
LOCAL = "configs/examples/workstation2_nv5_ros_local_runtime_binding.example.yaml"


def _tree(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def _arguments(path: Path) -> set[str]:
    result: set[str] = set()
    for node in ast.walk(_tree(path)):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != "add_argument" or not node.args:
            continue
        value = node.args[0]
        if isinstance(value, ast.Constant) and isinstance(value.value, str):
            result.add(value.value)
    return result


def test_008_deployment_resolves_one_session_owned_camera_and_q54_contract() -> None:
    resolved = RosDeploymentResolver(ROOT).resolve(DEPLOYMENT, local_binding=LOCAL)
    profile_ref = resolved.session.session.dataset_profile
    assert profile_ref is not None
    profile = load_mini_dataset_profile(ROOT, profile_ref.path)

    assert profile.profile_id == profile_ref.expected_id
    assert profile.q54.dimension == 54
    assert tuple(item.logical_id for item in load_dataset_camera_projections(ROOT, profile)) == (
        "scene_rgb",
        "left_wrist_rgb",
        "right_wrist_rgb",
    )
    topics = recording_topics(
        f"/{resolved.deployment.root_namespace}",
        resolved.route_plan,
        include_dataset_facts=True,
    )
    assert len(topics) == 22
    assert len(set(topics)) == len(topics)
    assert all("/operator_preview/" not in topic for topic in topics)


def test_control_and_renderer_clis_expose_no_camera_or_physical_lens_switch() -> None:
    control_args = _arguments(ROOT / "tools/run_isaac_nero_hand2_ros.py")
    renderer_args = _arguments(ROOT / "tools/render_mini_dataset_episode.py")
    preview_args = _arguments(ROOT / "tools/run_isaac_dataset_live_preview.py")

    forbidden = {
        "--camera",
        "--camera-profile",
        "--hfov",
        "--simulation-only",
        "--physical-calibration",
        "--depth",
    }
    assert control_args.isdisjoint(forbidden)
    assert renderer_args == {
        "--run-root",
        "--deployment",
        "--local-runtime-binding",
        "--verify-artifacts",
        "--render-variant",
        "--variant-profile",
    }
    assert preview_args == {
        "--deployment",
        "--local-runtime-binding",
        "--run-id",
        "--run-root",
        "--cpu-affinity",
        "--chain-preflight",
        "--wait-for-node",
        "--verify-artifacts",
    }


def test_008_runner_and_launch_select_dataset_facts_without_online_rgb() -> None:
    runner = (ROOT / "tools/run_isaac_nero_hand2_ros.py").read_text(encoding="utf-8")
    launch = (ROOT / "ros2/wujihand_ros2/launch/dual_teleoperation.launch.py").read_text(
        encoding="utf-8"
    )
    renderer = (ROOT / "src/wujihand/runtime/isaac_dataset_rgb_renderer.py").read_text(
        encoding="utf-8"
    )
    preview = (ROOT / "tools/run_isaac_dataset_live_preview.py").read_text(encoding="utf-8")

    assert "dataset_profile_ref = RESOLVED.session.session.dataset_profile" in runner
    assert "include_synthetic_d405=not DATASET_MODE" in runner
    assert "include_dataset_facts=DATASET_MODE" in runner
    assert "include_synthetic_d405=not dataset_mode" in launch
    assert "include_dataset_facts=dataset_mode" in launch
    assert "--external-preview-required" in launch
    assert "run_isaac_dataset_live_preview.py" in launch
    assert 'DATASET_PREVIEW_CPU_AFFINITY = "16-27"' in launch
    assert '"--wait-for-node"' in launch
    assert 'DATASET_AUXILIARY_CPU_AFFINITY = "28-31"' in launch
    assert 'return ["/usr/bin/taskset", "--cpu-list", cpu_affinity, *command]' in launch
    assert 'OPERATOR_PREVIEW_STATE_TOPIC = "operator_preview/simulation_state"' in runner
    assert 'OPERATOR_PREVIEW_STATE_TOPIC = "operator_preview/simulation_state"' in preview
    assert "operator_preview_state=operator_preview_state" in runner
    assert "ros2_jazzy_dual_teleoperation_dataset_rosbag_qos_v1.yaml" in launch
    assert "timeline.stop()" in renderer
    assert "current_time_step_index" in renderer
    assert "rep.orchestrator.step" in renderer
    assert "delta_time=0.0" in renderer
    assert "pause_timeline=True" in renderer
    assert "simulation-only" in renderer
    assert "physical D405 calibration fallback" in renderer


def test_split_preview_allows_only_recording_owned_unbounded_headless_control() -> None:
    runner = (ROOT / "tools/run_isaac_nero_hand2_ros.py").read_text(encoding="utf-8")

    assert "unbounded_external_preview_recording = (" in runner
    assert "and args.recording_enabled" in runner
    assert "and args.external_preview_required" in runner
    assert (
        "if not args.gui and args.frames == 0 and not unbounded_external_preview_recording:"
    ) in runner


def test_device_free_preview_qualification_is_explicit_and_dataset_ineligible() -> None:
    launch = (ROOT / "ros2/wujihand_ros2/launch/dual_teleoperation.launch.py").read_text(
        encoding="utf-8"
    )
    runner = (ROOT / "tools/run_isaac_nero_hand2_ros.py").read_text(encoding="utf-8")
    fixture = (ROOT / "ros2/wujihand_ros2/test/fixture_sources.py").read_text(encoding="utf-8")
    preview = (ROOT / "tools/run_isaac_dataset_live_preview.py").read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("qualification_fixture", default_value="false")' in launch
    assert '"synthetic_fixture"\n        if qualification_fixture' in launch
    assert 'consumer_command.extend(["--dataset-source-mode", dataset_source_mode])' in launch
    assert "if qualification_fixture:\n            continue" in launch
    assert (
        "DATASET_ELIGIBLE = DATASET_SOURCE_MODE is DatasetSourceMode.LIVE_TELEOPERATION" in runner
    )
    assert "dataset_eligible=DATASET_ELIGIBLE" in runner
    assert 'FIXTURE_PROFILE_ID: Final = "dataset_preview_e2e_aba_v1"' in (
        ROOT / "src/wujihand/application/qualification/dataset_preview_fixture.py"
    ).read_text(encoding="utf-8")
    assert "fixture_profile_sha256()" in fixture
    assert "gc.freeze()" in fixture
    assert fixture.index("gc.freeze()") < fixture.index("scheduler = FixedRateScheduler(")
    assert '"python_gc_frozen_during_run": python_gc_frozen_during_run' in fixture
    assert '"viewport_static_repeat": viewport_static_repeat_passed' in preview


def test_ros_interface_build_declares_all_008_messages() -> None:
    cmake = (ROOT / "ros2/wujihand_interfaces/CMakeLists.txt").read_text(encoding="utf-8")
    for message in (
        "DatasetEpisodeBoundary.msg",
        "DatasetRigidBodyTruth.msg",
        "DatasetKinematicLinkTruth.msg",
        "SimulationStateFrame.msg",
        "OperatorPreviewStateFrame.msg",
    ):
        assert message in cmake

    preview_message = (
        ROOT / "ros2/wujihand_interfaces/msg/OperatorPreviewStateFrame.msg"
    ).read_text(encoding="utf-8")
    assert "DatasetKinematicLinkTruth[<=128] kinematic_links" in preview_message


def test_operator_preview_uses_a_distinct_unrecorded_full_link_transport() -> None:
    runner = (ROOT / "tools/run_isaac_nero_hand2_ros.py").read_text(encoding="utf-8")
    preview = (ROOT / "tools/run_isaac_dataset_live_preview.py").read_text(encoding="utf-8")

    assert "factory=OperatorPreviewStateFrameMessage" in runner
    assert "OperatorPreviewStateFrameMessage," in runner
    assert "OperatorPreviewStateFrameMessage," in preview
    assert "SimulationStateFrameMessage," not in preview
    assert "OPERATOR_PREVIEW_MAX_KINEMATIC_LINKS = 128" in runner
    assert "timeline.stop()" in preview
    assert '"timeline_stopped_for_replay"' in preview
    assert '"local_physics_replay_enabled": False' in preview
    assert '"pose_application_phase": "synchronous_pre_render_transaction"' in preview
    assert 'PREVIEW_POSE_BACKEND = "usd"' in preview
    assert 'PREVIEW_POSE_READBACK_BACKEND = "usd"' in preview
    assert "visual_replay_only=True" in preview
    assert "GLOBAL_EVENT_UPDATE" not in preview
    assert "carb.eventdispatcher" not in preview
    assert "rep.orchestrator.step(" in preview
    assert "rt_subframes=0" in preview
    assert "delta_time=0.0" in preview
    assert "wait_for_render=True" in preview
    assert "PREVIEW_MULTI_TICK_RENDERING = False" in preview
    assert preview.count("RenderingManager.render()") == 1
    assert "restore_dataset_preview_frame" not in preview
    assert "update_articulations_kinematic" not in preview
    assert "physxfabric.force_update" not in preview
    assert "useFabricSceneDelegate" not in preview
    assert "viewport_sync_annotator.get_data" not in preview
