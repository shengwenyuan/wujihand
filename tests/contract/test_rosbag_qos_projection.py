from __future__ import annotations

from pathlib import Path

import yaml

from wujihand.runtime import ConfigRepository, RosDeploymentResolver
from wujihand_ros2.recording import recording_topics


ROOT = Path(__file__).parents[2]


def test_rosbag_input_qos_is_a_projection_of_committed_profile() -> None:
    profile = ConfigRepository(ROOT).load_ros_qos_profile(
        "configs/profiles/ros2_jazzy_dual_teleoperation_d405_qos_v1.yaml"
    )
    projection = yaml.safe_load(
        (ROOT / "configs/profiles/ros2_jazzy_dual_teleoperation_d405_rosbag_qos_v1.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert isinstance(projection, dict)
    resolved = RosDeploymentResolver(ROOT).resolve(
        "configs/deployments/isaac_nero_hand2_ros_dual_live_v2.yaml",
        local_binding=("configs/examples/workstation2_nv5_ros_local_runtime_binding.example.yaml"),
    )
    assert set(projection) == set(
        recording_topics(
            f"/{resolved.deployment.root_namespace}",
            resolved.route_plan,
            include_synthetic_d405=True,
        )
    )
    for topic, values in projection.items():
        channel = _channel(topic)
        policy = profile.policy(channel)
        assert values == {
            "history": policy.history,
            "depth": policy.depth,
            "reliability": policy.reliability,
            "durability": policy.durability,
        }


def test_dataset_rosbag_qos_closes_rgb_free_fact_allowlist() -> None:
    profile = ConfigRepository(ROOT).load_ros_qos_profile(
        "configs/profiles/ros2_jazzy_dual_teleoperation_dataset_qos_v1.yaml"
    )
    projection = yaml.safe_load(
        (
            ROOT
            / "configs/profiles/ros2_jazzy_dual_teleoperation_dataset_rosbag_qos_v1.yaml"
        ).read_text(encoding="utf-8")
    )
    assert isinstance(projection, dict)
    resolved = RosDeploymentResolver(ROOT).resolve(
        (
            "configs/deployments/"
            "isaac_nero_hand2_ros_dual_tframe_gripper_flange_collision_proxy_"
            "triview_q54_self_collision_v1.yaml"
        ),
        local_binding=(
            "configs/examples/workstation2_nv5_ros_local_runtime_binding.example.yaml"
        ),
    )
    topics = recording_topics(
        f"/{resolved.deployment.root_namespace}",
        resolved.route_plan,
        include_dataset_facts=True,
    )
    assert set(projection) == set(topics)
    assert not any("/wrist_camera/" in topic or topic in {"/tf", "/tf_static"} for topic in topics)
    assert topics[-2:] == (
        "/wujihand/v1/teleop/dataset/episode_boundary",
        "/wujihand/v1/teleop/dataset/simulation_state",
    )
    for topic, values in projection.items():
        policy = profile.policy(_channel(topic))
        assert values == {
            "history": policy.history,
            "depth": policy.depth,
            "reliability": policy.reliability,
            "durability": policy.durability,
        }


def _channel(topic: str) -> str:
    if topic.endswith("/input/tracker/lifecycle"):
        return "tracking_lifecycle"
    if "/input/tracker/" in topic:
        return "tracker_sample"
    if "/input/glove/" in topic:
        return "glove_observation"
    if topic.endswith("/command"):
        return "route_command"
    if topic.endswith("/feedback"):
        return "route_feedback"
    if topic.endswith("/safety"):
        return "safety_event"
    if topic.endswith("/runtime/tick"):
        return "trace_event"
    if topic.endswith("/scene/rigid_body_state"):
        return "scene_state"
    if topic.endswith("/recording/status"):
        return "run_status"
    if topic.endswith("/dataset/episode_boundary"):
        return "dataset_boundary"
    if topic.endswith("/dataset/simulation_state"):
        return "dataset_state"
    if topic.endswith("/color/image_raw") or topic.endswith("/depth/image_raw"):
        return "camera_image"
    if topic.endswith("/camera_info"):
        return "camera_info"
    if topic.endswith("/frame_truth"):
        return "camera_truth"
    if topic == "/tf":
        return "tf_dynamic"
    if topic == "/tf_static":
        return "tf_static"
    raise AssertionError(f"unmapped recording topic: {topic}")
