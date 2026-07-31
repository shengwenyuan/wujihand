from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import pytest
import yaml

from wujihand.runtime import (
    ConfigRepository,
    common_deployment_projection,
)
from wujihand.runtime.source_lock import sha256_file
from wujihand.specs import RosDeploymentSpec


ROOT = Path(__file__).parents[2]
ROS_FULL = (
    ROOT
    / "configs/deployments/isaac_nero_hand2_ros_dual_live_v2.yaml"
)
ROS_ARMS = (
    ROOT
    / "configs/deployments/"
    "isaac_nero_hand2_ros_dual_arm_only_live_v2.yaml"
)
NATIVE_FULL = (
    ROOT
    / "configs/deployments/isaac_nero_hand2_native_dual_live_v1.yaml"
)
NATIVE_SESSION = (
    ROOT
    / "configs/sessions/isaac_nero_dual_hand2_teleop_v1.yaml"
)
DUAL_SESSION = (
    ROOT / "configs/sessions/isaac_nero_dual_hand2_teleop_v1.yaml"
)
QOS = (
    ROOT
    / "configs/profiles/ros2_jazzy_dual_teleoperation_qos_v1.yaml"
)
LOCAL_EXAMPLE = (
    ROOT
    / "configs/examples/"
    "workstation2_nv5_ros_local_runtime_binding.example.yaml"
)


def ros_mapping() -> dict[str, Any]:
    value = yaml.safe_load(ROS_FULL.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def test_committed_ros_v2_contracts_are_strict_and_closed() -> None:
    repository = ConfigRepository(ROOT)
    full = repository.load_ros_deployment(ROS_FULL)
    arms = repository.load_ros_deployment(ROS_ARMS)
    qos = repository.load_ros_qos_profile(QOS)
    local = repository.load_ros_local_runtime_binding(LOCAL_EXAMPLE)

    assert RosDeploymentSpec.from_mapping(full.to_mapping()) == full
    assert full.execution_owner_process_id == "isaac_consumer"
    assert full.root_namespace == "wujihand/v1/teleop"
    assert {node.process_id for node in full.node_bindings} == {
        "vive_source",
        "glove_source",
        "isaac_consumer",
    }
    assert {process.process_id for process in arms.processes} == {
        "vive_source",
        "isaac_consumer",
    }
    assert qos.policy("tracker_sample").depth == 1
    assert qos.policy("tracker_sample").deadline_ms is None
    assert qos.policy("tracking_lifecycle").durability == "transient_local"
    assert local.ros_domain_id == 57
    assert local.process("isaac_consumer").setup_scripts[-1].endswith(
        "setup.bash"
    )


def test_ros_and_native_common_control_projection_match() -> None:
    repository = ConfigRepository(ROOT)
    native = repository.load_deployment(NATIVE_FULL)
    ros = repository.load_ros_deployment(ROS_FULL)
    native_session = repository.load_session(NATIVE_SESSION)
    dual_session = repository.load_session(DUAL_SESSION)
    mapping_path = repository.resolve_project_path(
        ros.tracking_setup.mapping.path,
        field="tracking mapping",
    )
    digest = sha256_file(mapping_path)

    assert common_deployment_projection(
        native,
        native_session,
        mapping_sha256=digest,
    ) == common_deployment_projection(
        ros,
        dual_session,
        mapping_sha256=digest,
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (
            lambda value: value.update(
                {"execution_owner_process_id": "missing"}
            ),
            "unknown process",
        ),
        (
            lambda value: value["node_bindings"].pop(),
            "cover every non-recorder",
        ),
        (
            lambda value: value["node_bindings"][1].update(
                {"node_name": "vive_source"}
            ),
            "node_name values must be unique",
        ),
        (
            lambda value: value.update({"unexpected": True}),
            "keys differ",
        ),
    ),
)
def test_ros_deployment_rejects_ambiguous_graph(
    mutation: Any,
    message: str,
) -> None:
    value = deepcopy(ros_mapping())
    mutation(value)

    with pytest.raises(ValueError, match=message):
        RosDeploymentSpec.from_mapping(value)
