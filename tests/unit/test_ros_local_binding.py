from __future__ import annotations

from wujihand.runtime import (
    RosProcessEnvironment,
    build_ros_local_runtime_binding,
)
from wujihand.specs import LocalDeviceBindingSpec


def test_ros_local_binding_reuses_only_explicit_device_facts() -> None:
    native = LocalDeviceBindingSpec.from_mapping(
        {
            "schema": "wujihand.local_device_binding.v1",
            "binding_id": "native_v1",
            "host_id": "workstation2",
            "processes": [],
            "sources": [
                {
                    "binding_key": "tracker_left",
                    "source_kind": "vive_tracker",
                    "device_identity": "tracker-left",
                    "endpoint": "openvr://runtime",
                    "calibration_id": "tracker_left_v1",
                }
            ],
        }
    )
    setup = (
        "/opt/ros/jazzy/setup.bash",
        "/workspace/install/setup.bash",
    )
    source = RosProcessEnvironment(
        executable="/venv/bin/python",
        environment_id="source_v1",
        setup_scripts=setup,
    )
    binding = build_ros_local_runtime_binding(
        native,
        binding_id="ros_v2",
        ros_domain_id=57,
        rmw_implementation="rmw_fastrtps_cpp",
        vive=source,
        glove=source,
        isaac=RosProcessEnvironment(
            executable="/isaac/bin/python",
            environment_id="isaac_v1",
            setup_scripts=setup,
        ),
    )

    assert binding.host_id == native.host_id
    assert binding.sources == native.sources
    assert {process.process_id for process in binding.processes} == {
        "vive_source",
        "glove_source",
        "isaac_consumer",
    }
