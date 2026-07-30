from __future__ import annotations

from copy import deepcopy
from typing import Any

import pytest

from wujihand.specs import DeploymentSpec, LocalDeviceBindingSpec


def _ref(path: str, expected_id: str) -> dict[str, str]:
    return {"path": path, "expected_id": expected_id}


def _deployment() -> dict[str, Any]:
    return {
        "schema": "wujihand.deployment.v1",
        "deployment_id": "native_dual_live_v1",
        "session": _ref("configs/sessions/dual.yaml", "dual"),
        "local_binding_id": "workstation2_nv4_v1",
        "tracking_setup": {
            "setup_revision": "workstation2_standing_pending_v1",
            "tracking_frame": "vive_tracking",
            "qualification_status": "pending",
            "mapping": _ref(
                "configs/calibrations/workstation2_v3.yaml",
                "workstation2_v3",
            ),
        },
        "processes": [
            {
                "process_id": "openvr_producer",
                "component_id": "openvr_dual_tracker_producer",
                "lifecycle": "managed",
                "depends_on": [],
            },
            {
                "process_id": "isaac_runtime",
                "component_id": "isaac_native_dual_runtime",
                "lifecycle": "in_process",
                "depends_on": ["openvr_producer"],
            },
        ],
        "sources": [
            {
                "source_id": "tracker_left",
                "kind": "vive_tracker",
                "side": "left",
                "logical_role": "operator_left",
                "process_id": "openvr_producer",
                "local_binding_key": "tracker_left",
            },
            {
                "source_id": "glove_left",
                "kind": "wuji_glove",
                "side": "left",
                "logical_role": "glove_left",
                "process_id": "isaac_runtime",
                "local_binding_key": "glove_left",
            },
        ],
        "control_bindings": [
            {
                "instance_id": "nero_left",
                "group_id": "arm_joints",
                "source_id": "tracker_left",
            },
            {
                "instance_id": "hand_left",
                "group_id": "finger_joints",
                "source_id": "glove_left",
            },
        ],
        "report_root": "artifacts/runs/nv4",
    }


def _local_binding() -> dict[str, Any]:
    return {
        "schema": "wujihand.local_device_binding.v1",
        "binding_id": "workstation2_nv4_v1",
        "host_id": "workstation2",
        "processes": [
            {
                "process_id": "openvr_producer",
                "executable": "/opt/openvr/bin/python",
                "environment_id": "openvr_fixture_v1",
            }
        ],
        "sources": [
            {
                "binding_key": "tracker_left",
                "source_kind": "vive_tracker",
                "device_identity": "TRACKER-LEFT",
                "endpoint": "udp://127.0.0.1:49154",
                "calibration_id": "tracker_left_handle_pending_v1",
            },
            {
                "binding_key": "glove_left",
                "source_kind": "wuji_glove",
                "device_identity": "GLOVE-LEFT",
                "endpoint": "wuji://192.168.1.100",
                "calibration_id": "glove_left_sdk_default_pending_v1",
            },
        ],
    }


def test_deployment_and_local_binding_round_trip() -> None:
    deployment = DeploymentSpec.from_mapping(_deployment())
    local_binding = LocalDeviceBindingSpec.from_mapping(_local_binding())

    assert DeploymentSpec.from_mapping(deployment.to_mapping()) == deployment
    assert (
        LocalDeviceBindingSpec.from_mapping(local_binding.to_mapping())
        == local_binding
    )
    assert deployment.source("tracker_left").logical_role == "operator_left"
    assert local_binding.process("openvr_producer").environment_id == (
        "openvr_fixture_v1"
    )
    assert local_binding.source("glove_left").source_kind == "wuji_glove"


def test_deployment_rejects_extra_fields_and_unsafe_report_path() -> None:
    extra = _deployment()
    extra["side"] = "left"
    with pytest.raises(ValueError, match="unexpected"):
        DeploymentSpec.from_mapping(extra)

    unsafe = _deployment()
    unsafe["report_root"] = "../artifacts"
    with pytest.raises(ValueError, match="safe project-relative"):
        DeploymentSpec.from_mapping(unsafe)


def test_live_and_fixture_sources_have_explicit_local_binding_semantics() -> None:
    missing_live_binding = _deployment()
    missing_live_binding["sources"][0]["local_binding_key"] = None
    with pytest.raises(ValueError, match="required for live"):
        DeploymentSpec.from_mapping(missing_live_binding)

    fixture_with_device = _deployment()
    fixture_with_device["sources"][0]["kind"] = "arm_hold_fixture"
    with pytest.raises(ValueError, match="must be null for fixture"):
        DeploymentSpec.from_mapping(fixture_with_device)


def test_deployment_sources_do_not_require_a_glove_route() -> None:
    arm_only = _deployment()
    hand = arm_only["sources"][1]
    hand.update(
        {
            "source_id": "hand_rest_left",
            "kind": "hand_rest_fixture",
            "logical_role": "hand_rest_left",
            "local_binding_key": None,
        }
    )
    arm_only["control_bindings"][1]["source_id"] = "hand_rest_left"

    deployment = DeploymentSpec.from_mapping(arm_only)

    assert {source.kind for source in deployment.sources} == {
        "vive_tracker",
        "hand_rest_fixture",
    }


def test_deployment_rejects_incomplete_or_duplicate_control_bindings() -> None:
    missing = _deployment()
    missing["control_bindings"].pop()
    with pytest.raises(ValueError, match="exactly cover"):
        DeploymentSpec.from_mapping(missing)

    duplicate_route = _deployment()
    repeated = deepcopy(duplicate_route["control_bindings"][0])
    repeated["source_id"] = "glove_left"
    duplicate_route["control_bindings"][1] = repeated
    with pytest.raises(ValueError, match="route values must be unique"):
        DeploymentSpec.from_mapping(duplicate_route)


def test_deployment_process_graph_must_be_closed_and_acyclic() -> None:
    unknown = _deployment()
    unknown["processes"][1]["depends_on"] = ["missing"]
    with pytest.raises(ValueError, match="unknown processes"):
        DeploymentSpec.from_mapping(unknown)

    cyclic = _deployment()
    cyclic["processes"][0]["depends_on"] = ["isaac_runtime"]
    with pytest.raises(ValueError, match="acyclic"):
        DeploymentSpec.from_mapping(cyclic)


def test_local_binding_rejects_fixture_and_duplicate_device_keys() -> None:
    fixture = _local_binding()
    fixture["sources"][0]["source_kind"] = "arm_hold_fixture"
    with pytest.raises(ValueError, match="source_kind"):
        LocalDeviceBindingSpec.from_mapping(fixture)

    duplicate = _local_binding()
    duplicate["sources"][1]["binding_key"] = "tracker_left"
    with pytest.raises(ValueError, match="binding_key values must be unique"):
        LocalDeviceBindingSpec.from_mapping(duplicate)

    relative = _local_binding()
    relative["processes"][0]["executable"] = "python"
    with pytest.raises(ValueError, match="absolute path"):
        LocalDeviceBindingSpec.from_mapping(relative)
