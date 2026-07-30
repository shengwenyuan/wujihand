"""Cross-layer contracts for the committed NV-4 DeploymentSpec templates."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from wujihand.runtime import (
    ConfigRepository,
    DeploymentResolver,
    NATIVE_DUAL_DEBUG_RUNTIME_COMPONENT,
    NATIVE_DUAL_RUNTIME_COMPONENT,
    build_native_dual_runtime_plan,
    build_openvr_producer_launch,
)
from wujihand.specs import LocalDeviceBindingSpec
from wujihand.specs import (
    NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT,
)


ROOT = Path(__file__).parents[2]
DEPLOYMENTS = ROOT / "configs/deployments"
DEPLOYMENT_NAMES = (
    "isaac_nero_hand2_native_dual_live_v1.yaml",
    "isaac_nero_hand2_left_single_live_v1.yaml",
    "isaac_nero_hand2_right_single_live_v1.yaml",
    "isaac_nero_hand2_right_single_debug_v1.yaml",
)
LOCAL_BINDING_EXAMPLE = (
    ROOT
    / "configs/examples/workstation2_nv4_local_device_binding.example.yaml"
)


def local_binding() -> LocalDeviceBindingSpec:
    return LocalDeviceBindingSpec.from_mapping(
        {
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
                    "device_identity": "TRACKER-LEFT-PRIVATE",
                    "endpoint": "udp://127.0.0.1:49154",
                    "calibration_id": "tracker_left_handle_pending_v1",
                },
                {
                    "binding_key": "tracker_right",
                    "source_kind": "vive_tracker",
                    "device_identity": "TRACKER-RIGHT-PRIVATE",
                    "endpoint": "udp://127.0.0.1:49155",
                    "calibration_id": "tracker_right_handle_pending_v1",
                },
                {
                    "binding_key": "glove_left",
                    "source_kind": "wuji_glove",
                    "device_identity": "GLOVE-LEFT-PRIVATE",
                    "endpoint": "wuji://192.168.1.100",
                    "calibration_id": "glove_left_sdk_default_pending_v1",
                },
                {
                    "binding_key": "glove_right",
                    "source_kind": "wuji_glove",
                    "device_identity": "GLOVE-RIGHT-PRIVATE",
                    "endpoint": "wuji://192.168.1.101",
                    "calibration_id": "glove_right_sdk_default_pending_v1",
                },
            ],
        }
    )


@pytest.mark.parametrize("name", DEPLOYMENT_NAMES)
def test_nv4_deployments_resolve_around_the_same_five_layer_session(
    name: str,
) -> None:
    resolved = DeploymentResolver(ROOT).resolve(
        DEPLOYMENTS / name,
        local_binding=local_binding(),
    )

    assert resolved.session.session.session_id == (
        "isaac_nero_dual_hand2_native_teleop_v1"
    )
    assert resolved.session.session.runtime_role == "teleop_consumer"
    assert (
        resolved.session.session.runtime.transport_contract
        == NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT
    )
    assert len(resolved.session.session.runtime.control_layouts) == 4
    assert len(resolved.deployment.control_bindings) == 4
    assert resolved.mapping.mapping_id == "vive_tracker_workcell_workstation2_v4"
    assert resolved.mapping.translation_scale == pytest.approx(1.0)
    assert resolved.mapping.max_translation_delta_m == pytest.approx(0.4)
    assert not resolved.tracking_qualified
    assert len(resolved.deployment_hash) == 64
    assert len(resolved.local_binding_hash) == 64
    assert resolved.process("openvr_producer").local_binding is not None


def test_live_session_reuses_lower_four_layers_without_mutating_qualification() -> None:
    repository = ConfigRepository(ROOT)
    resolver = DeploymentResolver(ROOT)
    live = resolver.resolve(
        DEPLOYMENTS / DEPLOYMENT_NAMES[0],
        local_binding=local_binding(),
    ).session
    qualification = resolver.session_resolver.resolve(
        ROOT
        / "configs/sessions/"
        "isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml"
    )

    assert qualification.session.runtime_role == "simulation"
    assert qualification.session.runtime.transport_contract is None
    assert live.assembly_path == qualification.assembly_path
    assert live.workcell_path == qualification.workcell_path
    assert {
        (instance.instance_id, instance.asset.asset_id, instance.binding.binding_id)
        for instance in live.instances
    } == {
        (instance.instance_id, instance.asset.asset_id, instance.binding.binding_id)
        for instance in qualification.instances
    }
    hashes = dict(live.referenced_file_hashes)
    assert (
        "configs/profiles/"
        "isaac_nero_hand2_native_dual_teleoperation_v1.yaml"
    ) in hashes
    assert (
        "configs/profiles/"
        "isaac_nero_dual_tabletop_qualification_v1.yaml"
    ) in hashes
    profile = repository.load_native_dual_teleoperation_profile(
        live.session.runtime.compatibility_profile
        or ""
    )
    assert (
        profile.transport_contract
        == NATIVE_DUAL_TELEOPERATION_TRANSPORT_CONTRACT
    )


def test_default_and_diagnostic_deployments_have_explicit_source_ownership() -> None:
    resolver = DeploymentResolver(ROOT)
    local = local_binding()
    default = resolver.resolve(
        DEPLOYMENTS / DEPLOYMENT_NAMES[0],
        local_binding=local,
    )
    left = resolver.resolve(
        DEPLOYMENTS / DEPLOYMENT_NAMES[1],
        local_binding=local,
    )
    right = resolver.resolve(
        DEPLOYMENTS / DEPLOYMENT_NAMES[2],
        local_binding=local,
    )

    assert {source.source.kind for source in default.sources} == {
        "vive_tracker",
        "wuji_glove",
    }
    assert left.source("arm_hold_right").local_binding is None
    assert left.source("hand_rest_right").local_binding is None
    assert left.source("tracker_left").local_binding is not None
    assert right.source("arm_hold_left").local_binding is None
    assert right.source("hand_rest_left").local_binding is None
    assert right.source("tracker_right").local_binding is not None
    assert {
        default.session.session_hash,
        left.session.session_hash,
        right.session.session_hash,
    } == {default.session.session_hash}
    assert len(
        {default.deployment_hash, left.deployment_hash, right.deployment_hash}
    ) == 3
    assert build_native_dual_runtime_plan(default).live_sides == (
        "left",
        "right",
    )
    assert build_native_dual_runtime_plan(left).live_sides == ("left",)
    assert build_native_dual_runtime_plan(right).live_sides == ("right",)
    assert build_native_dual_runtime_plan(default).arm_reset_key is None


def test_right_debug_deployment_declares_keyboard_arm_reset_capability() -> None:
    resolved = DeploymentResolver(ROOT).resolve(
        DEPLOYMENTS / DEPLOYMENT_NAMES[3],
        local_binding=local_binding(),
    )
    plan = build_native_dual_runtime_plan(resolved)

    assert (
        resolved.process("isaac_runtime").process.component_id
        == NATIVE_DUAL_DEBUG_RUNTIME_COMPONENT
    )
    assert plan.live_sides == ("right",)
    assert plan.arm_reset_key == "R"
    normal = DeploymentResolver(ROOT).resolve(
        DEPLOYMENTS / DEPLOYMENT_NAMES[2],
        local_binding=local_binding(),
    )
    assert (
        normal.process("isaac_runtime").process.component_id
        == NATIVE_DUAL_RUNTIME_COMPONENT
    )


def test_resolved_snapshot_redacts_local_identity_and_endpoint() -> None:
    resolved = DeploymentResolver(ROOT).resolve(
        DEPLOYMENTS / DEPLOYMENT_NAMES[0],
        local_binding=local_binding(),
    )

    assert "TRACKER-LEFT-PRIVATE" not in resolved.snapshot_json
    assert "192.168.1.100" not in resolved.snapshot_json
    snapshot = resolved.to_mapping()
    assert snapshot["deployment_hash"] == resolved.deployment_hash
    assert snapshot["local_binding_hash"] == resolved.local_binding_hash


def test_committed_local_binding_example_is_strictly_parseable() -> None:
    example = ConfigRepository(ROOT).load_local_device_binding(
        LOCAL_BINDING_EXAMPLE
    )

    assert example.binding_id == "workstation2_nv4_v1"
    assert {
        process.process_id for process in example.processes
    } == {"openvr_producer"}
    assert {source.binding_key for source in example.sources} == {
        "tracker_left",
        "tracker_right",
        "glove_left",
        "glove_right",
    }


def test_local_binding_changes_only_its_own_hash() -> None:
    resolver = DeploymentResolver(ROOT)
    path = DEPLOYMENTS / DEPLOYMENT_NAMES[0]
    baseline = resolver.resolve(path, local_binding=local_binding())
    sources = list(local_binding().sources)
    sources[0] = replace(sources[0], device_identity="TRACKER-LEFT-REPLACED")
    changed = resolver.resolve(
        path,
        local_binding=replace(local_binding(), sources=tuple(sources)),
    )

    assert changed.deployment_hash == baseline.deployment_hash
    assert changed.session.session_hash == baseline.session.session_hash
    assert changed.local_binding_hash != baseline.local_binding_hash


def test_resolver_rejects_missing_or_wrong_local_source_kind() -> None:
    resolver = DeploymentResolver(ROOT)
    path = DEPLOYMENTS / DEPLOYMENT_NAMES[0]
    binding = local_binding()
    with pytest.raises(ValueError, match="missing key"):
        resolver.resolve(
            path,
            local_binding=replace(binding, sources=binding.sources[1:]),
        )

    wrong = list(binding.sources)
    wrong[0] = replace(wrong[0], source_kind="wuji_glove")
    with pytest.raises(ValueError, match="does not match"):
        resolver.resolve(
            path,
            local_binding=replace(binding, sources=tuple(wrong)),
        )


def test_resolver_rejects_cross_side_control_binding(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    resolver = DeploymentResolver(ROOT)
    path = DEPLOYMENTS / DEPLOYMENT_NAMES[0]
    original = resolver.repository.load_deployment(path)
    bindings = list(original.control_bindings)
    bindings[0] = replace(bindings[0], source_id="tracker_right")
    bindings[2] = replace(bindings[2], source_id="tracker_left")
    swapped = replace(original, control_bindings=tuple(bindings))
    monkeypatch.setattr(
        resolver.repository,
        "load_deployment",
        lambda _path: swapped,
    )

    with pytest.raises(ValueError, match="does not match target side"):
        resolver.resolve(path, local_binding=local_binding())


@pytest.mark.parametrize(
    ("name", "expected_sides"),
    (
        (DEPLOYMENT_NAMES[0], ("left", "right")),
        (DEPLOYMENT_NAMES[1], ("left",)),
        (DEPLOYMENT_NAMES[2], ("right",)),
        (DEPLOYMENT_NAMES[3], ("right",)),
    ),
)
def test_openvr_process_launch_is_compiled_from_deployment_sources(
    name: str,
    expected_sides: tuple[str, ...],
) -> None:
    resolved = DeploymentResolver(ROOT).resolve(
        DEPLOYMENTS / name,
        local_binding=local_binding(),
    )

    launch = build_openvr_producer_launch(
        resolved,
        ROOT,
        producer_instance="openvr_fixture",
        transport_epoch=4,
    )

    assert tuple(stream.side for stream in launch.streams) == expected_sides
    assert launch.command[:2] == (
        "/opt/openvr/bin/python",
        (ROOT / "tools/stream_vive_trackers_udp.py").as_posix(),
    )
    assert launch.transport_epoch == 4
    rebound = launch.next_epoch()
    assert rebound.transport_epoch == 5
    assert rebound.previous_transport_epoch == 4
    assert "--previous-transport-epoch" in rebound.command


def test_resolver_rejects_missing_managed_process_binding() -> None:
    resolver = DeploymentResolver(ROOT)

    with pytest.raises(ValueError, match="exactly cover managed processes"):
        resolver.resolve(
            DEPLOYMENTS / DEPLOYMENT_NAMES[0],
            local_binding=replace(local_binding(), processes=()),
        )
