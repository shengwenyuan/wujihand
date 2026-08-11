from __future__ import annotations

from pathlib import Path

from wujihand.domain import HAND2_LAYOUT_IDS, HandSide
from wujihand.runtime.wuji_hand2_matched_chain import (
    MatchedChainPreflightReceipt,
    WujiSdkRuntimeFacts,
)
from wujihand.runtime.wuji_hand2_record_chain import (
    load_record_chain_qualification_policy,
    preflight_wuji_hand2_record_chain,
)
from wujihand.specs import RosLocalRuntimeBindingSpec


ROOT = Path(__file__).resolve().parents[2]
POLICY = (
    ROOT / "configs/qualifications/isaac_nero_hand2_record_chain_v2026_8_3_v1.yaml"
)
DEPLOYMENT = (
    ROOT
    / "configs/deployments/"
    "isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v2026_8_3_v1.yaml"
)


class _Manager:
    pass


def _calibration_id(side: HandSide) -> str:
    digest = "a" * 12 if side is HandSide.LEFT else "b" * 12
    return f"wuji_sdk.user.u_fixture.{side.value}.urdf_{digest}.sdk_2026.8.3"


def _runtime_binding(interpreter: Path) -> RosLocalRuntimeBindingSpec:
    return RosLocalRuntimeBindingSpec.from_mapping(
        {
            "schema": "wujihand.ros_local_runtime_binding.v2",
            "binding_id": "workstation2_nv5_ros_v2",
            "host_id": "workstation2",
            "ros_domain_id": 57,
            "rmw_implementation": "rmw_fastrtps_cpp",
            "dds_profile": None,
            "processes": [
                {
                    "process_id": "vive_source",
                    "executable": "/fixture/openvr-python",
                    "environment_id": "fixture_openvr_v1",
                    "setup_scripts": ["/opt/ros/jazzy/setup.bash"],
                },
                {
                    "process_id": "glove_source",
                    "executable": str(interpreter),
                    "environment_id": "fixture_sdk_2026_8_3_glove_v1",
                    "setup_scripts": ["/opt/ros/jazzy/setup.bash"],
                },
                {
                    "process_id": "isaac_consumer",
                    "executable": str(interpreter),
                    "environment_id": "fixture_sdk_2026_8_3_isaac_v1",
                    "setup_scripts": ["/opt/ros/jazzy/setup.bash"],
                },
            ],
            "sources": [
                {
                    "binding_key": "tracker_left",
                    "source_kind": "vive_tracker",
                    "device_identity": "LHR_LEFT_FIXTURE",
                    "endpoint": "openvr://runtime",
                    "calibration_id": "tracker_left_fixture_v1",
                },
                {
                    "binding_key": "tracker_right",
                    "source_kind": "vive_tracker",
                    "device_identity": "LHR_RIGHT_FIXTURE",
                    "endpoint": "openvr://runtime",
                    "calibration_id": "tracker_right_fixture_v1",
                },
                {
                    "binding_key": "glove_left",
                    "source_kind": "wuji_glove",
                    "device_identity": "WG_LEFT_FIXTURE",
                    "endpoint": "wuji://left",
                    "calibration_id": _calibration_id(HandSide.LEFT),
                },
                {
                    "binding_key": "glove_right",
                    "source_kind": "wuji_glove",
                    "device_identity": "WG_RIGHT_FIXTURE",
                    "endpoint": "wuji://right",
                    "calibration_id": _calibration_id(HandSide.RIGHT),
                },
            ],
        }
    )


def _receipt(side: HandSide, runtime: WujiSdkRuntimeFacts) -> MatchedChainPreflightReceipt:
    return MatchedChainPreflightReceipt(
        qualification_id="wuji_hand2_sdk_2026_8_3_description_v2026_8_3_v1",
        binding_id="fixture_matched_chain_v1",
        side=side,
        input_mode="stub",
        calibration_id=_calibration_id(side),
        serial_number=f"WG_{side.value.upper()}_FIXTURE",
        sdk_version="2026.8.3",
        sdk_module_path=runtime.module_path,
        sdk_wheel_sha256="8" * 64,
        sdk_user_id="u_fixture",
        sdk_user_display_name="Fixture",
        calibrated_urdf_path=Path(f"/fixture/{side.value}_hand.urdf"),
        calibrated_urdf_sha256=("a" if side is HandSide.LEFT else "b") * 64,
        description_release="v2026.8.3",
        hand2_model_revision="v2026.7.23",
        description_commit="8" * 40,
        description_source="wuji-description-v2026-8-3",
        description_artifact_path=(
            f"hand2/hand2_beta1/body/usd/{side.value}/wujihand2.usd"
        ),
        description_artifact_sha256="8" * 64,
        session_path=(
            f"configs/sessions/isaac_hand2_{side.value}_glove_qualification_"
            "v2026_8_3_v1.yaml"
        ),
        session_id=(
            f"isaac_hand2_{side.value}_glove_qualification_v2026_8_3_v1"
        ),
        session_hash="0" * 64,
        binding_root=f"{side.value[0]}_wrist",
        layout_id=HAND2_LAYOUT_IDS[side.value],
        studio_processes=(),
    )


def test_committed_record_chain_policy_is_qualification_only() -> None:
    policy = load_record_chain_qualification_policy(POLICY)

    assert policy.description.release == "v2026.8.3"
    assert policy.nero.attachment.quat_wxyz == (
        0.7071067811865476,
        0.0,
        0.7071067811865475,
        0.0,
    )
    assert policy.description.root_orientation_compensation(HandSide.LEFT) == (
        0.0,
        0.7071067811865476,
        -0.7071067811865475,
        0.0,
    )
    assert policy.description.root_orientation_compensation(HandSide.RIGHT) == (
        0.0,
        0.7071067811865476,
        0.7071067811865475,
        0.0,
    )
    assert policy.required_sdk_processes == ("glove_source", "isaac_consumer")


def test_record_chain_preflight_closes_both_hands_and_sdk_processes(
    tmp_path: Path,
    monkeypatch: object,
) -> None:
    interpreter = tmp_path / "isaac-python"
    overlay = tmp_path / "overlay"
    wheel = tmp_path / "wuji-sdk.whl"
    models = tmp_path / "models"
    runtime = WujiSdkRuntimeFacts(
        distribution_version="2026.8.3",
        module_version="2026.8.3",
        module_path=overlay / "wuji_sdk/__init__.py",
        executable_path=interpreter,
    )
    matched_local = tmp_path / "matched.yaml"
    matched_local.write_text(
        f"""schema: wujihand.wuji_hand2_matched_chain_local_binding.v1
binding_id: fixture_matched_chain_v1
interpreter: {interpreter}
sdk_module_root: {overlay}
sdk_wheel: {wheel}
user:
  user_id: u_fixture
  display_name: Fixture
  models_dir: {models}
hands:
  left:
    serial_number: WG_LEFT_FIXTURE
    urdf_path: {models / 'left_hand.urdf'}
    urdf_sha256: {'a' * 64}
  right:
    serial_number: WG_RIGHT_FIXTURE
    urdf_path: {models / 'right_hand.urdf'}
    urdf_sha256: {'b' * 64}
""",
        encoding="utf-8",
    )

    import wujihand.runtime.wuji_hand2_record_chain as record_chain

    monkeypatch.setattr(
        record_chain,
        "preflight_wuji_hand2_matched_chain",
        lambda *args, side, **kwargs: _receipt(side, runtime),
    )
    receipt = preflight_wuji_hand2_record_chain(
        ROOT,
        qualification_path=POLICY,
        deployment_path=DEPLOYMENT,
        local_runtime_binding_path=_runtime_binding(interpreter),
        matched_chain_binding_path=matched_local,
        input_mode="stub",
        sdk_runtime=runtime,
        user_manager=_Manager(),
        verify_artifacts=False,
    )

    assert receipt.description_release == "v2026.8.3"
    assert tuple(item.process_id for item in receipt.process_receipts) == (
        "glove_source",
        "isaac_consumer",
    )
    mapping = receipt.to_mapping()
    assert set(mapping["hands"]) == {"left", "right"}
    assert mapping["dataset"] == {
        "profile_id": "isaac_nero_hand2_triview_q54_mini_dataset_v1",
        "q54_profile_id": "isaac_nero_hand2_q54_dataset_v1",
        "source_mode": "synthetic_fixture",
        "qualification_only": True,
        "dataset_eligible": False,
    }
    assert set(mapping["deployment"]) == {
        "path",
        "deployment_id",
        "deployment_hash",
        "local_binding_hash",
        "session_id",
        "session_hash",
        "assembly_path",
        "assembly_id",
        "assembly_sha256",
    }
    assert set(mapping["description"]) == {
        "release",
        "root_orientation_compensation_quat_wxyz",
        "beta_warning",
    }
    assert mapping["description"]["root_orientation_compensation_quat_wxyz"] == {
        "left": [0.0, 0.7071067811865476, -0.7071067811865475, 0.0],
        "right": [0.0, 0.7071067811865476, 0.7071067811865475, 0.0],
    }
    assert all(
        item["resolved_executable"] == str(interpreter.resolve())
        and item["sdk_module_path"] == str(runtime.module_path)
        for item in mapping["sdk_processes"]
    )
