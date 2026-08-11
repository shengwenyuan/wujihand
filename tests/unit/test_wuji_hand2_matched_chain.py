from __future__ import annotations

from collections.abc import Mapping, Sequence
from pathlib import Path

import pytest

from wujihand.domain import HAND2_LAYOUT_IDS, HandSide
from wujihand.integrity import sha256_file
from wujihand.runtime.wuji_hand2_matched_chain import (
    WujiSdkRuntimeFacts,
    load_matched_chain_qualification_policy,
    preflight_wuji_hand2_matched_chain,
)


ROOT = Path(__file__).resolve().parents[2]
POLICY = ROOT / "configs/qualifications/wuji_hand2_matched_chain_v2026_8_3_v1.yaml"


class _Manager:
    def __init__(self) -> None:
        self.selected = ""
        self.switches: list[str] = []

    def list_users(self) -> Sequence[Mapping[str, object]]:
        return (
            {"user_id": "", "display_name": "Default", "is_default": True},
            {"user_id": "u_fixture", "display_name": "1011", "is_default": False},
        )

    def switch_user(self, user_id: str) -> None:
        self.selected = user_id
        self.switches.append(user_id)

    def current_user(self) -> Mapping[str, object]:
        return {
            "user_id": self.selected,
            "display_name": "1011",
            "is_default": False,
        }


def _write_inputs(tmp_path: Path, *, side: HandSide) -> tuple[Path, Path, WujiSdkRuntimeFacts]:
    wheel = tmp_path / "wuji_sdk-2026.8.3-fixture.whl"
    wheel.write_bytes(b"fixture wheel")
    overlay = tmp_path / "overlay"
    module = overlay / "wuji_sdk" / "__init__.py"
    module.parent.mkdir(parents=True)
    module.write_text("__version__ = '2026.8.3'\n", encoding="utf-8")
    interpreter = tmp_path / "isaac-python"
    interpreter.write_text("fixture\n", encoding="utf-8")

    home = tmp_path / "home"
    models = home / ".wuji/sdk/users/u_fixture/models"
    models.mkdir(parents=True)
    urdf = models / f"{side.value}_hand.urdf"
    urdf.write_text(
        f"""<robot name=\"{side.value}_hand\">
  <link name=\"wrist\"/>
  <link name=\"tip\"/>
  <joint name=\"wrist_tip\" type=\"fixed\">
    <parent link=\"wrist\"/><child link=\"tip\"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )

    session = (
        f"configs/sessions/isaac_hand2_{side.value}_glove_qualification_"
        "v2026_8_3_v1.yaml"
    )
    policy = tmp_path / "qualification.yaml"
    policy.write_text(
        f"""schema: wujihand.wuji_hand2_matched_chain_qualification.v1
qualification_id: fixture_wuji_hand2_matched_chain_v1
sdk:
  distribution: wuji-sdk
  package_version: 2026.8.3
  wheel_sha256: {sha256_file(wheel)}
description:
  release: v2026.8.3
  hand2_model_revision: v2026.7.23
  source_name: wuji-description-v2026-8-3
  commit: 8271644a78d69ed9a4adcf9165d882c64ad33dfa
  asset_revision: beta1_description_v2026_8_3
sides:
  left:
    session_path: configs/sessions/isaac_hand2_left_glove_qualification_v2026_8_3_v1.yaml
    session_id: isaac_hand2_left_glove_qualification_v2026_8_3_v1
    binding_root: l_wrist
    layout_id: wuji_hand2_left_firmware_v1
    urdf_filename: left_hand.urdf
    urdf_link_count: {2 if side is HandSide.LEFT else 48}
    urdf_joint_count: {1 if side is HandSide.LEFT else 47}
  right:
    session_path: configs/sessions/isaac_hand2_right_glove_qualification_v2026_8_3_v1.yaml
    session_id: isaac_hand2_right_glove_qualification_v2026_8_3_v1
    binding_root: r_wrist
    layout_id: wuji_hand2_right_firmware_v1
    urdf_filename: right_hand.urdf
    urdf_link_count: {2 if side is HandSide.RIGHT else 48}
    urdf_joint_count: {1 if side is HandSide.RIGHT else 47}
""",
        encoding="utf-8",
    )
    local = tmp_path / "local.yaml"
    other = HandSide.LEFT if side is HandSide.RIGHT else HandSide.RIGHT
    local.write_text(
        f"""schema: wujihand.wuji_hand2_matched_chain_local_binding.v1
binding_id: fixture_workstation2_v1
interpreter: {interpreter}
sdk_module_root: {overlay}
sdk_wheel: {wheel}
user:
  user_id: u_fixture
  display_name: \"1011\"
  models_dir: {models}
hands:
  left:
    serial_number: WG_LEFT_FIXTURE
    urdf_path: {urdf if side is HandSide.LEFT else models / 'left_hand.urdf'}
    urdf_sha256: "{sha256_file(urdf) if side is HandSide.LEFT else '0' * 64}"
  right:
    serial_number: WG_RIGHT_FIXTURE
    urdf_path: {urdf if side is HandSide.RIGHT else models / 'right_hand.urdf'}
    urdf_sha256: "{sha256_file(urdf) if side is HandSide.RIGHT else '0' * 64}"
""",
        encoding="utf-8",
    )
    assert session
    assert other is not side
    return (
        policy,
        local,
        WujiSdkRuntimeFacts(
            distribution_version="2026.8.3",
            module_version="2026.8.3",
            module_path=module,
            executable_path=interpreter,
        ),
    )


def test_committed_policy_selects_only_versioned_glove_sessions() -> None:
    policy = load_matched_chain_qualification_policy(POLICY)

    assert policy.sdk.package_version == "2026.8.3"
    assert policy.description.release == "v2026.8.3"
    assert policy.description.hand2_model_revision == "v2026.7.23"
    for side in HandSide:
        selected = policy.side(side)
        assert selected.layout_id == HAND2_LAYOUT_IDS[side.value]
        assert f"_{side.value}_glove_qualification_v2026_8_3_" in selected.session_path


@pytest.mark.parametrize("side", tuple(HandSide))
def test_preflight_closes_sdk_user_urdf_and_description_without_device(
    tmp_path: Path,
    side: HandSide,
) -> None:
    policy, local, runtime = _write_inputs(tmp_path, side=side)
    manager = _Manager()

    receipt = preflight_wuji_hand2_matched_chain(
        ROOT,
        qualification_path=policy,
        local_binding_path=local,
        side=side,
        input_mode="stub",
        sdk_runtime=runtime,
        user_manager=manager,
        home_dir=tmp_path / "home",
        verify_artifacts=True,
    )

    assert manager.switches == ["u_fixture"]
    assert receipt.sdk_version == "2026.8.3"
    assert receipt.description_release == "v2026.8.3"
    assert receipt.binding_root == f"{side.value[0]}_wrist"
    assert receipt.layout_id == HAND2_LAYOUT_IDS[side.value]
    assert receipt.calibration_id.startswith(f"wuji_sdk.user.u_fixture.{side.value}.")
    assert receipt.to_mapping()["device_access_attempted"] is False


def test_live_preflight_rejects_studio_ownership(tmp_path: Path) -> None:
    policy, local, runtime = _write_inputs(tmp_path, side=HandSide.LEFT)

    with pytest.raises(RuntimeError, match="Studio must be closed"):
        preflight_wuji_hand2_matched_chain(
            ROOT,
            qualification_path=policy,
            local_binding_path=local,
            side=HandSide.LEFT,
            input_mode="glove",
            sdk_runtime=runtime,
            user_manager=_Manager(),
            studio_processes=("pid=42:wuji-studio",),
            home_dir=tmp_path / "home",
        )


def test_preflight_rejects_mixed_sdk_version_before_session(tmp_path: Path) -> None:
    policy, local, runtime = _write_inputs(tmp_path, side=HandSide.RIGHT)
    wrong = WujiSdkRuntimeFacts(
        distribution_version="2026.7.21",
        module_version=runtime.module_version,
        module_path=runtime.module_path,
        executable_path=runtime.executable_path,
    )

    with pytest.raises(RuntimeError, match="version mismatch"):
        preflight_wuji_hand2_matched_chain(
            ROOT,
            qualification_path=policy,
            local_binding_path=local,
            side=HandSide.RIGHT,
            input_mode="stub",
            sdk_runtime=wrong,
            user_manager=_Manager(),
            home_dir=tmp_path / "home",
        )
