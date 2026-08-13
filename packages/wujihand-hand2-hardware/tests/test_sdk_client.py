from wujihand_hand2_hardware.sdk_client import _enum_text


class _SdkEnumWithoutName:
    def __str__(self) -> str:
        return "DeviceType.WujiHand2"


def test_sdk_enum_text_accepts_pyo3_qualified_display() -> None:
    assert _enum_text(_SdkEnumWithoutName()) == "WujiHand2"
