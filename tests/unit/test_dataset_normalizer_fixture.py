from __future__ import annotations

from types import SimpleNamespace

import pytest

from tools.normalize_mini_dataset_run import _fixture_drift


def _state(path: str, *, x: float = 0.0) -> dict[str, object]:
    return {
        "prim_path": path,
        "position_m": [x, 0.0, 0.0],
        "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
    }


def _artifact(
    *,
    declared: list[str],
    initial: list[dict[str, object]],
    final: list[dict[str, object]],
) -> SimpleNamespace:
    return SimpleNamespace(
        manifest={
            "scene": {
                "fixed_rigid_body_paths": declared,
                "fixed_body_states": initial,
            }
        },
        receipt={"final_fixed_body_states": final},
    )


def test_zero_declared_fixed_rigid_bodies_have_zero_drift() -> None:
    assert _fixture_drift(_artifact(declared=[], initial=[], final=[])) == (0.0, 0.0)


def test_declared_fixed_rigid_body_requires_captured_states() -> None:
    with pytest.raises(ValueError, match="captured inventory differs from declaration"):
        _fixture_drift(_artifact(declared=["/World/Table"], initial=[], final=[]))


def test_fixed_rigid_body_inventory_must_not_change_during_run() -> None:
    with pytest.raises(ValueError, match="initial/final inventories differ"):
        _fixture_drift(
            _artifact(
                declared=["/World/Table"],
                initial=[_state("/World/Table")],
                final=[_state("/World/Other")],
            )
        )


def test_declared_fixed_rigid_body_drift_is_still_measured() -> None:
    translation, rotation = _fixture_drift(
        _artifact(
            declared=["/World/Table"],
            initial=[_state("/World/Table")],
            final=[_state("/World/Table", x=0.01)],
        )
    )

    assert translation == pytest.approx(0.01)
    assert rotation == 0.0
