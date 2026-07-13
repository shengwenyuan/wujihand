from __future__ import annotations

import numpy as np
import pytest

from wujihand.domain.joints import JointLayout


def layout() -> JointLayout:
    return JointLayout(("a", "b"), (-1.0, -2.0), (1.0, 2.0), (3.0, 4.0))


def test_reorders_by_name() -> None:
    values = np.array([10.0, 20.0])
    indices = layout().indices_for(["b", "a"])
    np.testing.assert_array_equal(values[np.asarray(indices)], [20.0, 10.0])


def test_rejects_partial_or_different_layout() -> None:
    with pytest.raises(ValueError, match="full layout"):
        layout().indices_for(["a"])
    with pytest.raises(ValueError, match="layouts differ"):
        layout().indices_for(["a", "c"])


def test_clamps_and_rejects_non_finite() -> None:
    np.testing.assert_array_equal(layout().clamp([-5.0, 9.0]), [-1.0, 2.0])
    with pytest.raises(ValueError, match="NaN"):
        layout().validate_vector([np.nan, 0.0])
