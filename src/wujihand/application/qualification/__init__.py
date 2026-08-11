"""Hardware-independent qualification metrics."""

from .hand2_scripted import (
    Hand2QualificationTarget,
    Hand2SingleDigitPartition,
    build_hand2_qualification_targets,
    partition_hand2_single_digit_indices,
    qualification_gate_exit_code,
)
from .tracking_metrics import TrackingMetrics, compute_tracking_metrics
from .wuji_glove_stub import (
    WUJI_GLOVE_STUB_POSES,
    build_wuji_glove_stub_observations,
    wuji_glove_stub_keypoints,
)
from .live_readiness import (
    FULL_SCRIPTED_Q27_SETTLING_POLICY,
    GLOVE_LIVE_Q27_READINESS_POLICY,
    ROS_TELEOP_Q27_SETTLING_POLICY,
    Q27ReadinessPolicy,
    joint_target_max_errors_rad,
    q27_window_max_delta_rad,
)

__all__ = [
    "FULL_SCRIPTED_Q27_SETTLING_POLICY",
    "GLOVE_LIVE_Q27_READINESS_POLICY",
    "ROS_TELEOP_Q27_SETTLING_POLICY",
    "Hand2QualificationTarget",
    "Hand2SingleDigitPartition",
    "Q27ReadinessPolicy",
    "TrackingMetrics",
    "WUJI_GLOVE_STUB_POSES",
    "build_hand2_qualification_targets",
    "build_wuji_glove_stub_observations",
    "compute_tracking_metrics",
    "joint_target_max_errors_rad",
    "partition_hand2_single_digit_indices",
    "q27_window_max_delta_rad",
    "qualification_gate_exit_code",
    "wuji_glove_stub_keypoints",
]
