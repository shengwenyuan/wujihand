from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "ros2/wujihand_ros2"))

from wujihand.domain import (  # noqa: E402
    TrackedRigidBodySample,
    TrackingState,
)
from wujihand_interfaces.msg import (  # type: ignore[import-not-found]  # noqa: E402
    RunRecordingStatus,
    SceneRigidBodyState,
    TeleoperationTickTrace,
    TeleoperationTickTraceV2,
    TrackedRigidBodySample as TrackedRigidBodySampleMessage,
)
from wujihand_ros2.conversion import (  # noqa: E402
    tracked_sample_from_message,
    tracked_sample_to_message,
)


def test_generated_tracker_message_round_trip() -> None:
    sample = TrackedRigidBodySample(
        stream_id="tracker_left",
        device_serial="fixture-left",
        logical_role="operator_left",
        producer_instance="fixture-producer",
        transport_epoch=1,
        tracking_setup_revision="fixture-setup-v1",
        sequence=2,
        tracking_frame="vive_tracking",
        position_m=(0.1, 0.2, 0.3),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        connected=True,
        pose_valid=True,
        tracking_state=TrackingState.RUNNING,
        quality=1.0,
        host_time_ns=1000,
        device_time_ns=None,
    )

    message = tracked_sample_to_message(
        sample,
        factory=TrackedRigidBodySampleMessage,
    )

    assert tracked_sample_from_message(message) == sample


def test_generated_recording_interfaces_have_fixed_control_shapes() -> None:
    legacy_tick = TeleoperationTickTrace()
    tick = TeleoperationTickTraceV2()
    scene = SceneRigidBodyState()
    status = RunRecordingStatus()

    assert len(tick.arm_command_q7_rad) == 7
    assert len(tick.hand_intent_q20_rad) == 20
    assert len(tick.applied_target_q27_rad) == 27
    assert len(tick.physics_substep_indices) == 4
    assert len(tick.physics_substep_sim_times_s) == 4
    assert len(tick.physics_substep_start_ns) == 4
    assert len(tick.physics_substep_end_ns) == 4
    assert legacy_tick.spin_start_ns == 0
    assert len(scene.position_m) == 3
    assert status.state == ""
