from __future__ import annotations

import struct
from dataclasses import replace
from types import SimpleNamespace

import pytest

from teleoperation_quality.camera_integrity import CameraIntegrityAccumulator

IDENTITY = (
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
)


def header(stamp_ns: int, frame_id: str) -> SimpleNamespace:
    return SimpleNamespace(
        stamp=SimpleNamespace(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        ),
        frame_id=frame_id,
    )


def image(stamp_ns: int, frame_id: str, *, depth: bool) -> SimpleNamespace:
    bytes_per_pixel = 4 if depth else 3
    payload = struct.pack("<f", 1.0) * (640 * 480) if depth else bytes(640 * 480 * bytes_per_pixel)
    return SimpleNamespace(
        header=header(stamp_ns, frame_id),
        width=640,
        height=480,
        encoding="32FC1" if depth else "rgb8",
        is_bigendian=False,
        step=640 * bytes_per_pixel,
        data=payload,
    )


def camera_info(stamp_ns: int, frame_id: str) -> SimpleNamespace:
    return SimpleNamespace(
        header=header(stamp_ns, frame_id),
        width=640,
        height=480,
        distortion_model="plumb_bob",
        k=(116.47, 0.0, 320.0, 0.0, 116.47, 240.0, 0.0, 0.0, 1.0),
        d=(0.0,) * 5,
        r=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        p=(116.47, 0.0, 320.0, 0.0, 0.0, 116.47, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    )


def truth(
    side: str,
    index: int,
    numerator: int,
    denominator: int,
    *,
    scheduled_stamp_ns: int | None = None,
) -> SimpleNamespace:
    scaled = numerator * 1_000_000_000
    quotient, remainder = divmod(scaled, denominator)
    reference_stamp_ns = quotient + int(remainder * 2 >= denominator)
    stamp_ns = reference_stamp_ns if scheduled_stamp_ns is None else scheduled_stamp_ns
    return SimpleNamespace(
        schema="wujihand.simulation_camera_frame_truth.v2",
        run_id="fixture-run",
        side=side,
        camera_frame_index=index,
        stamp_ns=stamp_ns,
        world_frame_id="world",
        hand_base_frame_id=f"wujihand_{side}_hand_base",
        optical_frame_id=f"wujihand_{side}_wrist_camera_optical",
        control_tick_id=index * 2 + 1,
        physics_substep_index=index * 4 + 3,
        capture_sim_time_s=numerator / denominator,
        host_capture_start_ns=1_000 + index * 100,
        host_capture_end_ns=1_050 + index * 100,
        world_from_hand_base_row_major=IDENTITY,
        world_from_camera_optical_row_major=IDENTITY,
        hand_base_from_camera_optical_row_major=IDENTITY,
        completed_frame_identity=f"{numerator}/{denominator}",
        reference_time_numerator=numerator,
        reference_time_denominator=denominator,
    )


def transform(parent: str, child: str, stamp_ns: int) -> SimpleNamespace:
    return SimpleNamespace(
        header=header(stamp_ns, parent),
        child_frame_id=child,
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def declared_topics() -> set[str]:
    return {
        f"/wujihand/v1/teleop/{side}/wrist_camera/{suffix}"
        for side in ("left", "right")
        for suffix in (
            "color/image_raw",
            "depth/image_raw",
            "camera_info",
            "frame_truth",
        )
    } | {"/tf", "/tf_static"}


def populated_accumulator() -> CameraIntegrityAccumulator:
    accumulator = CameraIntegrityAccumulator(expected_run_id="fixture-run")
    static_transforms = []
    for side in ("left", "right"):
        hand_frame = f"wujihand_{side}_hand_base"
        optical_frame = f"wujihand_{side}_wrist_camera_optical"
        static_transforms.append(transform(hand_frame, optical_frame, 0))
        for index, (numerator, denominator) in enumerate(((1, 30), (1, 15))):
            message = truth(side, index, numerator, denominator)
            stamp_ns = message.stamp_ns
            base = f"/wujihand/v1/teleop/{side}/wrist_camera"
            accumulator.observe_camera(
                f"{base}/color/image_raw",
                image(stamp_ns, optical_frame, depth=False),
                bag_time_ns=10_000 + index,
            )
            accumulator.observe_camera(
                f"{base}/depth/image_raw",
                image(stamp_ns, optical_frame, depth=True),
                bag_time_ns=11_000 + index,
            )
            accumulator.observe_camera(
                f"{base}/camera_info",
                camera_info(stamp_ns, optical_frame),
                bag_time_ns=12_000 + index,
            )
            accumulator.observe_camera(
                f"{base}/frame_truth",
                message,
                bag_time_ns=13_000 + index,
            )
            accumulator.observe_tf(
                "/tf",
                SimpleNamespace(transforms=[transform("world", hand_frame, stamp_ns)]),
                bag_time_ns=14_000 + index,
            )
    accumulator.observe_tf(
        "/tf_static",
        SimpleNamespace(transforms=static_transforms),
        bag_time_ns=9_000,
    )
    return accumulator


def test_complete_dual_camera_bundles_and_tf_close() -> None:
    frames, transforms = populated_accumulator().finalize(declared_topics=declared_topics())

    assert len(frames) == 4
    assert len(transforms) == 6
    assert [frame.camera_frame_index for frame in frames if frame.side == "left"] == [0, 1]
    assert all(frame.finite_depth_pixels == 640 * 480 for frame in frames)


def test_camera_bundle_rejects_duplicate_payload() -> None:
    accumulator = populated_accumulator()
    optical_frame = "wujihand_left_wrist_camera_optical"

    with pytest.raises(ValueError, match="duplicate left camera color"):
        accumulator.observe_camera(
            "/wujihand/v1/teleop/left/wrist_camera/color/image_raw",
            image(33_333_333, optical_frame, depth=False),
            bag_time_ns=99_000,
        )


def test_camera_tf_rejects_direct_world_to_optical_edge() -> None:
    accumulator = populated_accumulator()
    accumulator.observe_tf(
        "/tf",
        SimpleNamespace(
            transforms=[
                transform(
                    "world",
                    "wujihand_left_wrist_camera_optical",
                    33_333_333,
                )
            ]
        ),
        bag_time_ns=99_000,
    )

    with pytest.raises(ValueError, match="direct world-to-optical"):
        accumulator.finalize(declared_topics=declared_topics())


def test_camera_sequence_rejects_wrong_control_phase() -> None:
    accumulator = populated_accumulator()
    bundle = accumulator._bundles[("left", 66_666_667)]
    bundle["truth"] = replace(bundle["truth"], control_tick_id=4)

    with pytest.raises(ValueError, match="second control tick"):
        accumulator.finalize(declared_topics=declared_topics())


def test_camera_truth_allows_bounded_rtx_reference_drift() -> None:
    message = truth(
        "left",
        33,
        2_033_333_332,
        1_000_000_000,
        scheduled_stamp_ns=2_033_333_333,
    )
    accumulator = CameraIntegrityAccumulator(expected_run_id="fixture-run")

    accumulator.observe_camera(
        "/wujihand/v1/teleop/left/wrist_camera/frame_truth",
        message,
        bag_time_ns=1,
    )

    invalid = truth(
        "left",
        33,
        2_033_332_000,
        1_000_000_000,
        scheduled_stamp_ns=2_033_333_333,
    )
    with pytest.raises(ValueError, match="exceeds rational frame identity tolerance"):
        accumulator.observe_camera(
            "/wujihand/v1/teleop/left/wrist_camera/frame_truth",
            invalid,
            bag_time_ns=2,
        )
