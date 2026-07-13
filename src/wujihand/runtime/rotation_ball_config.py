"""Strict loader for the fixed-flange rotation-and-ball scene profile."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, cast

import numpy as np
import yaml


def _vector(
    value: object, *, size: int, field: str, positive: bool = False
) -> tuple[float, ...]:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f"{field} must be a finite length-{size} vector")
    if positive and np.any(array <= 0.0):
        raise ValueError(f"{field} must be positive")
    return tuple(float(item) for item in array)


def _positive(value: Any, field: str, *, allow_zero: bool = False) -> float:
    number = float(value)
    if not np.isfinite(number) or (number < 0.0 if allow_zero else number <= 0.0):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{field} must be finite and {qualifier}")
    return number


@dataclass(frozen=True, slots=True)
class TableConfig:
    position_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    color_rgb: tuple[float, float, float]

    @property
    def top_z_m(self) -> float:
        return self.position_m[2] + self.size_m[2] / 2.0


@dataclass(frozen=True, slots=True)
class FlangeConfig:
    position_m: tuple[float, float, float]
    neutral_quat_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class WristDriveConfig:
    stiffness: float
    damping: float
    max_force: float


@dataclass(frozen=True, slots=True)
class WristConfig:
    pitch_limit_rad: float
    roll_limit_rad: float
    max_angular_velocity_rad_s: float
    stale_after_s: float
    disarm_after_s: float
    min_quality: float
    drive: WristDriveConfig


@dataclass(frozen=True, slots=True)
class BallConfig:
    center_m: tuple[float, float, float]
    radius_m: float
    mass_kg: float
    color_rgb: tuple[float, float, float]
    static_friction: float
    dynamic_friction: float
    restitution: float


@dataclass(frozen=True, slots=True)
class QualificationConfig:
    lift_height_m: float
    max_hand_relative_slip_m: float
    hold_time_s: float
    required_opposing_finger_groups: int
    trial_count: int
    required_successes: int


@dataclass(frozen=True, slots=True)
class ScriptConfig:
    pregrasp_delta_pitch_rad: float
    lifted_delta_pitch_rad: float
    close_q20: tuple[float, ...]
    hold_q20: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class RotationBallConfig:
    name: str
    provenance: Mapping[str, str]
    table: TableConfig
    flange: FlangeConfig
    wrist: WristConfig
    ball: BallConfig
    qualification: QualificationConfig
    script: ScriptConfig


def load_rotation_ball_config(path: str | Path) -> RotationBallConfig:
    """Load and validate the version-one Hand 2 rotation-ball profile."""

    data: dict[str, Any] = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError("unsupported rotation-ball profile schema")
    if data.get("product") != "wuji_hand_2_beta_1" or data.get("side") != "right":
        raise ValueError("rotation-ball profile is not Wuji Hand 2 Beta 1 right")

    table_data = data["table"]
    table = TableConfig(
        position_m=cast(
            tuple[float, float, float],
            _vector(table_data["position_m"], size=3, field="table.position_m"),
        ),
        size_m=cast(
            tuple[float, float, float],
            _vector(table_data["size_m"], size=3, field="table.size_m", positive=True),
        ),
        color_rgb=cast(
            tuple[float, float, float],
            _vector(table_data["color_rgb"], size=3, field="table.color_rgb"),
        ),
    )
    if any(component < 0.0 or component > 1.0 for component in table.color_rgb):
        raise ValueError("table.color_rgb must be in [0, 1]")

    flange_data = data["flange"]
    neutral = np.asarray(flange_data["neutral_quat_wxyz"], dtype=np.float64)
    if neutral.shape != (4,) or not np.isfinite(neutral).all():
        raise ValueError("flange.neutral_quat_wxyz must be a finite quaternion")
    norm = float(np.linalg.norm(neutral))
    if not np.isclose(norm, 1.0, atol=1e-6):
        raise ValueError("flange.neutral_quat_wxyz must be unit length")
    flange = FlangeConfig(
        position_m=cast(
            tuple[float, float, float],
            _vector(flange_data["position_m"], size=3, field="flange.position_m"),
        ),
        neutral_quat_wxyz=(
            float(neutral[0]),
            float(neutral[1]),
            float(neutral[2]),
            float(neutral[3]),
        ),
    )
    if flange.position_m[2] <= table.top_z_m:
        raise ValueError("flange must be above the table top")

    wrist_data = data["wrist"]
    drive_data = wrist_data["drive"]
    wrist = WristConfig(
        pitch_limit_rad=_positive(wrist_data["pitch_limit_rad"], "wrist.pitch_limit_rad"),
        roll_limit_rad=_positive(wrist_data["roll_limit_rad"], "wrist.roll_limit_rad"),
        max_angular_velocity_rad_s=_positive(
            wrist_data["max_angular_velocity_rad_s"], "wrist.max_angular_velocity_rad_s"
        ),
        stale_after_s=_positive(wrist_data["stale_after_s"], "wrist.stale_after_s"),
        disarm_after_s=_positive(wrist_data["disarm_after_s"], "wrist.disarm_after_s"),
        min_quality=float(wrist_data["min_quality"]),
        drive=WristDriveConfig(
            stiffness=_positive(drive_data["stiffness"], "wrist.drive.stiffness"),
            damping=_positive(drive_data["damping"], "wrist.drive.damping"),
            max_force=_positive(drive_data["max_force"], "wrist.drive.max_force"),
        ),
    )
    if wrist.disarm_after_s <= wrist.stale_after_s:
        raise ValueError("wrist.disarm_after_s must exceed stale_after_s")
    if not np.isfinite(wrist.min_quality) or not 0.0 <= wrist.min_quality <= 1.0:
        raise ValueError("wrist.min_quality must be finite and in [0, 1]")
    if wrist.pitch_limit_rad >= np.pi / 2.0 or wrist.roll_limit_rad >= np.pi / 2.0:
        raise ValueError("pitch/roll limits must stay below the 90-degree singularity")

    ball_data = data["ball"]
    ball = BallConfig(
        center_m=cast(
            tuple[float, float, float],
            _vector(ball_data["center_m"], size=3, field="ball.center_m"),
        ),
        radius_m=_positive(ball_data["radius_m"], "ball.radius_m"),
        mass_kg=_positive(ball_data["mass_kg"], "ball.mass_kg"),
        color_rgb=cast(
            tuple[float, float, float],
            _vector(ball_data["color_rgb"], size=3, field="ball.color_rgb"),
        ),
        static_friction=_positive(
            ball_data["static_friction"], "ball.static_friction", allow_zero=True
        ),
        dynamic_friction=_positive(
            ball_data["dynamic_friction"], "ball.dynamic_friction", allow_zero=True
        ),
        restitution=_positive(ball_data["restitution"], "ball.restitution", allow_zero=True),
    )
    if any(component < 0.0 or component > 1.0 for component in ball.color_rgb):
        raise ValueError("ball.color_rgb must be in [0, 1]")
    if not 0.0 <= ball.restitution <= 1.0:
        raise ValueError("ball.restitution must be in [0, 1]")
    if abs(ball.center_m[2] - (table.top_z_m + ball.radius_m)) > 1e-6:
        raise ValueError("ball must initially rest on the table top")

    qual_data = data["qualification"]
    qualification = QualificationConfig(
        lift_height_m=_positive(qual_data["lift_height_m"], "qualification.lift_height_m"),
        max_hand_relative_slip_m=_positive(
            qual_data["max_hand_relative_slip_m"],
            "qualification.max_hand_relative_slip_m",
        ),
        hold_time_s=_positive(qual_data["hold_time_s"], "qualification.hold_time_s"),
        required_opposing_finger_groups=int(qual_data["required_opposing_finger_groups"]),
        trial_count=int(qual_data["trial_count"]),
        required_successes=int(qual_data["required_successes"]),
    )
    if qualification.required_opposing_finger_groups < 1:
        raise ValueError("required_opposing_finger_groups must be positive")
    if not 1 <= qualification.required_successes <= qualification.trial_count:
        raise ValueError("required_successes must be within trial_count")

    script_data = data["script"]
    close_q20 = _vector(script_data["close_q20"], size=20, field="script.close_q20")
    hold_q20 = _vector(script_data["hold_q20"], size=20, field="script.hold_q20")
    script = ScriptConfig(
        pregrasp_delta_pitch_rad=float(script_data["pregrasp_delta_pitch_rad"]),
        lifted_delta_pitch_rad=float(script_data["lifted_delta_pitch_rad"]),
        close_q20=close_q20,
        hold_q20=hold_q20,
    )
    if not (
        0.0
        <= script.lifted_delta_pitch_rad
        < script.pregrasp_delta_pitch_rad
        <= wrist.pitch_limit_rad
    ):
        raise ValueError("script delta-pitch waypoints must be ordered inside the wrist limit")

    provenance = {key: str(value) for key, value in data["derived_from"].items()}
    return RotationBallConfig(
        name=str(data["name"]),
        provenance=provenance,
        table=table,
        flange=flange,
        wrist=wrist,
        ball=ball,
        qualification=qualification,
        script=script,
    )
