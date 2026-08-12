"""Small PhysX contact-report adapter shared by Isaac qualification runners."""

from __future__ import annotations

from collections import defaultdict
import math
from typing import cast


class IsaacContactTracker:
    """Collect per-frame contact pairs without treating aggregate force as contact."""

    def __init__(self, *, separation_epsilon_m: float) -> None:
        self.phase = "not_started"
        self.frame_index = -1
        self.separation_epsilon_m = separation_epsilon_m
        self._pairs: dict[tuple[str, str], dict[str, object]] = {}

    def set_frame(self, phase: str, frame_index: int) -> None:
        self.phase = phase
        self.frame_index = frame_index

    def callback(self, headers: object, data: object) -> None:
        for header in headers:  # type: ignore[union-attr]
            left = self._path(getattr(header, "collider0", 0))
            right = self._path(getattr(header, "collider1", 0))
            if left == "/" or right == "/":
                left = self._path(getattr(header, "actor0", 0))
                right = self._path(getattr(header, "actor1", 0))
            pair = cast(tuple[str, str], tuple(sorted((left, right))))
            record = self._pairs.setdefault(
                pair,
                {
                    "event_count": 0,
                    "minimum_separation_m": math.inf,
                    "event_phases": defaultdict(set),
                    "contact_phases": defaultdict(set),
                    "minimum_separation_by_phase_m": {},
                },
            )
            record["event_count"] = int(record["event_count"]) + 1
            cast(defaultdict[str, set[int]], record["event_phases"])[self.phase].add(
                self.frame_index
            )
            offset = int(getattr(header, "contact_data_offset", 0))
            count = int(getattr(header, "num_contact_data", 0))
            event_minimum = math.inf
            for index in range(offset, offset + count):
                separation = float(getattr(data[index], "separation"))  # type: ignore[index]
                event_minimum = min(event_minimum, separation)
                record["minimum_separation_m"] = min(
                    float(record["minimum_separation_m"]), separation
                )
            if event_minimum <= self.separation_epsilon_m:
                cast(defaultdict[str, set[int]], record["contact_phases"])[self.phase].add(
                    self.frame_index
                )
            if not math.isinf(event_minimum):
                phase_minimum = cast(dict[str, float], record["minimum_separation_by_phase_m"])
                phase_minimum[self.phase] = min(
                    phase_minimum.get(self.phase, math.inf), event_minimum
                )

    @staticmethod
    def _path(encoded: object) -> str:
        from pxr import PhysicsSchemaTools  # type: ignore[import-not-found]

        try:
            return str(PhysicsSchemaTools.intToSdfPath(int(encoded)))
        except (TypeError, ValueError):
            return "/"

    def to_mapping(self) -> dict[str, object]:
        result: dict[str, object] = {}
        for pair, raw in sorted(self._pairs.items()):
            event_phases = cast(defaultdict[str, set[int]], raw["event_phases"])
            contact_phases = cast(defaultdict[str, set[int]], raw["contact_phases"])
            minimum = float(raw["minimum_separation_m"])
            result[" <-> ".join(pair)] = {
                "paths": list(pair),
                "event_count": raw["event_count"],
                "minimum_separation_m": None if math.isinf(minimum) else minimum,
                "phase_event_frames": {
                    phase: len(frames) for phase, frames in sorted(event_phases.items())
                },
                "phase_contact_frames": {
                    phase: len(frames) for phase, frames in sorted(contact_phases.items())
                },
                "phase_minimum_separation_m": dict(
                    sorted(
                        cast(
                            dict[str, float],
                            raw["minimum_separation_by_phase_m"],
                        ).items()
                    )
                ),
            }
        return result


def author_isaac_contact_reports(
    stage: object,
    *,
    prim_path_prefix: str,
    threshold_n: float,
) -> tuple[str, ...]:
    from pxr import PhysxSchema, UsdPhysics  # type: ignore[import-not-found]

    authored: list[str] = []
    prefix = prim_path_prefix.rstrip("/") + "/"
    for prim in stage.Traverse():  # type: ignore[attr-defined]
        path = str(prim.GetPath())
        if not path.startswith(prefix) or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(threshold_n)
        authored.append(path)
    if not authored:
        raise RuntimeError(f"no rigid bodies accepted contact reports below {prim_path_prefix}")
    return tuple(sorted(authored))


__all__ = ["IsaacContactTracker", "author_isaac_contact_reports"]
