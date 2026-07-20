"""Deterministic empty-map Planning Adapter for offline ROS Action validation."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .simple_bucket_tip_planner import PlanningBounds, plan_bucket_tip_path


_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_TASK_MODE = {"dig": "MoveToDig", "dump": "CarryMaterial"}
_DEFAULT_BOUNDS = (-0.5, 4.0, -3.0, 1.5, -0.7, 1.2)


@dataclass(frozen=True)
class FixturePlanningRequest:
    frame_id: str
    input_source: str
    map_source: str
    start_m: tuple[float, float, float]
    start_stamp_s: float
    target_id: str
    target_kind: str
    target_status: str
    target_m: tuple[float, float, float]
    mission_id: str
    mission_sha256: str
    mission_phase: str
    planning_scope: str
    created_at_s: float

    def __post_init__(self) -> None:
        if self.frame_id != "machine_root_ros":
            raise ValueError("frame_id must be machine_root_ros")
        if self.input_source != "fixture":
            raise ValueError("fixture planner requires input_source=fixture")
        if self.map_source != "fixture_empty":
            raise ValueError("fixture planner requires map_source=fixture_empty")
        if self.planning_scope != "preview_global":
            raise ValueError("fixture planner only supports planning_scope=preview_global")
        if self.target_kind not in _TASK_MODE or self.mission_phase != self.target_kind:
            raise ValueError("target_kind and mission_phase must match dig or dump")
        if self.target_status not in {"placeholder", "rviz_adjusted", "field_validated"}:
            raise ValueError("target_status is invalid")
        for name in ("target_id", "mission_id"):
            if not isinstance(getattr(self, name), str) or not getattr(self, name).strip():
                raise ValueError(f"{name} must be non-empty")
        if not _SHA256.fullmatch(self.mission_sha256):
            raise ValueError("mission_sha256 must be lowercase sha256")
        _triplet("start_m", self.start_m)
        _triplet("target_m", self.target_m)
        start_stamp = _finite("start_stamp_s", self.start_stamp_s)
        created_at = _finite("created_at_s", self.created_at_s)
        if start_stamp <= 0.0 or created_at < start_stamp:
            raise ValueError("planning timestamps are inconsistent")


@dataclass(frozen=True)
class FixturePlanningResult:
    success: bool
    reason: str
    iterations: int
    task_mode: str
    waypoints: tuple[tuple[float, float, float], ...]


def plan_fixture_trajectory(
    request: FixturePlanningRequest,
    *,
    waypoint_count: int,
    bounds: Sequence[float] = _DEFAULT_BOUNDS,
) -> FixturePlanningResult:
    if isinstance(waypoint_count, bool) or waypoint_count < 1:
        raise ValueError("waypoint_count must be a positive integer")
    planned = plan_bucket_tip_path(
        start=np.asarray(request.start_m, dtype=np.float64),
        goal=np.asarray(request.target_m, dtype=np.float64),
        obstacles=[],
        bounds=PlanningBounds.from_values(tuple(float(value) for value in bounds)),
        collision_radius_m=0.05,
        step_size_m=0.2,
        edge_check_step_m=0.04,
        max_iterations=6000,
        goal_sample_rate=0.15,
        waypoint_count=waypoint_count,
        seed=8,
        reachable_workspace=None,
    )
    waypoints = tuple(tuple(float(value) for value in row) for row in planned.waypoints)
    return FixturePlanningResult(
        success=planned.success,
        reason=planned.reason,
        iterations=planned.iterations,
        task_mode=_TASK_MODE[request.target_kind],
        waypoints=waypoints,
    )


def _finite(name: str, value: float) -> float:
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


def _triplet(name: str, values: Sequence[float]) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"{name} must contain three values")
    return tuple(_finite(f"{name}[{index}]", value) for index, value in enumerate(values))
