"""Canonical content digest for immutable Trajectory Snapshots."""

from __future__ import annotations

import hashlib
import json
import math
from typing import Sequence


def trajectory_snapshot_sha256(
    *,
    frame_id: str,
    created_at_s: float,
    mission_id: str,
    mission_sha256: str,
    mission_phase: str,
    task_mode: str,
    planning_scope: str,
    control_stage: str,
    workspace_constraint: str,
    execution_eligible: bool,
    source_bucket_tip_stamp_s: float,
    source_local_map_stamp_s: float,
    inputs_frozen_at_s: float,
    valid_until_s: float,
    input_source: str,
    map_source: str,
    clock_mode: str,
    waypoints: Sequence[Sequence[float]],
    waypoint_tolerance_m: float,
    waypoint_dwell_s: float,
    tracking_timeout_s: float,
) -> str:
    """Hash every behavior-relevant field using a stable JSON representation."""
    payload = {
        "frame_id": _text("frame_id", frame_id),
        "created_at_s": _finite("created_at_s", created_at_s),
        "mission_id": _text("mission_id", mission_id),
        "mission_sha256": _text("mission_sha256", mission_sha256),
        "mission_phase": _text("mission_phase", mission_phase),
        "task_mode": _text("task_mode", task_mode),
        "planning_scope": _text("planning_scope", planning_scope),
        "control_stage": _text("control_stage", control_stage),
        "workspace_constraint": _text("workspace_constraint", workspace_constraint),
        "execution_eligible": bool(execution_eligible),
        "source_bucket_tip_stamp_s": _finite(
            "source_bucket_tip_stamp_s", source_bucket_tip_stamp_s
        ),
        "source_local_map_stamp_s": _finite(
            "source_local_map_stamp_s", source_local_map_stamp_s
        ),
        "inputs_frozen_at_s": _finite("inputs_frozen_at_s", inputs_frozen_at_s),
        "valid_until_s": _finite("valid_until_s", valid_until_s),
        "input_source": _text("input_source", input_source),
        "map_source": _text("map_source", map_source),
        "clock_mode": _text("clock_mode", clock_mode),
        "waypoints": [
            [_finite(f"waypoint[{index}]", value) for index, value in enumerate(point)]
            for point in waypoints
        ],
        "waypoint_tolerance_m": _finite(
            "waypoint_tolerance_m", waypoint_tolerance_m
        ),
        "waypoint_dwell_s": _finite("waypoint_dwell_s", waypoint_dwell_s),
        "tracking_timeout_s": _finite("tracking_timeout_s", tracking_timeout_s),
    }
    encoded = json.dumps(
        payload,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def trajectory_snapshot_message_sha256(message) -> str:
    """Compute the canonical digest from a ROS TrajectorySnapshot message."""
    return trajectory_snapshot_sha256(
        frame_id=message.header.frame_id,
        created_at_s=_time_seconds(message.header.stamp),
        mission_id=message.mission_id,
        mission_sha256=message.mission_sha256,
        mission_phase=message.mission_phase,
        task_mode=message.task_mode,
        planning_scope=message.planning_scope,
        control_stage=message.control_stage,
        workspace_constraint=message.workspace_constraint,
        execution_eligible=message.execution_eligible,
        source_bucket_tip_stamp_s=_time_seconds(message.source_bucket_tip_stamp),
        source_local_map_stamp_s=_time_seconds(message.source_local_map_stamp),
        inputs_frozen_at_s=_time_seconds(message.inputs_frozen_at),
        valid_until_s=_time_seconds(message.valid_until),
        input_source=message.input_source,
        map_source=message.map_source,
        clock_mode=message.clock_mode,
        waypoints=((point.x, point.y, point.z) for point in message.waypoints),
        waypoint_tolerance_m=message.waypoint_tolerance_m,
        waypoint_dwell_s=message.waypoint_dwell_s,
        tracking_timeout_s=message.tracking_timeout_s,
    )


def _finite(name: str, value: float) -> float:
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


def _text(name: str, value: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be non-empty")
    return value


def _time_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9
