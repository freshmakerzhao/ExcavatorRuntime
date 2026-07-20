"""按实时 Bucket Tip 重算 waypoint 观测切片，仅供 shadow/replay。"""

from __future__ import annotations

from typing import Any, Mapping, Sequence

import numpy as np

from localmap.localmap_core.trajectory import build_waypoint_observation_slice
from mission.trajectory_tracker import TrajectoryTracker


def advance_shadow_observation(
    tracker: TrajectoryTracker,
    trajectory: Mapping[str, Any],
    machine_profile: Mapping[str, Any],
    bucket_tip_m: Sequence[float],
    *,
    now_s: float,
) -> tuple[TrajectoryTracker, dict[str, Any]]:
    """推进不可变 tracker，并用同一帧的实时铲尖重算 15..26 维。"""
    if trajectory.get("frame_id") != "machine_root_ros":
        raise ValueError("shadow trajectory frame必须是machine_root_ros")
    trajectory_waypoints = tuple(
        tuple(float(component) for component in point)
        for point in trajectory.get("waypoints_base", ())
    )
    if trajectory_waypoints != tracker.waypoints:
        raise ValueError("tracker waypoints与trajectory snapshot不一致")
    updated_tracker, update = tracker.advance(bucket_tip_m, now_s=now_s)
    observation = build_waypoint_observation_slice(
        dict(trajectory),
        dict(machine_profile),
        np.asarray(bucket_tip_m, dtype=np.float64),
        updated_tracker.current_index,
    )
    return updated_tracker, {
        **observation,
        "mode": "shadow_no_motion",
        "tracker": {
            "distance_m": update.distance_m,
            "advanced": update.advanced,
            "completed": update.completed,
            "timed_out": update.timed_out,
        },
    }
