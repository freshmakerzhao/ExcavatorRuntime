#!/usr/bin/env python3
"""用实时 Bucket Tip 观测非执行轨迹；本进程没有动作发送能力。"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import time
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

from mission.contract import ExcavationMission, load_mission
from mission.shadow_observation import advance_shadow_observation
from mission.trajectory_tracker import TrajectoryTracker


DEFAULT_MISSION = AIRY_ROOT / "mission" / "config" / "excavation_cycle.json"
DEFAULT_TRAJECTORY = (
    AIRY_ROOT / "localmap" / "exports" / "live_preview" / "trajectory_command.preview_global.json"
)
DEFAULT_BUCKET_TIP = (
    AIRY_ROOT / "localmap" / "exports" / "live_latest" / "bucket_tip.machine_root.live.json"
)
DEFAULT_MACHINE_PROFILE = AIRY_ROOT.parent / "shared" / "machine_profile.json"
DEFAULT_OUTPUT = AIRY_ROOT / "mission" / "exports" / "shadow_latest" / "trajectory_shadow.json"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="持续重算waypoint进度和策略观测，仅记录shadow结果，永不发送动作。"
    )
    parser.add_argument("--mission", type=Path, default=DEFAULT_MISSION)
    parser.add_argument("--trajectory", type=Path, default=DEFAULT_TRAJECTORY)
    parser.add_argument("--bucket-tip", type=Path, default=DEFAULT_BUCKET_TIP)
    parser.add_argument("--machine-profile", type=Path, default=DEFAULT_MACHINE_PROFILE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--poll-hz", type=float, default=10.0)
    parser.add_argument("--max-bucket-age-s", type=float, default=1.0)
    parser.add_argument("--once", action="store_true", help="处理一个新样本后退出")
    return parser


def _load_object(path: Path, name: str) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取{name}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{name}必须是JSON object")
    return value


def _load_trajectory(
    path: Path,
    mission: ExcavationMission,
) -> tuple[dict, tuple[tuple[float, float, float], ...], str]:
    trajectory = _load_object(path, "trajectory")
    if trajectory.get("schema_version") != "trajectory_command.v1":
        raise ValueError("trajectory schema错误")
    if trajectory.get("frame_id") != mission.frame_id:
        raise ValueError("trajectory frame与Mission不一致")
    if trajectory.get("planning_scope") not in {"preview_global", "workspace_strict"}:
        raise ValueError("live shadow只接受非执行planning scope")
    if trajectory.get("execution_eligible") is not False:
        raise ValueError("live shadow拒绝execution_eligible轨迹")
    raw_waypoints = trajectory.get("waypoints_base")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("trajectory waypoints不能为空")
    waypoints = tuple(_triplet("trajectory waypoint", point) for point in raw_waypoints)
    if trajectory.get("waypoint_count") != len(waypoints):
        raise ValueError("trajectory waypoint_count不一致")
    phase_by_task_mode = {"MoveToDig": "dig", "CarryMaterial": "dump"}
    try:
        phase = phase_by_task_mode[trajectory.get("task_mode")]
    except KeyError as exc:
        raise ValueError("trajectory task_mode不是Mission阶段") from exc
    if math.dist(waypoints[-1], mission.targets[phase].position_m) > 1e-9:
        raise ValueError("trajectory终点与当前Mission Snapshot不一致")
    expected_reference = {
        "id": mission.mission_id,
        "sha256": mission.sha256,
        "phase": phase,
    }
    if trajectory.get("mission") != expected_reference:
        raise ValueError("trajectory Mission provenance与当前Mission Snapshot不一致")
    return trajectory, waypoints, phase


def _triplet(name: str, value: object) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name}必须是长度3数组")
    converted = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in converted):
        raise ValueError(f"{name}必须是有限数值")
    return converted


def _load_bucket_tip(path: Path, frame_id: str, max_age_s: float) -> tuple[float, tuple[float, float, float]]:
    bucket_tip = _load_object(path, "bucket_tip")
    if bucket_tip.get("frame_id") != frame_id or bucket_tip.get("status") != "live_from_tf":
        raise ValueError("bucket_tip必须是同frame的live_from_tf数据")
    raw_stamp = bucket_tip.get("stamp_s")
    if isinstance(raw_stamp, bool) or not isinstance(raw_stamp, int | float):
        raise ValueError("bucket_tip stamp_s必须是有限数值")
    stamp_s = float(raw_stamp)
    if not math.isfinite(stamp_s):
        raise ValueError("bucket_tip stamp_s必须是有限数值")
    age_s = time.time() - stamp_s
    if age_s < 0.0 or age_s > max_age_s:
        raise ValueError(f"bucket_tip已过期或来自未来: age_s={age_s:.3f}")
    return stamp_s, _triplet("bucket_tip.position_m", bucket_tip.get("position_m"))


def _write_atomic(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent, text=True
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        os.replace(temporary_name, path)
    except BaseException:
        Path(temporary_name).unlink(missing_ok=True)
        raise


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        if not math.isfinite(args.poll_hz) or args.poll_hz <= 0.0:
            raise ValueError("poll-hz必须大于0")
        if not math.isfinite(args.max_bucket_age_s) or args.max_bucket_age_s <= 0.0:
            raise ValueError("max-bucket-age-s必须大于0")
        mission = load_mission(args.mission)
        trajectory, waypoints, phase = _load_trajectory(args.trajectory, mission)
        machine_profile = _load_object(args.machine_profile, "machine_profile")
        tracker = TrajectoryTracker(
            waypoints=waypoints,
            tolerance_m=mission.limits.waypoint_tolerance_m,
            dwell_s=mission.limits.waypoint_dwell_s,
            timeout_s=mission.limits.tracking_timeout_s,
        )
        last_stamp_s: float | None = None
        while True:
            stamp_s, position_m = _load_bucket_tip(
                args.bucket_tip, mission.frame_id, args.max_bucket_age_s
            )
            if last_stamp_s is not None and stamp_s <= last_stamp_s:
                if args.once:
                    raise ValueError("bucket_tip时间戳没有推进")
                time.sleep(1.0 / args.poll_hz)
                continue
            tracker, observation = advance_shadow_observation(
                tracker,
                trajectory,
                machine_profile,
                position_m,
                now_s=stamp_s,
            )
            last_stamp_s = stamp_s
            status = {
                "schema_version": "trajectory_shadow.v1",
                "mode": "live_shadow_no_motion",
                "mission_id": mission.mission_id,
                "mission_sha256": mission.sha256,
                "phase": phase,
                "frame_id": mission.frame_id,
                "bucket_tip_stamp_s": stamp_s,
                "bucket_tip_position_m": list(position_m),
                "current_waypoint_index": tracker.current_index,
                "waypoint_count": len(tracker.waypoints),
                "distance_m": observation["tracker"]["distance_m"],
                "advanced": observation["tracker"]["advanced"],
                "completed": observation["tracker"]["completed"],
                "timed_out": observation["tracker"]["timed_out"],
                "observation_values_15_26": observation["values"],
                "action_datagrams": 0,
            }
            _write_atomic(args.output, status)
            print(
                f"shadow tip={list(position_m)} waypoint={tracker.current_index + 1}/{len(waypoints)} "
                f"distance_m={status['distance_m']:.4f} completed={status['completed']} "
                f"timed_out={status['timed_out']} action_datagrams=0",
                flush=True,
            )
            if args.once or status["completed"]:
                return 0
            if status["timed_out"]:
                return 2
            time.sleep(1.0 / args.poll_hz)
    except (OSError, ValueError) as exc:
        print(f"trajectory shadow failed: {exc}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
