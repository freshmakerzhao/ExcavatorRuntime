#!/usr/bin/env python3
"""从RRT*请求生成mock TrajectoryCommand。

注意：这个脚本不是RRT*，只用于在真实规划器接入前打通下游契约。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
PROJECT_ROOT = LOCALMAP_DIR.parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.io import load_json, write_json
from localmap_core.trajectory import build_trajectory_command, interpolate_waypoints


DEFAULT_REQUEST = LOCALMAP_DIR / "exports" / "rrt_star_request.mock.json"
DEFAULT_PROFILE = PROJECT_ROOT / "shared" / "machine_profile.json"
DEFAULT_OUTPUT = LOCALMAP_DIR / "exports" / "trajectory_command.mock.json"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造mock轨迹命令参数；waypoint_count只是规划输出数量，不改变策略lookahead。"""
    parser = argparse.ArgumentParser(description="从RRT*请求生成mock TrajectoryCommand。")
    parser.add_argument("--request", type=Path, default=DEFAULT_REQUEST, help="RRT*请求JSON")
    parser.add_argument("--machine-profile", type=Path, default=DEFAULT_PROFILE, help="机型profile路径")
    parser.add_argument("--waypoint-count", type=int, default=5, help="mock输出waypoint数量，默认5")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="输出TrajectoryCommand JSON")
    return parser


def main() -> int:
    """入口函数：RRT*请求 -> mock waypoint list -> TrajectoryCommand。"""
    args = build_arg_parser().parse_args()
    request = load_json(args.request)
    machine_profile = load_json(args.machine_profile)
    pitch_targets = machine_profile["task_profile"]["bucket_pitch_targets_deg"]
    task_mode = request["task_mode"]

    start = np.array(request["start_bucket_tip_base"], dtype=np.float64)
    goal = np.array(request["goal"]["position_m"], dtype=np.float64)
    waypoints = interpolate_waypoints(start, goal, args.waypoint_count)

    # 关键：target bucket pitch从machine_profile读取，保持Unity/部署侧一致。
    command = build_trajectory_command(
        timestamp_s=request["timestamp_s"],
        frame_id=request["frame_id"],
        task_mode=task_mode,
        target_bucket_pitch_deg=float(pitch_targets[task_mode]),
        waypoints_base=waypoints,
        target_threshold=float(request["planning_params"]["target_threshold"]),
        tube_radius=float(request["planning_params"]["tube_radius"]),
    )
    write_json(args.output, command)

    print(f"request: {args.request}")
    print(f"waypoint_count: {args.waypoint_count}")
    print(f"target_bucket_pitch_deg: {pitch_targets[task_mode]}")
    print(f"output: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
