#!/usr/bin/env python3
"""从TrajectoryCommand生成38维observation中的waypoint切片。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
PROJECT_ROOT = LOCALMAP_DIR.parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.io import load_json, write_json
from localmap_core.trajectory import build_waypoint_observation_slice


DEFAULT_TRAJECTORY = LOCALMAP_DIR / "exports" / "trajectory_command.mock.json"
DEFAULT_BUCKET_TIP = LOCALMAP_DIR / "config" / "bucket_tip.machine_root.measured.json"
DEFAULT_PROFILE = PROJECT_ROOT / "shared" / "machine_profile.json"
DEFAULT_OUTPUT = LOCALMAP_DIR / "exports" / "observation_waypoint_slice.mock.json"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造命令行参数；这里只生成idx 15..26，不生成完整38维observation。"""
    parser = argparse.ArgumentParser(description="生成observation中waypoint相关的idx 15..26切片。")
    parser.add_argument("--trajectory", type=Path, default=DEFAULT_TRAJECTORY, help="TrajectoryCommand JSON")
    parser.add_argument("--bucket-tip", type=Path, default=DEFAULT_BUCKET_TIP, help="bucket tip mock/状态JSON")
    parser.add_argument("--machine-profile", type=Path, default=DEFAULT_PROFILE, help="机型profile路径")
    parser.add_argument("--current-index", type=int, default=0, help="当前waypoint index")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="输出observation切片JSON")
    return parser


def main() -> int:
    """入口函数：trajectory + bucket tip + profile -> observation idx 15..26。"""
    args = build_arg_parser().parse_args()
    trajectory = load_json(args.trajectory)
    bucket_tip = load_json(args.bucket_tip)
    machine_profile = load_json(args.machine_profile)

    if bucket_tip["frame_id"] != trajectory["frame_id"]:
        raise SystemExit(f"bucket_tip frame {bucket_tip['frame_id']} 与 trajectory frame {trajectory['frame_id']} 不一致")

    # 关键：完整38维observation由部署侧状态估计器组装；这里仅验证waypoint如何进入idx 15..26。
    bucket_tip_base = np.array(bucket_tip["position_m"], dtype=np.float64)
    obs_slice = build_waypoint_observation_slice(
        trajectory_command=trajectory,
        machine_profile=machine_profile,
        bucket_tip_base=bucket_tip_base,
        current_waypoint_index=args.current_index,
    )
    write_json(args.output, obs_slice)

    print(f"trajectory: {args.trajectory}")
    print(f"indices: {obs_slice['indices']}")
    print(f"values: {obs_slice['values']}")
    print(f"output: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
