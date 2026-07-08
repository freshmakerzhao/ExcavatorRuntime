#!/usr/bin/env python3
"""从LocalMap生成RRT*请求JSON。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
PROJECT_ROOT = LOCALMAP_DIR.parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.io import load_json, write_json
from localmap_core.trajectory import build_rrt_star_request


DEFAULT_LOCAL_MAP = LOCALMAP_DIR / "exports" / "local_map_from_npz.mock.json"
DEFAULT_BUCKET_TIP = LOCALMAP_DIR / "config" / "bucket_tip.machine_root.measured.json"
DEFAULT_PROFILE = PROJECT_ROOT / "shared" / "machine_profile.json"
DEFAULT_OUTPUT = LOCALMAP_DIR / "exports" / "rrt_star_request.mock.json"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造命令行参数；target必须显式选择，避免规划到错误目标。"""
    parser = argparse.ArgumentParser(description="从LocalMap生成RRT*请求。")
    parser.add_argument("--local-map", type=Path, default=DEFAULT_LOCAL_MAP, help="LocalMap JSON路径")
    parser.add_argument("--machine-profile", type=Path, default=DEFAULT_PROFILE, help="机型profile路径")
    parser.add_argument("--bucket-tip", type=Path, default=DEFAULT_BUCKET_TIP, help="bucket tip mock/状态JSON")
    parser.add_argument("--target-id", default="mock_dig_001", help="目标id")
    parser.add_argument("--target-kind", choices=["dig", "dump"], default="dig", help="目标类型")
    parser.add_argument("--task-mode", choices=["MoveToDig", "CarryMaterial"], default="MoveToDig", help="任务模式")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="输出RRT*请求JSON")
    return parser


def main() -> int:
    """入口函数：LocalMap + bucket tip + profile -> RRT*请求。"""
    args = build_arg_parser().parse_args()
    local_map = load_json(args.local_map)
    machine_profile = load_json(args.machine_profile)
    bucket_tip = load_json(args.bucket_tip)

    if bucket_tip["frame_id"] != local_map["frame_id"]:
        raise SystemExit(f"bucket_tip frame {bucket_tip['frame_id']} 与 LocalMap frame {local_map['frame_id']} 不一致")

    # 关键：bucket_tip_base来自状态估计/FK；当前mock只用于离线契约联调。
    bucket_tip_base = np.array(bucket_tip["position_m"], dtype=np.float64)
    request = build_rrt_star_request(
        local_map=local_map,
        machine_profile=machine_profile,
        bucket_tip_base=bucket_tip_base,
        target_id=args.target_id,
        target_kind=args.target_kind,
        task_mode=args.task_mode,
    )
    write_json(args.output, request)

    print(f"local_map: {args.local_map}")
    print(f"machine_profile: {args.machine_profile}")
    print(f"bucket_tip: {bucket_tip_base.tolist()}")
    print(f"target: {args.target_kind}:{args.target_id}")
    print(f"output: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
