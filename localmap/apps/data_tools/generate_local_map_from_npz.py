#!/usr/bin/env python3
"""从离线NPZ点云生成第一版LocalMap JSON。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.generator import build_local_map
from localmap_core.geometry import preprocess_points
from localmap_core.io import load_extrinsics, load_json, load_npz_points, write_json


DEFAULT_NPZ = LOCALMAP_DIR / "exports" / "rslidar_points_first_frame.npz"
DEFAULT_EXTRINSICS = LOCALMAP_DIR / "config" / "extrinsics_rslidar_to_machine_root_ros.derived.v1.json"
DEFAULT_TARGETS = LOCALMAP_DIR / "config" / "targets.machine_root_ros.derived.v1.json"
DEFAULT_OUTPUT = LOCALMAP_DIR / "exports" / "local_map_from_npz.mock.json"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造命令行参数；真实外参和target后续都通过配置替换。"""
    parser = argparse.ArgumentParser(description="从Airy离线NPZ点云生成LocalMap JSON。")
    parser.add_argument("--npz", type=Path, default=DEFAULT_NPZ, help="export_first_cloud.py导出的NPZ")
    parser.add_argument("--extrinsics", type=Path, default=DEFAULT_EXTRINSICS, help="rslidar到machine_root外参JSON")
    parser.add_argument("--targets", type=Path, default=DEFAULT_TARGETS, help="dig/dump target配置JSON")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="输出LocalMap JSON")
    parser.add_argument("--raw-topic", default="/rslidar_points", help="原始点云topic")
    parser.add_argument("--raw-point-type", default="XYZIRT", help="原始点云点类型")
    parser.add_argument("--bag-path", default="bags/airy_20260706_202359", help="离线bag相对路径记录")
    parser.add_argument("--up-axis", choices=["x", "y", "z"], default="y", help="目标frame的竖直向上轴，Unity/machine_root默认y")
    parser.add_argument(
        "--bounds",
        type=float,
        nargs=6,
        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX", "Z_MIN", "Z_MAX"),
        help="可选裁剪范围，单位米，作用在machine_root点云上",
    )
    return parser


def parse_bounds(values: list[float] | None) -> dict[str, list[float]] | None:
    """把命令行六个数整理成XYZ裁剪配置；未传入时不裁剪。"""
    if values is None:
        return None
    x_min, x_max, y_min, y_max, z_min, z_max = values
    return {"x": [x_min, x_max], "y": [y_min, y_max], "z": [z_min, z_max]}


def main() -> int:
    """入口函数：NPZ点云 -> 外参变换 -> LocalMap JSON。"""
    args = build_arg_parser().parse_args()
    if not args.npz.exists():
        raise SystemExit(f"找不到NPZ点云: {args.npz}\n请先运行 scripts/export_first_cloud.py。")

    points, metadata = load_npz_points(args.npz)
    transform = load_extrinsics(args.extrinsics)
    targets = load_json(args.targets)
    bounds = parse_bounds(args.bounds)

    # 关键：这里完成rslidar到machine_root/MachineRoot的变换，RRT*只看输出LocalMap。
    points_base = preprocess_points(points, transform, bounds=bounds)
    timestamp_s = metadata["stamp_sec"] + metadata["stamp_nanosec"] / 1_000_000_000.0

    local_map = build_local_map(
        points_base=points_base,
        timestamp_s=timestamp_s,
        raw_topic=args.raw_topic,
        raw_frame_id=metadata["frame_id"],
        raw_point_type=args.raw_point_type,
        bag_path=args.bag_path,
        transform=transform,
        targets=targets,
        up_axis=args.up_axis,
    )
    write_json(args.output, local_map)

    print(f"npz: {args.npz}")
    print(f"input_points: {points.shape[0]}")
    print(f"machine_root_points: {points_base.shape[0]}")
    print(f"from_frame: {transform.from_frame}")
    print(f"to_frame: {transform.to_frame}")
    print(f"output: {args.output}")
    print(f"ground: {local_map['ground']}")
    print(f"dig_targets: {len(local_map['dig_targets'])}")
    print(f"dump_targets: {len(local_map['dump_targets'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
