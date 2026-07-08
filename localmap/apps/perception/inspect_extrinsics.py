#!/usr/bin/env python3
"""检查外参配置，并打印坐标轴在目标坐标系中的方向。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.geometry import rpy_to_rotation_matrix
from localmap_core.io import load_extrinsics


DEFAULT_EXTRINSICS = LOCALMAP_DIR / "config" / "extrinsics_rslidar_to_machine_root.measured.json"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造外参检查参数。"""
    parser = argparse.ArgumentParser(description="检查rslidar到目标frame的外参配置。")
    parser.add_argument("--extrinsics", type=Path, default=DEFAULT_EXTRINSICS, help="外参JSON路径")
    return parser


def main() -> int:
    """入口函数：打印平移、RPY和rslidar三轴映射方向。"""
    args = build_arg_parser().parse_args()
    transform = load_extrinsics(args.extrinsics)
    linear = transform.linear_matrix if transform.linear_matrix is not None else rpy_to_rotation_matrix(transform.rotation_rpy_rad)

    axes = {
        "rslidar +X": np.array([1.0, 0.0, 0.0]),
        "rslidar +Y": np.array([0.0, 1.0, 0.0]),
        "rslidar +Z": np.array([0.0, 0.0, 1.0]),
    }

    print(f"extrinsics: {args.extrinsics}")
    print(f"id: {transform.identifier}")
    print(f"from_frame: {transform.from_frame}")
    print(f"to_frame: {transform.to_frame}")
    print(f"status: {transform.status}")
    print(f"translation_m: {transform.translation_m.tolist()}")
    print(f"rotation_rpy_rad: {transform.rotation_rpy_rad.tolist()}")
    print(f"rotation_rpy_deg: {np.rad2deg(transform.rotation_rpy_rad).tolist()}")
    print(f"linear_matrix: {linear.tolist()}")
    print(f"linear_determinant: {float(np.linalg.det(linear))}")
    if transform.linear_matrix is not None:
        print("note: 使用axis_mapping_matrix；可表达右手雷达系到Unity/machine_root轴约定的映射。")

    # 关键：这里打印每根雷达轴在目标frame中的方向，用于现场核对前/右/上是否反了。
    for name, axis in axes.items():
        mapped = linear @ axis
        print(f"{name} -> {transform.to_frame} {mapped.tolist()}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
