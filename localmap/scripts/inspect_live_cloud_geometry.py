#!/usr/bin/env python3
"""实时检查转换后PointCloud2的frame、范围和地面高度。"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
except ModuleNotFoundError as exc:
    # 关键：该脚本依赖ROS2 Python模块，必须使用source后的系统Python运行。
    raise SystemExit(
        "无法导入ROS2 Python模块。请先执行：\n"
        "  source /opt/ros/jazzy/setup.zsh\n"
        "然后用系统Python运行：\n"
        "  /usr/bin/python3 localmap/scripts/inspect_live_cloud_geometry.py\n"
        f"原始错误: {exc}"
    ) from exc


POINT_FIELDS = ("x", "y", "z", "intensity", "ring", "timestamp")


def build_arg_parser() -> argparse.ArgumentParser:
    """构造实时几何检查参数。"""
    parser = argparse.ArgumentParser(description="检查实时PointCloud2是否已进入目标坐标系。")
    parser.add_argument("--topic", default="/localmap/machine_root_points", help="要检查的PointCloud2 topic")
    parser.add_argument("--expected-frame", default="machine_root", help="期望的frame_id")
    parser.add_argument("--frames", type=int, default=3, help="采样帧数")
    parser.add_argument("--timeout-s", type=float, default=5.0, help="等待点云超时时间")
    parser.add_argument("--up-axis", choices=["x", "y", "z"], default="y", help="目标坐标系竖直向上轴")
    parser.add_argument("--csv-output", type=Path, help="可选：保存最后一帧的XYZIRT样本CSV")
    parser.add_argument("--csv-points", type=int, default=2000, help="CSV最多保存点数")
    return parser


def structured_to_matrix(points: np.ndarray) -> np.ndarray:
    """把ROS结构化PointCloud2数组转成Nx6矩阵。"""
    matrix = np.column_stack(
        (
            points["x"].astype(np.float32),
            points["y"].astype(np.float32),
            points["z"].astype(np.float32),
            points["intensity"].astype(np.float32),
            points["ring"].astype(np.float32),
            points["timestamp"].astype(np.float64),
        )
    )
    # 关键：统计只看XYZ有限点，避免NaN点槽影响范围判断。
    valid_mask = np.all(np.isfinite(matrix[:, 0:3]), axis=1)
    return matrix[valid_mask]


def axis_index(axis: str) -> int:
    """把轴名转成XYZ列索引。"""
    return {"x": 0, "y": 1, "z": 2}[axis]


def format_range(values: np.ndarray) -> str:
    """格式化min/max，方便现场读数。"""
    return f"{float(np.min(values)):.3f} .. {float(np.max(values)):.3f}"


def format_percentiles(values: np.ndarray) -> str:
    """格式化常用分位数；地面检查主要看低分位和中位数。"""
    p01, p05, p50, p95, p99 = np.percentile(values, [1, 5, 50, 95, 99])
    return f"p01={p01:.3f}, p05={p05:.3f}, p50={p50:.3f}, p95={p95:.3f}, p99={p99:.3f}"


def write_csv(path: Path, matrix: np.ndarray, max_points: int) -> None:
    """保存最后一帧样本，便于用表格或Python进一步检查。"""
    import csv

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["x", "y", "z", "intensity", "ring", "timestamp"])
        writer.writerows(matrix[:max_points].tolist())


class LiveGeometryInspector:
    """一次性订阅若干帧PointCloud2并输出几何统计。"""

    def __init__(self, args: argparse.Namespace) -> None:
        """初始化检查节点和采样配置。"""
        self.args = args
        self.node = rclpy.create_node("airy_inspect_live_cloud_geometry")
        self.subscription = self.node.create_subscription(PointCloud2, args.topic, self.on_cloud, 10)
        self.samples: list[np.ndarray] = []
        self.last_frame_id = ""
        self.last_layout = {}
        self.last_stamp = ""

    def on_cloud(self, message: PointCloud2) -> None:
        """收到一帧点云后保存有效XYZIRT矩阵。"""
        points = point_cloud2.read_points(message, field_names=POINT_FIELDS, skip_nans=False)
        matrix = structured_to_matrix(points)
        self.samples.append(matrix)
        self.last_frame_id = message.header.frame_id
        self.last_stamp = f"{message.header.stamp.sec}.{message.header.stamp.nanosec:09d}"
        self.last_layout = {
            "height": message.height,
            "width": message.width,
            "point_step": message.point_step,
            "row_step": message.row_step,
            "raw_points": int(points.shape[0]),
            "valid_points": int(matrix.shape[0]),
        }

    def print_report(self) -> None:
        """打印坐标系是否正确的关键读数。"""
        if not self.samples:
            raise SystemExit(f"没有收到 {self.args.topic} 点云")

        matrix = self.samples[-1]
        up = axis_index(self.args.up_axis)
        ground_estimate = float(np.percentile(matrix[:, up], 5))
        frame_ok = self.last_frame_id == self.args.expected_frame

        print(f"topic: {self.args.topic}")
        print(f"frame_id: {self.last_frame_id} expected={self.args.expected_frame} ok={frame_ok}")
        print(f"stamp: {self.last_stamp}")
        print(f"sampled_frames: {len(self.samples)}")
        print(f"layout: {self.last_layout}")
        print(f"x_range_m: {format_range(matrix[:, 0])}")
        print(f"y_range_m: {format_range(matrix[:, 1])}")
        print(f"z_range_m: {format_range(matrix[:, 2])}")
        print(f"{self.args.up_axis}_percentiles_m: {format_percentiles(matrix[:, up])}")
        print(f"ground_estimate_{self.args.up_axis}_m_p05: {ground_estimate:.3f}")
        print("expectation: machine_root中通常 +Y 向上，地面点的Y低分位应接近0m。")
        print("expectation: 你移动已知物体时，它应在对应machine_root方向上变化。")

        if self.args.csv_output:
            write_csv(self.args.csv_output, matrix, self.args.csv_points)
            print(f"csv_output: {self.args.csv_output} rows={min(matrix.shape[0], self.args.csv_points)}")


def main() -> int:
    """入口函数：采样若干帧并打印几何报告。"""
    args = build_arg_parser().parse_args()
    rclpy.init()
    inspector = LiveGeometryInspector(args)

    deadline = inspector.node.get_clock().now().nanoseconds + int(args.timeout_s * 1_000_000_000)
    try:
        while rclpy.ok() and len(inspector.samples) < args.frames:
            rclpy.spin_once(inspector.node, timeout_sec=0.1)
            if inspector.node.get_clock().now().nanoseconds >= deadline:
                raise SystemExit(f"等待 {args.topic} 超时 {args.timeout_s}s，已收到 {len(inspector.samples)} 帧")
        inspector.print_report()
    finally:
        inspector.node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
