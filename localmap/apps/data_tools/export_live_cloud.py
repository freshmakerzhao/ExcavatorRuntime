#!/usr/bin/env python3
"""从在线/rslidar_points抓取一帧PointCloud2并导出为NPZ/CSV。"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

try:
    import numpy as np
    import rclpy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
except ModuleNotFoundError as exc:
    # 关键：ROS2 Jazzy Python模块依赖系统Python，conda python可能无法加载。
    raise SystemExit(
        "无法导入ROS2 Python模块。请先执行：\n"
        "  source /opt/ros/jazzy/setup.zsh\n"
        "然后用系统Python运行：\n"
        "  /usr/bin/python3 localmap/scripts/export_live_cloud.py\n"
        f"原始错误: {exc}"
    ) from exc


POINT_TOPIC = "/rslidar_points"
POINT_FIELDS = ("x", "y", "z", "intensity", "ring", "timestamp")
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[2] / "exports" / "live_latest"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造在线抓帧参数；默认只抓一帧，避免长时间占用实时链路。"""
    parser = argparse.ArgumentParser(description="从在线/rslidar_points抓一帧并导出NPZ/CSV。")
    parser.add_argument("--topic", default=POINT_TOPIC, help="PointCloud2 topic，默认/rslidar_points")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="输出目录")
    parser.add_argument("--timeout-s", type=float, default=5.0, help="等待点云的超时时间")
    parser.add_argument("--max-csv-points", type=int, default=2000, help="CSV最多导出有效点数量")
    parser.add_argument("--keep-nan", action="store_true", help="NPZ保留NaN点槽；默认只保存有效XYZ点")
    return parser


def structured_to_matrix(points: np.ndarray, keep_nan: bool) -> np.ndarray:
    """把XYZIRT结构化数组转成N x 6矩阵。"""
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
    if keep_nan:
        return matrix

    # 关键：实时LocalMap也必须过滤NaN，保持和离线bag处理一致。
    valid_mask = np.all(np.isfinite(matrix[:, 0:3]), axis=1)
    return matrix[valid_mask]


def write_csv_sample(csv_path: Path, matrix: np.ndarray, max_points: int) -> int:
    """写CSV样本，便于人工快速查看在线点云数值。"""
    sample = matrix[:max_points]
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["x", "y", "z", "intensity", "ring", "timestamp"])
        writer.writerows(sample.tolist())
    return int(sample.shape[0])


class OneShotCloudExporter:
    """订阅一次PointCloud2并保存；类里只保留ROS回调所需状态。"""

    def __init__(self, topic: str, output_dir: Path, keep_nan: bool, max_csv_points: int) -> None:
        """初始化订阅器和导出参数。"""
        self.topic = topic
        self.output_dir = output_dir
        self.keep_nan = keep_nan
        self.max_csv_points = max_csv_points
        self.saved = False
        self.node = rclpy.create_node("airy_export_live_cloud")
        self.subscription = self.node.create_subscription(PointCloud2, topic, self.on_cloud, 10)

    def on_cloud(self, message: PointCloud2) -> None:
        """收到第一帧点云后导出，然后标记完成。"""
        if self.saved:
            return

        points = point_cloud2.read_points(message, field_names=POINT_FIELDS, skip_nans=False)
        matrix = structured_to_matrix(points, keep_nan=self.keep_nan)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        npz_path = self.output_dir / "rslidar_points_live_frame.npz"
        csv_path = self.output_dir / "rslidar_points_live_frame_sample.csv"

        # 关键：NPZ格式与export_first_cloud.py保持一致，后续LocalMap生成脚本可以复用。
        np.savez_compressed(
            npz_path,
            points=matrix,
            columns=np.array(["x", "y", "z", "intensity", "ring", "timestamp"]),
            frame_id=np.array(message.header.frame_id),
            stamp_sec=np.array(message.header.stamp.sec),
            stamp_nanosec=np.array(message.header.stamp.nanosec),
            bag_time_ns=np.array(0),
            raw_height=np.array(message.height),
            raw_width=np.array(message.width),
            raw_point_step=np.array(message.point_step),
            raw_row_step=np.array(message.row_step),
            keep_nan=np.array(self.keep_nan),
        )
        csv_count = write_csv_sample(csv_path, matrix, self.max_csv_points)

        print(f"topic: {self.topic}")
        print(f"frame_id: {message.header.frame_id}")
        print(f"stamp: {message.header.stamp.sec}.{message.header.stamp.nanosec:09d}")
        print(f"raw_points: {points.shape[0]}")
        print(f"exported_points: {matrix.shape[0]}")
        print(f"layout: height={message.height} width={message.width} point_step={message.point_step}")
        print(f"npz: {npz_path}")
        print(f"csv_sample: {csv_path} rows={csv_count}")
        self.saved = True


def main() -> int:
    """入口函数：启动ROS节点，等待第一帧点云或超时退出。"""
    args = build_arg_parser().parse_args()
    rclpy.init()
    exporter = OneShotCloudExporter(args.topic, args.output_dir, args.keep_nan, args.max_csv_points)

    deadline = exporter.node.get_clock().now().nanoseconds + int(args.timeout_s * 1_000_000_000)
    try:
        while rclpy.ok() and not exporter.saved:
            rclpy.spin_once(exporter.node, timeout_sec=0.1)
            if exporter.node.get_clock().now().nanoseconds >= deadline:
                raise SystemExit(f"等待 {args.topic} 超时 {args.timeout_s}s，请确认rslidar_sdk正在发布。")
    finally:
        exporter.node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
