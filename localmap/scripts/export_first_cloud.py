#!/usr/bin/env python3
"""从Airy/RoboSense rosbag导出首帧PointCloud2，供离线LocalMap算法开发使用。"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

try:
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2
except ModuleNotFoundError as exc:
    # 关键：不要用conda/base的python3跑ROS2 bag脚本，要用/usr/bin/python3。
    raise SystemExit(
        "无法导入ROS2 Python模块。请先执行：\n"
        "  source /opt/ros/jazzy/setup.zsh\n"
        "然后用系统Python运行：\n"
        "  /usr/bin/python3 localmap/scripts/export_first_cloud.py bags/airy_20260706_202359\n"
        f"原始错误: {exc}"
    ) from exc


POINT_TOPIC = "/rslidar_points"
POINT_FIELDS = ("x", "y", "z", "intensity", "ring", "timestamp")
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[1] / "exports"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造导出参数；默认导出npz，CSV只导出采样点方便肉眼检查。"""
    parser = argparse.ArgumentParser(description="导出bag中第一帧/rslidar_points。")
    parser.add_argument("bag", type=Path, help="ros2 bag目录")
    parser.add_argument("--topic", default=POINT_TOPIC, help="PointCloud2 topic，默认/rslidar_points")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="输出目录，默认AiryLidar/localmap/exports",
    )
    parser.add_argument("--storage-id", default="mcap", help="bag存储格式，默认mcap")
    parser.add_argument("--max-csv-points", type=int, default=20000, help="CSV最多导出的有效点数")
    parser.add_argument("--keep-nan", action="store_true", help="NPZ中保留NaN点槽；默认只保存有效XYZ点")
    return parser


def open_reader(bag_path: Path, storage_id: str) -> rosbag2_py.SequentialReader:
    """打开rosbag reader；导出过程只读bag，不改变原始离线数据。"""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def find_first_cloud(
    reader: rosbag2_py.SequentialReader,
    topic: str,
    message_type: object,
) -> tuple[object, int]:
    """找到指定topic第一帧点云；第一版离线工具先以单帧为最小样本。"""
    while reader.has_next():
        current_topic, data, timestamp_ns = reader.read_next()
        if current_topic == topic:
            return deserialize_message(data, message_type), int(timestamp_ns)
    raise RuntimeError(f"topic {topic} 中没有点云消息")


def structured_to_matrix(points: np.ndarray, keep_nan: bool) -> np.ndarray:
    """把XYZIRT结构化数组转成二维矩阵，便于npz/csv和后续算法读取。"""
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

    # 关键：LocalMap算法第一步必须过滤无效XYZ，避免NaN进入地面/障碍拟合。
    valid_mask = np.all(np.isfinite(matrix[:, 0:3]), axis=1)
    return matrix[valid_mask]


def write_csv_sample(csv_path: Path, matrix: np.ndarray, max_points: int) -> int:
    """写一个小CSV样本，方便不用ROS工具也能快速查看点云数值。"""
    sample = matrix[:max_points]
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["x", "y", "z", "intensity", "ring", "timestamp"])
        writer.writerows(sample.tolist())
    return int(sample.shape[0])


def main() -> int:
    """入口函数：导出首帧NPZ和CSV样本，并打印元数据摘要。"""
    args = build_arg_parser().parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    reader = open_reader(args.bag, args.storage_id)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if args.topic not in topic_types:
        available = ", ".join(sorted(topic_types))
        raise SystemExit(f"找不到topic {args.topic}，当前bag包含: {available}")

    message_type = get_message(topic_types[args.topic])
    message, timestamp_ns = find_first_cloud(reader, args.topic, message_type)
    points = point_cloud2.read_points(message, field_names=POINT_FIELDS, skip_nans=False)
    matrix = structured_to_matrix(points, keep_nan=args.keep_nan)

    stem = f"{args.topic.strip('/').replace('/', '_')}_first_frame"
    npz_path = args.output_dir / f"{stem}.npz"
    csv_path = args.output_dir / f"{stem}_sample.csv"

    # 关键：NPZ保存完整数值矩阵和元数据，后续LocalMap原型可直接np.load读取。
    np.savez_compressed(
        npz_path,
        points=matrix,
        columns=np.array(["x", "y", "z", "intensity", "ring", "timestamp"]),
        frame_id=np.array(message.header.frame_id),
        stamp_sec=np.array(message.header.stamp.sec),
        stamp_nanosec=np.array(message.header.stamp.nanosec),
        bag_time_ns=np.array(timestamp_ns),
        raw_height=np.array(message.height),
        raw_width=np.array(message.width),
        raw_point_step=np.array(message.point_step),
        raw_row_step=np.array(message.row_step),
        keep_nan=np.array(args.keep_nan),
    )
    csv_count = write_csv_sample(csv_path, matrix, args.max_csv_points)

    print(f"bag: {args.bag}")
    print(f"topic: {args.topic}")
    print(f"frame_id: {message.header.frame_id}")
    print(f"stamp: {message.header.stamp.sec}.{message.header.stamp.nanosec:09d}")
    print(f"raw_points: {points.shape[0]}")
    print(f"exported_points: {matrix.shape[0]}")
    print(f"npz: {npz_path}")
    print(f"csv_sample: {csv_path} rows={csv_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
