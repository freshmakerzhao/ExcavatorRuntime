#!/usr/bin/env python3
"""检查Airy/RoboSense离线bag中的PointCloud2基础信息。

运行建议：
  source /opt/ros/jazzy/setup.zsh
  /usr/bin/python3 inspect_bag_points.py /path/to/bag
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable

try:
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2
except ModuleNotFoundError as exc:
    # 关键：ROS2 Jazzy的Python扩展绑定系统Python 3.12，conda/base的python3会加载失败。
    raise SystemExit(
        "无法导入ROS2 Python模块。请先执行：\n"
        "  source /opt/ros/jazzy/setup.zsh\n"
        "然后用系统Python运行：\n"
        "  /usr/bin/python3 localmap/scripts/inspect_bag_points.py bags/airy_20260706_202359 --frames 3\n"
        f"原始错误: {exc}"
    ) from exc


POINT_TOPIC = "/rslidar_points"
POINT_FIELDS = ("x", "y", "z", "intensity", "ring", "timestamp")


def build_arg_parser() -> argparse.ArgumentParser:
    """构造命令行参数，后续扩展更多topic或统计窗口时集中改这里。"""
    parser = argparse.ArgumentParser(description="检查Airy雷达bag中的/rslidar_points。")
    parser.add_argument("bag", type=Path, help="ros2 bag目录，例如 AiryLidar/bags/airy_xxx")
    parser.add_argument("--topic", default=POINT_TOPIC, help="PointCloud2 topic，默认/rslidar_points")
    parser.add_argument("--frames", type=int, default=5, help="用于统计的点云帧数，默认5帧")
    parser.add_argument(
        "--storage-id",
        default="mcap",
        help="bag存储格式，当前录包为mcap；如后续使用sqlite3可改为sqlite3",
    )
    return parser


def open_reader(bag_path: Path, storage_id: str) -> rosbag2_py.SequentialReader:
    """打开rosbag reader；这里不做写操作，保证离线数据只读。"""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def finite_range(values: np.ndarray) -> tuple[float | None, float | None]:
    """返回有限数值的[min, max]；全NaN时返回None，避免误导后续判断。"""
    finite_mask = np.isfinite(values)
    if not np.any(finite_mask):
        return None, None
    finite_values = values[finite_mask]
    return float(np.min(finite_values)), float(np.max(finite_values))


def summarize_points(points: np.ndarray) -> dict[str, object]:
    """统计一帧点云的有效点、XYZ范围、强度、ring和逐点时间戳范围。"""
    xyz = np.column_stack((points["x"], points["y"], points["z"]))
    finite_xyz_mask = np.all(np.isfinite(xyz), axis=1)
    valid_xyz = xyz[finite_xyz_mask]

    # 关键：RoboSense会保留NaN点槽，LocalMap阶段必须先过滤无效XYZ。
    summary: dict[str, object] = {
        "total_points": int(points.shape[0]),
        "valid_xyz_points": int(valid_xyz.shape[0]),
        "nan_xyz_points": int(points.shape[0] - valid_xyz.shape[0]),
    }

    for axis_name in ("x", "y", "z", "intensity", "timestamp"):
        summary[f"{axis_name}_range"] = finite_range(points[axis_name])

    summary["ring_range"] = (
        int(np.min(points["ring"])) if points.shape[0] else None,
        int(np.max(points["ring"])) if points.shape[0] else None,
    )
    return summary


def summarize_layouts(layouts: list[dict[str, object]]) -> dict[str, object]:
    """汇总多帧PointCloud2布局，方便发现SDK分帧宽度是否有轻微变化。"""
    heights = [int(layout["height"]) for layout in layouts]
    widths = [int(layout["width"]) for layout in layouts]
    point_steps = [int(layout["point_step"]) for layout in layouts]
    row_steps = [int(layout["row_step"]) for layout in layouts]
    return {
        "height_range": (min(heights), max(heights)),
        "width_range": (min(widths), max(widths)),
        "point_step_range": (min(point_steps), max(point_steps)),
        "row_step_range": (min(row_steps), max(row_steps)),
    }


def iter_topic_messages(
    reader: rosbag2_py.SequentialReader,
    topic: str,
    message_type: object,
) -> Iterable[tuple[object, int]]:
    """按顺序遍历指定topic消息；timestamp_ns来自bag记录时间。"""
    while reader.has_next():
        current_topic, data, timestamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        yield deserialize_message(data, message_type), int(timestamp_ns)


def main() -> int:
    """入口函数：读取若干帧并打印人能直接记笔记的摘要。"""
    args = build_arg_parser().parse_args()
    if not args.bag.exists():
        raise SystemExit(f"bag目录不存在: {args.bag}")

    reader = open_reader(args.bag, args.storage_id)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if args.topic not in topic_types:
        available = ", ".join(sorted(topic_types))
        raise SystemExit(f"找不到topic {args.topic}，当前bag包含: {available}")

    message_type = get_message(topic_types[args.topic])
    frame_summaries: list[dict[str, object]] = []
    first_header = None
    first_fields = None
    first_layout = None
    layouts: list[dict[str, object]] = []

    for message, timestamp_ns in iter_topic_messages(reader, args.topic, message_type):
        # 关键：只读取RSLidar当前XYZIRT字段，后续若改point_type需要同步这里。
        points = point_cloud2.read_points(message, field_names=POINT_FIELDS, skip_nans=False)
        summary = summarize_points(points)
        summary["bag_time_s"] = timestamp_ns / 1_000_000_000.0
        frame_summaries.append(summary)

        # 关键：Airy/RSLidar分帧时width可能轻微变化，记录每帧layout便于区分正常分帧和异常丢包。
        layouts.append(
            {
                "height": message.height,
                "width": message.width,
                "point_step": message.point_step,
                "row_step": message.row_step,
                "is_dense": bool(message.is_dense),
            }
        )

        if first_header is None:
            first_header = message.header
            first_fields = [(f.name, f.offset, f.datatype, f.count) for f in message.fields]
            first_layout = {
                "height": message.height,
                "width": message.width,
                "point_step": message.point_step,
                "row_step": message.row_step,
                "is_dense": bool(message.is_dense),
            }

        if len(frame_summaries) >= args.frames:
            break

    if not frame_summaries:
        raise SystemExit(f"topic {args.topic} 中没有读到点云帧")

    valid_counts = np.array([s["valid_xyz_points"] for s in frame_summaries], dtype=np.float64)
    total_counts = np.array([s["total_points"] for s in frame_summaries], dtype=np.float64)

    print(f"bag: {args.bag}")
    print(f"topic: {args.topic}")
    print(f"message_type: {topic_types[args.topic]}")
    print(f"sampled_frames: {len(frame_summaries)}")
    print(f"frame_id: {first_header.frame_id}")
    print(f"stamp: {first_header.stamp.sec}.{first_header.stamp.nanosec:09d}")
    print(f"layout: {first_layout}")
    print(f"layout_ranges: {summarize_layouts(layouts)}")
    print(f"fields: {first_fields}")
    print(f"points_per_frame: min={int(total_counts.min())} max={int(total_counts.max())}")
    print(
        "valid_xyz_per_frame: "
        f"min={int(valid_counts.min())} max={int(valid_counts.max())} "
        f"mean={valid_counts.mean():.1f}"
    )

    for index, summary in enumerate(frame_summaries):
        # 关键：逐帧打印XYZ范围，方便判断雷达姿态、遮挡和坐标方向是否异常。
        print(
            f"frame[{index}]: "
            f"bag_time_s={summary['bag_time_s']:.6f} "
            f"valid={summary['valid_xyz_points']}/{summary['total_points']} "
            f"x={summary['x_range']} y={summary['y_range']} z={summary['z_range']} "
            f"intensity={summary['intensity_range']} ring={summary['ring_range']} "
            f"timestamp={summary['timestamp_range']}"
        )

    if math.isclose(float(valid_counts.mean()), 0.0):
        print("warning: 有效XYZ点数为0，请检查DIFOP、point_type或雷达遮挡。")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
