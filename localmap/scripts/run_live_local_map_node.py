#!/usr/bin/env python3
"""实时从machine_root点云生成最小LocalMap JSON。"""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
except ModuleNotFoundError as exc:
    # 关键：实时LocalMap节点依赖ROS2 Python模块，必须用source后的系统Python运行。
    raise SystemExit(
        "无法导入ROS2 Python模块。请先执行：\n"
        "  source /opt/ros/jazzy/setup.zsh\n"
        "  source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh\n"
        "然后用系统Python运行：\n"
        "  /usr/bin/python3 localmap/scripts/run_live_local_map_node.py\n"
        f"原始错误: {exc}"
    ) from exc


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.generator import build_local_map
from localmap_core.geometry import Transform, bounds_mask, finite_xyz_mask
from localmap_core.io import load_json, write_json


POINT_FIELDS = ("x", "y", "z", "intensity", "ring", "timestamp")
DEFAULT_TARGETS = LOCALMAP_DIR / "config" / "targets.mock.json"
DEFAULT_OUTPUT = LOCALMAP_DIR / "exports" / "live_latest" / "local_map.live.json"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造实时LocalMap节点参数。"""
    parser = argparse.ArgumentParser(description="从实时machine_root点云生成LocalMap JSON。")
    parser.add_argument("--input-topic", default="/localmap/machine_root_points", help="已转换到目标frame的PointCloud2 topic")
    parser.add_argument("--output-json", type=Path, default=DEFAULT_OUTPUT, help="LocalMap JSON输出路径")
    parser.add_argument("--publish-topic", default="/localmap/local_map_json", help="可选发布LocalMap JSON字符串topic")
    parser.add_argument("--targets", type=Path, default=DEFAULT_TARGETS, help="dig/dump target配置JSON")
    parser.add_argument("--expected-frame", default="machine_root", help="期望输入点云frame_id")
    parser.add_argument("--up-axis", choices=["x", "y", "z"], default="y", help="目标坐标系竖直向上轴")
    parser.add_argument("--write-every", type=int, default=10, help="每收到多少帧写一次JSON；1表示每帧写")
    parser.add_argument("--publish-every", type=int, default=10, help="每收到多少帧发布一次JSON；0表示不发布")
    parser.add_argument("--log-every", type=int, default=30, help="每处理多少帧打印一次摘要；0表示关闭")
    parser.add_argument(
        "--bounds",
        type=float,
        nargs=6,
        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX", "Z_MIN", "Z_MAX"),
        help="可选点云裁剪范围，单位米，作用在目标frame点云上",
    )
    return parser


def parse_bounds(values: list[float] | None) -> dict[str, list[float]] | None:
    """把命令行裁剪范围整理成geometry.bounds_mask需要的格式。"""
    if values is None:
        return None
    x_min, x_max, y_min, y_max, z_min, z_max = values
    return {"x": [x_min, x_max], "y": [y_min, y_max], "z": [z_min, z_max]}


def structured_to_matrix(points: np.ndarray) -> np.ndarray:
    """把ROS结构化PointCloud2数组转为Nx6矩阵，列顺序固定为XYZIRT。"""
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
    # 关键：LocalMap只消费有效XYZ点，保持和离线bag处理一致。
    return matrix[finite_xyz_mask(matrix)]


def identity_transform(frame_id: str) -> Transform:
    """构造目标frame内的恒等变换；输入点云已经由实时转换节点对齐。"""
    return Transform(
        from_frame=frame_id,
        to_frame=frame_id,
        translation_m=np.array([0.0, 0.0, 0.0], dtype=np.float64),
        rotation_rpy_rad=np.array([0.0, 0.0, 0.0], dtype=np.float64),
        identifier=f"T_{frame_id}_{frame_id}.live_identity",
        status="measured",
    )


def atomic_write_json(path: Path, data: dict) -> None:
    """原子写LocalMap文件，避免RRT读取到半截JSON。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, delete=False) as temp_file:
        temp_path = Path(temp_file.name)
    try:
        write_json(temp_path, data)
        os.replace(temp_path, path)
    finally:
        if temp_path.exists():
            temp_path.unlink()


class LiveLocalMapNode(Node):
    """ROS2节点：订阅目标坐标点云，生成最小LocalMap。"""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("airy_live_local_map_node")
        self.args = args
        self.targets = load_json(args.targets)
        self.bounds = parse_bounds(args.bounds)
        self.frame_count = 0
        self.last_local_map: dict | None = None
        self.publisher = self.create_publisher(String, args.publish_topic, 10)
        self.subscription = self.create_subscription(PointCloud2, args.input_topic, self.on_cloud, 10)

        self.get_logger().info(
            "live LocalMap started: "
            f"input={args.input_topic}, expected_frame={args.expected_frame}, "
            f"output_json={args.output_json}, targets={args.targets}"
        )

    def on_cloud(self, message: PointCloud2) -> None:
        """点云回调：生成LocalMap，并按配置写文件/发布JSON。"""
        self.frame_count += 1
        if message.header.frame_id != self.args.expected_frame:
            self.get_logger().warning(
                f"收到frame_id={message.header.frame_id}，期望{self.args.expected_frame}；仍生成LocalMap",
                throttle_duration_sec=5.0,
            )

        raw_points = point_cloud2.read_points(message, field_names=POINT_FIELDS, skip_nans=False)
        points = structured_to_matrix(raw_points)
        if self.bounds is not None:
            # 关键：裁剪是在machine_root中进行，用于限制RRT关注的工作空间。
            points = points[bounds_mask(points, self.bounds)]

        timestamp_s = message.header.stamp.sec + message.header.stamp.nanosec / 1_000_000_000.0
        transform = identity_transform(message.header.frame_id)
        local_map = build_local_map(
            points_base=points,
            timestamp_s=timestamp_s,
            raw_topic=self.args.input_topic,
            raw_frame_id=message.header.frame_id,
            raw_point_type="XYZIRT",
            bag_path="live",
            transform=transform,
            targets=self.targets,
            up_axis=self.args.up_axis,
        )
        local_map["notes"] = [
            "实时LocalMap第一版：输入点云已由transform_live_cloud_to_base.py转换到统一frame。",
            "当前obstacles为空，dig/dump target来自配置；后续接入heightmap/voxel提取。",
        ]
        self.last_local_map = local_map

        if self.args.write_every > 0 and self.frame_count % self.args.write_every == 0:
            atomic_write_json(self.args.output_json, local_map)

        if self.args.publish_every > 0 and self.frame_count % self.args.publish_every == 0:
            msg = String()
            # 关键：先用JSON字符串topic打通链路，避免第一版引入自定义ROS消息编译负担。
            import json

            msg.data = json.dumps(local_map, ensure_ascii=False)
            self.publisher.publish(msg)

        if self.args.log_every > 0 and self.frame_count % self.args.log_every == 0:
            ground = local_map["ground"]["model"]
            self.get_logger().info(
                f"frames={self.frame_count}, points={points.shape[0]}, "
                f"frame={local_map['frame_id']}, ground_offset={ground['offset_m']:.3f}, "
                f"output={self.args.output_json}"
            )


def main() -> int:
    """入口函数：启动实时LocalMap节点。"""
    args = build_arg_parser().parse_args()
    rclpy.init()
    node = LiveLocalMapNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("收到退出信号，写出最后一帧LocalMap后关闭。", flush=True)
    finally:
        if node.last_local_map is not None:
            atomic_write_json(args.output_json, node.last_local_map)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
