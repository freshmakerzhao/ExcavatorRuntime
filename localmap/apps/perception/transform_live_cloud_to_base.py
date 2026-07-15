#!/usr/bin/env python3
"""实时把/rslidar_points变换到machine_root/base坐标系并重新发布。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from tf2_ros import StaticTransformBroadcaster
except ModuleNotFoundError as exc:
    # 关键：ROS2实时脚本必须运行在已source ROS环境的系统Python里。
    raise SystemExit(
        "无法导入ROS2 Python模块。请先执行：\n"
        "  source /opt/ros/jazzy/setup.zsh\n"
        "  source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh\n"
        "然后用系统Python运行：\n"
        "  /usr/bin/python3 localmap/scripts/transform_live_cloud_to_base.py\n"
        f"原始错误: {exc}"
    ) from exc


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.geometry import finite_xyz_mask, transform_xyzirt_points
from localmap_core.io import load_extrinsics


DEFAULT_EXTRINSICS = LOCALMAP_DIR / "config" / "extrinsics_rslidar_to_machine_root_ros.derived.v1.json"
POINT_FIELDS = ("x", "y", "z", "intensity", "ring", "timestamp")
POINT_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
        ("ring", "<u2"),
        ("timestamp", "<f8"),
    ]
)
ROS_POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
    PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
]


def build_arg_parser() -> argparse.ArgumentParser:
    """构造实时转换节点参数。"""
    parser = argparse.ArgumentParser(description="实时发布machine_root_ros右手坐标系下的PointCloud2。")
    parser.add_argument("--input-topic", default="/rslidar_points", help="原始PointCloud2 topic")
    parser.add_argument("--output-topic", default="/localmap/machine_root_ros_points", help="转换后的PointCloud2 topic")
    parser.add_argument("--extrinsics", type=Path, default=DEFAULT_EXTRINSICS, help="rslidar到目标frame的外参JSON")
    parser.add_argument("--qos-reliability", choices=["reliable", "best_effort"], default="reliable", help="订阅/发布QoS可靠性")
    parser.add_argument("--tf-parent", default="world", help="静态TF父frame；用于让RViz认识目标frame")
    parser.add_argument("--no-static-tf", action="store_true", help="不发布world->目标frame的静态TF")
    parser.add_argument("--log-every", type=int, default=30, help="每处理多少帧打印一次统计；0表示关闭")
    return parser


def make_qos(reliability: str) -> QoSProfile:
    """生成PointCloud2 QoS；默认reliable以匹配当前RViz配置。"""
    policy = ReliabilityPolicy.RELIABLE if reliability == "reliable" else ReliabilityPolicy.BEST_EFFORT
    return QoSProfile(depth=5, reliability=policy, durability=DurabilityPolicy.VOLATILE)


def structured_points_to_matrix(points: np.ndarray) -> np.ndarray:
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
    # 关键：在线发布给RViz前过滤NaN点槽，避免坐标变换和显示时出现无效点。
    return matrix[finite_xyz_mask(matrix)]


def matrix_to_structured_points(points: np.ndarray) -> np.ndarray:
    """把Nx6矩阵转回PointCloud2需要的结构化数组，保持XYZIRT字段布局。"""
    structured = np.empty(points.shape[0], dtype=POINT_DTYPE)
    structured["x"] = points[:, 0].astype(np.float32)
    structured["y"] = points[:, 1].astype(np.float32)
    structured["z"] = points[:, 2].astype(np.float32)
    structured["intensity"] = points[:, 3].astype(np.float32)
    structured["ring"] = np.clip(points[:, 4], 0, np.iinfo(np.uint16).max).astype(np.uint16)
    structured["timestamp"] = points[:, 5].astype(np.float64)
    return structured


def create_xyzirt_cloud(points: np.ndarray, stamp: object, frame_id: str) -> PointCloud2:
    """用变换后的XYZIRT矩阵创建新的PointCloud2消息。"""
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return point_cloud2.create_cloud(header, ROS_POINT_FIELDS, matrix_to_structured_points(points), point_step=POINT_DTYPE.itemsize)


class LiveCloudTransformer(Node):
    """ROS2节点：订阅原始rslidar点云，发布目标坐标系点云。"""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("airy_live_cloud_transformer")
        self.transform = load_extrinsics(args.extrinsics)
        self.output_topic = args.output_topic
        self.log_every = max(args.log_every, 0)
        self.frame_count = 0
        qos = make_qos(args.qos_reliability)

        # 关键：发布新topic而不覆盖原始/rslidar_points，便于RViz中对照调试。
        self.publisher = self.create_publisher(PointCloud2, args.output_topic, qos)
        self.subscription = self.create_subscription(PointCloud2, args.input_topic, self.on_cloud, qos)

        if not args.no_static_tf:
            self.publish_static_identity_tf(args.tf_parent, self.transform.to_frame)

        self.get_logger().info(
            "live transform started: "
            f"{args.input_topic}({self.transform.from_frame}) -> "
            f"{args.output_topic}({self.transform.to_frame}), extrinsics={args.extrinsics}"
        )

    def publish_static_identity_tf(self, parent_frame: str, child_frame: str) -> None:
        """发布一个静态TF，让RViz能够识别目标frame。"""
        if parent_frame == child_frame:
            self.get_logger().warning("tf_parent与目标frame相同，跳过静态TF发布")
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.static_tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f"published static TF: {parent_frame} -> {child_frame}")

    def on_cloud(self, message: PointCloud2) -> None:
        """点云回调：读取XYZIRT、执行外参变换、发布目标frame点云。"""
        if message.header.frame_id != self.transform.from_frame:
            self.get_logger().warning(
                f"收到frame_id={message.header.frame_id}，但外参from_frame={self.transform.from_frame}；仍按外参执行转换",
                throttle_duration_sec=5.0,
            )

        raw_points = point_cloud2.read_points(message, field_names=POINT_FIELDS, skip_nans=False)
        matrix = structured_points_to_matrix(raw_points)
        transformed = transform_xyzirt_points(matrix, self.transform)
        output = create_xyzirt_cloud(transformed, message.header.stamp, self.transform.to_frame)
        self.publisher.publish(output)

        self.frame_count += 1
        if self.log_every > 0 and self.frame_count % self.log_every == 0:
            self.get_logger().info(
                f"published {self.frame_count} frames, "
                f"valid_points={transformed.shape[0]}, frame_id={self.transform.to_frame}"
            )


def main() -> int:
    """入口函数：启动ROS2实时点云坐标转换节点。"""
    args = build_arg_parser().parse_args()
    rclpy.init()
    node = LiveCloudTransformer(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
