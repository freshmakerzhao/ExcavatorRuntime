#!/usr/bin/env python3
"""把TF项目输出的bucket tip位姿桥接到machine_root规划坐标系。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
except ModuleNotFoundError as exc:
    raise SystemExit(
        "无法导入ROS2 Python模块。请先source ROS环境，并使用/usr/bin/python3运行。\n"
        f"原始错误: {exc}"
    ) from exc


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.bucket_tip_bridge import build_bucket_tip_state, load_bucket_tip_frame_bridge
from localmap_core.io import write_json


DEFAULT_BRIDGE = LOCALMAP_DIR / "config" / "bucket_tip_tf_bridge.machine_root.json"
DEFAULT_OUTPUT_JSON = LOCALMAP_DIR / "exports" / "live_latest" / "bucket_tip.machine_root.live.json"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造命令行参数，默认对接excavator_kinematics包的输出。"""
    parser = argparse.ArgumentParser(description="把/bucket_tip_pose_map转换为machine_root bucket tip状态。")
    parser.add_argument("--input-topic", default="/bucket_tip_pose_map", help="TF/FK输出的PoseStamped topic")
    parser.add_argument("--output-topic", default="/localmap/bucket_tip_machine_root_pose", help="转换后的PoseStamped topic")
    parser.add_argument("--bridge", type=Path, default=DEFAULT_BRIDGE, help="fk_root到machine_root的bucket tip桥接配置")
    parser.add_argument("--output-json", type=Path, default=DEFAULT_OUTPUT_JSON, help="写给规划脚本读取的bucket tip JSON")
    parser.add_argument("--write-every", type=int, default=5, help="每收到多少帧写一次JSON；1表示每帧都写")
    parser.add_argument("--log-every", type=int, default=30, help="每处理多少帧打印一次统计；0表示关闭")
    return parser


def stamp_to_seconds(stamp: object) -> float:
    """把ROS stamp转换成浮点秒，便于写入JSON和离线排查。"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class BucketTipBridgeNode(Node):
    """ROS2节点：订阅FK内部bucket tip，发布machine_root bucket tip并写JSON。"""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("airy_bucket_tip_tf_bridge")
        self.bridge = load_bucket_tip_frame_bridge(args.bridge)
        self.input_topic = args.input_topic
        self.output_json = args.output_json
        self.write_every = max(args.write_every, 1)
        self.log_every = max(args.log_every, 0)
        self.frame_count = 0

        self.publisher = self.create_publisher(PoseStamped, args.output_topic, 10)
        self.subscription = self.create_subscription(PoseStamped, args.input_topic, self.on_pose, 20)
        self.get_logger().info(
            "bucket tip bridge started: "
            f"{args.input_topic}({self.bridge.source_frame}) -> "
            f"{args.output_topic}({self.bridge.target_frame}), json={args.output_json}"
        )

    def on_pose(self, message: PoseStamped) -> None:
        """位姿回调：只把位置作为规划权威输入，姿态后续再进入完整执行链路。"""
        if message.header.frame_id != self.bridge.source_frame:
            self.get_logger().warning(
                f"收到frame_id={message.header.frame_id}，但bridge source_frame={self.bridge.source_frame}；仍按bridge转换",
                throttle_duration_sec=5.0,
            )

        source_position = np.array(
            [message.pose.position.x, message.pose.position.y, message.pose.position.z],
            dtype=np.float64,
        )
        machine_position = self.bridge.transform_position(source_position)
        output = self.build_output_pose(message, machine_position)
        self.publisher.publish(output)

        self.frame_count += 1
        if self.frame_count % self.write_every == 0:
            state = build_bucket_tip_state(
                position_m=machine_position,
                frame_id=self.bridge.target_frame,
                stamp_s=stamp_to_seconds(message.header.stamp),
                source_topic=self.input_topic,
                bridge=self.bridge,
            )
            # 关键：run_planning_once.sh按文件读取bucket tip，因此这里持续刷新最新状态。
            write_json(self.output_json, state)

        if self.log_every > 0 and self.frame_count % self.log_every == 0:
            self.get_logger().info(
                f"published {self.frame_count} bucket tip frames, "
                f"position_m={machine_position.astype(float).round(4).tolist()}"
            )

    def build_output_pose(self, message: PoseStamped, machine_position: np.ndarray) -> PoseStamped:
        """创建machine_root PoseStamped；第一版姿态置为单位四元数，避免错误复用ROS源姿态。"""
        output = PoseStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.bridge.target_frame
        output.pose.position.x = float(machine_position[0])
        output.pose.position.y = float(machine_position[1])
        output.pose.position.z = float(machine_position[2])
        output.pose.orientation.x = 0.0
        output.pose.orientation.y = 0.0
        output.pose.orientation.z = 0.0
        output.pose.orientation.w = 1.0
        return output


def main() -> int:
    """入口函数：启动bucket tip桥接节点直到Ctrl+C。"""
    args = build_arg_parser().parse_args()
    rclpy.init()
    node = BucketTipBridgeNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
