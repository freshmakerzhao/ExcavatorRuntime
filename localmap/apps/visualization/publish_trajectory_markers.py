#!/usr/bin/env python3
"""把TrajectoryCommand发布成RViz MarkerArray，便于检查bucket-tip规划结果。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray
except ModuleNotFoundError as exc:
    raise SystemExit(
        "无法导入ROS2 Python模块。请先source ROS环境，并使用/usr/bin/python3运行。\n"
        f"原始错误: {exc}"
    ) from exc


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.io import load_json


DEFAULT_TRAJECTORY = LOCALMAP_DIR / "exports" / "trajectory_command.simple_rrt.json"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造轨迹可视化参数。"""
    parser = argparse.ArgumentParser(description="发布TrajectoryCommand RViz MarkerArray。")
    parser.add_argument("--trajectory", type=Path, default=DEFAULT_TRAJECTORY, help="TrajectoryCommand JSON")
    parser.add_argument("--topic", default="/localmap/planned_bucket_tip_markers", help="MarkerArray输出topic")
    parser.add_argument("--rate-hz", type=float, default=2.0, help="重复发布频率，便于RViz后打开也能看到")
    return parser


def make_color(red: float, green: float, blue: float, alpha: float = 1.0) -> ColorRGBA:
    """创建RViz颜色。"""
    color = ColorRGBA()
    color.r = float(red)
    color.g = float(green)
    color.b = float(blue)
    color.a = float(alpha)
    return color


def make_point(values: list[float]) -> Point:
    """把JSON里的[x,y,z]转换为ROS Point。"""
    point = Point()
    point.x = float(values[0])
    point.y = float(values[1])
    point.z = float(values[2])
    return point


class TrajectoryMarkerPublisher(Node):
    """周期发布规划轨迹的线段和waypoint点。"""

    def __init__(self, trajectory_path: Path, topic: str, rate_hz: float) -> None:
        super().__init__("airy_trajectory_marker_publisher")
        self.trajectory_path = trajectory_path
        self.trajectory: dict | None = None
        self.last_mtime_ns: int | None = None
        self.publisher = self.create_publisher(MarkerArray, topic, 1)
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self.publish_markers)
        self.get_logger().info(f"publishing trajectory markers: {trajectory_path} -> {topic}")

    def reload_if_changed(self) -> bool:
        """轨迹文件变化时重新读取，便于RViz持续显示最新规划路径。"""
        if not self.trajectory_path.exists():
            self.get_logger().warning(f"trajectory文件不存在，等待生成: {self.trajectory_path}", throttle_duration_sec=5.0)
            return False
        mtime_ns = self.trajectory_path.stat().st_mtime_ns
        if self.trajectory is not None and self.last_mtime_ns == mtime_ns:
            return True
        # 关键：发布节点可长期运行，规划脚本每次覆盖JSON后自动反映到RViz。
        self.trajectory = load_json(self.trajectory_path)
        self.last_mtime_ns = mtime_ns
        self.get_logger().info(f"loaded trajectory: {self.trajectory_path}")
        return True

    def publish_markers(self) -> None:
        """发布LINE_STRIP和SPHERE_LIST，二者都使用trajectory_command.frame_id。"""
        if not self.reload_if_changed() or self.trajectory is None:
            return
        frame_id = self.trajectory["frame_id"]
        waypoints = self.trajectory["waypoints_base"]
        stamp = self.get_clock().now().to_msg()

        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = stamp
        line.ns = "bucket_tip_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.03
        line.color = make_color(1.0, 0.1, 0.1, 1.0)
        line.points = [make_point(point) for point in waypoints]

        spheres = Marker()
        spheres.header.frame_id = frame_id
        spheres.header.stamp = stamp
        spheres.ns = "bucket_tip_waypoints"
        spheres.id = 1
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD
        # 关键：点的半径只服务RViz观察，不代表碰撞半径。
        spheres.scale.x = 0.08
        spheres.scale.y = 0.08
        spheres.scale.z = 0.08
        spheres.color = make_color(1.0, 0.9, 0.0, 1.0)
        spheres.points = [make_point(point) for point in waypoints]

        self.publisher.publish(MarkerArray(markers=[line, spheres]))


def main() -> int:
    """入口函数：持续发布trajectory marker直到Ctrl+C。"""
    args = build_arg_parser().parse_args()
    rclpy.init()
    node = TrajectoryMarkerPublisher(args.trajectory, args.topic, args.rate_hz)
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
