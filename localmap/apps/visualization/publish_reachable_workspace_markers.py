#!/usr/bin/env python3
"""发布Unity导出的bucket tip可达区域到RViz。"""

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
PROJECT_ROOT = LOCALMAP_DIR.parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.reachable_workspace import ReachableWorkspace, load_reachable_workspace


DEFAULT_WORKSPACE = PROJECT_ROOT / "shared" / "reachable_workspaces" / "scale_excavator_workspace.json"

SECTION_EDGES = ((0, 1), (0, 2), (1, 3), (2, 3))
LONGITUDINAL_EDGES = ((0, 0), (1, 1), (2, 2), (3, 3))
CELL_FACE_TRIANGLES = (
    (0, 1, 5),
    (0, 5, 4),
    (2, 6, 7),
    (2, 7, 3),
    (0, 4, 6),
    (0, 6, 2),
    (1, 3, 7),
    (1, 7, 5),
    (0, 2, 3),
    (0, 3, 1),
    (4, 5, 7),
    (4, 7, 6),
)


def build_arg_parser() -> argparse.ArgumentParser:
    """构造可达区域可视化参数。"""
    parser = argparse.ArgumentParser(description="发布bucket tip reachable workspace到RViz。")
    parser.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE, help="shared reachable workspace JSON")
    parser.add_argument("--mode", choices=["MoveToDig", "CarryMaterial"], default="MoveToDig", help="显示的任务模式")
    parser.add_argument("--topic", default="/localmap/reachable_workspace_markers", help="MarkerArray输出topic")
    parser.add_argument("--rate-hz", type=float, default=1.0, help="重复发布频率，便于RViz后打开也能看到")
    return parser


def make_color(red: float, green: float, blue: float, alpha: float = 1.0) -> ColorRGBA:
    """创建RViz颜色。"""
    color = ColorRGBA()
    color.r = float(red)
    color.g = float(green)
    color.b = float(blue)
    color.a = float(alpha)
    return color


def make_point(values) -> Point:
    """把numpy/list三维点转换为ROS Point。"""
    point = Point()
    point.x = float(values[0])
    point.y = float(values[1])
    point.z = float(values[2])
    return point


def make_marker_base(frame_id: str, stamp, marker_id: int, namespace: str, marker_type: int) -> Marker:
    """创建带公共header和pose的marker，减少重复字段。"""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def build_surface_points(workspace: ReachableWorkspace) -> list[Point]:
    """把相邻截面的表面三角片展开为TRIANGLE_LIST点序列。"""
    points: list[Point] = []
    for index in range(workspace.sections.shape[0] - 1):
        cell = [*workspace.sections[index], *workspace.sections[index + 1]]
        for triangle in CELL_FACE_TRIANGLES:
            # 关键：这些三角片只服务RViz显示，不参与RRT判断；RRT使用四面体inside检查。
            points.extend(make_point(cell[vertex]) for vertex in triangle)
    return points


def build_edge_points(workspace: ReachableWorkspace) -> list[Point]:
    """构造截面边线和截面之间的连线，帮助肉眼看清20点可达体。"""
    points: list[Point] = []
    for section in workspace.sections:
        for start_index, end_index in SECTION_EDGES:
            points.append(make_point(section[start_index]))
            points.append(make_point(section[end_index]))
    for index in range(workspace.sections.shape[0] - 1):
        current = workspace.sections[index]
        following = workspace.sections[index + 1]
        for start_index, end_index in LONGITUDINAL_EDGES:
            points.append(make_point(current[start_index]))
            points.append(make_point(following[end_index]))
    return points


class ReachableWorkspaceMarkerPublisher(Node):
    """周期发布bucket tip可达区域marker。"""

    def __init__(self, workspace_path: Path, mode: str, topic: str, rate_hz: float) -> None:
        super().__init__("airy_reachable_workspace_marker_publisher")
        self.workspace_path = workspace_path
        self.mode = mode
        self.workspace = load_reachable_workspace(workspace_path, mode=mode)
        self.publisher = self.create_publisher(MarkerArray, topic, 1)
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self.publish_markers)
        self.get_logger().info(f"publishing reachable workspace markers: {workspace_path} mode={mode} -> {topic}")

    def publish_markers(self) -> None:
        """发布半透明曲面、边线和anchor点。"""
        stamp = self.get_clock().now().to_msg()
        frame_id = self.workspace.frame_id

        surface = make_marker_base(frame_id, stamp, 0, "reachable_workspace_surface", Marker.TRIANGLE_LIST)
        surface.scale.x = 1.0
        surface.scale.y = 1.0
        surface.scale.z = 1.0
        surface.color = make_color(0.1, 0.7, 1.0, 0.18)
        surface.points = build_surface_points(self.workspace)

        edges = make_marker_base(frame_id, stamp, 1, "reachable_workspace_edges", Marker.LINE_LIST)
        edges.scale.x = 0.015
        edges.color = make_color(0.0, 0.95, 1.0, 0.95)
        edges.points = build_edge_points(self.workspace)

        anchors = make_marker_base(frame_id, stamp, 2, "reachable_workspace_anchors", Marker.SPHERE_LIST)
        anchors.scale.x = 0.04
        anchors.scale.y = 0.04
        anchors.scale.z = 0.04
        anchors.color = make_color(1.0, 0.85, 0.0, 1.0)
        anchors.points = [make_point(point) for point in self.workspace.anchor_points()]

        self.publisher.publish(MarkerArray(markers=[surface, edges, anchors]))


def main() -> int:
    """入口函数：持续发布reachable workspace marker直到Ctrl+C。"""
    args = build_arg_parser().parse_args()
    rclpy.init()
    node = ReachableWorkspaceMarkerPublisher(args.workspace, args.mode, args.topic, args.rate_hz)
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
