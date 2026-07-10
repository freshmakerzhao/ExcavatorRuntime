import argparse
import math
from typing import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


DEFAULT_JOINT_NAMES = ("swing_joint", "boom_joint", "arm_joint", "bucket_joint")
DEFAULT_JOINT_LABELS = ("swing", "boom", "arm", "bucket")
DEFAULT_INITIAL_ANGLES = (0.0, 0.0, 0.0, 0.0)
DEFAULT_LIMITS = (
    (-math.pi, math.pi),
    (-1.6, 1.6),
    (-1.6, 1.6),
    (-1.6, 1.6),
)


def build_joint_state(positions_rad: Sequence[float], stamp=None) -> JointState:
    """构造运动学节点订阅的JointState消息；角度单位固定为rad。"""
    if len(positions_rad) != len(DEFAULT_JOINT_NAMES):
        raise ValueError(f"positions_rad必须包含{len(DEFAULT_JOINT_NAMES)}个角度")
    message = JointState()
    if stamp is not None:
        message.header.stamp = stamp
    message.name = list(DEFAULT_JOINT_NAMES)
    message.position = [float(value) for value in positions_rad]
    return message


def format_angle(value_rad: float) -> str:
    """把滑块值格式化成rad/deg，方便人工核对。"""
    return f"{value_rad:.3f} rad / {math.degrees(value_rad):.1f} deg"


class JointSliderWindow:
    """Tk窗口：通过四个滑块发布/joint_states。"""

    def __init__(self, node: Node, args: argparse.Namespace) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.node = node
        self.publisher = node.create_publisher(JointState, args.topic, 10)
        self.topic = args.topic
        self.publish_on_change = args.publish_on_change
        self.periodic_publish = not args.no_periodic_publish
        self.period_ms = max(int(1000.0 / max(args.rate_hz, 0.1)), 20)
        self.initial_angles = tuple(args.initial)

        self.root = tk.Tk()
        self.root.title(args.title)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.values: list[tk.DoubleVar] = []
        self.value_labels: list[ttk.Label] = []
        self._build_layout(ttk, args)
        self.publish_once()
        self.root.after(self.period_ms, self._periodic_publish)
        self.root.after(20, self._pump_ros)

    def _build_layout(self, ttk, args: argparse.Namespace) -> None:
        """创建滑块、数值标签和按钮。"""
        root = self.root
        for row, (label, initial, limits) in enumerate(zip(DEFAULT_JOINT_LABELS, args.initial, args.limits)):
            ttk.Label(root, text=label, width=8).grid(row=row, column=0, padx=10, pady=8, sticky="w")
            value = self.tk.DoubleVar(value=float(initial))
            self.values.append(value)
            slider = ttk.Scale(
                root,
                from_=float(limits[0]),
                to=float(limits[1]),
                orient="horizontal",
                length=360,
                variable=value,
                command=lambda _unused, index=row: self._on_slider_changed(index),
            )
            slider.grid(row=row, column=1, padx=10, pady=8, sticky="ew")
            value_label = ttk.Label(root, text=format_angle(float(initial)), width=22)
            value_label.grid(row=row, column=2, padx=10, pady=8, sticky="w")
            self.value_labels.append(value_label)

        button_frame = ttk.Frame(root)
        button_frame.grid(row=len(DEFAULT_JOINT_LABELS), column=0, columnspan=3, pady=10)
        ttk.Button(button_frame, text="Publish", command=self.publish_once).grid(row=0, column=0, padx=6)
        ttk.Button(button_frame, text="Reset", command=self.reset).grid(row=0, column=1, padx=6)
        root.columnconfigure(1, weight=1)

    def _on_slider_changed(self, index: int) -> None:
        """响应滑块变化：刷新文字，并按需立即发布。"""
        self.value_labels[index].configure(text=format_angle(self.values[index].get()))
        if self.publish_on_change:
            self.publish_once()

    def current_positions(self) -> tuple[float, ...]:
        """读取当前四个滑块角度，单位rad。"""
        return tuple(float(value.get()) for value in self.values)

    def publish_once(self) -> None:
        """发布一次JointState；运动学节点收到后会刷新TF和bucket tip。"""
        stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(build_joint_state(self.current_positions(), stamp=stamp))

    def reset(self) -> None:
        """把滑块恢复到启动时的初始角度。"""
        for value, initial in zip(self.values, self.initial_angles):
            value.set(float(initial))
        for index in range(len(self.values)):
            self._on_slider_changed(index)
        self.publish_once()

    def _periodic_publish(self) -> None:
        """周期发布，保证RViz/TF在不拖动滑块时也能持续收到状态。"""
        if self.periodic_publish:
            self.publish_once()
        self.root.after(self.period_ms, self._periodic_publish)

    def _pump_ros(self) -> None:
        """让ROS2有机会处理内部事件，同时不阻塞Tk主循环。"""
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(20, self._pump_ros)

    def close(self) -> None:
        """关闭窗口。"""
        self.root.destroy()

    def run(self) -> None:
        """启动Tk主循环。"""
        self.root.mainloop()


def build_arg_parser() -> argparse.ArgumentParser:
    """构造命令行参数。"""
    parser = argparse.ArgumentParser(description="用GUI滑块发布挖掘机四个关节角到/joint_states。")
    parser.add_argument("--topic", default="/joint_states", help="JointState发布topic")
    parser.add_argument("--title", default="excavator joint sliders", help="窗口标题")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="周期发布频率")
    parser.add_argument("--publish-on-change", action="store_true", help="拖动滑块时立即发布")
    parser.add_argument("--no-periodic-publish", action="store_true", help="关闭周期发布，只在拖动/按钮时发布")
    parser.add_argument("--initial", nargs=4, type=float, default=list(DEFAULT_INITIAL_ANGLES), metavar=("SWING", "BOOM", "ARM", "BUCKET"), help="四个初始角度，单位rad")
    parser.add_argument("--min", nargs=4, type=float, default=[limit[0] for limit in DEFAULT_LIMITS], metavar=("SWING", "BOOM", "ARM", "BUCKET"), help="四个滑块最小角，单位rad")
    parser.add_argument("--max", nargs=4, type=float, default=[limit[1] for limit in DEFAULT_LIMITS], metavar=("SWING", "BOOM", "ARM", "BUCKET"), help="四个滑块最大角，单位rad")
    return parser


def main() -> int:
    """入口：创建ROS2节点和Tk滑块窗口。"""
    args = build_arg_parser().parse_args()
    args.limits = tuple(zip(args.min, args.max))
    rclpy.init()
    node = rclpy.create_node("excavator_joint_slider_publisher")
    try:
        window = JointSliderWindow(node, args)
        window.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
