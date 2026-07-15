#!/usr/bin/python3.12
import math
import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [
    ("swing_joint", -1.57, 1.57, 0.0),
    ("boom_joint", -3.14, 3.14, 0.0),
    ("arm_joint", -3.14, 3.14, 0.0),
    ("bucket_joint", -3.14, 3.14, 0.0),
]


def build_joint_state(positions: list[float]) -> JointState:
    """Build the URDF joint-state contract used by the verification slider."""
    if len(positions) != len(JOINTS):
        raise ValueError(f"expected {len(JOINTS)} joint positions, got {len(positions)}")

    message = JointState()
    message.name = [name for name, _lower, _upper, _default in JOINTS]
    message.position = [float(position) for position in positions]
    return message


class WajiJointSlider:
    def __init__(self):
        rclpy.init()
        self.node = Node("waji_joint_slider")
        self.publisher = self.node.create_publisher(JointState, "joint_states", 10)

        self.root = tk.Tk()
        self.root.title("waji joint sliders")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.vars = {}
        self.value_labels = {}
        for row, (name, lower, upper, default) in enumerate(JOINTS):
            initial = float(self.node.declare_parameter(name, default).value)
            initial = self._clamp(initial, lower, upper)

            tk.Label(self.root, text=name, width=8, anchor="w").grid(
                row=row, column=0, padx=8, pady=6
            )
            var = tk.DoubleVar(value=initial)
            scale = tk.Scale(
                self.root,
                from_=lower,
                to=upper,
                resolution=0.01,
                orient=tk.HORIZONTAL,
                length=360,
                variable=var,
                command=lambda _value, joint=name: self.on_slider(joint),
            )
            scale.grid(row=row, column=1, padx=8, pady=6)
            label = tk.Label(self.root, width=20, anchor="w")
            label.grid(row=row, column=2, padx=8, pady=6)

            self.vars[name] = var
            self.value_labels[name] = label
            self.update_label(name)

        button_frame = tk.Frame(self.root)
        button_frame.grid(row=len(JOINTS), column=0, columnspan=3, pady=8)
        tk.Button(button_frame, text="Publish", command=self.publish).pack(side=tk.LEFT, padx=6)
        tk.Button(button_frame, text="Reset", command=self.reset).pack(side=tk.LEFT, padx=6)

        self.closed = False
        self.publish()
        self.root.after(100, self.publish_loop)

    def on_slider(self, joint: str):
        self.update_label(joint)
        self.publish()

    def update_label(self, joint: str):
        value = self.vars[joint].get()
        self.value_labels[joint].config(
            text=f"{value:.3f} rad / {math.degrees(value):.1f} deg"
        )

    def publish_loop(self):
        if self.closed:
            return
        self.publish()
        self.root.after(100, self.publish_loop)

    def publish(self):
        msg = build_joint_state([self.vars[name].get() for name, _lower, _upper, _default in JOINTS])
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def reset(self):
        for name, _lower, _upper, default in JOINTS:
            self.vars[name].set(default)
            self.update_label(name)
        self.publish()

    def close(self):
        self.closed = True
        try:
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.root.destroy()
        except (KeyboardInterrupt, tk.TclError):
            pass

    def run(self):
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            if not self.closed:
                self.close()

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def main():
    WajiJointSlider().run()


if __name__ == "__main__":
    main()
