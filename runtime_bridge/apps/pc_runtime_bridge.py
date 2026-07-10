#!/usr/bin/env python3
"""PC 侧 runtime bridge：接收 Orin 状态，回发动作，并可选发布 /joint_states。"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from runtime_bridge.protocol import (
    ExcavatorStatePacket,
    MachineStatePacket,
    PacketDecodeError,
    decode_packet,
    encode_packet,
    make_zero_action,
)


DEFAULT_LATEST_STATE = PROJECT_ROOT / "runtime_bridge" / "exports" / "latest_state.json"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造 PC runtime bridge 参数。"""
    parser = argparse.ArgumentParser(description="接收Orin状态包并回发PC动作包。")
    parser.add_argument("--state-bind-host", default="0.0.0.0", help="监听Orin状态的本地地址")
    parser.add_argument("--state-port", type=int, default=18081, help="Orin -> PC 状态UDP端口")
    parser.add_argument("--orin-host", default="127.0.0.1", help="动作包发送到的Orin地址")
    parser.add_argument("--action-port", type=int, default=18082, help="PC -> Orin 动作UDP端口")
    parser.add_argument("--send-zero-action", action="store_true", help="每收到状态后回发四维零动作；仅用于链路联调")
    parser.add_argument("--action-valid-ms", type=int, default=100, help="动作有效期，Orin侧应据此做超时保护")
    parser.add_argument("--latest-state-json", type=Path, default=DEFAULT_LATEST_STATE, help="写出最近一次状态包")
    parser.add_argument("--write-every", type=int, default=10, help="每收到多少个状态写一次JSON")
    parser.add_argument("--print-every", type=int, default=20, help="每收到多少个状态打印一次；0表示不打印")
    parser.add_argument("--publish-joint-states", action="store_true", help="把状态包关节角发布为ROS2 /joint_states")
    return parser


class JointStatePublisher:
    """可选 ROS2 /joint_states 发布器；未启用时不导入ROS模块。"""

    def __init__(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
        except ModuleNotFoundError as exc:
            raise SystemExit(
                "无法导入ROS2 Python模块。请先source ROS环境，并使用/usr/bin/python3运行。\n"
                f"原始错误: {exc}"
            ) from exc

        self.rclpy = rclpy
        self.JointState = JointState
        rclpy.init(args=None)
        self.node = Node("pc_runtime_joint_state_bridge")
        self.publisher = self.node.create_publisher(JointState, "/joint_states", 10)

    def publish(self, state: ExcavatorStatePacket | MachineStatePacket) -> None:
        """发布 ROS2 JointState，供 excavator_kinematics 计算 bucket tip。"""
        message = self.JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
        # 关键：协议里是短名，ROS JointState 里是运动学包要求的 joint name。
        message.position = [
            state.joint_position_rad["swing"],
            state.joint_position_rad["boom"],
            state.joint_position_rad["arm"],
            state.joint_position_rad["bucket"],
        ]
        message.velocity = [
            state.joint_velocity_rad_s["swing"],
            state.joint_velocity_rad_s["boom"],
            state.joint_velocity_rad_s["arm"],
            state.joint_velocity_rad_s["bucket"],
        ]
        self.publisher.publish(message)
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self) -> None:
        """关闭 ROS2 节点。"""
        self.node.destroy_node()
        self.rclpy.shutdown()


def write_latest_state(path: Path, state: ExcavatorStatePacket | MachineStatePacket) -> None:
    """写出最近状态，方便 smoke check 或人工排查。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(state.to_dict(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    """入口：接收状态包，按配置写JSON、发JointState和回传动作。"""
    args = build_arg_parser().parse_args()
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind((args.state_bind_host, args.state_port))
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    action_destination = (args.orin_host, args.action_port)
    joint_state_publisher = JointStatePublisher() if args.publish_joint_states else None

    state_count = 0
    action_seq = 0
    print(
        "pc runtime bridge started: "
        f"state <- {args.state_bind_host}:{args.state_port}, action -> {action_destination}, "
        f"zero_action={args.send_zero_action}, publish_joint_states={args.publish_joint_states}",
        flush=True,
    )

    try:
        while True:
            payload, address = recv_sock.recvfrom(4096)
            try:
                packet = decode_packet(payload)
            except PacketDecodeError as exc:
                print(f"drop invalid packet from {address}: {exc}", flush=True)
                continue
            if not isinstance(packet, ExcavatorStatePacket | MachineStatePacket):
                continue

            state_count += 1
            if args.write_every > 0 and state_count % args.write_every == 0:
                write_latest_state(args.latest_state_json, packet)
            if joint_state_publisher is not None:
                joint_state_publisher.publish(packet)
            if args.send_zero_action:
                # 关键：零动作只用于链路联调，不代表最终 ONNX 输出。
                action = make_zero_action(action_seq, valid_for_ms=args.action_valid_ms)
                send_sock.sendto(encode_packet(action), action_destination)
                action_seq += 1

            if args.print_every > 0 and state_count % args.print_every == 0:
                age_ms = int(time.time() * 1000) - packet.stamp_ms
                if isinstance(packet, MachineStatePacket):
                    # 关键：正式协议下把安全状态也打出来，方便联调时一眼看出为何不执行动作。
                    safety = packet.safety
                    print(
                        f"state[{state_count}] from {address}: seq={packet.seq}, age={age_ms}ms, "
                        f"estop={safety['estop']}, sensor_valid={safety['sensor_valid']}, "
                        f"control_enabled={safety['control_enabled']}, faults={safety['fault_flags']}",
                        flush=True,
                    )
                else:
                    print(
                        f"state[{state_count}] from {address}: seq={packet.seq}, age={age_ms}ms, estop={packet.estop}",
                        flush=True,
                    )
    except KeyboardInterrupt:
        print("pc runtime bridge stopped", flush=True)
    finally:
        if joint_state_publisher is not None:
            joint_state_publisher.close()
        recv_sock.close()
        send_sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
