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
from runtime_bridge.action_journal import ActionJournalUnavailable, RecordedUdpSender
from runtime_bridge.runtime_config import DEFAULT_RUNTIME_CONFIG, load_runtime_config


DEFAULT_LATEST_STATE = PROJECT_ROOT / "runtime_bridge" / "exports" / "latest_state.json"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造通信诊断入口参数。"""
    parser = argparse.ArgumentParser(description="接收Orin状态包，可选回发零动作进行链路诊断。")
    parser.add_argument("--config", type=Path, default=DEFAULT_RUNTIME_CONFIG, help="运行配置JSON")
    parser.add_argument("--reply-zero", action="store_true", help="每收到状态后回发四维零动作")
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
    """诊断入口：接收状态包，按需写JSON、发JointState和回传零动作。"""
    args = build_arg_parser().parse_args()
    try:
        config = load_runtime_config(args.config)
    except (OSError, ValueError) as exc:
        print(f"runtime diagnostic configuration error: {exc}", file=sys.stderr, flush=True)
        return 2

    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(config.network.state_endpoint)
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    action_destination = config.network.action_endpoint
    try:
        action_sender = (
            RecordedUdpSender(
                send_sock,
                action_destination,
                journal_config=config.action_journal,
                source="pc_runtime_bridge",
            )
            if args.reply_zero
            else None
        )
    except OSError as exc:
        print(f"runtime action journal startup failed: {exc}", file=sys.stderr, flush=True)
        recv_sock.close()
        send_sock.close()
        return 2
    joint_state_publisher = JointStatePublisher() if args.publish_joint_states else None

    state_count = 0
    action_seq = 0
    exit_code = 0
    print(
        "pc runtime diagnostic started: "
        f"state <- {config.network.state_endpoint}, action -> {action_destination}, "
        f"reply_zero={args.reply_zero}, publish_joint_states={args.publish_joint_states}, "
        f"action_journal={action_sender.journal_path if action_sender else 'disabled'}",
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
            if config.diagnostics.write_every > 0 and state_count % config.diagnostics.write_every == 0:
                write_latest_state(DEFAULT_LATEST_STATE, packet)
            if joint_state_publisher is not None:
                joint_state_publisher.publish(packet)
            if args.reply_zero:
                # 关键：零动作只用于链路联调，不代表最终 ONNX 输出。
                action = make_zero_action(action_seq, valid_for_ms=config.network.action_valid_ms)
                action_sender.send(encode_packet(action))
                action_seq += 1

            if config.diagnostics.print_every > 0 and state_count % config.diagnostics.print_every == 0:
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
    except ActionJournalUnavailable as exc:
        print(f"pc runtime bridge stopped: {exc}", file=sys.stderr, flush=True)
        exit_code = 3
    finally:
        if joint_state_publisher is not None:
            joint_state_publisher.close()
        if action_sender is not None:
            try:
                action_sender.close()
            except ActionJournalUnavailable as exc:
                print(f"pc runtime journal close failed: {exc}", file=sys.stderr, flush=True)
                exit_code = 3
        recv_sock.close()
        send_sock.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
