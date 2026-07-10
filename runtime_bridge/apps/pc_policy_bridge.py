#!/usr/bin/env python3
"""PC 侧闭环 policy bridge：Orin 状态 + FK/规划观测 -> ONNX 动作 -> Orin。"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path
from typing import Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from runtime_bridge.fixed_actions import physical_velocity_action_from_normalized
from runtime_bridge.action_journal import ActionJournalUnavailable, RecordedUdpSender
from runtime_bridge.observation import (
    BucketTipObservation,
    ObservationBuilder,
    load_machine_profile,
    load_waypoint_slice_values,
)
from runtime_bridge.onnx_policy import OnnxPolicy, OnnxPolicyLoadError
from runtime_bridge.protocol import (
    ACTION_ORDER,
    MachineStatePacket,
    PacketDecodeError,
    PolicyActionPacket,
    decode_packet,
    encode_packet,
    estimate_remote_now_ms,
    make_zero_action,
    now_ms,
)
from runtime_bridge.runtime_config import DEFAULT_RUNTIME_CONFIG, load_runtime_config


def build_arg_parser() -> argparse.ArgumentParser:
    """构造 policy bridge 参数。"""
    parser = argparse.ArgumentParser(description="加载ONNX策略，接收Orin状态并回传4维动作。")
    parser.add_argument("--config", type=Path, default=DEFAULT_RUNTIME_CONFIG, help="运行配置JSON")
    parser.add_argument("--task-mode", default="MoveToDig", choices=("MoveToDig", "CarryMaterial"), help="任务模式")
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help="显式允许向Orin发送UDP动作；默认只推理和打印",
    )
    return parser


class RuntimeRosIo:
    """ROS2 I/O：发布 /joint_states 给 FK，并订阅 /bucket_tip_observation。"""

    def __init__(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Float32MultiArray
        except ModuleNotFoundError as exc:
            raise SystemExit(
                "无法导入ROS2 Python模块。请先 source /opt/ros/jazzy/setup.zsh 和 ros2_ws/install/setup.zsh。"
            ) from exc

        self.rclpy = rclpy
        self.JointState = JointState
        self.latest_bucket_tip: BucketTipObservation | None = None
        rclpy.init(args=None)
        self.node = Node("pc_policy_bridge")
        self.joint_publisher = self.node.create_publisher(JointState, "/joint_states", 10)
        self.node.create_subscription(Float32MultiArray, "/bucket_tip_observation", self._on_bucket_tip, 10)

    def _on_bucket_tip(self, message) -> None:
        """接收 FK 节点发布的 [x, y, z, pitch_rad]。"""
        values = list(message.data)
        if len(values) < 4:
            return
        stamp_ms = int(self.node.get_clock().now().nanoseconds / 1_000_000)
        self.latest_bucket_tip = BucketTipObservation(
            position_m=(float(values[0]), float(values[1]), float(values[2])),
            pitch_rad=float(values[3]),
            stamp_ms=stamp_ms,
        )

    def publish_joint_states(self, state: MachineStatePacket) -> None:
        """把 Orin 关节角发布给 FK 节点，驱动 bucket tip 更新。"""
        message = self.JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
        # 关键：协议短名转换为运动学包使用的 joint name。
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
        self.joint_publisher.publish(message)

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        """处理一次 ROS 回调。"""
        self.rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def close(self) -> None:
        """关闭 ROS2 节点。"""
        self.node.destroy_node()
        self.rclpy.shutdown()


def should_send_policy(state: MachineStatePacket) -> tuple[bool, str]:
    """检查安全门；不满足时仍可推理，但实际发送零动作。"""
    safety = state.safety
    if safety["estop"]:
        return False, "estop"
    if not safety["stm32_alive"]:
        return False, "stm32_not_alive"
    if not safety["sensor_valid"]:
        return False, "sensor_invalid"
    if not safety["control_enabled"]:
        return False, "control_disabled"
    if safety["fault_flags"]:
        return False, "fault_flags"
    return True, "policy"


def make_policy_action(
    seq: int,
    action: Sequence[float],
    valid_for_ms: int,
    action_type: str,
    stamp_ms: int | None = None,
) -> PolicyActionPacket:
    """构造发给 Orin 的4维动作包；动作单位由 action_type 明确。"""
    return PolicyActionPacket(
        seq=seq,
        stamp_ms=now_ms() if stamp_ms is None else int(stamp_ms),
        action=[float(value) for value in action],
        action_type=action_type,
        valid_for_ms=valid_for_ms,
        action_order=ACTION_ORDER,
    )


def denormalize_policy_action(action: Sequence[float], machine_profile: dict) -> list[float]:
    """把ONNX的[-1,1]策略动作转换为执行器物理速度命令。

    输出顺序仍是 boom, stick, bucket, swing；前三个单位 m/s，swing 单位 rad/s。
    """
    return physical_velocity_action_from_normalized(action, machine_profile)


def write_latest_observation(path: Path, observation: list[float], action: list[float], sent_action: PolicyActionPacket) -> None:
    """写出最近一次 observation/action，方便人工核查 ONNX 输入输出。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "stamp_ms": now_ms(),
        "observation_schema": "scale_excavator_v2_38d",
        "observation": observation,
        "policy_action": action,
        "sent_action": sent_action.to_dict(),
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    """主循环：状态包到来时完成 observation -> ONNX -> action。"""
    args = build_arg_parser().parse_args()
    try:
        config = load_runtime_config(args.config)
        config.artifacts.require_policy_inputs()
        machine_profile = load_machine_profile(config.artifacts.machine_profile)
        observation_builder = ObservationBuilder(machine_profile, task_mode=args.task_mode)
        policy = OnnxPolicy(config.artifacts.onnx)
    except (OSError, ValueError, OnnxPolicyLoadError) as exc:
        print(f"policy bridge configuration error: {exc}", file=sys.stderr, flush=True)
        return 2

    ros_io = RuntimeRosIo()

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
                source="pc_policy_bridge",
            )
            if args.enable_motion
            else None
        )
    except OSError as exc:
        print(f"policy action journal startup failed: {exc}", file=sys.stderr, flush=True)
        ros_io.close()
        recv_sock.close()
        send_sock.close()
        return 2
    previous_policy_action = [0.0, 0.0, 0.0, 0.0]
    state_count = 0
    action_seq = 0
    exit_code = 0

    print(
        "pc policy bridge started: "
        f"state <- {config.network.state_endpoint}, action -> {action_destination}, "
        f"onnx={config.artifacts.onnx}, enable_motion={args.enable_motion}, task_mode={args.task_mode}, "
        f"action_journal={action_sender.journal_path if action_sender else 'disabled'}",
        flush=True,
    )

    try:
        while True:
            payload, address = recv_sock.recvfrom(4096)
            received_pc_ms = now_ms()
            try:
                packet = decode_packet(payload)
            except PacketDecodeError as exc:
                print(f"drop invalid packet from {address}: {exc}", flush=True)
                continue
            if not isinstance(packet, MachineStatePacket):
                continue

            state_count += 1
            ros_io.publish_joint_states(packet)
            ros_io.spin_once(timeout_sec=0.0)
            ros_io.spin_once(timeout_sec=0.01)
            bucket_tip = ros_io.latest_bucket_tip
            if bucket_tip is None:
                if config.diagnostics.print_every > 0 and state_count % config.diagnostics.print_every == 0:
                    print("waiting for /bucket_tip_observation; is excavator_tf_node running?", flush=True)
                continue

            age_ms = int(time.time() * 1000) - int(bucket_tip.stamp_ms or 0)
            if age_ms > config.policy.bucket_tip_timeout_ms:
                print(f"skip stale bucket tip: age={age_ms}ms", flush=True)
                continue

            waypoint_values = load_waypoint_slice_values(config.artifacts.waypoint_slice)
            observation = observation_builder.build(
                packet,
                bucket_tip,
                waypoint_values,
                previous_action=previous_policy_action,
                episode_progress=0.0,
            )
            policy_action = policy.run(observation)
            actuator_velocity_action = denormalize_policy_action(policy_action, machine_profile)
            send_policy, reason = should_send_policy(packet)
            action_stamp_ms = (
                estimate_remote_now_ms(packet.stamp_ms, received_pc_ms)
                if config.network.action_time_source == "orin"
                else now_ms()
            )
            sent_packet = (
                make_policy_action(
                    action_seq,
                    actuator_velocity_action,
                    config.network.action_valid_ms,
                    "normalized_velocity_command",
                    stamp_ms=action_stamp_ms,
                )
                if send_policy
                else make_zero_action(
                    action_seq,
                    valid_for_ms=config.network.action_valid_ms,
                    stamp_ms=action_stamp_ms,
                )
            )
            if not send_policy:
                sent_packet = make_policy_action(
                    action_seq,
                    sent_packet.action,
                    config.network.action_valid_ms,
                    "normalized_velocity_command",
                    stamp_ms=action_stamp_ms,
                )

            if args.enable_motion:
                action_sender.send(encode_packet(sent_packet))
            previous_policy_action = list(policy_action) if send_policy else [0.0, 0.0, 0.0, 0.0]
            action_seq += 1

            if config.diagnostics.write_every > 0 and state_count % config.diagnostics.write_every == 0:
                write_latest_observation(
                    config.artifacts.latest_observation,
                    observation,
                    policy_action,
                    sent_packet,
                )
            if config.diagnostics.print_every > 0 and state_count % config.diagnostics.print_every == 0:
                print(
                    f"state[{state_count}] seq={packet.seq} obs[9:12]={observation[9:12]} "
                    f"pitch={observation[34]:.3f} policy={policy_action} sent={sent_packet.action} "
                    f"action_stamp={sent_packet.stamp_ms} state_stamp={packet.stamp_ms} reason={reason}",
                    flush=True,
                )
    except KeyboardInterrupt:
        print("pc policy bridge stopped", flush=True)
    except ActionJournalUnavailable as exc:
        print(f"pc policy bridge stopped: {exc}", file=sys.stderr, flush=True)
        exit_code = 3
    finally:
        ros_io.close()
        if action_sender is not None:
            try:
                action_sender.close()
            except ActionJournalUnavailable as exc:
                print(f"pc policy bridge journal close failed: {exc}", file=sys.stderr, flush=True)
                exit_code = 3
        recv_sock.close()
        send_sock.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
