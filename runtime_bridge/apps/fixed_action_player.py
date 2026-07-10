#!/usr/bin/env python3
"""PC侧固定动作播放器：接收 Orin 状态，发送固定挖掘/倾倒动作。

该脚本不做路径规划，也不替代 ONNX 轨迹跟踪。典型使用方式是：
外部 planner/pc_policy_bridge 先把 bucket tip 送到挖掘点或倾倒点，然后本脚本执行固定动作段。
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from dataclasses import replace
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from runtime_bridge.fixed_actions import FixedActionExecutor, FixedActionStatus, fixed_action_sequence
from runtime_bridge.observation import load_machine_profile
from runtime_bridge.protocol import (
    MachineStatePacket,
    PacketDecodeError,
    decode_packet,
    encode_packet,
    estimate_remote_now_ms,
    make_zero_action,
    now_ms,
)
from runtime_bridge.runtime_config import DEFAULT_RUNTIME_CONFIG, load_runtime_config


def build_arg_parser() -> argparse.ArgumentParser:
    """构造固定动作播放器参数。"""
    parser = argparse.ArgumentParser(description="通过UDP向Orin发送固定挖掘/倾倒动作。")
    parser.add_argument("action", choices=("dig", "dump"), help="固定动作类型：dig=挖掘，dump=倾倒")
    parser.add_argument("--config", type=Path, default=DEFAULT_RUNTIME_CONFIG, help="运行配置JSON")
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help="显式允许向Orin发送UDP动作；默认只计算和打印",
    )
    return parser


def can_send_motion(state: MachineStatePacket) -> tuple[bool, str]:
    """检查安全门；不满足时只发送零动作。"""
    safety = state.safety
    if safety["estop"]:
        return False, "estop"
    if not safety["stm32_alive"]:
        return False, "stm32_not_alive"
    if not safety["sensor_valid"]:
        return False, "sensor_invalid"
    if safety["fault_flags"]:
        return False, "fault_flags"
    if not safety["control_enabled"]:
        return False, "control_disabled"
    return True, "fixed_action"


def main() -> int:
    """主循环：收到状态后推进固定动作一步，并回发动作包。"""
    args = build_arg_parser().parse_args()
    try:
        config = load_runtime_config(args.config)
        config.artifacts.require_machine_profile()
        machine_profile = load_machine_profile(config.artifacts.machine_profile)
    except (OSError, ValueError) as exc:
        print(f"fixed action configuration error: {exc}", file=sys.stderr, flush=True)
        return 2

    tuning = config.fixed_action
    executor = FixedActionExecutor(
        fixed_action_sequence(args.action),
        machine_profile,
        kp=tuning.kp,
        min_action=tuning.min_action,
        max_action=tuning.max_action,
        tolerance=tuning.tolerance,
        step_timeout_s=tuning.step_timeout_s,
        hold_s=tuning.hold_s,
    )

    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(config.network.state_endpoint)
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = config.network.action_endpoint
    action_seq = 0
    state_count = 0
    started_at_s = time.monotonic()

    print(
        f"fixed action player started: action={args.action}, "
        f"state <- {config.network.state_endpoint}, action -> {destination}, "
        f"enable_motion={args.enable_motion}",
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
            can_send, reason = can_send_motion(packet)
            action_stamp_ms = (
                estimate_remote_now_ms(packet.stamp_ms, received_pc_ms)
                if config.network.action_time_source == "orin"
                else now_ms()
            )
            if can_send:
                # 关键：固定动作按收到的状态闭环推进；发送字段保持Orin兼容格式。
                action_packet, status = executor.step(
                    packet,
                    now_s=time.monotonic() - started_at_s,
                    seq=action_seq,
                    valid_for_ms=config.network.action_valid_ms,
                )
                action_packet = replace(action_packet, stamp_ms=action_stamp_ms)
            else:
                action_packet = make_zero_action(
                    action_seq,
                    config.network.action_valid_ms,
                    stamp_ms=action_stamp_ms,
                )
                status = FixedActionStatus(f"safety_zero:{reason}", executor.step_index, "安全零动作", 0.0, executor.done)

            if args.enable_motion:
                send_sock.sendto(encode_packet(action_packet), destination)

            action_seq += 1
            if config.diagnostics.print_every > 0 and state_count % config.diagnostics.print_every == 0:
                print(
                    f"state[{state_count}] seq={packet.seq} step={status.step_index}:{status.step_label} "
                    f"phase={status.phase} err={status.max_error:.3f} sent={action_packet.action} "
                    f"action_stamp={action_packet.stamp_ms} state_stamp={packet.stamp_ms} reason={reason}",
                    flush=True,
                )

            if status.done:
                # 完成后已经发送了一帧零动作，退出让上层 planner 决定下一阶段。
                print(f"fixed action completed: action={args.action}", flush=True)
                return 0
    except KeyboardInterrupt:
        print("fixed action player stopped", flush=True)
    finally:
        recv_sock.close()
        send_sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
