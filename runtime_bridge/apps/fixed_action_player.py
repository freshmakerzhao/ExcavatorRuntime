#!/usr/bin/env python3
"""PC 侧固定动作 dry-run：接收 Orin 状态，计算但绝不发送动作。"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from dataclasses import replace
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from runtime_bridge.fixed_actions import (
    FixedActionExecutor,
    FixedActionStatus,
    load_fixed_action_profile,
)
from runtime_bridge.observation import load_machine_profile
from runtime_bridge.protocol import (
    MachineStatePacket,
    PacketDecodeError,
    decode_packet,
    estimate_remote_now_ms,
    make_zero_action,
    now_ms,
)
from runtime_bridge.runtime_config import DEFAULT_RUNTIME_CONFIG, load_runtime_config
from runtime_bridge.live_control import evaluate_actuator_state, evaluate_motion_state


def build_arg_parser() -> argparse.ArgumentParser:
    """构造固定动作播放器参数。"""
    parser = argparse.ArgumentParser(description="固定挖掘/倾倒动作 dry-run（不发送 UDP 动作）。")
    parser.add_argument("action", choices=("dig", "dump"), help="固定动作类型：dig=挖掘，dump=倾倒")
    parser.add_argument("--config", type=Path, default=DEFAULT_RUNTIME_CONFIG, help="运行配置JSON")
    return parser


def main() -> int:
    """收到状态后推进固定动作模型，输出候选值但不创建发送 socket。"""
    args = build_arg_parser().parse_args()
    try:
        config = load_runtime_config(args.config)
        config.artifacts.require_fixed_action_inputs()
        machine_profile = load_machine_profile(config.artifacts.machine_profile)
        fixed_action_profile = load_fixed_action_profile(
            config.artifacts.fixed_action_profile,
            machine_profile_path=config.artifacts.machine_profile,
            urdf_path=config.artifacts.urdf,
            expected_sha256=config.fixed_action.expected_profile_sha256,
        )
    except (OSError, ValueError) as exc:
        print(f"fixed action configuration error: {exc}", file=sys.stderr, flush=True)
        return 2

    tuning = fixed_action_profile.controller
    executor = FixedActionExecutor(
        fixed_action_profile.sequence(args.action),
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
    action_seq = 0
    state_count = 0
    started_at_s = time.monotonic()
    exit_code = 0

    print(
        f"fixed action player started: action={args.action}, "
        f"state <- {config.network.state_endpoint}, dry_run=true, udp_motion=disabled",
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
            motion_decision = evaluate_motion_state(packet)
            can_send, reason = motion_decision.allowed, motion_decision.reason
            actuator_decision = evaluate_actuator_state(packet, machine_profile)
            if can_send and not actuator_decision.allowed:
                can_send, reason = False, actuator_decision.reason
            action_stamp_ms = (
                estimate_remote_now_ms(packet.stamp_ms, received_pc_ms)
                if config.network.action_time_source == "orin"
                else now_ms()
            )
            if can_send:
                # 仅计算候选动作；唯一真机 Command Sink 位于 live behavior server。
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

            action_seq += 1
            if config.diagnostics.print_every > 0 and state_count % config.diagnostics.print_every == 0:
                print(
                    f"state[{state_count}] seq={packet.seq} step={status.step_index}:{status.step_label} "
                    f"phase={status.phase} err={status.max_error:.3f} candidate={action_packet.action} "
                    f"action_stamp={action_packet.stamp_ms} state_stamp={packet.stamp_ms} reason={reason}",
                    flush=True,
                )

            if status.done:
                print(f"fixed action dry-run completed: action={args.action}", flush=True)
                break
            if status.failed:
                print(
                    f"fixed action failed: action={args.action} "
                    f"reason={status.reason_code} step={status.step_index}",
                    file=sys.stderr,
                    flush=True,
                )
                exit_code = 2
                break
    except KeyboardInterrupt:
        print("fixed action player stopped", flush=True)
    finally:
        recv_sock.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
