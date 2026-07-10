#!/usr/bin/env python3
"""本机模拟 Orin relay：发送状态包，接收 PC 返回的动作包。"""

from __future__ import annotations

import argparse
import math
import socket
import time
from pathlib import Path

import sys

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from runtime_bridge.protocol import MachineStatePacket, PacketDecodeError, PolicyActionPacket, decode_packet, encode_packet, now_ms


def build_arg_parser() -> argparse.ArgumentParser:
    """构造 mock Orin relay 参数。"""
    parser = argparse.ArgumentParser(description="模拟 Orin relay，向PC发送STM32状态并打印PC动作。")
    parser.add_argument("--pc-host", default="127.0.0.1", help="PC runtime 接收状态的IP")
    parser.add_argument("--state-port", type=int, default=18081, help="Orin -> PC 状态UDP端口")
    parser.add_argument("--action-bind-host", default="0.0.0.0", help="监听PC动作的本地地址")
    parser.add_argument("--action-port", type=int, default=18082, help="PC -> Orin 动作UDP端口")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="模拟状态发送频率，默认对齐当前STM32传感器10Hz")
    parser.add_argument("--print-every", type=int, default=20, help="每收到多少个动作打印一次；0表示不打印")
    return parser


def make_mock_state(seq: int, period_s: float) -> MachineStatePacket:
    """生成正式 machine_state_v1，便于PC端按真实协议联调。"""
    t = seq * period_s
    positions = {
        "swing": 0.15 * math.sin(t * 0.2),
        "boom": 0.35 + 0.05 * math.sin(t),
        "arm": -0.75 + 0.04 * math.cos(t * 0.7),
        "bucket": 0.45 + 0.03 * math.sin(t * 1.3),
    }
    velocities = {name: 0.0 for name in positions}
    return MachineStatePacket(
        seq=seq,
        stamp_ms=now_ms(),
        safety={
            "estop": False,
            "stm32_alive": True,
            "sensor_valid": True,
            "control_enabled": False,
            "fault_flags": [],
        },
        actuator_state={
            # 关键：actuator_state 是ONNX observation原始执行器状态，这里只是联调用的合理占位。
            "boom": {"position_m": 0.012 + 0.002 * math.sin(t), "velocity_mps": 0.0},
            "stick": {"position_m": -0.018 + 0.002 * math.cos(t), "velocity_mps": 0.0},
            "bucket": {"position_m": 0.006 + 0.001 * math.sin(t * 1.3), "velocity_mps": 0.0},
            "swing": {"position_rad": positions["swing"], "velocity_rad_s": velocities["swing"]},
        },
        joint_state={
            "position_rad": positions,
        },
    )


def main() -> int:
    """入口：循环发送状态并非阻塞接收动作。"""
    args = build_arg_parser().parse_args()
    period_s = 1.0 / max(args.rate_hz, 1.0)

    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind((args.action_bind_host, args.action_port))
    recv_sock.setblocking(False)

    destination = (args.pc_host, args.state_port)
    seq = 0
    action_count = 0
    print(f"mock Orin relay started: state -> {destination}, action <- {args.action_bind_host}:{args.action_port}", flush=True)

    interrupted = False
    try:
        while True:
            state = make_mock_state(seq, period_s)
            send_sock.sendto(encode_packet(state), destination)
            seq += 1

            while True:
                try:
                    payload, address = recv_sock.recvfrom(4096)
                except BlockingIOError:
                    break
                try:
                    packet = decode_packet(payload)
                except PacketDecodeError as exc:
                    print(f"drop invalid action from {address}: {exc}")
                    continue
                if isinstance(packet, PolicyActionPacket):
                    action_count += 1
                    if args.print_every > 0 and action_count % args.print_every == 0:
                        print(
                            f"action[{action_count}] from {address}: seq={packet.seq}, "
                            f"order={list(packet.action_order)}, action={packet.action}",
                            flush=True,
                        )

            time.sleep(period_s)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        send_sock.close()
        recv_sock.close()
        if interrupted:
            try:
                print("mock Orin relay stopped", flush=True)
            except KeyboardInterrupt:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
