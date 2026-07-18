#!/usr/bin/env python3
"""Convert an Orin log or PC Action Journal session into a replayable CSV."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MACHINE_PROFILE = AIRY_ROOT.parent / "shared" / "machine_profile.json"
ACTION_NAMES = ("boom", "stick", "bucket", "swing")
HEADER = (
    "sample_index", "timestamp_s", "unity_time_s", "phase", "mode",
    "boom_action_cmd", "stick_action_cmd", "bucket_action_cmd", "swing_action_cmd",
    "boom_v_ref_mps", "stick_v_ref_mps", "bucket_v_ref_mps", "swing_v_ref_radps",
    "boom_q_m", "stick_q_m", "bucket_q_m", "swing_q_rad",
    "boom_q_norm", "stick_q_norm", "bucket_q_norm", "swing_q_norm",
    "bucket_tip_world_x", "bucket_tip_world_y", "bucket_tip_world_z",
    "bucket_tip_local_x", "bucket_tip_local_y", "bucket_tip_local_z",
)
LOG_PATTERN = re.compile(
    r"STM32 TX policy_action seq=(?P<seq>\d+):\s*"
    r"(?P<stamp>\d+);(?P<boom>[^;\s]+);(?P<stick>[^;\s]+);"
    r"(?P<bucket>[^;\s]+);(?P<swing>[^;\s]+)"
)


@dataclass(frozen=True)
class AcceptedCommand:
    sequence: int
    stamp_ms: int
    action: tuple[float, float, float, float]


def _validate_command_order(commands: list[AcceptedCommand], *, source: str) -> None:
    for previous, current in zip(commands, commands[1:], strict=False):
        if current.sequence != previous.sequence + 1:
            raise ValueError(
                f"{source} sequence gap: {previous.sequence} -> {current.sequence}"
            )
        # The journal records wall-clock time at millisecond precision. Two
        # consecutive sends can legitimately share one millisecond; sequence
        # order remains authoritative in that case. Only an actual time
        # regression makes the stream unsafe to replay.
        if current.stamp_ms < previous.stamp_ms:
            raise ValueError(f"{source} timestamps must increase")


def _format(value: float) -> str:
    return f"{value:.12g}"


def parse_commands(text: str) -> list[AcceptedCommand]:
    commands: list[AcceptedCommand] = []
    for line in text.splitlines():
        match = LOG_PATTERN.search(line)
        if match is None:
            continue
        action = tuple(float(match.group(name)) for name in ACTION_NAMES)
        if not all(math.isfinite(value) for value in action):
            raise ValueError("policy_action contains a non-finite value")
        commands.append(
            AcceptedCommand(int(match.group("seq")), int(match.group("stamp")), action)
        )
    if not commands:
        raise ValueError("no accepted STM32 policy_action records found")
    _validate_command_order(commands, source="policy_action")
    return commands


def parse_pc_journal_commands(text: str) -> list[AcceptedCommand]:
    commands: list[AcceptedCommand] = []
    for line_number, line in enumerate(text.splitlines(), start=1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid PC action journal JSON at line {line_number}") from exc
        packet = record.get("packet")
        if (
            record.get("schema") != "pc_orin_action_send_v1"
            or not isinstance(packet, dict)
            or packet.get("type") != "policy_action"
            or packet.get("action_order") != list(ACTION_NAMES)
        ):
            raise ValueError(f"invalid PC action journal record at line {line_number}")
        action = packet.get("action")
        sequence = packet.get("seq")
        stamp_ms = record.get("recorded_at_pc_ms")
        if (
            not isinstance(action, list)
            or len(action) != len(ACTION_NAMES)
            or isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or isinstance(stamp_ms, bool)
            or not isinstance(stamp_ms, int)
        ):
            raise ValueError(f"invalid PC action packet at line {line_number}")
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            for value in action
        ):
            raise ValueError(
                f"non-numeric action value in PC journal at line {line_number}"
            )
        values = tuple(float(value) for value in action)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("PC policy_action contains a non-finite value")
        commands.append(AcceptedCommand(sequence, stamp_ms, values))
    if not commands:
        raise ValueError("no PC policy_action journal records found")
    _validate_command_order(commands, source="PC policy_action")
    return commands


def latest_motion_session(
    commands: list[AcceptedCommand], *, session_gap_ms: int
) -> list[AcceptedCommand]:
    if session_gap_ms <= 0:
        raise ValueError("session_gap_ms must be positive")
    sessions: list[list[AcceptedCommand]] = []
    current: list[AcceptedCommand] = []
    for command in commands:
        if current and command.stamp_ms - current[-1].stamp_ms > session_gap_ms:
            sessions.append(current)
            current = []
        current.append(command)
    if current:
        sessions.append(current)
    motion_sessions = [
        session for session in sessions if any(any(command.action) for command in session)
    ]
    if not motion_sessions:
        raise ValueError("PC action journal contains no non-zero motion session")
    return motion_sessions[-1]


def _normalized_action(
    action: tuple[float, float, float, float], machine_profile: dict
) -> tuple[float, float, float, float]:
    normalized: list[float] = []
    for name, value in zip(ACTION_NAMES, action, strict=True):
        actuator = machine_profile["actuators"][name]
        limit = float(
            actuator["max_speed_positive"] if value >= 0.0 else actuator["max_speed_negative"]
        )
        command = value / limit
        if abs(command) > 1.000001:
            raise ValueError(f"{name} velocity {value} exceeds machine profile limit {limit}")
        normalized.append(max(-1.0, min(1.0, command)))
    return tuple(normalized)  # type: ignore[return-value]


def export_csv(
    commands: list[AcceptedCommand],
    output: Path,
    machine_profile: dict,
    *,
    phase: str,
    mode: str,
    source: str = "orin_stm32_accepted_policy_action_log",
) -> None:
    intervals = [
        current.stamp_ms - previous.stamp_ms
        for previous, current in zip(commands, commands[1:], strict=False)
    ]
    terminal_interval_ms = int(round(statistics.median(intervals))) if intervals else 50
    terminal = AcceptedCommand(
        commands[-1].sequence + 1,
        commands[-1].stamp_ms + terminal_interval_ms,
        (0.0, 0.0, 0.0, 0.0),
    )
    rows = [*commands, terminal]
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8-sig", newline="") as csv_file:
        csv_file.write("# rl_excavator_open_loop_velocity_export\n")
        csv_file.write(f"# source={source}\n")
        csv_file.write("# success=unknown_hardware_replay_pending\n")
        csv_file.write("# units: boom/stick/bucket velocity=m/s, swing velocity=rad/s\n")
        csv_file.write("# profile_action_order=boom|stick|bucket|swing\n")
        csv_file.write("# state_columns_available=false\n")
        csv_file.write("# terminal_zero_appended=true\n")
        csv_file.write(f"# source_stamp_start_ms={commands[0].stamp_ms}\n")
        csv_file.write(f"# source_stamp_end_ms={commands[-1].stamp_ms}\n")
        writer = csv.writer(csv_file)
        writer.writerow(HEADER)
        start_ms = commands[0].stamp_ms
        for index, command in enumerate(rows):
            relative_s = (command.stamp_ms - start_ms) / 1000.0
            normalized = _normalized_action(command.action, machine_profile)
            writer.writerow(
                [
                    index,
                    _format(relative_s),
                    "",
                    phase,
                    mode,
                    *(_format(value) for value in normalized),
                    *(_format(value) for value in command.action),
                    *("" for _ in range(14)),
                ]
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input", type=Path, required=True, help="Orin text log or PC journal JSONL"
    )
    parser.add_argument(
        "--input-format",
        choices=("orin-log", "pc-journal"),
        default="orin-log",
    )
    parser.add_argument(
        "--latest-session",
        action="store_true",
        help="export the latest non-zero session separated by a command gap",
    )
    parser.add_argument("--session-gap-ms", type=int, default=1500)
    parser.add_argument("--output", type=Path, required=True, help="output CSV")
    parser.add_argument("--machine-profile", type=Path, default=DEFAULT_MACHINE_PROFILE)
    parser.add_argument("--phase", default="TrackingToDump")
    parser.add_argument("--mode", default="CarryMaterial")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    text = args.input.read_text(encoding="utf-8")
    if args.input_format == "pc-journal":
        commands = parse_pc_journal_commands(text)
        source = "pc_action_journal"
    else:
        commands = parse_commands(text)
        source = "orin_stm32_accepted_policy_action_log"
    if args.latest_session:
        commands = latest_motion_session(
            commands, session_gap_ms=args.session_gap_ms
        )
    machine_profile = json.loads(args.machine_profile.read_text(encoding="utf-8"))
    export_csv(
        commands,
        args.output,
        machine_profile,
        phase=args.phase,
        mode=args.mode,
        source=source,
    )
    print(f"input_records: {len(commands)}")
    print(f"output_records: {len(commands) + 1} (includes terminal zero)")
    print(f"output: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
