#!/usr/bin/env python3
"""运行不包含动作发送能力的 Mission shadow/replay。"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

from mission.contract import MissionContractError, load_mission
from mission.replay import MissionReplayError, run_mission_replay


DEFAULT_MISSION = AIRY_ROOT / "mission" / "replays" / "mission.placeholder.json"
DEFAULT_REPLAY = AIRY_ROOT / "mission" / "replays" / "nominal.placeholder.json"
DEFAULT_OUTPUT = AIRY_ROOT / "mission" / "exports" / "replay_latest" / "mission_events.json"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="离线验证挖掘Mission阶段、轨迹推进和失败闭锁；永不发送真机动作。"
    )
    parser.add_argument("--mission", type=Path, default=DEFAULT_MISSION)
    parser.add_argument("--replay", type=Path, default=DEFAULT_REPLAY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser


def _load_replay(path: Path) -> dict:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise MissionReplayError(f"无法读取Replay: {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise MissionReplayError("Replay根节点必须是JSON object")
    return data


def _write_json_atomic(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=path.parent,
        text=True,
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        os.replace(temporary_name, path)
    except BaseException:
        Path(temporary_name).unlink(missing_ok=True)
        raise


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        args.output.unlink(missing_ok=True)
        mission = load_mission(args.mission)
        result = run_mission_replay(mission, _load_replay(args.replay))
        output = {
            "schema_version": "mission_replay_result.v1",
            "mode": "shadow_replay_no_motion",
            "mission_id": mission.mission_id,
            "mission_sha256": mission.sha256,
            "mission_snapshot_sha256": mission.sha256,
            "frame_id": mission.frame_id,
            "target_status": mission.target_status,
            "final_state": result.final_state.value,
            "action_datagrams": result.action_datagrams,
            "dump_plan_start_m": list(result.dump_plan_start_m),
            "transitions": [
                {
                    "from_state": transition.from_state.value,
                    "event": transition.event.value,
                    "to_state": transition.to_state.value,
                    "reason": transition.reason,
                }
                for transition in result.transitions
            ],
        }
        _write_json_atomic(args.output, output)
        print(
            f"shadow replay complete: final_state={result.final_state.value}, "
            f"action_datagrams={result.action_datagrams}, output={args.output}"
        )
        return 0 if result.final_state.value == "completed" else 2
    except (MissionContractError, MissionReplayError, OSError, ValueError) as exc:
        print(f"shadow replay failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
