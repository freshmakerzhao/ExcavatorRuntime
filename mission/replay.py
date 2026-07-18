"""不创建网络发送器的 Mission shadow/replay 执行器。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

from mission.contract import ExcavationMission
from mission.state_machine import MissionEvent, MissionState, MissionStateMachine, MissionTransition
from mission.trajectory_tracker import TrajectoryTracker


class MissionReplayError(ValueError):
    """Replay 数据不能证明完整 Mission 阶段序列。"""


@dataclass(frozen=True)
class MissionReplayResult:
    final_state: MissionState
    transitions: tuple[MissionTransition, ...]
    action_datagrams: int
    dump_plan_start_m: tuple[float, float, float]


def run_mission_replay(
    mission: ExcavationMission,
    replay: Mapping[str, Any],
) -> MissionReplayResult:
    """执行纯内存 replay；该模块没有任何 UDP action sender。"""
    expected_fields = {
        "schema_version",
        "frame_id",
        "to_dig",
        "dig_settle_duration_s",
        "dig_primitive_result",
        "load_verification_result",
        "post_dig_bucket_tip_m",
        "to_dump",
        "dump_settle_duration_s",
        "dump_primitive_result",
        "empty_verification_result",
        "return_home_result",
    }
    missing = expected_fields - set(replay)
    unknown = set(replay) - expected_fields
    if missing or unknown:
        raise MissionReplayError(
            f"Replay字段不匹配: missing={sorted(missing)}, unknown={sorted(unknown)}"
        )
    if replay.get("schema_version") != "mission_replay.v1":
        raise MissionReplayError("Replay schema必须是mission_replay.v1")
    if replay.get("frame_id") != mission.frame_id:
        raise MissionReplayError("Replay frame与Mission不一致")

    machine = MissionStateMachine()
    machine, _ = machine.advance(MissionEvent.START)

    dig = _segment(replay, "to_dig")
    _require_same_point("dig goal", dig["waypoints"][-1], mission.targets["dig"].position_m)
    machine, _ = machine.advance(MissionEvent.PLAN_SUCCEEDED)
    if not _track_segment(mission, dig):
        machine, _ = machine.advance(MissionEvent.FAIL, reason="track_to_dig_failed")
        return _result(machine, (0.0, 0.0, 0.0))
    machine, _ = machine.advance(MissionEvent.TRACK_COMPLETED)
    if not _settled(replay.get("dig_settle_duration_s"), mission.limits.settle_s):
        machine, _ = machine.advance(MissionEvent.FAIL, reason="dig_settle_failed")
        return _result(machine, (0.0, 0.0, 0.0))
    machine, _ = machine.advance(MissionEvent.SETTLED)

    if replay.get("dig_primitive_result") != "completed":
        machine, _ = machine.advance(MissionEvent.FAIL, reason="dig_primitive_failed")
        return _result(machine, (0.0, 0.0, 0.0))
    machine, _ = machine.advance(MissionEvent.PRIMITIVE_COMPLETED)
    if replay.get("load_verification_result") != "passed":
        machine, _ = machine.advance(MissionEvent.FAIL, reason="load_verification_failed")
        return _result(machine, (0.0, 0.0, 0.0))
    machine, _ = machine.advance(MissionEvent.VERIFICATION_PASSED)

    post_dig_tip = _triplet("post_dig_bucket_tip_m", replay.get("post_dig_bucket_tip_m"))
    dump = _segment(replay, "to_dump")
    _require_same_point("dump start", dump["waypoints"][0], post_dig_tip)
    _require_same_point("dump goal", dump["waypoints"][-1], mission.targets["dump"].position_m)
    machine, _ = machine.advance(MissionEvent.PLAN_SUCCEEDED)
    if not _track_segment(mission, dump):
        machine, _ = machine.advance(MissionEvent.FAIL, reason="track_to_dump_failed")
        return _result(machine, post_dig_tip)
    machine, _ = machine.advance(MissionEvent.TRACK_COMPLETED)
    if not _settled(replay.get("dump_settle_duration_s"), mission.limits.settle_s):
        machine, _ = machine.advance(MissionEvent.FAIL, reason="dump_settle_failed")
        return _result(machine, post_dig_tip)
    machine, _ = machine.advance(MissionEvent.SETTLED)

    if replay.get("dump_primitive_result") != "completed":
        machine, _ = machine.advance(MissionEvent.FAIL, reason="dump_primitive_failed")
        return _result(machine, post_dig_tip)
    machine, _ = machine.advance(MissionEvent.PRIMITIVE_COMPLETED)
    if replay.get("empty_verification_result") != "passed":
        machine, _ = machine.advance(MissionEvent.FAIL, reason="empty_verification_failed")
        return _result(machine, post_dig_tip)
    machine, _ = machine.advance(MissionEvent.VERIFICATION_PASSED)
    if replay.get("return_home_result") != "completed":
        machine, _ = machine.advance(MissionEvent.FAIL, reason="return_home_failed")
        return _result(machine, post_dig_tip)
    machine, _ = machine.advance(MissionEvent.HOME_REACHED)
    return _result(machine, post_dig_tip)


def _track_segment(mission: ExcavationMission, segment: dict[str, Any]) -> bool:
    tracker = TrajectoryTracker(
        waypoints=tuple(segment["waypoints"]),
        tolerance_m=mission.limits.waypoint_tolerance_m,
        dwell_s=mission.limits.waypoint_dwell_s,
        timeout_s=mission.limits.tracking_timeout_s,
    )
    for sample_time_s, position_m in segment["tip_samples"]:
        tracker, update = tracker.advance(position_m, now_s=sample_time_s)
        if update.timed_out:
            return False
        if update.completed:
            return True
    return False


def _segment(replay: Mapping[str, Any], name: str) -> dict[str, Any]:
    value = replay.get(name)
    if not isinstance(value, dict):
        raise MissionReplayError(f"Replay缺少{name}")
    waypoints = value.get("waypoints")
    samples = value.get("tip_samples")
    if not isinstance(waypoints, list) or not waypoints:
        raise MissionReplayError(f"{name}.waypoints不能为空")
    if not isinstance(samples, list) or not samples:
        raise MissionReplayError(f"{name}.tip_samples不能为空")
    validated_samples = []
    previous_time_s: float | None = None
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict) or set(sample) != {"time_s", "position_m"}:
            raise MissionReplayError(
                f"{name}.tip_samples[{index}]字段必须是time_s/position_m"
            )
        time_s = _finite_number(f"{name}.tip_samples[{index}].time_s", sample["time_s"])
        if previous_time_s is not None and time_s < previous_time_s:
            raise MissionReplayError(f"{name}.tip_samples时间戳不能倒退")
        previous_time_s = time_s
        validated_samples.append(
            (time_s, _triplet(f"{name}.tip_samples[{index}].position_m", sample["position_m"]))
        )
    return {
        "waypoints": tuple(_triplet(f"{name}.waypoints", point) for point in waypoints),
        "tip_samples": tuple(validated_samples),
    }


def _triplet(name: str, value: object) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise MissionReplayError(f"{name}必须是长度3数组")
    return tuple(_finite_number(f"{name}[{index}]", item) for index, item in enumerate(value))


def _finite_number(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise MissionReplayError(f"{name}必须是有限数值")
    return float(value)


def _settled(value: object, required_s: float) -> bool:
    try:
        duration_s = _finite_number("settle_duration_s", value)
    except MissionReplayError:
        return False
    return duration_s + 1e-12 >= required_s


def _require_same_point(
    name: str,
    actual: tuple[float, float, float],
    expected: tuple[float, float, float],
) -> None:
    if math.dist(actual, expected) > 1e-9:
        raise MissionReplayError(f"{name}与Mission Snapshot不一致")


def _result(
    machine: MissionStateMachine,
    dump_plan_start_m: tuple[float, float, float],
) -> MissionReplayResult:
    return MissionReplayResult(
        final_state=machine.state,
        transitions=machine.transitions,
        action_datagrams=0,
        dump_plan_start_m=dump_plan_start_m,
    )
