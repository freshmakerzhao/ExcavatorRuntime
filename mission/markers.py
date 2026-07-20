"""与 ROS 消息无关的 Mission 目标可视化描述。"""

from __future__ import annotations

from dataclasses import dataclass

from mission.contract import ExcavationMission


@dataclass(frozen=True)
class MissionMarkerSpec:
    phase: str
    frame_id: str
    position_m: tuple[float, float, float]
    diameter_m: float
    color_rgba: tuple[float, float, float, float]
    label: str


def build_mission_marker_specs(mission: ExcavationMission) -> tuple[MissionMarkerSpec, ...]:
    """从同一个不可变 Mission Snapshot 生成 dig/dump 标记。"""
    colors = {
        "dig": (1.0, 0.45, 0.0, 0.85),
        "dump": (0.65, 0.15, 1.0, 0.85),
    }
    status = mission.target_status.upper()
    return tuple(
        MissionMarkerSpec(
            phase=phase,
            frame_id=mission.frame_id,
            position_m=mission.targets[phase].position_m,
            diameter_m=mission.targets[phase].radius_m * 2.0,
            color_rgba=colors[phase],
            label=f"{phase.upper()} [{status}] {mission.mission_id}",
        )
        for phase in ("dig", "dump")
    )
