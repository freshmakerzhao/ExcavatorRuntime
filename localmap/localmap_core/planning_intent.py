"""从唯一 target ID 推导一次规划所需的任务意图。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


class PlanningIntentError(ValueError):
    """目标不存在、重复或无法映射为任务模式。"""


@dataclass(frozen=True)
class PlanningIntent:
    target_id: str
    target_kind: str
    task_mode: str


def resolve_planning_intent(
    local_map: Mapping[str, Any],
    target_id: str,
    task_mode_by_target_kind: Mapping[str, str],
) -> PlanningIntent:
    """在 dig/dump 目标中唯一查找 ID，并推导任务模式。"""
    matches: list[str] = []
    for target_kind, collection_name in (
        ("dig", "dig_targets"),
        ("dump", "dump_targets"),
    ):
        matches.extend(
            target_kind
            for target in local_map.get(collection_name, [])
            if target.get("id") == target_id
        )

    if len(matches) != 1:
        raise PlanningIntentError(
            f"target_id 必须在 dig/dump targets 中唯一存在: {target_id!r}, matches={matches}"
        )

    target_kind = matches[0]
    try:
        task_mode = task_mode_by_target_kind[target_kind]
    except KeyError as exc:
        raise PlanningIntentError(f"target kind 缺少 task mode 映射: {target_kind}") from exc
    return PlanningIntent(
        target_id=target_id,
        target_kind=target_kind,
        task_mode=task_mode,
    )
