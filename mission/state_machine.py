"""挖掘—运输—倾倒 Mission 的不可变阶段状态机。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class MissionState(str, Enum):
    IDLE = "idle"
    PLANNING_TO_DIG = "planning_to_dig"
    TRACKING_TO_DIG = "tracking_to_dig"
    SETTLING_AFTER_DIG_TRACK = "settling_after_dig_track"
    DIGGING = "digging"
    VERIFYING_LOAD = "verifying_load"
    PLANNING_TO_DUMP = "planning_to_dump"
    TRACKING_TO_DUMP = "tracking_to_dump"
    SETTLING_AFTER_DUMP_TRACK = "settling_after_dump_track"
    DUMPING = "dumping"
    VERIFYING_EMPTY = "verifying_empty"
    RETURNING_HOME = "returning_home"
    COMPLETED = "completed"
    FAILED = "failed"
    STOPPED = "stopped"


class MissionEvent(str, Enum):
    START = "start"
    PLAN_SUCCEEDED = "plan_succeeded"
    TRACK_COMPLETED = "track_completed"
    SETTLED = "settled"
    PRIMITIVE_COMPLETED = "primitive_completed"
    VERIFICATION_PASSED = "verification_passed"
    HOME_REACHED = "home_reached"
    FAIL = "fail"
    STOP = "stop"


class MissionTransitionError(ValueError):
    """事件不能从当前 Mission 阶段发生。"""


@dataclass(frozen=True)
class MissionTransition:
    from_state: MissionState
    event: MissionEvent
    to_state: MissionState
    reason: str


_NOMINAL_TRANSITIONS = {
    (MissionState.IDLE, MissionEvent.START): MissionState.PLANNING_TO_DIG,
    (MissionState.PLANNING_TO_DIG, MissionEvent.PLAN_SUCCEEDED): MissionState.TRACKING_TO_DIG,
    (MissionState.TRACKING_TO_DIG, MissionEvent.TRACK_COMPLETED): MissionState.SETTLING_AFTER_DIG_TRACK,
    (MissionState.SETTLING_AFTER_DIG_TRACK, MissionEvent.SETTLED): MissionState.DIGGING,
    (MissionState.DIGGING, MissionEvent.PRIMITIVE_COMPLETED): MissionState.VERIFYING_LOAD,
    (MissionState.VERIFYING_LOAD, MissionEvent.VERIFICATION_PASSED): MissionState.PLANNING_TO_DUMP,
    (MissionState.PLANNING_TO_DUMP, MissionEvent.PLAN_SUCCEEDED): MissionState.TRACKING_TO_DUMP,
    (MissionState.TRACKING_TO_DUMP, MissionEvent.TRACK_COMPLETED): MissionState.SETTLING_AFTER_DUMP_TRACK,
    (MissionState.SETTLING_AFTER_DUMP_TRACK, MissionEvent.SETTLED): MissionState.DUMPING,
    (MissionState.DUMPING, MissionEvent.PRIMITIVE_COMPLETED): MissionState.VERIFYING_EMPTY,
    (MissionState.VERIFYING_EMPTY, MissionEvent.VERIFICATION_PASSED): MissionState.RETURNING_HOME,
    (MissionState.RETURNING_HOME, MissionEvent.HOME_REACHED): MissionState.COMPLETED,
}


@dataclass(frozen=True)
class MissionStateMachine:
    state: MissionState = MissionState.IDLE
    transitions: tuple[MissionTransition, ...] = ()

    def advance(
        self,
        event: MissionEvent,
        *,
        reason: str = "",
    ) -> tuple["MissionStateMachine", MissionTransition]:
        """应用一个显式事件并返回新的状态机快照。"""
        terminal = {MissionState.COMPLETED, MissionState.FAILED, MissionState.STOPPED}
        if self.state in terminal:
            raise MissionTransitionError(f"Mission终态 {self.state.value} 不接受新事件")
        if event == MissionEvent.FAIL:
            target = MissionState.FAILED
        elif event == MissionEvent.STOP:
            target = MissionState.STOPPED
        else:
            target = _NOMINAL_TRANSITIONS.get((self.state, event))
        if target is None:
            raise MissionTransitionError(
                f"Mission不能从 {self.state.value} 接受事件 {event.value}"
            )
        transition = MissionTransition(self.state, event, target, reason)
        return (
            MissionStateMachine(target, (*self.transitions, transition)),
            transition,
        )
