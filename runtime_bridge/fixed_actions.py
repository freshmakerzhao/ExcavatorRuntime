"""固定挖掘/倾倒动作执行器。

参考 Unity 的 ExcavationCycleTask：先由外部 planner/策略把 bucket tip 送到目标点，
再用固定的归一化关节增量序列完成挖掘或倾倒。这里不做路径规划，只根据 Orin
回传的 actuator_state 追踪固定动作段。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

from runtime_bridge.observation import normalize_position
from runtime_bridge.protocol import ACTION_ORDER, MachineStatePacket, PolicyActionPacket, make_zero_action, now_ms


@dataclass(frozen=True)
class FixedActionStep:
    """固定动作中的一段相对归一化关节位移。"""

    label: str
    delta_normalized_qpos: tuple[float, float, float, float]


@dataclass(frozen=True)
class FixedActionStatus:
    """固定动作执行状态，供命令行打印和后续 planner 编排使用。"""

    phase: str
    step_index: int
    step_label: str
    max_error: float
    done: bool


def fixed_action_sequence(name: str) -> list[FixedActionStep]:
    """返回内置固定动作序列；数值直接对齐 Unity ExcavationCycleTask。"""
    if name == "dig":
        return [
            FixedActionStep("下探", (0.50, 0.00, 0.00, 0.00)),
            FixedActionStep("收斗", (0.00, 0.00, -1.40, 0.00)),
            FixedActionStep("提斗", (-0.50, 0.20, 0.00, 0.00)),
        ]
    if name == "dump":
        return [
            FixedActionStep("打开铲斗", (0.00, 0.00, 1.40, 0.00)),
            FixedActionStep("回收铲斗", (0.00, 0.00, -1.40, 0.00)),
        ]
    raise ValueError(f"未知固定动作: {name}")


def physical_velocity_action_from_normalized(action: Sequence[float], machine_profile: dict[str, Any]) -> list[float]:
    """把训练语义的 [-1,1] 动作转换为发送给 Orin 的物理速度。

    输出仍放在 policy_action.action 字段中，action_type 保持 normalized_velocity_command 以兼容 Orin。
    """
    if len(action) != len(ACTION_ORDER):
        raise ValueError("action 必须是长度4数组")
    actuators = machine_profile["actuators"]
    result: list[float] = []
    for name, value in zip(ACTION_ORDER, action, strict=True):
        normalized = clamp(float(value), -1.0, 1.0)
        actuator = actuators[name]
        speed_limit = (
            float(actuator["max_speed_positive"])
            if normalized >= 0.0
            else float(actuator["max_speed_negative"])
        )
        # deploy_sign 是真机发送方向校准入口；机型常数只从 machine_profile 读取。
        deploy_sign = float(actuator.get("deploy_sign", 1.0) or 1.0)
        result.append(normalized * speed_limit * deploy_sign)
    return result


class FixedActionExecutor:
    """用 Orin 状态闭环追踪固定动作序列。"""

    def __init__(
        self,
        steps: Sequence[FixedActionStep],
        machine_profile: dict[str, Any],
        *,
        kp: float = 1.5,
        min_action: float = 0.08,
        max_action: float = 1.0,
        tolerance: float = 0.03,
        step_timeout_s: float = 3.0,
        hold_s: float = 0.15,
        lock_zero_axes: bool = True,
    ) -> None:
        if not steps:
            raise ValueError("固定动作序列不能为空")
        self.steps = list(steps)
        self.machine_profile = machine_profile
        self.kp = max(float(kp), 0.01)
        self.min_action = max(float(min_action), 0.0)
        self.max_action = max(float(max_action), 0.01)
        self.tolerance = max(float(tolerance), 0.001)
        self.step_timeout_s = max(float(step_timeout_s), 0.1)
        self.hold_s = max(float(hold_s), 0.0)
        self.lock_zero_axes = bool(lock_zero_axes)
        self.step_index = 0
        self.step_started_at_s: float | None = None
        self.hold_until_s: float | None = None
        self.step_start_qpos: tuple[float, float, float, float] | None = None

    @property
    def done(self) -> bool:
        """是否已经完成全部固定动作段。"""
        return self.step_index >= len(self.steps)

    def step(self, state: MachineStatePacket, *, now_s: float, seq: int, valid_for_ms: int) -> tuple[PolicyActionPacket, FixedActionStatus]:
        """根据当前状态推进一步，返回应发送给 Orin 的动作包。"""
        if self.done:
            return make_zero_action(seq, valid_for_ms), self._status("done", 0.0, True)

        if self.hold_until_s is not None:
            if now_s < self.hold_until_s:
                return make_zero_action(seq, valid_for_ms), self._status("hold", 0.0, False)
            self._advance_step()
            if self.done:
                return make_zero_action(seq, valid_for_ms), self._status("done", 0.0, True)

        current_qpos = current_normalized_joint_pose(state, self.machine_profile)
        if self.step_started_at_s is None:
            self.step_started_at_s = now_s
            self.step_start_qpos = current_qpos

        step = self.steps[self.step_index]
        start_qpos = self.step_start_qpos or current_qpos
        target_qpos = tuple(clamp(start_qpos[i] + step.delta_normalized_qpos[i]) for i in range(4))
        error = tuple(target_qpos[i] - current_qpos[i] for i in range(4))
        if self.lock_zero_axes:
            # 关键：与 Unity 一致，未参与该段动作的轴不追误差，避免无关关节抖动。
            error = tuple(0.0 if abs(step.delta_normalized_qpos[i]) <= 1e-4 else error[i] for i in range(4))
        max_error = max(abs(value) for value in error)
        timed_out = (now_s - self.step_started_at_s) >= self.step_timeout_s
        if max_error <= self.tolerance or timed_out:
            self.hold_until_s = now_s + self.hold_s
            return make_zero_action(seq, valid_for_ms), self._status("hold", max_error, False)

        normalized_action = tuple(self._servo_axis(value) for value in error)
        physical_action = physical_velocity_action_from_normalized(normalized_action, self.machine_profile)
        packet = PolicyActionPacket(
            seq=seq,
            stamp_ms=now_ms(),
            action=physical_action,
            action_type="normalized_velocity_command",
            valid_for_ms=valid_for_ms,
        )
        return packet, self._status("running", max_error, False)

    def _advance_step(self) -> None:
        """进入下一段固定动作。"""
        self.step_index += 1
        self.step_started_at_s = None
        self.hold_until_s = None
        self.step_start_qpos = None

    def _servo_axis(self, error: float) -> float:
        """把归一化位置误差转换为归一化速度动作。"""
        if abs(error) <= self.tolerance:
            return 0.0
        max_action = clamp(self.max_action, 0.01, 1.0)
        min_action = clamp(self.min_action, 0.0, max_action)
        raw_action = self.kp * error
        magnitude = clamp(abs(raw_action), min_action, max_action)
        return (1.0 if error >= 0.0 else -1.0) * magnitude

    def _status(self, phase: str, max_error: float, done: bool) -> FixedActionStatus:
        """构造当前执行状态。"""
        if self.done:
            return FixedActionStatus(phase, self.step_index, "完成", max_error, done)
        return FixedActionStatus(phase, self.step_index, self.steps[self.step_index].label, max_error, done)


def current_normalized_joint_pose(state: MachineStatePacket, machine_profile: dict[str, Any]) -> tuple[float, float, float, float]:
    """从 Orin actuator_state 计算 boom/stick/bucket/swing 的归一化姿态。"""
    actuators = machine_profile["actuators"]
    boom = normalize_position(state.actuator_state["boom"]["position_m"], actuators["boom"])
    stick = normalize_position(state.actuator_state["stick"]["position_m"], actuators["stick"])
    bucket = normalize_position(state.actuator_state["bucket"]["position_m"], actuators["bucket"])
    # 当前固定挖掘/倾倒序列不使用 swing 增量；这里保留0作为后续回转固定动作扩展点。
    swing = 0.0
    return (boom, stick, bucket, swing)


def clamp(value: float, lower: float = -1.0, upper: float = 1.0) -> float:
    """限制数值范围。"""
    return max(lower, min(upper, float(value)))
