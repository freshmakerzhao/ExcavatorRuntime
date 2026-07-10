"""ExcavatorRuntime 与 Orin relay 之间的 UDP JSON 协议。"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass
from typing import Any


JOINT_NAMES = ("swing", "boom", "arm", "bucket")
ACTUATOR_NAMES = ("boom", "stick", "bucket", "swing")
ACTION_ORDER = ("boom", "stick", "bucket", "swing")
MACHINE_STATE_SCHEMA_VERSION = "1.0"
POLICY_ACTION_SCHEMA_VERSION = "1.0"
MAX_PACKET_BYTES = 4096


class PacketDecodeError(ValueError):
    """收到的 UDP payload 不是当前协议支持的包。"""


@dataclass(frozen=True)
class ExcavatorStatePacket:
    """Orin -> PC：从 STM32 汇总来的挖掘机状态。"""

    seq: int
    stamp_ms: int
    joint_position_rad: dict[str, float]
    joint_velocity_rad_s: dict[str, float]
    estop: bool
    mode: str
    type: str = "excavator_state"

    def to_dict(self) -> dict[str, Any]:
        """转换为可 JSON 序列化的字典。"""
        return asdict(self)


@dataclass(frozen=True)
class MachineStatePacket:
    """Orin -> PC：正式真机本体状态 machine_state_v1。"""

    seq: int
    stamp_ms: int
    safety: dict[str, Any]
    actuator_state: dict[str, dict[str, float]]
    joint_state: dict[str, dict[str, float]]
    raw_sensor: dict[str, Any] | None = None
    source: str = "orin"
    machine_id: str = "scale_excavator_v1"
    type: str = "machine_state_v1"
    schema_version: str = MACHINE_STATE_SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        """转换为可 JSON 序列化的字典；raw_sensor 为空时不强制输出。"""
        data = asdict(self)
        if data["raw_sensor"] is None:
            data.pop("raw_sensor")
        return data

    @property
    def joint_position_rad(self) -> dict[str, float]:
        """兼容运动学桥：返回四关节角，单位rad。"""
        return self.joint_state["position_rad"]

    @property
    def joint_velocity_rad_s(self) -> dict[str, float]:
        """兼容运动学桥：返回四关节角速度，单位rad/s；Orin不发时为0。"""
        return self.joint_state.get("velocity_rad_s", {name: 0.0 for name in JOINT_NAMES})

    @property
    def estop(self) -> bool:
        """兼容旧打印逻辑：返回急停状态。"""
        return bool(self.safety["estop"])


@dataclass(frozen=True)
class PolicyActionPacket:
    """PC -> Orin：策略或规划侧输出的动作命令。"""

    seq: int
    stamp_ms: int
    action: list[float]
    action_type: str
    valid_for_ms: int
    action_order: tuple[str, str, str, str] = ACTION_ORDER
    type: str = "policy_action"
    schema_version: str = POLICY_ACTION_SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        """转换为Orin兼容字段顺序的JSON字典。"""
        return {
            "type": self.type,
            "schema_version": self.schema_version,
            "seq": self.seq,
            "stamp_ms": self.stamp_ms,
            "action_order": list(self.action_order),
            "action": list(self.action),
            "action_type": self.action_type,
            "valid_for_ms": self.valid_for_ms,
        }


@dataclass(frozen=True)
class HeartbeatPacket:
    """双向心跳包，用于判断链路是否还活着。"""

    seq: int
    stamp_ms: int
    role: str
    type: str = "heartbeat"

    def to_dict(self) -> dict[str, Any]:
        """转换为可 JSON 序列化的字典。"""
        return asdict(self)


def now_ms() -> int:
    """当前系统时间，单位毫秒。"""
    return int(time.time() * 1000)


def estimate_remote_now_ms(remote_stamp_ms: int, local_receive_ms: int, local_now_ms: int | None = None) -> int:
    """根据远端状态包时间戳估计远端当前时间，单位毫秒。

    Orin 会按自己的时钟检查 action.stamp_ms + valid_for_ms。PC 和 Orin 系统时钟
    有几十到上百毫秒偏差时，直接使用 PC 时间会导致 Orin 误判动作过期。
    """
    local_now = now_ms() if local_now_ms is None else int(local_now_ms)
    clock_offset_ms = int(remote_stamp_ms) - int(local_receive_ms)
    return local_now + clock_offset_ms


def encode_packet(packet: ExcavatorStatePacket | MachineStatePacket | PolicyActionPacket | HeartbeatPacket) -> bytes:
    """把协议包编码成 UDP payload。"""
    payload = json.dumps(packet.to_dict(), ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    if len(payload) > MAX_PACKET_BYTES:
        raise ValueError(f"packet too large: {len(payload)} bytes")
    return payload


def decode_packet(payload: bytes) -> ExcavatorStatePacket | MachineStatePacket | PolicyActionPacket | HeartbeatPacket:
    """从 UDP payload 解码协议包，并做最小字段校验。"""
    try:
        data = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PacketDecodeError(f"无法解析JSON: {exc}") from exc

    packet_type = data.get("type")
    if packet_type == "machine_state_v1":
        return decode_machine_state_packet(data)
    if packet_type == "excavator_state":
        return decode_state_packet(data)
    if packet_type == "policy_action":
        return decode_action_packet(data)
    if packet_type == "heartbeat":
        return decode_heartbeat_packet(data)
    raise PacketDecodeError(f"未知包类型: {packet_type}")


def decode_machine_state_packet(data: dict[str, Any]) -> MachineStatePacket:
    """校验并构造 machine_state_v1 包。"""
    schema_version = str(data.get("schema_version", MACHINE_STATE_SCHEMA_VERSION))
    if schema_version != MACHINE_STATE_SCHEMA_VERSION:
        raise PacketDecodeError(f"不支持的machine_state_v1版本: {schema_version}")

    safety = require_safety_state(data)
    actuator_state = require_actuator_state(data)
    joint_state = require_joint_state(data)
    raw_sensor = data.get("raw_sensor")
    if raw_sensor is not None and not isinstance(raw_sensor, dict):
        raise PacketDecodeError("raw_sensor 必须是字典")

    return MachineStatePacket(
        seq=require_int(data, "seq"),
        stamp_ms=require_int(data, "stamp_ms"),
        safety=safety,
        actuator_state=actuator_state,
        joint_state=joint_state,
        raw_sensor=raw_sensor,
        source=str(data.get("source", "orin")),
        machine_id=str(data.get("machine_id", "scale_excavator_v1")),
        schema_version=schema_version,
    )


def decode_state_packet(data: dict[str, Any]) -> ExcavatorStatePacket:
    """校验并构造 excavator_state 包。"""
    positions = require_joint_map(data, "joint_position_rad")
    velocities = data.get("joint_velocity_rad_s")
    if velocities is None:
        velocities = {name: 0.0 for name in JOINT_NAMES}
    else:
        velocities = require_joint_map(data, "joint_velocity_rad_s")

    return ExcavatorStatePacket(
        seq=require_int(data, "seq"),
        stamp_ms=require_int(data, "stamp_ms"),
        joint_position_rad=positions,
        joint_velocity_rad_s=velocities,
        estop=bool(data.get("estop", False)),
        mode=str(data.get("mode", "unknown")),
    )


def decode_action_packet(data: dict[str, Any]) -> PolicyActionPacket:
    """校验并构造 policy_action 包。"""
    schema_version = str(data.get("schema_version", POLICY_ACTION_SCHEMA_VERSION))
    if schema_version != POLICY_ACTION_SCHEMA_VERSION:
        raise PacketDecodeError(f"不支持的policy_action版本: {schema_version}")

    action = data.get("action")
    if not isinstance(action, list) or len(action) != 4:
        raise PacketDecodeError("action 必须是长度为4的数组")
    if not all(isinstance(value, int | float) for value in action):
        raise PacketDecodeError("action 中包含非数值")

    valid_for_ms = require_int(data, "valid_for_ms")
    if valid_for_ms <= 0:
        raise PacketDecodeError("valid_for_ms 必须大于0")

    action_order = tuple(data.get("action_order", ACTION_ORDER))
    if action_order != ACTION_ORDER:
        raise PacketDecodeError(f"action_order 必须是 {list(ACTION_ORDER)}")

    return PolicyActionPacket(
        seq=require_int(data, "seq"),
        stamp_ms=require_int(data, "stamp_ms"),
        action=[float(value) for value in action],
        action_type=str(data.get("action_type", "normalized_velocity_command")),
        valid_for_ms=valid_for_ms,
        action_order=ACTION_ORDER,
        schema_version=schema_version,
    )


def decode_heartbeat_packet(data: dict[str, Any]) -> HeartbeatPacket:
    """校验并构造 heartbeat 包。"""
    return HeartbeatPacket(
        seq=require_int(data, "seq"),
        stamp_ms=require_int(data, "stamp_ms"),
        role=str(data.get("role", "unknown")),
    )


def require_joint_map(data: dict[str, Any], key: str) -> dict[str, float]:
    """读取四关节数值字典。"""
    value = data.get(key)
    if not isinstance(value, dict):
        raise PacketDecodeError(f"{key} 必须是字典")

    missing = [name for name in JOINT_NAMES if name not in value]
    if missing:
        raise PacketDecodeError(f"{key} 缺少关节: {', '.join(missing)}")

    result: dict[str, float] = {}
    for name in JOINT_NAMES:
        joint_value = value[name]
        if not isinstance(joint_value, int | float):
            raise PacketDecodeError(f"{key}.{name} 不是数值")
        result[name] = float(joint_value)
    return result


def require_joint_state(data: dict[str, Any]) -> dict[str, dict[str, float]]:
    """读取正式协议中的 joint_state。"""
    value = data.get("joint_state")
    if not isinstance(value, dict):
        raise PacketDecodeError("joint_state 必须是字典")
    # 关键：FK 只硬依赖关节角；第一阶段Orin可不发关节角速度，PC侧补0。
    velocities = value.get("velocity_rad_s")
    if velocities is None:
        velocities = {name: 0.0 for name in JOINT_NAMES}
    else:
        velocities = require_joint_map(value, "velocity_rad_s")
    return {
        "position_rad": require_joint_map(value, "position_rad"),
        "velocity_rad_s": velocities,
    }


def require_safety_state(data: dict[str, Any]) -> dict[str, Any]:
    """读取 safety，并保证布尔字段和故障码类型明确。"""
    value = data.get("safety")
    if not isinstance(value, dict):
        raise PacketDecodeError("safety 必须是字典")

    result: dict[str, Any] = {}
    for key in ("estop", "stm32_alive", "sensor_valid", "control_enabled"):
        field_value = value.get(key)
        if not isinstance(field_value, bool):
            raise PacketDecodeError(f"safety.{key} 必须是bool")
        result[key] = field_value

    fault_flags = value.get("fault_flags")
    if not isinstance(fault_flags, list) or not all(isinstance(item, str) for item in fault_flags):
        raise PacketDecodeError("safety.fault_flags 必须是字符串数组")
    result["fault_flags"] = list(fault_flags)
    return result


def require_actuator_state(data: dict[str, Any]) -> dict[str, dict[str, float]]:
    """读取 actuator_state；这部分后续进入ONNX observation。"""
    value = data.get("actuator_state")
    if not isinstance(value, dict):
        raise PacketDecodeError("actuator_state 必须是字典")

    missing = [name for name in ACTUATOR_NAMES if name not in value]
    if missing:
        raise PacketDecodeError(f"actuator_state 缺少执行器: {', '.join(missing)}")

    result: dict[str, dict[str, float]] = {}
    for name in ("boom", "stick", "bucket"):
        result[name] = require_numeric_fields(value[name], f"actuator_state.{name}", ("position_m", "velocity_mps"))
    result["swing"] = require_numeric_fields(value["swing"], "actuator_state.swing", ("position_rad", "velocity_rad_s"))
    return result


def require_numeric_fields(value: Any, prefix: str, fields: tuple[str, str]) -> dict[str, float]:
    """读取一组数值字段，并转换成float。"""
    if not isinstance(value, dict):
        raise PacketDecodeError(f"{prefix} 必须是字典")

    result: dict[str, float] = {}
    for key in fields:
        field_value = value.get(key)
        if not isinstance(field_value, int | float):
            raise PacketDecodeError(f"{prefix}.{key} 必须是数值")
        result[key] = float(field_value)
    return result


def require_int(data: dict[str, Any], key: str) -> int:
    """读取整数字段，bool 不算合法整数。"""
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise PacketDecodeError(f"{key} 必须是整数")
    return int(value)


def make_zero_action(seq: int, valid_for_ms: int = 100, stamp_ms: int | None = None) -> PolicyActionPacket:
    """构造四维零速度动作；action_type沿用Orin兼容字段。"""
    return PolicyActionPacket(
        seq=seq,
        stamp_ms=now_ms() if stamp_ms is None else int(stamp_ms),
        action=[0.0, 0.0, 0.0, 0.0],
        action_type="normalized_velocity_command",
        valid_for_ms=valid_for_ms,
        action_order=ACTION_ORDER,
    )
