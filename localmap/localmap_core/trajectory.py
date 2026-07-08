"""RRT*输入、轨迹命令和38维observation中waypoint切片的契约工具。"""

from __future__ import annotations

from typing import Any

import numpy as np


WAYPOINT_OBS_START = 15
WAYPOINT_OBS_END_EXCLUSIVE = 27


def get_observation_normalizers(machine_profile: dict[str, Any]) -> dict[str, float]:
    """从machine_profile读取observation归一化常数，避免散落硬编码。"""
    return machine_profile["observation_schema"]["normalizers"]


def get_waypoint_lookahead(machine_profile: dict[str, Any]) -> int:
    """从machine_profile读取策略前视waypoint数量。"""
    return int(machine_profile["observation_schema"]["waypoint_lookahead"])


def select_target(local_map: dict[str, Any], target_id: str, target_kind: str) -> dict[str, Any]:
    """按id从LocalMap选择dig或dump目标。"""
    collection_name = {"dig": "dig_targets", "dump": "dump_targets"}[target_kind]
    for target in local_map[collection_name]:
        if target["id"] == target_id:
            return target
    raise ValueError(f"LocalMap中找不到{target_kind} target: {target_id}")


def build_rrt_star_request(
    local_map: dict[str, Any],
    machine_profile: dict[str, Any],
    bucket_tip_base: np.ndarray,
    target_id: str,
    target_kind: str,
    task_mode: str,
) -> dict[str, Any]:
    """把LocalMap整理成RRT*规划请求；不把原始点云暴露给RRT*。"""
    normalizers = get_observation_normalizers(machine_profile)
    target = select_target(local_map, target_id=target_id, target_kind=target_kind)

    # 关键：RRT*输入只包含语义地图、bucket tip起点和目标，不包含/rslidar_points原始点云。
    return {
        "schema_version": "rrt_star_request.v1",
        "timestamp_s": float(local_map["timestamp_s"]),
        "frame_id": local_map["frame_id"],
        "task_mode": task_mode,
        "start_bucket_tip_base": bucket_tip_base.astype(float).tolist(),
        "goal": {
            "id": target["id"],
            "kind": target_kind,
            "position_m": target["position_m"],
            "normal": target["normal"],
            "radius_m": target["radius_m"],
            "confidence": target["confidence"],
        },
        "ground": local_map["ground"],
        "obstacles": local_map["obstacles"],
        "planning_params": {
            "waypoint_lookahead": get_waypoint_lookahead(machine_profile),
            "target_threshold": float(normalizers["target_threshold"]),
            "tube_radius": float(normalizers["tube_radius"]),
        },
        "notes": [
            "RRT*请求来自LocalMap语义层，不直接消费原始点云。",
            "输出应为bucket_tip在同一frame_id下的waypoints_base。",
        ],
    }


def build_trajectory_command(
    timestamp_s: float,
    frame_id: str,
    task_mode: str,
    target_bucket_pitch_deg: float,
    waypoints_base: np.ndarray,
    target_threshold: float,
    tube_radius: float,
) -> dict[str, Any]:
    """构造RL tracker可消费的TrajectoryCommand。"""
    if waypoints_base.ndim != 2 or waypoints_base.shape[1] != 3:
        raise ValueError("waypoints_base必须是N x 3矩阵")

    # 关键：waypoints_base必须和bucket_tip_base处于同一个frame_id，后续observation只算相对误差。
    return {
        "schema_version": "trajectory_command.v1",
        "timestamp_s": float(timestamp_s),
        "frame_id": frame_id,
        "task_mode": task_mode,
        "target_bucket_pitch_deg": float(target_bucket_pitch_deg),
        "waypoints_base": waypoints_base.astype(float).tolist(),
        "waypoint_count": int(waypoints_base.shape[0]),
        "target_threshold": float(target_threshold),
        "tube_radius": float(tube_radius),
    }


def interpolate_waypoints(start: np.ndarray, goal: np.ndarray, waypoint_count: int) -> np.ndarray:
    """生成mock直线waypoints；仅用于打通接口，不替代RRT*。"""
    if waypoint_count < 1:
        raise ValueError("waypoint_count必须>=1")

    # 关键：真实避障/可达性由RRT*负责；这里的直线插值只服务离线契约联调。
    ratios = np.linspace(0.0, 1.0, waypoint_count, dtype=np.float64).reshape(-1, 1)
    return start.reshape(1, 3) * (1.0 - ratios) + goal.reshape(1, 3) * ratios


def lookahead_waypoints(
    waypoints: np.ndarray,
    current_waypoint_index: int,
    lookahead: int,
) -> np.ndarray:
    """提取未来lookahead个waypoint；越界时重复最后一个，保持observation维度固定。"""
    if waypoints.shape[0] == 0:
        raise ValueError("trajectory_command至少需要1个waypoint")

    selected = []
    last_index = waypoints.shape[0] - 1
    for offset in range(lookahead):
        index = min(current_waypoint_index + offset, last_index)
        selected.append(waypoints[index])
    return np.asarray(selected, dtype=np.float64)


def point_to_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
    """计算点到当前轨迹段的距离，用于tube_signed的第一版近似。"""
    segment = end - start
    length_sq = float(np.dot(segment, segment))
    if length_sq <= 1e-12:
        return float(np.linalg.norm(point - start))
    t = float(np.clip(np.dot(point - start, segment) / length_sq, 0.0, 1.0))
    projection = start + t * segment
    return float(np.linalg.norm(point - projection))


def compute_tube_signed(
    waypoints: np.ndarray,
    bucket_tip_base: np.ndarray,
    current_waypoint_index: int,
    tube_radius: float,
) -> float:
    """计算归一化tube偏离量；第一版用无符号距离，落在tube内返回0附近。"""
    if waypoints.shape[0] < 2:
        return 0.0

    start_index = max(current_waypoint_index - 1, 0)
    end_index = min(current_waypoint_index, waypoints.shape[0] - 1)
    if start_index == end_index:
        end_index = min(start_index + 1, waypoints.shape[0] - 1)

    distance = point_to_segment_distance(bucket_tip_base, waypoints[start_index], waypoints[end_index])
    if tube_radius <= 0.0:
        return 1.0

    # 关键：当前没有左右符号定义，先输出超出tube的比例；在tube内为0，超出后裁剪到1。
    outside_distance = max(0.0, distance - tube_radius)
    return float(np.clip(outside_distance / tube_radius, 0.0, 1.0))


def build_waypoint_observation_slice(
    trajectory_command: dict[str, Any],
    machine_profile: dict[str, Any],
    bucket_tip_base: np.ndarray,
    current_waypoint_index: int,
) -> dict[str, Any]:
    """构造38维observation中idx 15..26的waypoint相关切片。"""
    lookahead = get_waypoint_lookahead(machine_profile)
    normalizers = get_observation_normalizers(machine_profile)
    distance_normalizer = float(normalizers["distance_normalizer"])
    tube_radius = float(trajectory_command.get("tube_radius", normalizers["tube_radius"]))
    waypoints = np.asarray(trajectory_command["waypoints_base"], dtype=np.float64)
    waypoint_count = int(trajectory_command["waypoint_count"])

    future_waypoints = lookahead_waypoints(waypoints, current_waypoint_index, lookahead)

    # 关键：ONNX仍是38维；这里只计算idx 15..23的未来3个waypoint相对tip误差。
    errors = (future_waypoints - bucket_tip_base.reshape(1, 3)) / distance_normalizer
    progress = float(np.clip(current_waypoint_index / max(waypoint_count, 1), 0.0, 1.0))
    tube_signed = compute_tube_signed(waypoints, bucket_tip_base, current_waypoint_index, tube_radius)
    is_final = 1.0 if current_waypoint_index >= waypoint_count - 1 else 0.0

    values = errors.reshape(-1).astype(float).tolist()
    values.extend([progress, tube_signed, is_final])

    return {
        "schema_version": "observation_waypoint_slice.v1",
        "frame_id": trajectory_command["frame_id"],
        "indices": list(range(WAYPOINT_OBS_START, WAYPOINT_OBS_END_EXCLUSIVE)),
        "values": values,
        "current_waypoint_index": int(current_waypoint_index),
        "waypoint_count": waypoint_count,
        "notes": [
            "该切片只覆盖38维observation的idx 15..26。",
            "完整ONNX observation仍必须由部署侧状态估计器按权威38维表组装。",
        ],
    }
