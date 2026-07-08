"""从预处理点云生成第一版LocalMap JSON。"""

from __future__ import annotations

from typing import Any

import numpy as np

from .geometry import Transform, up_axis_index, up_axis_normal


def estimate_ground_plane(points_base: np.ndarray, up_axis: str = "y") -> dict[str, Any]:
    """估计最小地面平面；Unity/machine_root默认+Y向上。"""
    if points_base.size == 0:
        ground_height = 0.0
        confidence = 0.0
    else:
        # 关键：低分位比min更抗孤立噪点，先作为离线LocalMap的保守地面估计。
        ground_height = float(np.percentile(points_base[:, up_axis_index(up_axis)], 5.0))
        confidence = 0.5

    normal = up_axis_normal(up_axis)
    return {
        "model": {
            "type": "plane",
            "normal": normal.astype(float).tolist(),
            "offset_m": -ground_height,
        },
        "confidence": confidence,
    }


def transform_to_schema_dict(transform: Transform) -> dict[str, Any]:
    """把Transform转成schema要求的JSON字段。"""
    data = {
        "id": transform.identifier,
        "from_frame": transform.from_frame,
        "to_frame": transform.to_frame,
        "translation_m": transform.translation_m.astype(float).tolist(),
        "status": transform.status,
    }
    if transform.linear_matrix is not None:
        # 关键：显式记录非RPY轴映射，避免把右手系到Unity轴约定转换伪装成普通旋转。
        data["transform_kind"] = "axis_mapping_matrix"
        data["axis_mapping_matrix"] = transform.linear_matrix.astype(float).tolist()
        data["matrix_determinant"] = float(np.linalg.det(transform.linear_matrix))
    else:
        data["transform_kind"] = "rpy"
        data["rotation_rpy_rad"] = transform.rotation_rpy_rad.astype(float).tolist()
    return data


def build_local_map(
    points_base: np.ndarray,
    timestamp_s: float,
    raw_topic: str,
    raw_frame_id: str,
    raw_point_type: str,
    bag_path: str,
    transform: Transform,
    targets: dict[str, Any],
    up_axis: str = "y",
) -> dict[str, Any]:
    """生成RRT*可消费的最小LocalMap，不把原始点云直接暴露给规划器。"""
    dig_targets = targets.get("dig_targets", [])
    dump_targets = targets.get("dump_targets", [])

    # 关键：第一版障碍物先留空，后续从点云聚类/人工标注填充，不在这里伪造障碍。
    return {
        "schema_version": "local_map.v1",
        "timestamp_s": float(timestamp_s),
        "frame_id": transform.to_frame,
        "source": {
            "raw_topic": raw_topic,
            "raw_frame_id": raw_frame_id,
            "raw_point_type": raw_point_type,
            "bag_path": bag_path,
            "machine_root": transform.to_frame,
            "extrinsics": transform_to_schema_dict(transform),
        },
        "ground": estimate_ground_plane(points_base, up_axis=up_axis),
        "obstacles": [],
        "dig_targets": dig_targets,
        "dump_targets": dump_targets,
        "rrt_star_hint": {
            "planning_frame_id": transform.to_frame,
            "preferred_dig_target_id": dig_targets[0]["id"] if dig_targets else "",
            "preferred_dump_target_id": dump_targets[0]["id"] if dump_targets else "",
        },
        "notes": [
            "由AiryLidar离线点云生成；当前障碍物为空，dig/dump target来自配置。",
            "RRT*应消费此LocalMap，不应直接消费原始/rslidar_points。",
        ],
    }
