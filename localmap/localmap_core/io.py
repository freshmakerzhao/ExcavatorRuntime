"""LocalMap离线工具的JSON和NPZ读写。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from .geometry import Transform


def load_json(path: Path) -> dict[str, Any]:
    """读取JSON配置；统一入口方便后续加入schema校验。"""
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def write_json(path: Path, data: dict[str, Any]) -> None:
    """写入格式稳定的JSON，方便git diff和人工审阅。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(data, file, ensure_ascii=False, indent=2)
        file.write("\n")


def load_extrinsics(path: Path) -> Transform:
    """从JSON读取rslidar到machine_root/MachineRoot的外参。"""
    data = load_json(path)
    rotation_rpy_rad = data.get("rotation_rpy_rad")
    if rotation_rpy_rad is None and "rotation_rpy_deg" in data:
        # 关键：现场手工测量通常用角度记录，内部统一转成弧度参与矩阵计算。
        rotation_rpy_rad = np.deg2rad(np.array(data["rotation_rpy_deg"], dtype=np.float64)).tolist()
    if rotation_rpy_rad is None:
        rotation_rpy_rad = [0.0, 0.0, 0.0]

    linear_matrix = None
    if "axis_mapping_matrix" in data:
        # 关键：axis_mapping_matrix允许表达右手雷达系到Unity风格machine_root的轴约定转换。
        linear_matrix = np.array(data["axis_mapping_matrix"], dtype=np.float64)
        if linear_matrix.shape != (3, 3):
            raise ValueError("axis_mapping_matrix必须是3x3矩阵")

    return Transform(
        from_frame=str(data["from_frame"]),
        to_frame=str(data["to_frame"]),
        translation_m=np.array(data["translation_m"], dtype=np.float64),
        rotation_rpy_rad=np.array(rotation_rpy_rad, dtype=np.float64),
        identifier=str(data["id"]),
        status=str(data["status"]),
        linear_matrix=linear_matrix,
    )


def load_npz_points(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    """读取export_first_cloud.py导出的NPZ点云和元数据。"""
    with np.load(path, allow_pickle=False) as data:
        points = data["points"].astype(np.float64)
        metadata = {
            "frame_id": str(data["frame_id"]),
            "stamp_sec": int(data["stamp_sec"]),
            "stamp_nanosec": int(data["stamp_nanosec"]),
            "bag_time_ns": int(data["bag_time_ns"]),
            "columns": [str(column) for column in data["columns"]],
        }
    return points, metadata
