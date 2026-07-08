"""TF bucket tip到machine_root bucket tip的桥接工具。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .io import load_json


@dataclass(frozen=True)
class BucketTipFrameBridge:
    """从运动学TF坐标系到AiryLidar规划坐标系的固定桥接关系。"""

    source_frame: str
    target_frame: str
    translation_m: np.ndarray
    axis_mapping_matrix: np.ndarray
    identifier: str
    status: str

    def transform_position(self, source_position_m: np.ndarray) -> np.ndarray:
        """把source_frame下的bucket tip位置转换到target_frame。"""
        source_position_m = np.asarray(source_position_m, dtype=np.float64).reshape(3)
        # 关键：先做坐标轴约定转换，再加原点补偿；默认假设base_link原点等价machine_root。
        return self.axis_mapping_matrix @ source_position_m + self.translation_m

    def to_dict(self) -> dict[str, Any]:
        """输出可记录到JSON里的桥接元数据。"""
        return {
            "id": self.identifier,
            "source_frame": self.source_frame,
            "target_frame": self.target_frame,
            "translation_m": self.translation_m.astype(float).tolist(),
            "axis_mapping_matrix": self.axis_mapping_matrix.astype(float).tolist(),
            "status": self.status,
        }


def load_bucket_tip_frame_bridge(path: Path) -> BucketTipFrameBridge:
    """读取bucket tip坐标桥接配置。"""
    data = load_json(path)
    matrix = np.asarray(data["axis_mapping_matrix"], dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError("axis_mapping_matrix必须是3x3矩阵")
    return BucketTipFrameBridge(
        source_frame=str(data["source_frame"]),
        target_frame=str(data["target_frame"]),
        translation_m=np.asarray(data.get("translation_m", [0.0, 0.0, 0.0]), dtype=np.float64),
        axis_mapping_matrix=matrix,
        identifier=str(data["id"]),
        status=str(data["status"]),
    )


def build_bucket_tip_state(
    position_m: np.ndarray,
    frame_id: str,
    stamp_s: float,
    source_topic: str,
    bridge: BucketTipFrameBridge,
) -> dict[str, Any]:
    """构造run_planning_once.sh可直接读取的bucket tip JSON。"""
    return {
        "frame_id": frame_id,
        "position_m": np.asarray(position_m, dtype=np.float64).astype(float).tolist(),
        "stamp_s": float(stamp_s),
        "status": "live_from_tf",
        "source": {
            "topic": source_topic,
            "bridge": bridge.to_dict(),
        },
        "notes": [
            "由TF bucket tip bridge生成，供RRT规划起点使用。",
            "当前JSON只把position_m作为权威输入；姿态后续可扩展进入轨迹/observation链路。",
        ],
    }
