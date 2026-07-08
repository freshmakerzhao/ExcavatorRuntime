"""把OctoMap可视化占据点转换成LocalMap obstacle的适配工具。"""

from __future__ import annotations

from typing import Any

import numpy as np


def bounds_mask_xyz(points: np.ndarray, bounds: dict[str, list[float]] | None) -> np.ndarray:
    """在目标frame下裁剪XYZ点，避免把远处墙体/支架放进RRT*。"""
    if bounds is None:
        return np.ones(points.shape[0], dtype=bool)
    mask = np.ones(points.shape[0], dtype=bool)
    for index, axis in enumerate(("x", "y", "z")):
        if axis in bounds:
            lower, upper = bounds[axis]
            mask &= (points[:, index] >= float(lower)) & (points[:, index] <= float(upper))
    return mask


def centers_to_obstacle_boxes(
    centers: np.ndarray,
    box_size_m: float = 0.15,
    bounds: dict[str, list[float]] | None = None,
    max_obstacles: int = 2000,
    source: str = "lidar",
) -> list[dict[str, Any]]:
    """把occupied voxel中心点粗化为LocalMap box obstacles。

    OctoMap marker里可能有几万到十几万个小体素。第一版先按固定网格粗化，
    保留“哪里被占据”的形状趋势，而不是把每个体素都暴露给RRT*。
    """
    if centers.size == 0:
        return []
    points = np.asarray(centers, dtype=np.float64).reshape(-1, 3)
    points = points[bounds_mask_xyz(points, bounds)]
    if points.size == 0:
        return []

    resolution = float(box_size_m)
    keys = np.floor(points / resolution).astype(np.int64)
    unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)

    obstacles = []
    for index, key in enumerate(unique_keys[:max_obstacles]):
        members = points[inverse == index]
        # 关键：用落在同一粗网格内的点均值作为box中心，比直接用key中心更贴近现场点云。
        center = np.mean(members, axis=0)
        obstacles.append(
            {
                "id": f"octomap_box_{index:05d}",
                "shape": "box",
                "center_m": center.astype(float).tolist(),
                "size_m": [resolution, resolution, resolution],
                "confidence": 0.6,
                "source": source,
            }
        )
    return obstacles


def parse_bounds(values: list[float] | None) -> dict[str, list[float]] | None:
    """把命令行bounds整理成x/y/z字典。"""
    if values is None:
        return None
    x_min, x_max, y_min, y_max, z_min, z_max = values
    return {"x": [x_min, x_max], "y": [y_min, y_max], "z": [z_min, z_max]}
