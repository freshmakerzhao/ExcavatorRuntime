"""点云坐标变换和最小预处理。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Transform:
    """表示从雷达坐标系到目标坐标系的刚体外参。"""

    from_frame: str
    to_frame: str
    translation_m: np.ndarray
    rotation_rpy_rad: np.ndarray
    identifier: str
    status: str
    linear_matrix: np.ndarray | None = None


def rpy_to_rotation_matrix(rotation_rpy_rad: np.ndarray) -> np.ndarray:
    """把roll/pitch/yaw转成旋转矩阵，约定为Rz(yaw) * Ry(pitch) * Rx(roll)。"""
    roll, pitch, yaw = rotation_rpy_rad.astype(np.float64)

    # 关键：显式写出三轴旋转，避免后续外参方向排查时出现隐藏约定。
    cos_r, sin_r = np.cos(roll), np.sin(roll)
    cos_p, sin_p = np.cos(pitch), np.sin(pitch)
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)

    rot_x = np.array(
        [[1.0, 0.0, 0.0], [0.0, cos_r, -sin_r], [0.0, sin_r, cos_r]],
        dtype=np.float64,
    )
    rot_y = np.array(
        [[cos_p, 0.0, sin_p], [0.0, 1.0, 0.0], [-sin_p, 0.0, cos_p]],
        dtype=np.float64,
    )
    rot_z = np.array(
        [[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return rot_z @ rot_y @ rot_x


def transform_xyzirt_points(points: np.ndarray, transform: Transform) -> np.ndarray:
    """把XYZIRT点云从transform.from_frame变换到transform.to_frame。"""
    if points.ndim != 2 or points.shape[1] != 6:
        raise ValueError("points必须是N x 6矩阵，列顺序为x,y,z,intensity,ring,timestamp")

    # 关键：Unity风格machine_root可能需要右手雷达系到Unity轴约定的显式轴映射矩阵。
    linear = transform.linear_matrix if transform.linear_matrix is not None else rpy_to_rotation_matrix(transform.rotation_rpy_rad)
    transformed = points.astype(np.float64, copy=True)

    # 关键：只变换XYZ，intensity/ring/timestamp保持原样供后续调试和特征提取使用。
    transformed[:, 0:3] = points[:, 0:3] @ linear.T + transform.translation_m
    return transformed


def up_axis_normal(up_axis: str) -> np.ndarray:
    """返回目标坐标系的上方向单位法向量。"""
    axis_to_normal = {
        "x": np.array([1.0, 0.0, 0.0], dtype=np.float64),
        "y": np.array([0.0, 1.0, 0.0], dtype=np.float64),
        "z": np.array([0.0, 0.0, 1.0], dtype=np.float64),
    }
    if up_axis not in axis_to_normal:
        raise ValueError("up_axis必须是x/y/z之一")
    return axis_to_normal[up_axis]


def up_axis_index(up_axis: str) -> int:
    """返回目标坐标系上方向对应的列索引。"""
    axis_to_index = {"x": 0, "y": 1, "z": 2}
    if up_axis not in axis_to_index:
        raise ValueError("up_axis必须是x/y/z之一")
    return axis_to_index[up_axis]


def finite_xyz_mask(points: np.ndarray) -> np.ndarray:
    """返回XYZ均为有限数值的点；RoboSense点云中NaN点槽需要在这里过滤。"""
    return np.all(np.isfinite(points[:, 0:3]), axis=1)


def bounds_mask(points: np.ndarray, bounds: dict[str, list[float]] | None) -> np.ndarray:
    """按可选XYZ范围裁剪点云；未提供bounds时保留所有有限点。"""
    mask = np.ones(points.shape[0], dtype=bool)
    if not bounds:
        return mask

    axis_to_index = {"x": 0, "y": 1, "z": 2}
    for axis, index in axis_to_index.items():
        if axis not in bounds:
            continue
        lower, upper = bounds[axis]
        # 关键：裁剪范围是感知配置，不是机型常数；真实值后续从配置/标定传入。
        mask &= (points[:, index] >= lower) & (points[:, index] <= upper)
    return mask


def preprocess_points(
    points: np.ndarray,
    transform: Transform,
    bounds: dict[str, list[float]] | None = None,
) -> np.ndarray:
    """过滤无效点、完成外参变换，并按目标坐标系裁剪工作区域。"""
    valid_raw = points[finite_xyz_mask(points)]
    transformed = transform_xyzirt_points(valid_raw, transform)
    return transformed[bounds_mask(transformed, bounds)]
