"""machine_root_ros坐标系下的bucket tip可达区域工具。

Unity训练侧用20个anchor点定义bucket tip可达体；这里读取同一份JSON，
用于约束真实感知链路中的RRT bucket tip waypoint。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .io import load_json


ANCHORS_PER_SECTION = 4
TETRA_INDICES = (
    (0, 1, 3, 7),
    (0, 3, 2, 7),
    (0, 2, 6, 7),
    (0, 6, 4, 7),
    (0, 4, 5, 7),
    (0, 5, 1, 7),
)


@dataclass(frozen=True)
class WorkspaceTetra:
    """可达体中的一个四面体，便于inside检查和均匀采样。"""

    vertices: np.ndarray
    volume: float


@dataclass(frozen=True)
class ReachableWorkspace:
    """bucket tip可达区域；所有点都在machine_root_ros坐标系下。"""

    machine_id: str
    mode: str
    frame_id: str
    sections: np.ndarray
    tetras: tuple[WorkspaceTetra, ...]
    minimum: np.ndarray
    maximum: np.ndarray

    def contains(self, point: np.ndarray, epsilon: float = 1e-8) -> bool:
        """检查bucket tip点是否落在Unity导出的可达体内。"""
        point = np.asarray(point, dtype=np.float64)
        if bool(np.any(point < self.minimum - epsilon) or np.any(point > self.maximum + epsilon)):
            return False
        return any(point_in_tetra(point, tetra.vertices, epsilon=epsilon) for tetra in self.tetras)

    def segment_inside(self, start: np.ndarray, end: np.ndarray, step_m: float = 0.04) -> bool:
        """沿线段采样检查整条bucket tip边是否都在可达区域内。"""
        start = np.asarray(start, dtype=np.float64)
        end = np.asarray(end, dtype=np.float64)
        length = float(np.linalg.norm(end - start))
        sample_count = max(2, int(np.ceil(length / max(step_m, 1e-4))) + 1)
        for ratio in np.linspace(0.0, 1.0, sample_count):
            # 关键：RRT边不只检查障碍物，也要检查bucket tip是否始终处于可达体内。
            if not self.contains(start * (1.0 - ratio) + end * ratio):
                return False
        return True

    def sample(self, rng: np.random.Generator) -> np.ndarray:
        """按四面体体积采样一个可达区域内的bucket tip点。"""
        volumes = np.asarray([tetra.volume for tetra in self.tetras], dtype=np.float64)
        probabilities = volumes / float(np.sum(volumes))
        tetra = self.tetras[int(rng.choice(len(self.tetras), p=probabilities))]
        weights = rng.exponential(1.0, 4)
        weights = weights / np.sum(weights)
        return weights @ tetra.vertices

    def bounds_values(self) -> list[float]:
        """返回命令行友好的xmin/xmax/ymin/ymax/zmin/zmax。"""
        return [
            float(self.minimum[0]),
            float(self.maximum[0]),
            float(self.minimum[1]),
            float(self.maximum[1]),
            float(self.minimum[2]),
            float(self.maximum[2]),
        ]

    def anchor_points(self) -> np.ndarray:
        """返回N x 3 anchor点，供RViz marker和调试输出使用。"""
        return self.sections.reshape(-1, 3)


def parse_anchor_points(anchor_points: list[dict[str, Any]]) -> np.ndarray:
    """把JSON中的anchor字典解析成section x corner x xyz数组。"""
    if len(anchor_points) < ANCHORS_PER_SECTION * 2 or len(anchor_points) % ANCHORS_PER_SECTION != 0:
        raise ValueError("volume_anchor_points必须是至少2个截面，且每截面4个点")

    parsed = np.asarray(
        [[float(point["x"]), float(point["y"]), float(point["z"])] for point in anchor_points],
        dtype=np.float64,
    )
    return parsed.reshape(-1, ANCHORS_PER_SECTION, 3)


def tetra_volume(vertices: np.ndarray) -> float:
    """计算四面体体积；退化四面体会被丢弃。"""
    matrix = np.column_stack((vertices[1] - vertices[0], vertices[2] - vertices[0], vertices[3] - vertices[0]))
    return abs(float(np.linalg.det(matrix))) / 6.0


def build_tetras(sections: np.ndarray) -> tuple[WorkspaceTetra, ...]:
    """把相邻4点截面拼成四面体集合，用于可达体inside判断。"""
    tetras: list[WorkspaceTetra] = []
    for section_index in range(sections.shape[0] - 1):
        # 顶点顺序：当前截面4点 + 下一截面4点；每4点顺序沿用Unity导出的anchor定义。
        cell_vertices = np.vstack((sections[section_index], sections[section_index + 1]))
        for tetra_indices in TETRA_INDICES:
            vertices = cell_vertices[list(tetra_indices)]
            volume = tetra_volume(vertices)
            if volume > 1e-10:
                tetras.append(WorkspaceTetra(vertices=vertices, volume=volume))
    if not tetras:
        raise ValueError("可达区域anchor退化，无法构造有效四面体")
    return tuple(tetras)


def point_in_tetra(point: np.ndarray, vertices: np.ndarray, epsilon: float = 1e-8) -> bool:
    """用重心坐标判断点是否位于四面体内。"""
    matrix = np.column_stack((vertices[1] - vertices[0], vertices[2] - vertices[0], vertices[3] - vertices[0]))
    try:
        weights_123 = np.linalg.solve(matrix, point - vertices[0])
    except np.linalg.LinAlgError:
        return False
    weights = np.array(
        [1.0 - float(np.sum(weights_123)), float(weights_123[0]), float(weights_123[1]), float(weights_123[2])],
        dtype=np.float64,
    )
    return bool(np.all(weights >= -epsilon) and np.all(weights <= 1.0 + epsilon))


def load_reachable_workspace(path: Path, mode: str = "MoveToDig") -> ReachableWorkspace:
    """读取右手 ROS 规划所用的指定任务模式可达区域。"""
    data = load_json(path)
    frame_id = str(data["coordinate_frame"])
    if frame_id != "machine_root_ros":
        raise ValueError(f"当前只接受machine_root_ros可达区域，实际为: {frame_id}")

    for workspace in data["workspaces"]:
        if workspace["mode"] != mode:
            continue
        sections = parse_anchor_points(workspace["volume_anchor_points"])
        anchors = sections.reshape(-1, 3)
        return ReachableWorkspace(
            machine_id=str(data["machine_id"]),
            mode=mode,
            frame_id=frame_id,
            sections=sections,
            tetras=build_tetras(sections),
            minimum=np.min(anchors, axis=0),
            maximum=np.max(anchors, axis=0),
        )
    raise ValueError(f"找不到可达区域模式: {mode}")
