#!/usr/bin/env python3
"""从RRT*请求生成第一版bucket-tip避障TrajectoryCommand。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
PROJECT_ROOT = LOCALMAP_DIR.parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.io import load_json, write_json
from localmap_core.reachable_workspace import load_reachable_workspace
from localmap_core.simple_bucket_tip_planner import PlanningBounds, plan_bucket_tip_path
from localmap_core.trajectory import build_trajectory_command


DEFAULT_REQUEST = LOCALMAP_DIR / "exports" / "rrt_star_request.mock.json"
DEFAULT_PROFILE = PROJECT_ROOT / "shared" / "machine_profile.json"
DEFAULT_REACHABLE_WORKSPACE = LOCALMAP_DIR / "config" / "reachable_workspace.machine_root_ros.derived.v1.json"
DEFAULT_OUTPUT = LOCALMAP_DIR / "exports" / "trajectory_command.simple_rrt.json"


def mask_obstacles_near_points(obstacles: list[dict], points: list[np.ndarray], radius_m: float) -> list[dict]:
    """移除起点/目标附近的障碍物，用于允许bucket tip从当前位置出发并接近作业目标。"""
    if radius_m <= 0.0:
        return obstacles
    filtered = []
    for obstacle in obstacles:
        center = np.asarray(obstacle["center_m"], dtype=np.float64)
        # 关键：OctoMap会把地面/土堆也记为occupied；dig目标附近需要mask，否则规划器无法到达目标。
        if any(float(np.linalg.norm(center - point)) <= radius_m for point in points):
            continue
        filtered.append(obstacle)
    return filtered


def build_arg_parser() -> argparse.ArgumentParser:
    """构造命令行参数；第一版只规划bucket tip xyz，不做关节可达性。"""
    parser = argparse.ArgumentParser(description="从RRT*请求生成简单避障TrajectoryCommand。")
    parser.add_argument("--request", type=Path, default=DEFAULT_REQUEST, help="RRT*请求JSON")
    parser.add_argument("--machine-profile", type=Path, default=DEFAULT_PROFILE, help="机型profile路径")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="输出TrajectoryCommand JSON")
    parser.add_argument("--bounds", type=float, nargs=6, metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX", "Z_MIN", "Z_MAX"), help="规划边界，单位米，坐标系同request.frame_id")
    parser.add_argument("--reachable-workspace", type=Path, default=DEFAULT_REACHABLE_WORKSPACE, help="bucket tip可达区域JSON；默认复用shared/reachable_workspaces")
    parser.add_argument("--workspace-mode", choices=["MoveToDig", "CarryMaterial"], help="可达区域模式；默认跟task-mode一致")
    parser.add_argument("--disable-reachable-workspace", action="store_true", help="仅调试用：关闭bucket tip可达区域约束")
    parser.add_argument("--waypoint-count", type=int, default=5, help="输出waypoint数量；不改变ONNX lookahead=3")
    parser.add_argument("--collision-radius", type=float, default=0.08, help="bucket tip/铲斗简化碰撞半径，单位米")
    parser.add_argument("--step-size", type=float, default=0.20, help="RRT树扩展步长，单位米")
    parser.add_argument("--edge-check-step", type=float, default=0.04, help="边碰撞检测采样间距，单位米")
    parser.add_argument("--max-iterations", type=int, default=3000, help="最大采样次数")
    parser.add_argument("--goal-sample-rate", type=float, default=0.15, help="采样目标点的概率")
    parser.add_argument("--mask-start-radius", type=float, default=0.12, help="忽略起点附近obstacle半径，单位米")
    parser.add_argument("--mask-goal-radius", type=float, default=0.30, help="忽略目标附近obstacle半径，单位米")
    parser.add_argument("--seed", type=int, default=0, help="随机种子，便于复现实验")
    return parser


def main() -> int:
    """入口函数：request -> simple RRT避障waypoints -> TrajectoryCommand。"""
    args = build_arg_parser().parse_args()
    request = load_json(args.request)
    machine_profile = load_json(args.machine_profile)

    start = np.asarray(request["start_bucket_tip_base"], dtype=np.float64)
    goal = np.asarray(request["goal"]["position_m"], dtype=np.float64)
    bounds = PlanningBounds.from_values(args.bounds) if args.bounds else None
    workspace = None
    if not args.disable_reachable_workspace:
        workspace_mode = args.workspace_mode or request["task_mode"]
        workspace = load_reachable_workspace(args.reachable_workspace, mode=workspace_mode)
        if workspace.frame_id != request["frame_id"]:
            raise SystemExit(f"workspace frame {workspace.frame_id} 与 request frame {request['frame_id']} 不一致")
    obstacles = request.get("obstacles", [])
    obstacles = mask_obstacles_near_points(obstacles, [start], args.mask_start_radius)
    obstacles = mask_obstacles_near_points(obstacles, [goal], args.mask_goal_radius)

    result = plan_bucket_tip_path(
        start=start,
        goal=goal,
        obstacles=obstacles,
        bounds=bounds,
        collision_radius_m=args.collision_radius,
        step_size_m=args.step_size,
        edge_check_step_m=args.edge_check_step,
        max_iterations=args.max_iterations,
        goal_sample_rate=args.goal_sample_rate,
        waypoint_count=args.waypoint_count,
        seed=args.seed,
        reachable_workspace=workspace,
    )
    if not result.success:
        raise SystemExit(f"simple bucket-tip planner failed: reason={result.reason}, iterations={result.iterations}")

    pitch_targets = machine_profile["task_profile"]["bucket_pitch_targets_deg"]
    task_mode = request["task_mode"]
    command = build_trajectory_command(
        timestamp_s=request["timestamp_s"],
        frame_id=request["frame_id"],
        task_mode=task_mode,
        target_bucket_pitch_deg=float(pitch_targets[task_mode]),
        waypoints_base=result.waypoints,
        target_threshold=float(request["planning_params"]["target_threshold"]),
        tube_radius=float(request["planning_params"]["tube_radius"]),
    )
    command["planner"] = {
        "type": "simple_bucket_tip_rrt",
        "reason": result.reason,
        "iterations": result.iterations,
        "collision_radius_m": float(args.collision_radius),
        "input_obstacles": len(request.get("obstacles", [])),
        "used_obstacles": len(obstacles),
        "mask_start_radius_m": float(args.mask_start_radius),
        "mask_goal_radius_m": float(args.mask_goal_radius),
        "reachable_workspace": None
        if workspace is None
        else {
            "path": str(args.reachable_workspace),
            "mode": workspace.mode,
            "frame_id": workspace.frame_id,
            "bounds": workspace.bounds_values(),
            "anchor_count": int(workspace.anchor_points().shape[0]),
            "note": "RRT采样点、边和输出bucket-tip waypoints均受该可达体约束。",
        },
        "note": "第一版仅做bucket tip xyz避障，不做关节空间RRT*和自碰撞检查。",
    }
    write_json(args.output, command)

    print(f"request: {args.request}")
    print(f"planner: {command['planner']}")
    print(f"waypoint_count: {command['waypoint_count']}")
    print(f"target_bucket_pitch_deg: {pitch_targets[task_mode]}")
    print(f"output: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
