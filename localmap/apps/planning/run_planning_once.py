#!/usr/bin/env python3
"""从 live LocalMap 和 bucket tip 生成一次可供策略消费的轨迹产物。"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass, replace
from pathlib import Path
from collections.abc import Mapping


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
ROS_PYTHON = Path("/usr/bin/python3")
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.planning_inputs import LivePlanningInputs, load_live_planning_inputs
from localmap_core.planning_intent import PlanningIntent, resolve_planning_intent
from localmap_core.planning_profile import (
    DEFAULT_PLANNING_PROFILE,
    PlanningOutputs,
    PlanningProfile,
    load_planning_profile,
)


@dataclass(frozen=True)
class PlanningCommand:
    name: str
    argv: tuple[str, ...]


@dataclass(frozen=True)
class PreparedPlanningRun:
    commands: tuple[PlanningCommand, ...]
    snapshot: LivePlanningInputs
    local_map_snapshot: Path
    bucket_tip_snapshot: Path
    staging_outputs: PlanningOutputs
    final_outputs: PlanningOutputs


def build_arg_parser() -> argparse.ArgumentParser:
    """现场只需选择目标；稳定算法参数由 Planning Profile 提供。"""
    parser = argparse.ArgumentParser(description="从live感知输入生成一次bucket-tip规划轨迹。")
    parser.add_argument("target_id", help="LocalMap中唯一的dig或dump目标ID")
    parser.add_argument("--profile", type=Path, default=DEFAULT_PLANNING_PROFILE, help="Planning Profile JSON")
    parser.add_argument("--dry-run", action="store_true", help="验证输入并打印内部步骤，不执行规划")
    return parser


def build_planning_commands(
    profile: PlanningProfile,
    intent: PlanningIntent,
    *,
    python: Path,
) -> tuple[PlanningCommand, ...]:
    """把一个 Profile 和任务意图编译为四个内部 Adapter 调用。"""
    apps_dir = LOCALMAP_DIR / "apps"
    inputs = profile.inputs
    outputs = profile.outputs
    obstacle = profile.obstacle_adapter
    planner = profile.planner

    obstacles = (
        str(python),
        str(apps_dir / "perception" / "export_octomap_markers_to_local_map.py"),
        "--topic",
        inputs.octomap_topic,
        "--base-local-map",
        str(inputs.live_local_map),
        "--output",
        str(outputs.local_map),
        "--expected-frame",
        profile.expected_frame,
        "--timeout-s",
        str(profile.freshness.octomap_timeout_s),
        "--box-size",
        str(obstacle.box_size_m),
        "--max-obstacles",
        str(obstacle.max_obstacles),
        "--bounds",
        *(str(value) for value in obstacle.bounds),
    )
    request = (
        str(python),
        str(apps_dir / "planning" / "generate_rrt_request_from_local_map.py"),
        "--local-map",
        str(outputs.local_map),
        "--machine-profile",
        str(inputs.machine_profile),
        "--bucket-tip",
        str(inputs.live_bucket_tip),
        "--target-id",
        intent.target_id,
        "--target-kind",
        intent.target_kind,
        "--task-mode",
        intent.task_mode,
        "--output",
        str(outputs.request),
    )
    trajectory = (
        str(python),
        str(apps_dir / "planning" / "generate_simple_rrt_trajectory_from_request.py"),
        "--request",
        str(outputs.request),
        "--machine-profile",
        str(inputs.machine_profile),
        "--output",
        str(outputs.trajectory),
        "--bounds",
        *(str(value) for value in planner.bounds),
        "--reachable-workspace",
        str(inputs.reachable_workspace),
        "--workspace-mode",
        intent.task_mode,
        "--waypoint-count",
        str(planner.waypoint_count),
        "--collision-radius",
        str(planner.collision_radius_m),
        "--step-size",
        str(planner.step_size_m),
        "--edge-check-step",
        str(planner.edge_check_step_m),
        "--max-iterations",
        str(planner.max_iterations),
        "--goal-sample-rate",
        str(planner.goal_sample_rate),
        "--mask-start-radius",
        str(planner.start_mask_radius_m),
        "--mask-goal-radius",
        str(planner.goal_mask_radius_m),
        "--seed",
        str(planner.seed),
    )
    observation = (
        str(python),
        str(apps_dir / "planning" / "generate_observation_waypoint_slice.py"),
        "--trajectory",
        str(outputs.trajectory),
        "--bucket-tip",
        str(inputs.live_bucket_tip),
        "--machine-profile",
        str(inputs.machine_profile),
        "--current-index",
        "0",
        "--output",
        str(outputs.observation_slice),
    )
    return (
        PlanningCommand("obstacles", obstacles),
        PlanningCommand("request", request),
        PlanningCommand("trajectory", trajectory),
        PlanningCommand("observation", observation),
    )


def prepare_planning_run(
    profile: PlanningProfile,
    target_id: str,
    *,
    now_s: float,
    python: Path,
    staging_dir: Path,
) -> PreparedPlanningRun:
    """冻结一次 live 快照，并让四步命令只消费 staging 路径。"""
    live_inputs = load_live_planning_inputs(profile, now_s=now_s)
    intent = resolve_planning_intent(
        live_inputs.local_map,
        target_id,
        profile.task_mode_by_target_kind,
    )
    local_map_snapshot = staging_dir / "local_map.snapshot.json"
    bucket_tip_snapshot = staging_dir / "bucket_tip.snapshot.json"
    staging_outputs = replace(
        profile.outputs,
        directory=staging_dir,
        local_map=staging_dir / profile.outputs.local_map.name,
        request=staging_dir / profile.outputs.request.name,
        trajectory=staging_dir / profile.outputs.trajectory.name,
        observation_slice=staging_dir / profile.outputs.observation_slice.name,
    )
    staged_profile = replace(
        profile,
        inputs=replace(
            profile.inputs,
            live_local_map=local_map_snapshot,
            live_bucket_tip=bucket_tip_snapshot,
        ),
        outputs=staging_outputs,
    )
    return PreparedPlanningRun(
        commands=build_planning_commands(staged_profile, intent, python=python),
        snapshot=live_inputs,
        local_map_snapshot=local_map_snapshot,
        bucket_tip_snapshot=bucket_tip_snapshot,
        staging_outputs=staging_outputs,
        final_outputs=profile.outputs,
    )


def execute_planning_commands(
    commands: tuple[PlanningCommand, ...],
    *,
    dry_run: bool,
) -> None:
    """顺序执行内部步骤；dry-run 只打印，不启动子进程。"""
    for command in commands:
        print(f"[{command.name}] {shlex.join(command.argv)}", flush=True)
        if not dry_run:
            subprocess.run(list(command.argv), check=True)


def _thaw_json(value):
    if isinstance(value, Mapping):
        return {key: _thaw_json(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw_json(item) for item in value]
    return value


def _write_snapshot(path: Path, value: Mapping) -> None:
    path.write_text(
        json.dumps(_thaw_json(value), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def execute_prepared_run(prepared: PreparedPlanningRun, *, dry_run: bool) -> None:
    """执行同一快照的四步规划；全部成功后才原子发布产物。"""
    if dry_run:
        execute_planning_commands(prepared.commands, dry_run=True)
        return

    prepared.staging_outputs.directory.mkdir(parents=True, exist_ok=True)
    _write_snapshot(prepared.local_map_snapshot, prepared.snapshot.local_map)
    _write_snapshot(prepared.bucket_tip_snapshot, prepared.snapshot.bucket_tip)
    execute_planning_commands(prepared.commands, dry_run=False)

    staging_paths = (
        prepared.staging_outputs.local_map,
        prepared.staging_outputs.request,
        prepared.staging_outputs.trajectory,
        prepared.staging_outputs.observation_slice,
    )
    for path in staging_paths:
        if not path.is_file():
            raise ValueError(f"planning step 未生成预期产物: {path}")

    prepared.final_outputs.directory.mkdir(parents=True, exist_ok=True)
    publish_pairs = (
        (prepared.staging_outputs.local_map, prepared.final_outputs.local_map),
        (prepared.staging_outputs.request, prepared.final_outputs.request),
        (prepared.staging_outputs.trajectory, prepared.final_outputs.trajectory),
        (
            prepared.staging_outputs.observation_slice,
            prepared.final_outputs.observation_slice,
        ),
    )
    for source, destination in publish_pairs:
        os.replace(source, destination)


def require_ros_python(python: Path = ROS_PYTHON) -> None:
    """确认内部 Adapter 使用的解释器能加载 ROS2 与 NumPy。"""
    if not python.is_file():
        raise ValueError(f"ROS Python 不存在: {python}")
    subprocess.run(
        [str(python), "-c", "import rclpy, numpy"],
        check=True,
        capture_output=True,
        text=True,
    )


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        profile = load_planning_profile(args.profile)
        require_ros_python()
        for name, path in (
            ("machine_profile", profile.inputs.machine_profile),
            ("reachable_workspace", profile.inputs.reachable_workspace),
        ):
            if not path.is_file():
                raise ValueError(f"planning input {name} 不存在: {path}")
        if args.dry_run:
            prepared = prepare_planning_run(
                profile,
                args.target_id,
                now_s=time.time(),
                python=ROS_PYTHON,
                staging_dir=profile.outputs.directory / ".planning-dry-run",
            )
            execute_prepared_run(prepared, dry_run=True)
        else:
            profile.outputs.directory.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(
                prefix=".planning-",
                dir=profile.outputs.directory,
            ) as staging_directory:
                prepared = prepare_planning_run(
                    profile,
                    args.target_id,
                    now_s=time.time(),
                    python=ROS_PYTHON,
                    staging_dir=Path(staging_directory),
                )
                execute_prepared_run(prepared, dry_run=False)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"planning failed: {exc}", file=sys.stderr, flush=True)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
