#!/usr/bin/env python3
"""检查当前 ExcavatorRuntime 感知和规划链路是否处于可用状态。"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any


APP_PATH = Path(__file__).resolve()
LOCALMAP_DIR = APP_PATH.parents[2]
PROJECT_ROOT = LOCALMAP_DIR.parent
DEFAULT_EXPORT_DIR = LOCALMAP_DIR / "exports" / "live_latest"
DEFAULT_MISSION = PROJECT_ROOT / "mission" / "config" / "excavation_cycle.json"


@dataclass(frozen=True)
class CheckResult:
    """单项健康检查结果。"""

    name: str
    status: str
    detail: str


def write_json_for_test(path: Path, data: dict[str, Any]) -> None:
    """测试辅助：写入稳定JSON，避免测试重复文件写入细节。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def build_arg_parser() -> argparse.ArgumentParser:
    """构造 smoke check 命令行参数。"""
    parser = argparse.ArgumentParser(description="检查雷达/LocalMap/OctoMap/RRT基础链路是否健康。")
    parser.add_argument("--expected-frame", default="machine_root_ros", help="LocalMap和规划产物应使用的frame")
    parser.add_argument("--export-dir", type=Path, default=DEFAULT_EXPORT_DIR, help="live_latest产物目录")
    parser.add_argument("--local-map-json", type=Path, help="LocalMap live JSON路径")
    parser.add_argument("--trajectory-json", type=Path, help="TrajectoryCommand JSON路径")
    parser.add_argument("--bucket-tip-json", type=Path, help="bucket tip live JSON路径")
    parser.add_argument("--skip-ros", action="store_true", help="跳过ROS topic检查，只检查JSON文件")
    parser.add_argument(
        "--run-planning-phase",
        choices=("dig", "dump"),
        help="对Mission的指定阶段运行一次非执行全局预览，再检查轨迹文件",
    )
    parser.add_argument("--mission", type=Path, default=DEFAULT_MISSION)
    parser.add_argument("--require-trajectory", action="store_true", help="缺少轨迹文件时判定为失败；默认只警告")
    parser.add_argument("--topic-timeout-s", type=float, default=5.0, help="topic hz采样超时时间")
    parser.add_argument("--min-point-rate-hz", type=float, default=1.0, help="点云topic最低期望频率")
    return parser


def load_json_file(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    """读取JSON文件；错误以字符串返回，便于上层生成统一结果。"""
    if not path.exists():
        return None, f"文件不存在: {path}"
    try:
        with path.open("r", encoding="utf-8") as file:
            return json.load(file), None
    except json.JSONDecodeError as exc:
        return None, f"JSON解析失败: {path}: {exc}"


def check_local_map_json(path: Path, expected_frame: str) -> CheckResult:
    """检查LocalMap文件存在、frame正确且包含最小字段。"""
    data, error = load_json_file(path)
    if error is not None or data is None:
        return CheckResult("LocalMap JSON", "fail", error or "未知读取错误")

    frame_id = data.get("frame_id")
    if frame_id != expected_frame:
        return CheckResult("LocalMap JSON", "fail", f"frame_id={frame_id}，期望 {expected_frame}")

    missing = [key for key in ("ground", "dig_targets", "dump_targets") if key not in data]
    if missing:
        return CheckResult("LocalMap JSON", "fail", f"缺少字段: {', '.join(missing)}")

    dig_count = len(data.get("dig_targets") or [])
    dump_count = len(data.get("dump_targets") or [])
    return CheckResult("LocalMap JSON", "pass", f"frame={frame_id}, dig_targets={dig_count}, dump_targets={dump_count}")


def check_bucket_tip_json(path: Path, expected_frame: str) -> CheckResult:
    """检查规划必需的live bucket tip JSON。"""
    data, error = load_json_file(path)
    if error is not None or data is None:
        return CheckResult("Bucket tip JSON", "fail", error or "未知读取错误")

    frame_id = data.get("frame_id")
    position = data.get("position_m")
    if frame_id != expected_frame:
        return CheckResult("Bucket tip JSON", "fail", f"frame_id={frame_id}，期望 {expected_frame}")
    if data.get("status") != "live_from_tf":
        return CheckResult(
            "Bucket tip JSON",
            "fail",
            f"status={data.get('status')!r}，期望 'live_from_tf'",
        )
    if not is_xyz_vector(position):
        return CheckResult("Bucket tip JSON", "fail", "position_m 不是3个有限数值")
    if not is_finite_number(data.get("stamp_s")):
        return CheckResult("Bucket tip JSON", "fail", "stamp_s 不是有限数值")
    return CheckResult("Bucket tip JSON", "pass", f"frame={frame_id}, position_m={format_xyz(position)}")


def check_trajectory_json(path: Path, expected_frame: str, required: bool) -> CheckResult:
    """检查规划轨迹JSON；默认缺失只警告，run_planning后应设为必需。"""
    data, error = load_json_file(path)
    if error is not None or data is None:
        return CheckResult("Trajectory JSON", "fail" if required else "warn", error or "未知读取错误")

    frame_id = data.get("frame_id")
    waypoints = data.get("waypoints_base")
    if frame_id != expected_frame:
        return CheckResult("Trajectory JSON", "fail", f"frame_id={frame_id}，期望 {expected_frame}")
    if not isinstance(waypoints, list) or not waypoints:
        return CheckResult("Trajectory JSON", "fail", "waypoints_base 为空")
    invalid_index = next((index for index, waypoint in enumerate(waypoints) if not is_xyz_vector(waypoint)), None)
    if invalid_index is not None:
        return CheckResult("Trajectory JSON", "fail", f"waypoints_base[{invalid_index}] 不是长度为3的数值数组")
    return CheckResult("Trajectory JSON", "pass", f"frame={frame_id}, {len(waypoints)} 个 waypoint")


def is_xyz_vector(value: object) -> bool:
    """判断值是否是由3个有限数值组成的数组。"""
    return isinstance(value, list) and len(value) == 3 and all(is_finite_number(item) for item in value)


def is_finite_number(value: object) -> bool:
    """bool不是传感器数值；NaN/Inf也不能进入健康输入。"""
    return not isinstance(value, bool) and isinstance(value, int | float) and math.isfinite(value)


def format_xyz(value: list[float]) -> str:
    """把XYZ数组格式化成短字符串，方便终端阅读。"""
    return "[" + ", ".join(f"{float(item):.3f}" for item in value) + "]"


def run_process(command: list[str], timeout_s: float, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    """运行外部命令并捕获输出；所有ROS CLI检查都走这个入口。"""
    return subprocess.run(
        command,
        cwd=PROJECT_ROOT,
        env=env,
        text=True,
        capture_output=True,
        timeout=max(timeout_s, 1.0),
        check=False,
    )


def check_ros_topic_list() -> tuple[list[str], CheckResult]:
    """读取ROS topic列表。"""
    try:
        process = run_process(["ros2", "topic", "list"], timeout_s=5.0)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return [], CheckResult("ROS topic list", "fail", f"无法执行 ros2 topic list: {exc}")

    if process.returncode != 0:
        detail = (process.stderr or process.stdout).strip()
        return [], CheckResult("ROS topic list", "fail", detail or f"退出码 {process.returncode}")
    topics = [line.strip() for line in process.stdout.splitlines() if line.strip()]
    return topics, CheckResult("ROS topic list", "pass", f"{len(topics)} 个 topic")


def check_topic_exists(topic: str, topics: list[str], required: bool = True) -> CheckResult:
    """检查topic是否存在。"""
    if topic in topics:
        return CheckResult(f"Topic {topic}", "pass", "存在")
    return CheckResult(f"Topic {topic}", "fail" if required else "warn", "不存在")


def check_topic_rate(topic: str, timeout_s: float, min_rate_hz: float) -> CheckResult:
    """用ros2 topic hz估算topic频率。"""
    command = ["timeout", "--signal=INT", f"{timeout_s:.1f}s", "ros2", "topic", "hz", topic]
    try:
        process = run_process(command, timeout_s=timeout_s + 2.0)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return CheckResult(f"Rate {topic}", "fail", f"无法采样topic频率: {exc}")

    output = "\n".join(part for part in (process.stdout, process.stderr) if part)
    rates = [float(match) for match in re.findall(r"average rate:\s*([0-9.]+)", output)]
    if not rates:
        return CheckResult(f"Rate {topic}", "fail", "没有收到足够消息计算频率")
    rate = rates[-1]
    if rate < min_rate_hz:
        return CheckResult(f"Rate {topic}", "fail", f"{rate:.2f} Hz，低于 {min_rate_hz:.2f} Hz")
    return CheckResult(f"Rate {topic}", "pass", f"{rate:.2f} Hz")


def run_planning_once(phase: str, mission_path: Path = DEFAULT_MISSION) -> CheckResult:
    """运行一次Mission非执行预览；用于端到端 smoke。"""
    script = PROJECT_ROOT / "localmap" / "scripts" / "run_planning_once.sh"
    env = os.environ.copy()
    try:
        process = run_process(
            [
                str(script),
                "--mission",
                str(mission_path),
                "--phase",
                phase,
                "--planning-scope",
                "preview_global",
            ],
            timeout_s=60.0,
            env=env,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return CheckResult("Run planning once", "fail", f"规划脚本执行失败: {exc}")
    if process.returncode != 0:
        detail = (process.stderr or process.stdout).strip()
        return CheckResult("Run planning once", "fail", detail[-800:] if detail else f"退出码 {process.returncode}")
    return CheckResult("Run planning once", "pass", "规划脚本完成")


def build_checks(args: argparse.Namespace) -> list[CheckResult]:
    """执行所有启用的检查并返回结果列表。"""
    export_dir = args.export_dir
    local_map_json = args.local_map_json or export_dir / "local_map.live.json"
    trajectory_json = args.trajectory_json or export_dir / "trajectory_command.simple_rrt.json"
    bucket_tip_json = args.bucket_tip_json or export_dir / "bucket_tip.machine_root.live.json"

    results: list[CheckResult] = []
    if not args.skip_ros:
        topics, topic_list_result = check_ros_topic_list()
        results.append(topic_list_result)
        if topic_list_result.status == "pass":
            required_topics = [
                "/rslidar_points",
                "/localmap/machine_root_points",
                "/occupied_cells_vis_array",
                "/localmap/reachable_workspace_markers",
            ]
            for topic in required_topics:
                results.append(check_topic_exists(topic, topics))
            for topic in ("/rslidar_points", "/localmap/machine_root_points"):
                if topic in topics:
                    # 关键：只对点云采样频率；MarkerArray可能低频或仅在RViz订阅后显示。
                    results.append(check_topic_rate(topic, args.topic_timeout_s, args.min_point_rate_hz))

    results.append(check_local_map_json(local_map_json, args.expected_frame))
    results.append(check_bucket_tip_json(bucket_tip_json, args.expected_frame))

    require_trajectory = args.require_trajectory
    if args.run_planning_phase is not None:
        results.append(run_planning_once(args.run_planning_phase, args.mission))
        if args.trajectory_json is None:
            trajectory_json = (
                export_dir.parent
                / "live_preview"
                / "trajectory_command.preview_global.json"
            )
        require_trajectory = True
    results.append(check_trajectory_json(trajectory_json, args.expected_frame, required=require_trajectory))
    return results


def print_results(results: list[CheckResult]) -> None:
    """打印适合终端阅读的检查结果。"""
    for result in results:
        label = {"pass": "PASS", "warn": "WARN", "fail": "FAIL"}.get(result.status, result.status.upper())
        print(f"[{label}] {result.name}: {result.detail}")

    fail_count = sum(1 for result in results if result.status == "fail")
    warn_count = sum(1 for result in results if result.status == "warn")
    print()
    print(f"summary: fail={fail_count}, warn={warn_count}, total={len(results)}")


def exit_code_for_results(results: list[CheckResult]) -> int:
    """只要有必需检查失败，命令返回非零；warning不阻断。"""
    return 1 if any(result.status == "fail" for result in results) else 0


def main() -> int:
    """入口函数：执行 smoke check 并用退出码表达健康状态。"""
    args = build_arg_parser().parse_args()
    results = build_checks(args)
    print_results(results)
    return exit_code_for_results(results)


if __name__ == "__main__":
    raise SystemExit(main())
