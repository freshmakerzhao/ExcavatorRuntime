#!/usr/bin/env python3
"""一键运行离线bag到LocalMap/RRT/Observation切片的最小链路。"""

from __future__ import annotations

import argparse
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = LOCALMAP_DIR / "scripts"
DEFAULT_EXTRINSICS = LOCALMAP_DIR / "config" / "extrinsics_rslidar_to_machine_root.measured.json"
DEFAULT_BUCKET_TIP = LOCALMAP_DIR / "config" / "bucket_tip.machine_root.measured.json"
DEFAULT_TARGETS = LOCALMAP_DIR / "config" / "targets.mock.json"


@dataclass(frozen=True)
class PipelineConfig:
    """保存pipeline参数；集中管理可避免各步骤手工拼命令时错位。"""

    bag: Path
    output_dir: Path
    python: str
    extrinsics: Path
    bucket_tip: Path
    targets: Path
    topic: str
    inspect_frames: int
    max_csv_points: int
    storage_id: str
    target_id: str
    target_kind: str
    task_mode: str
    waypoint_count: int
    current_index: int
    up_axis: str
    bounds: list[float] | None
    reuse_export: bool
    skip_inspect: bool
    dry_run: bool


def topic_stem(topic: str) -> str:
    """把ROS topic转成导出文件名前缀。"""
    return topic.strip("/").replace("/", "_")


def default_output_dir_for_bag(bag: Path) -> Path:
    """根据bag目录名生成默认导出目录，保持一次bag一套产物。"""
    return LOCALMAP_DIR / "exports" / bag.name


def build_arg_parser() -> argparse.ArgumentParser:
    """构造命令行参数；默认跑machine_root离线验证链路。"""
    parser = argparse.ArgumentParser(description="离线运行 Airy bag -> LocalMap -> RRT request -> trajectory -> observation slice。")
    parser.add_argument("--bag", type=Path, required=True, help="ros2 bag目录，例如 bags/airy_repositioned_xxx")
    parser.add_argument("--output-dir", type=Path, help="导出目录，默认 localmap/exports/<bag目录名>")
    parser.add_argument("--python", default=sys.executable, help="执行子脚本的Python，ROS bag建议/usr/bin/python3")
    parser.add_argument("--extrinsics", type=Path, default=DEFAULT_EXTRINSICS, help="rslidar到machine_root外参JSON")
    parser.add_argument("--bucket-tip", type=Path, default=DEFAULT_BUCKET_TIP, help="bucket tip状态/测量JSON")
    parser.add_argument("--targets", type=Path, default=DEFAULT_TARGETS, help="dig/dump target配置JSON")
    parser.add_argument("--topic", default="/rslidar_points", help="PointCloud2 topic")
    parser.add_argument("--inspect-frames", type=int, default=3, help="inspect_bag_points采样帧数")
    parser.add_argument("--max-csv-points", type=int, default=2000, help="CSV样本最多点数")
    parser.add_argument("--storage-id", default="mcap", help="rosbag2 storage id")
    parser.add_argument("--target-id", default="mock_dig_001", help="RRT目标id")
    parser.add_argument("--target-kind", choices=["dig", "dump"], default="dig", help="RRT目标类型")
    parser.add_argument("--task-mode", choices=["MoveToDig", "CarryMaterial"], default="MoveToDig", help="任务模式")
    parser.add_argument("--waypoint-count", type=int, default=5, help="mock轨迹输出waypoint数量")
    parser.add_argument("--current-index", type=int, default=0, help="observation切片使用的当前waypoint index")
    parser.add_argument("--up-axis", choices=["x", "y", "z"], default="y", help="目标frame竖直向上轴")
    parser.add_argument(
        "--bounds",
        type=float,
        nargs=6,
        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX", "Z_MIN", "Z_MAX"),
        help="可选点云裁剪范围，单位米，作用在目标frame点云上",
    )
    parser.add_argument("--reuse-export", action="store_true", help="若NPZ已存在则跳过导出首帧")
    parser.add_argument("--skip-inspect", action="store_true", help="跳过bag结构检查")
    parser.add_argument("--dry-run", action="store_true", help="只打印将要执行的命令，不写产物")
    return parser


def parse_config(argv: list[str] | None = None) -> PipelineConfig:
    """解析命令行参数并补齐默认导出目录。"""
    args = build_arg_parser().parse_args(argv)
    output_dir = args.output_dir if args.output_dir is not None else default_output_dir_for_bag(args.bag)
    return PipelineConfig(
        bag=args.bag,
        output_dir=output_dir,
        python=args.python,
        extrinsics=args.extrinsics,
        bucket_tip=args.bucket_tip,
        targets=args.targets,
        topic=args.topic,
        inspect_frames=args.inspect_frames,
        max_csv_points=args.max_csv_points,
        storage_id=args.storage_id,
        target_id=args.target_id,
        target_kind=args.target_kind,
        task_mode=args.task_mode,
        waypoint_count=args.waypoint_count,
        current_index=args.current_index,
        up_axis=args.up_axis,
        bounds=args.bounds,
        reuse_export=args.reuse_export,
        skip_inspect=args.skip_inspect,
        dry_run=args.dry_run,
    )


def output_paths(config: PipelineConfig) -> dict[str, Path]:
    """统一定义pipeline产物路径，后续脚本只消费这些文件。"""
    stem = topic_stem(config.topic)
    return {
        "npz": config.output_dir / f"{stem}_first_frame.npz",
        "local_map": config.output_dir / "local_map_machine_root.measured.json",
        "rrt_request": config.output_dir / "rrt_star_request.machine_root.json",
        "trajectory": config.output_dir / "trajectory_command.machine_root.json",
        "observation": config.output_dir / "observation_waypoint_slice.machine_root.json",
    }


def build_commands(config: PipelineConfig) -> list[tuple[str, list[str]]]:
    """生成各步骤命令；这个函数也被测试使用，防止参数传错脚本。"""
    paths = output_paths(config)
    commands: list[tuple[str, list[str]]] = []

    if not config.skip_inspect:
        commands.append(
            (
                "inspect",
                [
                    config.python,
                    str(SCRIPTS_DIR / "inspect_bag_points.py"),
                    str(config.bag),
                    "--topic",
                    config.topic,
                    "--frames",
                    str(config.inspect_frames),
                    "--storage-id",
                    config.storage_id,
                ],
            )
        )

    if not (config.reuse_export and paths["npz"].exists()):
        commands.append(
            (
                "export",
                [
                    config.python,
                    str(SCRIPTS_DIR / "export_first_cloud.py"),
                    str(config.bag),
                    "--topic",
                    config.topic,
                    "--output-dir",
                    str(config.output_dir),
                    "--storage-id",
                    config.storage_id,
                    "--max-csv-points",
                    str(config.max_csv_points),
                ],
            )
        )

    local_map_command = [
        config.python,
        str(SCRIPTS_DIR / "generate_local_map_from_npz.py"),
        "--npz",
        str(paths["npz"]),
        "--bag-path",
        str(config.bag),
        "--extrinsics",
        str(config.extrinsics),
        "--targets",
        str(config.targets),
        "--up-axis",
        config.up_axis,
        "--output",
        str(paths["local_map"]),
    ]
    if config.bounds is not None:
        local_map_command.extend(["--bounds", *[str(value) for value in config.bounds]])
    commands.append(("local_map", local_map_command))

    commands.append(
        (
            "rrt_request",
            [
                config.python,
                str(SCRIPTS_DIR / "generate_rrt_request_from_local_map.py"),
                "--local-map",
                str(paths["local_map"]),
                "--bucket-tip",
                str(config.bucket_tip),
                "--target-id",
                config.target_id,
                "--target-kind",
                config.target_kind,
                "--task-mode",
                config.task_mode,
                "--output",
                str(paths["rrt_request"]),
            ],
        )
    )

    commands.append(
        (
            "trajectory",
            [
                config.python,
                str(SCRIPTS_DIR / "generate_mock_trajectory_from_rrt_request.py"),
                "--request",
                str(paths["rrt_request"]),
                "--waypoint-count",
                str(config.waypoint_count),
                "--output",
                str(paths["trajectory"]),
            ],
        )
    )

    commands.append(
        (
            "observation",
            [
                config.python,
                str(SCRIPTS_DIR / "generate_observation_waypoint_slice.py"),
                "--trajectory",
                str(paths["trajectory"]),
                "--bucket-tip",
                str(config.bucket_tip),
                "--current-index",
                str(config.current_index),
                "--output",
                str(paths["observation"]),
            ],
        )
    )
    return commands


def shell_join(command: list[str]) -> str:
    """生成便于复制的命令文本；只用于显示，不交给shell执行。"""
    return " ".join(f"'{part}'" if " " in part else part for part in command)


def run_command(name: str, command: list[str], dry_run: bool) -> None:
    """执行一个pipeline步骤；失败时保留原始返回码，方便定位。"""
    print(f"\n=== {name} ===", flush=True)
    print(shell_join(command), flush=True)
    if dry_run:
        return
    subprocess.run(command, check=True)


def validate_inputs(config: PipelineConfig) -> None:
    """提前检查关键输入文件，避免跑到中途才发现配置缺失。"""
    if not config.bag.exists():
        raise SystemExit(f"找不到bag目录: {config.bag}")
    for path, label in (
        (config.extrinsics, "外参"),
        (config.bucket_tip, "bucket tip"),
        (config.targets, "targets"),
    ):
        if not path.exists():
            raise SystemExit(f"找不到{label}配置: {path}")


def print_summary(config: PipelineConfig) -> None:
    """打印关键产物路径和几何检查提示。"""
    paths = output_paths(config)
    print("\n=== summary ===")
    for key, path in paths.items():
        print(f"{key}: {path}")
    print("check: local_map.frame_id 应与 bucket_tip.frame_id 一致，machine_root场景通常应为 machine_root")
    print("check: ground normal 在machine_root/Unity约定下通常应接近 [0, 1, 0]")


def main(argv: list[str] | None = None) -> int:
    """入口函数：顺序运行离线pipeline，并在最后打印产物摘要。"""
    config = parse_config(argv)
    validate_inputs(config)
    config.output_dir.mkdir(parents=True, exist_ok=True)

    for name, command in build_commands(config):
        run_command(name, command, config.dry_run)

    if not config.dry_run:
        print_summary(config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
