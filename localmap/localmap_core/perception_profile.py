"""实时感知栈的版本化配置与严格加载器。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PERCEPTION_PROFILE = PROJECT_ROOT / "localmap" / "config" / "perception.json"
PERCEPTION_PROFILE_SCHEMA = "perception_profile_v1"


class PerceptionProfileError(ValueError):
    """Perception Profile 不满足运行契约。"""


@dataclass(frozen=True)
class PerceptionInputs:
    rslidar_config: Path
    extrinsics: Path
    targets: Path
    bucket_tip_bridge: Path


@dataclass(frozen=True)
class PerceptionOutputs:
    live_local_map: Path
    live_bucket_tip: Path
    log_dir: Path


@dataclass(frozen=True)
class PerceptionTopics:
    raw_cloud: str
    machine_cloud: str
    octomap_cells: str
    bucket_tip_fk: str
    bucket_tip_machine_root: str


@dataclass(frozen=True)
class LocalMapSettings:
    bounds: tuple[float, float, float, float, float, float]
    write_every: int
    publish_every: int


@dataclass(frozen=True)
class OctomapSettings:
    resolution_m: float
    max_range_m: float
    filter_ground_plane: bool
    reset_interval_s: float
    crop_bounds: tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class PerceptionProfile:
    profile_id: str
    expected_frame: str
    inputs: PerceptionInputs
    outputs: PerceptionOutputs
    topics: PerceptionTopics
    local_map: LocalMapSettings
    octomap: OctomapSettings


def _validate_fields(section: str, data: object, expected: set[str]) -> dict:
    if not isinstance(data, dict):
        raise PerceptionProfileError(f"{section} 必须是JSON object")
    missing = expected - set(data)
    if missing:
        raise PerceptionProfileError(f"{section} 缺少字段: {', '.join(sorted(missing))}")
    unknown = set(data) - expected
    if unknown:
        raise PerceptionProfileError(f"{section} 包含未知字段: {', '.join(sorted(unknown))}")
    return data


def _require_string(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise PerceptionProfileError(f"{name} 必须是非空字符串，实际为 {value!r}")
    return value


def _require_topic(name: str, value: object) -> str:
    topic = _require_string(name, value)
    if not topic.startswith("/") or topic == "/":
        raise PerceptionProfileError(f"{name} 必须是绝对ROS topic，实际为 {topic!r}")
    return topic


def _require_int(name: str, value: object, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise PerceptionProfileError(f"{name} 必须是 {minimum}..{maximum} 的整数，实际为 {value!r}")
    return value


def _require_number(name: str, value: object, minimum: float, maximum: float) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, int | float)
        or not math.isfinite(value)
        or not minimum <= value <= maximum
    ):
        raise PerceptionProfileError(f"{name} 必须在 {minimum}..{maximum} 范围内，实际为 {value!r}")
    return float(value)


def _require_bounds(name: str, value: object) -> tuple[float, float, float, float, float, float]:
    if not isinstance(value, list) or len(value) != 6:
        raise PerceptionProfileError(f"{name} 必须包含6个数值")
    if any(
        isinstance(item, bool)
        or not isinstance(item, int | float)
        or not math.isfinite(item)
        for item in value
    ):
        raise PerceptionProfileError(f"{name} 必须包含6个有限数值")
    bounds = tuple(float(item) for item in value)
    if any(bounds[index + 1] <= bounds[index] for index in (0, 2, 4)):
        raise PerceptionProfileError(f"{name} 每个最大值必须大于最小值")
    return bounds


def _resolve_path(name: str, value: object, project_root: Path) -> Path:
    candidate = Path(_require_string(name, value))
    return candidate if candidate.is_absolute() else project_root / candidate


def _require_crop_within_local_map(
    crop: tuple[float, float, float, float, float, float],
    local_map: tuple[float, float, float, float, float, float],
) -> None:
    for index in (0, 2, 4):
        if crop[index] < local_map[index] or crop[index + 1] > local_map[index + 1]:
            raise PerceptionProfileError("octomap.crop_bounds 必须位于 local_map.bounds 内")


def load_perception_profile(
    path: Path = DEFAULT_PERCEPTION_PROFILE,
    *,
    project_root: Path = PROJECT_ROOT,
) -> PerceptionProfile:
    """加载唯一的感知配置；相对路径统一按AiryLidar根目录解析。"""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PerceptionProfileError(f"无法读取perception profile: {path}: {exc}") from exc

    root = _validate_fields(
        "root",
        data,
        {"schema", "profile_id", "expected_frame", "inputs", "outputs", "topics", "local_map", "octomap"},
    )
    if root["schema"] != PERCEPTION_PROFILE_SCHEMA:
        raise PerceptionProfileError(
            f"perception profile schema 必须是 {PERCEPTION_PROFILE_SCHEMA}，实际为 {root['schema']!r}"
        )
    profile_id = _require_string("profile_id", root["profile_id"])
    if root["expected_frame"] != "machine_root":
        raise PerceptionProfileError(
            f"expected_frame 必须是 'machine_root'，实际为 {root['expected_frame']!r}"
        )

    inputs = _validate_fields(
        "inputs",
        root["inputs"],
        {"rslidar_config", "extrinsics", "targets", "bucket_tip_bridge"},
    )
    outputs = _validate_fields(
        "outputs",
        root["outputs"],
        {"live_local_map", "live_bucket_tip", "log_dir"},
    )
    topics = _validate_fields(
        "topics",
        root["topics"],
        {"raw_cloud", "machine_cloud", "octomap_cells", "bucket_tip_fk", "bucket_tip_machine_root"},
    )
    local_map = _validate_fields("local_map", root["local_map"], {"bounds", "write_every", "publish_every"})
    octomap = _validate_fields(
        "octomap",
        root["octomap"],
        {"resolution_m", "max_range_m", "filter_ground_plane", "reset_interval_s", "crop_bounds"},
    )

    local_bounds = _require_bounds("local_map.bounds", local_map["bounds"])
    crop_bounds = _require_bounds("octomap.crop_bounds", octomap["crop_bounds"])
    _require_crop_within_local_map(crop_bounds, local_bounds)
    if not isinstance(octomap["filter_ground_plane"], bool):
        raise PerceptionProfileError("octomap.filter_ground_plane 必须是boolean")
    resolved_topics = {
        name: _require_topic(f"topics.{name}", value)
        for name, value in topics.items()
    }
    if len(set(resolved_topics.values())) != len(resolved_topics):
        raise PerceptionProfileError("感知输入、输出ROS topic不能相同")

    return PerceptionProfile(
        profile_id=profile_id,
        expected_frame=root["expected_frame"],
        inputs=PerceptionInputs(
            rslidar_config=_resolve_path("inputs.rslidar_config", inputs["rslidar_config"], project_root),
            extrinsics=_resolve_path("inputs.extrinsics", inputs["extrinsics"], project_root),
            targets=_resolve_path("inputs.targets", inputs["targets"], project_root),
            bucket_tip_bridge=_resolve_path(
                "inputs.bucket_tip_bridge",
                inputs["bucket_tip_bridge"],
                project_root,
            ),
        ),
        outputs=PerceptionOutputs(
            live_local_map=_resolve_path("outputs.live_local_map", outputs["live_local_map"], project_root),
            live_bucket_tip=_resolve_path("outputs.live_bucket_tip", outputs["live_bucket_tip"], project_root),
            log_dir=_resolve_path("outputs.log_dir", outputs["log_dir"], project_root),
        ),
        topics=PerceptionTopics(
            raw_cloud=resolved_topics["raw_cloud"],
            machine_cloud=resolved_topics["machine_cloud"],
            octomap_cells=resolved_topics["octomap_cells"],
            bucket_tip_fk=resolved_topics["bucket_tip_fk"],
            bucket_tip_machine_root=resolved_topics["bucket_tip_machine_root"],
        ),
        local_map=LocalMapSettings(
            bounds=local_bounds,
            write_every=_require_int("local_map.write_every", local_map["write_every"], 1, 10000),
            publish_every=_require_int("local_map.publish_every", local_map["publish_every"], 1, 10000),
        ),
        octomap=OctomapSettings(
            resolution_m=_require_number("octomap.resolution_m", octomap["resolution_m"], 0.001, 1.0),
            max_range_m=_require_number("octomap.max_range_m", octomap["max_range_m"], 0.1, 100.0),
            filter_ground_plane=octomap["filter_ground_plane"],
            reset_interval_s=_require_number(
                "octomap.reset_interval_s",
                octomap["reset_interval_s"],
                0.0,
                3600.0,
            ),
            crop_bounds=crop_bounds,
        ),
    )
