"""PC-Orin 运行配置：用一个不可变对象隐藏网络、制品路径和运行常量。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUNTIME_CONFIG = PROJECT_ROOT / "runtime_bridge" / "config" / "runtime.json"
RUNTIME_CONFIG_SCHEMA = "runtime_bridge_config_v1"


class RuntimeConfigError(ValueError):
    """运行配置格式或取值不满足部署契约。"""


@dataclass(frozen=True)
class NetworkConfig:
    state_bind_host: str
    state_port: int
    orin_host: str
    action_port: int
    action_valid_ms: int
    action_time_source: str

    @property
    def state_endpoint(self) -> tuple[str, int]:
        return self.state_bind_host, self.state_port

    @property
    def action_endpoint(self) -> tuple[str, int]:
        return self.orin_host, self.action_port


@dataclass(frozen=True)
class ArtifactConfig:
    onnx: Path
    machine_profile: Path
    waypoint_slice: Path
    latest_observation: Path


@dataclass(frozen=True)
class PolicyConfig:
    bucket_tip_timeout_ms: int


@dataclass(frozen=True)
class DiagnosticsConfig:
    print_every: int
    write_every: int


@dataclass(frozen=True)
class RuntimeConfig:
    network: NetworkConfig
    artifacts: ArtifactConfig
    policy: PolicyConfig
    diagnostics: DiagnosticsConfig


def _validate_fields(section: str, data: object, expected: set[str]) -> None:
    if not isinstance(data, dict):
        raise RuntimeConfigError(f"{section} 必须是 JSON object")
    missing = expected - set(data)
    if missing:
        names = ", ".join(sorted(missing))
        raise RuntimeConfigError(f"{section} 缺少字段: {names}")
    unknown = set(data) - expected
    if unknown:
        names = ", ".join(sorted(unknown))
        raise RuntimeConfigError(f"{section} 包含未知字段: {names}")


def _require_int_range(name: str, value: object, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise RuntimeConfigError(f"{name} 必须是 {minimum}..{maximum} 的整数，实际为 {value!r}")
    return value


def _require_non_empty_string(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RuntimeConfigError(f"{name} 必须是非空字符串，实际为 {value!r}")
    return value


def load_runtime_config(
    path: Path = DEFAULT_RUNTIME_CONFIG,
    *,
    project_root: Path = PROJECT_ROOT,
) -> RuntimeConfig:
    """加载运行配置；所有相对制品路径统一相对 AiryLidar 根目录解析。"""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    _validate_fields("root", data, {"schema", "network", "artifacts", "policy", "diagnostics"})
    network = data["network"]
    artifacts = data["artifacts"]
    policy = data["policy"]
    diagnostics = data["diagnostics"]

    if data.get("schema") != RUNTIME_CONFIG_SCHEMA:
        raise RuntimeConfigError(
            f"runtime config schema 必须是 {RUNTIME_CONFIG_SCHEMA}，实际为 {data.get('schema')!r}"
        )
    _validate_fields(
        "network",
        network,
        {
            "state_bind_host",
            "state_port",
            "orin_host",
            "action_port",
            "action_valid_ms",
            "action_time_source",
        },
    )
    _validate_fields(
        "artifacts",
        artifacts,
        {"onnx", "machine_profile", "waypoint_slice", "latest_observation"},
    )
    _validate_fields("policy", policy, {"bucket_tip_timeout_ms"})
    _validate_fields("diagnostics", diagnostics, {"print_every", "write_every"})
    _require_non_empty_string("network.state_bind_host", network.get("state_bind_host"))
    _require_non_empty_string("network.orin_host", network.get("orin_host"))
    _require_int_range("network.state_port", network.get("state_port"), 1, 65535)
    _require_int_range("network.action_port", network.get("action_port"), 1, 65535)
    _require_int_range("network.action_valid_ms", network.get("action_valid_ms"), 1, 60000)
    if network.get("action_time_source") not in {"orin", "pc"}:
        raise RuntimeConfigError(
            "network.action_time_source 必须是 'orin' 或 'pc'，"
            f"实际为 {network.get('action_time_source')!r}"
        )
    _require_int_range(
        "policy.bucket_tip_timeout_ms",
        policy.get("bucket_tip_timeout_ms"),
        1,
        60000,
    )
    _require_int_range("diagnostics.print_every", diagnostics.get("print_every"), 0, 1000000)
    _require_int_range("diagnostics.write_every", diagnostics.get("write_every"), 0, 1000000)

    def resolve(name: str, value: object) -> Path:
        if not isinstance(value, str) or not value.strip():
            raise RuntimeConfigError(f"artifacts.{name} 必须是非空路径字符串")
        candidate = Path(value)
        return candidate if candidate.is_absolute() else project_root / candidate

    artifact_config = ArtifactConfig(
        onnx=resolve("onnx", artifacts["onnx"]),
        machine_profile=resolve("machine_profile", artifacts["machine_profile"]),
        waypoint_slice=resolve("waypoint_slice", artifacts["waypoint_slice"]),
        latest_observation=resolve("latest_observation", artifacts["latest_observation"]),
    )
    for name in ("onnx", "machine_profile", "waypoint_slice"):
        artifact_path = getattr(artifact_config, name)
        if not artifact_path.is_file():
            raise RuntimeConfigError(f"artifacts.{name} 不存在或不是文件: {artifact_path}")

    return RuntimeConfig(
        network=NetworkConfig(**network),
        artifacts=artifact_config,
        policy=PolicyConfig(**policy),
        diagnostics=DiagnosticsConfig(**diagnostics),
    )
