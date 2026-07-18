"""Named joint-pose contract and fail-closed ReturnHome shadow observer."""

from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping


_POSE_STATUSES = {"placeholder", "measured", "field_validated"}


@dataclass(frozen=True)
class NamedJointPose:
    pose_id: str
    status: str
    joint_order: tuple[str, ...]
    position_rad: tuple[float, ...]
    joint_limits_rad: tuple[tuple[float, float], ...]
    tolerance_rad: float
    dwell_s: float
    timeout_s: float
    note: str


@dataclass(frozen=True)
class NamedJointPoseSet:
    machine_id: str
    urdf_sha256: str
    joint_order: tuple[str, ...]
    poses: Mapping[str, NamedJointPose]
    sha256: str


@dataclass(frozen=True)
class ReturnHomeUpdate:
    sample_accepted: bool
    within_tolerance: bool
    completed: bool
    timed_out: bool
    max_error_rad: float
    elapsed_s: float
    current_position_rad: tuple[float, ...]
    error_rad: tuple[float, ...]


@dataclass(frozen=True)
class ReturnHomeSession:
    pose: NamedJointPose
    accepted_at_s: float
    last_sample_stamp_s: float | None = None
    within_since_s: float | None = None
    completed: bool = False

    @classmethod
    def start(cls, pose: NamedJointPose, *, accepted_at_s: float) -> "ReturnHomeSession":
        _require_finite("accepted_at_s", accepted_at_s)
        return cls(pose=pose, accepted_at_s=float(accepted_at_s))

    def observe(
        self,
        positions_by_name: Mapping[str, float],
        *,
        sample_stamp_s: float,
        now_s: float,
    ) -> tuple["ReturnHomeSession", ReturnHomeUpdate]:
        _require_finite("sample_stamp_s", sample_stamp_s)
        _require_finite("now_s", now_s)
        expected_names = set(self.pose.joint_order)
        if set(positions_by_name) != expected_names:
            raise ValueError("joint names must exactly match the named pose contract")
        if sample_stamp_s <= self.accepted_at_s:
            raise ValueError("joint sample must be newer than goal acceptance")
        if self.last_sample_stamp_s is not None and sample_stamp_s <= self.last_sample_stamp_s:
            raise ValueError("joint sample timestamp must increase monotonically")

        current = tuple(float(positions_by_name[name]) for name in self.pose.joint_order)
        if not all(math.isfinite(value) for value in current):
            raise ValueError("joint positions must be finite")
        for joint_name, value, limits in zip(
            self.pose.joint_order,
            current,
            self.pose.joint_limits_rad,
            strict=True,
        ):
            if value < limits[0] or value > limits[1]:
                raise ValueError(f"current {joint_name} is outside URDF joint limits")
        errors = tuple(
            target - actual
            for target, actual in zip(self.pose.position_rad, current, strict=True)
        )
        max_error = max(abs(error) for error in errors)
        within_tolerance = max_error <= self.pose.tolerance_rad
        within_since_s = (
            self.within_since_s
            if within_tolerance and self.within_since_s is not None
            else (now_s if within_tolerance else None)
        )
        elapsed_s = max(0.0, now_s - self.accepted_at_s)
        timed_out = elapsed_s > self.pose.timeout_s
        completed = (
            not timed_out
            and within_since_s is not None
            and now_s - within_since_s >= self.pose.dwell_s
        )
        next_session = ReturnHomeSession(
            pose=self.pose,
            accepted_at_s=self.accepted_at_s,
            last_sample_stamp_s=float(sample_stamp_s),
            within_since_s=within_since_s,
            completed=completed,
        )
        update = ReturnHomeUpdate(
            sample_accepted=True,
            within_tolerance=within_tolerance,
            completed=completed,
            timed_out=timed_out,
            max_error_rad=max_error,
            elapsed_s=elapsed_s,
            current_position_rad=current,
            error_rad=errors,
        )
        return next_session, update


def load_named_joint_pose_set(
    path: str | Path, *, urdf_path: str | Path
) -> NamedJointPoseSet:
    config_path = Path(path)
    raw = config_path.read_bytes()
    document = json.loads(raw)
    if document.get("schema_version") != "named_joint_poses.v1":
        raise ValueError("unsupported named joint pose schema")

    joint_order = tuple(str(name) for name in document.get("joint_order", ()))
    if not joint_order or len(set(joint_order)) != len(joint_order):
        raise ValueError("joint_order must contain unique joint names")
    expected_urdf_sha = hashlib.sha256(Path(urdf_path).read_bytes()).hexdigest()
    configured_urdf_sha = str(document.get("urdf_sha256", ""))
    if configured_urdf_sha != expected_urdf_sha:
        raise ValueError("named joint poses do not match the installed URDF SHA-256")
    joint_limits = _load_urdf_joint_limits(Path(urdf_path), joint_order)

    poses: dict[str, NamedJointPose] = {}
    raw_poses = document.get("poses")
    if not isinstance(raw_poses, dict) or not raw_poses:
        raise ValueError("poses must be a non-empty object")
    for pose_id, values in raw_poses.items():
        status = str(values.get("status", ""))
        if status not in _POSE_STATUSES:
            raise ValueError(f"pose {pose_id} status is invalid")
        position = tuple(float(value) for value in values.get("position_rad", ()))
        if len(position) != len(joint_order) or not all(
            math.isfinite(value) for value in position
        ):
            raise ValueError(f"pose {pose_id} has invalid position_rad")
        for joint_name, joint_position in zip(joint_order, position, strict=True):
            lower, upper = joint_limits[joint_name]
            if joint_position < lower or joint_position > upper:
                raise ValueError(
                    f"pose {pose_id} {joint_name} is outside URDF joint limits"
                )
        tolerance = _positive_float(values, "tolerance_rad", pose_id)
        dwell = _nonnegative_float(values, "dwell_s", pose_id)
        timeout = _positive_float(values, "timeout_s", pose_id)
        if dwell >= timeout:
            raise ValueError(f"pose {pose_id} dwell_s must be less than timeout_s")
        poses[str(pose_id)] = NamedJointPose(
            pose_id=str(pose_id),
            status=status,
            joint_order=joint_order,
            position_rad=position,
            joint_limits_rad=tuple(joint_limits[name] for name in joint_order),
            tolerance_rad=tolerance,
            dwell_s=dwell,
            timeout_s=timeout,
            note=str(values.get("note", "")),
        )

    return NamedJointPoseSet(
        machine_id=str(document.get("machine_id", "")),
        urdf_sha256=configured_urdf_sha,
        joint_order=joint_order,
        poses=MappingProxyType(poses),
        sha256=hashlib.sha256(raw).hexdigest(),
    )


def _positive_float(values: Mapping[str, object], key: str, pose_id: str) -> float:
    value = float(values.get(key, 0.0))
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"pose {pose_id} {key} must be positive")
    return value


def _nonnegative_float(values: Mapping[str, object], key: str, pose_id: str) -> float:
    value = float(values.get(key, -1.0))
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"pose {pose_id} {key} must be non-negative")
    return value


def _require_finite(name: str, value: float) -> None:
    if not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")


def _load_urdf_joint_limits(
    urdf_path: Path, joint_order: tuple[str, ...]
) -> dict[str, tuple[float, float]]:
    root = ET.fromstring(urdf_path.read_bytes())
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    limits: dict[str, tuple[float, float]] = {}
    for joint_name in joint_order:
        joint = joints.get(joint_name)
        limit = joint.find("limit") if joint is not None else None
        if joint is None or joint.get("type") != "revolute" or limit is None:
            raise ValueError(f"URDF joint {joint_name} must be a bounded revolute joint")
        try:
            lower = float(limit.attrib["lower"])
            upper = float(limit.attrib["upper"])
        except (KeyError, ValueError) as exc:
            raise ValueError(f"URDF joint {joint_name} has invalid limits") from exc
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f"URDF joint {joint_name} has invalid limits")
        limits[joint_name] = (lower, upper)
    return limits
