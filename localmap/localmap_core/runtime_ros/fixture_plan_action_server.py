#!/usr/bin/python3
"""Typed, non-motion Plan Action using an explicit empty-map fixture."""

from __future__ import annotations

import math
import re
import threading
import time
from dataclasses import dataclass

import rclpy
from airy_excavator_interfaces.action import Plan
from airy_excavator_interfaces.msg import TrajectorySnapshot
from airy_excavator_interfaces.snapshot_digest import (
    trajectory_snapshot_message_sha256,
)
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from localmap_core.fixture_planning import FixturePlanningRequest, plan_fixture_trajectory


PLAN_ACTION = "/planning/plan"
BUCKET_TIP_TOPIC = "/bucket_tip_pose_machine_root_ros"
TRAJECTORY_TOPIC = "/planning/trajectory_snapshot"
PATH_TOPIC = "/planning/preview_path"
MARKERS_TOPIC = "/planning/preview_markers"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class _TipSample:
    sequence: int
    message: PoseStamped


class FixturePlanActionNode(Node):
    """Fixture-only Plan server; it cannot consume live maps or send motion."""

    def __init__(self, *, context=None, max_bucket_tip_age_s: float = 0.5) -> None:
        super().__init__("fixture_plan_server", context=context)
        self._max_bucket_tip_age_s = float(max_bucket_tip_age_s)
        if not math.isfinite(self._max_bucket_tip_age_s) or self._max_bucket_tip_age_s <= 0:
            raise ValueError("max_bucket_tip_age_s must be positive")
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._goal_reserved = False
        self._tip_sequence = 0
        self._latest_tip: _TipSample | None = None

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._trajectory_publisher = self.create_publisher(
            TrajectorySnapshot, TRAJECTORY_TOPIC, latched_qos
        )
        self._path_publisher = self.create_publisher(Path, PATH_TOPIC, latched_qos)
        self._marker_publisher = self.create_publisher(
            MarkerArray, MARKERS_TOPIC, latched_qos
        )
        self.create_subscription(
            PoseStamped,
            BUCKET_TIP_TOPIC,
            self._on_tip,
            10,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            Plan,
            PLAN_ACTION,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "fixture Plan ready: input_source=fixture map_source=fixture_empty "
            "execution_eligible=false action_datagrams=0"
        )

    def destroy_node(self):
        self._action_server.destroy()
        return super().destroy_node()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_tip(self, message: PoseStamped) -> None:
        with self._lock:
            self._tip_sequence += 1
            self._latest_tip = _TipSample(self._tip_sequence, message)

    def _on_goal(self, request: Plan.Goal) -> GoalResponse:
        try:
            _validate_target(
                request,
                now_s=self._now_s(),
                max_age_s=self._max_bucket_tip_age_s,
            )
        except ValueError as exc:
            self.get_logger().warning("Plan goal rejected: %s" % exc)
            return GoalResponse.REJECT
        with self._lock:
            if self._goal_reserved:
                return GoalResponse.REJECT
            self._goal_reserved = True
        self._clear_preview()
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> Plan.Result:
        started_at_s = self._now_s()
        with self._lock:
            baseline_sequence = self._tip_sequence
        self._publish_feedback(goal_handle, "waiting_for_bucket_tip", 0)
        try:
            sample = self._wait_for_tip(goal_handle, baseline_sequence, started_at_s)
            if isinstance(sample, Plan.Result):
                return sample
            tip = sample.message
            stamp_s = _time_to_seconds(tip.header.stamp)
            age_s = self._now_s() - stamp_s
            if tip.header.frame_id != "machine_root_ros":
                return self._finish(
                    goal_handle, Plan.Result.OUTCOME_FAILED, "FRAME_MISMATCH",
                    "Bucket Tip frame must be machine_root_ros",
                )
            if stamp_s <= 0.0 or age_s < -1e-6 or age_s > self._max_bucket_tip_age_s:
                return self._finish(
                    goal_handle, Plan.Result.OUTCOME_FAILED, "STALE_BUCKET_TIP",
                    f"Bucket Tip age is invalid: {age_s:.3f}s",
                )

            self._publish_feedback(goal_handle, "planning_fixture_empty_map", 0)
            target = goal_handle.request.target
            created_at_s = self._now_s()
            request = FixturePlanningRequest(
                frame_id="machine_root_ros",
                input_source="fixture",
                map_source="fixture_empty",
                start_m=(tip.pose.position.x, tip.pose.position.y, tip.pose.position.z),
                start_stamp_s=stamp_s,
                target_id=target.target_id,
                target_kind=target.target_kind,
                target_status=target.target_status,
                target_m=(target.position.x, target.position.y, target.position.z),
                mission_id=target.mission_id,
                mission_sha256=target.mission_sha256,
                mission_phase=target.mission_phase,
                planning_scope=goal_handle.request.planning_scope,
                created_at_s=created_at_s,
            )
            planned = plan_fixture_trajectory(request, waypoint_count=5)
            if goal_handle.is_cancel_requested:
                return self._finish(
                    goal_handle, Plan.Result.OUTCOME_CANCELLED, "CANCELLED",
                    "Planning cancelled",
                )
            if not planned.success:
                return self._finish(
                    goal_handle, Plan.Result.OUTCOME_FAILED, "PLANNING_FAILED", planned.reason,
                )

            trajectory = _trajectory_message(
                request,
                planned.task_mode,
                planned.waypoints,
                created_at_s=created_at_s,
                source_local_map_stamp_s=created_at_s,
            )
            self._trajectory_publisher.publish(trajectory)
            self._publish_preview(trajectory)
            self._publish_feedback(goal_handle, "published", planned.iterations)
            return self._finish(
                goal_handle, Plan.Result.OUTCOME_SUCCEEDED, "SUCCEEDED",
                "Fixture trajectory planned", trajectory=trajectory,
            )
        except Exception as exc:
            self.get_logger().error("Plan failed unexpectedly: %s" % exc)
            return self._finish(
                goal_handle, Plan.Result.OUTCOME_FAILED, "INTERNAL_ERROR", str(exc),
            )

    def _wait_for_tip(self, goal_handle, baseline_sequence: int, started_at_s: float):
        while rclpy.ok(context=self.context):
            if goal_handle.is_cancel_requested:
                return self._finish(
                    goal_handle, Plan.Result.OUTCOME_CANCELLED, "CANCELLED",
                    "Planning cancelled",
                )
            with self._lock:
                sample = self._latest_tip
            if sample is not None and sample.sequence > baseline_sequence:
                return sample
            if self._now_s() - started_at_s > self._max_bucket_tip_age_s:
                return self._finish(
                    goal_handle, Plan.Result.OUTCOME_FAILED, "STALE_BUCKET_TIP",
                    "No fresh Bucket Tip sample arrived",
                )
            time.sleep(0.01)
        return self._finish(
            goal_handle, Plan.Result.OUTCOME_FAILED, "INTERNAL_ERROR", "ROS context stopped",
        )

    def _finish(
        self,
        goal_handle,
        outcome: int,
        reason: str,
        message: str,
        *,
        trajectory: TrajectorySnapshot | None = None,
    ) -> Plan.Result:
        try:
            if outcome == Plan.Result.OUTCOME_SUCCEEDED:
                goal_handle.succeed()
            elif outcome == Plan.Result.OUTCOME_CANCELLED:
                self._clear_preview()
                goal_handle.canceled()
            else:
                self._clear_preview()
                goal_handle.abort()
        finally:
            with self._lock:
                self._goal_reserved = False
        result = Plan.Result()
        result.outcome = outcome
        result.reason_code = reason
        result.message = message
        if trajectory is not None:
            result.trajectory = trajectory
        result.action_datagrams = 0
        return result

    def _publish_feedback(self, goal_handle, stage: str, iterations: int) -> None:
        feedback = Plan.Feedback()
        feedback.stage = stage
        feedback.iterations = int(iterations)
        goal_handle.publish_feedback(feedback)

    def _clear_preview(self) -> None:
        path = Path()
        path.header.frame_id = "machine_root_ros"
        path.header.stamp = self.get_clock().now().to_msg()
        self._path_publisher.publish(path)
        clear = Marker()
        clear.action = Marker.DELETEALL
        self._marker_publisher.publish(MarkerArray(markers=[clear]))

    def _publish_preview(self, trajectory: TrajectorySnapshot) -> None:
        path = Path()
        path.header = trajectory.header
        for point in trajectory.waypoints:
            pose = PoseStamped()
            pose.header = trajectory.header
            pose.pose.position = point
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_publisher.publish(path)

        line = Marker()
        line.header = trajectory.header
        line.ns = "planned_bucket_tip_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.015
        line.color.r = 0.1
        line.color.g = 0.9
        line.color.b = 0.3
        line.color.a = 1.0
        line.points = list(trajectory.waypoints)
        self._marker_publisher.publish(MarkerArray(markers=[line]))


def _validate_target(request: Plan.Goal, *, now_s: float, max_age_s: float) -> None:
    target = request.target
    if request.planning_scope != "preview_global":
        raise ValueError("fixture Plan only supports preview_global")
    if target.header.frame_id != "machine_root_ros":
        raise ValueError("target frame must be machine_root_ros")
    target_stamp_s = _time_to_seconds(target.header.stamp)
    target_age_s = now_s - target_stamp_s
    if target_stamp_s <= 0.0 or target_age_s < -1e-6 or target_age_s > max_age_s:
        raise ValueError(f"target timestamp is stale or invalid: age={target_age_s:.3f}s")
    if target.target_kind not in {"dig", "dump"} or target.mission_phase != target.target_kind:
        raise ValueError("target kind and Mission phase mismatch")
    if target.target_status not in {"placeholder", "rviz_adjusted", "field_validated"}:
        raise ValueError("target_status is invalid")
    if not target.target_id.strip() or not target.mission_id.strip():
        raise ValueError("target and Mission IDs must be non-empty")
    if not _SHA256.fullmatch(target.mission_sha256):
        raise ValueError("mission_sha256 must be lowercase sha256")
    values = (
        target.position.x, target.position.y, target.position.z,
        target.normal.x, target.normal.y, target.normal.z, target.radius_m,
    )
    if not all(math.isfinite(value) for value in values) or target.radius_m <= 0.0:
        raise ValueError("target geometry is invalid")
    normal_norm = math.sqrt(target.normal.x**2 + target.normal.y**2 + target.normal.z**2)
    if not math.isclose(normal_norm, 1.0, abs_tol=1e-6):
        raise ValueError("target normal must be a unit vector")


def _trajectory_message(
    request: FixturePlanningRequest,
    task_mode: str,
    waypoints: tuple[tuple[float, float, float], ...],
    *,
    created_at_s: float,
    source_local_map_stamp_s: float,
) -> TrajectorySnapshot:
    message = TrajectorySnapshot()
    message.header.frame_id = request.frame_id
    message.header.stamp = _seconds_to_time(created_at_s)
    message.mission_id = request.mission_id
    message.mission_sha256 = request.mission_sha256
    message.mission_phase = request.mission_phase
    message.task_mode = task_mode
    message.planning_scope = request.planning_scope
    message.control_stage = "none"
    message.workspace_constraint = "none"
    message.execution_eligible = False
    message.source_bucket_tip_stamp = _seconds_to_time(request.start_stamp_s)
    message.source_local_map_stamp = _seconds_to_time(source_local_map_stamp_s)
    message.inputs_frozen_at = _seconds_to_time(created_at_s)
    message.valid_until = _seconds_to_time(created_at_s + 10.0)
    message.input_source = request.input_source
    message.map_source = request.map_source
    message.clock_mode = "ros_clock"
    message.waypoints = [Point(x=x, y=y, z=z) for x, y, z in waypoints]
    message.waypoint_tolerance_m = 0.05
    message.waypoint_dwell_s = 0.3
    message.tracking_timeout_s = 20.0
    digest = trajectory_snapshot_message_sha256(message)
    message.trajectory_id = f"fixture-{request.target_id}-{digest[:12]}"
    message.trajectory_sha256 = digest
    return message


def _seconds_to_time(value: float) -> Time:
    seconds = int(math.floor(value))
    nanoseconds = int(round((value - seconds) * 1e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=seconds, nanosec=nanoseconds)


def _time_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main() -> None:
    rclpy.init()
    node = FixturePlanActionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
