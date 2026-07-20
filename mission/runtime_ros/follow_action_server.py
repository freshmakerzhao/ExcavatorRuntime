#!/usr/bin/python3
"""ROS Action Adapter for the shadow-only Follow Machine Behavior."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from airy_excavator_interfaces.action import Follow, ReturnHome
from airy_excavator_interfaces.msg import (
    HomePoseCatalog,
    RuntimeStatus,
    TrajectorySnapshot,
)
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from mission.follow import (
    FollowSession,
    FollowTrajectorySnapshot,
    TrajectoryDigestMismatch,
)
from mission.home import (
    NamedJointPose,
    ReturnHomeSession,
    ReturnHomeUpdate,
    load_named_joint_pose_set,
)
from mission.runtime_ros.no_motion_backend import NoMotionBackend


FOLLOW_ACTION = "/excavator/follow"
RETURN_HOME_ACTION = "/excavator/return_home"
BUCKET_TIP_TOPIC = "/bucket_tip_pose_machine_root_ros"
JOINT_STATES_TOPIC = "/joint_states"
RUNTIME_STATUS_TOPIC = "/mission/runtime_status"
FOLLOW_MARKERS_TOPIC = "/mission/follow_markers"
HOME_POSE_CATALOG_TOPIC = "/mission/home_pose_catalog"


@dataclass(frozen=True)
class _TipSample:
    sequence: int
    message: PoseStamped
    received_at_s: float


@dataclass(frozen=True)
class _JointSample:
    sequence: int
    message: JointState
    received_at_s: float


class MachineBehaviorNode(Node):
    """Shadow Machine Behaviors sharing one fail-closed execution lease."""

    def __init__(
        self,
        *,
        input_source: str = "fixture",
        execution_mode: str = "shadow",
        max_bucket_tip_age_s: float = 0.5,
        max_joint_state_age_s: float = 0.5,
        context=None,
    ) -> None:
        super().__init__("machine_behavior_shadow_server", context=context)
        self.declare_parameter("input_source", input_source)
        self.declare_parameter("execution_mode", execution_mode)
        self.declare_parameter("max_bucket_tip_age_s", max_bucket_tip_age_s)
        self.declare_parameter("max_joint_state_age_s", max_joint_state_age_s)
        self._input_source = str(self.get_parameter("input_source").value)
        self._execution_mode = str(self.get_parameter("execution_mode").value)
        self._max_bucket_tip_age_s = float(
            self.get_parameter("max_bucket_tip_age_s").value
        )
        self._max_joint_state_age_s = float(
            self.get_parameter("max_joint_state_age_s").value
        )
        if self._input_source not in {"fixture", "replay", "live"}:
            raise ValueError("input_source must be fixture, replay, or live")
        if self._execution_mode != "shadow":
            raise ValueError("Follow runtime only implements execution_mode=shadow")
        if not math.isfinite(self._max_bucket_tip_age_s) or self._max_bucket_tip_age_s <= 0.0:
            raise ValueError("max_bucket_tip_age_s must be positive")
        if not math.isfinite(self._max_joint_state_age_s) or self._max_joint_state_age_s <= 0.0:
            raise ValueError("max_joint_state_age_s must be positive")

        description_share = Path(get_package_share_directory("waji_description"))
        self._pose_set = load_named_joint_pose_set(
            description_share / "config/named_joint_poses.json",
            urdf_path=description_share / "urdf/waji.urdf",
        )

        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._active_behavior = ""
        self._lease_held = False
        self._finishing = False
        self._tip_sequence = 0
        self._latest_tip: _TipSample | None = None
        self._joint_sequence = 0
        self._latest_joint: _JointSample | None = None
        self._last_rejection_reason = ""
        self._last_rejection_message = ""
        self._backend = NoMotionBackend()

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            RuntimeStatus, RUNTIME_STATUS_TOPIC, latched_qos
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, FOLLOW_MARKERS_TOPIC, latched_qos
        )
        self._home_pose_catalog_publisher = self.create_publisher(
            HomePoseCatalog, HOME_POSE_CATALOG_TOPIC, latched_qos
        )
        self.create_subscription(
            PoseStamped,
            BUCKET_TIP_TOPIC,
            self._on_bucket_tip,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self._on_joint_state,
            10,
            callback_group=self._callback_group,
        )
        self._follow_action_server = ActionServer(
            self,
            Follow,
            FOLLOW_ACTION,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self._return_home_action_server = ActionServer(
            self,
            ReturnHome,
            RETURN_HOME_ACTION,
            execute_callback=self._execute_return_home,
            goal_callback=self._on_return_home_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self._status_timer = self.create_timer(
            0.5, self._publish_status, callback_group=self._callback_group
        )
        self._publish_status()
        self._publish_home_pose_catalog()
        self.get_logger().info(
            "Machine Behavior shadow server ready: input_source=%s execution_mode=shadow "
            "motion_backend=none action_datagrams=0" % self._input_source
        )

    def destroy_node(self):
        self._status_timer.cancel()
        self._follow_action_server.destroy()
        self._return_home_action_server.destroy()
        return super().destroy_node()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_home_pose_catalog(self) -> None:
        catalog = HomePoseCatalog()
        catalog.header.stamp = self.get_clock().now().to_msg()
        catalog.header.frame_id = "machine_root_ros"
        catalog.pose_set_sha256 = self._pose_set.sha256
        catalog.pose_ids = sorted(self._pose_set.poses)
        catalog.pose_statuses = [
            self._pose_set.poses[pose_id].status for pose_id in catalog.pose_ids
        ]
        self._home_pose_catalog_publisher.publish(catalog)

    def _on_bucket_tip(self, message: PoseStamped) -> None:
        with self._lock:
            self._tip_sequence += 1
            self._latest_tip = _TipSample(
                sequence=self._tip_sequence,
                message=message,
                received_at_s=self._now_s(),
            )

    def _on_joint_state(self, message: JointState) -> None:
        with self._lock:
            self._joint_sequence += 1
            self._latest_joint = _JointSample(
                sequence=self._joint_sequence,
                message=message,
                received_at_s=self._now_s(),
            )

    def _on_goal(self, goal_request: Follow.Goal) -> GoalResponse:
        try:
            snapshot = _snapshot_from_message(goal_request.trajectory)
            snapshot.validate_for_shadow(
                expected_input_source=self._input_source,
                now_s=self._now_s(),
            )
        except TrajectoryDigestMismatch as exc:
            self._reject("TRAJECTORY_PROVENANCE_MISMATCH", str(exc))
            return GoalResponse.REJECT
        except (TypeError, ValueError) as exc:
            self._reject("INVALID_TRAJECTORY", str(exc))
            return GoalResponse.REJECT

        if not self._reserve_lease("Follow"):
            return GoalResponse.REJECT
        self._publish_markers(snapshot, current_index=0)
        return GoalResponse.ACCEPT

    def _on_return_home_goal(self, goal_request: ReturnHome.Goal) -> GoalResponse:
        pose = self._pose_set.poses.get(goal_request.home_pose_id)
        if pose is None:
            self._reject("UNKNOWN_HOME_POSE", "home_pose_id is not configured")
            return GoalResponse.REJECT
        if goal_request.pose_set_sha256 != self._pose_set.sha256:
            self._reject("POSE_SET_MISMATCH", "named pose set SHA-256 does not match")
            return GoalResponse.REJECT
        if not self._reserve_lease("ReturnHome"):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _reserve_lease(self, behavior: str) -> bool:
        with self._lock:
            if self._active_behavior or self._finishing:
                self._last_rejection_reason = "BUSY"
                self._last_rejection_message = (
                    f"{self._active_behavior or 'finishing behavior'} blocks the Machine Behavior lease"
                )
                accepted = False
            else:
                self._active_behavior = behavior
                self._lease_held = True
                self._last_rejection_reason = ""
                self._last_rejection_message = ""
                accepted = True
        self._publish_status()
        return accepted

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> Follow.Result:
        snapshot = _snapshot_from_message(goal_handle.request.trajectory)
        accepted_at_s = self._now_s()
        session = FollowSession.start(snapshot, accepted_at_s=accepted_at_s)
        with self._lock:
            baseline_sequence = self._tip_sequence
        processed_sequence = baseline_sequence
        last_unique_stamp_s: float | None = None
        last_unique_received_at_s = accepted_at_s
        latest_distance_m = -1.0
        try:
            while rclpy.ok(context=self.context):
                now_s = self._now_s()
                if goal_handle.is_cancel_requested:
                    return self._finish(
                        goal_handle,
                        outcome=Follow.Result.OUTCOME_CANCELLED,
                        reason_code="CANCELLED",
                        message="Follow goal cancelled",
                        final_index=session.tracker.current_index,
                        final_distance_m=latest_distance_m,
                    )
                if now_s - accepted_at_s > snapshot.tracking_timeout_s:
                    return self._abort(
                        goal_handle,
                        "TIMEOUT",
                        "Follow tracking timeout",
                        session,
                        latest_distance_m,
                    )

                with self._lock:
                    sample = self._latest_tip
                if sample is not None and sample.sequence > processed_sequence:
                    processed_sequence = sample.sequence
                    message = sample.message
                    stamp_s = _time_to_seconds(message.header.stamp)
                    if message.header.frame_id != snapshot.frame_id:
                        return self._abort(
                            goal_handle,
                            "FRAME_MISMATCH",
                            "Bucket Tip frame does not match trajectory",
                            session,
                            latest_distance_m,
                        )
                    age_s = now_s - stamp_s
                    if stamp_s <= 0.0 or age_s < -1e-6 or age_s > self._max_bucket_tip_age_s:
                        return self._abort(
                            goal_handle,
                            "STALE_BUCKET_TIP",
                            f"Bucket Tip age is invalid: {age_s:.3f}s",
                            session,
                            latest_distance_m,
                        )
                    if last_unique_stamp_s is not None and stamp_s < last_unique_stamp_s:
                        return self._abort(
                            goal_handle,
                            "OUT_OF_ORDER_BUCKET_TIP",
                            "Bucket Tip source timestamp moved backwards",
                            session,
                            latest_distance_m,
                        )
                    if last_unique_stamp_s is None or stamp_s > last_unique_stamp_s:
                        point = message.pose.position
                        latest_point = (float(point.x), float(point.y), float(point.z))
                        last_unique_stamp_s = stamp_s
                        last_unique_received_at_s = sample.received_at_s
                        session, update = session.observe(
                            latest_point,
                            sample_stamp_s=stamp_s,
                            now_s=now_s,
                        )
                        if update.sample_accepted:
                            latest_distance_m = update.distance_m
                            goal_handle.publish_feedback(
                                _feedback(
                                    message,
                                    update.current_waypoint_index,
                                    update.waypoint_count,
                                    update.distance_m,
                                    update.elapsed_s,
                                )
                            )
                            self._publish_markers(
                                snapshot, current_index=update.current_waypoint_index
                            )
                        if update.timed_out:
                            return self._abort(
                                goal_handle,
                                "TIMEOUT",
                                "Follow tracker timed out",
                                session,
                                latest_distance_m,
                            )
                        if update.completed:
                            return self._finish(
                                goal_handle,
                                outcome=Follow.Result.OUTCOME_SUCCEEDED,
                                reason_code="SUCCEEDED",
                                message="Shadow observed all trajectory waypoints",
                                final_index=session.tracker.current_index,
                                final_distance_m=latest_distance_m,
                            )

                if now_s - last_unique_received_at_s > self._max_bucket_tip_age_s:
                    return self._abort(
                        goal_handle,
                        "STALE_BUCKET_TIP",
                        "No fresh Bucket Tip sample arrived",
                        session,
                        latest_distance_m,
                    )
                time.sleep(0.01)
        except Exception as exc:  # fail closed at the ROS Adapter boundary
            self.get_logger().error("Follow failed unexpectedly: %s" % exc)
            return self._abort(
                goal_handle,
                "INTERNAL_ERROR",
                str(exc),
                session,
                latest_distance_m,
            )

        return self._abort(
            goal_handle,
            "INTERNAL_ERROR",
            "ROS context stopped",
            session,
            latest_distance_m,
        )

    def _execute_return_home(self, goal_handle) -> ReturnHome.Result:
        pose = self._pose_set.poses[goal_handle.request.home_pose_id]
        accepted_at_s = self._now_s()
        session = ReturnHomeSession.start(pose, accepted_at_s=accepted_at_s)
        with self._lock:
            processed_sequence = self._joint_sequence
        last_unique_received_at_s = accepted_at_s
        final_max_error_rad = -1.0
        try:
            while rclpy.ok(context=self.context):
                now_s = self._now_s()
                if goal_handle.is_cancel_requested:
                    return self._finish_return_home(
                        goal_handle,
                        outcome=ReturnHome.Result.OUTCOME_CANCELLED,
                        reason_code="CANCELLED",
                        message="ReturnHome goal cancelled",
                        final_max_error_rad=final_max_error_rad,
                    )
                if now_s - accepted_at_s > pose.timeout_s:
                    return self._finish_return_home(
                        goal_handle,
                        outcome=ReturnHome.Result.OUTCOME_FAILED,
                        reason_code="TIMEOUT",
                        message="ReturnHome observation timeout",
                        final_max_error_rad=final_max_error_rad,
                    )

                with self._lock:
                    sample = self._latest_joint
                if sample is not None and sample.sequence > processed_sequence:
                    processed_sequence = sample.sequence
                    message = sample.message
                    stamp_s = _time_to_seconds(message.header.stamp)
                    age_s = now_s - stamp_s
                    if (
                        stamp_s <= 0.0
                        or age_s < -1e-6
                        or age_s > self._max_joint_state_age_s
                    ):
                        return self._finish_return_home(
                            goal_handle,
                            outcome=ReturnHome.Result.OUTCOME_FAILED,
                            reason_code="STALE_JOINT_STATE",
                            message=f"JointState age is invalid: {age_s:.3f}s",
                            final_max_error_rad=final_max_error_rad,
                        )
                    if len(message.name) != len(message.position):
                        raise ValueError("JointState names and positions have different lengths")
                    positions = {
                        name: float(position)
                        for name, position in zip(
                            message.name, message.position, strict=True
                        )
                    }
                    if len(positions) != len(message.name):
                        raise ValueError("JointState contains duplicate joint names")
                    session, update = session.observe(
                        positions,
                        sample_stamp_s=stamp_s,
                        now_s=now_s,
                    )
                    last_unique_received_at_s = sample.received_at_s
                    final_max_error_rad = update.max_error_rad
                    goal_handle.publish_feedback(
                        _return_home_feedback(message, pose, update)
                    )
                    if update.timed_out:
                        return self._finish_return_home(
                            goal_handle,
                            outcome=ReturnHome.Result.OUTCOME_FAILED,
                            reason_code="TIMEOUT",
                            message="ReturnHome observation timeout",
                            final_max_error_rad=final_max_error_rad,
                        )
                    if update.completed:
                        return self._finish_return_home(
                            goal_handle,
                            outcome=ReturnHome.Result.OUTCOME_SUCCEEDED,
                            reason_code="SUCCEEDED",
                            message=f"Shadow observed named pose {pose.pose_id}",
                            final_max_error_rad=final_max_error_rad,
                        )

                if now_s - last_unique_received_at_s > self._max_joint_state_age_s:
                    return self._finish_return_home(
                        goal_handle,
                        outcome=ReturnHome.Result.OUTCOME_FAILED,
                        reason_code="STALE_JOINT_STATE",
                        message="No fresh JointState sample arrived",
                        final_max_error_rad=final_max_error_rad,
                    )
                time.sleep(0.01)
        except ValueError as exc:
            return self._finish_return_home(
                goal_handle,
                outcome=ReturnHome.Result.OUTCOME_FAILED,
                reason_code="INVALID_JOINT_STATE",
                message=str(exc),
                final_max_error_rad=final_max_error_rad,
            )
        except Exception as exc:  # fail closed at the ROS Adapter boundary
            self.get_logger().error("ReturnHome failed unexpectedly: %s" % exc)
            return self._finish_return_home(
                goal_handle,
                outcome=ReturnHome.Result.OUTCOME_FAILED,
                reason_code="INTERNAL_ERROR",
                message=str(exc),
                final_max_error_rad=final_max_error_rad,
            )

        return self._finish_return_home(
            goal_handle,
            outcome=ReturnHome.Result.OUTCOME_FAILED,
            reason_code="INTERNAL_ERROR",
            message="ROS context stopped",
            final_max_error_rad=final_max_error_rad,
        )

    def _finish_return_home(
        self,
        goal_handle,
        *,
        outcome: int,
        reason_code: str,
        message: str,
        final_max_error_rad: float,
    ) -> ReturnHome.Result:
        quiescent = self._backend.stop_and_confirm()
        if not quiescent:
            outcome = ReturnHome.Result.OUTCOME_FAILED
            reason_code = "INTERNAL_ERROR"
            message = "no-motion backend did not confirm quiescence"
        self._prepare_terminal()
        try:
            if outcome == ReturnHome.Result.OUTCOME_SUCCEEDED:
                goal_handle.succeed()
            elif outcome == ReturnHome.Result.OUTCOME_CANCELLED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        finally:
            self._complete_terminal()

        result = ReturnHome.Result()
        result.outcome = outcome
        result.reason_code = reason_code
        result.message = message
        result.final_max_error_rad = float(final_max_error_rad)
        result.quiescence_confirmed = quiescent
        result.action_datagrams = self._backend.action_datagrams
        return result

    def _abort(
        self,
        goal_handle,
        reason_code: str,
        message: str,
        session: FollowSession,
        distance_m: float,
    ) -> Follow.Result:
        return self._finish(
            goal_handle,
            outcome=Follow.Result.OUTCOME_FAILED,
            reason_code=reason_code,
            message=message,
            final_index=session.tracker.current_index,
            final_distance_m=distance_m,
        )

    def _finish(
        self,
        goal_handle,
        *,
        outcome: int,
        reason_code: str,
        message: str,
        final_index: int,
        final_distance_m: float,
    ) -> Follow.Result:
        quiescent = self._backend.stop_and_confirm()
        if not quiescent:
            outcome = Follow.Result.OUTCOME_FAILED
            reason_code = "INTERNAL_ERROR"
            message = "no-motion backend did not confirm quiescence"
        self._prepare_terminal()
        try:
            if outcome == Follow.Result.OUTCOME_SUCCEEDED:
                goal_handle.succeed()
            elif outcome == Follow.Result.OUTCOME_CANCELLED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        finally:
            self._complete_terminal()

        result = Follow.Result()
        result.outcome = outcome
        result.reason_code = reason_code
        result.message = message
        result.final_waypoint_index = int(final_index)
        result.final_distance_m = float(final_distance_m)
        result.quiescence_confirmed = quiescent
        result.action_datagrams = self._backend.action_datagrams
        return result

    def _reject(self, reason: str, message: str) -> None:
        with self._lock:
            self._last_rejection_reason = reason
            self._last_rejection_message = message
        self._publish_status()

    def _prepare_terminal(self) -> None:
        with self._lock:
            self._lease_held = False
            self._finishing = True
        self._publish_status()

    def _complete_terminal(self) -> None:
        with self._lock:
            self._active_behavior = ""
            self._finishing = False
        self._publish_status()

    def _publish_status(self) -> None:
        with self._lock:
            active_behavior = self._active_behavior
            lease_held = self._lease_held
            rejection_reason = self._last_rejection_reason
            rejection_message = self._last_rejection_message
        status = RuntimeStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "machine_root_ros"
        status.input_source = self._input_source
        status.execution_mode = self._execution_mode
        status.control_stage = "none"
        status.motion_backend = "none"
        status.motion_authorized = False
        status.sender_constructed = self._backend.sender_constructed
        status.quiescent = not lease_held and self._backend.action_datagrams == 0
        status.action_datagrams = self._backend.action_datagrams
        status.state_fresh = False
        status.control_enabled = False
        status.sensor_valid = False
        status.stm32_alive = False
        status.estop = False
        status.fault_free = True
        status.motion_gate_reason = "shadow_no_motion"
        status.active_behavior = active_behavior
        status.last_rejection_reason = rejection_reason
        status.last_rejection_message = rejection_message
        self._status_publisher.publish(status)

    def _publish_markers(
        self,
        snapshot: FollowTrajectorySnapshot,
        *,
        current_index: int,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.action = Marker.DELETEALL

        line = Marker()
        line.header.frame_id = snapshot.frame_id
        line.header.stamp = stamp
        line.ns = "follow_trajectory"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.015
        line.color.r = 0.1
        line.color.g = 0.8
        line.color.b = 1.0
        line.color.a = 1.0
        line.points = [Point(x=x, y=y, z=z) for x, y, z in snapshot.waypoints]

        waypoint = Marker()
        waypoint.header.frame_id = snapshot.frame_id
        waypoint.header.stamp = stamp
        waypoint.ns = "follow_current_waypoint"
        waypoint.id = 1
        waypoint.type = Marker.SPHERE
        waypoint.action = Marker.ADD
        waypoint.pose.orientation.w = 1.0
        target = snapshot.waypoints[min(current_index, len(snapshot.waypoints) - 1)]
        waypoint.pose.position = Point(x=target[0], y=target[1], z=target[2])
        waypoint.scale.x = waypoint.scale.y = waypoint.scale.z = 0.06
        waypoint.color.r = 1.0
        waypoint.color.g = 0.8
        waypoint.color.b = 0.1
        waypoint.color.a = 1.0
        self._marker_publisher.publish(MarkerArray(markers=[clear, line, waypoint]))


def _snapshot_from_message(message: TrajectorySnapshot) -> FollowTrajectorySnapshot:
    return FollowTrajectorySnapshot(
        frame_id=message.header.frame_id,
        created_at_s=_time_to_seconds(message.header.stamp),
        trajectory_id=message.trajectory_id,
        trajectory_sha256=message.trajectory_sha256,
        mission_id=message.mission_id,
        mission_sha256=message.mission_sha256,
        mission_phase=message.mission_phase,
        task_mode=message.task_mode,
        planning_scope=message.planning_scope,
        control_stage=message.control_stage,
        workspace_constraint=message.workspace_constraint,
        execution_eligible=bool(message.execution_eligible),
        source_bucket_tip_stamp_s=_time_to_seconds(message.source_bucket_tip_stamp),
        source_local_map_stamp_s=_time_to_seconds(message.source_local_map_stamp),
        inputs_frozen_at_s=_time_to_seconds(message.inputs_frozen_at),
        valid_until_s=_time_to_seconds(message.valid_until),
        input_source=message.input_source,
        map_source=message.map_source,
        clock_mode=message.clock_mode,
        waypoints=tuple((point.x, point.y, point.z) for point in message.waypoints),
        waypoint_tolerance_m=message.waypoint_tolerance_m,
        waypoint_dwell_s=message.waypoint_dwell_s,
        tracking_timeout_s=message.tracking_timeout_s,
    )


def _time_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _feedback(
    message: PoseStamped,
    current_index: int,
    waypoint_count: int,
    distance_m: float,
    elapsed_s: float,
) -> Follow.Feedback:
    feedback = Follow.Feedback()
    feedback.bucket_tip_stamp = message.header.stamp
    feedback.bucket_tip = message.pose.position
    feedback.current_waypoint_index = int(current_index)
    feedback.waypoint_count = int(waypoint_count)
    feedback.distance_m = float(distance_m)
    feedback.elapsed_s = float(elapsed_s)
    feedback.tracking_state = "tracking"
    feedback.action_datagrams = 0
    return feedback


def _return_home_feedback(
    message: JointState,
    pose: NamedJointPose,
    update: ReturnHomeUpdate,
) -> ReturnHome.Feedback:
    feedback = ReturnHome.Feedback()
    feedback.joint_state_stamp = message.header.stamp
    feedback.joint_names = list(pose.joint_order)
    feedback.current_position_rad = list(update.current_position_rad)
    feedback.target_position_rad = list(pose.position_rad)
    feedback.error_rad = list(update.error_rad)
    feedback.max_error_rad = float(update.max_error_rad)
    feedback.elapsed_s = float(update.elapsed_s)
    feedback.state = "within_tolerance" if update.within_tolerance else "observing"
    feedback.action_datagrams = 0
    return feedback


def main() -> None:
    rclpy.init()
    node = MachineBehaviorNode()
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
