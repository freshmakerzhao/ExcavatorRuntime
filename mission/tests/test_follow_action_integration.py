import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus
from airy_excavator_interfaces.action import Follow
from airy_excavator_interfaces.msg import RuntimeStatus
from airy_excavator_interfaces.snapshot_digest import (
    trajectory_snapshot_message_sha256,
)
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from mission.runtime_ros.follow_action_server import MachineBehaviorNode


def wait_future(future, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), "ROS future did not complete"
    return future.result()


def time_msg(seconds):
    stamp = rclpy.time.Time(seconds=seconds).to_msg()
    return stamp


def build_goal(now_s):
    goal = Follow.Goal()
    snapshot = goal.trajectory
    snapshot.header.frame_id = "machine_root_ros"
    snapshot.header.stamp = time_msg(now_s)
    snapshot.trajectory_id = "integration-trajectory"
    snapshot.trajectory_sha256 = "0" * 64
    snapshot.mission_id = "integration-mission"
    snapshot.mission_sha256 = "b" * 64
    snapshot.mission_phase = "dig"
    snapshot.task_mode = "MoveToDig"
    snapshot.planning_scope = "preview_global"
    snapshot.control_stage = "none"
    snapshot.workspace_constraint = "none"
    snapshot.execution_eligible = False
    snapshot.source_bucket_tip_stamp = time_msg(now_s - 0.02)
    snapshot.source_local_map_stamp = time_msg(now_s - 0.02)
    snapshot.inputs_frozen_at = time_msg(now_s)
    snapshot.valid_until = time_msg(now_s + 5.0)
    snapshot.input_source = "fixture"
    snapshot.map_source = "fixture_empty"
    snapshot.clock_mode = "ros_clock"
    snapshot.waypoints = [Point(x=0.25, y=-0.1, z=0.4)]
    snapshot.waypoint_tolerance_m = 0.02
    snapshot.waypoint_dwell_s = 0.05
    snapshot.tracking_timeout_s = 2.0
    snapshot.trajectory_sha256 = trajectory_snapshot_message_sha256(snapshot)
    return goal


def start_harness(max_bucket_tip_age_s=0.5):
    context = rclpy.context.Context()
    rclpy.init(context=context)
    server = MachineBehaviorNode(
        input_source="fixture",
        max_bucket_tip_age_s=max_bucket_tip_age_s,
        context=context,
    )
    client_node = rclpy.create_node("follow_integration_client", context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    client = ActionClient(client_node, Follow, "/excavator/follow")
    assert client.wait_for_server(timeout_sec=2.0)
    return context, server, client_node, executor, spin_thread, client


def stop_harness(harness):
    context, server, client_node, executor, spin_thread, _ = harness
    executor.shutdown(timeout_sec=1.0)
    spin_thread.join(timeout=1.0)
    client_node.destroy_node()
    server.destroy_node()
    rclpy.shutdown(context=context)


def test_follow_action_completes_from_fresh_pose_samples_without_datagrams():
    harness = start_harness()
    _, _, client_node, _, _, client = harness
    try:
        publisher = client_node.create_publisher(
            PoseStamped, "/bucket_tip_pose_machine_root_ros", 10
        )
        now_s = client_node.get_clock().now().nanoseconds * 1e-9
        goal_handle = wait_future(client.send_goal_async(build_goal(now_s)))
        assert goal_handle.accepted

        for _ in range(4):
            pose = PoseStamped()
            pose.header.frame_id = "machine_root_ros"
            pose.header.stamp = client_node.get_clock().now().to_msg()
            pose.pose.position = Point(x=0.25, y=-0.1, z=0.4)
            publisher.publish(pose)
            time.sleep(0.08)

        response = wait_future(goal_handle.get_result_async())
        assert response.status == GoalStatus.STATUS_SUCCEEDED
        assert response.result.reason_code == "SUCCEEDED"
        assert response.result.quiescence_confirmed
        assert response.result.action_datagrams == 0
    finally:
        stop_harness(harness)


def test_follow_action_without_fresh_pose_aborts_quiescently():
    harness = start_harness(max_bucket_tip_age_s=0.1)
    _, _, client_node, _, _, client = harness
    try:
        now_s = client_node.get_clock().now().nanoseconds * 1e-9
        goal_handle = wait_future(client.send_goal_async(build_goal(now_s)))
        assert goal_handle.accepted

        response = wait_future(goal_handle.get_result_async())
        assert response.status == GoalStatus.STATUS_ABORTED
        assert response.result.reason_code == "STALE_BUCKET_TIP"
        assert response.result.quiescence_confirmed
        assert response.result.action_datagrams == 0
    finally:
        stop_harness(harness)


def test_follow_cancel_releases_the_behavior_lease_before_result():
    harness = start_harness(max_bucket_tip_age_s=1.0)
    _, _, client_node, _, _, client = harness
    try:
        now_s = client_node.get_clock().now().nanoseconds * 1e-9
        goal_handle = wait_future(client.send_goal_async(build_goal(now_s)))
        assert goal_handle.accepted

        cancel = wait_future(goal_handle.cancel_goal_async())
        assert cancel.goals_canceling
        response = wait_future(goal_handle.get_result_async())
        assert response.status == GoalStatus.STATUS_CANCELED
        assert response.result.reason_code == "CANCELLED"
        assert response.result.quiescence_confirmed
        assert response.result.action_datagrams == 0

        next_goal = wait_future(
            client.send_goal_async(
                build_goal(client_node.get_clock().now().nanoseconds * 1e-9)
            )
        )
        assert next_goal.accepted
        wait_future(next_goal.cancel_goal_async())
        wait_future(next_goal.get_result_async())
    finally:
        stop_harness(harness)


def test_second_follow_goal_is_rejected_as_busy_without_preemption():
    harness = start_harness(max_bucket_tip_age_s=1.0)
    _, _, client_node, _, _, client = harness
    statuses = []
    client_node.create_subscription(
        RuntimeStatus,
        "/mission/runtime_status",
        statuses.append,
        10,
    )
    try:
        now_s = client_node.get_clock().now().nanoseconds * 1e-9
        first = wait_future(client.send_goal_async(build_goal(now_s)))
        assert first.accepted
        second = wait_future(
            client.send_goal_async(
                build_goal(client_node.get_clock().now().nanoseconds * 1e-9)
            )
        )
        assert not second.accepted
        deadline = time.monotonic() + 1.0
        while not any(status.last_rejection_reason == "BUSY" for status in statuses):
            assert time.monotonic() < deadline
            time.sleep(0.01)

        wait_future(first.cancel_goal_async())
        response = wait_future(first.get_result_async())
        assert response.status == GoalStatus.STATUS_CANCELED
    finally:
        stop_harness(harness)


def test_follow_rejects_tampered_trajectory_and_reports_provenance_reason():
    harness = start_harness()
    _, _, client_node, _, _, client = harness
    statuses = []
    client_node.create_subscription(
        RuntimeStatus,
        "/mission/runtime_status",
        statuses.append,
        10,
    )
    try:
        goal = build_goal(client_node.get_clock().now().nanoseconds * 1e-9)
        goal.trajectory.waypoints[0].x += 1.0
        handle = wait_future(client.send_goal_async(goal))
        assert not handle.accepted

        deadline = time.monotonic() + 1.0
        while not any(
            status.last_rejection_reason == "TRAJECTORY_PROVENANCE_MISMATCH"
            for status in statuses
        ):
            assert time.monotonic() < deadline
            time.sleep(0.01)
    finally:
        stop_harness(harness)
