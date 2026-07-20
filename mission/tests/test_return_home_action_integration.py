import threading
import time
import hashlib
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus
from airy_excavator_interfaces.action import Follow, ReturnHome
from airy_excavator_interfaces.msg import HomePoseCatalog, RuntimeStatus
from airy_excavator_interfaces.snapshot_digest import (
    trajectory_snapshot_message_sha256,
)
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from mission.runtime_ros.follow_action_server import MachineBehaviorNode


JOINT_NAMES = ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
POSE_SET_PATH = (
    Path(__file__).resolve().parents[2]
    / "kinematics/waji_description/config/named_joint_poses.json"
)
POSE_SET_SHA256 = hashlib.sha256(POSE_SET_PATH.read_bytes()).hexdigest()


def wait_future(future, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), "ROS future did not complete"
    return future.result()


def start_harness(max_joint_state_age_s=0.5):
    context = rclpy.context.Context()
    rclpy.init(context=context)
    server = MachineBehaviorNode(
        input_source="fixture",
        max_joint_state_age_s=max_joint_state_age_s,
        context=context,
    )
    client_node = rclpy.create_node("return_home_integration_client", context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    client = ActionClient(client_node, ReturnHome, "/excavator/return_home")
    assert client.wait_for_server(timeout_sec=2.0)
    return context, server, client_node, executor, spin_thread, client


def stop_harness(harness):
    context, server, client_node, executor, spin_thread, _ = harness
    executor.shutdown(timeout_sec=1.0)
    spin_thread.join(timeout=1.0)
    client_node.destroy_node()
    server.destroy_node()
    rclpy.shutdown(context=context)


def build_goal():
    goal = ReturnHome.Goal()
    goal.home_pose_id = "transport_home"
    goal.pose_set_sha256 = POSE_SET_SHA256
    return goal


def build_follow_goal(now_s):
    goal = Follow.Goal()
    snapshot = goal.trajectory
    snapshot.header.frame_id = "machine_root_ros"
    snapshot.header.stamp = rclpy.time.Time(seconds=now_s).to_msg()
    snapshot.trajectory_id = "shared-lease-test"
    snapshot.trajectory_sha256 = "0" * 64
    snapshot.mission_id = "shared-lease-mission"
    snapshot.mission_sha256 = "b" * 64
    snapshot.mission_phase = "dig"
    snapshot.task_mode = "MoveToDig"
    snapshot.planning_scope = "preview_global"
    snapshot.control_stage = "none"
    snapshot.workspace_constraint = "none"
    snapshot.execution_eligible = False
    snapshot.source_bucket_tip_stamp = rclpy.time.Time(seconds=now_s - 0.01).to_msg()
    snapshot.source_local_map_stamp = rclpy.time.Time(seconds=now_s - 0.01).to_msg()
    snapshot.inputs_frozen_at = rclpy.time.Time(seconds=now_s).to_msg()
    snapshot.valid_until = rclpy.time.Time(seconds=now_s + 5.0).to_msg()
    snapshot.input_source = "fixture"
    snapshot.map_source = "fixture_empty"
    snapshot.clock_mode = "ros_clock"
    snapshot.waypoints = [Point(x=0.0, y=0.0, z=0.0)]
    snapshot.waypoint_tolerance_m = 0.02
    snapshot.waypoint_dwell_s = 0.1
    snapshot.tracking_timeout_s = 2.0
    snapshot.trajectory_sha256 = trajectory_snapshot_message_sha256(snapshot)
    return goal


def test_home_pose_catalog_exposes_pose_id_status_and_contract_sha():
    harness = start_harness()
    _, _, client_node, _, _, _ = harness
    catalogs = []
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    client_node.create_subscription(
        HomePoseCatalog, "/mission/home_pose_catalog", catalogs.append, qos
    )
    try:
        deadline = time.monotonic() + 1.0
        while not catalogs and time.monotonic() < deadline:
            time.sleep(0.01)
        assert catalogs
        assert catalogs[-1].pose_set_sha256 == POSE_SET_SHA256
        assert catalogs[-1].pose_ids == ["transport_home"]
        assert catalogs[-1].pose_statuses == ["placeholder"]
    finally:
        stop_harness(harness)


def test_return_home_completes_from_fresh_joint_samples_without_datagrams():
    harness = start_harness()
    _, _, client_node, _, _, client = harness
    try:
        publisher = client_node.create_publisher(JointState, "/joint_states", 10)
        goal_handle = wait_future(client.send_goal_async(build_goal()))
        assert goal_handle.accepted

        for _ in range(5):
            message = JointState()
            message.header.stamp = client_node.get_clock().now().to_msg()
            message.name = JOINT_NAMES
            message.position = [0.0, 0.0, 0.0, 0.0]
            publisher.publish(message)
            time.sleep(0.1)

        response = wait_future(goal_handle.get_result_async())
        assert response.status == GoalStatus.STATUS_SUCCEEDED
        assert response.result.reason_code == "SUCCEEDED"
        assert response.result.quiescence_confirmed
        assert response.result.action_datagrams == 0
    finally:
        stop_harness(harness)


def test_return_home_without_fresh_joint_state_aborts_quiescently():
    harness = start_harness(max_joint_state_age_s=0.1)
    _, _, _, _, _, client = harness
    try:
        goal_handle = wait_future(client.send_goal_async(build_goal()))
        assert goal_handle.accepted

        response = wait_future(goal_handle.get_result_async())
        assert response.status == GoalStatus.STATUS_ABORTED
        assert response.result.reason_code == "STALE_JOINT_STATE"
        assert response.result.quiescence_confirmed
        assert response.result.action_datagrams == 0
    finally:
        stop_harness(harness)


def test_return_home_rejects_joint_state_with_missing_contract_joint():
    harness = start_harness()
    _, _, client_node, _, _, client = harness
    try:
        publisher = client_node.create_publisher(JointState, "/joint_states", 10)
        goal_handle = wait_future(client.send_goal_async(build_goal()))
        assert goal_handle.accepted

        message = JointState()
        message.header.stamp = client_node.get_clock().now().to_msg()
        message.name = JOINT_NAMES[:-1]
        message.position = [0.0, 0.0, 0.0]
        publisher.publish(message)

        response = wait_future(goal_handle.get_result_async())
        assert response.status == GoalStatus.STATUS_ABORTED
        assert response.result.reason_code == "INVALID_JOINT_STATE"
        assert response.result.quiescence_confirmed
        assert response.result.action_datagrams == 0
    finally:
        stop_harness(harness)


def test_return_home_cancel_releases_lease_before_result():
    harness = start_harness(max_joint_state_age_s=1.0)
    _, _, _, _, _, client = harness
    try:
        first = wait_future(client.send_goal_async(build_goal()))
        assert first.accepted
        cancel = wait_future(first.cancel_goal_async())
        assert cancel.goals_canceling
        response = wait_future(first.get_result_async())
        assert response.status == GoalStatus.STATUS_CANCELED
        assert response.result.reason_code == "CANCELLED"
        assert response.result.quiescence_confirmed
        assert response.result.action_datagrams == 0

        second = wait_future(client.send_goal_async(build_goal()))
        assert second.accepted
        wait_future(second.cancel_goal_async())
        wait_future(second.get_result_async())
    finally:
        stop_harness(harness)


def test_active_return_home_rejects_follow_through_the_shared_lease():
    harness = start_harness(max_joint_state_age_s=1.0)
    _, _, client_node, _, _, return_home_client = harness
    statuses = []
    client_node.create_subscription(
        RuntimeStatus,
        "/mission/runtime_status",
        statuses.append,
        10,
    )
    follow_client = ActionClient(client_node, Follow, "/excavator/follow")
    assert follow_client.wait_for_server(timeout_sec=2.0)
    try:
        return_home = wait_future(return_home_client.send_goal_async(build_goal()))
        assert return_home.accepted
        now_s = client_node.get_clock().now().nanoseconds * 1e-9
        follow = wait_future(follow_client.send_goal_async(build_follow_goal(now_s)))
        assert not follow.accepted

        deadline = time.monotonic() + 1.0
        while not any(
            status.active_behavior == "ReturnHome"
            and status.last_rejection_reason == "BUSY"
            for status in statuses
        ):
            assert time.monotonic() < deadline
            time.sleep(0.01)

        wait_future(return_home.cancel_goal_async())
        response = wait_future(return_home.get_result_async())
        assert response.status == GoalStatus.STATUS_CANCELED
    finally:
        follow_client.destroy()
        stop_harness(harness)
