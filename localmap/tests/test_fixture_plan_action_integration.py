import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus
from airy_excavator_interfaces.action import Plan
from airy_excavator_interfaces.snapshot_digest import (
    trajectory_snapshot_message_sha256,
)
from geometry_msgs.msg import Point, PoseStamped, Vector3
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from localmap_core.runtime_ros.fixture_plan_action_server import FixturePlanActionNode


def wait_future(future, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), "ROS future did not complete"
    return future.result()


def build_goal(node, target=(0.8, 0.2, 0.1)):
    goal = Plan.Goal()
    goal.planning_scope = "preview_global"
    goal.target.header.frame_id = "machine_root_ros"
    goal.target.header.stamp = node.get_clock().now().to_msg()
    goal.target.target_id = "fixture-dig"
    goal.target.target_kind = "dig"
    goal.target.target_status = "placeholder"
    goal.target.mission_id = "fixture-mission"
    goal.target.mission_sha256 = "a" * 64
    goal.target.mission_phase = "dig"
    goal.target.position = Point(x=target[0], y=target[1], z=target[2])
    goal.target.normal = Vector3(x=0.0, y=0.0, z=1.0)
    goal.target.radius_m = 0.03
    return goal


def test_fixture_plan_action_publishes_typed_snapshot_and_path():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    server = FixturePlanActionNode(context=context)
    client_node = rclpy.create_node("fixture_plan_integration_client", context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    paths = []
    client_node.create_subscription(Path, "/planning/preview_path", paths.append, 10)
    try:
        client = ActionClient(client_node, Plan, "/planning/plan")
        publisher = client_node.create_publisher(
            PoseStamped, "/bucket_tip_pose_machine_root_ros", 10
        )
        assert client.wait_for_server(timeout_sec=2.0)
        goal_handle = wait_future(client.send_goal_async(build_goal(client_node)))
        assert goal_handle.accepted

        pose = PoseStamped()
        pose.header.frame_id = "machine_root_ros"
        pose.header.stamp = client_node.get_clock().now().to_msg()
        pose.pose.position = Point(x=0.2, y=-0.1, z=0.3)
        publisher.publish(pose)

        response = wait_future(goal_handle.get_result_async())
        assert response.status == GoalStatus.STATUS_SUCCEEDED
        assert response.result.reason_code == "SUCCEEDED"
        assert response.result.action_datagrams == 0
        trajectory = response.result.trajectory
        assert trajectory.header.frame_id == "machine_root_ros"
        assert trajectory.input_source == "fixture"
        assert trajectory.map_source == "fixture_empty"
        assert trajectory.clock_mode == "ros_clock"
        assert trajectory.trajectory_sha256 == trajectory_snapshot_message_sha256(
            trajectory
        )
        assert not trajectory.execution_eligible
        assert len(trajectory.waypoints) == 5
        assert trajectory.waypoints[0] == pose.pose.position
        assert trajectory.waypoints[-1] == build_goal(client_node).target.position

        deadline = time.monotonic() + 1.0
        while not any(path.poses for path in paths):
            assert time.monotonic() < deadline
            time.sleep(0.01)

        failed_goal = wait_future(
            client.send_goal_async(build_goal(client_node, target=(10.0, 0.0, 0.0)))
        )
        assert failed_goal.accepted
        pose.header.stamp = client_node.get_clock().now().to_msg()
        publisher.publish(pose)
        failed = wait_future(failed_goal.get_result_async())
        assert failed.status == GoalStatus.STATUS_ABORTED
        assert failed.result.reason_code == "PLANNING_FAILED"
        deadline = time.monotonic() + 1.0
        while paths and paths[-1].poses:
            assert time.monotonic() < deadline
            time.sleep(0.01)
        assert paths and not paths[-1].poses

        stale_goal = build_goal(client_node)
        stale_goal.target.header.stamp.sec = 0
        stale_goal.target.header.stamp.nanosec = 0
        stale = wait_future(client.send_goal_async(stale_goal))
        assert not stale.accepted
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown(context=context)
