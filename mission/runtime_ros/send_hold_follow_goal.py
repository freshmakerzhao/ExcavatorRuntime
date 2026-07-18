#!/usr/bin/python3
"""Send a one-waypoint Follow goal at the current Bucket Tip for offline validation."""

from __future__ import annotations

import argparse
import hashlib
import sys
import time

import rclpy
from airy_excavator_interfaces.action import Follow
from airy_excavator_interfaces.snapshot_digest import (
    trajectory_snapshot_message_sha256,
)
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

from mission.runtime_ros.follow_action_server import BUCKET_TIP_TOPIC, FOLLOW_ACTION


class HoldFollowClient(Node):
    def __init__(self, *, input_source: str) -> None:
        super().__init__("hold_follow_shadow_client")
        self.input_source = input_source
        self.latest_pose: PoseStamped | None = None
        self.create_subscription(PoseStamped, BUCKET_TIP_TOPIC, self._on_pose, 10)
        self.client = ActionClient(self, Follow, FOLLOW_ACTION)

    def _on_pose(self, message: PoseStamped) -> None:
        self.latest_pose = message

    def build_goal(self) -> Follow.Goal:
        if self.latest_pose is None:
            raise ValueError("Bucket Tip is unavailable")
        now = self.get_clock().now()
        position = self.latest_pose.pose.position
        goal = Follow.Goal()
        snapshot = goal.trajectory
        snapshot.header.frame_id = "machine_root_ros"
        snapshot.header.stamp = now.to_msg()
        snapshot.trajectory_id = f"hold-current-{now.nanoseconds}"
        snapshot.trajectory_sha256 = "0" * 64
        snapshot.mission_id = "offline-hold-follow"
        snapshot.mission_sha256 = hashlib.sha256(b"offline-hold-follow").hexdigest()
        snapshot.mission_phase = "dig"
        snapshot.task_mode = "MoveToDig"
        snapshot.planning_scope = "preview_global"
        snapshot.control_stage = "none"
        snapshot.workspace_constraint = "none"
        snapshot.execution_eligible = False
        snapshot.source_bucket_tip_stamp = self.latest_pose.header.stamp
        snapshot.source_local_map_stamp = now.to_msg()
        snapshot.inputs_frozen_at = now.to_msg()
        snapshot.valid_until = rclpy.time.Time(
            nanoseconds=now.nanoseconds + 10_000_000_000
        ).to_msg()
        snapshot.input_source = self.input_source
        snapshot.map_source = f"{self.input_source}_none"
        snapshot.clock_mode = "ros_clock"
        snapshot.waypoints = [
            Point(x=position.x, y=position.y, z=position.z)
        ]
        snapshot.waypoint_tolerance_m = 0.02
        snapshot.waypoint_dwell_s = 0.3
        snapshot.tracking_timeout_s = 5.0
        snapshot.trajectory_sha256 = trajectory_snapshot_message_sha256(snapshot)
        return goal


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate Follow with the current Bucket Tip.")
    parser.add_argument("--input-source", choices=("fixture", "replay", "live"), default="fixture")
    parser.add_argument("--wait-s", type=float, default=5.0)
    return parser


def run() -> int:
    args = build_arg_parser().parse_args()
    rclpy.init()
    node = HoldFollowClient(input_source=args.input_source)
    try:
        deadline = time.monotonic() + args.wait_s
        while node.latest_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest_pose is None:
            print("Follow diagnostic failed: Bucket Tip is unavailable", file=sys.stderr)
            return 2
        if not node.client.wait_for_server(timeout_sec=args.wait_s):
            print("Follow diagnostic failed: Action Server is unavailable", file=sys.stderr)
            return 2

        def feedback(message) -> None:
            value = message.feedback
            print(
                f"feedback: waypoint={value.current_waypoint_index + 1}/{value.waypoint_count} "
                f"distance_m={value.distance_m:.4f} action_datagrams={value.action_datagrams}",
                flush=True,
            )

        send_future = node.client.send_goal_async(node.build_goal(), feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=args.wait_s)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("Follow diagnostic failed: goal rejected", file=sys.stderr)
            return 2
        print("goal accepted", flush=True)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.wait_s + 2.0)
        if not result_future.done():
            print("Follow diagnostic failed: result timeout", file=sys.stderr)
            return 2
        result = result_future.result().result
        print(
            f"result: {result.reason_code} quiescence_confirmed={result.quiescence_confirmed} "
            f"action_datagrams={result.action_datagrams}",
            flush=True,
        )
        return 0 if result.reason_code == "SUCCEEDED" else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
