#!/usr/bin/python3
"""Send a ReturnHome shadow goal for offline JointState observation."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from airy_excavator_interfaces.action import ReturnHome
from rclpy.action import ActionClient
from rclpy.node import Node

from mission.home import load_named_joint_pose_set
from mission.runtime_ros.follow_action_server import RETURN_HOME_ACTION


class ReturnHomeClient(Node):
    def __init__(self) -> None:
        super().__init__("return_home_shadow_client")
        self.client = ActionClient(self, ReturnHome, RETURN_HOME_ACTION)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Observe a named home pose through the shadow-only ReturnHome Action."
    )
    parser.add_argument("--pose", default="transport_home")
    parser.add_argument("--wait-s", type=float, default=15.0)
    return parser


def run() -> int:
    args = build_arg_parser().parse_args()
    description_share = Path(get_package_share_directory("waji_description"))
    pose_set = load_named_joint_pose_set(
        description_share / "config/named_joint_poses.json",
        urdf_path=description_share / "urdf/waji.urdf",
    )
    if args.pose not in pose_set.poses:
        print(f"ReturnHome diagnostic failed: unknown pose {args.pose}", file=sys.stderr)
        return 2

    rclpy.init()
    node = ReturnHomeClient()
    try:
        if not node.client.wait_for_server(timeout_sec=args.wait_s):
            print("ReturnHome diagnostic failed: Action Server is unavailable", file=sys.stderr)
            return 2
        goal = ReturnHome.Goal()
        goal.home_pose_id = args.pose
        goal.pose_set_sha256 = pose_set.sha256

        def feedback(message) -> None:
            value = message.feedback
            print(
                f"feedback: state={value.state} max_error_rad={value.max_error_rad:.5f} "
                f"elapsed_s={value.elapsed_s:.2f} action_datagrams={value.action_datagrams}",
                flush=True,
            )

        send_future = node.client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=args.wait_s)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("ReturnHome diagnostic failed: goal rejected", file=sys.stderr)
            return 2
        print(
            f"goal accepted: pose={args.pose} status={pose_set.poses[args.pose].status} "
            "execution_mode=shadow",
            flush=True,
        )
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.wait_s)
        if not result_future.done():
            print("ReturnHome diagnostic failed: result timeout", file=sys.stderr)
            return 2
        result = result_future.result().result
        print(
            f"result: {result.reason_code} max_error_rad={result.final_max_error_rad:.5f} "
            f"quiescence_confirmed={result.quiescence_confirmed} "
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
