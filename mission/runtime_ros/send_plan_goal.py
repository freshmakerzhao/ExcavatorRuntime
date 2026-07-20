#!/usr/bin/python3
"""Submit one typed Mission target to the non-motion Plan Action."""

from __future__ import annotations

import argparse
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from airy_excavator_interfaces.action import Plan
from geometry_msgs.msg import Point, Vector3
from rclpy.action import ActionClient
from rclpy.node import Node

from mission.contract import load_mission


def build_arg_parser() -> argparse.ArgumentParser:
    default_mission = (
        get_package_share_directory("airy_mission_runtime")
        + "/config/excavation_cycle.json"
    )
    parser = argparse.ArgumentParser(description="Plan one Mission phase without motion.")
    parser.add_argument("phase", choices=("dig", "dump"))
    parser.add_argument("--mission", default=default_mission)
    parser.add_argument("--wait-s", type=float, default=5.0)
    return parser


class PlanClient(Node):
    def __init__(self) -> None:
        super().__init__("mission_plan_client")
        self.client = ActionClient(self, Plan, "/planning/plan")


def run() -> int:
    args = build_arg_parser().parse_args()
    mission = load_mission(args.mission)
    target = mission.targets[args.phase]
    rclpy.init()
    node = PlanClient()
    try:
        if not node.client.wait_for_server(timeout_sec=args.wait_s):
            print("Plan failed: Action Server is unavailable", file=sys.stderr)
            return 2
        goal = Plan.Goal()
        goal.planning_scope = "preview_global"
        goal.target.header.frame_id = mission.frame_id
        goal.target.header.stamp = node.get_clock().now().to_msg()
        goal.target.target_id = f"{mission.mission_id}:{args.phase}"
        goal.target.target_kind = args.phase
        goal.target.target_status = mission.target_status
        goal.target.mission_id = mission.mission_id
        goal.target.mission_sha256 = mission.sha256
        goal.target.mission_phase = args.phase
        goal.target.position = Point(
            x=target.position_m[0], y=target.position_m[1], z=target.position_m[2]
        )
        goal.target.normal = Vector3(
            x=target.normal[0], y=target.normal[1], z=target.normal[2]
        )
        goal.target.radius_m = target.radius_m

        def feedback(message) -> None:
            print(
                f"feedback: stage={message.feedback.stage} "
                f"iterations={message.feedback.iterations}",
                flush=True,
            )

        send_future = node.client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=args.wait_s)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print("Plan failed: goal rejected", file=sys.stderr)
            return 2
        print("goal accepted", flush=True)
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.wait_s)
        if not result_future.done():
            print("Plan failed: result timeout", file=sys.stderr)
            return 2
        result = result_future.result().result
        print(
            f"result: {result.reason_code} trajectory_id={result.trajectory.trajectory_id} "
            f"waypoints={len(result.trajectory.waypoints)} "
            f"execution_eligible={result.trajectory.execution_eligible} "
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
