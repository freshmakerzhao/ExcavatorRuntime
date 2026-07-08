import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from localmap_core.reachable_workspace import load_reachable_workspace
from localmap_core.simple_bucket_tip_planner import PlanningBounds, plan_bucket_tip_path


class ReachableWorkspaceTest(unittest.TestCase):
    def make_workspace_json(self) -> dict:
        return {
            "schema_version": "0.1.0",
            "machine_id": "test_excavator",
            "coordinate_frame": "machine_root",
            "workspaces": [
                {
                    "mode": "MoveToDig",
                    "volume_anchor_points": [
                        {"x": 0.0, "y": 1.0, "z": 0.0},
                        {"x": 0.0, "y": 0.0, "z": 0.0},
                        {"x": 0.0, "y": 1.0, "z": 1.0},
                        {"x": 0.0, "y": 0.0, "z": 1.0},
                        {"x": 1.0, "y": 1.0, "z": 0.0},
                        {"x": 1.0, "y": 0.0, "z": 0.0},
                        {"x": 1.0, "y": 1.0, "z": 1.0},
                        {"x": 1.0, "y": 0.0, "z": 1.0},
                    ],
                }
            ],
        }

    def test_loads_workspace_and_checks_points_in_machine_root(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "workspace.json"
            path.write_text(json.dumps(self.make_workspace_json()), encoding="utf-8")

            workspace = load_reachable_workspace(path, mode="MoveToDig")

        self.assertEqual(workspace.frame_id, "machine_root")
        self.assertTrue(workspace.contains(np.array([0.5, 0.5, 0.5])))
        self.assertTrue(workspace.contains(np.array([1.0, 0.0, 1.0])))
        self.assertFalse(workspace.contains(np.array([1.2, 0.5, 0.5])))
        self.assertFalse(workspace.contains(np.array([0.5, -0.1, 0.5])))

    def test_planner_keeps_bucket_tip_waypoints_inside_workspace(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "workspace.json"
            path.write_text(json.dumps(self.make_workspace_json()), encoding="utf-8")
            workspace = load_reachable_workspace(path, mode="MoveToDig")

        result = plan_bucket_tip_path(
            start=np.array([0.1, 0.5, 0.1]),
            goal=np.array([0.9, 0.5, 0.9]),
            obstacles=[],
            bounds=PlanningBounds.from_values([-1.0, 2.0, -1.0, 2.0, -1.0, 2.0]),
            reachable_workspace=workspace,
            waypoint_count=5,
            seed=3,
        )

        self.assertTrue(result.success, result.reason)
        for waypoint in result.waypoints:
            self.assertTrue(workspace.contains(waypoint), waypoint.tolist())

    def test_planner_rejects_goal_outside_workspace(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "workspace.json"
            path.write_text(json.dumps(self.make_workspace_json()), encoding="utf-8")
            workspace = load_reachable_workspace(path, mode="MoveToDig")

        result = plan_bucket_tip_path(
            start=np.array([0.1, 0.5, 0.1]),
            goal=np.array([1.5, 0.5, 0.5]),
            obstacles=[],
            reachable_workspace=workspace,
            seed=3,
        )

        self.assertFalse(result.success)
        self.assertEqual(result.reason, "goal_out_of_reachable_workspace")


if __name__ == "__main__":
    unittest.main()
