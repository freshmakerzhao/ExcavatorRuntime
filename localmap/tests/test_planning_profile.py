import json
import sys
import tempfile
import unittest
from pathlib import Path


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.planning_profile import load_planning_profile


def valid_profile_payload():
    return {
        "schema": "planning_profile_v1",
        "profile_id": "scale_excavator_live",
        "expected_frame": "machine_root",
        "inputs": {
            "live_local_map": "localmap/exports/live_latest/local_map.live.json",
            "live_bucket_tip": "localmap/exports/live_latest/bucket_tip.machine_root.live.json",
            "octomap_topic": "/occupied_cells_vis_array",
            "machine_profile": "../shared/machine_profile.json",
            "reachable_workspace": "../shared/reachable_workspaces/scale_excavator_workspace.json",
        },
        "output_dir": "localmap/exports/live_latest",
        "freshness": {
            "local_map_max_age_ms": 500,
            "bucket_tip_max_age_ms": 500,
            "octomap_timeout_s": 5.0,
        },
        "obstacle_adapter": {
            "bounds": [-1.5, 3.0, -0.42, 1.0, -0.5, 4.0],
            "box_size_m": 0.2,
            "max_obstacles": 1000,
        },
        "planner": {
            "bounds": [-1.5, 3.0, -0.7, 1.0, -0.5, 4.0],
            "collision_radius_m": 0.05,
            "step_size_m": 0.2,
            "edge_check_step_m": 0.04,
            "max_iterations": 6000,
            "goal_sample_rate": 0.15,
            "start_mask_radius_m": 0.15,
            "goal_mask_radius_m": 0.45,
            "waypoint_count": 5,
            "seed": 8,
        },
        "task_mode_by_target_kind": {
            "dig": "MoveToDig",
            "dump": "CarryMaterial",
        },
    }


class PlanningProfileTest(unittest.TestCase):
    def test_loads_profile_and_derives_internal_artifact_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            profile_path = project_root / "planning.json"
            profile_path.write_text(json.dumps(valid_profile_payload()), encoding="utf-8")

            profile = load_planning_profile(profile_path, project_root=project_root)

        self.assertEqual(profile.profile_id, "scale_excavator_live")
        self.assertEqual(profile.expected_frame, "machine_root")
        self.assertEqual(profile.inputs.live_local_map, project_root / "localmap/exports/live_latest/local_map.live.json")
        self.assertEqual(profile.outputs.trajectory.name, "trajectory_command.simple_rrt.json")
        self.assertEqual(profile.outputs.observation_slice.name, "observation_waypoint_slice.simple_rrt.json")
        self.assertEqual(profile.planner.bounds, (-1.5, 3.0, -0.7, 1.0, -0.5, 4.0))
        self.assertEqual(profile.planner.goal_mask_radius_m, 0.45)
        self.assertEqual(profile.planner.max_iterations, 6000)
        self.assertEqual(profile.task_mode_by_target_kind["dump"], "CarryMaterial")

    def test_rejects_unknown_profile_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            profile_data = valid_profile_payload()
            profile_data["planner"]["max_iteration"] = 10
            profile_path = project_root / "planning.json"
            profile_path.write_text(json.dumps(profile_data), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "planner.*未知字段.*max_iteration"):
                load_planning_profile(profile_path, project_root=project_root)

    def test_rejects_unsupported_profile_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            profile_data = valid_profile_payload()
            profile_data["schema"] = "planning_profile_v0"
            profile_path = project_root / "planning.json"
            profile_path.write_text(json.dumps(profile_data), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "schema.*planning_profile_v1"):
                load_planning_profile(profile_path, project_root=project_root)

    def test_rejects_invalid_obstacle_bounds(self):
        invalid_bounds = (
            [-1.0, 1.0, 0.5, 0.5, -1.0, 1.0],
            [-1.0, True, 0.0, 1.0, -1.0, 1.0],
        )
        for bounds in invalid_bounds:
            with self.subTest(bounds=bounds), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                profile_data = valid_profile_payload()
                profile_data["obstacle_adapter"]["bounds"] = bounds
                profile_path = project_root / "planning.json"
                profile_path.write_text(json.dumps(profile_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, "obstacle_adapter.bounds"):
                    load_planning_profile(profile_path, project_root=project_root)

    def test_rejects_invalid_numeric_settings(self):
        invalid_cases = (
            (("freshness", "local_map_max_age_ms"), True, "freshness.local_map_max_age_ms"),
            (("obstacle_adapter", "box_size_m"), 0.0, "obstacle_adapter.box_size_m"),
            (("planner", "goal_sample_rate"), 1.5, "planner.goal_sample_rate"),
            (("planner", "max_iterations"), 0, "planner.max_iterations"),
            (("planner", "waypoint_count"), True, "planner.waypoint_count"),
        )
        for field_path, invalid_value, expected_error in invalid_cases:
            with self.subTest(field=expected_error), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                profile_data = valid_profile_payload()
                profile_data[field_path[0]][field_path[1]] = invalid_value
                profile_path = project_root / "planning.json"
                profile_path.write_text(json.dumps(profile_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, expected_error):
                    load_planning_profile(profile_path, project_root=project_root)

    def test_rejects_wrong_frame_or_task_mapping(self):
        invalid_cases = (
            (("expected_frame",), "fake_base", "expected_frame"),
            (("task_mode_by_target_kind", "dig"), "CarryMaterial", "task_mode_by_target_kind"),
        )
        for field_path, invalid_value, expected_error in invalid_cases:
            with self.subTest(field=field_path), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                profile_data = valid_profile_payload()
                if len(field_path) == 1:
                    profile_data[field_path[0]] = invalid_value
                else:
                    profile_data[field_path[0]][field_path[1]] = invalid_value
                profile_path = project_root / "planning.json"
                profile_path.write_text(json.dumps(profile_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, expected_error):
                    load_planning_profile(profile_path, project_root=project_root)


if __name__ == "__main__":
    unittest.main()
