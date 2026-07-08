import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from localmap_core.trajectory import (
    build_rrt_star_request,
    build_trajectory_command,
    build_waypoint_observation_slice,
)


class TrajectoryContractsTest(unittest.TestCase):
    def test_rrt_request_uses_local_map_targets_and_profile_thresholds(self):
        local_map = {
            "timestamp_s": 10.0,
            "frame_id": "machine_root",
            "ground": {"model": {"type": "plane", "normal": [0.0, 0.0, 1.0], "offset_m": 0.0}, "confidence": 0.5},
            "obstacles": [{"id": "obs_1", "shape": "box", "center_m": [1.0, 0.0, 0.2], "size_m": [0.2, 0.2, 0.4], "confidence": 0.8}],
            "dig_targets": [{"id": "dig_1", "position_m": [2.0, 0.0, 0.1], "normal": [0.0, 0.0, 1.0], "radius_m": 0.3, "confidence": 0.7}],
            "dump_targets": [],
        }
        profile = {
            "observation_schema": {
                "waypoint_lookahead": 3,
                "normalizers": {"tube_radius": 0.04, "target_threshold": 0.03},
            }
        }

        request = build_rrt_star_request(
            local_map=local_map,
            machine_profile=profile,
            bucket_tip_base=np.array([0.0, 0.0, 0.0]),
            target_id="dig_1",
            target_kind="dig",
            task_mode="MoveToDig",
        )

        self.assertEqual(request["frame_id"], "machine_root")
        self.assertEqual(request["start_bucket_tip_base"], [0.0, 0.0, 0.0])
        self.assertEqual(request["goal"]["id"], "dig_1")
        self.assertEqual(request["planning_params"]["target_threshold"], 0.03)
        self.assertEqual(request["planning_params"]["tube_radius"], 0.04)
        self.assertEqual(len(request["obstacles"]), 1)

    def test_trajectory_command_carries_waypoints_in_machine_root(self):
        waypoints = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.1], [2.0, 0.0, 0.2]], dtype=np.float64)
        command = build_trajectory_command(
            timestamp_s=10.0,
            frame_id="machine_root",
            task_mode="MoveToDig",
            target_bucket_pitch_deg=70.0,
            waypoints_base=waypoints,
            target_threshold=0.03,
            tube_radius=0.04,
        )

        self.assertEqual(command["frame_id"], "machine_root")
        self.assertEqual(command["waypoints_base"][2], [2.0, 0.0, 0.2])
        self.assertEqual(command["waypoint_count"], 3)

    def test_waypoints_enter_observation_indices_15_to_26(self):
        trajectory = {
            "frame_id": "machine_root",
            "waypoints_base": [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
            "waypoint_count": 3,
            "tube_radius": 0.5,
        }
        profile = {
            "observation_schema": {
                "total_dim": 38,
                "waypoint_lookahead": 3,
                "normalizers": {"distance_normalizer": 2.0, "tube_radius": 0.5},
            }
        }

        obs_slice = build_waypoint_observation_slice(
            trajectory_command=trajectory,
            machine_profile=profile,
            bucket_tip_base=np.array([0.5, 0.0, 0.0]),
            current_waypoint_index=1,
        )

        self.assertEqual(obs_slice["indices"], list(range(15, 27)))
        np.testing.assert_allclose(obs_slice["values"][0:9], [0.75, 0.0, 0.0, 1.25, 0.0, 0.0, 1.25, 0.0, 0.0])
        self.assertAlmostEqual(obs_slice["values"][9], 1.0 / 3.0)
        self.assertAlmostEqual(obs_slice["values"][10], 0.0)
        self.assertAlmostEqual(obs_slice["values"][11], 0.0)


if __name__ == "__main__":
    unittest.main()
