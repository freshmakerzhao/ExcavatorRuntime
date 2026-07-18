import sys
import unittest
from pathlib import Path

import numpy as np


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from apps.planning.generate_simple_rrt_trajectory_from_request import (
    mask_obstacles_near_points,
)


class PlannerObstacleMaskTest(unittest.TestCase):
    def test_masks_live_octomap_box_whose_inflated_shape_contains_start(self):
        start = np.array([0.5864810770938116, -0.38456999451502794, -0.44583250911661043])
        offending_box = {
            "id": "octomap_box_00393",
            "shape": "box",
            "center_m": [0.7, -0.5, -0.425],
            "size_m": [0.2, 0.2, 0.2],
            "source": "lidar",
        }
        self.assertGreater(np.linalg.norm(np.asarray(offending_box["center_m"]) - start), 0.15)

        filtered = mask_obstacles_near_points(
            [offending_box],
            [start],
            radius_m=0.15,
            collision_radius_m=0.05,
        )

        self.assertEqual(filtered, [])

    def test_mask_does_not_hide_non_lidar_semantic_obstacle(self):
        start = np.array([0.0, 0.0, 0.0])
        semantic_box = {
            "id": "keep_out",
            "shape": "box",
            "center_m": [0.0, 0.0, 0.0],
            "size_m": [0.2, 0.2, 0.2],
            "source": "configured_keep_out",
        }

        filtered = mask_obstacles_near_points(
            [semantic_box],
            [start],
            radius_m=0.15,
            collision_radius_m=0.05,
        )

        self.assertEqual(filtered, [semantic_box])


if __name__ == "__main__":
    unittest.main()
