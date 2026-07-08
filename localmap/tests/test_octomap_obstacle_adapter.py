import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from localmap_core.octomap_obstacle_adapter import centers_to_obstacle_boxes


class OctomapObstacleAdapterTest(unittest.TestCase):
    def test_groups_nearby_voxel_centers_into_coarser_boxes(self):
        centers = np.array(
            [
                [0.01, 0.20, 0.01],
                [0.04, 0.22, 0.03],
                [0.52, 0.30, 0.01],
            ],
            dtype=np.float64,
        )

        obstacles = centers_to_obstacle_boxes(
            centers=centers,
            box_size_m=0.20,
            max_obstacles=100,
            source="lidar",
        )

        self.assertEqual(len(obstacles), 2)
        self.assertEqual(obstacles[0]["shape"], "box")
        self.assertEqual(obstacles[0]["size_m"], [0.2, 0.2, 0.2])

    def test_applies_bounds_before_grouping(self):
        centers = np.array([[0.0, 0.1, 0.0], [3.0, 0.1, 0.0]], dtype=np.float64)

        obstacles = centers_to_obstacle_boxes(
            centers=centers,
            box_size_m=0.20,
            bounds={"x": [-0.5, 0.5], "y": [0.0, 1.0], "z": [-0.5, 0.5]},
        )

        self.assertEqual(len(obstacles), 1)
        np.testing.assert_allclose(obstacles[0]["center_m"], [0.0, 0.1, 0.0], atol=0.11)


if __name__ == "__main__":
    unittest.main()
