import sys
import unittest
import json
from pathlib import Path

import numpy as np


LOCALMAP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOCALMAP_ROOT))

from localmap_core.io import load_extrinsics


class RosFrameMigrationTest(unittest.TestCase):
    def test_right_handed_lidar_extrinsics_are_a_proper_rotation_into_machine_root_ros(self):
        ros = load_extrinsics(LOCALMAP_ROOT / "config" / "extrinsics_rslidar_to_machine_root_ros.derived.v1.json")

        self.assertEqual(ros.to_frame, "machine_root_ros")
        self.assertAlmostEqual(float(np.linalg.det(ros.linear_matrix)), 1.0)
        np.testing.assert_allclose(ros.linear_matrix @ ros.linear_matrix.T, np.identity(3))

    def test_right_handed_targets_are_explicitly_labeled(self):
        ros = json.loads(
            (LOCALMAP_ROOT / "config" / "targets.machine_root_ros.derived.v1.json").read_text(encoding="utf-8")
        )

        self.assertEqual(ros["frame_id"], "machine_root_ros")
        for key in ("dig_targets", "dump_targets"):
            for ros_target in ros[key]:
                position = np.asarray(ros_target["position_m"], dtype=np.float64)
                normal = np.asarray(ros_target["normal"], dtype=np.float64)
                self.assertEqual(position.shape, (3,))
                self.assertEqual(normal.shape, (3,))
                self.assertTrue(np.all(np.isfinite(position)))
                self.assertTrue(np.all(np.isfinite(normal)))


if __name__ == "__main__":
    unittest.main()
