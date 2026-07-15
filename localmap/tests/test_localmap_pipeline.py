import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from localmap_core.generator import build_local_map
from localmap_core.geometry import Transform, preprocess_points, transform_xyzirt_points
from localmap_core.io import load_json, write_json


class LocalMapPipelineTest(unittest.TestCase):
    def test_preprocess_transforms_finite_points_into_machine_root(self):
        points = np.array(
            [
                [1.0, 0.0, 0.0, 10.0, 0.0, 100.0],
                [np.nan, 0.0, 0.0, 20.0, 1.0, 101.0],
                [3.0, 0.0, 0.0, 30.0, 2.0, 102.0],
            ],
            dtype=np.float64,
        )
        transform = Transform(
            from_frame="rslidar",
            to_frame="machine_root_ros",
            translation_m=np.array([1.0, 2.0, 3.0]),
            rotation_rpy_rad=np.array([0.0, 0.0, 0.0]),
            identifier="test",
            status="mock",
        )

        transformed = preprocess_points(
            points,
            transform,
            bounds={"x": [0.0, 3.0], "y": [0.0, 3.0], "z": [0.0, 4.0]},
        )

        self.assertEqual(transformed.shape, (1, 6))
        np.testing.assert_allclose(transformed[0, :3], [2.0, 2.0, 3.0])

    def test_transform_preserves_non_xyz_channels(self):
        points = np.array([[1.0, 2.0, 3.0, 42.0, 7.0, 123.0]], dtype=np.float64)
        transform = Transform(
            from_frame="rslidar",
            to_frame="machine_root_ros",
            translation_m=np.array([0.5, 0.0, -0.5]),
            rotation_rpy_rad=np.array([0.0, 0.0, 0.0]),
            identifier="test",
            status="mock",
        )

        transformed = transform_xyzirt_points(points, transform)

        np.testing.assert_allclose(transformed[0, :3], [1.5, 2.0, 2.5])
        np.testing.assert_allclose(transformed[0, 3:], [42.0, 7.0, 123.0])

    def test_axis_mapping_matrix_supports_unity_machine_root_axes(self):
        points = np.array([[2.0, 3.0, 4.0, 42.0, 7.0, 123.0]], dtype=np.float64)
        transform = Transform(
            from_frame="rslidar",
            to_frame="machine_root_ros",
            translation_m=np.array([1.0, 10.0, 100.0]),
            rotation_rpy_rad=np.array([0.0, 0.0, 0.0]),
            identifier="axis_mapping_test",
            status="measured",
            linear_matrix=np.array(
                [
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0],
                    [0.0, -1.0, 0.0],
                ],
                dtype=np.float64,
            ),
        )

        transformed = transform_xyzirt_points(points, transform)

        np.testing.assert_allclose(transformed[0, :3], [3.0, 6.0, 97.0])
        np.testing.assert_allclose(transformed[0, 3:], [42.0, 7.0, 123.0])

    def test_build_local_map_writes_schema_shaped_json(self):
        points_base = np.array(
            [
                [0.0, 0.0, 0.1, 1.0, 0.0, 100.0],
                [1.0, 0.0, 0.2, 2.0, 1.0, 101.0],
                [2.0, 0.0, 0.3, 3.0, 2.0, 102.0],
            ],
            dtype=np.float64,
        )
        transform = Transform(
            from_frame="rslidar",
            to_frame="machine_root_ros",
            translation_m=np.array([0.0, 0.0, 0.0]),
            rotation_rpy_rad=np.array([0.0, 0.0, 0.0]),
            identifier="test",
            status="mock",
        )
        targets = {
            "dig_targets": [
                {
                    "id": "dig_test",
                    "position_m": [1.0, 0.0, 0.0],
                    "normal": [0.0, 0.0, 1.0],
                    "radius_m": 0.2,
                    "confidence": 0.5,
                }
            ],
            "dump_targets": [],
        }

        local_map = build_local_map(
            points_base=points_base,
            timestamp_s=100.0,
            raw_topic="/rslidar_points",
            raw_frame_id="rslidar",
            raw_point_type="XYZIRT",
            bag_path="bags/test",
            transform=transform,
            targets=targets,
        )

        self.assertEqual(local_map["schema_version"], "local_map.v1")
        self.assertEqual(local_map["frame_id"], "machine_root_ros")
        self.assertEqual(local_map["source"]["extrinsics"]["id"], "test")
        self.assertEqual(local_map["dig_targets"][0]["id"], "dig_test")

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "local_map.json"
            write_json(path, local_map)
            loaded = load_json(path)

        self.assertEqual(loaded["ground"]["model"]["type"], "plane")
        self.assertEqual(loaded["ground"]["model"]["normal"], [0.0, 1.0, 0.0])
        self.assertEqual(loaded["source"]["raw_point_type"], "XYZIRT")


if __name__ == "__main__":
    unittest.main()
