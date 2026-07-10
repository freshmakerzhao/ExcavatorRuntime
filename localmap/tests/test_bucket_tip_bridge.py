from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from localmap_core.bucket_tip_bridge import build_bucket_tip_state, load_bucket_tip_frame_bridge
from localmap_core.io import write_json


class BucketTipBridgeTest(unittest.TestCase):
    def test_fk_root_bucket_tip_position_maps_to_machine_root_axes(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge_path = Path(directory) / "bridge.json"
            write_json(
                bridge_path,
                {
                    "id": "test_bridge",
                    "source_frame": "fk_root",
                    "target_frame": "machine_root",
                    "translation_m": [0.0, 0.0, 0.0],
                    "axis_mapping_matrix": [
                        [0.0, -1.0, 0.0],
                        [0.0, 0.0, 1.0],
                        [1.0, 0.0, 0.0],
                    ],
                    "status": "test",
                },
            )

            bridge = load_bucket_tip_frame_bridge(bridge_path)
            # 关键：fk_root/ROS +X前/+Y左/+Z上 转成 machine_root +Z前/-X左/+Y上。
            position = bridge.transform_position(np.array([1.0, 2.0, 3.0]))

            np.testing.assert_allclose(position, np.array([-2.0, 3.0, 1.0]))

    def test_machine_root_offset_can_be_added_after_axis_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge_path = Path(directory) / "bridge.json"
            write_json(
                bridge_path,
                {
                    "id": "test_bridge_with_offset",
                    "source_frame": "fk_root",
                    "target_frame": "machine_root",
                    "translation_m": [0.1, 0.2, 0.3],
                    "axis_mapping_matrix": [
                        [0.0, -1.0, 0.0],
                        [0.0, 0.0, 1.0],
                        [1.0, 0.0, 0.0],
                    ],
                    "status": "test",
                },
            )

            bridge = load_bucket_tip_frame_bridge(bridge_path)
            position = bridge.transform_position(np.array([1.0, 2.0, 3.0]))

            np.testing.assert_allclose(position, np.array([-1.9, 3.2, 1.3]))

    def test_bucket_tip_state_matches_planning_json_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge_path = Path(directory) / "bridge.json"
            write_json(
                bridge_path,
                {
                    "id": "test_bridge",
                    "source_frame": "fk_root",
                    "target_frame": "machine_root",
                    "translation_m": [0.0, 0.0, 0.0],
                    "axis_mapping_matrix": [
                        [0.0, -1.0, 0.0],
                        [0.0, 0.0, 1.0],
                        [1.0, 0.0, 0.0],
                    ],
                    "status": "test",
                },
            )
            bridge = load_bucket_tip_frame_bridge(bridge_path)

            state = build_bucket_tip_state(
                position_m=np.array([0.4, 0.5, 0.6]),
                frame_id="machine_root",
                stamp_s=12.5,
                source_topic="/bucket_tip_pose_map",
                bridge=bridge,
            )

            self.assertEqual(state["frame_id"], "machine_root")
            self.assertEqual(state["position_m"], [0.4, 0.5, 0.6])
            self.assertEqual(state["source"]["topic"], "/bucket_tip_pose_map")

    def test_project_bridge_config_uses_fk_root_as_source_frame(self):
        bridge_path = Path(__file__).resolve().parents[1] / "config" / "bucket_tip_tf_bridge.machine_root.json"

        bridge = load_bucket_tip_frame_bridge(bridge_path)

        self.assertEqual(bridge.source_frame, "fk_root")
        self.assertEqual(bridge.target_frame, "machine_root")
        np.testing.assert_allclose(
            bridge.axis_mapping_matrix,
            np.array(
                [
                    [0.0, -1.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [1.0, 0.0, 0.0],
                ]
            ),
        )


if __name__ == "__main__":
    unittest.main()
