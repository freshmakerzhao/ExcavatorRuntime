from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from localmap_core.bucket_tip_bridge import build_bucket_tip_state, load_bucket_tip_frame_bridge
from localmap_core.io import write_json


class BucketTipBridgeTest(unittest.TestCase):
    def test_right_handed_bucket_tip_bridge_preserves_position(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge_path = Path(directory) / "bridge.json"
            write_json(
                bridge_path,
                {
                    "id": "test_bridge",
                    "source_frame": "machine_root_ros",
                    "target_frame": "machine_root_ros",
                    "translation_m": [0.0, 0.0, 0.0],
                    "axis_mapping_matrix": [
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                    "status": "test",
                },
            )

            bridge = load_bucket_tip_frame_bridge(bridge_path)
            position = bridge.transform_position(np.array([1.0, 2.0, 3.0]))

            np.testing.assert_allclose(position, np.array([1.0, 2.0, 3.0]))

    def test_machine_root_ros_offset_can_be_added_after_identity_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge_path = Path(directory) / "bridge.json"
            write_json(
                bridge_path,
                {
                    "id": "test_bridge_with_offset",
                    "source_frame": "machine_root_ros",
                    "target_frame": "machine_root_ros",
                    "translation_m": [0.1, 0.2, 0.3],
                    "axis_mapping_matrix": [
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                    "status": "test",
                },
            )

            bridge = load_bucket_tip_frame_bridge(bridge_path)
            position = bridge.transform_position(np.array([1.0, 2.0, 3.0]))

            np.testing.assert_allclose(position, np.array([1.1, 2.2, 3.3]))

    def test_bucket_tip_state_matches_planning_json_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge_path = Path(directory) / "bridge.json"
            write_json(
                bridge_path,
                {
                    "id": "test_bridge",
                    "source_frame": "machine_root_ros",
                    "target_frame": "machine_root_ros",
                    "translation_m": [0.0, 0.0, 0.0],
                    "axis_mapping_matrix": [
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                    "status": "test",
                },
            )
            bridge = load_bucket_tip_frame_bridge(bridge_path)

            state = build_bucket_tip_state(
                position_m=np.array([0.4, 0.5, 0.6]),
                frame_id="machine_root_ros",
                stamp_s=12.5,
                source_topic="/bucket_tip_pose_machine_root_ros",
                bridge=bridge,
            )

            self.assertEqual(state["frame_id"], "machine_root_ros")
            self.assertEqual(state["position_m"], [0.4, 0.5, 0.6])
            self.assertEqual(state["source"]["topic"], "/bucket_tip_pose_machine_root_ros")

    def test_project_bridge_config_is_machine_root_ros_identity(self):
        bridge_path = Path(__file__).resolve().parents[1] / "config" / "bucket_tip_tf_bridge.machine_root_ros.identity.v1.json"

        bridge = load_bucket_tip_frame_bridge(bridge_path)

        self.assertEqual(bridge.source_frame, "machine_root_ros")
        self.assertEqual(bridge.target_frame, "machine_root_ros")
        np.testing.assert_allclose(
            bridge.axis_mapping_matrix,
            np.array(
                [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ]
            ),
        )


if __name__ == "__main__":
    unittest.main()
