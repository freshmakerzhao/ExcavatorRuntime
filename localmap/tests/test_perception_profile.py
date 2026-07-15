import json
import sys
import tempfile
import unittest
from pathlib import Path


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.perception_profile import (
    DEFAULT_PERCEPTION_PROFILE,
    PerceptionProfileError,
    load_perception_profile,
    perception_stack_environment,
)


class PerceptionProfileTest(unittest.TestCase):
    def test_loads_canonical_live_profile(self):
        profile = load_perception_profile(DEFAULT_PERCEPTION_PROFILE)

        self.assertEqual(profile.profile_id, "scale_excavator_machine_root_ros")
        self.assertEqual(profile.expected_frame, "machine_root_ros")
        self.assertEqual(profile.topics.raw_cloud, "/rslidar_points")
        self.assertEqual(profile.topics.machine_cloud, "/localmap/machine_root_ros_points")
        self.assertEqual(profile.local_map.bounds, (-0.5, 4.0, -3.0, 1.5, -0.7, 1.2))
        self.assertEqual(profile.octomap.crop_bounds, (-0.5, 4.0, -3.0, 1.5, -0.7, 1.2))
        self.assertEqual(profile.octomap.reset_interval_s, 1.0)
        self.assertTrue(profile.inputs.rslidar_config.is_absolute())
        self.assertTrue(profile.outputs.live_local_map.is_absolute())

    def test_canonical_profile_is_right_handed_end_to_end(self):
        profile = load_perception_profile(DEFAULT_PERCEPTION_PROFILE)

        self.assertEqual(profile.topics.bucket_tip_fk, "/bucket_tip_pose_machine_root_ros")
        self.assertEqual(profile.topics.bucket_tip_machine_root, "/localmap/bucket_tip_machine_root_ros_pose")
        environment = perception_stack_environment(profile)
        self.assertEqual(environment["MACHINE_ROOT_FRAME"], "machine_root_ros")
        self.assertEqual(environment["LOCAL_MAP_UP_AXIS"], "z")
        self.assertEqual(environment["OCTOMAP_POINT_CLOUD_MIN_Z"], "-0.7")

    def test_rejects_unknown_profile_fields(self):
        data = self._default_data()
        data["unused_toggle"] = True

        with self.assertRaisesRegex(PerceptionProfileError, "未知字段"):
            self._load_temp(data)

    def test_rejects_invalid_frame_and_bounds(self):
        data = self._default_data()
        data["expected_frame"] = "map"

        with self.assertRaisesRegex(PerceptionProfileError, "machine_root_ros"):
            self._load_temp(data)

        data = self._default_data()
        data["octomap"]["crop_bounds"] = [-1.5, -1.5, -0.42, 1.0, -0.5, 4.0]

        with self.assertRaisesRegex(PerceptionProfileError, "最大值"):
            self._load_temp(data)

    def test_rejects_invalid_rates_and_octomap_settings(self):
        data = self._default_data()
        data["local_map"]["write_every"] = True

        with self.assertRaisesRegex(PerceptionProfileError, "write_every"):
            self._load_temp(data)

        data = self._default_data()
        data["octomap"]["resolution_m"] = 0.0

        with self.assertRaisesRegex(PerceptionProfileError, "resolution_m"):
            self._load_temp(data)

    def test_rejects_topic_alias_that_would_create_a_cloud_loop(self):
        data = self._default_data()
        data["topics"]["machine_cloud"] = data["topics"]["raw_cloud"]

        with self.assertRaisesRegex(PerceptionProfileError, "不能相同"):
            self._load_temp(data)

    @staticmethod
    def _default_data():
        return json.loads(DEFAULT_PERCEPTION_PROFILE.read_text(encoding="utf-8"))

    @staticmethod
    def _load_temp(data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "perception.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            return load_perception_profile(path)


if __name__ == "__main__":
    unittest.main()
