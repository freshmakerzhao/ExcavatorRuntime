from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

import math

from excavator_kinematics.excavator_tf_node import (
    apply_joint_sign_and_offset,
    bucket_tip_observation_values,
    bucket_tip_pitch_rad_from_matrix,
    fk_root_position_to_unity,
    quat_from_rpy,
    transform_matrix,
)


class UnityBucketTipConversionTest(unittest.TestCase):
    def test_fk_root_position_maps_to_unity_left_handed_axes(self):
        # fk_root/ROS: +X前、+Y左、+Z上；Unity/machine_root: +X右、+Y上、+Z前。
        self.assertEqual(fk_root_position_to_unity((1.0, 2.0, 3.0)), (-2.0, 3.0, 1.0))

    def test_zero_position_stays_at_unity_origin(self):
        self.assertEqual(fk_root_position_to_unity((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))

    def test_bucket_tip_pitch_is_angle_between_tip_z_and_fk_root_z(self):
        identity_tip = transform_matrix((0.0, 0.0, 0.0), quat_from_rpy(0.0, 0.0, 0.0))
        right_angle_tip = transform_matrix((0.0, 0.0, 0.0), quat_from_rpy(0.0, math.pi / 2.0, 0.0))
        reversed_tip = transform_matrix((0.0, 0.0, 0.0), quat_from_rpy(0.0, math.pi, 0.0))

        self.assertAlmostEqual(bucket_tip_pitch_rad_from_matrix(identity_tip), 0.0)
        self.assertAlmostEqual(bucket_tip_pitch_rad_from_matrix(right_angle_tip), math.pi / 2.0)
        self.assertAlmostEqual(bucket_tip_pitch_rad_from_matrix(reversed_tip), math.pi)

    def test_bucket_tip_observation_values_include_unity_xyz_and_pitch(self):
        values = bucket_tip_observation_values((1.0, 2.0, 3.0), 0.42)

        self.assertEqual(values, [1.0, 2.0, 3.0, 0.42])

    def test_joint_sign_maps_sensor_angle_to_fk_angle_before_offset(self):
        self.assertEqual(apply_joint_sign_and_offset(0.5, -1.0, 0.1), -0.4)
        self.assertEqual(apply_joint_sign_and_offset(0.5, 1.0, 0.1), 0.6)


if __name__ == "__main__":
    unittest.main()
