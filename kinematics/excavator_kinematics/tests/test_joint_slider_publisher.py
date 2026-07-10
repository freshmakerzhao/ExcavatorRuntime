from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from excavator_kinematics.joint_slider_publisher import DEFAULT_JOINT_NAMES, build_joint_state, format_angle


class JointSliderPublisherTest(unittest.TestCase):
    def test_build_joint_state_uses_expected_joint_order_and_radians(self):
        message = build_joint_state([0.1, 0.2, 0.3, 0.4])

        self.assertEqual(message.name, list(DEFAULT_JOINT_NAMES))
        self.assertEqual(list(message.position), [0.1, 0.2, 0.3, 0.4])

    def test_format_angle_shows_radian_and_degree(self):
        self.assertEqual(format_angle(1.5707963267948966), "1.571 rad / 90.0 deg")


if __name__ == "__main__":
    unittest.main()
