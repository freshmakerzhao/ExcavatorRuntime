"""Public contract for the URDF-model verification slider."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SLIDER_PATH = PACKAGE_ROOT / "scripts" / "waji_joint_slider.py"


def _load_slider_module():
    spec = spec_from_file_location("waji_joint_slider", SLIDER_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError("URDF slider script is not available")
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class JointSliderContractTest(unittest.TestCase):
    def test_slider_publishes_all_urdf_joint_names_in_radians(self):
        slider = _load_slider_module()

        message = slider.build_joint_state([0.1, -0.2, 0.3, -0.4])

        self.assertEqual(
            message.name,
            ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"],
        )
        self.assertEqual(list(message.position), [0.1, -0.2, 0.3, -0.4])


if __name__ == "__main__":
    unittest.main()
