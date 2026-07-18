import sys
import unittest
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

from mission.contract import load_mission
from mission.markers import build_mission_marker_specs


class MissionMarkersTest(unittest.TestCase):
    def test_builds_distinct_dig_and_dump_marker_specs_from_same_snapshot(self):
        mission = load_mission(AIRY_ROOT / "mission" / "replays" / "mission.placeholder.json")

        specs = build_mission_marker_specs(mission)

        self.assertEqual([spec.phase for spec in specs], ["dig", "dump"])
        self.assertEqual(specs[0].position_m, (0.6, -0.3, -0.4))
        self.assertEqual(specs[1].position_m, (0.45, 0.3, 0.1))
        self.assertNotEqual(specs[0].color_rgba, specs[1].color_rgba)
        self.assertIn("PLACEHOLDER", specs[0].label)
        self.assertEqual(specs[0].frame_id, "machine_root_ros")


if __name__ == "__main__":
    unittest.main()
