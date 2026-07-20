import json
import sys
import unittest
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

from mission.shadow_observation import advance_shadow_observation
from mission.trajectory_tracker import TrajectoryTracker


class ShadowObservationTest(unittest.TestCase):
    def test_recomputes_relative_waypoint_values_from_live_tip_and_tracker_index(self):
        machine_profile = json.loads(
            (AIRY_ROOT.parent / "shared" / "machine_profile.json").read_text(encoding="utf-8")
        )
        trajectory = {
            "schema_version": "trajectory_command.v1",
            "frame_id": "machine_root_ros",
            "waypoints_base": [[0.0, 0.0, 0.0], [0.6, -0.3, -0.4]],
            "waypoint_count": 2,
            "tube_radius": 0.04,
            "planning_scope": "preview_global",
            "execution_eligible": False,
        }
        tracker = TrajectoryTracker(
            waypoints=((0.0, 0.0, 0.0), (0.6, -0.3, -0.4)),
            tolerance_m=0.05,
            dwell_s=0.3,
            timeout_s=20.0,
        )

        tracker, first = advance_shadow_observation(
            tracker, trajectory, machine_profile, (0.0, 0.0, 0.0), now_s=0.0
        )
        tracker, second = advance_shadow_observation(
            tracker, trajectory, machine_profile, (0.0, 0.0, 0.0), now_s=0.3
        )
        tracker, third = advance_shadow_observation(
            tracker, trajectory, machine_profile, (0.4, -0.2, -0.2), now_s=0.4
        )

        self.assertEqual(first["current_waypoint_index"], 0)
        self.assertEqual(second["current_waypoint_index"], 1)
        self.assertEqual(third["current_waypoint_index"], 1)
        self.assertNotEqual(second["values"], third["values"])
        self.assertEqual(third["mode"], "shadow_no_motion")

    def test_rejects_frame_mismatch(self):
        tracker = TrajectoryTracker(((0.0, 0.0, 0.0),), 0.05, 0.3, 20.0)
        with self.assertRaisesRegex(ValueError, "frame"):
            advance_shadow_observation(
                tracker,
                {
                    "frame_id": "machine_root",
                    "waypoints_base": [[0.0, 0.0, 0.0]],
                    "waypoint_count": 1,
                },
                {},
                (0.0, 0.0, 0.0),
                now_s=0.0,
            )


if __name__ == "__main__":
    unittest.main()
