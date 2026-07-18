import unittest

from localmap_core.fixture_planning import FixturePlanningRequest, plan_fixture_trajectory


class FixturePlanningTests(unittest.TestCase):
    def test_empty_fixture_map_returns_reproducible_machine_root_ros_waypoints(self):
        request = FixturePlanningRequest(
            frame_id="machine_root_ros",
            input_source="fixture",
            map_source="fixture_empty",
            start_m=(0.2, -0.1, 0.3),
            start_stamp_s=10.0,
            target_id="dig-fixture",
            target_kind="dig",
            target_status="placeholder",
            target_m=(0.8, 0.2, 0.1),
            mission_id="mission-fixture",
            mission_sha256="a" * 64,
            mission_phase="dig",
            planning_scope="preview_global",
            created_at_s=10.1,
        )

        result = plan_fixture_trajectory(request, waypoint_count=5)

        self.assertTrue(result.success)
        self.assertEqual(result.reason, "straight_line")
        self.assertEqual(len(result.waypoints), 5)
        self.assertEqual(result.waypoints[0], request.start_m)
        self.assertEqual(result.waypoints[-1], request.target_m)
        self.assertEqual(result.task_mode, "MoveToDig")

    def test_fixture_planner_rejects_live_or_execution_claims(self):
        common = dict(
            frame_id="machine_root_ros",
            input_source="fixture",
            map_source="fixture_empty",
            start_m=(0.2, -0.1, 0.3),
            start_stamp_s=10.0,
            target_id="dig-fixture",
            target_kind="dig",
            target_status="placeholder",
            target_m=(0.8, 0.2, 0.1),
            mission_id="mission-fixture",
            mission_sha256="a" * 64,
            mission_phase="dig",
            planning_scope="preview_global",
            created_at_s=10.1,
        )
        with self.assertRaisesRegex(ValueError, "input_source"):
            FixturePlanningRequest(**{**common, "input_source": "live"})
        with self.assertRaisesRegex(ValueError, "planning_scope"):
            FixturePlanningRequest(**{**common, "planning_scope": "execution_strict"})


if __name__ == "__main__":
    unittest.main()
