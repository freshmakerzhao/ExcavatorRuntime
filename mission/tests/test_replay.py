import sys
import unittest
from copy import deepcopy
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
MISSION_FIXTURE = AIRY_ROOT / "mission" / "replays" / "mission.placeholder.json"
sys.path.insert(0, str(AIRY_ROOT))

from mission.contract import load_mission
from mission.replay import run_mission_replay
from mission.state_machine import MissionState


def nominal_replay() -> dict:
    return {
        "schema_version": "mission_replay.v1",
        "frame_id": "machine_root_ros",
        "to_dig": {
            "waypoints": [[0.0, 0.0, 0.0], [0.6, -0.3, -0.4]],
            "tip_samples": [
                {"time_s": 0.0, "position_m": [0.0, 0.0, 0.0]},
                {"time_s": 0.3, "position_m": [0.0, 0.0, 0.0]},
                {"time_s": 0.4, "position_m": [0.6, -0.3, -0.4]},
                {"time_s": 0.7, "position_m": [0.6, -0.3, -0.4]},
            ],
        },
        "dig_primitive_result": "completed",
        "dig_settle_duration_s": 0.5,
        "load_verification_result": "passed",
        "post_dig_bucket_tip_m": [0.55, -0.25, -0.2],
        "to_dump": {
            "waypoints": [[0.55, -0.25, -0.2], [0.45, 0.3, 0.1]],
            "tip_samples": [
                {"time_s": 1.0, "position_m": [0.55, -0.25, -0.2]},
                {"time_s": 1.3, "position_m": [0.55, -0.25, -0.2]},
                {"time_s": 1.4, "position_m": [0.45, 0.3, 0.1]},
                {"time_s": 1.7, "position_m": [0.45, 0.3, 0.1]},
            ],
        },
        "dump_primitive_result": "completed",
        "dump_settle_duration_s": 0.5,
        "empty_verification_result": "passed",
        "return_home_result": "completed",
    }


class MissionReplayTest(unittest.TestCase):
    def test_nominal_replay_runs_full_cycle_without_action_datagrams(self):
        mission = load_mission(MISSION_FIXTURE)

        result = run_mission_replay(mission, nominal_replay())

        self.assertEqual(result.final_state, MissionState.COMPLETED)
        self.assertEqual(result.action_datagrams, 0)
        self.assertEqual(result.dump_plan_start_m, (0.55, -0.25, -0.2))
        self.assertEqual(len(result.transitions), 12)

    def test_tracking_timeout_fails_closed_before_dig_primitive(self):
        mission = load_mission(MISSION_FIXTURE)
        replay = deepcopy(nominal_replay())
        replay["to_dig"]["tip_samples"] = [
            {"time_s": 0.0, "position_m": [1.0, 1.0, 1.0]},
            {"time_s": 21.0, "position_m": [1.0, 1.0, 1.0]},
        ]

        result = run_mission_replay(mission, replay)

        self.assertEqual(result.final_state, MissionState.FAILED)
        self.assertEqual(result.action_datagrams, 0)
        self.assertEqual(len(result.transitions), 3)
        self.assertEqual(result.transitions[-1].reason, "track_to_dig_failed")

    def test_dump_plan_must_start_from_live_post_dig_tip(self):
        mission = load_mission(MISSION_FIXTURE)
        replay = deepcopy(nominal_replay())
        replay["to_dump"]["waypoints"][0] = [0.0, 0.0, 0.0]

        with self.assertRaisesRegex(ValueError, "dump start"):
            run_mission_replay(mission, replay)

    def test_settle_duration_must_satisfy_mission_limit(self):
        mission = load_mission(MISSION_FIXTURE)
        replay = deepcopy(nominal_replay())
        replay["dig_settle_duration_s"] = 0.49

        result = run_mission_replay(mission, replay)

        self.assertEqual(result.final_state, MissionState.FAILED)
        self.assertEqual(result.transitions[-1].reason, "dig_settle_failed")

    def test_malformed_tip_sample_is_contract_error(self):
        mission = load_mission(MISSION_FIXTURE)
        replay = deepcopy(nominal_replay())
        replay["to_dig"]["tip_samples"][0] = {"time_s": 0.0}

        with self.assertRaisesRegex(ValueError, "tip_samples"):
            run_mission_replay(mission, replay)

    def test_return_home_failure_prevents_mission_completion(self):
        mission = load_mission(MISSION_FIXTURE)
        replay = deepcopy(nominal_replay())
        replay["return_home_result"] = "failed"

        result = run_mission_replay(mission, replay)

        self.assertEqual(result.final_state, MissionState.FAILED)
        self.assertEqual(result.transitions[-1].reason, "return_home_failed")


if __name__ == "__main__":
    unittest.main()
