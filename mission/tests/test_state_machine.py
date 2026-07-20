import sys
import unittest
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

from mission.state_machine import (
    MissionEvent,
    MissionState,
    MissionStateMachine,
    MissionTransitionError,
)


class MissionStateMachineTest(unittest.TestCase):
    def test_nominal_replay_requires_tracking_primitives_and_verification(self):
        machine = MissionStateMachine()
        events = (
            MissionEvent.START,
            MissionEvent.PLAN_SUCCEEDED,
            MissionEvent.TRACK_COMPLETED,
            MissionEvent.SETTLED,
            MissionEvent.PRIMITIVE_COMPLETED,
            MissionEvent.VERIFICATION_PASSED,
            MissionEvent.PLAN_SUCCEEDED,
            MissionEvent.TRACK_COMPLETED,
            MissionEvent.SETTLED,
            MissionEvent.PRIMITIVE_COMPLETED,
            MissionEvent.VERIFICATION_PASSED,
            MissionEvent.HOME_REACHED,
        )
        visited = []

        for event in events:
            machine, transition = machine.advance(event)
            visited.append(transition.to_state)

        self.assertEqual(
            visited,
            [
                MissionState.PLANNING_TO_DIG,
                MissionState.TRACKING_TO_DIG,
                MissionState.SETTLING_AFTER_DIG_TRACK,
                MissionState.DIGGING,
                MissionState.VERIFYING_LOAD,
                MissionState.PLANNING_TO_DUMP,
                MissionState.TRACKING_TO_DUMP,
                MissionState.SETTLING_AFTER_DUMP_TRACK,
                MissionState.DUMPING,
                MissionState.VERIFYING_EMPTY,
                MissionState.RETURNING_HOME,
                MissionState.COMPLETED,
            ],
        )

    def test_failure_is_terminal_and_cannot_skip_into_later_phase(self):
        machine = MissionStateMachine()
        machine, _ = machine.advance(MissionEvent.START)
        machine, _ = machine.advance(MissionEvent.PLAN_SUCCEEDED)

        machine, transition = machine.advance(
            MissionEvent.FAIL,
            reason="trajectory_timeout",
        )

        self.assertEqual(transition.to_state, MissionState.FAILED)
        self.assertEqual(transition.reason, "trajectory_timeout")
        with self.assertRaises(MissionTransitionError):
            machine.advance(MissionEvent.TRACK_COMPLETED)


if __name__ == "__main__":
    unittest.main()
