import sys
import unittest
from pathlib import Path
from types import MappingProxyType


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.planning_intent import resolve_planning_intent


class PlanningIntentTest(unittest.TestCase):
    def test_derives_target_kind_and_task_mode_from_target_id(self):
        local_map = {
            "dig_targets": [{"id": "dig_01"}],
            "dump_targets": [{"id": "dump_01"}],
        }
        task_modes = MappingProxyType(
            {"dig": "MoveToDig", "dump": "CarryMaterial"}
        )

        intent = resolve_planning_intent(local_map, "dig_01", task_modes)

        self.assertEqual(intent.target_id, "dig_01")
        self.assertEqual(intent.target_kind, "dig")
        self.assertEqual(intent.task_mode, "MoveToDig")

    def test_rejects_duplicate_target_id_within_same_collection(self):
        local_map = {
            "dig_targets": [{"id": "duplicate"}, {"id": "duplicate"}],
            "dump_targets": [],
        }
        task_modes = {"dig": "MoveToDig", "dump": "CarryMaterial"}

        with self.assertRaisesRegex(ValueError, "唯一"):
            resolve_planning_intent(local_map, "duplicate", task_modes)


if __name__ == "__main__":
    unittest.main()
