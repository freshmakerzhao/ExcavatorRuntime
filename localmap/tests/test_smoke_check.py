import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "apps" / "diagnostics" / "run_smoke_check.py"
SPEC = importlib.util.spec_from_file_location("run_smoke_check", SCRIPT_PATH)
smoke = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = smoke
SPEC.loader.exec_module(smoke)


class SmokeCheckTest(unittest.TestCase):
    def test_local_map_frame_must_match_machine_root(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "local_map.json"
            smoke.write_json_for_test(
                path,
                {
                    "schema_version": "local_map.v1",
                    "frame_id": "rslidar",
                    "ground": {"model": {"type": "plane"}},
                    "dig_targets": [],
                    "dump_targets": [],
                },
            )

            result = smoke.check_local_map_json(path, expected_frame="machine_root")

        self.assertEqual(result.status, "fail")
        self.assertIn("期望 machine_root", result.detail)

    def test_trajectory_requires_machine_root_waypoints(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.json"
            smoke.write_json_for_test(
                path,
                {
                    "schema_version": "trajectory_command.v1",
                    "frame_id": "machine_root",
                    "waypoints_base": [[0.0, 0.1, 0.2], [0.2, 0.3, 0.4]],
                },
            )

            result = smoke.check_trajectory_json(path, expected_frame="machine_root", required=True)

        self.assertEqual(result.status, "pass")
        self.assertIn("2 个 waypoint", result.detail)

    def test_smoke_report_fails_when_any_required_check_fails(self):
        results = [
            smoke.CheckResult("local_map", "pass", "ok"),
            smoke.CheckResult("trajectory", "warn", "缺少轨迹"),
            smoke.CheckResult("octomap", "fail", "没有 topic"),
        ]

        self.assertEqual(smoke.exit_code_for_results(results), 1)


if __name__ == "__main__":
    unittest.main()
