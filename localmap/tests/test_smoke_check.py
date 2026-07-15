import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "apps" / "diagnostics" / "run_smoke_check.py"
SPEC = importlib.util.spec_from_file_location("run_smoke_check", SCRIPT_PATH)
smoke = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = smoke
SPEC.loader.exec_module(smoke)


class SmokeCheckTest(unittest.TestCase):
    def test_run_planning_option_requires_explicit_target_id(self):
        args = smoke.build_arg_parser().parse_args(["--run-planning", "mock_dig_001"])

        self.assertEqual(args.run_planning, "mock_dig_001")

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

            result = smoke.check_local_map_json(path, expected_frame="machine_root_ros")

        self.assertEqual(result.status, "fail")
        self.assertIn("期望 machine_root", result.detail)

    def test_trajectory_requires_machine_root_waypoints(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.json"
            smoke.write_json_for_test(
                path,
                {
                    "schema_version": "trajectory_command.v1",
                    "frame_id": "machine_root_ros",
                    "waypoints_base": [[0.0, 0.1, 0.2], [0.2, 0.3, 0.4]],
                },
            )

            result = smoke.check_trajectory_json(path, expected_frame="machine_root_ros", required=True)

        self.assertEqual(result.status, "pass")
        self.assertIn("2 个 waypoint", result.detail)

    def test_live_bucket_tip_is_required(self):
        result = smoke.check_bucket_tip_json(
            Path("/definitely/missing/bucket_tip.json"),
            expected_frame="machine_root_ros",
        )

        self.assertEqual(result.status, "fail")

    def test_bucket_tip_placeholder_is_not_healthy(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bucket_tip.json"
            smoke.write_json_for_test(
                path,
                {
                    "frame_id": "machine_root_ros",
                    "position_m": [0.1, 0.2, 0.3],
                    "status": "placeholder",
                },
            )

            result = smoke.check_bucket_tip_json(path, expected_frame="machine_root_ros")

        self.assertEqual(result.status, "fail")
        self.assertIn("live_from_tf", result.detail)

    def test_bucket_tip_rejects_non_finite_position(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bucket_tip.json"
            smoke.write_json_for_test(
                path,
                {
                    "frame_id": "machine_root_ros",
                    "position_m": [0.1, math.nan, 0.3],
                    "stamp_s": 1.0,
                    "status": "live_from_tf",
                },
            )

            result = smoke.check_bucket_tip_json(path, expected_frame="machine_root_ros")

        self.assertEqual(result.status, "fail")

    def test_smoke_report_fails_when_any_required_check_fails(self):
        results = [
            smoke.CheckResult("local_map", "pass", "ok"),
            smoke.CheckResult("trajectory", "warn", "缺少轨迹"),
            smoke.CheckResult("octomap", "fail", "没有 topic"),
        ]

        self.assertEqual(smoke.exit_code_for_results(results), 1)

    def test_planning_runner_receives_target_id(self):
        completed = smoke.subprocess.CompletedProcess([], returncode=0, stdout="", stderr="")

        with mock.patch.object(smoke, "run_process", return_value=completed) as run_process:
            result = smoke.run_planning_once("mock_dig_001")

        command = run_process.call_args.args[0]
        self.assertEqual(command[-1], "mock_dig_001")
        self.assertEqual(result.status, "pass")


if __name__ == "__main__":
    unittest.main()
