import sys
import io
import json
import subprocess
import tempfile
import unittest
from contextlib import redirect_stdout
from dataclasses import replace
from pathlib import Path
from types import MappingProxyType
from unittest.mock import patch


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from apps.planning.run_planning_once import (
    build_arg_parser,
    build_planning_commands,
    execute_planning_commands,
    execute_prepared_run,
    PlanningCommand,
    PreparedPlanningRun,
    prepare_planning_run,
)
from localmap_core.planning_intent import PlanningIntent
from localmap_core.planning_inputs import LivePlanningInputs
from localmap_core.planning_profile import PlanningOutputs, load_planning_profile


class PlanningRunnerTest(unittest.TestCase):
    def test_cli_exposes_only_target_profile_and_dry_run(self):
        args = build_arg_parser().parse_args(["dig_01"])

        self.assertEqual(set(vars(args)), {"target_id", "profile", "dry_run"})
        self.assertEqual(args.target_id, "dig_01")
        self.assertFalse(args.dry_run)

    def test_profile_compiles_to_four_internal_planning_steps(self):
        profile = load_planning_profile()
        intent = PlanningIntent(
            target_id="dig_01",
            target_kind="dig",
            task_mode="MoveToDig",
        )

        commands = build_planning_commands(
            profile,
            intent,
            python=Path("/usr/bin/python3"),
        )

        self.assertEqual([command.name for command in commands], ["obstacles", "request", "trajectory", "observation"])
        by_name = {command.name: command.argv for command in commands}
        self.assertIn("-0.42", by_name["obstacles"])
        self.assertIn("--target-kind", by_name["request"])
        self.assertIn("dig", by_name["request"])
        self.assertIn("MoveToDig", by_name["request"])
        self.assertIn("-0.7", by_name["trajectory"])
        self.assertIn("--reachable-workspace", by_name["trajectory"])
        self.assertNotIn("--disable-reachable-workspace", by_name["trajectory"])
        self.assertIn("0.45", by_name["trajectory"])
        self.assertIn("--current-index", by_name["observation"])

    def test_prepares_commands_from_fresh_inputs_and_target_id_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            local_map_path = root / "local_map.json"
            bucket_tip_path = root / "bucket_tip.json"
            local_map_path.write_text(
                json.dumps(
                    {
                        "schema_version": "local_map.v1",
                        "timestamp_s": 999.8,
                        "frame_id": "machine_root_ros",
                        "dig_targets": [{"id": "dig_01"}],
                        "dump_targets": [],
                    }
                ),
                encoding="utf-8",
            )
            bucket_tip_path.write_text(
                json.dumps(
                    {
                        "stamp_s": 999.9,
                        "frame_id": "machine_root_ros",
                        "status": "live_from_tf",
                        "position_m": [0.1, 0.2, 0.3],
                    }
                ),
                encoding="utf-8",
            )
            profile = load_planning_profile()
            profile = replace(
                profile,
                inputs=replace(
                    profile.inputs,
                    live_local_map=local_map_path,
                    live_bucket_tip=bucket_tip_path,
                ),
            )

            staging_dir = root / "staging"
            prepared = prepare_planning_run(
                profile,
                "dig_01",
                now_s=1000.0,
                python=Path("/usr/bin/python3"),
                staging_dir=staging_dir,
            )

        by_name = dict((command.name, command.argv) for command in prepared.commands)
        request = by_name["request"]
        self.assertIn("dig", request)
        self.assertIn("MoveToDig", request)
        self.assertIn(str(staging_dir / "local_map.snapshot.json"), by_name["obstacles"])
        self.assertIn(str(staging_dir / "bucket_tip.snapshot.json"), request)
        self.assertNotIn(str(local_map_path), by_name["obstacles"])
        self.assertNotIn(str(bucket_tip_path), request)

    def test_dry_run_prints_steps_without_starting_subprocesses(self):
        commands = (PlanningCommand("example", ("python3", "step.py")),)
        output = io.StringIO()

        with patch("apps.planning.run_planning_once.subprocess.run") as run_process:
            with redirect_stdout(output):
                execute_planning_commands(commands, dry_run=True)

        run_process.assert_not_called()
        self.assertIn("[example] python3 step.py", output.getvalue())

    def test_failed_step_does_not_publish_partial_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            staging = root / "staging"
            final = root / "final"
            staging.mkdir()
            final.mkdir()

            def outputs(path):
                return PlanningOutputs(
                    directory=path,
                    local_map=path / "local_map.json",
                    request=path / "request.json",
                    trajectory=path / "trajectory.json",
                    observation_slice=path / "observation.json",
                )

            staging_outputs = outputs(staging)
            final_outputs = outputs(final)
            final_outputs.observation_slice.write_text("previous", encoding="utf-8")
            prepared = PreparedPlanningRun(
                commands=(
                    PlanningCommand("first", ("python3", "first.py")),
                    PlanningCommand("second", ("python3", "second.py")),
                ),
                snapshot=LivePlanningInputs(
                    local_map=MappingProxyType({"frame_id": "machine_root_ros"}),
                    bucket_tip=MappingProxyType({"frame_id": "machine_root_ros"}),
                ),
                local_map_snapshot=staging / "local_map.snapshot.json",
                bucket_tip_snapshot=staging / "bucket_tip.snapshot.json",
                staging_outputs=staging_outputs,
                final_outputs=final_outputs,
            )

            with patch(
                "apps.planning.run_planning_once.subprocess.run",
                side_effect=(None, subprocess.CalledProcessError(1, ["second.py"])),
            ):
                with self.assertRaises(subprocess.CalledProcessError):
                    execute_prepared_run(prepared, dry_run=False)

            self.assertFalse(final_outputs.local_map.exists())
            self.assertEqual(final_outputs.observation_slice.read_text(encoding="utf-8"), "previous")


if __name__ == "__main__":
    unittest.main()
