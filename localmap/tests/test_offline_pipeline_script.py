import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "run_offline_localmap_pipeline.py"
SPEC = importlib.util.spec_from_file_location("run_offline_localmap_pipeline", SCRIPT_PATH)
pipeline = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = pipeline
SPEC.loader.exec_module(pipeline)


class OfflinePipelineScriptTest(unittest.TestCase):
    def test_pipeline_wires_extrinsics_and_bucket_tip_to_correct_steps(self):
        config = pipeline.PipelineConfig(
            bag=Path("bags/test_bag"),
            output_dir=Path("localmap/exports/test_bag"),
            python="/usr/bin/python3",
            extrinsics=Path("localmap/config/extrinsics.json"),
            bucket_tip=Path("localmap/config/bucket_tip.json"),
            targets=Path("localmap/config/targets.json"),
            topic="/rslidar_points",
            inspect_frames=3,
            max_csv_points=2000,
            storage_id="mcap",
            target_id="mock_dig_001",
            target_kind="dig",
            task_mode="MoveToDig",
            waypoint_count=5,
            current_index=0,
            up_axis="y",
            bounds=None,
            reuse_export=False,
            skip_inspect=False,
            dry_run=True,
        )

        commands = dict(pipeline.build_commands(config))

        self.assertIn("--extrinsics", commands["local_map"])
        self.assertIn("localmap/config/extrinsics.json", commands["local_map"])
        self.assertIn("--bucket-tip", commands["rrt_request"])
        self.assertIn("localmap/config/bucket_tip.json", commands["rrt_request"])
        self.assertNotIn("--bucket-tip", commands["trajectory"])
        self.assertIn("--bucket-tip", commands["observation"])

    def test_default_output_paths_follow_bag_name_and_topic(self):
        config = pipeline.PipelineConfig(
            bag=Path("bags/airy_case"),
            output_dir=Path("localmap/exports/airy_case"),
            python="python3",
            extrinsics=Path("extrinsics.json"),
            bucket_tip=Path("bucket_tip.json"),
            targets=Path("targets.json"),
            topic="/rslidar_points",
            inspect_frames=1,
            max_csv_points=10,
            storage_id="mcap",
            target_id="mock_dig_001",
            target_kind="dig",
            task_mode="MoveToDig",
            waypoint_count=5,
            current_index=0,
            up_axis="y",
            bounds=None,
            reuse_export=False,
            skip_inspect=True,
            dry_run=True,
        )

        paths = pipeline.output_paths(config)

        self.assertEqual(paths["npz"], Path("localmap/exports/airy_case/rslidar_points_first_frame.npz"))
        self.assertEqual(paths["local_map"], Path("localmap/exports/airy_case/local_map_machine_root.measured.json"))
        self.assertEqual(paths["observation"], Path("localmap/exports/airy_case/observation_waypoint_slice.machine_root.json"))


if __name__ == "__main__":
    unittest.main()
