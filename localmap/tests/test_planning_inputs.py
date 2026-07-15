import json
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.planning_inputs import load_live_planning_inputs
from localmap_core.planning_profile import load_planning_profile


class PlanningInputsTest(unittest.TestCase):
    def test_loads_fresh_machine_root_inputs_as_immutable_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            local_map_path = root / "local_map.json"
            bucket_tip_path = root / "bucket_tip.json"
            local_map_path.write_text(
                json.dumps(
                    {
                        "schema_version": "local_map.v1",
                        "timestamp_s": 999.8,
                        "frame_id": "machine_root",
                        "dig_targets": [],
                        "dump_targets": [],
                    }
                ),
                encoding="utf-8",
            )
            bucket_tip_path.write_text(
                json.dumps(
                    {
                        "stamp_s": 999.9,
                        "frame_id": "machine_root",
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

            snapshot = load_live_planning_inputs(profile, now_s=1000.0)

        self.assertEqual(snapshot.local_map["frame_id"], "machine_root")
        self.assertEqual(snapshot.bucket_tip["position_m"], (0.1, 0.2, 0.3))
        with self.assertRaises(TypeError):
            snapshot.local_map["frame_id"] = "fake_base"

    def test_rejects_stale_live_bucket_tip(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            local_map_path = root / "local_map.json"
            bucket_tip_path = root / "bucket_tip.json"
            local_map_path.write_text(
                json.dumps(
                    {
                        "schema_version": "local_map.v1",
                        "timestamp_s": 999.8,
                        "frame_id": "machine_root",
                    }
                ),
                encoding="utf-8",
            )
            bucket_tip_path.write_text(
                json.dumps(
                    {
                        "stamp_s": 998.0,
                        "frame_id": "machine_root",
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

            with self.assertRaisesRegex(ValueError, "bucket_tip.stamp_s.*过期"):
                load_live_planning_inputs(profile, now_s=1000.0)

    def test_accepts_live_local_map_written_one_cycle_ago(self):
        """LocalMap按5帧落盘；正常调度抖动不能使一次规划随机失败。"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            local_map_path = root / "local_map.json"
            bucket_tip_path = root / "bucket_tip.json"
            local_map_path.write_text(
                json.dumps(
                    {
                        "schema_version": "local_map.v1",
                        # 真实失败样本：规划读取时LocalMap为529.6 ms旧。
                        "timestamp_s": 999.4704,
                        "frame_id": "machine_root",
                    }
                ),
                encoding="utf-8",
            )
            bucket_tip_path.write_text(
                json.dumps(
                    {
                        "stamp_s": 999.9,
                        "frame_id": "machine_root",
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

            snapshot = load_live_planning_inputs(profile, now_s=1000.0)

        self.assertEqual(snapshot.local_map["timestamp_s"], 999.4704)


if __name__ == "__main__":
    unittest.main()
