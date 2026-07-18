import json
import hashlib
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]


class LiveShadowCliTest(unittest.TestCase):
    def test_once_recomputes_observation_without_action_sender(self):
        script = AIRY_ROOT / "mission" / "apps" / "run_trajectory_shadow.py"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mission_path = AIRY_ROOT / "mission" / "replays" / "mission.placeholder.json"
            trajectory = root / "trajectory.json"
            bucket_tip = root / "bucket_tip.json"
            output = root / "shadow.json"
            trajectory.write_text(
                json.dumps(
                    {
                        "schema_version": "trajectory_command.v1",
                        "frame_id": "machine_root_ros",
                        "task_mode": "MoveToDig",
                        "waypoints_base": [[0.0, 0.0, 0.0], [0.6, -0.3, -0.4]],
                        "waypoint_count": 2,
                        "tube_radius": 0.04,
                        "planning_scope": "preview_global",
                        "execution_eligible": False,
                        "mission": {
                            "id": "replay_cycle_placeholder",
                            "sha256": hashlib.sha256(mission_path.read_bytes()).hexdigest(),
                            "phase": "dig",
                        },
                    }
                ),
                encoding="utf-8",
            )
            bucket_tip.write_text(
                json.dumps(
                    {
                        "frame_id": "machine_root_ros",
                        "status": "live_from_tf",
                        "stamp_s": time.time(),
                        "position_m": [0.1, 0.0, 0.0],
                    }
                ),
                encoding="utf-8",
            )

            completed = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--mission",
                    str(mission_path),
                    "--trajectory",
                    str(trajectory),
                    "--bucket-tip",
                    str(bucket_tip),
                    "--machine-profile",
                    str(AIRY_ROOT.parent / "shared" / "machine_profile.json"),
                    "--output",
                    str(output),
                    "--once",
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            result = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(result["mode"], "live_shadow_no_motion")
            self.assertEqual(result["action_datagrams"], 0)
            self.assertEqual(result["current_waypoint_index"], 0)
            self.assertEqual(result["phase"], "dig")
            self.assertEqual(len(result["observation_values_15_26"]), 12)

    def test_help_has_no_motion_option(self):
        script = AIRY_ROOT / "mission" / "apps" / "run_trajectory_shadow.py"
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0)
        self.assertNotIn("enable-motion", completed.stdout)


if __name__ == "__main__":
    unittest.main()
