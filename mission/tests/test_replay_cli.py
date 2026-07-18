import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]


class MissionReplayCliTest(unittest.TestCase):
    def test_cli_writes_auditable_shadow_result_without_motion_option(self):
        script = AIRY_ROOT / "mission" / "apps" / "run_mission_replay.py"
        mission = AIRY_ROOT / "mission" / "replays" / "mission.placeholder.json"
        replay = AIRY_ROOT / "mission" / "replays" / "nominal.placeholder.json"
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "mission_events.json"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--mission",
                    str(mission),
                    "--replay",
                    str(replay),
                    "--output",
                    str(output),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            data = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(data["schema_version"], "mission_replay_result.v1")
            self.assertEqual(data["final_state"], "completed")
            self.assertEqual(data["action_datagrams"], 0)
            self.assertEqual(data["mission_sha256"], data["mission_snapshot_sha256"])
            self.assertNotIn("enable_motion", completed.stdout)

    def test_cli_has_no_enable_motion_argument(self):
        script = AIRY_ROOT / "mission" / "apps" / "run_mission_replay.py"
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
