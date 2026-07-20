import sys
import subprocess
import os
import signal
import time
import unittest
from pathlib import Path


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.replay_clock import select_cloud_header_stamp


class ReplayClockTest(unittest.TestCase):
    def test_replay_mode_uses_current_stamp_without_changing_live_default(self):
        recorded_stamp = object()
        current_stamp = object()

        self.assertIs(
            select_cloud_header_stamp(recorded_stamp, current_stamp, replay_restamp=False),
            recorded_stamp,
        )
        self.assertIs(
            select_cloud_header_stamp(recorded_stamp, current_stamp, replay_restamp=True),
            current_stamp,
        )

    def test_transform_cli_exposes_explicit_replay_only_restamp(self):
        script = LOCALMAP_DIR / "apps" / "perception" / "transform_live_cloud_to_base.py"

        completed = subprocess.run(
            [
                "bash",
                "-c",
                'source /opt/ros/jazzy/setup.bash && exec /usr/bin/python3 "$@"',
                "bash",
                str(script),
                "--help",
            ],
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--restamp-for-replay", completed.stdout)

    def test_perception_stack_rejects_restamp_with_real_lidar_driver(self):
        script = LOCALMAP_DIR / "scripts" / "run_perception_stack.sh"
        environment = {
            **os.environ,
            "RUN_RSLIDAR": "1",
            "REPLAY_RESTAMP_CLOUD": "1",
        }

        completed = subprocess.run(
            [str(script)],
            env=environment,
            check=False,
            capture_output=True,
            text=True,
            timeout=10.0,
        )

        self.assertEqual(completed.returncode, 2)
        self.assertIn("RUN_RSLIDAR=0", completed.stderr)

    def test_perception_stack_ctrl_c_is_a_clean_operator_shutdown(self):
        script = LOCALMAP_DIR / "scripts" / "run_perception_stack.sh"
        environment = {
            **os.environ,
            "RUN_RSLIDAR": "0",
            "RUN_TRANSFORM": "0",
            "RUN_LIVE_LOCAL_MAP": "0",
            "RUN_OCTOMAP": "0",
            "RUN_REACHABLE_WORKSPACE_MARKERS": "0",
            "RUN_TRAJECTORY_MARKERS": "0",
            "RUN_BUCKET_TIP_BRIDGE": "0",
        }

        process = subprocess.Popen(
            [str(script)],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            time.sleep(0.5)
            process.send_signal(signal.SIGINT)
            stdout, stderr = process.communicate(timeout=5.0)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5.0)

        self.assertEqual(process.returncode, 0, stdout + stderr)

    def test_perception_child_shutdown_is_bounded_when_term_is_ignored(self):
        lifecycle = (
            LOCALMAP_DIR / "apps" / "perception" / "process_lifecycle.sh"
        )
        completed = subprocess.run(
            [
                "bash",
                "-c",
                r"""
set -e
source "$1"
setsid /usr/bin/python3 -c 'import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)' &
child=$!
sleep 0.1
terminate_processes 1 "$child"
! kill -0 "$child" 2>/dev/null
""",
                "bash",
                str(lifecycle),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=4.0,
        )

        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)

    def test_perception_shutdown_terminates_descendant_process_group(self):
        lifecycle = (
            LOCALMAP_DIR / "apps" / "perception" / "process_lifecycle.sh"
        )
        completed = subprocess.run(
            [
                "bash",
                "-c",
                r"""
set -e
source "$1"
pid_file=$(mktemp)
setsid bash -c "/usr/bin/python3 -c 'import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)' & echo \$! > \"\$1\"; wait" bash "$pid_file" &
leader=$!
trap 'kill -KILL -- -"$leader" 2>/dev/null || true; rm -f "$pid_file"' EXIT
for _ in $(seq 1 20); do
  [[ -s "$pid_file" ]] && break
  sleep 0.05
done
descendant=$(cat "$pid_file")
terminate_processes 1 "$leader"
! kill -0 "$leader" 2>/dev/null
descendant_state=$(ps -o stat= -p "$descendant" 2>/dev/null || true)
[[ -z "$descendant_state" || "$descendant_state" == Z* ]]
""",
                "bash",
                str(lifecycle),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=4.0,
        )

        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)


if __name__ == "__main__":
    unittest.main()
