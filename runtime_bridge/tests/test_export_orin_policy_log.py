import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = PROJECT_ROOT / "runtime_bridge" / "apps" / "export_orin_policy_log_to_open_loop_csv.py"


class ExportOrinPolicyLogTest(unittest.TestCase):
    def _run_pc_journal(self, records, root):
        input_path = root / "pc_action_journal.jsonl"
        output_path = root / "output.csv"
        input_path.write_text(
            "".join(json.dumps(record) + "\n" for record in records),
            encoding="utf-8",
        )
        return subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "--input",
                str(input_path),
                "--input-format",
                "pc-journal",
                "--output",
                str(output_path),
            ],
            cwd=PROJECT_ROOT,
            check=False,
            capture_output=True,
            text=True,
        )

    def test_cli_exports_reference_compatible_velocity_rows_and_terminal_zero(self):
        source = (
            "2026-07-18 20:46:48,429 INFO orin_state_sender: STM32 TX policy_action "
            "seq=0: 2421635;0.016402572;-0.0172848548;-0.0053483555;0.0173053164\n"
            "2026-07-18 20:46:48,477 INFO orin_state_sender: STM32 TX policy_action "
            "seq=1: 2421685;-0.0185;-0.0357;-0.0419;-0.6\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "orin.log"
            output_path = root / "follow_dump.csv"
            input_path.write_text(source, encoding="utf-8")

            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--input",
                    str(input_path),
                    "--output",
                    str(output_path),
                    "--phase",
                    "TrackingToDump",
                    "--mode",
                    "CarryMaterial",
                ],
                cwd=PROJECT_ROOT,
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            with output_path.open(encoding="utf-8-sig", newline="") as csv_file:
                rows = list(csv.reader(line for line in csv_file if not line.startswith("#")))

        header, first, second, terminal = rows
        self.assertEqual(header[0:5], ["sample_index", "timestamp_s", "unity_time_s", "phase", "mode"])
        self.assertEqual(first[0:5], ["0", "0", "", "TrackingToDump", "CarryMaterial"])
        self.assertAlmostEqual(float(first[5]), 0.016402572 / 0.0351)
        self.assertAlmostEqual(float(first[6]), -0.0172848548 / 0.0357)
        self.assertAlmostEqual(float(first[9]), 0.016402572)
        self.assertEqual(first[13:], [""] * 14)
        self.assertEqual(second[1], "0.05")
        self.assertEqual([float(value) for value in second[5:9]], [-1.0, -1.0, -1.0, -1.0])
        self.assertEqual(terminal[1], "0.1")
        self.assertEqual([float(value) for value in terminal[5:13]], [0.0] * 8)

    def test_cli_exports_latest_pc_action_journal_session_for_fixed_action_replay(self):
        records = []
        for sequence, stamp_ms, bucket in (
            (10, 1000, -0.01),
            (11, 1050, 0.0),
            (12, 5000, 0.02),
            (13, 5050, 0.0),
        ):
            records.append(
                {
                    "schema": "pc_orin_action_send_v1",
                    "recorded_at_pc_ms": stamp_ms,
                    "source": "live_machine_behavior_server",
                    "sent_bytes": 200,
                    "packet": {
                        "type": "policy_action",
                        "schema_version": "1.0",
                        "seq": sequence,
                        "stamp_ms": stamp_ms,
                        "action_order": ["boom", "stick", "bucket", "swing"],
                        "action": [0.0, 0.0, bucket, 0.0],
                        "action_type": "normalized_velocity_command",
                        "valid_for_ms": 100,
                    },
                }
            )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "pc_action_journal.jsonl"
            output_path = root / "execute_dump.csv"
            input_path.write_text(
                "".join(json.dumps(record) + "\n" for record in records),
                encoding="utf-8",
            )

            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--input",
                    str(input_path),
                    "--input-format",
                    "pc-journal",
                    "--latest-session",
                    "--session-gap-ms",
                    "1000",
                    "--output",
                    str(output_path),
                    "--phase",
                    "ExecuteDump",
                    "--mode",
                    "FixedAction",
                ],
                cwd=PROJECT_ROOT,
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            with output_path.open(encoding="utf-8-sig", newline="") as csv_file:
                lines = csv_file.readlines()
            rows = list(csv.reader(line for line in lines if not line.startswith("#")))

        self.assertIn("# source=pc_action_journal\n", lines)
        self.assertEqual(len(rows), 4)
        self.assertEqual(rows[1][3:5], ["ExecuteDump", "FixedAction"])
        self.assertAlmostEqual(float(rows[1][11]), 0.02)
        self.assertEqual(rows[2][1], "0.05")
        self.assertEqual([float(value) for value in rows[-1][5:13]], [0.0] * 8)

    def test_pc_journal_allows_adjacent_commands_recorded_in_same_millisecond(self):
        def record(sequence, stamp_ms, boom):
            return {
                "schema": "pc_orin_action_send_v1",
                "recorded_at_pc_ms": stamp_ms,
                "source": "live_machine_behavior_server",
                "sent_bytes": 200,
                "packet": {
                    "type": "policy_action",
                    "seq": sequence,
                    "action_order": ["boom", "stick", "bucket", "swing"],
                    "action": [boom, 0.0, 0.0, 0.0],
                },
            }

        records = (
            record(20, 1000, 0.01),
            record(21, 1000, 0.02),
            record(22, 1050, 0.0),
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            completed = self._run_pc_journal(records, root)

            self.assertEqual(completed.returncode, 0, completed.stderr)
            with (root / "output.csv").open(encoding="utf-8-sig", newline="") as csv_file:
                rows = list(csv.reader(line for line in csv_file if not line.startswith("#")))

        self.assertEqual(rows[1][1], "0")
        self.assertEqual(rows[2][1], "0")
        self.assertEqual(rows[3][1], "0.05")

    def test_pc_journal_rejects_sequence_gaps_and_time_regression(self):
        def record(sequence, stamp_ms):
            return {
                "schema": "pc_orin_action_send_v1",
                "recorded_at_pc_ms": stamp_ms,
                "source": "live_machine_behavior_server",
                "sent_bytes": 200,
                "packet": {
                    "type": "policy_action",
                    "seq": sequence,
                    "action_order": ["boom", "stick", "bucket", "swing"],
                    "action": [0.01, 0.0, 0.0, 0.0],
                },
            }

        invalid_cases = (
            ([record(5, 1000), record(7, 1050)], "sequence gap"),
            ([record(5, 1000), record(6, 900)], "timestamps must increase"),
        )
        for records, expected_error in invalid_cases:
            with self.subTest(expected_error=expected_error), tempfile.TemporaryDirectory() as directory:
                completed = self._run_pc_journal(records, Path(directory))

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(expected_error, completed.stderr)

    def test_pc_journal_rejects_boolean_action_values(self):
        record = {
            "schema": "pc_orin_action_send_v1",
            "recorded_at_pc_ms": 1000,
            "source": "live_machine_behavior_server",
            "sent_bytes": 200,
            "packet": {
                "type": "policy_action",
                "seq": 5,
                "action_order": ["boom", "stick", "bucket", "swing"],
                "action": [True, 0.0, 0.0, 0.0],
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            completed = self._run_pc_journal([record], Path(directory))

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("non-numeric action value", completed.stderr)


if __name__ == "__main__":
    unittest.main()
