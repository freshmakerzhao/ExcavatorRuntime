from pathlib import Path
import json
import subprocess
import sys
import tempfile
import unittest

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from localmap_core.bucket_tip_calibration import evaluate_bucket_tip_records


class BucketTipCalibrationTest(unittest.TestCase):
    def test_static_records_report_repeatable_machine_root_bias(self):
        report = evaluate_bucket_tip_records(
            {
                "schema": "bucket_tip_calibration.v1",
                "samples": [
                    {
                        "sample_id": "static-a",
                        "phase": "static",
                        "state_seq": 101,
                        "fk_tip_machine_root_m": [0.2, 0.3, 0.4],
                        "measured_tip_machine_root_m": [0.3, 0.28, 0.43],
                    },
                    {
                        "sample_id": "static-b",
                        "phase": "static",
                        "state_seq": 102,
                        "fk_tip_machine_root_m": [0.5, 0.1, 0.6],
                        "measured_tip_machine_root_m": [0.6, 0.08, 0.63],
                    },
                ],
            }
        )

        static = report["phases"]["static"]
        np.testing.assert_allclose(static["mean_error_m"], [0.1, -0.02, 0.03])
        self.assertAlmostEqual(static["max_error_norm_m"], np.linalg.norm([0.1, -0.02, 0.03]))
        self.assertEqual(report["quality"]["duplicate_state_sequences"], [])

    def test_cli_writes_a_replayable_report_without_touching_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture_path = root / "capture.json"
            report_path = root / "report.json"
            capture = {
                "schema": "bucket_tip_calibration.v1",
                "samples": [
                    {
                        "sample_id": "static-a",
                        "phase": "static",
                        "state_seq": 1,
                        "fk_tip_machine_root_m": [1.0, 2.0, 3.0],
                        "measured_tip_machine_root_m": [1.1, 2.0, 3.0],
                    }
                ],
            }
            capture_path.write_text(json.dumps(capture), encoding="utf-8")
            command = [
                sys.executable,
                str(Path(__file__).resolve().parents[1] / "apps" / "diagnostics" / "evaluate_bucket_tip_calibration.py"),
                "--capture",
                str(capture_path),
                "--output",
                str(report_path),
            ]

            subprocess.run(command, check=True, capture_output=True, text=True)

            self.assertEqual(json.loads(capture_path.read_text(encoding="utf-8")), capture)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertAlmostEqual(report["phases"]["static"]["mean_error_m"][0], 0.1)
            self.assertEqual(report["phases"]["static"]["mean_error_m"][1:], [0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
