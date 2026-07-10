import json
import tempfile
import unittest
from pathlib import Path

from runtime_bridge.runtime_config import load_runtime_config


def valid_config_payload():
    return {
        "schema": "runtime_bridge_config_v2",
        "network": {
            "state_bind_host": "0.0.0.0",
            "state_port": 18081,
            "orin_host": "192.168.2.88",
            "action_port": 18082,
            "action_valid_ms": 100,
            "action_time_source": "orin",
        },
        "artifacts": {
            "onnx": "model.onnx",
            "machine_profile": "machine_profile.json",
            "waypoint_slice": "waypoint_slice.json",
            "latest_observation": "exports/latest_observation.json",
        },
        "policy": {"bucket_tip_timeout_ms": 500},
        "fixed_action": {
            "kp": 1.5,
            "min_action": 0.08,
            "max_action": 1.0,
            "tolerance": 0.03,
            "step_timeout_s": 3.0,
            "hold_s": 0.15,
        },
        "diagnostics": {"print_every": 1, "write_every": 5},
    }


class RuntimeConfigTest(unittest.TestCase):
    def test_loads_runtime_settings_and_resolves_artifact_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            for relative_path in ("model.onnx", "machine_profile.json", "waypoint_slice.json"):
                (project_root / relative_path).touch()
            config_path = project_root / "runtime.json"
            config_path.write_text(
                json.dumps(valid_config_payload()),
                encoding="utf-8",
            )

            config = load_runtime_config(config_path, project_root=project_root)

        self.assertEqual(config.network.state_endpoint, ("0.0.0.0", 18081))
        self.assertEqual(config.network.action_endpoint, ("192.168.2.88", 18082))
        self.assertEqual(config.network.action_valid_ms, 100)
        self.assertEqual(config.network.action_time_source, "orin")
        self.assertEqual(config.artifacts.onnx, project_root / "model.onnx")
        self.assertEqual(config.artifacts.latest_observation, project_root / "exports/latest_observation.json")
        self.assertEqual(config.policy.bucket_tip_timeout_ms, 500)
        self.assertEqual(config.fixed_action.kp, 1.5)
        self.assertEqual(config.fixed_action.step_timeout_s, 3.0)
        self.assertEqual(config.diagnostics.write_every, 5)

    def test_rejects_unknown_fields_instead_of_silently_ignoring_them(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            config_data = valid_config_payload()
            config_data["network"]["state_poort"] = 18081
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(config_data), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "network.*未知字段"):
                load_runtime_config(config_path, project_root=project_root)

    def test_rejects_unsupported_config_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            config_data = valid_config_payload()
            config_data["schema"] = "runtime_bridge_config_v0"
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(config_data), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "schema"):
                load_runtime_config(config_path, project_root=project_root)

    def test_rejects_v1_config_after_fixed_action_settings_were_added(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            for relative_path in ("model.onnx", "machine_profile.json", "waypoint_slice.json"):
                (project_root / relative_path).touch()
            config_data = valid_config_payload()
            config_data["schema"] = "runtime_bridge_config_v1"
            del config_data["fixed_action"]
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(config_data), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "schema.*runtime_bridge_config_v2"):
                load_runtime_config(config_path, project_root=project_root)

    def test_rejects_boolean_or_out_of_range_network_numbers(self):
        invalid_values = (True, 0, 65536)
        for invalid_value in invalid_values:
            with self.subTest(state_port=invalid_value), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                config_data = valid_config_payload()
                config_data["network"]["state_port"] = invalid_value
                config_path = project_root / "runtime.json"
                config_path.write_text(json.dumps(config_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, "network.state_port"):
                    load_runtime_config(config_path, project_root=project_root)

    def test_reports_missing_required_field(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            config_data = valid_config_payload()
            del config_data["network"]["action_port"]
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(config_data), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "network.*缺少字段.*action_port"):
                load_runtime_config(config_path, project_root=project_root)

    def test_rejects_missing_input_artifacts_before_runtime_starts(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(valid_config_payload()), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "artifacts.onnx.*不存在"):
                load_runtime_config(config_path, project_root=project_root)

    def test_rejects_unknown_action_time_source(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            for relative_path in ("model.onnx", "machine_profile.json", "waypoint_slice.json"):
                (project_root / relative_path).touch()
            config_data = valid_config_payload()
            config_data["network"]["action_time_source"] = "automatic"
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(config_data), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "network.action_time_source"):
                load_runtime_config(config_path, project_root=project_root)

    def test_rejects_invalid_timeout_and_diagnostic_intervals(self):
        invalid_cases = (
            (("network", "action_valid_ms"), True, "network.action_valid_ms"),
            (("policy", "bucket_tip_timeout_ms"), 0, "policy.bucket_tip_timeout_ms"),
            (("diagnostics", "print_every"), -1, "diagnostics.print_every"),
        )
        for field_path, invalid_value, expected_error in invalid_cases:
            with self.subTest(field=expected_error), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                for relative_path in ("model.onnx", "machine_profile.json", "waypoint_slice.json"):
                    (project_root / relative_path).touch()
                config_data = valid_config_payload()
                config_data[field_path[0]][field_path[1]] = invalid_value
                config_path = project_root / "runtime.json"
                config_path.write_text(json.dumps(config_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, expected_error):
                    load_runtime_config(config_path, project_root=project_root)

    def test_rejects_empty_or_non_string_network_hosts(self):
        invalid_cases = (
            ("state_bind_host", ""),
            ("orin_host", 1234),
        )
        for field, invalid_value in invalid_cases:
            with self.subTest(field=field), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                for relative_path in ("model.onnx", "machine_profile.json", "waypoint_slice.json"):
                    (project_root / relative_path).touch()
                config_data = valid_config_payload()
                config_data["network"][field] = invalid_value
                config_path = project_root / "runtime.json"
                config_path.write_text(json.dumps(config_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, f"network.{field}"):
                    load_runtime_config(config_path, project_root=project_root)

    def test_rejects_invalid_fixed_action_tuning(self):
        invalid_cases = (
            ({"kp": True}, "fixed_action.kp"),
            ({"min_action": 0.8, "max_action": 0.2}, "min_action.*max_action"),
        )
        for changes, expected_error in invalid_cases:
            with self.subTest(changes=changes), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                for relative_path in ("model.onnx", "machine_profile.json", "waypoint_slice.json"):
                    (project_root / relative_path).touch()
                config_data = valid_config_payload()
                config_data["fixed_action"].update(changes)
                config_path = project_root / "runtime.json"
                config_path.write_text(json.dumps(config_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, expected_error):
                    load_runtime_config(config_path, project_root=project_root)


if __name__ == "__main__":
    unittest.main()
