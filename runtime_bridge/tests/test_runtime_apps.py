import unittest
from pathlib import Path

from runtime_bridge.apps.pc_runtime_bridge import build_arg_parser
from runtime_bridge.runtime_config import load_runtime_config


PROJECT_ROOT = Path(__file__).resolve().parents[2]


class RuntimeAppsTest(unittest.TestCase):
    def test_diagnostic_bridge_exposes_only_runtime_intent(self):
        parser = build_arg_parser()
        defaults = parser.parse_args([])

        self.assertEqual(
            set(vars(defaults)),
            {"config", "reply_zero", "publish_joint_states"},
        )
        self.assertFalse(defaults.reply_zero)
        self.assertTrue(parser.parse_args(["--reply-zero"]).reply_zero)

    def test_shipped_mock_config_uses_loopback_network(self):
        config = load_runtime_config(
            PROJECT_ROOT / "runtime_bridge" / "config" / "runtime.mock.json"
        )

        self.assertEqual(config.network.state_endpoint, ("127.0.0.1", 18081))
        self.assertEqual(config.network.action_endpoint, ("127.0.0.1", 18082))


if __name__ == "__main__":
    unittest.main()
