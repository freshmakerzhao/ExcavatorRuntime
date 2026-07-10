import unittest

from runtime_bridge.apps.fixed_action_player import build_arg_parser
from runtime_bridge.fixed_actions import FixedActionExecutor, fixed_action_sequence, physical_velocity_action_from_normalized
from runtime_bridge.protocol import MachineStatePacket, decode_packet, encode_packet


def sample_profile():
    return {
        "actuators": {
            "boom": {
                "range": [-0.1, 0.1],
                "sign": 1,
                "deploy_sign": 1,
                "max_speed_positive": 0.04,
                "max_speed_negative": 0.02,
            },
            "stick": {
                "range": [-0.2, 0.2],
                "sign": 1,
                "deploy_sign": 1,
                "max_speed_positive": 0.05,
                "max_speed_negative": 0.05,
            },
            "bucket": {
                "range": [-0.3, 0.1],
                "sign": 1,
                "deploy_sign": 1,
                "max_speed_positive": 0.03,
                "max_speed_negative": 0.06,
            },
            "swing": {
                "range": [None, None],
                "sign": 1,
                "deploy_sign": 1,
                "max_speed_positive": 0.6,
                "max_speed_negative": 0.6,
            },
        }
    }


def sample_state(boom=0.0, stick=0.0, bucket=-0.1, control_enabled=True):
    return MachineStatePacket(
        seq=1,
        stamp_ms=1000,
        safety={
            "estop": False,
            "stm32_alive": True,
            "sensor_valid": True,
            "control_enabled": control_enabled,
            "fault_flags": [],
        },
        actuator_state={
            "boom": {"position_m": boom, "velocity_mps": 0.0},
            "stick": {"position_m": stick, "velocity_mps": 0.0},
            "bucket": {"position_m": bucket, "velocity_mps": 0.0},
            "swing": {"position_rad": 0.0, "velocity_rad_s": 0.0},
        },
        joint_state={"position_rad": {"swing": 0.0, "boom": 0.0, "arm": 0.0, "bucket": 0.0}},
    )


class FixedActionTest(unittest.TestCase):
    def test_fixed_action_player_requires_explicit_motion_enable(self):
        parser = build_arg_parser()
        defaults = parser.parse_args(["dig"])

        self.assertEqual(set(vars(defaults)), {"action", "config", "enable_motion"})
        self.assertFalse(defaults.enable_motion)
        self.assertTrue(parser.parse_args(["dig", "--enable-motion"]).enable_motion)

    def test_denormalizes_action_to_physical_velocity(self):
        action = physical_velocity_action_from_normalized([1.0, -1.0, 0.5, -0.5], sample_profile())

        self.assertEqual(action, [0.04, -0.05, 0.015, -0.3])

    def test_dig_first_step_generates_orin_compatible_physical_velocity_packet(self):
        executor = FixedActionExecutor(fixed_action_sequence("dig"), sample_profile())

        packet, status = executor.step(sample_state(), now_s=0.0, seq=7, valid_for_ms=100)
        decoded = decode_packet(encode_packet(packet))

        self.assertFalse(status.done)
        self.assertEqual(decoded.action_type, "normalized_velocity_command")
        self.assertEqual(decoded.action_order, ("boom", "stick", "bucket", "swing"))
        self.assertAlmostEqual(decoded.action[0], 0.03)
        self.assertEqual(decoded.action[1:], [0.0, 0.0, 0.0])

    def test_dump_sequence_opens_bucket_then_reports_done_after_target_reached(self):
        executor = FixedActionExecutor(fixed_action_sequence("dump"), sample_profile(), tolerance=0.03)

        first_packet, first_status = executor.step(sample_state(bucket=-0.1), now_s=0.0, seq=1, valid_for_ms=100)
        self.assertFalse(first_status.done)
        self.assertGreater(first_packet.action[2], 0.0)

        done_packet, done_status = executor.step(sample_state(bucket=0.1), now_s=0.2, seq=2, valid_for_ms=100)
        self.assertFalse(done_status.done)
        self.assertEqual(done_packet.action, [0.0, 0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
