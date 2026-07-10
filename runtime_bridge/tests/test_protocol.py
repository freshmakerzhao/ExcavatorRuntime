import unittest

from runtime_bridge.protocol import (
    ExcavatorStatePacket,
    MachineStatePacket,
    PacketDecodeError,
    PolicyActionPacket,
    decode_packet,
    encode_packet,
    estimate_remote_now_ms,
)


class RuntimeBridgeProtocolTest(unittest.TestCase):
    def test_state_packet_round_trips_through_json_bytes(self):
        packet = MachineStatePacket(
            seq=7,
            stamp_ms=123456,
            safety={
                "estop": False,
                "stm32_alive": True,
                "sensor_valid": True,
                "control_enabled": False,
                "fault_flags": [],
            },
            actuator_state={
                "boom": {"position_m": 0.012, "velocity_mps": 0.001},
                "stick": {"position_m": -0.018, "velocity_mps": 0.0},
                "bucket": {"position_m": 0.006, "velocity_mps": -0.002},
                "swing": {"position_rad": 0.25, "velocity_rad_s": 0.01},
            },
            joint_state={
                "position_rad": {"swing": 0.1, "boom": 0.2, "arm": -0.3, "bucket": 0.4},
                "velocity_rad_s": {"swing": 0.0, "boom": 0.0, "arm": 0.0, "bucket": 0.0},
            },
            raw_sensor={"stm32_stamp_ms": 12345678},
        )

        decoded = decode_packet(encode_packet(packet))

        self.assertEqual(decoded, packet)

    def test_action_packet_clips_to_four_values_and_validity(self):
        packet = PolicyActionPacket(
            seq=11,
            stamp_ms=123500,
            action=[0.0, 0.25, -0.5, 1.0],
            action_type="normalized_velocity_command",
            valid_for_ms=100,
        )

        decoded = decode_packet(encode_packet(packet))

        self.assertEqual(decoded, packet)
        self.assertEqual(decoded.action_order, ("boom", "stick", "bucket", "swing"))

    def test_action_packet_json_keeps_orin_compatible_field_order(self):
        packet = PolicyActionPacket(
            seq=456,
            stamp_ms=1780000000100,
            action=[0.0, 0.1, -0.1, 0.0],
            action_type="normalized_velocity_command",
            valid_for_ms=100,
        )

        payload = encode_packet(packet).decode("utf-8")

        self.assertTrue(
            payload.startswith(
                '{"type":"policy_action","schema_version":"1.0","seq":456,'
                '"stamp_ms":1780000000100,"action_order":["boom","stick","bucket","swing"],'
                '"action":[0.0,0.1,-0.1,0.0],"action_type":"normalized_velocity_command","valid_for_ms":100'
            ),
            payload,
        )

    def test_estimates_remote_action_stamp_from_state_packet_clock(self):
        action_stamp_ms = estimate_remote_now_ms(
            remote_stamp_ms=1783595742624,
            local_receive_ms=1783595742519,
            local_now_ms=1783595742534,
        )

        self.assertEqual(action_stamp_ms, 1783595742639)

    def test_decode_rejects_missing_joint(self):
        raw = (
            b'{"type":"machine_state_v1","seq":1,"stamp_ms":2,'
            b'"safety":{"estop":false,"stm32_alive":true,"sensor_valid":true,"control_enabled":false,"fault_flags":[]},'
            b'"actuator_state":{"boom":{"position_m":0,"velocity_mps":0},"stick":{"position_m":0,"velocity_mps":0},'
            b'"bucket":{"position_m":0,"velocity_mps":0},"swing":{"position_rad":0,"velocity_rad_s":0}},'
            b'"joint_state":{"position_rad":{"swing":0}}}'
        )

        with self.assertRaises(PacketDecodeError):
            decode_packet(raw)

    def test_decode_machine_state_defaults_missing_joint_velocity_to_zero(self):
        raw = (
            b'{"type":"machine_state_v1","schema_version":"1.0","seq":1,"stamp_ms":2,'
            b'"safety":{"estop":false,"stm32_alive":true,"sensor_valid":true,"control_enabled":false,"fault_flags":[]},'
            b'"actuator_state":{"boom":{"position_m":0,"velocity_mps":0},"stick":{"position_m":0,"velocity_mps":0},'
            b'"bucket":{"position_m":0,"velocity_mps":0},"swing":{"position_rad":0,"velocity_rad_s":0}},'
            b'"joint_state":{"position_rad":{"swing":0.1,"boom":0.2,"arm":0.3,"bucket":0.4}}}'
        )

        packet = decode_packet(raw)

        self.assertEqual(packet.joint_position_rad, {"swing": 0.1, "boom": 0.2, "arm": 0.3, "bucket": 0.4})
        self.assertEqual(packet.joint_velocity_rad_s, {"swing": 0.0, "boom": 0.0, "arm": 0.0, "bucket": 0.0})

    def test_decode_rejects_bad_safety_flags(self):
        raw = (
            b'{"type":"machine_state_v1","seq":1,"stamp_ms":2,'
            b'"safety":{"estop":false,"stm32_alive":true,"sensor_valid":"yes","control_enabled":false,"fault_flags":[]},'
            b'"actuator_state":{"boom":{"position_m":0,"velocity_mps":0},"stick":{"position_m":0,"velocity_mps":0},'
            b'"bucket":{"position_m":0,"velocity_mps":0},"swing":{"position_rad":0,"velocity_rad_s":0}},'
            b'"joint_state":{"position_rad":{"swing":0,"boom":0,"arm":0,"bucket":0},'
            b'"velocity_rad_s":{"swing":0,"boom":0,"arm":0,"bucket":0}}}'
        )

        with self.assertRaises(PacketDecodeError):
            decode_packet(raw)

    def test_legacy_excavator_state_still_decodes_during_transition(self):
        packet = ExcavatorStatePacket(
            seq=7,
            stamp_ms=123456,
            joint_position_rad={"swing": 0.1, "boom": 0.2, "arm": -0.3, "bucket": 0.4},
            joint_velocity_rad_s={"swing": 0.0, "boom": 0.0, "arm": 0.0, "bucket": 0.0},
            estop=False,
            mode="autonomy_ready",
        )

        decoded = decode_packet(encode_packet(packet))

        self.assertEqual(decoded, packet)


if __name__ == "__main__":
    unittest.main()
