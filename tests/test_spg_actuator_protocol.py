from __future__ import annotations

import struct
import unittest

from hal.can_bus import CANFrame

from robot_controller.QHRR0_HW.SPG_actuator.dongilC_motor_protocol import (
    SPGActuatorProtocol,
    SPGMITConfig,
    float_to_uint,
)


class SPGActuatorProtocolTest(unittest.TestCase):
    def test_mit_status_position_uses_signed_feedback_range(self) -> None:
        protocol = SPGActuatorProtocol(
            command_id=0x141,
            feedback_id=0x141,
            mit_config=SPGMITConfig(feedback_position_max=12.56),
            expose_single_turn_position=True,
        )

        payload = bytearray(8)
        payload[0] = SPGActuatorProtocol.CMD_MIT_CONTROL
        struct.pack_into("<h", payload, 6, int(round(-1.0 / 12.56 * 32767.0)))

        state = protocol.decode_frame(CANFrame(can_id=0x141, data=bytes(payload)))

        self.assertIsNotNone(state)
        self.assertAlmostEqual(state.position_rad, -1.0, places=3)

    def test_set_zero_ack_uses_signed_output_offset_degrees(self) -> None:
        protocol = SPGActuatorProtocol(
            command_id=0x141,
            feedback_id=0x141,
            mit_config=SPGMITConfig(),
        )

        payload = bytearray(8)
        payload[0] = SPGActuatorProtocol.CMD_MIT_SET_ZERO
        struct.pack_into("<h", payload, 6, -4500)

        state = protocol.decode_frame(CANFrame(can_id=0x141, data=bytes(payload)))

        self.assertIsNotNone(state)
        self.assertEqual(state.mode, "MIT_SET_ZERO_ACK")
        self.assertEqual(state.raw["offset_i16"], -4500)
        self.assertAlmostEqual(state.raw["offset_deg"], -45.0)

    def test_set_zero_command_encodes_int16_centidegrees(self) -> None:
        protocol = SPGActuatorProtocol(
            command_id=0x141,
            feedback_id=0x141,
            mit_config=SPGMITConfig(),
        )

        frame = protocol.encode_mit_set_zero_frame(offset_deg=30.0)

        self.assertEqual(frame.data, bytes([0xC3, 0, 0, 0, 0, 0, 0xB8, 0x0B]))

    def test_clear_error_flag_ack_decodes_remaining_fault(self) -> None:
        protocol = SPGActuatorProtocol(
            command_id=0x141,
            feedback_id=0x141,
            mit_config=SPGMITConfig(),
        )

        payload = bytes([SPGActuatorProtocol.CMD_CLEAR_ERROR_FLAG, 7, 0, 0, 0, 0, 0, 0])
        state = protocol.decode_frame(CANFrame(can_id=0x141, data=payload))

        self.assertIsNotNone(state)
        self.assertEqual(state.mode, "CLEAR_ERROR_ACK")
        self.assertFalse(state.is_enabled)
        self.assertEqual(state.fault_code, 7)
        self.assertEqual(state.raw["fault_code_after_clear"], 7)

    def test_mit_params_decode_and_write_encoding(self) -> None:
        protocol = SPGActuatorProtocol(
            command_id=0x141,
            feedback_id=0x141,
            mit_config=SPGMITConfig(),
        )

        read_payload = bytearray([0xC4, 45, 33, 0, 0, 0, 0, 0])
        struct.pack_into("<H", read_payload, 3, 1234)
        struct.pack_into("<H", read_payload, 5, 900)
        state = protocol.decode_frame(CANFrame(can_id=0x141, data=bytes(read_payload)))

        self.assertIsNotNone(state)
        self.assertEqual(state.mode, "MIT_PARAMS")
        self.assertEqual(state.raw["v_max_rad_s"], 45)
        self.assertEqual(state.raw["tau_max_nm"], 33)
        self.assertAlmostEqual(state.raw["kt_out_nm_per_a"], 1.234)
        self.assertAlmostEqual(state.raw["gear_ratio"], 9.0)

        write_frame = protocol.encode_write_mit_params_frame(
            v_max_rad_s=45,
            tau_max_nm=33,
            kt_input_nm_per_a=0.123,
            gear_ratio=9.0,
        )

        self.assertEqual(write_frame.data, bytes([0xC5, 45, 33, 123, 0, 132, 3, 0]))

    def test_float_to_uint_uses_firmware_style_half_away_rounding(self) -> None:
        self.assertEqual(float_to_uint(0.5, 0.0, 1.0, 1), 1)


if __name__ == "__main__":
    unittest.main()
