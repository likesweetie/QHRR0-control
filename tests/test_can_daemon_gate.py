from __future__ import annotations

import unittest

from robot_controller.process.can_daemon.main import (
    SPG_CMD_MIT_ENTER,
    motor_enable_block_reason,
)
from robot_controller.utils.hal_can_bus import CANFrame


class CanDaemonGateTest(unittest.TestCase):
    def test_disarmed_blocks_motor_enable(self) -> None:
        reason = motor_enable_block_reason(
            CANFrame(can_id=0x141, data=bytes([SPG_CMD_MIT_ENTER]) + b"\x00" * 7),
            actuator_can_ids={0x141},
            read_control_state=lambda: {"safety_state": "DISARMED"},
        )

        self.assertIsNotNone(reason)
        self.assertIn("DISARMED", reason or "")

    def test_damping_allows_motor_enable_after_arm(self) -> None:
        reason = motor_enable_block_reason(
            CANFrame(can_id=0x141, data=bytes([SPG_CMD_MIT_ENTER]) + b"\x00" * 7),
            actuator_can_ids={0x141},
            read_control_state=lambda: {"safety_state": "DAMPING", "safety_reason": "no command"},
        )

        self.assertIsNone(reason)

    def test_operator_estop_damping_blocks_motor_enable(self) -> None:
        reason = motor_enable_block_reason(
            CANFrame(can_id=0x141, data=bytes([SPG_CMD_MIT_ENTER]) + b"\x00" * 7),
            actuator_can_ids={0x141},
            read_control_state=lambda: {"safety_state": "DAMPING", "safety_reason": "operator E-stop"},
        )

        self.assertIsNotNone(reason)
        self.assertIn("E-stop", reason or "")

    def test_disable_frame_is_not_blocked(self) -> None:
        reason = motor_enable_block_reason(
            CANFrame(can_id=0x141, data=b"\xC2" + b"\x00" * 7),
            actuator_can_ids={0x141},
            read_control_state=lambda: {"safety_state": "DISARMED"},
        )

        self.assertIsNone(reason)

    def test_missing_controller_state_blocks_motor_enable(self) -> None:
        reason = motor_enable_block_reason(
            CANFrame(can_id=0x141, data=bytes([SPG_CMD_MIT_ENTER]) + b"\x00" * 7),
            actuator_can_ids={0x141},
            read_control_state=lambda: None,
        )

        self.assertIsNotNone(reason)
        self.assertIn("unavailable", reason or "")


if __name__ == "__main__":
    unittest.main()
