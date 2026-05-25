from __future__ import annotations

import copy
import math
import time
import unittest
from pathlib import Path

from robot_controller.command import CommandValidator, JointCommand, PolicyCommand
from robot_controller.core.config import load_robot_controller_config


CONFIG = Path("config/app_config/robot_controller.yaml")


def command_for(can_ids: list[int], *, position: float = 0.0) -> PolicyCommand:
    return PolicyCommand(
        source="test",
        timestamp=time.time(),
        targets=[
            JointCommand(
                can_id=can_id,
                position_rad=position,
                velocity_rad_s=0.0,
                kp=0.0,
                kd=0.5,
                torque_ff_nm=0.0,
            )
            for can_id in can_ids
        ],
    )


class CommandValidatorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_robot_controller_config(CONFIG)
        self.validator = CommandValidator(
            expected_can_ids=self.config.can.motors.can_ids,
            protocol_range=self.config.can.mit_protocol_range,
        )

    def test_valid_command_passes(self) -> None:
        command = command_for(self.config.can.motors.can_ids)
        self.assertIs(self.validator.validate(command), command)

    def test_nan_rejects(self) -> None:
        command = command_for(self.config.can.motors.can_ids, position=math.nan)
        with self.assertRaisesRegex(ValueError, "non-finite"):
            self.validator.validate(command)

    def test_unknown_can_id_rejects(self) -> None:
        ids = copy.copy(self.config.can.motors.can_ids)
        ids[-1] = 0x7AA
        with self.assertRaisesRegex(ValueError, "unknown CAN ID"):
            self.validator.validate(command_for(ids))

    def test_missing_actuator_rejects(self) -> None:
        ids = self.config.can.motors.can_ids[:-1]
        with self.assertRaisesRegex(ValueError, "missing actuator"):
            self.validator.validate(command_for(ids))

    def test_duplicate_can_id_rejects(self) -> None:
        ids = copy.copy(self.config.can.motors.can_ids)
        ids[1] = ids[0]
        with self.assertRaisesRegex(ValueError, "duplicate CAN ID"):
            self.validator.validate(command_for(ids))

    def test_limit_rejects(self) -> None:
        command = command_for(
            self.config.can.motors.can_ids,
            position=self.config.can.mit_protocol_range.position_rad + 0.1,
        )
        with self.assertRaisesRegex(ValueError, "out of range"):
            self.validator.validate(command)


if __name__ == "__main__":
    unittest.main()
