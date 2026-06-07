from __future__ import annotations

import unittest
import uuid

from robot_controller.shm.operator_command import (
    OPERATOR_ZERO_TARGET_MAGIC,
    OperatorCommandC,
    OperatorCommandCode,
    OperatorCommandShm,
)
from robot_controller.controller import RobotController
from robot_controller.state_machine import ControllerMode, ControllerStateMachine


class OperatorCommandShmTest(unittest.TestCase):
    def test_zero_set_targets_round_trip(self) -> None:
        name = f"qhrr_test_operator_command_{uuid.uuid4().hex}"
        shm = OperatorCommandShm.create(name)
        try:
            writer = OperatorCommandShm.open_writer(name)
            reader = OperatorCommandShm.open_reader(name)
            try:
                writer.publish_zero_set(
                    [
                        (0x141, 8250),
                        (0x142, -12000),
                        (0x143, 2000),
                    ]
                )

                command = reader.read_relaxed()
                self.assertEqual(command.command, OperatorCommandCode.ZERO_SET)
                self.assertEqual(command.zero_target_count, 3)
                self.assertEqual(command.zero_target_magic, OPERATOR_ZERO_TARGET_MAGIC)
                self.assertEqual(command.zero_targets[0].can_id, 0x141)
                self.assertEqual(command.zero_targets[0].offset_count, 8250)
                self.assertEqual(command.zero_targets[1].can_id, 0x142)
                self.assertEqual(command.zero_targets[1].offset_count, -12000)
                self.assertEqual(command.zero_targets[2].can_id, 0x143)
                self.assertEqual(command.zero_targets[2].offset_count, 2000)
                self.assertEqual(
                    RobotController._zero_set_offsets_by_can_id(command),
                    {0x141: 8250, 0x142: -12000, 0x143: 2000},
                )

                writer.publish(OperatorCommandCode.ENABLE)
                command = reader.read_relaxed()
                self.assertEqual(command.command, OperatorCommandCode.ENABLE)
                self.assertEqual(command.zero_target_count, 0)
            finally:
                writer.close()
                reader.close()
        finally:
            shm.unlink()
            shm.close()

    def test_controller_rejects_zero_targets_without_magic(self) -> None:
        command = OperatorCommandShm.create(f"qhrr_test_operator_command_{uuid.uuid4().hex}")
        try:
            item = command.read_relaxed()
            item.zero_target_count = 1
            item.zero_targets[0].can_id = 0x141
            item.zero_targets[0].offset_count = 8250

            with self.assertRaisesRegex(RuntimeError, "invalid zero target magic"):
                RobotController._zero_set_offsets_by_can_id(item)
        finally:
            command.unlink()
            command.close()

    def test_controller_consumes_repeated_operator_command_as_none(self) -> None:
        controller = RobotController.__new__(RobotController)
        controller._last_consumed_operator_timestamp_ns = None
        controller._last_ignored_operator_timestamp_ns = None
        command = OperatorCommandC()
        command.timestamp_ns = 123
        command.command = int(OperatorCommandCode.ZERO_SET)

        first = controller._consume_operator_command(command)
        with self.assertLogs("robot_controller.controller", level="WARNING"):
            second = controller._consume_operator_command(command)

        self.assertEqual(first.command, OperatorCommandCode.ZERO_SET)
        self.assertEqual(second.command, OperatorCommandCode.NONE)

    def test_consumed_zero_set_releases_state_machine(self) -> None:
        controller = RobotController.__new__(RobotController)
        controller._last_consumed_operator_timestamp_ns = None
        controller._last_ignored_operator_timestamp_ns = None
        command = OperatorCommandC()
        command.timestamp_ns = 123
        command.command = int(OperatorCommandCode.ZERO_SET)
        state_machine = ControllerStateMachine(
            enable_duration_s=0.0,
            mode=ControllerMode.NORMAL,
            mode_enter_time=1.0,
        )

        state_machine.update(controller._consume_operator_command(command), 2.0)
        with self.assertLogs("robot_controller.controller", level="WARNING"):
            state_machine.update(controller._consume_operator_command(command), 3.0)

        self.assertEqual(state_machine.mode, ControllerMode.DISABLED)


if __name__ == "__main__":
    unittest.main()
