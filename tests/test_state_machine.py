from __future__ import annotations

import unittest

from robot_controller.shm.operator_command import OperatorCommandC, OperatorCommandCode
from robot_controller.state_machine import ControllerMode, ControllerStateMachine


def command(code: OperatorCommandCode) -> OperatorCommandC:
    item = OperatorCommandC()
    item.timestamp_ns = 1
    item.command = int(code)
    item.target_mask = 0
    return item


class ControllerStateMachineTest(unittest.TestCase):
    def test_enable_duration_transitions_to_normal(self) -> None:
        sm = ControllerStateMachine(enable_duration_s=0.5, mode_enter_time=10.0)
        sm.update(command(OperatorCommandCode.ENABLE), 10.0)
        self.assertEqual(sm.mode, ControllerMode.ENABLING)
        sm.update(None, 10.49)
        self.assertEqual(sm.mode, ControllerMode.ENABLING)
        sm.update(None, 10.5)
        self.assertEqual(sm.mode, ControllerMode.NORMAL)

    def test_repeated_enable_does_not_block_normal_transition(self) -> None:
        sm = ControllerStateMachine(enable_duration_s=0.5, mode_enter_time=10.0)
        enable = command(OperatorCommandCode.ENABLE)
        sm.update(enable, 10.0)
        self.assertEqual(sm.mode, ControllerMode.ENABLING)
        sm.update(enable, 10.5)
        self.assertEqual(sm.mode, ControllerMode.NORMAL)

    def test_estop_latches_until_reset_fault(self) -> None:
        sm = ControllerStateMachine(enable_duration_s=0.0, mode_enter_time=1.0)
        sm.update(command(OperatorCommandCode.ESTOP), 1.0)
        self.assertEqual(sm.mode, ControllerMode.ESTOP)
        sm.update(command(OperatorCommandCode.ENABLE), 2.0)
        self.assertEqual(sm.mode, ControllerMode.ESTOP)
        sm.update(command(OperatorCommandCode.RESET_FAULT), 3.0)
        self.assertEqual(sm.mode, ControllerMode.DISABLED)


if __name__ == "__main__":
    unittest.main()
