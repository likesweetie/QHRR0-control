from __future__ import annotations

import copy
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from robot_controller.command import CommandReadResult, CommandReadStatus, JointCommand, PolicyCommand
from robot_controller.core.config import load_robot_controller_config
from robot_controller.hardware.robot_hardware import HardwareStatus
from robot_controller.processes import ProcessHealth
from robot_controller.safety import ControlAction, OperatorCommand, SafetyController, SafetyInputs, SafetyState


CONFIG = Path("config/app_config/robot_controller.yaml")


def feedback(*, motor_stale: bool = False, imu_stale: bool = False, mit_enabled: bool = True):
    return SimpleNamespace(
        motors=SimpleNamespace(
            actuators=[
                SimpleNamespace(
                    state=SimpleNamespace(is_enabled=mit_enabled),
                ),
            ],
            has_stale_feedback=motor_stale,
            stale_reason="actuator feedback stale: 0x141",
        ),
        imu=SimpleNamespace(
            has_stale_feedback=imu_stale,
            stale_reason="IMU feedback stale: quat",
        ),
    )


def health(**overrides) -> ProcessHealth:
    values = {
        "can_daemon_alive": True,
        "task_controller_alive": True,
        "aux_reader_alive": True,
        "dashboard_alive": True,
    }
    values.update(overrides)
    return ProcessHealth(**values)


def policy_command(config, *, timestamp: float | None = None) -> PolicyCommand:
    return PolicyCommand(
        source="test",
        timestamp=time.time() if timestamp is None else timestamp,
        targets=[
            JointCommand(can_id=can_id, position_rad=0.0, velocity_rad_s=0.0, kp=0.0, kd=0.5, torque_ff_nm=0.0)
            for can_id in config.can.motors.can_ids
        ],
    )


def inputs(config, *, command=None, status=CommandReadStatus.AVAILABLE, reason="ok", validation_error=None, **kwargs):
    cmd = command if command is not None else policy_command(config)
    now = kwargs.get("now", time.monotonic())
    return SafetyInputs(
        feedback=kwargs.get("feedback_value", feedback()),
        command=CommandReadResult(cmd, status, reason, None if cmd is None else cmd.timestamp),
        validated_command=None if validation_error else cmd,
        command_validation_error=validation_error,
        process_health=kwargs.get("process_health", health()),
        hardware_status=kwargs.get("hardware_status", HardwareStatus(can_connected=True)),
        operator_command=kwargs.get("operator_command"),
        now=now,
        now_unix=kwargs.get("now_unix", time.time()),
    )


class SafetyControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_robot_controller_config(CONFIG)

    def test_missing_command_enters_damping(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        decision = safety.evaluate(
            inputs(
                self.config,
                command=None,
                status=CommandReadStatus.NO_COMMAND,
                reason="no command",
            )
        )
        self.assertEqual(decision.state, SafetyState.DAMPING)
        self.assertEqual(decision.action, ControlAction.SEND_DAMPING)

    def test_missing_command_without_mit_enabled_does_not_send_damping(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        decision = safety.evaluate(
            inputs(
                self.config,
                command=None,
                status=CommandReadStatus.NO_COMMAND,
                reason="no command",
                feedback_value=feedback(mit_enabled=False),
            )
        )
        self.assertEqual(decision.state, SafetyState.DAMPING)
        self.assertEqual(decision.action, ControlAction.NO_OUTPUT)

    def test_missing_command_keeps_damping_while_mit_enabled(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        now = time.monotonic()
        first = safety.evaluate(
            inputs(
                self.config,
                command=None,
                status=CommandReadStatus.NO_COMMAND,
                reason="no command",
                now=now,
            )
        )
        second = safety.evaluate(
            inputs(
                self.config,
                command=None,
                status=CommandReadStatus.NO_COMMAND,
                reason="no command",
                now=now + 0.01,
            )
        )
        self.assertEqual(first.action, ControlAction.SEND_DAMPING)
        self.assertEqual(second.action, ControlAction.SEND_DAMPING)

    def test_command_loss_damping_timeout_faults_but_keeps_damping(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        now = time.monotonic()
        safety.evaluate(
            inputs(
                self.config,
                command=None,
                status=CommandReadStatus.NO_COMMAND,
                reason="no command",
                now=now,
            )
        )
        decision = safety.evaluate(
            inputs(
                self.config,
                command=None,
                status=CommandReadStatus.NO_COMMAND,
                reason="no command",
                now=now + self.config.safety.damping_timeout_s,
            )
        )
        self.assertEqual(decision.state, SafetyState.FAULT_LATCHED)
        self.assertEqual(decision.action, ControlAction.SEND_DAMPING)

    def test_damping_timeout_latched_fault_continues_damping(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        now = time.monotonic()
        safety.evaluate(
            inputs(
                self.config,
                command=None,
                status=CommandReadStatus.NO_COMMAND,
                reason="no command",
                now=now,
            )
        )
        safety.evaluate(
            inputs(
                self.config,
                command=None,
                status=CommandReadStatus.NO_COMMAND,
                reason="no command",
                now=now + self.config.safety.damping_timeout_s,
            )
        )
        decision = safety.evaluate(
            inputs(
                self.config,
                command=None,
                status=CommandReadStatus.NO_COMMAND,
                reason="no command",
                now=now + self.config.safety.damping_timeout_s + 0.01,
            )
        )
        self.assertEqual(decision.state, SafetyState.FAULT_LATCHED)
        self.assertEqual(decision.action, ControlAction.SEND_DAMPING)

    def test_stale_command_enters_damping(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        cmd = policy_command(self.config, timestamp=time.time() - 1.0)
        decision = safety.evaluate(inputs(self.config, command=cmd))
        self.assertEqual(decision.state, SafetyState.DAMPING)
        self.assertEqual(decision.action, ControlAction.SEND_DAMPING)

    def test_invalid_command_faults(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        decision = safety.evaluate(inputs(self.config, validation_error="NaN"))
        self.assertEqual(decision.state, SafetyState.FAULT_LATCHED)
        self.assertEqual(decision.action, ControlAction.DISABLE_MOTORS)

    def test_feedback_stale_faults_by_config(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        decision = safety.evaluate(inputs(self.config, feedback_value=feedback(motor_stale=True)))
        self.assertEqual(decision.state, SafetyState.FAULT_LATCHED)
        self.assertEqual(decision.action, ControlAction.DISABLE_MOTORS)

    def test_can_daemon_dead_faults(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        decision = safety.evaluate(inputs(self.config, process_health=health(can_daemon_alive=False)))
        self.assertEqual(decision.state, SafetyState.FAULT_LATCHED)
        self.assertEqual(decision.action, ControlAction.DISABLE_MOTORS)

    def test_task_controller_dead_damps(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        decision = safety.evaluate(inputs(self.config, process_health=health(task_controller_alive=False)))
        self.assertEqual(decision.state, SafetyState.DAMPING)
        self.assertEqual(decision.action, ControlAction.SEND_DAMPING)

    def test_missing_command_damps_even_before_feedback_is_ready(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        decision = safety.evaluate(
            inputs(
                self.config,
                command=None,
                status=CommandReadStatus.NO_COMMAND,
                reason="no command",
                feedback_value=feedback(motor_stale=True, imu_stale=True),
            )
        )
        self.assertEqual(decision.state, SafetyState.DAMPING)
        self.assertEqual(decision.action, ControlAction.SEND_DAMPING)

    def test_damping_recovers_with_fresh_command_and_feedback(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        safety.evaluate(
            inputs(
                self.config,
                command=None,
                status=CommandReadStatus.NO_COMMAND,
                reason="no command",
            )
        )
        decision = safety.evaluate(inputs(self.config))
        self.assertEqual(decision.state, SafetyState.RUNNING)
        self.assertEqual(decision.action, ControlAction.SEND_POLICY_COMMAND)

    def test_fault_latched_requires_clear(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        safety.evaluate(inputs(self.config, validation_error="NaN"))
        decision = safety.evaluate(inputs(self.config))
        self.assertEqual(decision.state, SafetyState.FAULT_LATCHED)
        self.assertEqual(decision.action, ControlAction.NO_OUTPUT)

    def test_fault_clear_returns_to_disarmed(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        safety.evaluate(inputs(self.config, validation_error="NaN"))
        decision = safety.evaluate(
            inputs(self.config, operator_command=OperatorCommand(clear_fault=True))
        )
        self.assertEqual(decision.state, SafetyState.DISARMED)
        self.assertEqual(decision.action, ControlAction.NO_OUTPUT)

    def test_operator_estop_enters_damping(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        decision = safety.evaluate(
            inputs(self.config, operator_command=OperatorCommand(estop=True))
        )
        self.assertEqual(decision.state, SafetyState.DAMPING)
        self.assertEqual(decision.action, ControlAction.SEND_DAMPING)
        self.assertEqual(decision.reason, "operator E-stop")

    def test_operator_estop_damping_does_not_auto_recover_with_fresh_command(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        safety.evaluate(inputs(self.config, operator_command=OperatorCommand(estop=True)))
        decision = safety.evaluate(inputs(self.config))
        self.assertEqual(decision.state, SafetyState.DAMPING)
        self.assertEqual(decision.action, ControlAction.SEND_DAMPING)
        self.assertEqual(decision.reason, "operator E-stop")

    def test_operator_arm_clears_estop_damping(self) -> None:
        safety = SafetyController(self.config)
        safety.start()
        safety.evaluate(inputs(self.config, operator_command=OperatorCommand(estop=True)))
        decision = safety.evaluate(inputs(self.config, operator_command=OperatorCommand(arm=True)))
        self.assertEqual(decision.state, SafetyState.RUNNING)
        self.assertEqual(decision.action, ControlAction.SEND_POLICY_COMMAND)

    def test_hardware_starts_disarmed(self) -> None:
        config = copy.deepcopy(self.config)
        config.runtime.mode = "hardware"
        safety = SafetyController(config)
        decision = safety.start()
        self.assertEqual(decision.state, SafetyState.DISARMED)
        self.assertEqual(decision.action, ControlAction.NO_OUTPUT)


if __name__ == "__main__":
    unittest.main()
