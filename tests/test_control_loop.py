from __future__ import annotations

import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from robot_controller.command import CommandReadResult, CommandReadStatus, JointCommand, PolicyCommand
from robot_controller.control_loop import RobotControlLoop
from robot_controller.core.config import load_robot_controller_config
from robot_controller.core.state import RobotControllerState
from robot_controller.hardware.robot_hardware import HardwareStatus
from robot_controller.processes import ProcessHealth
from robot_controller.safety import OperatorCommand, SafetyController


CONFIG = Path("config/app_config/robot_controller.yaml")


class FakeMotors:
    def __init__(self) -> None:
        self.policy_commands = 0
        self.damping = 0
        self.disable = 0

    def send_policy_mit_batch(self, _command) -> None:
        self.policy_commands += 1

    def send_velocity_damping(self, _reason: str) -> None:
        self.damping += 1

    def disable_all(self, _reason: str) -> None:
        self.disable += 1


class FakeImu:
    def request_on_tick(self, _now: float) -> None:
        return


class FakeHardware:
    def __init__(self, feedback, *, can_connected: bool = True) -> None:
        self.motors = FakeMotors()
        self.imu = FakeImu()
        self.feedback = feedback
        self.can_connected = can_connected

    def read_feedback(self, _now: float):
        return self.feedback

    def status(self) -> HardwareStatus:
        return HardwareStatus(can_connected=self.can_connected)


class FakeSource:
    def __init__(self, result: CommandReadResult) -> None:
        self.result = result

    def read_latest(self) -> CommandReadResult:
        return self.result


class FakeOperatorCommandSource:
    def __init__(self, command: OperatorCommand | None = None) -> None:
        self.command = command

    def read_latest(self) -> OperatorCommand | None:
        command = self.command
        self.command = None
        return command


class FakePublisher:
    def __init__(self) -> None:
        self.publish_count = 0

    def publish(self, **_kwargs) -> None:
        self.publish_count += 1


class FakeProcesses:
    def __init__(self, health: ProcessHealth) -> None:
        self._health = health

    def health(self) -> ProcessHealth:
        return self._health


class FakeValidator:
    def __init__(self, command) -> None:
        self.command = command

    def validate(self, _command):
        return self.command


def feedback(*, motor_stale: bool = False, imu_stale: bool = False, mit_enabled: bool = True):
    return SimpleNamespace(
        motors=SimpleNamespace(
            actuators=[
                SimpleNamespace(
                    state=SimpleNamespace(is_enabled=mit_enabled),
                ),
            ],
            has_stale_feedback=motor_stale,
            stale_reason="actuator feedback stale",
        ),
        imu=SimpleNamespace(
            has_stale_feedback=imu_stale,
            stale_reason="IMU feedback stale",
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


def command(config, *, timestamp: float | None = None) -> PolicyCommand:
    return PolicyCommand(
        source="test",
        timestamp=time.time() if timestamp is None else timestamp,
        targets=[
            JointCommand(can_id=can_id, position_rad=0.0, velocity_rad_s=0.0, kp=0.0, kd=0.5, torque_ff_nm=0.0)
            for can_id in config.can.motors.can_ids
        ],
    )


class ControlLoopTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_robot_controller_config(CONFIG)

    def make_loop(
        self,
        read_result: CommandReadResult,
        process_health: ProcessHealth | None = None,
        *,
        fb=None,
        operator_command: OperatorCommand | None = None,
    ):
        cmd = read_result.command
        hardware = FakeHardware(fb or feedback())
        publisher = FakePublisher()
        loop = RobotControlLoop(
            hardware=hardware,
            policy_command_source=FakeSource(read_result),
            operator_command_source=FakeOperatorCommandSource(operator_command),
            command_validator=FakeValidator(cmd),
            safety=SafetyController(self.config),
            state_publisher=publisher,
            processes=FakeProcesses(process_health or health()),
        )
        loop.safety.start()
        return loop, hardware, publisher

    def test_fresh_valid_command_sends_policy_batch(self) -> None:
        cmd = command(self.config)
        loop, hardware, publisher = self.make_loop(
            CommandReadResult(cmd, CommandReadStatus.AVAILABLE, "ok", cmd.timestamp)
        )
        loop.run_once(RobotControllerState.RUNNING)
        self.assertEqual(hardware.motors.policy_commands, 1)
        self.assertEqual(hardware.motors.damping, 0)
        self.assertEqual(publisher.publish_count, 1)

    def test_stale_command_sends_damping(self) -> None:
        cmd = command(self.config, timestamp=time.time() - 1.0)
        loop, hardware, _publisher = self.make_loop(
            CommandReadResult(cmd, CommandReadStatus.AVAILABLE, "ok", cmd.timestamp)
        )
        loop.run_once(RobotControllerState.RUNNING)
        self.assertEqual(hardware.motors.policy_commands, 0)
        self.assertEqual(hardware.motors.damping, 1)

    def test_missing_command_keeps_sending_damping_while_mit_enabled(self) -> None:
        loop, hardware, _publisher = self.make_loop(
            CommandReadResult(None, CommandReadStatus.NO_COMMAND, "no command", None)
        )
        loop.run_once(RobotControllerState.RUNNING)
        loop.run_once(RobotControllerState.RUNNING)
        self.assertEqual(hardware.motors.policy_commands, 0)
        self.assertEqual(hardware.motors.damping, 2)

    def test_can_daemon_dead_disables_motors(self) -> None:
        cmd = command(self.config)
        loop, hardware, _publisher = self.make_loop(
            CommandReadResult(cmd, CommandReadStatus.AVAILABLE, "ok", cmd.timestamp),
            process_health=health(can_daemon_alive=False),
        )
        loop.run_once(RobotControllerState.RUNNING)
        self.assertEqual(hardware.motors.disable, 1)
        self.assertEqual(hardware.motors.policy_commands, 0)

    def test_fault_latched_does_not_send_policy(self) -> None:
        cmd = command(self.config)
        loop, hardware, _publisher = self.make_loop(
            CommandReadResult(cmd, CommandReadStatus.AVAILABLE, "ok", cmd.timestamp),
            fb=feedback(motor_stale=True),
        )
        loop.run_once(RobotControllerState.RUNNING)
        loop.run_once(RobotControllerState.RUNNING)
        self.assertEqual(hardware.motors.disable, 1)
        self.assertEqual(hardware.motors.policy_commands, 0)

    def test_operator_estop_sends_damping(self) -> None:
        cmd = command(self.config)
        loop, hardware, _publisher = self.make_loop(
            CommandReadResult(cmd, CommandReadStatus.AVAILABLE, "ok", cmd.timestamp),
            operator_command=OperatorCommand(estop=True),
        )
        loop.run_once(RobotControllerState.RUNNING)
        self.assertEqual(hardware.motors.damping, 1)
        self.assertEqual(hardware.motors.disable, 0)
        self.assertEqual(hardware.motors.policy_commands, 0)


if __name__ == "__main__":
    unittest.main()
