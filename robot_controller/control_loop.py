from __future__ import annotations

import time
from dataclasses import dataclass

from robot_controller.command.command_validator import CommandValidator
from robot_controller.command.policy_command import CommandReadResult, PolicyCommand
from robot_controller.command.shm_policy_command_source import ShmPolicyCommandSource
from robot_controller.core.state import RobotControllerState
from robot_controller.hardware.robot_hardware import RobotFeedback, RobotHardware
from robot_controller.processes.child_process_manager import ChildProcessManager
from robot_controller.safety.safety_controller import OperatorCommand, SafetyController, SafetyInputs
from robot_controller.safety.safety_state import ControlAction, SafetyDecision
from robot_controller.state.state_publisher import StatePublisher


@dataclass(frozen=True)
class ControlTickResult:
    feedback: RobotFeedback
    command: CommandReadResult
    validated_command: PolicyCommand | None
    decision: SafetyDecision


class RobotControlLoop:
    def __init__(
        self,
        *,
        hardware: RobotHardware,
        policy_command_source: ShmPolicyCommandSource,
        operator_command_source,
        command_validator: CommandValidator,
        safety: SafetyController,
        state_publisher: StatePublisher,
        processes: ChildProcessManager,
    ) -> None:
        self.hardware = hardware
        self.policy_command_source = policy_command_source
        self.operator_command_source = operator_command_source
        self.command_validator = command_validator
        self.safety = safety
        self.state_publisher = state_publisher
        self.processes = processes

    def run_once(self, controller_state: RobotControllerState) -> ControlTickResult:
        now = time.monotonic()
        now_unix = time.time()

        self.hardware.imu.request_on_tick(now)
        feedback = self.hardware.read_feedback(now)
        command = self.policy_command_source.read_latest()
        operator_command: OperatorCommand | None = self.operator_command_source.read_latest()
        validated_command, validation_error = self._validate_command(command)

        decision = self.safety.evaluate(
            SafetyInputs(
                feedback=feedback,
                command=command,
                validated_command=validated_command,
                command_validation_error=validation_error,
                process_health=self.processes.health(),
                hardware_status=self.hardware.status(),
                operator_command=operator_command,
                now=now,
                now_unix=now_unix,
            )
        )

        if decision.action == ControlAction.SEND_POLICY_COMMAND:
            if validated_command is None:
                raise RuntimeError("SafetyController requested policy command without validated command")
            self.hardware.motors.send_policy_mit_batch(validated_command)
        elif decision.action == ControlAction.SEND_DAMPING:
            self.hardware.motors.send_velocity_damping(decision.reason)
        elif decision.action == ControlAction.DISABLE_MOTORS:
            self.hardware.motors.disable_all(decision.reason)
        elif decision.action == ControlAction.NO_OUTPUT:
            pass
        elif decision.action == ControlAction.SHUTDOWN:
            self.hardware.motors.disable_all(decision.reason)
        else:
            raise RuntimeError(f"Unhandled control action: {decision.action}")

        self.state_publisher.publish(
            feedback=feedback,
            command=command,
            controller_state=controller_state,
            safety_state=self.safety.state,
            decision=decision,
        )

        return ControlTickResult(
            feedback=feedback,
            command=command,
            validated_command=validated_command,
            decision=decision,
        )

    def _validate_command(
        self,
        command: CommandReadResult,
    ) -> tuple[PolicyCommand | None, str | None]:
        if command.command is None:
            return None, None
        try:
            return self.command_validator.validate(command.command), None
        except ValueError as exc:
            return None, str(exc)
