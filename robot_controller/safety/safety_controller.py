from __future__ import annotations

from dataclasses import dataclass

from robot_controller.command.policy_command import CommandReadResult, CommandReadStatus, PolicyCommand
from robot_controller.core.config import RobotControllerConfig
from robot_controller.hardware.robot_hardware import HardwareStatus, RobotFeedback
from robot_controller.processes.child_process_manager import ProcessHealth

from .safety_state import ControlAction, SafetyDecision, SafetyState


@dataclass(frozen=True)
class OperatorCommand:
    arm: bool = False
    clear_fault: bool = False
    estop: bool = False


@dataclass(frozen=True)
class SafetyInputs:
    feedback: RobotFeedback
    command: CommandReadResult
    validated_command: PolicyCommand | None
    command_validation_error: str | None
    process_health: ProcessHealth
    hardware_status: HardwareStatus
    operator_command: OperatorCommand | None
    now: float
    now_unix: float


class SafetyController:
    def __init__(self, config: RobotControllerConfig):
        self.config = config
        self.state = SafetyState.CREATED
        self._damping_started_t: float | None = None
        self._last_damping_reason: str | None = None
        self._fault_reason: str | None = None
        self._fault_action: ControlAction | None = None
        self._operator_damping_latched = False

    def start(self) -> SafetyDecision:
        if self.config.runtime.mode == "hardware":
            self.state = SafetyState.DISARMED
            return SafetyDecision(
                state=self.state,
                action=ControlAction.NO_OUTPUT,
                reason="hardware mode starts disarmed and requires operator arming",
            )
        self.state = SafetyState.RUNNING
        return SafetyDecision(
            state=self.state,
            action=ControlAction.NO_OUTPUT,
            reason="simulation mode starts running after startup gates pass",
        )

    def begin_shutdown(self) -> SafetyDecision:
        self.state = SafetyState.SHUTTING_DOWN
        return SafetyDecision(
            state=self.state,
            action=ControlAction.DISABLE_MOTORS,
            reason="controller shutdown",
        )

    def stop(self) -> None:
        self.state = SafetyState.STOPPED

    def evaluate(self, inputs: SafetyInputs) -> SafetyDecision:
        operator = inputs.operator_command
        if operator is not None and operator.estop:
            self._operator_damping_latched = True

        if self.state == SafetyState.FAULT_LATCHED:
            if operator is not None and operator.clear_fault:
                self.state = SafetyState.DISARMED
                self._fault_reason = None
                self._fault_action = None
                self._operator_damping_latched = False
                return SafetyDecision(
                    state=self.state,
                    action=ControlAction.NO_OUTPUT,
                    reason="operator cleared fault; controller is disarmed",
                )
            if not inputs.hardware_status.can_connected:
                return self._fault("CAN daemon client is not connected")
            if not inputs.process_health.can_daemon_alive:
                return self._fault("CAN daemon process is dead")
            if self._fault_action == ControlAction.SEND_DAMPING:
                reason = self._fault_reason or "fault latched"
                if self._has_mit_enabled_actuator(inputs.feedback):
                    return SafetyDecision(
                        state=self.state,
                        action=ControlAction.SEND_DAMPING,
                        reason=reason,
                        fault_code="FAULT_LATCHED",
                    )
                return SafetyDecision(
                    state=self.state,
                    action=ControlAction.NO_OUTPUT,
                    reason=f"{reason}; no MIT-enabled actuator feedback",
                    fault_code="FAULT_LATCHED",
                )
            return SafetyDecision(
                state=self.state,
                action=ControlAction.NO_OUTPUT,
                reason=self._fault_reason or "fault latched",
                fault_code="FAULT_LATCHED",
            )

        if self.state == SafetyState.DISARMED:
            if operator is None or not operator.arm:
                return SafetyDecision(
                    state=self.state,
                    action=ControlAction.NO_OUTPUT,
                    reason="controller disarmed",
                )
            self.state = SafetyState.ARMING

        if not inputs.hardware_status.can_connected:
            return self._fault("CAN daemon client is not connected")
        if not inputs.process_health.can_daemon_alive:
            return self._fault("CAN daemon process is dead")

        if self._operator_damping_latched:
            if operator is not None and operator.arm:
                self._operator_damping_latched = False
                self._damping_started_t = None
                self._last_damping_reason = None
                self.state = SafetyState.ARMING
            else:
                return self._operator_damping(inputs, "operator E-stop")

        if not inputs.process_health.task_controller_alive:
            return self._enter_command_loss_damping(inputs, "task_controller process is dead")
        if not inputs.process_health.aux_reader_alive and self.config.runtime.mode == "hardware":
            return self._fault("aux_reader process is dead in hardware mode")

        if self.state == SafetyState.ARMING:
            self.state = SafetyState.RUNNING

        if inputs.command_validation_error is not None:
            return self._fault(f"invalid policy command: {inputs.command_validation_error}")

        if inputs.command.status != CommandReadStatus.AVAILABLE or inputs.command.command is None:
            return self._enter_command_loss_damping(inputs, inputs.command.reason)

        age_s = inputs.now_unix - inputs.command.command.timestamp
        if age_s > self.config.can.command_timeout_s:
            return self._enter_command_loss_damping(
                inputs,
                f"policy command stale: source={inputs.command.command.source}",
            )

        if inputs.validated_command is None:
            return self._fault("policy command was not validated")

        if inputs.feedback.motors.has_stale_feedback:
            return self._feedback_stale(inputs.now, inputs.feedback.motors.stale_reason)
        if inputs.feedback.imu.has_stale_feedback:
            return self._feedback_stale(inputs.now, inputs.feedback.imu.stale_reason)

        if self.state == SafetyState.DAMPING:
            if (
                self._damping_started_t is not None
                and inputs.now - self._damping_started_t >= self.config.safety.damping_timeout_s
            ):
                return self._damping_timeout_fault(inputs)
            self.state = SafetyState.RUNNING
            self._damping_started_t = None
            self._last_damping_reason = None
            return SafetyDecision(
                state=self.state,
                action=ControlAction.SEND_POLICY_COMMAND,
                reason=f"recovered from damping with fresh policy command from {inputs.command.command.source}",
            )

        self.state = SafetyState.RUNNING
        self._damping_started_t = None
        self._last_damping_reason = None
        return SafetyDecision(
            state=self.state,
            action=ControlAction.SEND_POLICY_COMMAND,
            reason=f"fresh policy command from {inputs.command.command.source}",
        )

    def _feedback_stale(self, now: float, reason: str) -> SafetyDecision:
        action = self.config.safety.feedback_stale_action
        if action == "fault":
            return self._fault(reason)
        if action == "disable":
            return self._fault(reason, action=ControlAction.DISABLE_MOTORS)
        return self._enter_damping(now, reason)

    def _enter_command_loss_damping(self, inputs: SafetyInputs, reason: str) -> SafetyDecision:
        action = self.config.safety.command_loss_action
        if action == "fault":
            return self._fault(reason)
        if action == "disable":
            return self._fault(reason, action=ControlAction.DISABLE_MOTORS)
        if not self._has_mit_enabled_actuator(inputs.feedback):
            self.state = SafetyState.DAMPING
            self._damping_started_t = None
            self._last_damping_reason = reason
            return SafetyDecision(
                state=self.state,
                action=ControlAction.NO_OUTPUT,
                reason=f"{reason}; no MIT-enabled actuator feedback",
            )
        return self._enter_damping(inputs.now, reason, repeat_output=True)

    def _enter_damping(self, now: float, reason: str, *, repeat_output: bool = False) -> SafetyDecision:
        entered = (
            self.state != SafetyState.DAMPING
            or self._last_damping_reason != reason
            or self._damping_started_t is None
        )
        if entered:
            self._damping_started_t = now
            self._last_damping_reason = reason
        assert self._damping_started_t is not None
        if now - self._damping_started_t >= self.config.safety.damping_timeout_s:
            timeout_action = ControlAction.SEND_DAMPING if repeat_output else ControlAction.NO_OUTPUT
            return self._fault("damping timeout elapsed", action=timeout_action)
        self.state = SafetyState.DAMPING
        return SafetyDecision(
            state=self.state,
            action=ControlAction.SEND_DAMPING if repeat_output or entered else ControlAction.NO_OUTPUT,
            reason=reason,
        )

    def _operator_damping(self, inputs: SafetyInputs, reason: str) -> SafetyDecision:
        self.state = SafetyState.DAMPING
        self._last_damping_reason = reason
        self._damping_started_t = inputs.now
        return SafetyDecision(
            state=self.state,
            action=(
                ControlAction.SEND_DAMPING
                if self._has_mit_enabled_actuator(inputs.feedback)
                else ControlAction.NO_OUTPUT
            ),
            reason=reason,
        )

    @staticmethod
    def _has_mit_enabled_actuator(feedback: RobotFeedback) -> bool:
        for item in getattr(feedback.motors, "actuators", ()):
            state = getattr(item, "state", None)
            if getattr(state, "is_enabled", None) is True:
                return True
        return False

    def _fault(
        self,
        reason: str,
        *,
        action: ControlAction = ControlAction.DISABLE_MOTORS,
    ) -> SafetyDecision:
        self.state = SafetyState.FAULT_LATCHED
        self._fault_reason = reason
        self._fault_action = action
        return SafetyDecision(
            state=self.state,
            action=action,
            reason=reason,
            fault_code="FAULT_LATCHED",
        )

    def _damping_timeout_fault(self, inputs: SafetyInputs) -> SafetyDecision:
        action = (
            ControlAction.SEND_DAMPING
            if self._has_mit_enabled_actuator(inputs.feedback)
            else ControlAction.NO_OUTPUT
        )
        return self._fault("damping timeout elapsed", action=action)
