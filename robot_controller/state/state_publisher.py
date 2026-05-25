from __future__ import annotations

import time
from typing import Callable

from robot_controller.command.policy_command import CommandReadResult
from robot_controller.core.config import RobotControllerConfig
from robot_controller.core.robot_state_shm import RobotStateShmWriter
from robot_controller.core.state import RobotControllerState
from robot_controller.hardware.robot_hardware import RobotFeedback
from robot_controller.safety.safety_state import SafetyDecision, SafetyState


class StatePublisher:
    def __init__(
        self,
        config: RobotControllerConfig,
        *,
        process_status_provider: Callable[[], dict[str, object]],
    ) -> None:
        self.config = config
        self.process_status_provider = process_status_provider
        self.control_state_writer = RobotStateShmWriter(
            name=config.shm.control_state.name,
            size_bytes=config.shm.control_state.size_bytes,
        )
        self.dashboard_state_writer = RobotStateShmWriter(
            name=config.shm.dashboard_state.name,
            size_bytes=config.shm.dashboard_state.size_bytes,
        )
        self.last_control_state_publish_t = 0.0
        self.last_dashboard_state_publish_t = 0.0

    def close(self) -> None:
        self.control_state_writer.close()
        self.dashboard_state_writer.close()

    def publish(
        self,
        *,
        feedback: RobotFeedback,
        command: CommandReadResult,
        controller_state: RobotControllerState,
        safety_state: SafetyState,
        decision: SafetyDecision,
        force: bool = False,
    ) -> None:
        now = time.monotonic()
        control_period_s = 1.0 / float(self.config.shm.control_state.publish_hz)
        dashboard_period_s = 1.0 / float(self.config.shm.dashboard_state.publish_hz)

        if force or now - self.last_control_state_publish_t >= control_period_s:
            self.control_state_writer.publish(
                self._control_state_snapshot(
                    feedback=feedback,
                    command=command,
                    controller_state=controller_state,
                    safety_state=safety_state,
                    decision=decision,
                    now=now,
                )
            )
            self.last_control_state_publish_t = now

        if force or now - self.last_dashboard_state_publish_t >= dashboard_period_s:
            self.dashboard_state_writer.publish(
                self._dashboard_state_snapshot(
                    feedback=feedback,
                    command=command,
                    controller_state=controller_state,
                    safety_state=safety_state,
                    decision=decision,
                    now=now,
                )
            )
            self.last_dashboard_state_publish_t = now

    def _control_state_snapshot(
        self,
        *,
        feedback: RobotFeedback,
        command: CommandReadResult,
        controller_state: RobotControllerState,
        safety_state: SafetyState,
        decision: SafetyDecision,
        now: float,
    ) -> dict:
        imu = feedback.imu.state
        return {
            "schema": "qhrr.control_state.v1",
            "timestamp_monotonic": now,
            "timestamp_unix": time.time(),
            "controller_state": controller_state.name,
            "safety_state": safety_state.name,
            "control_action": decision.action.name,
            "safety_reason": decision.reason,
            "fault_code": decision.fault_code,
            "policy_command": self._command_snapshot(command),
            "imu": {
                "quat_xyzw": list(imu.quat_xyzw) if imu.quat_xyzw is not None else None,
                "projected_gravity_b": (
                    list(imu.projected_gravity_b)
                    if imu.projected_gravity_b is not None
                    else None
                ),
                "angular_velocity_rad_s": (
                    list(imu.angular_velocity_rad_s)
                    if imu.angular_velocity_rad_s is not None
                    else None
                ),
                "last_quat_t": imu.last_quat_t,
                "last_gyro_t": imu.last_gyro_t,
                "quat_online": feedback.imu.quat_online,
                "gyro_online": feedback.imu.gyro_online,
                "quat_stale": feedback.imu.quat_stale,
                "gyro_stale": feedback.imu.gyro_stale,
            },
            "actuators": [
                self._control_actuator_snapshot(item)
                for item in feedback.motors.actuators
            ],
        }

    def _dashboard_state_snapshot(
        self,
        *,
        feedback: RobotFeedback,
        command: CommandReadResult,
        controller_state: RobotControllerState,
        safety_state: SafetyState,
        decision: SafetyDecision,
        now: float,
    ) -> dict:
        imu = feedback.imu.state
        return {
            "schema": "qhrr.dashboard_state.v1",
            "timestamp_monotonic": now,
            "timestamp_unix": time.time(),
            "controller_state": controller_state.name,
            "safety_state": safety_state.name,
            "control_action": decision.action.name,
            "safety_reason": decision.reason,
            "fault_code": decision.fault_code,
            "policy_command": self._command_snapshot(command),
            "can": {
                "iface": self.config.can.interface,
                "command_timeout_s": self.config.can.command_timeout_s,
            },
            "processes": self.process_status_provider(),
            "imu": {
                "quat_xyzw": list(imu.quat_xyzw) if imu.quat_xyzw is not None else None,
                "projected_gravity_b": (
                    list(imu.projected_gravity_b)
                    if imu.projected_gravity_b is not None
                    else None
                ),
                "angular_velocity_rad_s": (
                    list(imu.angular_velocity_rad_s)
                    if imu.angular_velocity_rad_s is not None
                    else None
                ),
                "last_quat_t": imu.last_quat_t,
                "last_gyro_t": imu.last_gyro_t,
                "quat_comm": self._freshness_to_dict(feedback.imu.quat_comm),
                "gyro_comm": self._freshness_to_dict(feedback.imu.gyro_comm),
            },
            "actuators": [
                self._actuator_snapshot(item)
                for item in feedback.motors.actuators
            ],
        }

    @staticmethod
    def _command_snapshot(command: CommandReadResult) -> dict:
        return {
            "status": command.status.name,
            "reason": command.reason,
            "timestamp": command.timestamp,
            "source": None if command.command is None else command.command.source,
        }

    @staticmethod
    def _control_actuator_snapshot(item) -> dict:
        state = item.state
        return {
            "can_id": item.can_id,
            "position_rad": state.position_rad,
            "velocity_rad_s": state.velocity_rad_s,
            "torque_nm": state.torque_nm,
            "current_a": state.current_a,
            "is_enabled": state.is_enabled,
            "fault_code": state.fault_code,
            "last_feedback_t": state.last_feedback_t,
            "age_s": item.age_s,
            "online": item.online,
            "stale": item.stale,
        }

    def _actuator_snapshot(self, item) -> dict:
        state = item.state
        return {
            "can_id": item.can_id,
            "name": item.name,
            "position_rad": state.position_rad,
            "velocity_rad_s": state.velocity_rad_s,
            "torque_nm": state.torque_nm,
            "current_a": state.current_a,
            "temperature_c": state.temperature_c,
            "voltage_v": state.voltage_v,
            "fault_code": state.fault_code,
            "is_enabled": state.is_enabled,
            "mode": state.mode,
            "last_feedback_t": state.last_feedback_t,
            "age_s": item.age_s,
            "raw": state.raw,
            "comm": self._freshness_to_dict(item.comm),
        }

    @staticmethod
    def _freshness_to_dict(status) -> dict:
        return {
            "is_online": bool(status.is_online),
            "is_stale": bool(status.is_stale),
            "rx_count": int(status.rx_count),
            "timeout_count": int(status.timeout_count),
            "decode_error_count": int(status.decode_error_count),
            "last_rx_t": float(status.last_rx_t),
            "last_fault_t": float(status.last_fault_t),
        }
