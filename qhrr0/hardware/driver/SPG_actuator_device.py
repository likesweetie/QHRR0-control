"""
RobotController adapter for DongilC/OpenRobot SPG actuators.

This module is the boundary where driver-level device data is converted to the
robot_controller hardware interface protocol.
"""

from __future__ import annotations

from dataclasses import fields

from qhrr0.app.hal.can import CANFrame
from qhrr0.app.robot_controller.hardware_interfaces import protocol as controller_protocol

from .SPG_actuator_driver import (
    ActuatorState as DriverActuatorState,
    SPGActuatorDriver,
    SPG_IQ_COUNT_TO_AMP,
    SPG_MIT_DEFAULT_CONFIG,
    SPGMITConfig,
)


class SPGActuatorDevice:
    def __init__(
        self,
        *,
        command_id: int,
        feedback_id: int | None = None,
        mit_config: SPGMITConfig = SPG_MIT_DEFAULT_CONFIG,
        gear_ratio: float = 1.0,
        feedback_speed_is_motor_side: bool = True,
        expose_single_turn_position: bool = False,
        iq_count_to_amp: float | None = SPG_IQ_COUNT_TO_AMP,
        torque_constant_nm_per_a: float | None = None,
        gear_efficiency: float = 1.0,
    ) -> None:
        if feedback_id is None:
            raise ValueError("feedback_id must be configured explicitly")
        if gear_ratio <= 0.0:
            raise ValueError("gear_ratio must be positive")

        self.driver = SPGActuatorDriver(
            command_id=command_id,
            feedback_id=feedback_id,
            mit_config=mit_config,
        )
        self.gear_ratio = float(gear_ratio)
        self.feedback_speed_is_motor_side = bool(feedback_speed_is_motor_side)
        self.expose_single_turn_position = bool(expose_single_turn_position)
        self.iq_count_to_amp = iq_count_to_amp
        self.torque_constant_nm_per_a = torque_constant_nm_per_a
        self.gear_efficiency = float(gear_efficiency)

    def rx_can_ids(self) -> list[int]:
        return self.driver.rx_can_ids()

    def decode_frame(self, frame: CANFrame) -> controller_protocol.ActuatorState | None:
        raw_state = self.driver.decode_frame(frame)
        if raw_state is None:
            return None
        return self.to_controller_state(raw_state)

    def encode_command(self, command: controller_protocol.ActuatorCommand) -> CANFrame:
        mode_name = getattr(command.mode, "name", str(command.mode)).upper()
        if mode_name == "ENABLE":
            return self.driver.encode_enable_frame()
        if mode_name == "DISABLE":
            return self.driver.encode_disable_frame()
        if mode_name in {"SET_ZERO", "ZERO_POSITION"}:
            return self.driver.encode_zero_position_frame(offset_deg=0.0)
        if mode_name == "CONTROL":
            return self.driver.encode_impedance_command_frame(
                position_rad=float(command.position_rad),
                velocity_rad_s=float(command.velocity_rad_s),
                kp=float(command.kp),
                kd=float(command.kd),
                torque_ff_nm=float(command.torque_nm),
            )
        raise ValueError(f"Unsupported controller actuator command mode: {command.mode}")

    def to_controller_state(
        self,
        raw_state: DriverActuatorState,
        freshness: controller_protocol.Freshness | None = None,
    ) -> controller_protocol.ActuatorState:
        velocity_rad_s = raw_state.velocity_rad_s
        if velocity_rad_s is not None and self.feedback_speed_is_motor_side:
            velocity_rad_s /= self.gear_ratio

        current_a = raw_state.current_a
        if current_a is None and raw_state.iq_counts is not None and self.iq_count_to_amp is not None:
            current_a = raw_state.iq_counts * self.iq_count_to_amp

        torque_nm = raw_state.torque_nm
        if torque_nm is None and current_a is not None and self.torque_constant_nm_per_a is not None:
            torque_nm = (
                current_a
                * self.torque_constant_nm_per_a
                * self.gear_ratio
                * self.gear_efficiency
            )

        position_rad = raw_state.position_rad
        if not self.expose_single_turn_position:
            position_rad = None

        fault_code = 0 if raw_state.fault_code is None else int(raw_state.fault_code)
        return _make_controller_actuator_state(
            position_rad=0.0 if position_rad is None else float(position_rad),
            velocity_rad_s=0.0 if velocity_rad_s is None else float(velocity_rad_s),
            torque_nm=0.0 if torque_nm is None else float(torque_nm),
            enabled=bool(raw_state.is_enabled) if raw_state.is_enabled is not None else False,
            faulted=fault_code != 0,
            last_fault_code=fault_code,
            freshness=freshness or controller_protocol.Freshness(last_update_t=raw_state.last_feedback_t),
        )

    def make_enable_frame(self) -> CANFrame:
        return self.driver.encode_enable_frame()

    def make_disable_frame(self) -> CANFrame:
        return self.driver.encode_disable_frame()

    def make_clear_fault_frame(self) -> CANFrame:
        return self.driver.encode_clear_fault_frame()

    def make_impedance_command_frame(
        self,
        *,
        position_rad: float,
        velocity_rad_s: float,
        kp: float,
        kd: float,
        torque_ff_nm: float = 0.0,
    ) -> CANFrame:
        return self.driver.encode_impedance_command_frame(
            position_rad=position_rad,
            velocity_rad_s=velocity_rad_s,
            kp=kp,
            kd=kd,
            torque_ff_nm=torque_ff_nm,
        )

    def make_zero_position_frame(self, offset_deg: float = 0.0) -> CANFrame:
        return self.driver.encode_zero_position_frame(offset_deg=offset_deg)


SPGActuatorProtocol = SPGActuatorDevice


def _make_controller_actuator_state(**values) -> controller_protocol.ActuatorState:
    supported_fields = {field.name for field in fields(controller_protocol.ActuatorState)}
    return controller_protocol.ActuatorState(
        **{
            name: value
            for name, value in values.items()
            if name in supported_fields
        }
    )
