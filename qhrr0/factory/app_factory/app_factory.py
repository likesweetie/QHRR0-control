# factory/app_factory/app_factory.py
from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

from factory.robot_factory.base.robot_base import Robot

from .app_validation_rules import (
    AppValidationContext,
    validate_app_by_rules,
)


@dataclass(frozen=True, slots=True, kw_only=True)
class HardwareSafetyOptions:
    hardware_requested: bool = False
    motor_enable_confirmed: bool = False
    estop_ok: bool = False


@dataclass(frozen=True, slots=True, kw_only=True)
class ActuatorBuildSpec:
    name: str
    driver: str
    can_id: int
    sign: float
    offset_rad: float
    group: str


@dataclass(frozen=True, slots=True, kw_only=True)
class ImuBuildSpec:
    name: str
    driver: str
    can_id: int


@dataclass(frozen=True, slots=True, kw_only=True)
class ControllerBuildSpec:
    robot_name: str
    controller_name: str
    controller_type: str

    runtime_mode: str
    control_hz: float
    shutdown_timeout_s: float

    can_interface: str
    can_bitrate: int
    can_command_timeout_s: float

    velocity_damping_kd: float
    damping_timeout_s: float
    command_loss_action: str
    feedback_stale_action: str

    actuators: tuple[ActuatorBuildSpec, ...]
    imu: ImuBuildSpec

    metadata: dict[str, Any] | None = None


@dataclass(frozen=True, slots=True, kw_only=True)
class ProcessBuildSpec:
    name: str
    command: tuple[str, ...]
    start_order: int
    stop_order: int
    new_terminal: bool
    terminal_command: tuple[str, ...]
    working_dir: str
    env_vars: dict[str, str]


@dataclass(frozen=True, slots=True, kw_only=True)
class AppBuildSpec:
    controller: ControllerBuildSpec
    processes: tuple[ProcessBuildSpec, ...]


class AppFactory:
    """Factory for creating app construction specifications.

    This factory must not import app runtime implementations.
    It only validates config and emits implementation-agnostic build specs.
    """

    DEFAULT_VALIDATION_RULES: tuple[str, ...] = (
        "runtime_mode",
        "can_interface_supported_by_robot",
        "safety_config",
        "robot_controller_config",
        "state_machine_config",
        "safety_vs_protocol_range",
        "can_config",
        "mit_protocol_range",
        "processes",
        "shm",
        "hardware_safety",
    )

    def create_app_spec(
        self,
        *,
        robot: Robot,
        config: object,
        options: HardwareSafetyOptions | None = None,
        required_validation_rules: Iterable[str] | None = None,
    ) -> AppBuildSpec:
        rules = (
            tuple(required_validation_rules)
            if required_validation_rules is not None
            else self.DEFAULT_VALIDATION_RULES
        )

        ctx = AppValidationContext(
            config=config,
            robot=robot,
            options=options,
        )
        validate_app_by_rules(ctx, rules)

        return AppBuildSpec(
            controller=self._create_controller_build_spec(
                robot=robot,
                config=config,
            ),
            processes=self._create_process_build_specs(config),
        )

    def _create_controller_build_spec(
        self,
        *,
        robot: Robot,
        config: object,
    ) -> ControllerBuildSpec:
        return ControllerBuildSpec(
            robot_name=robot.name,
            controller_name=robot.controller.name,
            controller_type=robot.controller.controller_type,
            runtime_mode=str(config.runtime.mode),
            control_hz=float(config.robot_controller.control_hz),
            shutdown_timeout_s=float(config.robot_controller.shutdown_timeout_s),
            can_interface=str(config.can.interface),
            can_bitrate=int(config.can.bitrate),
            can_command_timeout_s=float(config.can.command_timeout_s),
            velocity_damping_kd=float(config.safety.velocity_damping_kd),
            damping_timeout_s=float(config.safety.damping_timeout_s),
            command_loss_action=str(config.safety.command_loss_action),
            feedback_stale_action=str(config.safety.feedback_stale_action),
            actuators=tuple(
                ActuatorBuildSpec(
                    name=actuator.name,
                    driver=actuator.driver,
                    can_id=actuator.can_id,
                    sign=actuator.sign,
                    offset_rad=actuator.offset_rad,
                    group=actuator.group,
                )
                for actuator in robot.actuators
            ),
            imu=ImuBuildSpec(
                name=robot.imu.name,
                driver=robot.imu.driver,
                can_id=robot.imu.can_id,
            ),
            metadata=robot.controller.metadata,
        )

    def _create_process_build_specs(self, config: object) -> tuple[ProcessBuildSpec, ...]:
        return tuple(
            ProcessBuildSpec(
                name=str(process.name),
                command=tuple(str(part) for part in process.command),
                start_order=int(process.start_order),
                stop_order=int(process.stop_order),
                new_terminal=bool(process.new_terminal),
                terminal_command=tuple(str(part) for part in process.terminal_command),
                working_dir=str(process.working_dir),
                env_vars={
                    str(key): str(value)
                    for key, value in process.env_vars.items()
                },
            )
            for process in config.processes
        )