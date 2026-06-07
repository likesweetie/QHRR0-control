from __future__ import annotations

from dataclasses import dataclass

from robot_controller.config.loader import ConfigError


@dataclass(frozen=True)
class HardwareSafetyOptions:
    hardware_requested: bool
    motor_enable_confirmed: bool
    estop_ok: bool


def validate_app_config(config) -> None:
    if config.runtime.mode not in ("simulation", "hardware"):
        raise ConfigError("runtime.mode must be 'simulation' or 'hardware'")
    if not config.robot_platform.can.allowed_interfaces:
        raise ConfigError("robot_platform.can.allowed_interfaces must not be empty")
    if config.can.interface not in set(config.robot_platform.can.allowed_interfaces):
        raise ConfigError("can.interface must be listed in robot_platform.can.allowed_interfaces")
    if config.safety.velocity_damping_kd < 0.0:
        raise ConfigError("safety.velocity_damping_kd must be >= 0")
    if config.safety.damping_timeout_s <= 0.0:
        raise ConfigError("safety.damping_timeout_s must be > 0")
    if config.safety.command_loss_action not in ("damping", "disable", "fault"):
        raise ConfigError("safety.command_loss_action must be damping, disable, or fault")
    if config.safety.feedback_stale_action not in ("damping", "disable", "fault"):
        raise ConfigError("safety.feedback_stale_action must be damping, disable, or fault")
    if config.robot_controller.control_hz <= 0.0:
        raise ConfigError("robot_controller.control_hz must be > 0")
    if config.robot_controller.shutdown_timeout_s < 0.0:
        raise ConfigError("robot_controller.shutdown_timeout_s must be >= 0")
    if config.state_machine.enable_duration_s < 0.0:
        raise ConfigError("state_machine.enable_duration_s must be >= 0")

    if config.shm.mit_command.target_count != len(config.can.motors.can_ids):
        raise ConfigError("shm.mit_command.target_count must match len(can.motors.can_ids)")
    if config.safety.velocity_damping_kd > config.can.mit_protocol_range.kd:
        raise ConfigError("safety.velocity_damping_kd must be <= can.mit_protocol_range.kd")

    process_names = {process.name for process in config.processes}
    if "can_daemon" not in process_names:
        raise ConfigError("processes must include a 'can_daemon' subprocess")


def validate_runtime_safety(config, options: HardwareSafetyOptions) -> None:
    interface = str(config.can.interface)
    is_virtual_can = interface.startswith("vcan")
    is_real_can = interface.startswith("can")

    if config.runtime.mode == "simulation":
        if is_real_can:
            raise ConfigError("simulation mode rejects real CAN interface")
        return

    if config.runtime.mode != "hardware":
        raise ConfigError("runtime.mode must be 'simulation' or 'hardware'")
    if not options.hardware_requested:
        raise ConfigError("hardware mode requires --hardware")
    if not options.motor_enable_confirmed:
        raise ConfigError("hardware mode requires --i-understand-this-can-enable-motors")
    if is_virtual_can:
        raise ConfigError("hardware mode rejects virtual CAN interface")
    if not is_real_can:
        raise ConfigError("hardware mode requires a real CAN interface")
    if interface not in set(config.robot_platform.can.allowed_interfaces):
        raise ConfigError("hardware CAN interface must be listed in robot_platform.can.allowed_interfaces")
    if not config.hardware.allow_real_can:
        raise ConfigError("hardware mode requires hardware.allow_real_can")
    if not options.estop_ok:
        raise ConfigError("hardware mode requires --estop-ok")
