from __future__ import annotations

import re
from dataclasses import dataclass

from robot_controller.core.config import ConfigError, RobotControllerConfig


REAL_CAN_INTERFACE = re.compile(r"^can[0-9]+$")


@dataclass(frozen=True)
class HardwareSafetyOptions:
    hardware_requested: bool
    motor_enable_confirmed: bool
    estop_ok: bool


def validate_runtime_safety(
    config: RobotControllerConfig,
    options: HardwareSafetyOptions,
) -> None:
    mode = config.runtime.mode
    interface = config.can.interface

    if mode == "simulation":
        _validate_simulation_mode(config, interface)
        return

    if mode == "hardware":
        _validate_hardware_mode(config, options, interface)
        return

    raise ConfigError("runtime.mode must be 'simulation' or 'hardware'")


def _validate_simulation_mode(config: RobotControllerConfig, interface: str) -> None:
    if REAL_CAN_INTERFACE.fullmatch(interface):
        raise ConfigError(
            f"simulation mode rejects real CAN interface '{interface}'. "
            "Use vcan or switch to explicit hardware mode."
        )
    if config.can.motors.enter_on_start:
        raise ConfigError("simulation mode forbids can.motors.enter_on_start")


def _validate_hardware_mode(
    config: RobotControllerConfig,
    options: HardwareSafetyOptions,
    interface: str,
) -> None:
    if not options.hardware_requested:
        raise ConfigError("hardware mode requires --hardware")
    if not options.motor_enable_confirmed:
        raise ConfigError("hardware mode requires --i-understand-this-can-enable-motors")
    if interface.startswith("vcan"):
        raise ConfigError("hardware mode rejects virtual CAN interfaces")
    if interface not in set(config.hardware.allowed_can_interfaces):
        allowed = ", ".join(config.hardware.allowed_can_interfaces)
        raise ConfigError(f"hardware CAN interface '{interface}' is not in allowed list: {allowed}")
    if not config.hardware.allow_real_can:
        raise ConfigError("hardware.allow_real_can must be true in hardware mode")
    if not config.hardware.require_manual_arm:
        raise ConfigError("hardware.require_manual_arm must be true in hardware mode")
    if config.hardware.allow_enable_on_start:
        raise ConfigError("hardware.allow_enable_on_start must be false in hardware mode")
    if config.can.motors.enter_on_start:
        raise ConfigError("hardware mode forbids can.motors.enter_on_start")
    if config.hardware.require_estop and not options.estop_ok:
        raise ConfigError("hardware mode requires --estop-ok when hardware.require_estop is true")
