from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from robot_controller.config.loader import (
    ConfigError,
    require_bool,
    require_float,
    require_int,
    require_mapping,
)
from robot_controller.platform.config import PlatformConfig


@dataclass
class MitCommandShmConfig:
    name: str
    target_count: int


@dataclass
class RobotStateShmConfig:
    name: str
    size_bytes: int
    publish_hz: float


@dataclass
class OperatorCommandShmConfig:
    name: str
    size_bytes: int


@dataclass
class ShmConfig:
    cleanup_stale_on_start: bool
    unlink_on_shutdown: bool
    mit_command: MitCommandShmConfig
    aux_command: RobotStateShmConfig
    operator_command: OperatorCommandShmConfig
    control_state: RobotStateShmConfig
    dashboard_state: RobotStateShmConfig


def parse_shm_config(raw: dict[str, Any], platform: PlatformConfig) -> ShmConfig:
    aux_command_raw = require_mapping(raw, "aux_command", "shm")
    operator_command_raw = require_mapping(raw, "operator_command", "shm")
    control_state_raw = require_mapping(raw, "control_state", "shm")
    dashboard_state_raw = require_mapping(raw, "dashboard_state", "shm")

    config = ShmConfig(
        cleanup_stale_on_start=require_bool(raw, "cleanup_stale_on_start", "shm"),
        unlink_on_shutdown=require_bool(raw, "unlink_on_shutdown", "shm"),
        mit_command=MitCommandShmConfig(
            name=platform.shm.mit_command,
            target_count=len(platform.enabled_actuators),
        ),
        aux_command=RobotStateShmConfig(
            name=platform.shm.aux_command,
            size_bytes=require_int(aux_command_raw, "size_bytes", "shm.aux_command"),
            publish_hz=require_float(aux_command_raw, "publish_hz", "shm.aux_command"),
        ),
        operator_command=OperatorCommandShmConfig(
            name=platform.shm.operator_command,
            size_bytes=require_int(operator_command_raw, "size_bytes", "shm.operator_command"),
        ),
        control_state=RobotStateShmConfig(
            name=platform.shm.control_state,
            size_bytes=require_int(control_state_raw, "size_bytes", "shm.control_state"),
            publish_hz=require_float(control_state_raw, "publish_hz", "shm.control_state"),
        ),
        dashboard_state=RobotStateShmConfig(
            name=platform.shm.dashboard_state,
            size_bytes=require_int(dashboard_state_raw, "size_bytes", "shm.dashboard_state"),
            publish_hz=require_float(dashboard_state_raw, "publish_hz", "shm.dashboard_state"),
        ),
    )
    validate_shm_config(config)
    return config


def validate_shm_config(config: ShmConfig) -> None:
    if config.mit_command.target_count <= 0:
        raise ConfigError("shm.mit_command.target_count must be > 0")
    if config.control_state.size_bytes < 4096:
        raise ConfigError("shm.control_state.size_bytes must be >= 4096")
    if config.control_state.publish_hz <= 0.0:
        raise ConfigError("shm.control_state.publish_hz must be > 0")
    if config.aux_command.size_bytes < 4096:
        raise ConfigError("shm.aux_command.size_bytes must be >= 4096")
    if config.aux_command.publish_hz <= 0.0:
        raise ConfigError("shm.aux_command.publish_hz must be > 0")
    if config.operator_command.size_bytes < 4096:
        raise ConfigError("shm.operator_command.size_bytes must be >= 4096")
    if config.dashboard_state.size_bytes < 4096:
        raise ConfigError("shm.dashboard_state.size_bytes must be >= 4096")
    if config.dashboard_state.publish_hz <= 0.0:
        raise ConfigError("shm.dashboard_state.publish_hz must be > 0")

    names = {
        config.mit_command.name,
        config.aux_command.name,
        config.operator_command.name,
        config.control_state.name,
        config.dashboard_state.name,
    }
    if len(names) != 5:
        raise ConfigError("shm segment names must be unique")
