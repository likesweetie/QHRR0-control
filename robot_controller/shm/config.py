from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from robot_controller.config.loader import (
    ConfigError,
    require_float,
    require_int,
    require_key,
    require_mapping,
)


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
    mit_command: MitCommandShmConfig
    aux_command: RobotStateShmConfig
    operator_command: OperatorCommandShmConfig
    control_state: RobotStateShmConfig
    dashboard_state: RobotStateShmConfig


def parse_shm_config(raw: dict[str, Any], target_count: int) -> ShmConfig:
    if "cleanup_stale_on_start" in raw:
        raise ConfigError("shm.cleanup_stale_on_start was removed; cleanup is controller default behavior")
    if "unlink_on_shutdown" in raw:
        raise ConfigError("shm.unlink_on_shutdown was removed; unlink is controller default behavior")

    mit_command_raw = require_mapping(raw, "mit_command", "shm")
    aux_command_raw = require_mapping(raw, "aux_command", "shm")
    operator_command_raw = require_mapping(raw, "operator_command", "shm")
    control_state_raw = require_mapping(raw, "control_state", "shm")
    dashboard_state_raw = require_mapping(raw, "dashboard_state", "shm")

    config = ShmConfig(
        mit_command=MitCommandShmConfig(
            name=_require_non_empty_string(mit_command_raw, "name", "shm.mit_command"),
            target_count=target_count,
        ),
        aux_command=RobotStateShmConfig(
            name=_require_non_empty_string(aux_command_raw, "name", "shm.aux_command"),
            size_bytes=require_int(aux_command_raw, "size_bytes", "shm.aux_command"),
            publish_hz=require_float(aux_command_raw, "publish_hz", "shm.aux_command"),
        ),
        operator_command=OperatorCommandShmConfig(
            name=_require_non_empty_string(operator_command_raw, "name", "shm.operator_command"),
            size_bytes=require_int(operator_command_raw, "size_bytes", "shm.operator_command"),
        ),
        control_state=RobotStateShmConfig(
            name=_require_non_empty_string(control_state_raw, "name", "shm.control_state"),
            size_bytes=require_int(control_state_raw, "size_bytes", "shm.control_state"),
            publish_hz=require_float(control_state_raw, "publish_hz", "shm.control_state"),
        ),
        dashboard_state=RobotStateShmConfig(
            name=_require_non_empty_string(dashboard_state_raw, "name", "shm.dashboard_state"),
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


def _require_non_empty_string(raw: dict[str, Any], key: str, path: str) -> str:
    value = require_key(raw, key, path)
    if value is None:
        raise ConfigError(f"Config key must not be empty: {path}.{key}")
    text = str(value).strip()
    if not text:
        raise ConfigError(f"Config key must not be empty: {path}.{key}")
    return text
