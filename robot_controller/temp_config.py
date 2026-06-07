from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_controller.config.loader import (
    ConfigError,
    load_yaml_mapping,
    parse_int,
    require_bool,
    require_float,
    require_int,
    require_key,
    require_list,
    require_mapping,
)


class RobotPlatformConfigError(ConfigError):
    pass


@dataclass(frozen=True)
class RobotPlatformRobotConfig:
    name: str


@dataclass(frozen=True)
class RobotPlatformAssetConfig:
    mujoco_model_path: str


@dataclass(frozen=True)
class RobotPlatformCanConfig:
    allowed_interfaces: tuple[str, ...]


@dataclass(frozen=True)
class RobotPlatformActuatorConfig:
    name: str
    driver: str
    can_id: int
    mujoco_joint: str
    mujoco_actuator: str
    sign: float
    offset_rad: float


@dataclass(frozen=True)
class RobotPlatformConfig:
    path: Path
    robot: RobotPlatformRobotConfig
    assets: RobotPlatformAssetConfig
    can: RobotPlatformCanConfig
    actuators: tuple[RobotPlatformActuatorConfig, ...]


def load_robot_platform_config(path: str | Path) -> RobotPlatformConfig:
    config_path = Path(path).resolve()
    raw = load_yaml_mapping(config_path)
    _reject_removed_robot_platform_root_keys(raw)

    config = RobotPlatformConfig(
        path=config_path,
        robot=parse_robot_config(require_mapping(raw, "robot", "<root>")),
        assets=parse_asset_config(require_mapping(raw, "assets", "<root>")),
        can=parse_robot_platform_can_config(require_mapping(raw, "can", "<root>")),
        actuators=parse_actuators(require_list(raw, "actuators", "<root>")),
    )
    validate_robot_platform_config(config)
    return config


def parse_robot_config(raw: dict[str, Any]) -> RobotPlatformRobotConfig:
    return RobotPlatformRobotConfig(
        name=_require_robot_platform_non_empty_string(raw, "name", "robot"),
    )


def parse_asset_config(raw: dict[str, Any]) -> RobotPlatformAssetConfig:
    _reject_removed_robot_platform_keys(raw, {"policy_config_dir", "pd_config_path"}, "assets")
    return RobotPlatformAssetConfig(
        mujoco_model_path=_require_robot_platform_non_empty_string(
            raw,
            "mujoco_model_path",
            "assets",
        ),
    )


def parse_robot_platform_can_config(raw: dict[str, Any]) -> RobotPlatformCanConfig:
    _reject_removed_robot_platform_keys(raw, {"interface", "bitrate", "daemon_socket"}, "can")
    allowed_interfaces = tuple(
        _robot_platform_non_empty_string(item, f"can.allowed_interfaces[{index}]")
        for index, item in enumerate(require_list(raw, "allowed_interfaces", "can"))
    )
    return RobotPlatformCanConfig(allowed_interfaces=allowed_interfaces)


def parse_actuators(raw: list[Any]) -> tuple[RobotPlatformActuatorConfig, ...]:
    actuators: list[RobotPlatformActuatorConfig] = []
    seen_can_ids: set[int] = set()
    seen_names: set[str] = set()

    for index, item in enumerate(raw):
        path = f"actuators[{index}]"
        if not isinstance(item, dict):
            raise RobotPlatformConfigError(f"Config key must be a mapping: {path}")
        _reject_removed_robot_platform_keys(item, {"enabled"}, path)

        name = _require_robot_platform_non_empty_string(item, "name", path)
        can_id = parse_int(require_key(item, "can_id", path))
        if name in seen_names:
            raise RobotPlatformConfigError(f"Duplicate actuator name: {name}")
        if can_id in seen_can_ids:
            raise RobotPlatformConfigError(f"Duplicate actuator CAN ID: 0x{can_id:X}")
        seen_names.add(name)
        seen_can_ids.add(can_id)

        sign = float(require_key(item, "sign", path))
        if sign == 0.0:
            raise RobotPlatformConfigError(f"{path}.sign must not be 0")

        actuators.append(
            RobotPlatformActuatorConfig(
                name=name,
                driver=_require_robot_platform_non_empty_string(item, "driver", path),
                can_id=can_id,
                mujoco_joint=_require_robot_platform_non_empty_string(item, "mujoco_joint", path),
                mujoco_actuator=_require_robot_platform_non_empty_string(
                    item,
                    "mujoco_actuator",
                    path,
                ),
                sign=sign,
                offset_rad=float(require_key(item, "offset_rad", path)),
            )
        )

    if not actuators:
        raise RobotPlatformConfigError("actuators must not be empty")
    return tuple(actuators)


def validate_robot_platform_config(config: RobotPlatformConfig) -> None:
    if not config.robot.name:
        raise RobotPlatformConfigError("robot.name must not be empty")
    if not config.assets.mujoco_model_path:
        raise RobotPlatformConfigError("assets.mujoco_model_path must not be empty")
    if not config.can.allowed_interfaces:
        raise RobotPlatformConfigError("can.allowed_interfaces must not be empty")
    if len(set(config.can.allowed_interfaces)) != len(config.can.allowed_interfaces):
        raise RobotPlatformConfigError("can.allowed_interfaces must not contain duplicates")


def _reject_removed_robot_platform_root_keys(raw: dict[str, Any]) -> None:
    _reject_removed_robot_platform_keys(
        raw,
        {
            "robots",
            "shm",
            "imu",
            "spg_mit",
            "policy_config_dir",
            "pd_config_path",
        },
        "<root>",
    )


def _reject_removed_robot_platform_keys(
    raw: dict[str, Any],
    removed_keys: set[str],
    path: str,
) -> None:
    for key in sorted(removed_keys):
        if key in raw:
            raise RobotPlatformConfigError(f"{path}.{key} is not supported by robot_platform.yaml")


def _require_robot_platform_non_empty_string(raw: dict[str, Any], key: str, path: str) -> str:
    return _robot_platform_non_empty_string(require_key(raw, key, path), f"{path}.{key}")


def _robot_platform_non_empty_string(value: Any, path: str) -> str:
    if value is None:
        raise RobotPlatformConfigError(f"Config key must not be empty: {path}")
    text = str(value).strip()
    if not text:
        raise RobotPlatformConfigError(f"Config key must not be empty: {path}")
    return text


PlatformConfigError = RobotPlatformConfigError
PlatformRobotConfig = RobotPlatformRobotConfig
PlatformRobotAssetConfig = RobotPlatformAssetConfig
PlatformCanConfig = RobotPlatformCanConfig
PlatformActuatorConfig = RobotPlatformActuatorConfig
PlatformConfig = RobotPlatformConfig
load_platform_config = load_robot_platform_config


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
            name=_require_shm_non_empty_string(mit_command_raw, "name", "shm.mit_command"),
            target_count=target_count,
        ),
        aux_command=RobotStateShmConfig(
            name=_require_shm_non_empty_string(aux_command_raw, "name", "shm.aux_command"),
            size_bytes=require_int(aux_command_raw, "size_bytes", "shm.aux_command"),
            publish_hz=require_float(aux_command_raw, "publish_hz", "shm.aux_command"),
        ),
        operator_command=OperatorCommandShmConfig(
            name=_require_shm_non_empty_string(
                operator_command_raw,
                "name",
                "shm.operator_command",
            ),
            size_bytes=require_int(operator_command_raw, "size_bytes", "shm.operator_command"),
        ),
        control_state=RobotStateShmConfig(
            name=_require_shm_non_empty_string(control_state_raw, "name", "shm.control_state"),
            size_bytes=require_int(control_state_raw, "size_bytes", "shm.control_state"),
            publish_hz=require_float(control_state_raw, "publish_hz", "shm.control_state"),
        ),
        dashboard_state=RobotStateShmConfig(
            name=_require_shm_non_empty_string(
                dashboard_state_raw,
                "name",
                "shm.dashboard_state",
            ),
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


def _require_shm_non_empty_string(raw: dict[str, Any], key: str, path: str) -> str:
    value = require_key(raw, key, path)
    if value is None:
        raise ConfigError(f"Config key must not be empty: {path}.{key}")
    text = str(value).strip()
    if not text:
        raise ConfigError(f"Config key must not be empty: {path}.{key}")
    return text


@dataclass
class ProcessConfig:
    name: str
    command: list[str]
    start_order: int
    stop_order: int
    new_terminal: bool
    terminal_command: list[str]
    working_dir: str
    env_vars: dict[str, str]

    @property
    def env(self) -> dict[str, str]:
        return self.env_vars


def parse_process_config(item: Any, index: int) -> ProcessConfig:
    if not isinstance(item, dict):
        raise ConfigError(f"Config key must be a mapping: processes[{index}]")
    command = require_list(item, "command", f"processes[{index}]")
    if not command:
        raise ConfigError(f"Config key must not be empty: processes[{index}].command")
    if "env" in item:
        raise ConfigError("processes[*].env was renamed to processes[*].env_vars")
    env_vars_raw = require_mapping(item, "env_vars", f"processes[{index}]")
    config = ProcessConfig(
        name=str(require_key(item, "name", f"processes[{index}]")),
        command=[str(part) for part in command],
        start_order=require_int(item, "start_order", f"processes[{index}]"),
        stop_order=require_int(item, "stop_order", f"processes[{index}]"),
        new_terminal=require_bool(item, "new_terminal", f"processes[{index}]"),
        terminal_command=[
            str(part)
            for part in require_list(item, "terminal_command", f"processes[{index}]")
        ],
        working_dir=str(require_key(item, "working_dir", f"processes[{index}]")),
        env_vars={str(key): str(value) for key, value in env_vars_raw.items()},
    )
    validate_process_config(config)
    return config


def load_processes_config(path: str | Path) -> list[ProcessConfig]:
    raw = load_yaml_mapping(path)
    configs = [
        parse_process_config(item, index)
        for index, item in enumerate(require_list(raw, "processes", str(path)))
    ]
    validate_processes_config(configs)
    return configs


def validate_process_config(config: ProcessConfig) -> None:
    if not config.name:
        raise ConfigError("process name must not be empty")
    if not config.command:
        raise ConfigError(f"process {config.name} command must not be empty")
    if not config.working_dir:
        raise ConfigError(f"process {config.name} working_dir must not be empty")
    if config.new_terminal and not config.terminal_command:
        raise ConfigError(
            f"process {config.name} terminal_command must not be empty when new_terminal is true"
        )


def validate_processes_config(configs: list[ProcessConfig]) -> None:
    names = [process.name for process in configs]
    if len(set(names)) != len(names):
        raise ConfigError("processes must not contain duplicate names")
