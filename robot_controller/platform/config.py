from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_controller.config.loader import (
    ConfigError,
    load_yaml_mapping,
    parse_int,
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
    _reject_removed_root_keys(raw)

    config = RobotPlatformConfig(
        path=config_path,
        robot=parse_robot_config(require_mapping(raw, "robot", "<root>")),
        assets=parse_asset_config(require_mapping(raw, "assets", "<root>")),
        can=parse_can_config(require_mapping(raw, "can", "<root>")),
        actuators=parse_actuators(require_list(raw, "actuators", "<root>")),
    )
    validate_robot_platform_config(config)
    return config


def parse_robot_config(raw: dict[str, Any]) -> RobotPlatformRobotConfig:
    return RobotPlatformRobotConfig(
        name=_require_non_empty_string(raw, "name", "robot"),
    )


def parse_asset_config(raw: dict[str, Any]) -> RobotPlatformAssetConfig:
    _reject_removed_keys(raw, {"policy_config_dir", "pd_config_path"}, "assets")
    return RobotPlatformAssetConfig(
        mujoco_model_path=_require_non_empty_string(raw, "mujoco_model_path", "assets"),
    )


def parse_can_config(raw: dict[str, Any]) -> RobotPlatformCanConfig:
    _reject_removed_keys(raw, {"interface", "bitrate", "daemon_socket"}, "can")
    allowed_interfaces = tuple(
        _non_empty_string(item, f"can.allowed_interfaces[{index}]")
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
        _reject_removed_keys(item, {"enabled"}, path)

        name = _require_non_empty_string(item, "name", path)
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
                driver=_require_non_empty_string(item, "driver", path),
                can_id=can_id,
                mujoco_joint=_require_non_empty_string(item, "mujoco_joint", path),
                mujoco_actuator=_require_non_empty_string(item, "mujoco_actuator", path),
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


def _reject_removed_root_keys(raw: dict[str, Any]) -> None:
    _reject_removed_keys(
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


def _reject_removed_keys(raw: dict[str, Any], removed_keys: set[str], path: str) -> None:
    for key in sorted(removed_keys):
        if key in raw:
            raise RobotPlatformConfigError(f"{path}.{key} is not supported by robot_platform.yaml")


def _require_non_empty_string(raw: dict[str, Any], key: str, path: str) -> str:
    return _non_empty_string(require_key(raw, key, path), f"{path}.{key}")


def _non_empty_string(value: Any, path: str) -> str:
    if value is None:
        raise RobotPlatformConfigError(f"Config key must not be empty: {path}")
    text = str(value).strip()
    if not text:
        raise RobotPlatformConfigError(f"Config key must not be empty: {path}")
    return text


# Compatibility names for the staged config refactor.
PlatformConfigError = RobotPlatformConfigError
PlatformRobotConfig = RobotPlatformRobotConfig
PlatformRobotAssetConfig = RobotPlatformAssetConfig
PlatformCanConfig = RobotPlatformCanConfig
PlatformActuatorConfig = RobotPlatformActuatorConfig
PlatformConfig = RobotPlatformConfig
load_platform_config = load_robot_platform_config
