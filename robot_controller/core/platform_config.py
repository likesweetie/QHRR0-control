from __future__ import annotations

# Compatibility layer. Platform config now lives in robot_controller.platform.config.

from robot_controller.platform.config import (
    PlatformActuatorConfig,
    PlatformCanConfig,
    PlatformConfig,
    PlatformConfigError,
    PlatformImuConfig,
    PlatformRobotAssetConfig,
    PlatformRobotConfig,
    PlatformShmConfig,
    PlatformSpgMitConfig,
    load_platform_config,
    load_yaml_mapping,
    parse_actuators,
    parse_int,
    parse_robot_assets,
)
from robot_controller.config.loader import (
    require_key,
    require_list,
    require_mapping,
    resolve_config_path,
)

__all__ = [
    "PlatformActuatorConfig",
    "PlatformCanConfig",
    "PlatformConfig",
    "PlatformConfigError",
    "PlatformImuConfig",
    "PlatformRobotAssetConfig",
    "PlatformRobotConfig",
    "PlatformShmConfig",
    "PlatformSpgMitConfig",
    "load_platform_config",
    "load_yaml_mapping",
    "parse_actuators",
    "parse_int",
    "parse_robot_assets",
    "require_key",
    "require_list",
    "require_mapping",
    "resolve_config_path",
]
