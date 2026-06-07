from __future__ import annotations

# Compatibility layer. Platform config now lives in robot_controller.platform.config.

from robot_controller.config.loader import (
    require_key,
    require_list,
    require_mapping,
    resolve_config_path,
)
from robot_controller.platform.config import (
    PlatformActuatorConfig,
    PlatformCanConfig,
    PlatformConfig,
    PlatformConfigError,
    PlatformRobotAssetConfig,
    PlatformRobotConfig,
    RobotPlatformActuatorConfig,
    RobotPlatformAssetConfig,
    RobotPlatformCanConfig,
    RobotPlatformConfig,
    RobotPlatformConfigError,
    RobotPlatformRobotConfig,
    load_platform_config,
    load_robot_platform_config,
    load_yaml_mapping,
    parse_actuators,
    parse_int,
)

__all__ = [
    "PlatformActuatorConfig",
    "PlatformCanConfig",
    "PlatformConfig",
    "PlatformConfigError",
    "PlatformRobotAssetConfig",
    "PlatformRobotConfig",
    "RobotPlatformActuatorConfig",
    "RobotPlatformAssetConfig",
    "RobotPlatformCanConfig",
    "RobotPlatformConfig",
    "RobotPlatformConfigError",
    "RobotPlatformRobotConfig",
    "load_platform_config",
    "load_robot_platform_config",
    "load_yaml_mapping",
    "parse_actuators",
    "parse_int",
    "require_key",
    "require_list",
    "require_mapping",
    "resolve_config_path",
]
