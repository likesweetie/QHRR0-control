from __future__ import annotations

from robot_controller.temp_config import (
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
    parse_actuators,
    parse_asset_config,
    parse_robot_config,
    parse_robot_platform_can_config,
    validate_robot_platform_config,
)

parse_can_config = parse_robot_platform_can_config

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
    "parse_actuators",
    "parse_asset_config",
    "parse_can_config",
    "parse_robot_config",
    "parse_robot_platform_can_config",
    "validate_robot_platform_config",
]
