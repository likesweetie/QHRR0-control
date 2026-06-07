from __future__ import annotations

# Compatibility layer. New code should import from robot_controller.config,
# robot_controller.platform.config, robot_controller.shm.config, or
# robot_controller.supervisor.config.

from robot_controller.config.can import (
    CANDaemonConfig,
    CanConfig,
    ImuConfig,
    MitProtocolRangeConfig,
    MotorConfig,
)
from robot_controller.config.can_device import (
    CanDeviceConfig,
    CanDeviceConfigError,
    CanDeviceImuConfig,
    SpgMitDriverConfig,
    load_can_device_config,
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
)
from robot_controller.config import (
    ConfigError,
    HardwareSafetyConfig,
    HardwareSafetyOptions,
    RobotControllerConfig,
    RobotControllerCoreConfig,
    RuntimeModeConfig,
    SafetyPolicyConfig,
    StateMachineConfig,
    load_robot_controller_config,
    resolve_config_arg,
    validate_app_config,
    validate_runtime_safety,
)
from robot_controller.config.loader import (
    load_yaml_mapping,
    resolve_config_path,
)
from robot_controller.shm.config import (
    MitCommandShmConfig,
    OperatorCommandShmConfig,
    RobotStateShmConfig,
    ShmConfig,
)
from robot_controller.supervisor.config import ProcessConfig

__all__ = [
    "CANDaemonConfig",
    "CanConfig",
    "CanDeviceConfig",
    "CanDeviceConfigError",
    "CanDeviceImuConfig",
    "ConfigError",
    "HardwareSafetyConfig",
    "HardwareSafetyOptions",
    "ImuConfig",
    "MitCommandShmConfig",
    "MitProtocolRangeConfig",
    "MotorConfig",
    "OperatorCommandShmConfig",
    "PlatformActuatorConfig",
    "PlatformCanConfig",
    "PlatformConfig",
    "PlatformConfigError",
    "PlatformRobotAssetConfig",
    "PlatformRobotConfig",
    "ProcessConfig",
    "RobotPlatformActuatorConfig",
    "RobotPlatformAssetConfig",
    "RobotPlatformCanConfig",
    "RobotPlatformConfig",
    "RobotPlatformConfigError",
    "RobotPlatformRobotConfig",
    "RobotControllerConfig",
    "RobotControllerCoreConfig",
    "RobotStateShmConfig",
    "RuntimeModeConfig",
    "SafetyPolicyConfig",
    "ShmConfig",
    "SpgMitDriverConfig",
    "StateMachineConfig",
    "load_can_device_config",
    "load_platform_config",
    "load_robot_platform_config",
    "load_robot_controller_config",
    "load_yaml_mapping",
    "resolve_config_path",
    "resolve_config_arg",
    "validate_app_config",
    "validate_runtime_safety",
]
