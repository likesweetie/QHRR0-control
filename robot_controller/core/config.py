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
    "PlatformImuConfig",
    "PlatformRobotAssetConfig",
    "PlatformRobotConfig",
    "PlatformShmConfig",
    "PlatformSpgMitConfig",
    "ProcessConfig",
    "RobotControllerConfig",
    "RobotControllerCoreConfig",
    "RobotStateShmConfig",
    "RuntimeModeConfig",
    "SafetyPolicyConfig",
    "ShmConfig",
    "StateMachineConfig",
    "load_platform_config",
    "load_robot_controller_config",
    "load_yaml_mapping",
    "resolve_config_path",
    "validate_app_config",
    "validate_runtime_safety",
]
