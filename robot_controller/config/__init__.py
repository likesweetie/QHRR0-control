from __future__ import annotations

__all__ = [
    "ConfigError",
    "CanDeviceConfig",
    "CanDeviceConfigError",
    "CanDeviceImuConfig",
    "HardwareSafetyConfig",
    "HardwareSafetyOptions",
    "ConfigPathRegistry",
    "DEFAULT_CONFIG_PATHS",
    "RobotControllerConfig",
    "RobotControllerCoreConfig",
    "RuntimeModeConfig",
    "SafetyPolicyConfig",
    "SpgMitDriverConfig",
    "StateMachineConfig",
    "load_can_device_config",
    "load_config_paths",
    "load_robot_controller_config",
    "resolve_config_arg",
    "validate_app_config",
    "validate_runtime_safety",
]


def __getattr__(name: str):
    if name == "ConfigError":
        from .loader import ConfigError

        return ConfigError
    if name in {
        "CanDeviceConfig",
        "CanDeviceConfigError",
        "CanDeviceImuConfig",
        "SpgMitDriverConfig",
        "load_can_device_config",
    }:
        from . import can_device

        return getattr(can_device, name)
    if name in {
        "ConfigPathRegistry",
        "DEFAULT_CONFIG_PATHS",
        "load_config_paths",
    }:
        from . import paths

        return getattr(paths, name)
    if name == "resolve_config_arg":
        from .cli import resolve_config_arg

        return resolve_config_arg
    if name in {
        "HardwareSafetyConfig",
        "RobotControllerConfig",
        "RobotControllerCoreConfig",
        "RuntimeModeConfig",
        "SafetyPolicyConfig",
        "StateMachineConfig",
        "load_robot_controller_config",
    }:
        from . import app

        return getattr(app, name)
    if name in {
        "HardwareSafetyOptions",
        "validate_app_config",
        "validate_runtime_safety",
    }:
        from . import validation

        return getattr(validation, name)
    raise AttributeError(name)
