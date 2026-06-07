from __future__ import annotations

__all__ = [
    "ConfigError",
    "HardwareSafetyConfig",
    "HardwareSafetyOptions",
    "RobotControllerConfig",
    "RobotControllerCoreConfig",
    "RuntimeModeConfig",
    "SafetyPolicyConfig",
    "StateMachineConfig",
    "load_robot_controller_config",
    "validate_app_config",
    "validate_runtime_safety",
]


def __getattr__(name: str):
    if name == "ConfigError":
        from .loader import ConfigError

        return ConfigError
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
