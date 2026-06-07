from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_controller.config.can import CanConfig, parse_can_config
from robot_controller.config.can_device import CanDeviceConfig, load_can_device_config
from robot_controller.config.loader import (
    ConfigError,
    load_yaml_mapping,
    require_bool,
    require_float,
    require_key,
    require_mapping,
)
from robot_controller.config.paths import ConfigPathRegistry, load_config_paths
from robot_controller.config.validation import validate_app_config
from robot_controller.platform.config import (
    RobotPlatformConfig,
    load_robot_platform_config,
)
from robot_controller.shm.config import ShmConfig, parse_shm_config
from robot_controller.process_supervisor.config import ProcessConfig, load_processes_config


@dataclass
class RobotControllerCoreConfig:
    name: str
    control_hz: float
    shutdown_timeout_s: float


@dataclass
class StateMachineConfig:
    enable_duration_s: float


@dataclass
class RuntimeModeConfig:
    mode: str


@dataclass
class HardwareSafetyConfig:
    allow_real_can: bool


@dataclass
class SafetyPolicyConfig:
    velocity_damping_kd: float
    damping_timeout_s: float
    command_loss_action: str
    feedback_stale_action: str


@dataclass
class RobotControllerConfig:
    robot_platform: RobotPlatformConfig
    can_device: CanDeviceConfig
    runtime: RuntimeModeConfig
    hardware: HardwareSafetyConfig
    safety: SafetyPolicyConfig
    state_machine: StateMachineConfig
    robot_controller: RobotControllerCoreConfig
    shm: ShmConfig
    can: CanConfig
    processes: list[ProcessConfig]

    @property
    def platform(self) -> RobotPlatformConfig:
        return self.robot_platform


def parse_runtime_config(raw: dict[str, Any]) -> RuntimeModeConfig:
    return RuntimeModeConfig(mode=str(require_key(raw, "mode", "runtime")))


def parse_hardware_safety_config(raw: dict[str, Any]) -> HardwareSafetyConfig:
    _reject_removed_keys(
        raw,
        {
            "require_manual_arm",
            "require_estop",
            "allow_enable_on_start",
            "allowed_can_interfaces",
        },
        "hardware",
    )
    return HardwareSafetyConfig(
        allow_real_can=require_bool(raw, "allow_real_can", "hardware"),
    )


def parse_safety_policy_config(raw: dict[str, Any]) -> SafetyPolicyConfig:
    return SafetyPolicyConfig(
        velocity_damping_kd=require_float(raw, "velocity_damping_kd", "safety"),
        damping_timeout_s=require_float(raw, "damping_timeout_s", "safety"),
        command_loss_action=str(require_key(raw, "command_loss_action", "safety")),
        feedback_stale_action=str(require_key(raw, "feedback_stale_action", "safety")),
    )


def parse_state_machine_config(raw: dict[str, Any]) -> StateMachineConfig:
    return StateMachineConfig(
        enable_duration_s=require_float(raw, "enable_duration_s", "state_machine"),
    )


def parse_robot_controller_core_config(raw: dict[str, Any]) -> RobotControllerCoreConfig:
    return RobotControllerCoreConfig(
        name=str(require_key(raw, "name", "robot_controller")),
        control_hz=require_float(raw, "control_hz", "robot_controller"),
        shutdown_timeout_s=require_float(raw, "shutdown_timeout_s", "robot_controller"),
    )


def load_robot_controller_config(
    path: str | Path | None = None,
    *,
    config_paths: ConfigPathRegistry | None = None,
) -> RobotControllerConfig:
    paths = load_config_paths() if config_paths is None else config_paths
    config_path = Path(path).resolve() if path is not None else paths.config("robot_controller")
    raw = load_yaml_mapping(config_path)
    _reject_removed_keys(raw, {"platform_config", "processes_config"}, "<root>")

    robot_platform = load_robot_platform_config(paths.config("robot_platform"))
    can_device = load_can_device_config(paths.config("can_device"))

    config = RobotControllerConfig(
        robot_platform=robot_platform,
        can_device=can_device,
        runtime=parse_runtime_config(require_mapping(raw, "runtime", "<root>")),
        hardware=parse_hardware_safety_config(require_mapping(raw, "hardware", "<root>")),
        safety=parse_safety_policy_config(require_mapping(raw, "safety", "<root>")),
        state_machine=parse_state_machine_config(require_mapping(raw, "state_machine", "<root>")),
        robot_controller=parse_robot_controller_core_config(
            require_mapping(raw, "robot_controller", "<root>")
        ),
        shm=parse_shm_config(
            require_mapping(raw, "shm", "<root>"),
            target_count=len(robot_platform.actuators),
        ),
        can=parse_can_config(require_mapping(raw, "can", "<root>"), robot_platform, can_device),
        processes=load_processes_config(paths.config("processes")),
    )
    validate_app_config(config)
    return config


def _reject_removed_keys(raw: dict[str, Any], removed_keys: set[str], path: str) -> None:
    for key in sorted(removed_keys):
        if key in raw:
            raise ConfigError(f"{path}.{key} is not supported by robot_controller.yaml")
