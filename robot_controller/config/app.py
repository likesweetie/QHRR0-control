from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_controller.config.can import CanConfig, parse_can_config
from robot_controller.config.loader import (
    load_yaml_mapping,
    require_bool,
    require_float,
    require_key,
    require_list,
    require_mapping,
    resolve_config_path,
)
from robot_controller.config.validation import validate_app_config
from robot_controller.platform.config import PlatformConfig, load_platform_config
from robot_controller.shm.config import ShmConfig, parse_shm_config
from robot_controller.supervisor.config import ProcessConfig, load_processes_config


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
    require_manual_arm: bool
    require_estop: bool
    allow_enable_on_start: bool
    allowed_can_interfaces: list[str]


@dataclass
class SafetyPolicyConfig:
    velocity_damping_kd: float
    damping_timeout_s: float
    command_loss_action: str
    feedback_stale_action: str


@dataclass
class RobotControllerConfig:
    platform: PlatformConfig
    runtime: RuntimeModeConfig
    hardware: HardwareSafetyConfig
    safety: SafetyPolicyConfig
    state_machine: StateMachineConfig
    robot_controller: RobotControllerCoreConfig
    shm: ShmConfig
    can: CanConfig
    processes: list[ProcessConfig]


def parse_runtime_config(raw: dict[str, Any]) -> RuntimeModeConfig:
    return RuntimeModeConfig(mode=str(require_key(raw, "mode", "runtime")))


def parse_hardware_safety_config(raw: dict[str, Any]) -> HardwareSafetyConfig:
    return HardwareSafetyConfig(
        allow_real_can=require_bool(raw, "allow_real_can", "hardware"),
        require_manual_arm=require_bool(raw, "require_manual_arm", "hardware"),
        require_estop=require_bool(raw, "require_estop", "hardware"),
        allow_enable_on_start=require_bool(raw, "allow_enable_on_start", "hardware"),
        allowed_can_interfaces=[
            str(item)
            for item in require_list(raw, "allowed_can_interfaces", "hardware")
        ],
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


def load_robot_controller_config(path: str | Path) -> RobotControllerConfig:
    config_path = Path(path)
    raw = load_yaml_mapping(config_path)

    platform_config_path = resolve_config_path(
        config_path,
        str(require_key(raw, "platform_config", "<root>")),
        "platform_config",
    )
    platform = load_platform_config(platform_config_path)

    processes_config_path = resolve_config_path(
        config_path,
        str(require_key(raw, "processes_config", "<root>")),
        "processes_config",
    )

    config = RobotControllerConfig(
        platform=platform,
        runtime=parse_runtime_config(require_mapping(raw, "runtime", "<root>")),
        hardware=parse_hardware_safety_config(require_mapping(raw, "hardware", "<root>")),
        safety=parse_safety_policy_config(require_mapping(raw, "safety", "<root>")),
        state_machine=parse_state_machine_config(require_mapping(raw, "state_machine", "<root>")),
        robot_controller=parse_robot_controller_core_config(
            require_mapping(raw, "robot_controller", "<root>")
        ),
        shm=parse_shm_config(require_mapping(raw, "shm", "<root>"), platform),
        can=parse_can_config(require_mapping(raw, "can", "<root>"), platform),
        processes=load_processes_config(processes_config_path),
    )
    validate_app_config(config)
    return config
