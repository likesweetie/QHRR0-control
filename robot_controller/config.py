from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class ProcessConfig:
    name: str
    command: list[str]
    required: bool = True
    start_order: int = 0
    stop_order: int = 0
    restart: bool = False
    enabled: bool = True


@dataclass
class MitCommandShmConfig:
    name: str = "qhrr_mit_command"
    motor_count: int = 12


@dataclass
class RobotStateShmConfig:
    name: str = "qhrr_robot_state"
    enabled: bool = False


@dataclass
class ShmConfig:
    cleanup_stale_on_start: bool = True
    unlink_on_shutdown: bool = True
    mit_command: MitCommandShmConfig = field(default_factory=MitCommandShmConfig)
    robot_state: RobotStateShmConfig = field(default_factory=RobotStateShmConfig)


@dataclass
class MotorConfig:
    ids: list[int] = field(default_factory=lambda: [0x141, 0x142, 0x143])
    enter_on_start: bool = True
    exit_on_shutdown: bool = True
    set_zero_on_start: bool = False


@dataclass
class MitLimitsConfig:
    position_rad: float = 12.5
    velocity_rad_s: float = 45.0
    kp: float = 500.0
    kd: float = 5.0
    torque_ff_nm: float = 33.0


@dataclass
class CanConfig:
    interface: str = "vcan0"
    daemon_process: str = ""
    command_source: str = "shm"
    command_timeout_s: float = 0.05
    direct_socketcan: bool = True
    bringup_delay_s: float = 0.01
    motors: MotorConfig = field(default_factory=MotorConfig)
    mit_limits: MitLimitsConfig = field(default_factory=MitLimitsConfig)


@dataclass
class RobotControllerCoreConfig:
    name: str = "qhrr_robot_controller"
    control_hz: float = 500.0
    startup_timeout_s: float = 5.0
    shutdown_timeout_s: float = 2.0


@dataclass
class RobotControllerConfig:
    robot_controller: RobotControllerCoreConfig = field(default_factory=RobotControllerCoreConfig)
    shm: ShmConfig = field(default_factory=ShmConfig)
    can: CanConfig = field(default_factory=CanConfig)
    processes: list[ProcessConfig] = field(default_factory=list)


def _as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _int_list(value: Any, default: list[int]) -> list[int]:
    if not isinstance(value, list):
        return list(default)
    return [int(item, 0) if isinstance(item, str) else int(item) for item in value]


def _process_config(item: dict[str, Any]) -> ProcessConfig:
    command = item.get("command", [])
    if isinstance(command, str):
        command = command.split()
    return ProcessConfig(
        name=str(item["name"]),
        command=[str(part) for part in command],
        required=bool(item.get("required", True)),
        start_order=int(item.get("start_order", 0)),
        stop_order=int(item.get("stop_order", 0)),
        restart=bool(item.get("restart", False)),
        enabled=bool(item.get("enabled", True)),
    )


def load_robot_controller_config(path: str | Path) -> RobotControllerConfig:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as fp:
        raw = yaml.safe_load(fp) or {}

    core_raw = _as_dict(raw.get("robot_controller"))
    shm_raw = _as_dict(raw.get("shm"))
    mit_raw = _as_dict(shm_raw.get("mit_command"))
    state_raw = _as_dict(shm_raw.get("robot_state"))
    can_raw = _as_dict(raw.get("can"))
    motors_raw = _as_dict(can_raw.get("motors"))
    limits_raw = _as_dict(can_raw.get("mit_limits"))

    default_motors = MotorConfig()
    processes = [
        _process_config(item)
        for item in _as_list(raw.get("processes"))
        if isinstance(item, dict) and item.get("name")
    ]

    return RobotControllerConfig(
        robot_controller=RobotControllerCoreConfig(
            name=str(core_raw.get("name", "qhrr_robot_controller")),
            control_hz=float(core_raw.get("control_hz", 500.0)),
            startup_timeout_s=float(core_raw.get("startup_timeout_s", 5.0)),
            shutdown_timeout_s=float(core_raw.get("shutdown_timeout_s", 2.0)),
        ),
        shm=ShmConfig(
            cleanup_stale_on_start=bool(shm_raw.get("cleanup_stale_on_start", True)),
            unlink_on_shutdown=bool(shm_raw.get("unlink_on_shutdown", True)),
            mit_command=MitCommandShmConfig(
                name=str(mit_raw.get("name", "qhrr_mit_command")),
                motor_count=int(mit_raw.get("motor_count", 12)),
            ),
            robot_state=RobotStateShmConfig(
                name=str(state_raw.get("name", "qhrr_robot_state")),
                enabled=bool(state_raw.get("enabled", False)),
            ),
        ),
        can=CanConfig(
            interface=str(can_raw.get("interface", "vcan0")),
            daemon_process=str(can_raw.get("daemon_process", "")),
            command_source=str(can_raw.get("command_source", "shm")),
            command_timeout_s=float(can_raw.get("command_timeout_s", 0.05)),
            direct_socketcan=bool(can_raw.get("direct_socketcan", True)),
            bringup_delay_s=float(can_raw.get("bringup_delay_s", 0.01)),
            motors=MotorConfig(
                ids=_int_list(motors_raw.get("ids"), default_motors.ids),
                enter_on_start=bool(motors_raw.get("enter_on_start", True)),
                exit_on_shutdown=bool(motors_raw.get("exit_on_shutdown", True)),
                set_zero_on_start=bool(motors_raw.get("set_zero_on_start", False)),
            ),
            mit_limits=MitLimitsConfig(
                position_rad=float(limits_raw.get("position_rad", 12.5)),
                velocity_rad_s=float(limits_raw.get("velocity_rad_s", 45.0)),
                kp=float(limits_raw.get("kp", 500.0)),
                kd=float(limits_raw.get("kd", 5.0)),
                torque_ff_nm=float(limits_raw.get("torque_ff_nm", 33.0)),
            ),
        ),
        processes=processes,
    )
