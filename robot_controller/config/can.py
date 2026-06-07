from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from robot_controller.config.loader import (
    ConfigError,
    optional_float_or_none,
    require_bool,
    require_float,
    require_int,
    require_mapping,
)
from robot_controller.platform.config import PlatformConfig


@dataclass
class MotorConfig:
    can_ids: list[int]
    enter_on_start: bool
    exit_on_shutdown: bool
    set_zero_on_start: bool


@dataclass
class ImuConfig:
    enabled: bool
    request_all_on_start: bool
    request_all_each_tick: bool
    startup_request_count: int
    startup_request_delay_s: float


@dataclass
class CANDaemonConfig:
    rx_timeout_s: float
    tx_timeout_s: float
    join_timeout_s: float
    max_tx_queue_size: int
    send_block: bool
    send_timeout_s: float | None
    ipc_socket_path: str
    connect_timeout_s: float


@dataclass
class MitProtocolRangeConfig:
    position_rad: float
    velocity_rad_s: float
    kp: float
    kd: float
    torque_ff_nm: float
    feedback_position_rad: float


@dataclass
class CanConfig:
    interface: str
    command_timeout_s: float
    bringup_delay_s: float
    daemon: CANDaemonConfig
    motors: MotorConfig
    imu: ImuConfig
    mit_protocol_range: MitProtocolRangeConfig


def parse_can_config(raw: dict[str, Any], platform: PlatformConfig) -> CanConfig:
    daemon_raw = require_mapping(raw, "daemon", "can")
    motors_raw = require_mapping(raw, "motors", "can")
    imu_raw = require_mapping(raw, "imu", "can")

    config = CanConfig(
        interface=platform.can.interface,
        command_timeout_s=require_float(raw, "command_timeout_s", "can"),
        bringup_delay_s=require_float(raw, "bringup_delay_s", "can"),
        daemon=CANDaemonConfig(
            rx_timeout_s=require_float(daemon_raw, "rx_timeout_s", "can.daemon"),
            tx_timeout_s=require_float(daemon_raw, "tx_timeout_s", "can.daemon"),
            join_timeout_s=require_float(daemon_raw, "join_timeout_s", "can.daemon"),
            max_tx_queue_size=require_int(daemon_raw, "max_tx_queue_size", "can.daemon"),
            send_block=require_bool(daemon_raw, "send_block", "can.daemon"),
            send_timeout_s=optional_float_or_none(daemon_raw, "send_timeout_s", "can.daemon"),
            ipc_socket_path=platform.can.daemon_socket,
            connect_timeout_s=require_float(daemon_raw, "connect_timeout_s", "can.daemon"),
        ),
        motors=MotorConfig(
            can_ids=[actuator.can_id for actuator in platform.enabled_actuators],
            enter_on_start=require_bool(motors_raw, "enter_on_start", "can.motors"),
            exit_on_shutdown=require_bool(motors_raw, "exit_on_shutdown", "can.motors"),
            set_zero_on_start=require_bool(motors_raw, "set_zero_on_start", "can.motors"),
        ),
        imu=ImuConfig(
            enabled=require_bool(imu_raw, "enabled", "can.imu"),
            request_all_on_start=require_bool(imu_raw, "request_all_on_start", "can.imu"),
            request_all_each_tick=require_bool(imu_raw, "request_all_each_tick", "can.imu"),
            startup_request_count=require_int(imu_raw, "startup_request_count", "can.imu"),
            startup_request_delay_s=require_float(imu_raw, "startup_request_delay_s", "can.imu"),
        ),
        mit_protocol_range=MitProtocolRangeConfig(
            position_rad=float(platform.spg_mit.p_max_rad),
            velocity_rad_s=float(platform.spg_mit.v_max_rad_s),
            kp=float(platform.spg_mit.kp_max),
            kd=float(platform.spg_mit.kd_max),
            torque_ff_nm=float(platform.spg_mit.tau_max_nm),
            feedback_position_rad=float(platform.spg_mit.feedback_position_max_rad),
        ),
    )
    validate_can_config(config)
    return config


def validate_can_config(config: CanConfig) -> None:
    if config.command_timeout_s <= 0.0:
        raise ConfigError("can.command_timeout_s must be > 0")
    if config.bringup_delay_s < 0.0:
        raise ConfigError("can.bringup_delay_s must be >= 0")
    if config.daemon.rx_timeout_s < 0.0:
        raise ConfigError("can.daemon.rx_timeout_s must be >= 0")
    if config.daemon.tx_timeout_s < 0.0:
        raise ConfigError("can.daemon.tx_timeout_s must be >= 0")
    if config.daemon.join_timeout_s < 0.0:
        raise ConfigError("can.daemon.join_timeout_s must be >= 0")
    if config.daemon.max_tx_queue_size <= 0:
        raise ConfigError("can.daemon.max_tx_queue_size must be > 0")
    if config.daemon.send_timeout_s is not None and config.daemon.send_timeout_s < 0.0:
        raise ConfigError("can.daemon.send_timeout_s must be null or >= 0")
    if not config.daemon.ipc_socket_path:
        raise ConfigError("can.daemon.ipc_socket_path must not be empty")
    if config.daemon.connect_timeout_s <= 0.0:
        raise ConfigError("can.daemon.connect_timeout_s must be > 0")
    if not config.motors.can_ids:
        raise ConfigError("can.motors.can_ids must not be empty")
    if len(set(config.motors.can_ids)) != len(config.motors.can_ids):
        raise ConfigError("can.motors.can_ids must not contain duplicates")
    if config.imu.startup_request_count < 0:
        raise ConfigError("can.imu.startup_request_count must be >= 0")
    if config.imu.startup_request_delay_s < 0.0:
        raise ConfigError("can.imu.startup_request_delay_s must be >= 0")
    if config.mit_protocol_range.position_rad <= 0.0:
        raise ConfigError("can.mit_protocol_range.position_rad must be > 0")
    if config.mit_protocol_range.velocity_rad_s <= 0.0:
        raise ConfigError("can.mit_protocol_range.velocity_rad_s must be > 0")
    if config.mit_protocol_range.kp < 0.0:
        raise ConfigError("can.mit_protocol_range.kp must be >= 0")
    if config.mit_protocol_range.kd < 0.5:
        raise ConfigError("can.mit_protocol_range.kd must be >= 0.5 for shutdown damping")
    if config.mit_protocol_range.torque_ff_nm <= 0.0:
        raise ConfigError("can.mit_protocol_range.torque_ff_nm must be > 0")
    if config.mit_protocol_range.feedback_position_rad <= 0.0:
        raise ConfigError("can.mit_protocol_range.feedback_position_rad must be > 0")
