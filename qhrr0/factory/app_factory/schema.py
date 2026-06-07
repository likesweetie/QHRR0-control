from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from qhrr0.app.hal.driver.actuators import SPGMITConfig


@dataclass(frozen=True, slots=True)
class ProcessLaunchSpec:
    name: str
    command: tuple[str, ...]
    start_order: int
    stop_order: int
    new_terminal: bool
    terminal_command: tuple[str, ...]
    working_dir: str
    env_vars: dict[str, str]

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "command": list(self.command),
            "start_order": self.start_order,
            "stop_order": self.stop_order,
            "new_terminal": self.new_terminal,
            "terminal_command": list(self.terminal_command),
            "working_dir": self.working_dir,
            "env_vars": dict(self.env_vars),
        }


@dataclass(frozen=True, slots=True)
class RobotControllerActuator:
    name: str
    driver: str
    can_id: int


@dataclass(frozen=True, slots=True)
class CanClientRuntimeConfig:
    ipc_socket_path: str
    connect_timeout_s: float
    command_timeout_s: float


@dataclass(frozen=True, slots=True)
class ImuRuntimeConfig:
    name: str
    request_id: int
    quat_id: int
    gyro_id: int
    cmd_get_quat: int
    cmd_get_gyro: int
    cmd_get_all: int
    quat_scale: float
    gyro_scale: float
    normalize_quat: bool
    enabled: bool
    request_all_on_start: bool
    request_all_each_tick: bool
    startup_request_count: int
    startup_request_delay_s: float


@dataclass(frozen=True, slots=True)
class ShmRuntimeConfig:
    mit_command_name: str
    aux_command_name: str
    aux_command_size_bytes: int
    operator_command_name: str
    operator_command_size_bytes: int
    control_state_name: str
    control_state_size_bytes: int
    control_state_publish_hz: float
    dashboard_state_name: str
    dashboard_state_size_bytes: int
    dashboard_state_publish_hz: float


@dataclass(frozen=True, slots=True)
class ControllerSafetyConfig:
    velocity_damping_kd: float


@dataclass(frozen=True, slots=True)
class ControllerTimingConfig:
    control_hz: float
    shutdown_timeout_s: float
    enable_duration_s: float


@dataclass(frozen=True, slots=True)
class RobotControllerRuntimeConfig:
    actuators: tuple[RobotControllerActuator, ...]
    mit: SPGMITConfig
    iq_count_to_amp: float | None
    can: CanClientRuntimeConfig
    imu: ImuRuntimeConfig
    shm: ShmRuntimeConfig
    safety: ControllerSafetyConfig
    timing: ControllerTimingConfig


@dataclass(frozen=True, slots=True)
class CanMonitorConfig:
    bus_window_s: float
    heartbeat_window_s: float
    node_timeout_s: float
    stuff_factor: float


@dataclass(frozen=True, slots=True)
class SpgMonitorConfig:
    default_mit_poll_hz: float


@dataclass(frozen=True, slots=True)
class DashboardControllerStateConfig:
    enabled: bool
    stale_timeout_s: float


@dataclass(frozen=True, slots=True)
class DashboardCanDaemonStaticConfig:
    connect_timeout_s: float


@dataclass(frozen=True, slots=True)
class ZeroSetTarget:
    actuator: str
    can_id: int
    offset_deg: float
    offset_count: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "actuator": self.actuator,
            "can_id": self.can_id,
            "offset_deg": self.offset_deg,
            "offset_count": self.offset_count,
        }


@dataclass(frozen=True, slots=True)
class ZeroSetPreset:
    id: str
    label: str
    targets: tuple[ZeroSetTarget, ...]

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "targets": [target.as_dict() for target in self.targets],
        }


@dataclass(frozen=True, slots=True)
class DashboardStaticConfig:
    host: str
    port: int
    state_update_rate: float
    robot_controller_state: DashboardControllerStateConfig
    can_monitor: CanMonitorConfig
    can_daemon: DashboardCanDaemonStaticConfig
    spg_monitor: SpgMonitorConfig
    zero_set_presets: tuple[ZeroSetPreset, ...]

    def as_dict(self) -> dict[str, Any]:
        return {
            "dashboard": {
                "host": self.host,
                "port": self.port,
                "state_update_rate": self.state_update_rate,
                "zero_set_presets": [preset.as_dict() for preset in self.zero_set_presets],
            },
            "robot_controller_state": {
                "enabled": self.robot_controller_state.enabled,
                "stale_timeout_s": self.robot_controller_state.stale_timeout_s,
            },
            "can_monitor": {
                "bus_window_s": self.can_monitor.bus_window_s,
                "heartbeat_window_s": self.can_monitor.heartbeat_window_s,
                "node_timeout_s": self.can_monitor.node_timeout_s,
                "stuff_factor": self.can_monitor.stuff_factor,
            },
            "can_daemon": {
                "connect_timeout_s": self.can_daemon.connect_timeout_s,
            },
            "spg_monitor": {
                "default_mit_poll_hz": self.spg_monitor.default_mit_poll_hz,
            },
            "zero_set_presets": [preset.as_dict() for preset in self.zero_set_presets],
        }


@dataclass(frozen=True, slots=True)
class DashboardCanRuntimeConfig:
    iface: str
    bitrate: int
    ipc_socket_path: str


@dataclass(frozen=True, slots=True)
class DashboardImuRuntimeConfig:
    request_id: int
    quat_id: int
    gyro_id: int
    quat_scale: float
    gyro_scale: float
    normalize_quat: bool


@dataclass(frozen=True, slots=True)
class DashboardSpgRuntimeConfig:
    feedback_position_max_rad: float
    iq_full_scale_count: float
    iq_full_scale_current_a: float
    p_max_rad: float
    v_max_rad_s: float
    kp_max: float
    kd_max: float
    tau_max_nm: float


@dataclass(frozen=True, slots=True)
class DashboardActuatorConfig:
    name: str
    can_id: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "can_id": self.can_id,
        }


@dataclass(frozen=True, slots=True)
class DashboardShmRuntimeConfig:
    control_shm_name: str
    dashboard_shm_name: str
    operator_shm_name: str
    operator_shm_size_bytes: int


@dataclass(frozen=True, slots=True)
class DashboardSafetyRuntimeConfig:
    tx_enabled_by_default: bool
    allow_actuator_commands: bool


@dataclass(frozen=True, slots=True)
class DashboardPlatformRuntimeConfig:
    can: DashboardCanRuntimeConfig
    imu: DashboardImuRuntimeConfig
    spg: DashboardSpgRuntimeConfig
    actuators: tuple[DashboardActuatorConfig, ...]
    shm: DashboardShmRuntimeConfig
    safety: DashboardSafetyRuntimeConfig

    def as_dict(self) -> dict[str, Any]:
        return {
            "can": {
                "iface": self.can.iface,
                "bitrate": self.can.bitrate,
                "ipc_socket_path": self.can.ipc_socket_path,
            },
            "imu": {
                "request_id": self.imu.request_id,
                "quat_id": self.imu.quat_id,
                "gyro_id": self.imu.gyro_id,
                "quat_scale": self.imu.quat_scale,
                "gyro_scale": self.imu.gyro_scale,
                "normalize_quat": self.imu.normalize_quat,
            },
            "spg": {
                "feedback_position_max_rad": self.spg.feedback_position_max_rad,
                "iq_full_scale_count": self.spg.iq_full_scale_count,
                "iq_full_scale_current_a": self.spg.iq_full_scale_current_a,
                "p_max_rad": self.spg.p_max_rad,
                "v_max_rad_s": self.spg.v_max_rad_s,
                "kp_max": self.spg.kp_max,
                "kd_max": self.spg.kd_max,
                "tau_max_nm": self.spg.tau_max_nm,
            },
            "actuators": [actuator.as_dict() for actuator in self.actuators],
            "shm": {
                "control_shm_name": self.shm.control_shm_name,
                "dashboard_shm_name": self.shm.dashboard_shm_name,
                "operator_shm_name": self.shm.operator_shm_name,
                "operator_shm_size_bytes": self.shm.operator_shm_size_bytes,
            },
            "safety": {
                "tx_enabled_by_default": self.safety.tx_enabled_by_default,
                "allow_actuator_commands": self.safety.allow_actuator_commands,
            },
        }


@dataclass(frozen=True, slots=True)
class DashboardRuntimeConfig:
    static: DashboardStaticConfig
    platform: DashboardPlatformRuntimeConfig
    shutdown_timeout_s: float
    processes: tuple[ProcessLaunchSpec, ...]
    runtime_config_path: Path

    def effective_config(self) -> dict[str, Any]:
        static = self.static
        platform = self.platform
        return {
            "dashboard": {
                "host": static.host,
                "port": static.port,
                "state_update_rate": static.state_update_rate,
                "zero_set_presets": [preset.as_dict() for preset in static.zero_set_presets],
            },
            "robot_controller_state": {
                "enabled": static.robot_controller_state.enabled,
                "stale_timeout_s": static.robot_controller_state.stale_timeout_s,
                "control_shm_name": platform.shm.control_shm_name,
                "dashboard_shm_name": platform.shm.dashboard_shm_name,
                "operator_shm_name": platform.shm.operator_shm_name,
                "operator_shm_size_bytes": platform.shm.operator_shm_size_bytes,
            },
            "can": {
                "iface": platform.can.iface,
                "bitrate": platform.can.bitrate,
                "bus_window_s": static.can_monitor.bus_window_s,
                "heartbeat_window_s": static.can_monitor.heartbeat_window_s,
                "node_timeout_s": static.can_monitor.node_timeout_s,
                "stuff_factor": static.can_monitor.stuff_factor,
            },
            "can_daemon": {
                "ipc_socket_path": platform.can.ipc_socket_path,
                "connect_timeout_s": static.can_daemon.connect_timeout_s,
            },
            "imu": {
                "request_id": platform.imu.request_id,
                "quat_id": platform.imu.quat_id,
                "gyro_id": platform.imu.gyro_id,
                "quat_scale": platform.imu.quat_scale,
                "gyro_scale": platform.imu.gyro_scale,
                "normalize_quat": platform.imu.normalize_quat,
            },
            "spg": {
                "default_mit_poll_hz": static.spg_monitor.default_mit_poll_hz,
                "feedback_position_max_rad": platform.spg.feedback_position_max_rad,
                "iq_full_scale_count": platform.spg.iq_full_scale_count,
                "iq_full_scale_current_a": platform.spg.iq_full_scale_current_a,
                "p_max_rad": platform.spg.p_max_rad,
                "v_max_rad_s": platform.spg.v_max_rad_s,
                "kp_max": platform.spg.kp_max,
                "kd_max": platform.spg.kd_max,
                "tau_max_nm": platform.spg.tau_max_nm,
            },
            "safety": {
                "tx_enabled_by_default": platform.safety.tx_enabled_by_default,
                "allow_actuator_commands": platform.safety.allow_actuator_commands,
            },
            "actuators": [actuator.as_dict() for actuator in platform.actuators],
            "zero_set_presets": [preset.as_dict() for preset in static.zero_set_presets],
        }

    def to_payload(self) -> dict[str, Any]:
        return {
            "dashboard_config": self.effective_config(),
            "static_config": self.static.as_dict(),
            "platform_runtime": self.platform.as_dict(),
            "shutdown_timeout_s": self.shutdown_timeout_s,
            "processes": [process.as_dict() for process in self.processes],
        }


@dataclass(frozen=True, slots=True)
class AppRuntimeSpec:
    controller: RobotControllerRuntimeConfig
    processes: tuple[ProcessLaunchSpec, ...]
    dashboard: DashboardRuntimeConfig | None
