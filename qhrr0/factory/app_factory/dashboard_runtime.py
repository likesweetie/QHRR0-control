from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from qhrr0.factory.robot_factory.base.robot_base import Robot
from qhrr0.qhrr0_spec import QHRR0

from .app_validation_rules import (
    AppConfigError,
    reject_removed_dashboard_keys,
    reject_unknown_keys,
    require_bool,
    require_float,
    require_int,
    require_key,
    require_mapping,
    require_no_platform_owned_dashboard_keys,
    resolve_zero_set_presets,
    value,
)
from .loaders import config_path, load_yaml_mapping
from .schema import (
    CanMonitorConfig,
    DashboardActuatorConfig,
    DashboardCanDaemonStaticConfig,
    DashboardCanRuntimeConfig,
    DashboardControllerStateConfig,
    DashboardImuRuntimeConfig,
    DashboardPlatformRuntimeConfig,
    DashboardRuntimeConfig,
    DashboardSafetyRuntimeConfig,
    DashboardShmRuntimeConfig,
    DashboardSpgRuntimeConfig,
    DashboardStaticConfig,
    ProcessLaunchSpec,
    SpgMonitorConfig,
    ZeroSetPreset,
    ZeroSetTarget,
)


def build_dashboard_runtime(
    config: Mapping[str, Any],
    config_paths: Mapping[str, Mapping[str, Path]],
    *,
    processes: tuple[ProcessLaunchSpec, ...],
    runtime_config_path: str | Path,
    robot: Robot = QHRR0,
) -> DashboardRuntimeConfig:
    static = load_dashboard_static_config(config_path(config_paths, "dashboard"), robot=robot)
    return DashboardRuntimeConfig(
        static=static,
        platform=build_dashboard_platform_runtime(config, robot=robot),
        shutdown_timeout_s=float(value(config, "robot_controller.shutdown_timeout_s")),
        processes=tuple(processes),
        runtime_config_path=Path(runtime_config_path),
    )


def write_dashboard_runtime_config(runtime: DashboardRuntimeConfig) -> Path:
    path = runtime.runtime_config_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(runtime.to_payload(), indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return path


def load_dashboard_runtime_config(path: str | Path) -> DashboardRuntimeConfig:
    runtime_path = Path(path)
    try:
        payload = json.loads(runtime_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise AppConfigError(f"dashboard runtime config is not valid JSON: {runtime_path}") from exc
    if not isinstance(payload, dict):
        raise AppConfigError(f"dashboard runtime config must be a JSON object: {runtime_path}")
    reject_unknown_keys(
        payload,
        {"dashboard_config", "static_config", "platform_runtime", "shutdown_timeout_s", "processes"},
        str(runtime_path),
    )
    static = _parse_dashboard_static_config(
        require_mapping(payload, "static_config", str(runtime_path)),
        zero_set_presets_are_resolved=True,
    )
    platform = _parse_dashboard_platform_runtime(
        require_mapping(payload, "platform_runtime", str(runtime_path))
    )
    processes_raw = require_key(payload, "processes", str(runtime_path))
    if not isinstance(processes_raw, list):
        raise AppConfigError(f"Config key must be a list: {runtime_path}.processes")
    return DashboardRuntimeConfig(
        static=static,
        platform=platform,
        shutdown_timeout_s=float(require_key(payload, "shutdown_timeout_s", str(runtime_path))),
        processes=tuple(_parse_process_launch_spec(item, index) for index, item in enumerate(processes_raw)),
        runtime_config_path=runtime_path,
    )


def load_dashboard_static_config(path: str | Path, *, robot: Robot = QHRR0) -> DashboardStaticConfig:
    raw = load_yaml_mapping(path)
    require_no_platform_owned_dashboard_keys(raw)
    reject_removed_dashboard_keys(raw)
    return _parse_dashboard_static_config(raw, robot=robot)


def build_dashboard_platform_runtime(
    config: Mapping[str, Any],
    *,
    robot: Robot = QHRR0,
) -> DashboardPlatformRuntimeConfig:
    spg = value(config, "can_device.drivers.spg_mit")
    imu = value(config, "can_device.imu")
    return DashboardPlatformRuntimeConfig(
        can=DashboardCanRuntimeConfig(
            iface=str(value(config, "can.interface")),
            bitrate=int(value(config, "can.bitrate")),
            ipc_socket_path=str(value(config, "can.daemon.ipc_socket_path")),
        ),
        imu=DashboardImuRuntimeConfig(
            request_id=int(imu["request_id"]),
            quat_id=int(imu["quat_id"]),
            gyro_id=int(imu["gyro_id"]),
            quat_scale=float(imu["quat_scale"]),
            gyro_scale=float(imu["gyro_scale"]),
            normalize_quat=bool(imu["normalize_quat"]),
        ),
        spg=DashboardSpgRuntimeConfig(
            feedback_position_max_rad=float(spg["feedback_position_max_rad"]),
            iq_full_scale_count=float(spg["iq_full_scale_count"]),
            iq_full_scale_current_a=float(spg["iq_full_scale_current_a"]),
            p_max_rad=float(spg["p_max_rad"]),
            v_max_rad_s=float(spg["v_max_rad_s"]),
            kp_max=float(spg["kp_max"]),
            kd_max=float(spg["kd_max"]),
            tau_max_nm=float(spg["tau_max_nm"]),
        ),
        actuators=tuple(
            DashboardActuatorConfig(
                name=str(actuator.name),
                can_id=int(actuator.can_id),
            )
            for actuator in robot.actuators
        ),
        shm=DashboardShmRuntimeConfig(
            control_shm_name=str(value(config, "shm.control_state.name")),
            dashboard_shm_name=str(value(config, "shm.dashboard_state.name")),
            operator_shm_name=str(value(config, "shm.operator_command.name")),
            operator_shm_size_bytes=int(value(config, "shm.operator_command.size_bytes")),
        ),
        safety=DashboardSafetyRuntimeConfig(
            tx_enabled_by_default=False,
            allow_actuator_commands=False,
        ),
    )


def _parse_dashboard_static_config(
    raw: Mapping[str, Any],
    *,
    robot: Robot = QHRR0,
    zero_set_presets_are_resolved: bool = False,
) -> DashboardStaticConfig:
    raw_copy = {str(key): item for key, item in raw.items()}
    reject_unknown_keys(
        raw_copy,
        {
            "dashboard",
            "robot_controller_state",
            "can_monitor",
            "can_daemon",
            "spg_monitor",
            "zero_set_presets",
        },
        "dashboard",
    )
    if zero_set_presets_are_resolved:
        _reject_unknown_resolved_zero_set_keys(raw_copy)
    else:
        _reject_unknown_zero_set_keys(raw_copy)
        resolve_zero_set_presets(raw_copy, robot=robot)

    dashboard = require_mapping(raw_copy, "dashboard", "dashboard")
    reject_unknown_keys(dashboard, {"host", "port", "state_update_rate", "zero_set_presets"}, "dashboard.dashboard")
    controller_state = require_mapping(raw_copy, "robot_controller_state", "dashboard")
    reject_unknown_keys(controller_state, {"enabled", "stale_timeout_s"}, "dashboard.robot_controller_state")
    can_monitor = require_mapping(raw_copy, "can_monitor", "dashboard")
    reject_unknown_keys(
        can_monitor,
        {"bus_window_s", "heartbeat_window_s", "node_timeout_s", "stuff_factor"},
        "dashboard.can_monitor",
    )
    can_daemon = require_mapping(raw_copy, "can_daemon", "dashboard")
    reject_unknown_keys(can_daemon, {"connect_timeout_s"}, "dashboard.can_daemon")
    spg_monitor = require_mapping(raw_copy, "spg_monitor", "dashboard")
    reject_unknown_keys(spg_monitor, {"default_mit_poll_hz"}, "dashboard.spg_monitor")

    return DashboardStaticConfig(
        host=str(require_key(dashboard, "host", "dashboard.dashboard")),
        port=require_int(dashboard, "port", "dashboard.dashboard"),
        state_update_rate=require_float(dashboard, "state_update_rate", "dashboard.dashboard"),
        robot_controller_state=DashboardControllerStateConfig(
            enabled=require_bool(controller_state, "enabled", "dashboard.robot_controller_state"),
            stale_timeout_s=require_float(
                controller_state,
                "stale_timeout_s",
                "dashboard.robot_controller_state",
            ),
        ),
        can_monitor=CanMonitorConfig(
            bus_window_s=require_float(can_monitor, "bus_window_s", "dashboard.can_monitor"),
            heartbeat_window_s=require_float(can_monitor, "heartbeat_window_s", "dashboard.can_monitor"),
            node_timeout_s=require_float(can_monitor, "node_timeout_s", "dashboard.can_monitor"),
            stuff_factor=require_float(can_monitor, "stuff_factor", "dashboard.can_monitor"),
        ),
        can_daemon=DashboardCanDaemonStaticConfig(
            connect_timeout_s=require_float(can_daemon, "connect_timeout_s", "dashboard.can_daemon"),
        ),
        spg_monitor=SpgMonitorConfig(
            default_mit_poll_hz=require_float(
                spg_monitor,
                "default_mit_poll_hz",
                "dashboard.spg_monitor",
            ),
        ),
        zero_set_presets=_parse_zero_set_presets(raw_copy["zero_set_presets"]),
    )


def _parse_dashboard_platform_runtime(raw: Mapping[str, Any]) -> DashboardPlatformRuntimeConfig:
    reject_unknown_keys(raw, {"can", "imu", "spg", "actuators", "shm", "safety"}, "platform_runtime")
    can = require_mapping(raw, "can", "platform_runtime")
    reject_unknown_keys(can, {"iface", "bitrate", "ipc_socket_path"}, "platform_runtime.can")
    imu = require_mapping(raw, "imu", "platform_runtime")
    reject_unknown_keys(
        imu,
        {"request_id", "quat_id", "gyro_id", "quat_scale", "gyro_scale", "normalize_quat"},
        "platform_runtime.imu",
    )
    spg = require_mapping(raw, "spg", "platform_runtime")
    reject_unknown_keys(
        spg,
        {
            "feedback_position_max_rad",
            "iq_full_scale_count",
            "iq_full_scale_current_a",
            "p_max_rad",
            "v_max_rad_s",
            "kp_max",
            "kd_max",
            "tau_max_nm",
        },
        "platform_runtime.spg",
    )
    shm = require_mapping(raw, "shm", "platform_runtime")
    reject_unknown_keys(
        shm,
        {"control_shm_name", "dashboard_shm_name", "operator_shm_name", "operator_shm_size_bytes"},
        "platform_runtime.shm",
    )
    safety = require_mapping(raw, "safety", "platform_runtime")
    reject_unknown_keys(
        safety,
        {"tx_enabled_by_default", "allow_actuator_commands"},
        "platform_runtime.safety",
    )
    actuators_raw = require_key(raw, "actuators", "platform_runtime")
    if not isinstance(actuators_raw, list):
        raise AppConfigError("Config key must be a list: platform_runtime.actuators")
    return DashboardPlatformRuntimeConfig(
        can=DashboardCanRuntimeConfig(
            iface=str(require_key(can, "iface", "platform_runtime.can")),
            bitrate=require_int(can, "bitrate", "platform_runtime.can"),
            ipc_socket_path=str(require_key(can, "ipc_socket_path", "platform_runtime.can")),
        ),
        imu=DashboardImuRuntimeConfig(
            request_id=require_int(imu, "request_id", "platform_runtime.imu"),
            quat_id=require_int(imu, "quat_id", "platform_runtime.imu"),
            gyro_id=require_int(imu, "gyro_id", "platform_runtime.imu"),
            quat_scale=require_float(imu, "quat_scale", "platform_runtime.imu"),
            gyro_scale=require_float(imu, "gyro_scale", "platform_runtime.imu"),
            normalize_quat=require_bool(imu, "normalize_quat", "platform_runtime.imu"),
        ),
        spg=DashboardSpgRuntimeConfig(
            feedback_position_max_rad=require_float(
                spg,
                "feedback_position_max_rad",
                "platform_runtime.spg",
            ),
            iq_full_scale_count=require_float(spg, "iq_full_scale_count", "platform_runtime.spg"),
            iq_full_scale_current_a=require_float(
                spg,
                "iq_full_scale_current_a",
                "platform_runtime.spg",
            ),
            p_max_rad=require_float(spg, "p_max_rad", "platform_runtime.spg"),
            v_max_rad_s=require_float(spg, "v_max_rad_s", "platform_runtime.spg"),
            kp_max=require_float(spg, "kp_max", "platform_runtime.spg"),
            kd_max=require_float(spg, "kd_max", "platform_runtime.spg"),
            tau_max_nm=require_float(spg, "tau_max_nm", "platform_runtime.spg"),
        ),
        actuators=tuple(_parse_dashboard_actuator(item, index) for index, item in enumerate(actuators_raw)),
        shm=DashboardShmRuntimeConfig(
            control_shm_name=str(require_key(shm, "control_shm_name", "platform_runtime.shm")),
            dashboard_shm_name=str(require_key(shm, "dashboard_shm_name", "platform_runtime.shm")),
            operator_shm_name=str(require_key(shm, "operator_shm_name", "platform_runtime.shm")),
            operator_shm_size_bytes=require_int(
                shm,
                "operator_shm_size_bytes",
                "platform_runtime.shm",
            ),
        ),
        safety=DashboardSafetyRuntimeConfig(
            tx_enabled_by_default=require_bool(
                safety,
                "tx_enabled_by_default",
                "platform_runtime.safety",
            ),
            allow_actuator_commands=require_bool(
                safety,
                "allow_actuator_commands",
                "platform_runtime.safety",
            ),
        ),
    )


def _parse_zero_set_presets(raw: Any) -> tuple[ZeroSetPreset, ...]:
    if not isinstance(raw, list):
        raise AppConfigError("Dashboard config key 'zero_set_presets' must be a list")
    presets = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise AppConfigError(f"zero_set_presets[{index}] must be a mapping")
        targets_raw = item.get("targets")
        if not isinstance(targets_raw, list):
            raise AppConfigError(f"zero_set_presets[{index}].targets must be a list")
        presets.append(
            ZeroSetPreset(
                id=str(item["id"]),
                label=str(item["label"]),
                targets=tuple(_parse_zero_set_target(target, target_index) for target_index, target in enumerate(targets_raw)),
            )
        )
    return tuple(presets)


def _parse_zero_set_target(raw: Any, index: int) -> ZeroSetTarget:
    if not isinstance(raw, dict):
        raise AppConfigError(f"zero_set_presets target #{index} must be a mapping")
    return ZeroSetTarget(
        actuator=str(raw["actuator"]),
        can_id=int(raw["can_id"]),
        offset_deg=float(raw["offset_deg"]),
        offset_count=int(raw["offset_count"]),
    )


def _parse_dashboard_actuator(raw: Any, index: int) -> DashboardActuatorConfig:
    if not isinstance(raw, dict):
        raise AppConfigError(f"platform_runtime.actuators[{index}] must be a mapping")
    reject_unknown_keys(raw, {"name", "can_id"}, f"platform_runtime.actuators[{index}]")
    return DashboardActuatorConfig(
        name=str(require_key(raw, "name", f"platform_runtime.actuators[{index}]")),
        can_id=require_int(raw, "can_id", f"platform_runtime.actuators[{index}]"),
    )


def _parse_process_launch_spec(raw: Any, index: int) -> ProcessLaunchSpec:
    if not isinstance(raw, dict):
        raise AppConfigError(f"dashboard runtime processes[{index}] must be a mapping")
    reject_unknown_keys(
        raw,
        {
            "name",
            "command",
            "start_order",
            "stop_order",
            "new_terminal",
            "terminal_command",
            "working_dir",
            "env_vars",
        },
        f"dashboard_runtime.processes[{index}]",
    )
    command = require_key(raw, "command", f"dashboard_runtime.processes[{index}]")
    terminal_command = require_key(raw, "terminal_command", f"dashboard_runtime.processes[{index}]")
    if not isinstance(command, list):
        raise AppConfigError(f"Config key must be a list: dashboard_runtime.processes[{index}].command")
    if not isinstance(terminal_command, list):
        raise AppConfigError(
            f"Config key must be a list: dashboard_runtime.processes[{index}].terminal_command"
        )
    env_vars = require_mapping(raw, "env_vars", f"dashboard_runtime.processes[{index}]")
    return ProcessLaunchSpec(
        name=str(require_key(raw, "name", f"dashboard_runtime.processes[{index}]")),
        command=tuple(str(part) for part in command),
        start_order=require_int(raw, "start_order", f"dashboard_runtime.processes[{index}]"),
        stop_order=require_int(raw, "stop_order", f"dashboard_runtime.processes[{index}]"),
        new_terminal=require_bool(raw, "new_terminal", f"dashboard_runtime.processes[{index}]"),
        terminal_command=tuple(str(part) for part in terminal_command),
        working_dir=str(require_key(raw, "working_dir", f"dashboard_runtime.processes[{index}]")),
        env_vars={str(key): str(value) for key, value in env_vars.items()},
    )


def _reject_unknown_zero_set_keys(raw: Mapping[str, Any]) -> None:
    presets = raw.get("zero_set_presets")
    if presets is None:
        dashboard = raw.get("dashboard")
        if isinstance(dashboard, Mapping):
            presets = dashboard.get("zero_set_presets")
    if presets is None:
        return
    if not isinstance(presets, list):
        raise AppConfigError("Dashboard config key 'zero_set_presets' must be a list")
    for index, item in enumerate(presets):
        if not isinstance(item, Mapping):
            raise AppConfigError(f"zero_set_presets[{index}] must be a mapping")
        reject_unknown_keys(item, {"id", "label", "targets"}, f"zero_set_presets[{index}]")
        targets = item.get("targets")
        if not isinstance(targets, list):
            raise AppConfigError(f"zero_set_presets[{index}].targets must be a list")
        for target_index, target in enumerate(targets):
            if not isinstance(target, Mapping):
                raise AppConfigError(f"zero_set_presets[{index}].targets[{target_index}] must be a mapping")
            reject_unknown_keys(
                target,
                {"actuator", "offset_count", "offset_deg"},
                f"zero_set_presets[{index}].targets[{target_index}]",
            )


def _reject_unknown_resolved_zero_set_keys(raw: Mapping[str, Any]) -> None:
    presets = raw.get("zero_set_presets")
    if not isinstance(presets, list):
        raise AppConfigError("Dashboard static runtime key 'zero_set_presets' must be a list")
    for index, item in enumerate(presets):
        if not isinstance(item, Mapping):
            raise AppConfigError(f"zero_set_presets[{index}] must be a mapping")
        reject_unknown_keys(item, {"id", "label", "targets"}, f"zero_set_presets[{index}]")
        targets = item.get("targets")
        if not isinstance(targets, list):
            raise AppConfigError(f"zero_set_presets[{index}].targets must be a list")
        for target_index, target in enumerate(targets):
            if not isinstance(target, Mapping):
                raise AppConfigError(f"zero_set_presets[{index}].targets[{target_index}] must be a mapping")
            reject_unknown_keys(
                target,
                {"actuator", "can_id", "offset_count", "offset_deg"},
                f"zero_set_presets[{index}].targets[{target_index}]",
            )
