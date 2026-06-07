from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml

from qhrr0.factory.robot_factory.base.robot_base import Robot
from qhrr0.qhrr0_spec import QHRR0

from .app_validation_rules import (
    AppConfigError,
    non_empty_string,
    optional_float_or_none,
    parse_int,
    reject_removed_keys,
    reject_unknown_keys,
    require_bool,
    require_float,
    require_int,
    require_key,
    require_list,
    require_mapping,
    require_non_empty_string,
    require_robot_matches_config,
    require_shm_name,
    require_spg_mit_driver,
    validate_actuator_drivers,
    validate_app_config,
    validate_can_config,
    validate_can_device_config,
    validate_imu_device_config,
    validate_process_config,
    validate_processes_config,
    validate_shm_config,
    validate_spg_mit_driver_config,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATHS = PACKAGE_ROOT / "config" / "config_paths.yaml"
DEFAULT_CONTROLLER_CONFIG = PACKAGE_ROOT / "config" / "app_config" / "robot_controller.yaml"


def load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as fp:
        raw = yaml.safe_load(fp)
    if not isinstance(raw, dict):
        raise AppConfigError(f"{config_path} must contain a YAML mapping")
    return raw


def load_config_paths(path: str | Path = DEFAULT_CONFIG_PATHS) -> dict[str, dict[str, Path]]:
    config_path_value = Path(path)
    raw = load_yaml_mapping(config_path_value)
    reject_unknown_keys(raw, {"app", "robot", "policy"}, str(config_path_value))
    base = _project_root_from_config_paths(config_path_value)
    app_paths = _parse_path_mapping(require_mapping(raw, "app", str(config_path_value)), base)
    robot_paths = _parse_path_mapping(require_mapping(raw, "robot", str(config_path_value)), base)
    duplicates = set(app_paths) & set(robot_paths)
    if duplicates:
        raise AppConfigError(f"duplicate config path keys: {sorted(duplicates)}")
    return {
        "configs": {
            **app_paths,
            **robot_paths,
        },
        "policy": _parse_path_mapping(require_mapping(raw, "policy", str(config_path_value)), base),
    }


def config_path(paths: Mapping[str, Mapping[str, Path]], key: str) -> Path:
    try:
        return paths["configs"][key]
    except KeyError as exc:
        raise AppConfigError(f"unknown config path key: configs.{key}") from exc


def policy_path(paths: Mapping[str, Mapping[str, Path]], key: str) -> Path:
    try:
        return paths["policy"][key]
    except KeyError as exc:
        raise AppConfigError(f"unknown config path key: policy.{key}") from exc


def load_robot_controller_config(
    path: str | Path | None = None,
    *,
    config_paths: Mapping[str, Mapping[str, Path]] | None = None,
    robot: Robot = QHRR0,
) -> dict[str, Any]:
    paths = load_config_paths() if config_paths is None else config_paths
    controller_path = Path(path).resolve() if path is not None else config_path(paths, "robot_controller")
    raw = load_yaml_mapping(controller_path)
    reject_removed_keys(raw, {"platform_config", "processes_config"}, "<root>")
    reject_unknown_keys(
        raw,
        {
            "runtime",
            "hardware",
            "can",
            "safety",
            "state_machine",
            "robot_controller",
            "shm",
        },
        "<root>",
    )

    can_device = load_can_device_config(config_path(paths, "can_device"))
    can = parse_can_config(require_mapping(raw, "can", "<root>"), can_device, robot=robot)
    config = {
        "robot": {
            "name": str(robot.name),
            "can_interfaces": tuple(str(item) for item in robot.can_interfaces),
            "actuators": tuple(
                {
                    "name": str(actuator.name),
                    "driver": str(actuator.driver),
                    "can_id": int(actuator.can_id),
                }
                for actuator in robot.actuators
            ),
        },
        "can_device": can_device,
        "runtime": parse_runtime_section(require_mapping(raw, "runtime", "<root>")),
        "hardware": parse_hardware_safety_config(require_mapping(raw, "hardware", "<root>")),
        "safety": parse_safety_policy_config(require_mapping(raw, "safety", "<root>")),
        "state_machine": parse_state_machine_config(require_mapping(raw, "state_machine", "<root>")),
        "robot_controller": parse_robot_controller_core_config(
            require_mapping(raw, "robot_controller", "<root>")
        ),
        "shm": parse_shm_config(
            require_mapping(raw, "shm", "<root>"),
            target_count=len(robot.actuators),
        ),
        "can": can,
        "processes": load_processes_config(config_path(paths, "processes")),
    }
    validate_app_config(config)
    require_robot_matches_config(robot, config)
    return config


def load_can_device_config(path: str | Path) -> dict[str, Any]:
    config_path_value = Path(path).resolve()
    raw = load_yaml_mapping(config_path_value)
    reject_unknown_keys(raw, {"imu", "drivers"}, "<root>")
    config = {
        "path": config_path_value,
        "imu": parse_imu_device_config(require_mapping(raw, "imu", "<root>")),
        "drivers": parse_driver_configs(require_mapping(raw, "drivers", "<root>")),
    }
    validate_can_device_config(config)
    return config


def parse_runtime_section(raw: Mapping[str, Any]) -> dict[str, Any]:
    reject_unknown_keys(raw, {"mode"}, "runtime")
    return {
        "mode": str(require_key(raw, "mode", "runtime")),
    }


def parse_hardware_safety_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    reject_unknown_keys(raw, {"allow_real_can"}, "hardware")
    reject_removed_keys(
        raw,
        {
            "require_manual_arm",
            "require_estop",
            "allow_enable_on_start",
        },
        "hardware",
    )
    return {
        "allow_real_can": require_bool(raw, "allow_real_can", "hardware"),
    }


def parse_safety_policy_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    reject_unknown_keys(
        raw,
        {
            "velocity_damping_kd",
            "damping_timeout_s",
            "command_loss_action",
            "feedback_stale_action",
        },
        "safety",
    )
    return {
        "velocity_damping_kd": require_float(raw, "velocity_damping_kd", "safety"),
        "damping_timeout_s": require_float(raw, "damping_timeout_s", "safety"),
        "command_loss_action": str(require_key(raw, "command_loss_action", "safety")),
        "feedback_stale_action": str(require_key(raw, "feedback_stale_action", "safety")),
    }


def parse_state_machine_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    reject_unknown_keys(raw, {"enable_duration_s"}, "state_machine")
    return {
        "enable_duration_s": require_float(raw, "enable_duration_s", "state_machine"),
    }


def parse_robot_controller_core_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    reject_unknown_keys(raw, {"name", "control_hz", "shutdown_timeout_s"}, "robot_controller")
    return {
        "name": str(require_key(raw, "name", "robot_controller")),
        "control_hz": require_float(raw, "control_hz", "robot_controller"),
        "shutdown_timeout_s": require_float(raw, "shutdown_timeout_s", "robot_controller"),
    }


def parse_imu_device_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    reject_unknown_keys(
        raw,
        {
            "type",
            "request_id",
            "quat_id",
            "gyro_id",
            "cmd_get_quat",
            "cmd_get_gyro",
            "cmd_get_all",
            "quat_scale",
            "gyro_scale",
            "normalize_quat",
        },
        "imu",
    )
    config = {
        "type": require_non_empty_string(raw, "type", "imu"),
        "request_id": parse_int(require_key(raw, "request_id", "imu")),
        "quat_id": parse_int(require_key(raw, "quat_id", "imu")),
        "gyro_id": parse_int(require_key(raw, "gyro_id", "imu")),
        "cmd_get_quat": parse_int(require_key(raw, "cmd_get_quat", "imu")),
        "cmd_get_gyro": parse_int(require_key(raw, "cmd_get_gyro", "imu")),
        "cmd_get_all": parse_int(require_key(raw, "cmd_get_all", "imu")),
        "quat_scale": float(require_key(raw, "quat_scale", "imu")),
        "gyro_scale": float(require_key(raw, "gyro_scale", "imu")),
        "normalize_quat": require_bool(raw, "normalize_quat", "imu"),
    }
    validate_imu_device_config(config)
    return config


def parse_driver_configs(raw: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    drivers: dict[str, dict[str, Any]] = {}
    for key, item in raw.items():
        name = non_empty_string(key, "drivers key")
        if not isinstance(item, dict):
            raise AppConfigError(f"Config key must be a mapping: drivers.{name}")
        drivers[name] = parse_spg_mit_driver_config(item, f"drivers.{name}")
    if not drivers:
        raise AppConfigError("drivers must not be empty")
    return drivers


def parse_spg_mit_driver_config(raw: Mapping[str, Any], path: str) -> dict[str, Any]:
    reject_unknown_keys(
        raw,
        {
            "p_max_rad",
            "v_max_rad_s",
            "kp_max",
            "kd_max",
            "tau_max_nm",
            "feedback_position_max_rad",
            "iq_full_scale_count",
            "iq_full_scale_current_a",
            "set_zero_hold_s",
        },
        path,
    )
    config = {
        "p_max_rad": float(require_key(raw, "p_max_rad", path)),
        "v_max_rad_s": float(require_key(raw, "v_max_rad_s", path)),
        "kp_max": float(require_key(raw, "kp_max", path)),
        "kd_max": float(require_key(raw, "kd_max", path)),
        "tau_max_nm": float(require_key(raw, "tau_max_nm", path)),
        "feedback_position_max_rad": float(require_key(raw, "feedback_position_max_rad", path)),
        "iq_full_scale_count": float(require_key(raw, "iq_full_scale_count", path)),
        "iq_full_scale_current_a": float(require_key(raw, "iq_full_scale_current_a", path)),
        "set_zero_hold_s": float(require_key(raw, "set_zero_hold_s", path)),
    }
    validate_spg_mit_driver_config(config, path)
    return config


def parse_can_config(
    raw: Mapping[str, Any],
    can_device: Mapping[str, Any],
    *,
    robot: Robot = QHRR0,
) -> dict[str, Any]:
    if "motors" in raw:
        raise AppConfigError("can.motors was removed; motor CAN IDs come from qhrr0_spec")
    reject_unknown_keys(
        raw,
        {
            "interface",
            "bitrate",
            "daemon_socket",
            "command_timeout_s",
            "bringup_delay_s",
            "daemon",
            "imu",
        },
        "can",
    )

    daemon_raw = require_mapping(raw, "daemon", "can")
    reject_unknown_keys(
        daemon_raw,
        {
            "rx_timeout_s",
            "tx_timeout_s",
            "join_timeout_s",
            "max_tx_queue_size",
            "send_block",
            "send_timeout_s",
            "connect_timeout_s",
        },
        "can.daemon",
    )
    imu_raw = require_mapping(raw, "imu", "can")
    reject_unknown_keys(
        imu_raw,
        {
            "enabled",
            "request_all_on_start",
            "request_all_each_tick",
            "startup_request_count",
            "startup_request_delay_s",
        },
        "can.imu",
    )
    interface = str(require_key(raw, "interface", "can"))
    if interface not in set(robot.can_interfaces):
        raise AppConfigError("can.interface is not listed in qhrr0 can_interfaces")

    validate_actuator_drivers(can_device, robot=robot)
    spg = require_spg_mit_driver(can_device)

    config = {
        "interface": interface,
        "bitrate": require_int(raw, "bitrate", "can"),
        "command_timeout_s": require_float(raw, "command_timeout_s", "can"),
        "bringup_delay_s": require_float(raw, "bringup_delay_s", "can"),
        "daemon": {
            "rx_timeout_s": require_float(daemon_raw, "rx_timeout_s", "can.daemon"),
            "tx_timeout_s": require_float(daemon_raw, "tx_timeout_s", "can.daemon"),
            "join_timeout_s": require_float(daemon_raw, "join_timeout_s", "can.daemon"),
            "max_tx_queue_size": require_int(daemon_raw, "max_tx_queue_size", "can.daemon"),
            "send_block": require_bool(daemon_raw, "send_block", "can.daemon"),
            "send_timeout_s": optional_float_or_none(daemon_raw, "send_timeout_s", "can.daemon"),
            "ipc_socket_path": str(require_key(raw, "daemon_socket", "can")),
            "connect_timeout_s": require_float(daemon_raw, "connect_timeout_s", "can.daemon"),
        },
        "motors": {
            "can_ids": [int(actuator.can_id) for actuator in robot.actuators],
        },
        "imu": {
            "enabled": require_bool(imu_raw, "enabled", "can.imu"),
            "request_all_on_start": require_bool(imu_raw, "request_all_on_start", "can.imu"),
            "request_all_each_tick": require_bool(imu_raw, "request_all_each_tick", "can.imu"),
            "startup_request_count": require_int(imu_raw, "startup_request_count", "can.imu"),
            "startup_request_delay_s": require_float(imu_raw, "startup_request_delay_s", "can.imu"),
        },
        "mit_protocol_range": {
            "position_rad": float(spg["p_max_rad"]),
            "velocity_rad_s": float(spg["v_max_rad_s"]),
            "kp": float(spg["kp_max"]),
            "kd": float(spg["kd_max"]),
            "torque_ff_nm": float(spg["tau_max_nm"]),
            "feedback_position_rad": float(spg["feedback_position_max_rad"]),
        },
    }
    validate_can_config(config)
    return config


def parse_shm_config(raw: Mapping[str, Any], target_count: int) -> dict[str, Any]:
    if "cleanup_stale_on_start" in raw:
        raise AppConfigError("shm.cleanup_stale_on_start was removed; cleanup is controller default behavior")
    if "unlink_on_shutdown" in raw:
        raise AppConfigError("shm.unlink_on_shutdown was removed; unlink is controller default behavior")
    reject_unknown_keys(
        raw,
        {
            "mit_command",
            "aux_command",
            "operator_command",
            "control_state",
            "dashboard_state",
        },
        "shm",
    )

    mit_command_raw = require_mapping(raw, "mit_command", "shm")
    reject_unknown_keys(mit_command_raw, {"name"}, "shm.mit_command")
    aux_command_raw = require_mapping(raw, "aux_command", "shm")
    reject_unknown_keys(aux_command_raw, {"name", "size_bytes", "publish_hz"}, "shm.aux_command")
    operator_command_raw = require_mapping(raw, "operator_command", "shm")
    reject_unknown_keys(operator_command_raw, {"name", "size_bytes"}, "shm.operator_command")
    control_state_raw = require_mapping(raw, "control_state", "shm")
    reject_unknown_keys(control_state_raw, {"name", "size_bytes", "publish_hz"}, "shm.control_state")
    dashboard_state_raw = require_mapping(raw, "dashboard_state", "shm")
    reject_unknown_keys(dashboard_state_raw, {"name", "size_bytes", "publish_hz"}, "shm.dashboard_state")
    config = {
        "mit_command": {
            "name": require_shm_name(mit_command_raw, "name", "shm.mit_command"),
            "target_count": int(target_count),
        },
        "aux_command": {
            "name": require_shm_name(aux_command_raw, "name", "shm.aux_command"),
            "size_bytes": require_int(aux_command_raw, "size_bytes", "shm.aux_command"),
            "publish_hz": require_float(aux_command_raw, "publish_hz", "shm.aux_command"),
        },
        "operator_command": {
            "name": require_shm_name(operator_command_raw, "name", "shm.operator_command"),
            "size_bytes": require_int(operator_command_raw, "size_bytes", "shm.operator_command"),
        },
        "control_state": {
            "name": require_shm_name(control_state_raw, "name", "shm.control_state"),
            "size_bytes": require_int(control_state_raw, "size_bytes", "shm.control_state"),
            "publish_hz": require_float(control_state_raw, "publish_hz", "shm.control_state"),
        },
        "dashboard_state": {
            "name": require_shm_name(dashboard_state_raw, "name", "shm.dashboard_state"),
            "size_bytes": require_int(dashboard_state_raw, "size_bytes", "shm.dashboard_state"),
            "publish_hz": require_float(dashboard_state_raw, "publish_hz", "shm.dashboard_state"),
        },
    }
    validate_shm_config(config)
    return config


def parse_process_config(item: Any, index: int) -> dict[str, Any]:
    if not isinstance(item, dict):
        raise AppConfigError(f"Config key must be a mapping: processes[{index}]")
    if "command" in item:
        raise AppConfigError("processes[*].command was removed; commands are generated by process builders")
    if "env" in item:
        raise AppConfigError("processes[*].env was renamed to processes[*].env_vars")
    reject_unknown_keys(
        item,
        {
            "name",
            "start_order",
            "stop_order",
            "new_terminal",
            "terminal_command",
            "working_dir",
            "env_vars",
        },
        f"processes[{index}]",
    )
    env_vars_raw = require_mapping(item, "env_vars", f"processes[{index}]")
    config = {
        "name": str(require_key(item, "name", f"processes[{index}]")),
        "start_order": require_int(item, "start_order", f"processes[{index}]"),
        "stop_order": require_int(item, "stop_order", f"processes[{index}]"),
        "new_terminal": require_bool(item, "new_terminal", f"processes[{index}]"),
        "terminal_command": [
            str(part)
            for part in require_list(item, "terminal_command", f"processes[{index}]")
        ],
        "working_dir": str(require_key(item, "working_dir", f"processes[{index}]")),
        "env_vars": {str(key): str(value) for key, value in env_vars_raw.items()},
    }
    validate_process_config(config)
    return config


def load_processes_config(path: str | Path) -> dict[str, Any]:
    raw = load_yaml_mapping(path)
    reject_unknown_keys(raw, {"processes", "subprocesses"}, str(path))
    configs = [
        parse_process_config(item, index)
        for index, item in enumerate(require_list(raw, "processes", str(path)))
    ]
    validate_processes_config(configs)
    subprocesses = parse_subprocesses_config(require_mapping(raw, "subprocesses", str(path)))
    return {
        "orchestration": configs,
        "subprocesses": subprocesses,
    }


def parse_subprocesses_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    reject_unknown_keys(raw, {"can_daemon", "aux_reader", "task_controller", "dashboard"}, "subprocesses")
    return {
        "can_daemon": parse_can_daemon_subprocess_config(
            require_mapping(raw, "can_daemon", "subprocesses")
        ),
        "aux_reader": parse_aux_reader_subprocess_config(
            require_mapping(raw, "aux_reader", "subprocesses")
        ),
        "task_controller": parse_task_controller_subprocess_config(
            require_mapping(raw, "task_controller", "subprocesses")
        ),
        "dashboard": parse_dashboard_subprocess_config(
            require_mapping(raw, "dashboard", "subprocesses")
        ),
    }


def parse_can_daemon_subprocess_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    reject_unknown_keys(raw, {"replace_existing_socket"}, "subprocesses.can_daemon")
    return {
        "replace_existing_socket": require_bool(raw, "replace_existing_socket", "subprocesses.can_daemon"),
    }


def parse_aux_reader_subprocess_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    reject_unknown_keys(raw, {"joystick_dev"}, "subprocesses.aux_reader")
    return {
        "joystick_dev": require_non_empty_string(raw, "joystick_dev", "subprocesses.aux_reader"),
    }


def parse_task_controller_subprocess_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    reject_unknown_keys(
        raw,
        {"control_hz", "rate_log_interval_s", "project_root"},
        "subprocesses.task_controller",
    )
    return {
        "control_hz": require_float(raw, "control_hz", "subprocesses.task_controller"),
        "rate_log_interval_s": optional_float_or_none(
            raw,
            "rate_log_interval_s",
            "subprocesses.task_controller",
        ),
        "project_root": require_non_empty_string(raw, "project_root", "subprocesses.task_controller"),
    }


def parse_dashboard_subprocess_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    reject_unknown_keys(raw, set(), "subprocesses.dashboard")
    return {}


def _project_root_from_config_paths(path: Path) -> Path:
    resolved = path.resolve()
    try:
        return resolved.parents[1]
    except IndexError as exc:
        raise AppConfigError(f"config paths file is too shallow to resolve package root: {path}") from exc


def _parse_path_mapping(raw: Mapping[object, object], base: Path) -> dict[str, Path]:
    paths: dict[str, Path] = {}
    for key, item in raw.items():
        if key is None:
            raise AppConfigError("config path keys must not be empty")
        if item is None:
            raise AppConfigError(f"config path value must not be empty: {key}")
        key_text = str(key)
        item_text = str(item)
        if not key_text:
            raise AppConfigError("config path keys must not be empty")
        if not item_text:
            raise AppConfigError(f"config path value must not be empty: {key_text}")
        candidate = Path(item_text)
        if not candidate.is_absolute():
            candidate = base / candidate
        paths[key_text] = candidate.resolve()
    return paths
