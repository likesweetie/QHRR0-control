from __future__ import annotations

from typing import Any, Mapping

from qhrr0.factory.robot_factory.base.robot_base import Robot
from qhrr0.qhrr0_spec import QHRR0


class AppConfigError(ValueError):
    pass


def require_robot_matches_config(robot: Robot, config: Mapping[str, Any]) -> None:
    imu_config = value(config, "can_device.imu")
    if robot.imu.driver != imu_config["type"]:
        raise AppConfigError(
            "QHRR0 IMU driver does not match can_device.imu.type: "
            f"{robot.imu.driver!r} != {imu_config['type']!r}"
        )


def validate_runtime_safety(config: Mapping[str, Any], options: Mapping[str, Any]) -> None:
    interface = str(value(config, "can.interface"))
    is_virtual_can = interface.startswith("vcan")
    is_real_can = interface.startswith("can")
    if value(config, "runtime.mode") == "simulation":
        if is_real_can:
            raise AppConfigError("simulation mode rejects real CAN interface")
        return
    if value(config, "runtime.mode") != "hardware":
        raise AppConfigError("runtime.mode must be 'simulation' or 'hardware'")
    if not bool(options.get("hardware_requested", False)):
        raise AppConfigError("hardware mode requires --hardware")
    if not bool(options.get("motor_enable_confirmed", False)):
        raise AppConfigError("hardware mode requires --i-understand-this-can-enable-motors")
    if is_virtual_can:
        raise AppConfigError("hardware mode rejects virtual CAN interface")
    if not is_real_can:
        raise AppConfigError("hardware mode requires a real CAN interface")
    if not value(config, "hardware.allow_real_can"):
        raise AppConfigError("hardware mode requires hardware.allow_real_can")
    if not bool(options.get("estop_ok", False)):
        raise AppConfigError("hardware mode requires --estop-ok")


def validate_app_config(config: Mapping[str, Any]) -> None:
    if value(config, "runtime.mode") not in ("simulation", "hardware"):
        raise AppConfigError("runtime.mode must be 'simulation' or 'hardware'")
    if float(value(config, "safety.velocity_damping_kd")) < 0.0:
        raise AppConfigError("safety.velocity_damping_kd must be >= 0")
    if float(value(config, "safety.damping_timeout_s")) <= 0.0:
        raise AppConfigError("safety.damping_timeout_s must be > 0")
    if value(config, "safety.command_loss_action") not in ("damping", "disable", "fault"):
        raise AppConfigError("safety.command_loss_action must be damping, disable, or fault")
    if value(config, "safety.feedback_stale_action") not in ("damping", "disable", "fault"):
        raise AppConfigError("safety.feedback_stale_action must be damping, disable, or fault")
    if float(value(config, "robot_controller.control_hz")) <= 0.0:
        raise AppConfigError("robot_controller.control_hz must be > 0")
    if float(value(config, "robot_controller.shutdown_timeout_s")) < 0.0:
        raise AppConfigError("robot_controller.shutdown_timeout_s must be >= 0")
    if float(value(config, "state_machine.enable_duration_s")) < 0.0:
        raise AppConfigError("state_machine.enable_duration_s must be >= 0")
    if int(value(config, "shm.mit_command.target_count")) != len(value(config, "can.motors.can_ids")):
        raise AppConfigError("shm.mit_command.target_count must match len(can.motors.can_ids)")
    if float(value(config, "safety.velocity_damping_kd")) > float(value(config, "can.mit_protocol_range.kd")):
        raise AppConfigError("safety.velocity_damping_kd must be <= can.mit_protocol_range.kd")
    process_names = {str(process["name"]) for process in value(config, "processes.orchestration")}
    if "can_daemon" not in process_names:
        raise AppConfigError("processes must include a 'can_daemon' subprocess")


def validate_can_device_config(config: Mapping[str, Any]) -> None:
    validate_imu_device_config(value(config, "imu"))
    for name, driver in value(config, "drivers").items():
        _non_empty_string(name, "drivers key")
        validate_spg_mit_driver_config(driver, f"drivers.{name}")


def validate_imu_device_config(config: Mapping[str, Any]) -> None:
    if config["type"] != "e2box":
        raise AppConfigError("imu.type must be 'e2box'")
    for key in ("request_id", "quat_id", "gyro_id"):
        if int(config[key]) <= 0:
            raise AppConfigError(f"imu.{key} must be > 0")
    for key in ("cmd_get_quat", "cmd_get_gyro", "cmd_get_all"):
        if int(config[key]) < 0:
            raise AppConfigError(f"imu.{key} must be >= 0")
    if float(config["quat_scale"]) <= 0.0:
        raise AppConfigError("imu.quat_scale must be > 0")
    if float(config["gyro_scale"]) <= 0.0:
        raise AppConfigError("imu.gyro_scale must be > 0")


def validate_spg_mit_driver_config(config: Mapping[str, Any], path: str) -> None:
    for key in (
        "p_max_rad",
        "v_max_rad_s",
        "kp_max",
        "kd_max",
        "tau_max_nm",
        "feedback_position_max_rad",
        "iq_full_scale_count",
        "iq_full_scale_current_a",
    ):
        if float(config[key]) <= 0.0:
            raise AppConfigError(f"{path}.{key} must be > 0")
    if float(config["set_zero_hold_s"]) < 0.0:
        raise AppConfigError(f"{path}.set_zero_hold_s must be >= 0")


def validate_can_config(config: Mapping[str, Any]) -> None:
    if not config["interface"]:
        raise AppConfigError("can.interface must not be empty")
    if int(config["bitrate"]) <= 0:
        raise AppConfigError("can.bitrate must be > 0")
    if float(config["command_timeout_s"]) <= 0.0:
        raise AppConfigError("can.command_timeout_s must be > 0")
    if float(config["bringup_delay_s"]) < 0.0:
        raise AppConfigError("can.bringup_delay_s must be >= 0")
    daemon = value(config, "daemon")
    for key in ("rx_timeout_s", "tx_timeout_s", "join_timeout_s"):
        if float(daemon[key]) < 0.0:
            raise AppConfigError(f"can.daemon.{key} must be >= 0")
    if int(daemon["max_tx_queue_size"]) <= 0:
        raise AppConfigError("can.daemon.max_tx_queue_size must be > 0")
    if daemon["send_timeout_s"] is not None and float(daemon["send_timeout_s"]) < 0.0:
        raise AppConfigError("can.daemon.send_timeout_s must be null or >= 0")
    if not daemon["ipc_socket_path"]:
        raise AppConfigError("can.daemon.ipc_socket_path must not be empty")
    if float(daemon["connect_timeout_s"]) <= 0.0:
        raise AppConfigError("can.daemon.connect_timeout_s must be > 0")
    can_ids = value(config, "motors.can_ids")
    if not can_ids:
        raise AppConfigError("can.motors.can_ids must not be empty")
    if len(set(can_ids)) != len(can_ids):
        raise AppConfigError("can.motors.can_ids must not contain duplicates")
    imu = value(config, "imu")
    if int(imu["startup_request_count"]) < 0:
        raise AppConfigError("can.imu.startup_request_count must be >= 0")
    if float(imu["startup_request_delay_s"]) < 0.0:
        raise AppConfigError("can.imu.startup_request_delay_s must be >= 0")
    protocol = value(config, "mit_protocol_range")
    for key in ("position_rad", "velocity_rad_s", "torque_ff_nm", "feedback_position_rad"):
        if float(protocol[key]) <= 0.0:
            raise AppConfigError(f"can.mit_protocol_range.{key} must be > 0")
    if float(protocol["kp"]) < 0.0:
        raise AppConfigError("can.mit_protocol_range.kp must be >= 0")
    if float(protocol["kd"]) < 0.5:
        raise AppConfigError("can.mit_protocol_range.kd must be >= 0.5 for shutdown damping")


def validate_shm_config(config: Mapping[str, Any]) -> None:
    if int(value(config, "mit_command.target_count")) <= 0:
        raise AppConfigError("shm.mit_command.target_count must be > 0")
    for path, minimum in (
        ("control_state.size_bytes", 4096),
        ("aux_command.size_bytes", 4096),
        ("operator_command.size_bytes", 4096),
        ("dashboard_state.size_bytes", 4096),
    ):
        if int(value(config, path)) < minimum:
            raise AppConfigError(f"shm.{path} must be >= {minimum}")
    for path in ("control_state.publish_hz", "aux_command.publish_hz", "dashboard_state.publish_hz"):
        if float(value(config, path)) <= 0.0:
            raise AppConfigError(f"shm.{path} must be > 0")
    names = {
        value(config, "mit_command.name"),
        value(config, "aux_command.name"),
        value(config, "operator_command.name"),
        value(config, "control_state.name"),
        value(config, "dashboard_state.name"),
    }
    if len(names) != 5:
        raise AppConfigError("shm segment names must be unique")


def validate_process_config(config: Mapping[str, Any]) -> None:
    if not config["name"]:
        raise AppConfigError("process name must not be empty")
    if not config["working_dir"]:
        raise AppConfigError(f"process {config['name']} working_dir must not be empty")
    if config["new_terminal"] and not config["terminal_command"]:
        raise AppConfigError(
            f"process {config['name']} terminal_command must not be empty when new_terminal is true"
        )


def validate_processes_config(configs: list[Mapping[str, Any]]) -> None:
    names = [str(process["name"]) for process in configs]
    if len(set(names)) != len(names):
        raise AppConfigError("processes must not contain duplicate names")


def require_no_platform_owned_dashboard_keys(config: Mapping[str, Any]) -> None:
    checks = (
        ("can", ("iface", "bitrate")),
        ("can_daemon", ("ipc_socket_path",)),
        ("robot_controller_state", ("control_shm_name", "dashboard_shm_name", "operator_shm_name", "operator_shm_size_bytes")),
        (
            "imu",
            (
                "request_id",
                "quat_id",
                "gyro_id",
                "cmd_get_all",
                "cmd_get_quat",
                "cmd_get_gyro",
                "quat_scale",
                "gyro_scale",
                "normalize_quat",
            ),
        ),
        (
            "spg",
            (
                "feedback_position_max_rad",
                "iq_full_scale_count",
                "iq_full_scale_current_a",
                "p_max_rad",
                "v_max_rad_s",
                "kp_max",
                "kd_max",
                "tau_max_nm",
            ),
        ),
    )
    for section, keys in checks:
        section_value = config.get(section)
        if not isinstance(section_value, dict):
            continue
        for key in keys:
            if key in section_value:
                raise AppConfigError(f"Dashboard config must not override platform-owned key: {section}.{key}")
    if "actuators" in config:
        raise AppConfigError("Dashboard config must not define platform-owned key: actuators")


def reject_removed_dashboard_keys(raw: Mapping[str, Any]) -> None:
    for key in ("platform_config", "robot_controller_config"):
        if key in raw:
            raise AppConfigError(f"Dashboard config key '{key}' is no longer supported")
    dashboard = require_mapping(raw, "dashboard", "dashboard")
    if "state_hz" in dashboard:
        raise AppConfigError("Dashboard config key 'dashboard.state_hz' was renamed to dashboard.state_update_rate")
    if "transmit_ids" in dashboard:
        raise AppConfigError("Dashboard config key 'dashboard.transmit_ids' was removed")
    safety = raw.get("safety")
    if isinstance(safety, dict) and "allow_direct_can_transmit" in safety:
        raise AppConfigError("Dashboard config key 'safety.allow_direct_can_transmit' was removed")


def resolve_zero_set_presets(config: dict[str, Any], *, robot: Robot = QHRR0) -> None:
    dashboard = require_mapping(config, "dashboard", "dashboard")
    raw = config.get("zero_set_presets", dashboard.get("zero_set_presets", []))
    if raw is None:
        raw = []
    if not isinstance(raw, list):
        raise AppConfigError("Dashboard config key 'zero_set_presets' must be a list")
    actuators = {actuator.name: actuator.can_id for actuator in robot.actuators}
    resolved = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise AppConfigError(f"zero_set_presets[{index}] must be a mapping")
        targets_raw = item.get("targets")
        if not isinstance(targets_raw, list) or not targets_raw:
            raise AppConfigError(f"zero_set_presets[{index}].targets must be a non-empty list")
        targets = []
        for target_index, target in enumerate(targets_raw):
            if not isinstance(target, dict):
                raise AppConfigError(f"zero_set_presets[{index}].targets[{target_index}] must be a mapping")
            if "can_id" in target:
                raise AppConfigError(f"zero_set_presets[{index}].targets[{target_index}].can_id must come from actuator")
            if "actuator" not in target:
                raise AppConfigError(f"zero_set_presets[{index}].targets[{target_index}] requires actuator")
            name = str(target["actuator"])
            if name not in actuators:
                raise AppConfigError(
                    f"zero_set_presets[{index}].targets[{target_index}] references unknown actuator: {name}"
                )
            if "offset_count" in target and "offset_deg" in target:
                raise AppConfigError(
                    f"zero_set_presets[{index}].targets[{target_index}] must use either offset_count or offset_deg, not both"
                )
            if "offset_deg" in target:
                offset_deg = float(target["offset_deg"])
                offset_count = round_half_away_from_zero(offset_deg * 100.0)
            elif "offset_count" in target:
                offset_count = int(target["offset_count"])
                offset_deg = offset_count * 0.01
            else:
                raise AppConfigError(f"zero_set_presets[{index}].targets[{target_index}] requires offset_deg or offset_count")
            if not (-32768 <= offset_count <= 32767):
                raise AppConfigError(
                    f"zero_set_presets[{index}].targets[{target_index}] offset_count out of int16 range: {offset_count}"
                )
            targets.append(
                {
                    "actuator": name,
                    "can_id": actuators[name],
                    "offset_deg": offset_deg,
                    "offset_count": offset_count,
                }
            )
        resolved.append(
            {
                "id": str(item.get("id") or f"preset_{index + 1}"),
                "label": str(item.get("label") or f"Preset {index + 1}"),
                "targets": targets,
            }
        )
    config["zero_set_presets"] = resolved
    dashboard["zero_set_presets"] = resolved


def validate_actuator_drivers(can_device: Mapping[str, Any], *, robot: Robot = QHRR0) -> None:
    driver_names = set(value(can_device, "drivers"))
    for actuator in robot.actuators:
        if actuator.driver not in driver_names:
            raise AppConfigError(
                f"QHRR0 actuator {actuator.name} references unknown driver: {actuator.driver}"
            )


def require_spg_mit_driver(can_device: Mapping[str, Any]) -> Mapping[str, Any]:
    drivers = value(can_device, "drivers")
    if "spg_mit" not in drivers:
        raise AppConfigError("can_device.drivers.spg_mit is required")
    return drivers["spg_mit"]


def require_key(mapping: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in mapping:
        raise AppConfigError(f"Missing required config key: {path}.{key}")
    return mapping[key]


def require_mapping(mapping: Mapping[str, Any], key: str, path: str) -> dict[str, Any]:
    item = require_key(mapping, key, path)
    if not isinstance(item, dict):
        raise AppConfigError(f"Config key must be a mapping: {path}.{key}")
    return item


def require_list(mapping: Mapping[str, Any], key: str, path: str) -> list[Any]:
    item = require_key(mapping, key, path)
    if not isinstance(item, list):
        raise AppConfigError(f"Config key must be a list: {path}.{key}")
    return item


def require_bool(mapping: Mapping[str, Any], key: str, path: str) -> bool:
    item = require_key(mapping, key, path)
    if not isinstance(item, bool):
        raise AppConfigError(f"Config key must be a boolean: {path}.{key}")
    return item


def require_float(mapping: Mapping[str, Any], key: str, path: str) -> float:
    return float(require_key(mapping, key, path))


def require_int(mapping: Mapping[str, Any], key: str, path: str) -> int:
    return int(require_key(mapping, key, path))


def optional_float_or_none(mapping: Mapping[str, Any], key: str, path: str) -> float | None:
    item = require_key(mapping, key, path)
    return None if item is None else float(item)


def parse_int(item: Any) -> int:
    return int(item, 0) if isinstance(item, str) else int(item)


def value(mapping: Mapping[str, Any], path: str) -> Any:
    current: Any = mapping
    for part in path.split("."):
        if not isinstance(current, Mapping) or part not in current:
            raise AppConfigError(f"Missing required config key: {path}")
        current = current[part]
    return current


def reject_removed_keys(raw: Mapping[str, Any], removed_keys: set[str], path: str) -> None:
    for key in sorted(removed_keys):
        if key in raw:
            raise AppConfigError(f"{path}.{key} is not supported by robot_controller.yaml")


def reject_unknown_keys(raw: Mapping[str, Any], supported_keys: set[str], path: str) -> None:
    for key in sorted(set(raw) - supported_keys):
        raise AppConfigError(f"Unsupported config key: {path}.{key}")


def require_non_empty_string(raw: Mapping[str, Any], key: str, path: str) -> str:
    return _non_empty_string(require_key(raw, key, path), f"{path}.{key}")


def non_empty_string(item: Any, path: str) -> str:
    return _non_empty_string(item, path)


def _non_empty_string(item: Any, path: str) -> str:
    if item is None:
        raise AppConfigError(f"Config key must not be empty: {path}")
    text = str(item).strip()
    if not text:
        raise AppConfigError(f"Config key must not be empty: {path}")
    return text


def require_shm_non_empty_string(raw: Mapping[str, Any], key: str, path: str) -> str:
    item = require_key(raw, key, path)
    if item is None:
        raise AppConfigError(f"Config key must not be empty: {path}.{key}")
    text = str(item).strip()
    if not text:
        raise AppConfigError(f"Config key must not be empty: {path}.{key}")
    return text


def require_shm_name(raw: Mapping[str, Any], key: str, path: str) -> str:
    return require_shm_non_empty_string(raw, key, path)


def round_half_away_from_zero(item: float) -> int:
    return int(item + 0.5) if item >= 0.0 else int(item - 0.5)
