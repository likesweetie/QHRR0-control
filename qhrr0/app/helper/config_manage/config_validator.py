from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any


def validate_robot_controller_config(config: Mapping[str, Any]) -> None:
    if not isinstance(config, Mapping):
        raise TypeError(f"RobotController config must be a mapping, got {type(config).__name__}")

    robot_platform = config["robot_platform"]
    hardware = config["hardware"]
    can = config["can"]
    shm = config["shm"]
    state_machine = config["state_machine"]
    robot_controller = config["robot_controller"]
    safety = config["safety"]
    processes = config["processes"]

    for name, value in (
        ("robot_platform", robot_platform),
        ("hardware", hardware),
        ("can", can),
        ("shm", shm),
        ("state_machine", state_machine),
        ("robot_controller", robot_controller),
        ("safety", safety),
    ):
        if not isinstance(value, Mapping):
            raise TypeError(
                f"RobotController config '{name}' must be a mapping, "
                f"got {type(value).__name__}"
            )

    hardware_can = hardware["can"]
    hardware_can_drivers = hardware_can["drivers"]
    spg_mit = hardware_can_drivers["spg_mit"]
    can_servers = can["servers"]
    can_imu = can["imu"]
    mit_protocol_range = can["mit_protocol_range"]
    shm_mit_command = shm["mit_command"]
    shm_operator_command = shm["operator_command"]
    shm_control_state = shm["control_state"]
    shm_dashboard_state = shm["dashboard_state"]

    for name, value in (
        ("hardware.can", hardware_can),
        ("hardware.can.drivers", hardware_can_drivers),
        ("hardware.can.drivers.spg_mit", spg_mit),
        ("can.imu", can_imu),
        ("can.mit_protocol_range", mit_protocol_range),
        ("shm.mit_command", shm_mit_command),
        ("shm.operator_command", shm_operator_command),
        ("shm.control_state", shm_control_state),
        ("shm.dashboard_state", shm_dashboard_state),
    ):
        if not isinstance(value, Mapping):
            raise TypeError(
                f"RobotController config '{name}' must be a mapping, "
                f"got {type(value).__name__}"
            )

    required_values = (
        ("can.command_timeout_s", can["command_timeout_s"]),
        ("can.imu.enabled", can_imu["enabled"]),
        ("can.imu.request_all_on_start", can_imu["request_all_on_start"]),
        ("can.imu.request_all_each_tick", can_imu["request_all_each_tick"]),
        ("can.imu.startup_request_count", can_imu["startup_request_count"]),
        ("can.imu.startup_request_delay_s", can_imu["startup_request_delay_s"]),
        ("can.mit_protocol_range.position_rad", mit_protocol_range["position_rad"]),
        ("can.mit_protocol_range.velocity_rad_s", mit_protocol_range["velocity_rad_s"]),
        ("can.mit_protocol_range.kp", mit_protocol_range["kp"]),
        ("can.mit_protocol_range.kd", mit_protocol_range["kd"]),
        ("can.mit_protocol_range.torque_ff_nm", mit_protocol_range["torque_ff_nm"]),
        ("can.mit_protocol_range.feedback_position_rad", mit_protocol_range["feedback_position_rad"]),
        ("hardware.can.drivers.spg_mit.iq_full_scale_count", spg_mit["iq_full_scale_count"]),
        ("hardware.can.drivers.spg_mit.iq_full_scale_current_a", spg_mit["iq_full_scale_current_a"]),
        ("shm.mit_command.name", shm_mit_command["name"]),
        ("shm.operator_command.name", shm_operator_command["name"]),
        ("shm.control_state.name", shm_control_state["name"]),
        ("shm.control_state.publish_hz", shm_control_state["publish_hz"]),
        ("shm.dashboard_state.name", shm_dashboard_state["name"]),
        ("shm.dashboard_state.publish_hz", shm_dashboard_state["publish_hz"]),
        ("state_machine.enable_duration_s", state_machine["enable_duration_s"]),
        ("robot_controller.control_hz", robot_controller["control_hz"]),
        ("robot_controller.shutdown_timeout_s", robot_controller["shutdown_timeout_s"]),
        ("safety.velocity_damping_kd", safety["velocity_damping_kd"]),
    )

    for name, value in (
        ("can.imu.enabled", can_imu["enabled"]),
        ("can.imu.request_all_on_start", can_imu["request_all_on_start"]),
        ("can.imu.request_all_each_tick", can_imu["request_all_each_tick"]),
    ):
        if not isinstance(value, bool):
            raise TypeError(
                f"RobotController config '{name}' must be bool, "
                f"got {type(value).__name__}"
            )

    for name, value in required_values:
        if value is None:
            raise ValueError(f"RobotController config '{name}' must not be null")

    for name, value in (
        ("can.command_timeout_s", can["command_timeout_s"]),
        ("hardware.can.drivers.spg_mit.iq_full_scale_count", spg_mit["iq_full_scale_count"]),
        ("shm.control_state.publish_hz", shm_control_state["publish_hz"]),
        ("shm.dashboard_state.publish_hz", shm_dashboard_state["publish_hz"]),
        ("robot_controller.control_hz", robot_controller["control_hz"]),
        ("robot_controller.shutdown_timeout_s", robot_controller["shutdown_timeout_s"]),
    ):
        if isinstance(value, bool) or float(value) <= 0.0:
            raise ValueError(f"RobotController config '{name}' must be positive")

    for name, value in (
        ("can.imu.startup_request_delay_s", can_imu["startup_request_delay_s"]),
        ("state_machine.enable_duration_s", state_machine["enable_duration_s"]),
        ("safety.velocity_damping_kd", safety["velocity_damping_kd"]),
    ):
        if isinstance(value, bool) or float(value) < 0.0:
            raise ValueError(f"RobotController config '{name}' must be >= 0")

    if isinstance(can_imu["startup_request_count"], bool) or int(can_imu["startup_request_count"]) < 0:
        raise ValueError("RobotController config 'can.imu.startup_request_count' must be >= 0")

    if isinstance(processes, str) or not isinstance(processes, Sequence):
        raise TypeError("RobotController config 'processes' must be a sequence of process configs")
    if isinstance(can_servers, str) or not isinstance(can_servers, Sequence) or not can_servers:
        raise TypeError("RobotController config 'can.servers' must be a non-empty sequence")
    seen_can_server_names: set[str] = set()
    for index, server in enumerate(can_servers):
        if not isinstance(server, Mapping):
            raise TypeError(f"RobotController config 'can.servers[{index}]' must be a mapping")
        name = server["name"]
        ipc_socket_path = server["ipc_socket_path"]
        connect_timeout_s = server["connect_timeout_s"]
        if not isinstance(name, str) or not name:
            raise ValueError(f"RobotController config 'can.servers[{index}].name' must be a non-empty string")
        if name in seen_can_server_names:
            raise ValueError(f"Duplicate CAN server name: {name}")
        seen_can_server_names.add(name)
        if not isinstance(ipc_socket_path, str) or not ipc_socket_path:
            raise ValueError(
                f"RobotController config 'can.servers[{index}].ipc_socket_path' "
                "must be a non-empty string"
            )
        if isinstance(connect_timeout_s, bool) or float(connect_timeout_s) <= 0.0:
            raise ValueError(
                f"RobotController config 'can.servers[{index}].connect_timeout_s' "
                "must be positive"
            )


def validate_supported_actuator_drivers(
    robot_spec: Any,
    supported_drivers: tuple[str, ...] = ("spg_mit",),
) -> None:
    unsupported_drivers = {
        spec.driver
        for spec in robot_spec.actuators
        if spec.driver not in supported_drivers
    }
    if unsupported_drivers:
        raise ValueError(f"Unsupported actuator drivers: {sorted(unsupported_drivers)}")


def validate_control_mode_fsm_config(enable_duration_s: float) -> None:
    if enable_duration_s < 0.0:
        raise ValueError("enable_duration_s must be >= 0")


def validate_process_supervisor_config(process_configs: Sequence[Mapping[str, Any]]) -> None:
    if isinstance(process_configs, str) or not isinstance(process_configs, Sequence):
        raise TypeError("ProcessSupervisor config must be a sequence of process configs")

    seen_names: set[str] = set()
    for index, config in enumerate(process_configs):
        if not isinstance(config, Mapping):
            raise TypeError(
                f"ProcessSupervisor config item #{index} must be a mapping, "
                f"got {type(config).__name__}"
            )

        name = config["name"]
        command = config["command"]
        start_order = config["start_order"]
        stop_order = config["stop_order"]
        new_terminal = config["new_terminal"]
        terminal_command = config["terminal_command"]
        working_dir = config["working_dir"]
        env_vars = config["env_vars"]

        if not isinstance(name, str) or not name:
            raise ValueError(f"ProcessSupervisor config item #{index} has invalid name")
        if name in seen_names:
            raise ValueError(f"Duplicate process name: {name}")
        seen_names.add(name)

        if isinstance(command, str) or not isinstance(command, Sequence) or not command:
            raise ValueError(f"Process {name} command must be a non-empty sequence")
        if isinstance(terminal_command, str) or not isinstance(terminal_command, Sequence):
            raise TypeError(f"Process {name} terminal_command must be a sequence")
        if not isinstance(new_terminal, bool):
            raise TypeError(f"Process {name} new_terminal must be bool")
        if new_terminal and not terminal_command:
            raise ValueError(f"Process {name} requires terminal_command")
        if not isinstance(working_dir, str) or not working_dir:
            raise ValueError(f"Process {name} working_dir must be a non-empty string")
        if not isinstance(env_vars, Mapping):
            raise TypeError(f"Process {name} env_vars must be a mapping")
        if isinstance(start_order, bool):
            raise TypeError(f"Process {name} start_order must be int-like")
        if isinstance(stop_order, bool):
            raise TypeError(f"Process {name} stop_order must be int-like")
        int(start_order)
        int(stop_order)


def validate_can_server_config(config: Mapping[str, Any]) -> None:
    if not isinstance(config, Mapping):
        raise TypeError(f"CAN server config must be a mapping, got {type(config).__name__}")

    can = config["can"]
    if not isinstance(can, Mapping):
        raise TypeError(f"CAN server config 'can' must be a mapping, got {type(can).__name__}")

    daemon = can["daemon"]
    if not isinstance(daemon, Mapping):
        raise TypeError(
            f"CAN server config 'can.daemon' must be a mapping, "
            f"got {type(daemon).__name__}"
        )

    interface = can["interface"]
    ipc_socket_path = daemon["ipc_socket_path"]
    rx_timeout_s = daemon["rx_timeout_s"]
    tx_timeout_s = daemon["tx_timeout_s"]
    join_timeout_s = daemon["join_timeout_s"]
    max_tx_queue_size = daemon["max_tx_queue_size"]
    send_block = daemon["send_block"]
    send_timeout_s = daemon["send_timeout_s"]

    if not isinstance(interface, str) or not interface:
        raise ValueError("CAN server config 'can.interface' must be a non-empty string")
    if not isinstance(ipc_socket_path, str) or not ipc_socket_path:
        raise ValueError("CAN server config 'can.daemon.ipc_socket_path' must be a non-empty string")

    for name, value in (
        ("can.daemon.rx_timeout_s", rx_timeout_s),
        ("can.daemon.tx_timeout_s", tx_timeout_s),
        ("can.daemon.join_timeout_s", join_timeout_s),
    ):
        if isinstance(value, bool) or float(value) <= 0.0:
            raise ValueError(f"CAN server config '{name}' must be positive")

    if isinstance(max_tx_queue_size, bool) or int(max_tx_queue_size) <= 0:
        raise ValueError("CAN server config 'can.daemon.max_tx_queue_size' must be positive")

    if not isinstance(send_block, bool):
        raise TypeError(
            f"CAN server config 'can.daemon.send_block' must be bool, "
            f"got {type(send_block).__name__}"
        )
    if send_timeout_s is not None and (isinstance(send_timeout_s, bool) or float(send_timeout_s) < 0.0):
        raise ValueError("CAN server config 'can.daemon.send_timeout_s' must be null or >= 0")


def validate_shm_config(config: Mapping[str, Any]) -> None:
    if not isinstance(config, Mapping):
        raise TypeError(f"SHM config must be a mapping, got {type(config).__name__}")

    mit_command = config["mit_command"]
    aux_command = config["aux_command"]
    operator_command = config["operator_command"]
    control_state = config["control_state"]
    dashboard_state = config["dashboard_state"]

    for name, value in (
        ("mit_command", mit_command),
        ("aux_command", aux_command),
        ("operator_command", operator_command),
        ("control_state", control_state),
        ("dashboard_state", dashboard_state),
    ):
        if not isinstance(value, Mapping):
            raise TypeError(f"SHM config '{name}' must be a mapping, got {type(value).__name__}")

    for name, value in (
        ("mit_command.name", mit_command["name"]),
        ("aux_command.name", aux_command["name"]),
        ("operator_command.name", operator_command["name"]),
        ("control_state.name", control_state["name"]),
        ("dashboard_state.name", dashboard_state["name"]),
    ):
        if not isinstance(value, str) or not value:
            raise ValueError(f"SHM config '{name}' must be a non-empty string")

    for name, value in (
        ("aux_command.size_bytes", aux_command["size_bytes"]),
        ("operator_command.size_bytes", operator_command["size_bytes"]),
        ("control_state.size_bytes", control_state["size_bytes"]),
        ("dashboard_state.size_bytes", dashboard_state["size_bytes"]),
    ):
        if isinstance(value, bool) or int(value) <= 0:
            raise ValueError(f"SHM config '{name}' must be positive")

    for name, value in (
        ("control_state.publish_hz", control_state["publish_hz"]),
        ("dashboard_state.publish_hz", dashboard_state["publish_hz"]),
    ):
        if isinstance(value, bool) or float(value) <= 0.0:
            raise ValueError(f"SHM config '{name}' must be positive")
