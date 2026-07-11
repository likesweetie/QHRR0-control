import pytest

from qhrr0.app.helper.config_manage import (
    validate_can_server_config,
    validate_control_mode_fsm_config,
    validate_process_supervisor_config,
    validate_robot_controller_config,
    validate_shm_config,
    validate_supported_actuator_drivers,
)


def _valid_robot_controller_config() -> dict:
    return {
        "robot_platform": {},
        "hardware": {
            "can": {
                "drivers": {
                    "spg_mit": {
                        "iq_full_scale_count": 2048.0,
                        "iq_full_scale_current_a": 33.0,
                    },
                },
            },
        },
        "can": {
            "servers": [
                {
                    "name": "can_server",
                    "ipc_socket_path": "/tmp/qhrr_can.sock",
                    "connect_timeout_s": 1.0,
                },
            ],
            "daemon": {
                "ipc_socket_path": "/tmp/qhrr_can.sock",
                "connect_timeout_s": 1.0,
            },
            "command_timeout_s": 0.05,
            "imu": {
                "enabled": True,
                "request_all_on_start": True,
                "request_all_each_tick": False,
                "startup_request_count": 3,
                "startup_request_delay_s": 0.01,
            },
            "mit_protocol_range": {
                "position_rad": 12.5,
                "velocity_rad_s": 45.0,
                "kp": 500.0,
                "kd": 5.0,
                "torque_ff_nm": 33.0,
                "feedback_position_rad": 12.56,
            },
        },
        "shm": {
            "mit_command": {"name": "qhrr_mit_command"},
            "operator_command": {"name": "qhrr_operator_command"},
            "control_state": {
                "name": "qhrr_control_state",
                "publish_hz": 500.0,
            },
            "dashboard_state": {
                "name": "qhrr_dashboard_state",
                "publish_hz": 10.0,
            },
        },
        "state_machine": {
            "enable_duration_s": 0.5,
        },
        "robot_controller": {
            "control_hz": 500.0,
            "shutdown_timeout_s": 2.0,
        },
        "safety": {
            "velocity_damping_kd": 0.5,
        },
        "processes": [],
    }


def test_validate_robot_controller_config_accepts_valid_mapping() -> None:
    validate_robot_controller_config(_valid_robot_controller_config())


def test_validate_robot_controller_config_rejects_bad_bool() -> None:
    config = _valid_robot_controller_config()
    config["can"]["imu"]["enabled"] = "yes"

    with pytest.raises(TypeError, match="can.imu.enabled"):
        validate_robot_controller_config(config)


def test_validate_robot_controller_config_rejects_non_positive_rate() -> None:
    config = _valid_robot_controller_config()
    config["robot_controller"]["control_hz"] = 0

    with pytest.raises(ValueError, match="robot_controller.control_hz"):
        validate_robot_controller_config(config)


class _ActuatorSpec:
    def __init__(self, driver: str) -> None:
        self.driver = driver


class _RobotSpec:
    def __init__(self, drivers: list[str]) -> None:
        self.actuators = [
            _ActuatorSpec(driver)
            for driver in drivers
        ]


def test_validate_supported_actuator_drivers_accepts_spg_mit() -> None:
    validate_supported_actuator_drivers(_RobotSpec(["spg_mit"]))


def test_validate_supported_actuator_drivers_rejects_unknown_driver() -> None:
    with pytest.raises(ValueError, match="Unsupported actuator drivers"):
        validate_supported_actuator_drivers(_RobotSpec(["spg_mit", "unknown"]))


def test_validate_control_mode_fsm_config_accepts_non_negative_duration() -> None:
    validate_control_mode_fsm_config(0.0)
    validate_control_mode_fsm_config(0.5)


def test_validate_control_mode_fsm_config_rejects_negative_duration() -> None:
    with pytest.raises(ValueError, match="enable_duration_s"):
        validate_control_mode_fsm_config(-0.1)


def _valid_process_configs() -> list[dict]:
    return [
        {
            "name": "can_server",
            "command": ["python3", "-m", "module"],
            "start_order": 0,
            "stop_order": 10,
            "new_terminal": False,
            "terminal_command": [],
            "working_dir": ".",
            "env_vars": {},
        },
    ]


def test_validate_process_supervisor_config_accepts_valid_mapping() -> None:
    validate_process_supervisor_config(_valid_process_configs())


def test_validate_process_supervisor_config_rejects_duplicate_name() -> None:
    configs = _valid_process_configs()
    configs.append(dict(configs[0]))

    with pytest.raises(ValueError, match="Duplicate process name"):
        validate_process_supervisor_config(configs)


def test_validate_process_supervisor_config_requires_terminal_command() -> None:
    configs = _valid_process_configs()
    configs[0]["new_terminal"] = True

    with pytest.raises(ValueError, match="requires terminal_command"):
        validate_process_supervisor_config(configs)


def _valid_can_server_config() -> dict:
    return {
        "can": {
            "interface": "vcan0",
            "daemon": {
                "ipc_socket_path": "/tmp/qhrr_can.sock",
                "rx_timeout_s": 0.001,
                "tx_timeout_s": 0.001,
                "join_timeout_s": 1.0,
                "max_tx_queue_size": 4096,
                "send_block": False,
                "send_timeout_s": None,
            },
        },
    }


def test_validate_can_server_config_accepts_valid_mapping() -> None:
    validate_can_server_config(_valid_can_server_config())


def test_validate_can_server_config_rejects_bad_send_block() -> None:
    config = _valid_can_server_config()
    config["can"]["daemon"]["send_block"] = "false"

    with pytest.raises(TypeError, match="send_block"):
        validate_can_server_config(config)


def test_validate_can_server_config_rejects_non_positive_queue_size() -> None:
    config = _valid_can_server_config()
    config["can"]["daemon"]["max_tx_queue_size"] = 0

    with pytest.raises(ValueError, match="max_tx_queue_size"):
        validate_can_server_config(config)


def _valid_shm_config() -> dict:
    return {
        "mit_command": {"name": "qhrr_mit_command"},
        "aux_command": {
            "name": "qhrr_aux_command",
            "size_bytes": 4096,
        },
        "operator_command": {
            "name": "qhrr_operator_command",
            "size_bytes": 4096,
        },
        "control_state": {
            "name": "qhrr_control_state",
            "size_bytes": 16384,
            "publish_hz": 500.0,
        },
        "dashboard_state": {
            "name": "qhrr_dashboard_state",
            "size_bytes": 65536,
            "publish_hz": 10.0,
        },
    }


def test_validate_shm_config_accepts_valid_mapping() -> None:
    validate_shm_config(_valid_shm_config())


def test_validate_shm_config_rejects_empty_name() -> None:
    config = _valid_shm_config()
    config["control_state"]["name"] = ""

    with pytest.raises(ValueError, match="control_state.name"):
        validate_shm_config(config)


def test_validate_shm_config_rejects_non_positive_size() -> None:
    config = _valid_shm_config()
    config["operator_command"]["size_bytes"] = 0

    with pytest.raises(ValueError, match="operator_command.size_bytes"):
        validate_shm_config(config)
