from qhrr0.app.robot_controller.shm.manager import ShmManager


def _shm_config() -> dict:
    return {
        "mit_command": {"name": "test_mit_command"},
        "aux_command": {
            "name": "test_aux_command",
            "size_bytes": 4096,
        },
        "operator_command": {
            "name": "test_operator_command",
            "size_bytes": 4096,
        },
        "control_state": {
            "name": "test_control_state",
            "size_bytes": 16384,
            "publish_hz": 500.0,
        },
        "dashboard_state": {
            "name": "test_dashboard_state",
            "size_bytes": 65536,
            "publish_hz": 10.0,
        },
    }


def test_shm_manager_uses_dict_config_for_segment_names() -> None:
    manager = ShmManager(_shm_config())

    assert manager._segment_names() == (
        "test_mit_command",
        "test_aux_command",
        "test_operator_command",
        "test_control_state",
        "test_dashboard_state",
    )
