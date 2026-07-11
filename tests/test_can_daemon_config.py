from pathlib import Path

import pytest

from qhrr0.app.robot_controller.subprocesses.can_daemon import main as can_daemon_main
from qhrr0.app.robot_controller.subprocesses.can_daemon.main import load_can_daemon_config


def _write_config(path: Path, text: str) -> Path:
    path.write_text(text, encoding="utf-8")
    return path


def test_load_can_daemon_config_composes_controller_and_daemon_yaml(tmp_path: Path) -> None:
    controller_config = _write_config(
        tmp_path / "robot_controller.yaml",
        "\n".join(
            [
                "can:",
                "  interfaces:",
                "    - vcan0",
                "  daemon_socket: /tmp/qhrr_can.sock",
            ]
        ),
    )
    daemon_config = _write_config(
        tmp_path / "can_daemon.yaml",
        "\n".join(
            [
                "daemon:",
                "  rx_timeout_s: 0.001",
                "  tx_timeout_s: 0.001",
                "  join_timeout_s: 1.0",
                "  max_tx_queue_size: 4096",
                "  send_block: false",
                "  send_timeout_s: null",
            ]
        ),
    )

    config = load_can_daemon_config(controller_config, daemon_config)

    assert config["can"]["interface"] == "vcan0"
    assert config["can"]["daemon"]["ipc_socket_path"] == "/tmp/qhrr_can.sock"
    assert config["can"]["daemon"]["max_tx_queue_size"] == 4096


def test_load_can_daemon_config_rejects_ambiguous_interfaces(tmp_path: Path) -> None:
    controller_config = _write_config(
        tmp_path / "robot_controller.yaml",
        "\n".join(
            [
                "can:",
                "  interfaces:",
                "    - vcan0",
                "    - can0",
                "  daemon_socket: /tmp/qhrr_can.sock",
            ]
        ),
    )
    daemon_config = _write_config(
        tmp_path / "can_daemon.yaml",
        "\n".join(
            [
                "daemon:",
                "  rx_timeout_s: 0.001",
                "  tx_timeout_s: 0.001",
                "  join_timeout_s: 1.0",
                "  max_tx_queue_size: 4096",
                "  send_block: false",
                "  send_timeout_s: null",
            ]
        ),
    )

    with pytest.raises(ValueError, match="exactly one CAN interface"):
        load_can_daemon_config(controller_config, daemon_config)


def test_main_rejects_cli_arguments(monkeypatch) -> None:
    monkeypatch.setattr(can_daemon_main.sys, "argv", ["can_daemon", "--config-key", "robot_controller"])

    with pytest.raises(SystemExit, match="does not accept CLI arguments"):
        can_daemon_main.main()
