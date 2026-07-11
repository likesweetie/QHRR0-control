from pathlib import Path
import sys

import pytest

from qhrr0.app.robot_controller.subprocesses.can_server import main as can_server_main
from qhrr0.app.robot_controller.subprocesses.can_server.main import load_can_server_config


def _write_config(path: Path, text: str) -> Path:
    path.write_text(text, encoding="utf-8")
    return path


def test_load_can_server_config_accepts_valid_yaml(tmp_path: Path) -> None:
    config_path = _write_config(
        tmp_path / "can_server.yaml",
        "\n".join(
            [
                "can:",
                "  interface: vcan0",
                "  daemon:",
                "    ipc_socket_path: /tmp/qhrr_can.sock",
                "    rx_timeout_s: 0.001",
                "    tx_timeout_s: 0.001",
                "    join_timeout_s: 1.0",
                "    max_tx_queue_size: 4096",
                "    send_block: false",
                "    send_timeout_s: null",
            ]
        ),
    )

    config = load_can_server_config(config_path)

    assert config["can"]["interface"] == "vcan0"
    assert config["can"]["daemon"]["ipc_socket_path"] == "/tmp/qhrr_can.sock"
    assert config["can"]["daemon"]["max_tx_queue_size"] == 4096


def test_load_can_server_config_rejects_missing_socket_path(tmp_path: Path) -> None:
    config_path = _write_config(
        tmp_path / "can_server.yaml",
        "\n".join(
            [
                "can:",
                "  interface: vcan0",
                "  daemon:",
                "    rx_timeout_s: 0.001",
                "    tx_timeout_s: 0.001",
                "    join_timeout_s: 1.0",
                "    max_tx_queue_size: 4096",
                "    send_block: false",
                "    send_timeout_s: null",
            ]
        ),
    )

    with pytest.raises(KeyError, match="ipc_socket_path"):
        load_can_server_config(config_path)


def test_main_rejects_unknown_cli_arguments(monkeypatch) -> None:
    monkeypatch.setattr(sys, "argv", ["can_server", "--config-key", "robot_controller"])

    with pytest.raises(SystemExit):
        can_server_main.main()
