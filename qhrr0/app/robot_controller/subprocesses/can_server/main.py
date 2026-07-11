from __future__ import annotations

import argparse
import json
import logging
import signal
import socket
import threading
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from ....hal.can import CANFrame, CANDaemon as HALCANDaemon, SocketCANBus
from ....helper.config_manage import YAMLconfigLoader, validate_can_server_config


logger = logging.getLogger(__name__)
CONFIG_ROOT = Path(__file__).resolve().parents[4] / "config"
CONFIG_PATHS_PATH = CONFIG_ROOT / "config_paths.yaml"
CONTROLLER_CONFIG_KEY = "robot_controller"
CAN_SERVER_CONFIG_KEY = "can_server"
REPLACE_EXISTING_SOCKET = True


class RxSubscribers:
    def __init__(self) -> None:
        self._subscribers: set[socket.socket] = set()
        self._lock = threading.Lock()

    def add(self, sock: socket.socket) -> None:
        with self._lock:
            self._subscribers.add(sock)

    def remove(self, sock: socket.socket) -> None:
        with self._lock:
            self._subscribers.discard(sock)

    def publish(self, frame: CANFrame) -> None:
        payload = json.dumps(
            {
                "type": "rx",
                "can_id": int(frame.can_id),
                "data": bytes(frame.data).hex(),
            },
            separators=(",", ":"),
        ).encode("utf-8") + b"\n"

        with self._lock:
            subscribers = list(self._subscribers)

        for sock in subscribers:
            try:
                sock.sendall(payload)
            except OSError:
                self.remove(sock)
                sock.close()


class CANServer:
    def __init__(
        self,
        config: Mapping[str, Any],
        replace_existing_socket: bool = False,
    ) -> None:
        validate_can_server_config(config)
        self.config = config
        self.socket_path = Path(self.config["can"]["daemon"]["ipc_socket_path"])
        self.replace_existing_socket = replace_existing_socket
        self.subscribers = RxSubscribers()
        self._stop_event = threading.Event()
        self._server_sock: socket.socket | None = None
        self._bus: SocketCANBus | None = None
        self._daemon: HALCANDaemon | None = None
        self._client_threads: list[threading.Thread] = []

    def run(self) -> None:
        self._install_signal_handlers()
        self._start_can()
        self._start_server()
        logger.info("CAN server subprocess ready: %s", self.socket_path)
        while not self._stop_event.is_set():
            assert self._server_sock is not None
            try:
                client_sock, _addr = self._server_sock.accept()
            except TimeoutError:
                continue
            except OSError:
                if not self._stop_event.is_set():
                    logger.exception("CAN server accept failed")
                break
            thread = threading.Thread(
                target=self._handle_client,
                args=(client_sock,),
                daemon=True,
            )
            thread.start()
            self._client_threads.append(thread)
        self.shutdown()

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._server_sock is not None:
            self._server_sock.close()
            self._server_sock = None
        if self._daemon is not None:
            self._daemon.stop(float(self.config["can"]["daemon"]["join_timeout_s"]))
            self._daemon = None
        if self._bus is not None:
            self._bus.close()
            self._bus = None
        if self.socket_path.exists():
            self.socket_path.unlink()

    def _start_can(self) -> None:
        self._bus = SocketCANBus(self.config["can"]["interface"])
        self._disable_recv_own_messages(self._bus)
        daemon_config = self.config["can"]["daemon"]
        self._daemon = HALCANDaemon(
            can_bus=self._bus,
            rx_timeout=float(daemon_config["rx_timeout_s"]),
            tx_timeout=float(daemon_config["tx_timeout_s"]),
            join_timeout=float(daemon_config["join_timeout_s"]),
            max_tx_queue_size=int(daemon_config["max_tx_queue_size"]),
        )
        self._daemon.register_wildcard_callback(self.subscribers.publish)
        self._daemon.start()

    def _start_server(self) -> None:
        if self.socket_path.exists():
            if not self.replace_existing_socket:
                raise RuntimeError(f"CAN server IPC socket already exists: {self.socket_path}")
            logger.warning("Replacing existing CAN server IPC socket: %s", self.socket_path)
            self.socket_path.unlink()
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        self._server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._server_sock.bind(str(self.socket_path))
        self._server_sock.listen(8)
        self._server_sock.settimeout(0.2)

    def _handle_client(self, client_sock: socket.socket) -> None:
        file_obj = client_sock.makefile("rwb")
        try:
            hello = self._read_json_line(file_obj)
            if hello.get("type") != "hello":
                raise RuntimeError("Client must send hello first")
            role = hello.get("role")
            self._write_json_line(file_obj, {"type": "hello_ack", "ok": True})
            if role == "tx":
                self._handle_tx_client(file_obj)
                return
            if role == "rx":
                self.subscribers.add(client_sock)
                client_sock.settimeout(0.2)
                while not self._stop_event.is_set():
                    try:
                        if client_sock.recv(1, socket.MSG_PEEK) == b"":
                            return
                    except TimeoutError:
                        continue
                    except OSError:
                        return
                return
            raise RuntimeError(f"Unknown CAN server client role: {role}")
        except Exception:
            logger.exception("CAN server client handler failed")
        finally:
            self.subscribers.remove(client_sock)
            file_obj.close()
            client_sock.close()

    def _handle_tx_client(self, file_obj) -> None:
        while not self._stop_event.is_set():
            message = self._read_json_line(file_obj)
            if message.get("type") != "tx":
                self._write_json_line(file_obj, {"type": "tx_result", "ok": False, "error": "expected tx"})
                continue
            try:
                frame = CANFrame(
                    can_id=int(message["can_id"]),
                    data=bytes.fromhex(str(message["data"])),
                )
                assert self._daemon is not None
                ok = self._daemon.send(
                    frame,
                    block=bool(self.config["can"]["daemon"]["send_block"]),
                    timeout=self.config["can"]["daemon"]["send_timeout_s"],
                )
                self._write_json_line(file_obj, {"type": "tx_result", "ok": bool(ok)})
            except Exception as exc:
                self._write_json_line(file_obj, {"type": "tx_result", "ok": False, "error": str(exc)})

    def _install_signal_handlers(self) -> None:
        def handle_signal(_signum: int, _frame: object) -> None:
            self._stop_event.set()

        signal.signal(signal.SIGINT, handle_signal)
        signal.signal(signal.SIGTERM, handle_signal)

    @staticmethod
    def _disable_recv_own_messages(can_bus: SocketCANBus) -> None:
        raw_socket = getattr(can_bus, "_socket", None)
        if raw_socket is None:
            raise RuntimeError("HAL SocketCANBus does not expose its raw socket")
        if not hasattr(socket, "SOL_CAN_RAW") or not hasattr(socket, "CAN_RAW_RECV_OWN_MSGS"):
            raise RuntimeError("Python socket module does not expose CAN_RAW_RECV_OWN_MSGS")
        raw_socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)

    @staticmethod
    def _write_json_line(file_obj, message: dict) -> None:
        file_obj.write(json.dumps(message, separators=(",", ":")).encode("utf-8") + b"\n")
        file_obj.flush()

    @staticmethod
    def _read_json_line(file_obj) -> dict:
        line = file_obj.readline()
        if not line:
            raise OSError("CAN server client disconnected")
        message = json.loads(line.decode("utf-8"))
        if not isinstance(message, dict):
            raise RuntimeError("CAN server message must be a JSON object")
        return message


def app_config_path(config_key: str) -> Path:
    config_paths = YAMLconfigLoader(CONFIG_PATHS_PATH).load()
    try:
        relative_path = config_paths["app"][config_key]
    except KeyError:
        raise KeyError(f"Unknown app config key: {config_key}") from None
    return CONFIG_ROOT.parent / str(relative_path)


def load_can_server_config(path: Path | str) -> Mapping[str, Any]:
    return YAMLconfigLoader(path, validator=validate_can_server_config).load()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one QHRR CAN server subprocess.")
    parser.add_argument(
        "--config",
        type=Path,
        default=app_config_path(CAN_SERVER_CONFIG_KEY),
        help="Path to can_server YAML config.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_can_server_config(args.config)
    CANServer(config, replace_existing_socket=REPLACE_EXISTING_SOCKET).run()


if __name__ == "__main__":
    main()
