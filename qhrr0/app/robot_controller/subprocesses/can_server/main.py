from __future__ import annotations

import json
import logging
import queue
import select
import signal
import socket
import threading
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO

from ....hal.can import CANBus, CANFrame, CANDaemon, SocketCANBus
from ....helper.config_manage import validate_can_server_config


logger = logging.getLogger(__name__)

_STANDARD_CAN_ID_MAX = 0x7FF
_CLASSIC_CAN_MAX_DATA_LENGTH = 8


@dataclass(frozen=True, slots=True)
class CANServerSettings:
    interface: str
    receive_own_messages: bool
    socket_path: Path

    daemon_rx_timeout_s: float
    daemon_tx_timeout_s: float
    daemon_join_timeout_s: float
    daemon_max_tx_queue_size: int
    daemon_send_block: bool
    daemon_send_timeout_s: float | None

    listen_backlog: int
    accept_timeout_s: float
    max_rx_publish_queue_size: int
    subscriber_send_timeout_s: float
    client_join_timeout_s: float
    max_ipc_line_bytes: int

    @classmethod
    def from_mapping(
        cls,
        config: Mapping[str, Any],
    ) -> CANServerSettings:
        validate_can_server_config(config)

        can = config["can"]
        daemon = can["daemon"]
        server = can.get("server", {})

        # Backward compatibility for the previous schema, where the IPC
        # socket path was incorrectly stored under can.daemon.
        socket_path = server.get(
            "ipc_socket_path",
            daemon.get("ipc_socket_path"),
        )
        assert socket_path is not None

        join_timeout_s = float(daemon["join_timeout_s"])

        return cls(
            interface=str(can["interface"]),
            receive_own_messages=bool(
                can.get("receive_own_messages", False)
            ),
            socket_path=Path(str(socket_path)),
            daemon_rx_timeout_s=float(daemon["rx_timeout_s"]),
            daemon_tx_timeout_s=float(daemon["tx_timeout_s"]),
            daemon_join_timeout_s=join_timeout_s,
            daemon_max_tx_queue_size=int(
                daemon["max_tx_queue_size"]
            ),
            daemon_send_block=bool(daemon["send_block"]),
            daemon_send_timeout_s=(
                None
                if daemon["send_timeout_s"] is None
                else float(daemon["send_timeout_s"])
            ),
            listen_backlog=int(server.get("listen_backlog", 8)),
            accept_timeout_s=float(
                server.get("accept_timeout_s", 0.2)
            ),
            max_rx_publish_queue_size=int(
                server.get("max_rx_publish_queue_size", 4096)
            ),
            subscriber_send_timeout_s=float(
                server.get("subscriber_send_timeout_s", 0.05)
            ),
            client_join_timeout_s=float(
                server.get("client_join_timeout_s", join_timeout_s)
            ),
            max_ipc_line_bytes=int(
                server.get("max_ipc_line_bytes", 4096)
            ),
        )


class RxSubscribers:
    """Thread-safe registry of RX subscriber sockets."""

    def __init__(self, *, send_timeout_s: float) -> None:
        self._send_timeout_s = float(send_timeout_s)
        self._subscribers: set[socket.socket] = set()
        self._lock = threading.Lock()

    def add(self, sock: socket.socket) -> None:
        sock.settimeout(self._send_timeout_s)
        with self._lock:
            self._subscribers.add(sock)

    def remove(self, sock: socket.socket) -> None:
        with self._lock:
            self._subscribers.discard(sock)

    def publish(self, frame: CANFrame) -> None:
        payload = _encode_rx_message(frame)

        with self._lock:
            subscribers = tuple(self._subscribers)

        for sock in subscribers:
            try:
                sock.sendall(payload)
            except OSError:
                logger.info("Removing disconnected CAN RX subscriber")
                self.remove(sock)
                _close_socket(sock)

    def close_all(self) -> None:
        with self._lock:
            subscribers = tuple(self._subscribers)
            self._subscribers.clear()

        for sock in subscribers:
            _close_socket(sock)


class CANServer:
    """
    IPC gateway between RobotController clients and one HAL CAN worker.

    Queue ownership:
    - HAL CANDaemon owns the only TX queue before the physical CAN bus.
    - CANServer owns one bounded RX publish queue so HAL RX is never blocked
      by Unix-socket writes to subscribers.
    """

    def __init__(
        self,
        config: Mapping[str, Any],
        replace_existing_socket: bool = False,
        bus_factory: Callable[[CANServerSettings], CANBus] | None = None,
    ) -> None:
        self.settings = CANServerSettings.from_mapping(config)
        self.replace_existing_socket = bool(replace_existing_socket)
        self._bus_factory = bus_factory or _build_socketcan_bus

        self._stop_event = threading.Event()
        self._shutdown_lock = threading.Lock()
        self._shutdown_started = False
        self._fatal_error: BaseException | None = None

        self._server_sock: socket.socket | None = None
        self._owns_socket_path = False

        self._bus: CANBus | None = None
        self._daemon: CANDaemon | None = None

        self._rx_publish_queue: queue.Queue[CANFrame] = queue.Queue(
            maxsize=self.settings.max_rx_publish_queue_size
        )
        self._publisher_thread: threading.Thread | None = None
        self._subscribers = RxSubscribers(
            send_timeout_s=self.settings.subscriber_send_timeout_s
        )

        self._clients_lock = threading.Lock()
        self._client_sockets: set[socket.socket] = set()
        self._client_threads: set[threading.Thread] = set()

        self._tx_client_lock = threading.Lock()
        self._active_tx_client: socket.socket | None = None

    @property
    def socket_path(self) -> Path:
        return self.settings.socket_path

    def run(self) -> None:
        self._install_signal_handlers()

        try:
            self._start_ipc_server()
            self._start_publisher()
            self._start_can()
            logger.info(
                "CAN server ready: interface=%s socket=%s",
                self.settings.interface,
                self.socket_path,
            )
            self._accept_loop()
        finally:
            self.shutdown()

        if self._fatal_error is not None:
            raise RuntimeError("CAN server stopped after fatal error") from self._fatal_error

    def shutdown(self) -> None:
        with self._shutdown_lock:
            if self._shutdown_started:
                return
            self._shutdown_started = True

        self._stop_event.set()
        self._close_server_socket()

        daemon = self._daemon
        self._daemon = None
        if daemon is not None:
            daemon.unregister_wildcard_callback(
                self._enqueue_received_frame
            )
            daemon.stop(self.settings.daemon_join_timeout_s)

        self._close_all_client_sockets()
        self._subscribers.close_all()
        self._join_publisher_thread()
        self._join_client_threads()

        bus = self._bus
        self._bus = None
        if bus is not None:
            bus.close()

        self._remove_owned_socket_path()

    def _start_can(self) -> None:
        bus = self._bus_factory(self.settings)
        daemon = CANDaemon(
            can_bus=bus,
            rx_timeout=self.settings.daemon_rx_timeout_s,
            tx_timeout=self.settings.daemon_tx_timeout_s,
            join_timeout=self.settings.daemon_join_timeout_s,
            max_tx_queue_size=self.settings.daemon_max_tx_queue_size,
        )
        daemon.register_wildcard_callback(self._enqueue_received_frame)

        try:
            daemon.start()
        except Exception:
            bus.close()
            raise

        self._bus = bus
        self._daemon = daemon

    def _start_ipc_server(self) -> None:
        self._prepare_socket_path()
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)

        server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            server_sock.bind(str(self.socket_path))
            server_sock.listen(self.settings.listen_backlog)
            server_sock.settimeout(self.settings.accept_timeout_s)
        except Exception:
            server_sock.close()
            raise

        self._server_sock = server_sock
        self._owns_socket_path = True

    def _start_publisher(self) -> None:
        thread = threading.Thread(
            target=self._publisher_loop,
            name=f"CAN_SERVER_{self.settings.interface}_RX_PUBLISH",
            daemon=True,
        )
        self._publisher_thread = thread
        thread.start()

    def _accept_loop(self) -> None:
        while not self._stop_event.is_set():
            server_sock = self._server_sock
            if server_sock is None:
                return

            try:
                client_sock, _ = server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                if not self._stop_event.is_set():
                    logger.exception("CAN server accept failed")
                return

            self._start_client_thread(client_sock)

    def _start_client_thread(self, client_sock: socket.socket) -> None:
        thread = threading.Thread(
            target=self._client_thread_main,
            args=(client_sock,),
            name=f"CAN_SERVER_CLIENT_{client_sock.fileno()}",
            daemon=True,
        )

        with self._clients_lock:
            self._client_sockets.add(client_sock)
            self._client_threads.add(thread)

        try:
            thread.start()
        except Exception:
            with self._clients_lock:
                self._client_sockets.discard(client_sock)
                self._client_threads.discard(thread)
            _close_socket(client_sock)
            raise

    def _client_thread_main(self, client_sock: socket.socket) -> None:
        current_thread = threading.current_thread()
        try:
            self._handle_client(client_sock)
        finally:
            self._subscribers.remove(client_sock)
            self._release_tx_client(client_sock)
            _close_socket(client_sock)
            with self._clients_lock:
                self._client_sockets.discard(client_sock)
                self._client_threads.discard(current_thread)

    def _handle_client(self, client_sock: socket.socket) -> None:
        file_obj = client_sock.makefile("rwb")
        try:
            hello = self._read_json_line(file_obj)
            if hello.get("type") != "hello":
                self._write_json_line(
                    file_obj,
                    {
                        "type": "hello_ack",
                        "ok": False,
                        "error": "client must send hello first",
                    },
                )
                return

            role = hello.get("role")
            if role == "tx":
                if not self._claim_tx_client(client_sock):
                    self._write_json_line(
                        file_obj,
                        {
                            "type": "hello_ack",
                            "ok": False,
                            "error": "a TX client is already connected",
                        },
                    )
                    return

                self._write_json_line(
                    file_obj,
                    {"type": "hello_ack", "ok": True},
                )
                self._handle_tx_client(file_obj)
                return

            if role == "rx":
                self._write_json_line(
                    file_obj,
                    {"type": "hello_ack", "ok": True},
                )
                self._subscribers.add(client_sock)
                self._wait_for_rx_client_disconnect(client_sock)
                return

            self._write_json_line(
                file_obj,
                {
                    "type": "hello_ack",
                    "ok": False,
                    "error": f"unknown client role: {role!r}",
                },
            )
        except (OSError, ValueError, RuntimeError, json.JSONDecodeError):
            if not self._stop_event.is_set():
                logger.exception("CAN server client handler failed")
        finally:
            try:
                file_obj.close()
            except OSError:
                pass

    def _handle_tx_client(self, file_obj: BinaryIO) -> None:
        while not self._stop_event.is_set():
            try:
                message = self._read_json_line(file_obj)
            except OSError:
                return

            if message.get("type") != "tx":
                self._write_json_line(
                    file_obj,
                    {
                        "type": "tx_result",
                        "ok": False,
                        "error": "expected tx message",
                    },
                )
                continue

            try:
                frame = _decode_tx_frame(message)
                daemon = self._daemon
                if daemon is None:
                    raise RuntimeError("CAN worker is not available")

                accepted = daemon.send(
                    frame,
                    block=self.settings.daemon_send_block,
                    timeout=self.settings.daemon_send_timeout_s,
                )
                response = {
                    "type": "tx_result",
                    "ok": accepted,
                }
                if not accepted:
                    response["error"] = "HAL CAN TX queue is full"
                self._write_json_line(file_obj, response)
            except Exception as exc:
                self._write_json_line(
                    file_obj,
                    {
                        "type": "tx_result",
                        "ok": False,
                        "error": str(exc),
                    },
                )

    def _wait_for_rx_client_disconnect(
        self,
        client_sock: socket.socket,
    ) -> None:
        while not self._stop_event.is_set():
            try:
                readable, _, _ = select.select(
                    [client_sock],
                    [],
                    [],
                    self.settings.accept_timeout_s,
                )
            except (OSError, ValueError):
                return

            if not readable:
                continue

            try:
                peeked = client_sock.recv(1, socket.MSG_PEEK)
            except OSError:
                return

            if peeked == b"":
                return

            # RX subscribers must not send application data after hello.
            logger.warning("RX subscriber sent unexpected data; disconnecting")
            return

    def _enqueue_received_frame(self, frame: CANFrame) -> None:
        try:
            self._rx_publish_queue.put_nowait(frame)
        except queue.Full:
            error = RuntimeError(
                "CAN server RX publish queue overflow; stopping server"
            )
            self._fatal_error = error
            logger.critical(
                "%s: interface=%s queue_size=%d",
                error,
                self.settings.interface,
                self.settings.max_rx_publish_queue_size,
            )
            self._stop_event.set()

    def _publisher_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                frame = self._rx_publish_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                self._subscribers.publish(frame)
            finally:
                self._rx_publish_queue.task_done()

    def _claim_tx_client(self, client_sock: socket.socket) -> bool:
        with self._tx_client_lock:
            if self._active_tx_client is not None:
                return False
            self._active_tx_client = client_sock
            return True

    def _release_tx_client(self, client_sock: socket.socket) -> None:
        with self._tx_client_lock:
            if self._active_tx_client is client_sock:
                self._active_tx_client = None

    def _prepare_socket_path(self) -> None:
        if not self.socket_path.exists():
            return

        if not self.socket_path.is_socket():
            raise RuntimeError(
                "CAN server IPC path exists and is not a Unix socket: "
                f"{self.socket_path}"
            )

        if _unix_socket_is_accepting(self.socket_path):
            raise RuntimeError(
                "Another CAN server is already listening on: "
                f"{self.socket_path}"
            )

        if not self.replace_existing_socket:
            raise RuntimeError(
                "Stale CAN server IPC socket exists: "
                f"{self.socket_path}"
            )

        logger.warning(
            "Removing stale CAN server IPC socket: %s",
            self.socket_path,
        )
        self.socket_path.unlink()

    def _close_server_socket(self) -> None:
        server_sock = self._server_sock
        self._server_sock = None
        if server_sock is not None:
            _close_socket(server_sock)

    def _close_all_client_sockets(self) -> None:
        with self._clients_lock:
            sockets = tuple(self._client_sockets)

        for client_sock in sockets:
            _close_socket(client_sock)

    def _join_client_threads(self) -> None:
        current = threading.current_thread()
        with self._clients_lock:
            threads = tuple(self._client_threads)

        for thread in threads:
            if thread is current:
                continue
            thread.join(timeout=self.settings.client_join_timeout_s)
            if thread.is_alive():
                logger.error(
                    "CAN server client thread did not stop: %s",
                    thread.name,
                )

    def _join_publisher_thread(self) -> None:
        thread = self._publisher_thread
        self._publisher_thread = None
        if thread is None or thread is threading.current_thread():
            return

        thread.join(timeout=self.settings.client_join_timeout_s)
        if thread.is_alive():
            logger.error("CAN RX publisher thread did not stop")

    def _remove_owned_socket_path(self) -> None:
        if not self._owns_socket_path:
            return
        self._owns_socket_path = False

        try:
            if self.socket_path.exists() and self.socket_path.is_socket():
                self.socket_path.unlink()
        except OSError:
            logger.exception(
                "Failed to remove CAN server socket: %s",
                self.socket_path,
            )

    def _install_signal_handlers(self) -> None:
        if threading.current_thread() is not threading.main_thread():
            return

        def handle_signal(_signum: int, _frame: object) -> None:
            self._stop_event.set()

        signal.signal(signal.SIGINT, handle_signal)
        signal.signal(signal.SIGTERM, handle_signal)

    def _write_json_line(
        self,
        file_obj: BinaryIO,
        message: Mapping[str, Any],
    ) -> None:
        payload = json.dumps(
            dict(message),
            separators=(",", ":"),
        ).encode("utf-8") + b"\n"

        if len(payload) > self.settings.max_ipc_line_bytes:
            raise ValueError("CAN server response exceeds IPC line limit")

        file_obj.write(payload)
        file_obj.flush()

    def _read_json_line(self, file_obj: BinaryIO) -> dict[str, Any]:
        line = file_obj.readline(self.settings.max_ipc_line_bytes + 1)
        if not line:
            raise OSError("CAN server client disconnected")
        if len(line) > self.settings.max_ipc_line_bytes:
            raise ValueError("CAN server IPC message is too large")
        if not line.endswith(b"\n"):
            raise ValueError("CAN server IPC message is not newline terminated")

        message = json.loads(line.decode("utf-8"))
        if not isinstance(message, dict):
            raise RuntimeError("CAN server message must be a JSON object")
        return message


def _build_socketcan_bus(settings: CANServerSettings) -> CANBus:
    return SocketCANBus(
        settings.interface,
        receive_own_messages=settings.receive_own_messages,
    )


def _encode_rx_message(frame: CANFrame) -> bytes:
    return json.dumps(
        {
            "type": "rx",
            "can_id": int(frame.can_id),
            "data": bytes(frame.data).hex(),
        },
        separators=(",", ":"),
    ).encode("utf-8") + b"\n"


def _decode_tx_frame(message: Mapping[str, Any]) -> CANFrame:
    can_id = int(message["can_id"])
    if not 0 <= can_id <= _STANDARD_CAN_ID_MAX:
        raise ValueError(
            f"Only standard 11-bit CAN IDs are supported: 0x{can_id:X}"
        )

    data = bytes.fromhex(str(message["data"]))
    if len(data) > _CLASSIC_CAN_MAX_DATA_LENGTH:
        raise ValueError(
            "Classic CAN payload must be <= 8 bytes, "
            f"got {len(data)}"
        )

    return CANFrame(can_id=can_id, data=data)


def _unix_socket_is_accepting(path: Path) -> bool:
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    probe.settimeout(0.1)
    try:
        probe.connect(str(path))
        return True
    except OSError:
        return False
    finally:
        probe.close()


def _close_socket(sock: socket.socket) -> None:
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    try:
        sock.close()
    except OSError:
        pass