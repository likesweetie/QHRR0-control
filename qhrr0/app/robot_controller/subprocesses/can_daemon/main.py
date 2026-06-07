from __future__ import annotations

import argparse
import json
import logging
import signal
import socket
import threading
from pathlib import Path

from qhrr0.app.hal.can_bus import CANFrame, CANDaemon, SocketCANBus


logger = logging.getLogger(__name__)


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


class CANSubprocessDaemon:
    def __init__(
        self,
        *,
        can_interface: str,
        ipc_socket_path: str,
        rx_timeout_s: float,
        tx_timeout_s: float,
        join_timeout_s: float,
        max_tx_queue_size: int,
        send_block: bool,
        send_timeout_s: float | None,
        replace_existing_socket: bool,
    ) -> None:
        self.can_interface = str(can_interface)
        self.rx_timeout_s = float(rx_timeout_s)
        self.tx_timeout_s = float(tx_timeout_s)
        self.join_timeout_s = float(join_timeout_s)
        self.max_tx_queue_size = int(max_tx_queue_size)
        self.send_block = bool(send_block)
        self.send_timeout_s = None if send_timeout_s is None else float(send_timeout_s)
        self.socket_path = Path(ipc_socket_path)
        self.replace_existing_socket = replace_existing_socket
        self.subscribers = RxSubscribers()
        self._stop_event = threading.Event()
        self._server_sock: socket.socket | None = None
        self._can_bus: SocketCANBus | None = None
        self._can_daemon: CANDaemon | None = None
        self._client_threads: list[threading.Thread] = []

    def run(self) -> None:
        self._install_signal_handlers()
        self._start_can()
        self._start_server()
        logger.info("CAN daemon subprocess ready: %s", self.socket_path)
        while not self._stop_event.is_set():
            assert self._server_sock is not None
            try:
                client_sock, _addr = self._server_sock.accept()
            except TimeoutError:
                continue
            except OSError:
                if not self._stop_event.is_set():
                    logger.exception("CAN daemon accept failed")
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
        if self._can_daemon is not None:
            self._can_daemon.stop(self.join_timeout_s)
            self._can_daemon = None
        if self._can_bus is not None:
            self._can_bus.close()
            self._can_bus = None
        if self.socket_path.exists():
            self.socket_path.unlink()

    def _start_can(self) -> None:
        self._can_bus = SocketCANBus(self.can_interface)
        self._disable_recv_own_messages(self._can_bus)
        self._can_daemon = CANDaemon(
            can_bus=self._can_bus,
            rx_timeout=self.rx_timeout_s,
            tx_timeout=self.tx_timeout_s,
            join_timeout=self.join_timeout_s,
            max_tx_queue_size=self.max_tx_queue_size,
        )
        self._can_daemon.register_wildcard_callback(self.subscribers.publish)
        self._can_daemon.start()

    def _start_server(self) -> None:
        if self.socket_path.exists():
            if not self.replace_existing_socket:
                raise RuntimeError(f"CAN daemon IPC socket already exists: {self.socket_path}")
            logger.warning("Replacing existing CAN daemon IPC socket: %s", self.socket_path)
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
            raise RuntimeError(f"Unknown CAN daemon client role: {role}")
        except Exception:
            logger.exception("CAN daemon client handler failed")
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
                assert self._can_daemon is not None
                ok = self._can_daemon.send(
                    frame,
                    block=self.send_block,
                    timeout=self.send_timeout_s,
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
            raise OSError("CAN daemon client disconnected")
        message = json.loads(line.decode("utf-8"))
        if not isinstance(message, dict):
            raise RuntimeError("CAN daemon message must be a JSON object")
        return message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="QHRR SocketCAN daemon subprocess")
    parser.add_argument("--can-interface", required=True)
    parser.add_argument("--ipc-socket-path", required=True)
    parser.add_argument("--rx-timeout-s", type=float, required=True)
    parser.add_argument("--tx-timeout-s", type=float, required=True)
    parser.add_argument("--join-timeout-s", type=float, required=True)
    parser.add_argument("--max-tx-queue-size", type=int, required=True)
    parser.add_argument("--send-block", action="store_true")
    parser.add_argument("--send-timeout-s", type=float, default=None)
    parser.add_argument("--replace-existing-socket", action="store_true")
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper()),
        format="[%(levelname)s] %(name)s: %(message)s",
    )
    daemon = CANSubprocessDaemon(
        can_interface=args.can_interface,
        ipc_socket_path=args.ipc_socket_path,
        rx_timeout_s=args.rx_timeout_s,
        tx_timeout_s=args.tx_timeout_s,
        join_timeout_s=args.join_timeout_s,
        max_tx_queue_size=args.max_tx_queue_size,
        send_block=bool(args.send_block),
        send_timeout_s=args.send_timeout_s,
        replace_existing_socket=bool(args.replace_existing_socket),
    )
    daemon.run()


if __name__ == "__main__":
    main()
