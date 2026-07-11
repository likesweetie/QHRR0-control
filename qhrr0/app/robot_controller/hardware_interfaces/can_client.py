from __future__ import annotations

import json
import logging
import socket
import threading
import time
from collections import deque
from collections.abc import Callable

from ..hal.can.dispatcher import CANDispatcher
from ..hal.can.frame import CANFrame


CAN_SFF_MASK = 0x7FF
TX_ECHO_REJECT_WINDOW_S = 0.25

logger = logging.getLogger(__name__)


class CANClient:
    """IPC client for one robot-controller CAN server."""

    def __init__(
        self,
        *,
        name: str,
        socket_path: str,
        connect_timeout_s: float,
        rx_enabled: bool = True,
    ) -> None:
        self.name = str(name)
        self.socket_path = str(socket_path)
        self.connect_timeout_s = float(connect_timeout_s)
        self.rx_enabled = bool(rx_enabled)
        self.dispatcher = CANDispatcher()
        self._tx_sock: socket.socket | None = None
        self._rx_sock: socket.socket | None = None
        self._tx_file = None
        self._rx_file = None
        self._tx_lock = threading.Lock()
        self._rx_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._recent_tx_frames: deque[tuple[float, int, bytes]] = deque(maxlen=512)

    def connect(self) -> None:
        if self.is_connected():
            return

        deadline = time.monotonic() + self.connect_timeout_s
        while True:
            try:
                self._tx_sock, self._tx_file = self._connect_role("tx")
                if self.rx_enabled:
                    self._rx_sock, self._rx_file = self._connect_role("rx")
                    self._rx_thread = threading.Thread(
                        target=self._rx_loop,
                        name=f"CAN_CLIENT_{self.name}_RX",
                        daemon=True,
                    )
                    self._rx_thread.start()
                return
            except OSError:
                self.close()
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)

    def close(self) -> None:
        self._stop_event.set()
        if self._tx_file is not None:
            self._tx_file.close()
            self._tx_file = None
        if self._rx_file is not None:
            self._rx_file.close()
            self._rx_file = None
        for sock in (self._tx_sock, self._rx_sock):
            if sock is not None:
                sock.close()
        self._tx_sock = None
        self._rx_sock = None
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        self._recent_tx_frames.clear()
        self._stop_event.clear()

    def is_connected(self) -> bool:
        return self._tx_file is not None

    def register_callback(self, can_id: int, callback: Callable[[CANFrame], None]) -> None:
        self.dispatcher.register(can_id, callback)

    def register_wildcard_callback(self, callback: Callable[[CANFrame], None]) -> None:
        self.dispatcher.register_wildcard(callback)

    def send_frame(self, frame: CANFrame) -> None:
        if frame.can_id < 0 or frame.can_id > CAN_SFF_MASK:
            raise ValueError(f"Only standard 11-bit CAN IDs are supported: 0x{frame.can_id:X}")
        if self._tx_file is None:
            raise RuntimeError(f"CAN client {self.name} is not connected")

        request = {
            "type": "tx",
            "can_id": int(frame.can_id),
            "data": bytes(frame.data).hex(),
        }
        with self._tx_lock:
            self._write_json_line(self._tx_file, request)
            response = self._read_json_line(self._tx_file)
        if response.get("type") != "tx_result":
            raise RuntimeError(f"Unexpected CAN server response from {self.name}: {response}")
        if not bool(response.get("ok")):
            raise RuntimeError(str(response.get("error") or f"CAN server {self.name} TX failed"))

        self._remember_tx_frame(frame)

    def send(self, frame: CANFrame) -> bool:
        self.send_frame(frame)
        return True

    def is_recent_tx_echo(self, frame: CANFrame) -> bool:
        now = time.monotonic()
        while self._recent_tx_frames and now - self._recent_tx_frames[0][0] > TX_ECHO_REJECT_WINDOW_S:
            self._recent_tx_frames.popleft()

        can_id = int(frame.can_id)
        data = bytes(frame.data)
        return any(
            tx_can_id == can_id and tx_data == data
            for _tx_t, tx_can_id, tx_data in self._recent_tx_frames
        )

    def _connect_role(self, role: str):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(self.socket_path)
        file_obj = sock.makefile("rwb")
        self._write_json_line(file_obj, {"type": "hello", "role": role})
        response = self._read_json_line(file_obj)
        if response.get("type") != "hello_ack" or not bool(response.get("ok")):
            raise RuntimeError(f"CAN server {self.name} rejected role {role}: {response}")
        return sock, file_obj

    def _rx_loop(self) -> None:
        file_obj = self._rx_file
        assert file_obj is not None
        while not self._stop_event.is_set():
            try:
                message = self._read_json_line(file_obj)
            except OSError:
                if not self._stop_event.is_set():
                    logger.exception("CAN server RX socket failed: %s", self.name)
                return
            if message.get("type") != "rx":
                continue
            frame = CANFrame(
                can_id=int(message["can_id"]),
                data=bytes.fromhex(str(message["data"])),
            )
            self.dispatcher.dispatch(frame)

    def _remember_tx_frame(self, frame: CANFrame) -> None:
        self._recent_tx_frames.append((time.monotonic(), int(frame.can_id), bytes(frame.data)))

    @staticmethod
    def _write_json_line(file_obj, message: dict) -> None:
        file_obj.write(json.dumps(message, separators=(",", ":")).encode("utf-8") + b"\n")
        file_obj.flush()

    @staticmethod
    def _read_json_line(file_obj) -> dict:
        line = file_obj.readline()
        if not line:
            raise OSError("CAN server socket closed")
        message = json.loads(line.decode("utf-8"))
        if not isinstance(message, dict):
            raise RuntimeError("CAN server message must be a JSON object")
        return message
