from __future__ import annotations

import logging
import time
from collections import deque
from typing import Callable

from robot_controller.core.config import RobotControllerConfig
from robot_controller.utils.can_daemon_client import CANProcessClient
from robot_controller.utils.hal_can_bus import CANFrame


CAN_SFF_MASK = 0x7FF
TX_ECHO_REJECT_WINDOW_S = 0.25

logger = logging.getLogger(__name__)


class CanTransport:
    def __init__(self, config: RobotControllerConfig):
        self.config = config
        self.client: CANProcessClient | None = None
        self.recent_tx_frames: deque[tuple[float, int, bytes]] = deque(maxlen=512)

    def connect(self) -> None:
        if self.client is not None:
            return
        daemon_config = self.config.can.daemon
        self.client = CANProcessClient(
            socket_path=daemon_config.ipc_socket_path,
            connect_timeout_s=daemon_config.connect_timeout_s,
        )
        self.client.connect()

    def close(self) -> None:
        if self.client is not None:
            self.client.close()
            self.client = None

    def is_connected(self) -> bool:
        return self.client is not None

    def register_callback(self, can_id: int, callback: Callable[[CANFrame], None]) -> None:
        if self.client is None:
            raise RuntimeError("CAN transport must be connected before callback registration")
        self.client.register_callback(can_id, callback)

    def registered_ids(self) -> list[int]:
        if self.client is None:
            return []
        return sorted(self.client.dispatcher.registered_ids())

    def send_frame(self, frame: CANFrame) -> None:
        if frame.can_id < 0 or frame.can_id > CAN_SFF_MASK:
            raise ValueError(f"Only standard 11-bit CAN IDs are supported: 0x{frame.can_id:X}")
        self.connect()
        assert self.client is not None
        ok = self.client.send(
            frame,
            block=self.config.can.daemon.send_block,
            timeout=self.config.can.daemon.send_timeout_s,
        )
        if not ok:
            raise RuntimeError(f"CAN daemon TX queue is full: can_id=0x{frame.can_id:X}")
        self.remember_tx_frame(frame)

    def remember_tx_frame(self, frame: CANFrame) -> None:
        self.recent_tx_frames.append((time.monotonic(), int(frame.can_id), bytes(frame.data)))

    def is_recent_tx_echo(self, frame: CANFrame) -> bool:
        now = time.monotonic()
        while self.recent_tx_frames and now - self.recent_tx_frames[0][0] > TX_ECHO_REJECT_WINDOW_S:
            self.recent_tx_frames.popleft()

        can_id = int(frame.can_id)
        data = bytes(frame.data)
        return any(
            tx_can_id == can_id and tx_data == data
            for _tx_t, tx_can_id, tx_data in self.recent_tx_frames
        )
