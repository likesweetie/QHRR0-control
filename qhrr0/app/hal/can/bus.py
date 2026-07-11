from __future__ import annotations

import socket
import struct
from abc import ABC, abstractmethod

from .frame import CANFrame


CLASSIC_CAN_FRAME_FMT = "=IB3x8s"
CLASSIC_CAN_FRAME_SIZE = struct.calcsize(CLASSIC_CAN_FRAME_FMT)
STANDARD_CAN_ID_MAX = 0x7FF
CLASSIC_CAN_MAX_DATA_LENGTH = 8


class CANBus(ABC):
    """Abstract CAN bus interface."""

    @abstractmethod
    def send_frame(self, frame: CANFrame) -> None:
        raise NotImplementedError

    @abstractmethod
    def recv_frame(self, timeout: float = 0.0) -> CANFrame | None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError


class SocketCANBus(CANBus):
    """Classic-CAN bus backed by Linux SocketCAN."""

    def __init__(
        self,
        interface_name: str,
        *,
        receive_own_messages: bool = False,
    ) -> None:
        normalized_interface_name = str(interface_name).strip()
        if not normalized_interface_name:
            raise ValueError("interface_name must not be empty")

        self._interface_name = normalized_interface_name
        self._socket: socket.socket | None = socket.socket(
            socket.AF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW,
        )

        try:
            self._configure_receive_own_messages(receive_own_messages)
            self._socket.bind((self._interface_name,))
        except Exception:
            self.close()
            raise

    @property
    def interface_name(self) -> str:
        return self._interface_name

    def send_frame(self, frame: CANFrame) -> None:
        sock = self._require_socket()
        can_id = int(frame.can_id)
        data = bytes(frame.data)

        if not 0 <= can_id <= STANDARD_CAN_ID_MAX:
            raise ValueError(
                f"Only standard 11-bit CAN IDs are supported: 0x{can_id:X}"
            )
        if len(data) > CLASSIC_CAN_MAX_DATA_LENGTH:
            raise ValueError(
                "Classic CAN payload must be <= 8 bytes, "
                f"got {len(data)}"
            )

        raw = struct.pack(
            CLASSIC_CAN_FRAME_FMT,
            can_id,
            len(data),
            data.ljust(CLASSIC_CAN_MAX_DATA_LENGTH, b"\x00"),
        )
        sock.sendall(raw)

    def recv_frame(self, timeout: float = 0.0) -> CANFrame | None:
        sock = self._require_socket()
        if timeout < 0.0:
            raise ValueError("timeout must be >= 0")

        sock.settimeout(timeout)
        try:
            raw = sock.recv(CLASSIC_CAN_FRAME_SIZE)
        except (socket.timeout, TimeoutError):
            return None

        if len(raw) != CLASSIC_CAN_FRAME_SIZE:
            raise OSError(
                "Incomplete SocketCAN frame: "
                f"expected {CLASSIC_CAN_FRAME_SIZE} bytes, got {len(raw)}"
            )

        can_id, dlc, data = struct.unpack(CLASSIC_CAN_FRAME_FMT, raw)
        can_id &= socket.CAN_SFF_MASK
        return CANFrame(can_id=can_id, data=data[:dlc])

    def close(self) -> None:
        sock = self._socket
        self._socket = None
        if sock is not None:
            sock.close()

    def _configure_receive_own_messages(self, enabled: bool) -> None:
        sock = self._require_socket()
        if not hasattr(socket, "SOL_CAN_RAW") or not hasattr(
            socket,
            "CAN_RAW_RECV_OWN_MSGS",
        ):
            if enabled:
                raise RuntimeError(
                    "Python socket module does not expose "
                    "CAN_RAW_RECV_OWN_MSGS"
                )
            return

        sock.setsockopt(
            socket.SOL_CAN_RAW,
            socket.CAN_RAW_RECV_OWN_MSGS,
            int(bool(enabled)),
        )

    def _require_socket(self) -> socket.socket:
        if self._socket is None:
            raise RuntimeError("SocketCANBus is closed")
        return self._socket