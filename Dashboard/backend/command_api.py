from __future__ import annotations

import socket

from .can_decode import (
    E2BOX_CMD_GET_ALL,
    E2BOX_REQ_ID,
    SPG_CMD_MIT_CONTROL,
    SPG_CMD_MIT_ENTER,
    SPG_CMD_MIT_EXIT,
    SPG_CMD_MIT_SET_ZERO,
)
from .socketcan_io import send_can_frame
from .socketcan_io import CAN_FRAME_SIZE
from .state import MonitorState


class CommandError(RuntimeError):
    pass


def format_tx_result(can_id: int, data: bytes, sent_bytes: int) -> dict:
    return {
        "can_id": f"0x{can_id:03X}",
        "data": data.hex(" ").upper(),
        "sent_bytes": sent_bytes,
    }


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def float_to_uint(value: float, value_min: float, value_max: float, bits: int) -> int:
    value = clamp(value, value_min, value_max)
    span = value_max - value_min
    max_int = (1 << bits) - 1
    return int(round((value - value_min) * max_int / span))


def pack_mit_payload(
    q_rad: float,
    qd_rad_s: float,
    kp: float,
    kd: float,
    tau_ff_nm: float,
    *,
    p_max: float = 12.5,
    v_max: float = 45.0,
    kp_max: float = 500.0,
    kd_max: float = 5.0,
    tau_max: float = 33.0,
) -> bytes:
    p_u = float_to_uint(q_rad, -p_max, p_max, 16)
    v_u = float_to_uint(qd_rad_s, -v_max, v_max, 12)
    kp_u = float_to_uint(kp, 0.0, kp_max, 12)
    kd_u = float_to_uint(kd, 0.0, kd_max, 8)
    t_u = float_to_uint(tau_ff_nm, -tau_max, tau_max, 8)

    data = bytearray(8)
    data[0] = SPG_CMD_MIT_CONTROL
    data[1] = (p_u >> 8) & 0xFF
    data[2] = p_u & 0xFF
    data[3] = (v_u >> 4) & 0xFF
    data[4] = ((v_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F)
    data[5] = kp_u & 0xFF
    data[6] = kd_u & 0xFF
    data[7] = t_u & 0xFF
    return bytes(data)


class CommandService:
    def __init__(self, state: MonitorState) -> None:
        self.state = state
        self.sock: socket.socket | None = None

    def set_socket(self, sock: socket.socket | None) -> None:
        self.sock = sock

    def lock_tx(self) -> None:
        self.state.tx_enabled = False

    def unlock_tx(self) -> None:
        self.state.tx_enabled = True

    def require_tx(self) -> socket.socket:
        if not self.state.tx_enabled:
            raise CommandError("TX is locked")
        if self.sock is None:
            raise CommandError("CAN socket is not connected")
        return self.sock

    def send_raw(self, can_id: int, data: bytes) -> dict:
        sock = self.require_tx()
        sent_bytes = send_can_frame(sock, can_id, data)
        if sent_bytes != CAN_FRAME_SIZE:
            raise CommandError(f"Incomplete CAN frame write: {sent_bytes}/{CAN_FRAME_SIZE} bytes")
        self.state.mark_tx(can_id, data)
        self.state.socket_error = None
        return format_tx_result(can_id, data, sent_bytes)

    def request_imu_all(self) -> dict:
        return self.send_raw(E2BOX_REQ_ID, bytes([E2BOX_CMD_GET_ALL]))

    def send_motor_command(self, motor_id: int, opcode: int, suffix: bytes = b"") -> dict:
        if not self.state.allow_motor_commands:
            raise CommandError("Motor commands are disabled by config")
        data = bytes([opcode]) + b"\x00" * 7
        if suffix:
            data = (bytes([opcode]) + b"\x00" * max(0, 7 - len(suffix)) + suffix)[0:8]
        return self.send_raw(self.state.can_id_for_motor_id(motor_id), data)

    def motor_enter(self, motor_id: int) -> dict:
        return self.send_motor_command(motor_id, SPG_CMD_MIT_ENTER)

    def motor_exit(self, motor_id: int) -> dict:
        return self.send_motor_command(motor_id, SPG_CMD_MIT_EXIT)

    def motor_zero(self, motor_id: int, offset_count: int = 0) -> dict:
        suffix = int(offset_count).to_bytes(2, "little", signed=True)
        return self.send_motor_command(motor_id, SPG_CMD_MIT_SET_ZERO, suffix=suffix)

    def motor_mit_hold(self, motor_id: int) -> dict:
        if not self.state.allow_motor_commands:
            raise CommandError("Motor commands are disabled by config")
        payload = pack_mit_payload(
            q_rad=0.0,
            qd_rad_s=0.0,
            kp=0.0,
            kd=0.5,
            tau_ff_nm=0.0,
        )
        return self.send_raw(self.state.can_id_for_motor_id(motor_id), payload)
