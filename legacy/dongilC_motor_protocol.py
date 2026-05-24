#!/usr/bin/env python3

import math
import signal
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

IFACE = "can0"
FRAME_FMT = "=IB3x8s"

g_run = True

ENC_MOD = 16384
ENC_HALF = ENC_MOD // 2
CNT2RAD = 2.0 * math.pi / ENC_MOD


@dataclass
class OutputAngleEstimator:
    gear_ratio: float
    motor_accum_rad: float = 0.0
    output_accum_rad: float = 0.0
    last_enc_u16: Optional[int] = None
    initialized: bool = False

    def reset_from_current(self, enc_u16: int, output_rad: float = 0.0) -> None:
        """
        현재 샘플을 기준점으로 잡음.
        output_rad는 사용자가 정의하는 출력축 절대 기준각.
        """
        self.last_enc_u16 = wrap_u14(enc_u16)
        self.output_accum_rad = output_rad
        self.motor_accum_rad = output_rad * self.gear_ratio
        self.initialized = True

    def update_from_enc(self, enc_u16: int) -> float:
        """
        MIT 응답의 enc_u16(모터단 single-turn 추정)으로부터
        출력단 누적 각도(rad)를 추정
        """
        enc_u16 = wrap_u14(enc_u16)

        if not self.initialized or self.last_enc_u16 is None:
            self.reset_from_current(enc_u16, 0.0)
            return self.output_accum_rad

        dcnt = shortest_delta_u14(enc_u16, self.last_enc_u16)
        dmotor = dcnt * CNT2RAD

        self.motor_accum_rad += dmotor
        self.output_accum_rad = self.motor_accum_rad / self.gear_ratio
        self.last_enc_u16 = enc_u16

        return self.output_accum_rad

    def get_output_rad(self) -> float:
        return self.output_accum_rad

    def get_motor_rad(self) -> float:
        return self.motor_accum_rad


@dataclass
class EncoderData:
    temp_c: int
    encoder_position_u16: int   # original - offset
    encoder_original_u16: int   # raw
    encoder_offset_u16: int     # stored offset


@dataclass
class MotorUnwrapState:
    last_enc_u16: Optional[int] = None
    accum_motor_rad: float = 0.0
    initialized: bool = False

    def reset(self, enc_u16: int, init_rad: float = 0.0) -> None:
        self.last_enc_u16 = wrap_u14(enc_u16)
        self.accum_motor_rad = init_rad
        self.initialized = True

    def update(self, enc_u16: int) -> float:
        enc_u16 = wrap_u14(enc_u16)

        if not self.initialized or self.last_enc_u16 is None:
            self.reset(enc_u16, 0.0)
            return self.accum_motor_rad

        dcnt = shortest_delta_u14(enc_u16, self.last_enc_u16)
        self.accum_motor_rad += dcnt * CNT2RAD
        self.last_enc_u16 = enc_u16
        return self.accum_motor_rad
    


def wrap_u14(x: int) -> int:
    return x & 0x3FFF


def shortest_delta_u14(curr_u16: int, prev_u16: int) -> int:
    """
    14-bit single-turn encoder count의 wrap-around를 고려한 signed delta count
    반환 범위: [-8192, 8191] 근처
    """
    curr = wrap_u14(curr_u16)
    prev = wrap_u14(prev_u16)

    delta = curr - prev
    if delta > ENC_HALF:
        delta -= ENC_MOD
    elif delta < -ENC_HALF:
        delta += ENC_MOD
    return delta


@dataclass
class MITConfig:
    p_max: float = 12.5
    v_max: float = 45.0
    kp_max: float = 500.0
    kd_max: float = 5.0
    t_max: float = 33.0


@dataclass
class MotorStatus:
    temp_c: int
    iq_counts: int
    speed_dps: int
    enc_u16: int

def open_can(iface: str) -> socket.socket:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((iface,))
    return sock


def send_frame(sock: socket.socket, can_id: int, data8: bytes) -> None:
    if len(data8) != 8:
        raise ValueError("CAN payload must be exactly 8 bytes")
    frame = struct.pack(FRAME_FMT, can_id, 8, data8)
    sock.send(frame)


def recv_frame(
    sock: socket.socket, timeout: float = 0.0
) -> Optional[Tuple[int, int, bytes]]:
    sock.settimeout(timeout)
    try:
        frame = sock.recv(16)
    except (TimeoutError, OSError):
        return None

    can_id, dlc, data = struct.unpack(FRAME_FMT, frame)
    return can_id, dlc, data[:dlc]


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = clamp(x, x_min, x_max)
    span = x_max - x_min
    max_int = (1 << bits) - 1
    return int(round((x - x_min) * max_int / span))

def flush_rx(sock: socket.socket, max_frames: int = 256) -> None:
    for _ in range(max_frames):
        rx = recv_frame(sock, timeout=0.0)
        if rx is None:
            break


def u16_to_deg(count: int) -> float:
    # 문서상 CAN SID 엔코더 값은 14-bit 정규화, CNT2DEG = 360/16384
    return (count & 0x3FFF) * 360.0 / 16384.0

def rad_to_u14_count(rad: float) -> int:
    """
    rad -> 14-bit encoder count (0~16383)
    """
    two_pi = 2.0 * math.pi
    rad_wrapped = rad % two_pi
    cnt = int(round(rad_wrapped * 16384.0 / two_pi)) % 16384
    return cnt

def u14_count_to_rad(cnt: int) -> float:
    """
    14-bit encoder count -> rad
    """
    return (cnt & 0x3FFF) * (2.0 * math.pi) / 16384.0

def set_current_position_as_rad(
    sock: socket.socket,
    can_id: int,
    desired_rad: float,
) -> int:
    """
    현재 물리 위치가 desired_rad 로 읽히도록 encoder offset을 계산하여 EEPROM에 저장

    encoder_position = encoder_original - encoder_offset
    offset = original - desired

    return: stored offset (count)
    """

    # 현재 encoder 상태 읽기
    st = read_encoder_data(sock, can_id)

    original_cnt = st.encoder_original_u16 & 0x3FFF
    desired_cnt = rad_to_u14_count(desired_rad)

    # offset 계산 (14-bit wrap)
    offset_cnt = (desired_cnt - original_cnt) % 16384

    flush_rx(sock)

    send_write_encoder_offset(sock, can_id, offset_cnt)

    payload = wait_for_response(
        sock,
        can_id,
        expected_cmd=0x91,
        timeout=1.0,
    )

    ack_offset = struct.unpack("<H", payload[6:8])[0]

    return ack_offset




def zero_set_encoder(sock: socket.socket, can_id: int) -> int:
    """
    현재 모터 위치를 엔코더 영점으로 EEPROM에 저장.
    반환값은 저장된 offset(u16).
    """
    flush_rx(sock)
    send_write_current_pos_as_zero(sock, can_id)
    payload = wait_for_response(sock, can_id, expected_cmd=0x19, timeout=1.0)
    offset_u16 = struct.unpack("<H", payload[6:8])[0]
    return offset_u16


def signed_encoder_position_deg(pos_u16: int) -> float:
    # 문서 설명대로 encoder position은 original-offset 이므로 음수 개념이 생길 수 있음.
    # 14-bit 범위를 signed로 해석: [0, 16383] -> [-8192, 8191] 근사 해석
    v = pos_u16 & 0x3FFF
    if v >= 8192:
        v -= 16384
    return v * 360.0 / 16384.0


def parse_read_encoder_data(payload8: bytes) -> EncoderData:
    if len(payload8) != 8 or payload8[0] != 0x90:
        raise ValueError("Invalid 0x90 response")

    temp_c = struct.unpack("b", payload8[1:2])[0]
    encoder_position_u16 = struct.unpack("<H", payload8[2:4])[0]
    encoder_original_u16 = struct.unpack("<H", payload8[4:6])[0]
    encoder_offset_u16 = struct.unpack("<H", payload8[6:8])[0]

    return EncoderData(
        temp_c=temp_c,
        encoder_position_u16=encoder_position_u16,
        encoder_original_u16=encoder_original_u16,
        encoder_offset_u16=encoder_offset_u16,
    )




def parse_status_common(payload8: bytes, expected_cmd: int) -> MotorStatus:
    if len(payload8) != 8 or payload8[0] != expected_cmd:
        raise ValueError("Invalid response")

    temp_c = struct.unpack("b", payload8[1:2])[0]
    iq_counts = struct.unpack("<h", payload8[2:4])[0]
    speed_dps = struct.unpack("<h", payload8[4:6])[0]
    enc_u16 = struct.unpack("<H", payload8[6:8])[0]

    return MotorStatus(
        temp_c=temp_c,
        iq_counts=iq_counts,
        speed_dps=speed_dps,
        enc_u16=enc_u16,
    )


def send_mit_enter(sock: socket.socket, can_id: int) -> None:
    send_frame(sock, can_id, bytes([0xC1, 0, 0, 0, 0, 0, 0, 0]))


def send_mit_exit(sock: socket.socket, can_id: int) -> None:
    send_frame(sock, can_id, bytes([0xC2, 0, 0, 0, 0, 0, 0, 0]))


def send_read_encoder_data(sock: socket.socket, can_id: int) -> None:
    send_frame(sock, can_id, bytes([0x90, 0, 0, 0, 0, 0, 0, 0]))


def send_write_current_pos_as_zero(sock: socket.socket, can_id: int) -> None:
    send_frame(sock, can_id, bytes([0x19, 0, 0, 0, 0, 0, 0, 0]))

def send_write_encoder_offset(sock: socket.socket, can_id: int, offset_u16: int) -> None:
    """
    0x91 Write Encoder Offset
    TX: [0x91, 0, 0, 0, 0, 0, offset_lo, offset_hi]
    """
    offset_u16 &= 0xFFFF
    payload = bytes([
        0x91, 0, 0, 0, 0, 0,
        offset_u16 & 0xFF,
        (offset_u16 >> 8) & 0xFF,
    ])
    send_frame(sock, can_id, payload)

def wait_for_response(
    sock: socket.socket,
    can_id: int,
    expected_cmd: int,
    timeout: float = 0.5,
) -> bytes:
    t0 = time.time()
    while (time.time() - t0) < timeout:
        rx = recv_frame(sock, timeout=0.05)
        if rx is None:
            continue
        rid, dlc, data = rx
        if rid != can_id:
            continue
        if dlc != 8 or len(data) != 8:
            continue
        if data[0] != expected_cmd:
            continue
        return data
    raise TimeoutError(f"No response for cmd=0x{expected_cmd:02X} from can_id=0x{can_id:X}")


def read_encoder_data(sock: socket.socket, can_id: int) -> EncoderData:
    flush_rx(sock)
    send_read_encoder_data(sock, can_id)
    payload = wait_for_response(sock, can_id, expected_cmd=0x90, timeout=0.5)
    return parse_read_encoder_data(payload)


def print_encoder_state(prefix: str, st: EncoderData) -> None:
    pos_deg_signed = signed_encoder_position_deg(st.encoder_position_u16)
    org_deg = u16_to_deg(st.encoder_original_u16)
    off_deg = u16_to_deg(st.encoder_offset_u16)

    print(f"{prefix}")
    print(f"  temp              : {st.temp_c} C")
    print(f"  encoder_position  : {st.encoder_position_u16:5d} cnt  ({pos_deg_signed:+8.3f} deg)")
    print(f"  encoder_original  : {st.encoder_original_u16:5d} cnt  ({org_deg:8.3f} deg)")
    print(f"  encoder_offset    : {st.encoder_offset_u16:5d} cnt  ({off_deg:8.3f} deg)")


def pack_mit_payload(
    p_des: float,
    v_des: float,
    kp: float,
    kd: float,
    tau_ff: float,
    cfg: MITConfig,
) -> bytes:
    p_u = float_to_uint(p_des, -cfg.p_max, cfg.p_max, 16)
    v_u = float_to_uint(v_des, -cfg.v_max, cfg.v_max, 12)
    kp_u = float_to_uint(kp, 0.0, cfg.kp_max, 12)
    kd_u = float_to_uint(kd, 0.0, cfg.kd_max, 8)
    t_u = float_to_uint(tau_ff, -cfg.t_max, cfg.t_max, 8)

    data = bytearray(8)
    data[0] = 0xC0
    data[1] = (p_u >> 8) & 0xFF
    data[2] = p_u & 0xFF
    data[3] = (v_u >> 4) & 0xFF
    data[4] = ((v_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F)
    data[5] = kp_u & 0xFF
    data[6] = kd_u & 0xFF
    data[7] = t_u & 0xFF
    return bytes(data)


def pack_mit_set_zero_payload(offset_deg: float = 0.0) -> bytes:
    """Create MIT Set Zero Position (0xC3) CAN payload.

    Protocol:
        DATA[0]   = 0xC3
        DATA[1:6] = 0x00
        DATA[6:7] = offset (int16, little-endian, 0.01 deg/LSB)

    Meaning:
        offset_deg = 0.0   -> current position becomes 0 deg
        offset_deg = 30.0  -> current position becomes +30 deg

    Args:
        offset_deg: Output-side zero offset in degrees.

    Returns:
        8-byte CAN payload.
    """
    scale = 100.0  # 0.01 deg/LSB
    offset_raw = int(round(offset_deg * scale))

    if not (-32768 <= offset_raw <= 32767):
        raise ValueError(
            f"offset_deg={offset_deg} is out of range for int16 "
            f"({-327.68} ~ {327.67} deg)"
        )

    data = bytearray(8)
    data[0] = 0xC3
    data[1] = 0x00
    data[2] = 0x00
    data[3] = 0x00
    data[4] = 0x00
    data[5] = 0x00

    offset_bytes = offset_raw.to_bytes(2, byteorder="little", signed=True)
    data[6] = offset_bytes[0]
    data[7] = offset_bytes[1]

    return bytes(data)


def send_mit_control(
    sock: socket.socket,
    can_id: int,
    p_des: float,
    v_des: float,
    kp: float,
    kd: float,
    tau_ff: float,
    cfg: MITConfig,
) -> None:
    payload = pack_mit_payload(p_des, v_des, kp, kd, tau_ff, cfg)
    send_frame(sock, can_id, payload)


def send_mit_zero(
    sock: socket.socket,
    can_id: int,
    offset: float,
) -> None:
    payload = pack_mit_set_zero_payload(offset_deg=offset)
    send_frame(sock, can_id, payload)




def send_all_mit(
    sock: socket.socket,
    can_ids: List[int],
    target_map: Dict[int, float],
    v_des_map: Dict[int, float],
    kp_map: Dict[int, float],
    kd_map: Dict[int, float],
    tau_ff_map: Dict[int, float],
    cfg: MITConfig,
) -> None:
    for can_id in can_ids:
        p_des = target_map[can_id]
        v_des = v_des_map[can_id]
        kp = kp_map[can_id]
        kd = kd_map[can_id]
        tau_ff = tau_ff_map[can_id]
        send_mit_control(sock, can_id, p_des, v_des, kp, kd, tau_ff, cfg)




def drain_rx_buffer(
    sock: socket.socket,
    can_ids: List[int],
    expected_cmd: int = 0xC0,
    max_frames: int = 4096,
    timeout=0.0,
) -> Tuple[Dict[int, MotorStatus], int]:
    latest_status: Dict[int, MotorStatus] = {}
    recv_count = 0
    can_id_set = set(can_ids)

    for _ in range(max_frames):
        rx = recv_frame(sock, timeout=timeout)
        if rx is None:
            break

        rid, dlc, data = rx

        if rid not in can_id_set:
            continue
        if dlc != 8:
            continue
        if len(data) != 8:
            continue
        if data[0] != expected_cmd:
            continue

        try:
            st = parse_status_common(data, expected_cmd=expected_cmd)  
            # print("parse")
            latest_status[rid] = st
            recv_count += 1
        except ValueError:
            continue

    return latest_status, recv_count

def shortest_delta_u14(curr_u16: int, prev_u16: int) -> int:
    curr = wrap_u14(curr_u16)
    prev = wrap_u14(prev_u16)

    delta = curr - prev
    if delta > ENC_HALF:
        delta -= ENC_MOD
    elif delta < -ENC_HALF:
        delta += ENC_MOD
    return delta


