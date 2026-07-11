"""
DongilC/OpenRobot SPG CAN codec.

This module knows OpenRobot motor CAN payloads only. It does not import or
return robot_controller protocol types.
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass, field
from typing import Any

from qhrr0.app.hal.can import CANFrame


ENC_MOD = 16384
ENC_HALF = ENC_MOD // 2
CNT2RAD = 2.0 * math.pi / ENC_MOD

SPG_MIT_P_MAX_RAD = 12.5
SPG_MIT_V_MAX_RAD_S = 45.0
SPG_MIT_KP_MAX = 500.0
SPG_MIT_KD_MAX = 5.0
SPG_MIT_TAU_MAX_NM = 33.0
SPG_MIT_FEEDBACK_POSITION_MAX_RAD = 12.56

SPG_IQ_FULL_SCALE_COUNT = 2048.0
SPG_IQ_FULL_SCALE_CURRENT_A = 33.0
SPG_IQ_COUNT_TO_AMP = SPG_IQ_FULL_SCALE_CURRENT_A / SPG_IQ_FULL_SCALE_COUNT

SPG_SET_ZERO_HOLD_S = 0.020


@dataclass(slots=True)
class ActuatorState:
    position_rad: float | None = None
    velocity_rad_s: float | None = None
    torque_nm: float | None = None
    current_a: float | None = None
    temperature_c: float | None = None

    is_enabled: bool | None = None
    fault_code: int | None = None
    fault_codes: tuple[int, ...] = ()
    mode: str | None = None

    iq_counts: int | None = None
    speed_dps: int | None = None
    mit_position_i16: int | None = None

    encoder_position_u16: int | None = None
    encoder_original_u16: int | None = None
    encoder_offset_u16: int | None = None

    phase_current_a: float | None = None
    phase_current_b: float | None = None
    phase_current_c: float | None = None

    v_max_rad_s: int | None = None
    tau_max_nm: int | None = None
    kt_out_nm_per_a: float | None = None
    gear_ratio: float | None = None

    zero_offset_deg: float | None = None
    last_feedback_t: float = 0.0
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class SPGMITConfig:
    p_max: float = SPG_MIT_P_MAX_RAD
    v_max: float = SPG_MIT_V_MAX_RAD_S
    kp_max: float = SPG_MIT_KP_MAX
    kd_max: float = SPG_MIT_KD_MAX
    tau_max: float = SPG_MIT_TAU_MAX_NM
    feedback_position_max: float = SPG_MIT_FEEDBACK_POSITION_MAX_RAD


SPG_MIT_DEFAULT_CONFIG = SPGMITConfig()


@dataclass(frozen=True)
class SPGMITStatus:
    temp_c: int
    iq_counts: int
    speed_dps: int
    mit_position_i16: int


@dataclass(frozen=True)
class SPGEncoderData:
    temp_c: int
    encoder_position_u16: int
    encoder_original_u16: int
    encoder_offset_u16: int


@dataclass(frozen=True)
class SPGMITParams:
    v_max_rad_s: int
    tau_max_nm: int
    kt_out_nm_per_a: float
    gear_ratio: float


class SPGActuatorDriver:
    CMD_MIT_CONTROL = 0xC0
    CMD_MIT_ENTER = 0xC1
    CMD_MIT_EXIT = 0xC2
    CMD_MIT_SET_ZERO = 0xC3
    CMD_READ_MIT_PARAMS = 0xC4
    CMD_WRITE_MIT_PARAMS = 0xC5

    CMD_READ_ENCODER_DATA = 0x90
    CMD_WRITE_CURRENT_POS_AS_ZERO = 0x19
    CMD_WRITE_ENCODER_OFFSET = 0x91
    CMD_CLEAR_ERROR_FLAG = 0x9B

    def __init__(
        self,
        *,
        command_id: int,
        feedback_id: int,
        mit_config: SPGMITConfig,
    ) -> None:
        self.command_id = int(command_id)
        self.feedback_id = int(feedback_id)
        self.mit_config = mit_config

    def rx_can_ids(self) -> list[int]:
        return [self.feedback_id]

    def decode_frame(self, frame: CANFrame) -> ActuatorState | None:
        if int(frame.can_id) != self.feedback_id:
            return None
        return self.decode_payload(bytes(frame.data))

    def decode_payload(self, payload8: bytes) -> ActuatorState | None:
        if len(payload8) != 8:
            raise ValueError(f"SPG payload must be 8 bytes, got {len(payload8)}")

        cmd = payload8[0]
        if cmd == self.CMD_MIT_CONTROL:
            return self._decode_mit_status(payload8)
        if cmd == self.CMD_READ_ENCODER_DATA:
            return self._decode_encoder_data(payload8)
        if cmd == self.CMD_READ_MIT_PARAMS:
            return self._decode_mit_params(payload8)
        if cmd in (
            self.CMD_MIT_ENTER,
            self.CMD_MIT_EXIT,
            self.CMD_WRITE_CURRENT_POS_AS_ZERO,
            self.CMD_WRITE_ENCODER_OFFSET,
            self.CMD_MIT_SET_ZERO,
            self.CMD_CLEAR_ERROR_FLAG,
        ):
            return self._decode_ack_like_frame(payload8)
        return None

    def encode_enable_frame(self) -> CANFrame:
        return self._frame(bytes([self.CMD_MIT_ENTER, 0, 0, 0, 0, 0, 0, 0]))

    def encode_disable_frame(self) -> CANFrame:
        return self._frame(bytes([self.CMD_MIT_EXIT, 0, 0, 0, 0, 0, 0, 0]))

    def encode_clear_fault_frame(self) -> CANFrame:
        return self._frame(bytes([self.CMD_CLEAR_ERROR_FLAG, 0, 0, 0, 0, 0, 0, 0]))

    def encode_torque_command_frame(self, torque_nm: float) -> CANFrame:
        return self.encode_impedance_command_frame(
            position_rad=0.0,
            velocity_rad_s=0.0,
            kp=0.0,
            kd=0.0,
            torque_ff_nm=torque_nm,
        )

    def encode_impedance_command_frame(
        self,
        *,
        position_rad: float,
        velocity_rad_s: float,
        kp: float,
        kd: float,
        torque_ff_nm: float = 0.0,
    ) -> CANFrame:
        return self._frame(
            self._pack_mit_payload(
                position_rad=position_rad,
                velocity_rad_s=velocity_rad_s,
                kp=kp,
                kd=kd,
                torque_ff_nm=torque_ff_nm,
            )
        )

    def encode_zero_position_frame(self, offset_deg: float = 0.0) -> CANFrame:
        return self.encode_mit_set_zero_frame(offset_deg=offset_deg)

    def encode_mit_set_zero_frame(self, offset_deg: float = 0.0) -> CANFrame:
        return self._frame(self._pack_mit_set_zero_payload(offset_deg=offset_deg))

    def encode_read_mit_params_frame(self) -> CANFrame:
        return self._frame(bytes([self.CMD_READ_MIT_PARAMS, 0, 0, 0, 0, 0, 0, 0]))

    def encode_write_mit_params_frame(
        self,
        *,
        v_max_rad_s: int,
        tau_max_nm: int,
        kt_input_nm_per_a: float,
        gear_ratio: float,
    ) -> CANFrame:
        if not (0 <= int(v_max_rad_s) <= 255):
            raise ValueError("v_max_rad_s must fit uint8")
        if not (0 <= int(tau_max_nm) <= 255):
            raise ValueError("tau_max_nm must fit uint8")

        kt_raw = round_half_away_from_zero(kt_input_nm_per_a * 1000.0)
        gear_ratio_raw = round_half_away_from_zero(gear_ratio * 100.0)
        if not (0 <= kt_raw <= 0xFFFF):
            raise ValueError("kt_input_nm_per_a is out of uint16 range")
        if not (0 <= gear_ratio_raw <= 0xFFFF):
            raise ValueError("gear_ratio is out of uint16 range")

        data = bytearray(8)
        data[0] = self.CMD_WRITE_MIT_PARAMS
        data[1] = int(v_max_rad_s) & 0xFF
        data[2] = int(tau_max_nm) & 0xFF
        struct.pack_into("<H", data, 3, kt_raw)
        struct.pack_into("<H", data, 5, gear_ratio_raw)
        return self._frame(bytes(data))

    def encode_read_encoder_data_frame(self) -> CANFrame:
        return self._frame(bytes([self.CMD_READ_ENCODER_DATA, 0, 0, 0, 0, 0, 0, 0]))

    def encode_write_current_position_as_zero_frame(self) -> CANFrame:
        return self._frame(bytes([self.CMD_WRITE_CURRENT_POS_AS_ZERO, 0, 0, 0, 0, 0, 0, 0]))

    def encode_write_encoder_offset_frame(self, offset_u16: int) -> CANFrame:
        offset_u16 &= 0xFFFF
        return self._frame(
            bytes([
                self.CMD_WRITE_ENCODER_OFFSET,
                0,
                0,
                0,
                0,
                0,
                offset_u16 & 0xFF,
                (offset_u16 >> 8) & 0xFF,
            ])
        )

    def encode_set_current_position_as_rad_frame(
        self,
        *,
        original_u16: int,
        desired_rad: float,
    ) -> CANFrame:
        original_cnt = wrap_u14(original_u16)
        desired_cnt = rad_to_u14_count(desired_rad)
        return self.encode_write_encoder_offset_frame((desired_cnt - original_cnt) % ENC_MOD)

    def _decode_mit_status(self, payload8: bytes) -> ActuatorState:
        status = self._parse_mit_status_v14(payload8)
        position_output_rad = status.mit_position_i16 / 32767.0 * self.mit_config.feedback_position_max
        return ActuatorState(
            temperature_c=float(status.temp_c),
            iq_counts=status.iq_counts,
            speed_dps=status.speed_dps,
            mit_position_i16=status.mit_position_i16,
            position_rad=position_output_rad,
            velocity_rad_s=math.radians(status.speed_dps),
            last_feedback_t=time.monotonic(),
            raw={"cmd": self.CMD_MIT_CONTROL},
        )

    def _decode_encoder_data(self, payload8: bytes) -> ActuatorState:
        enc = self._parse_encoder_data(payload8)
        return ActuatorState(
            temperature_c=float(enc.temp_c),
            encoder_position_u16=enc.encoder_position_u16,
            encoder_original_u16=enc.encoder_original_u16,
            encoder_offset_u16=enc.encoder_offset_u16,
            last_feedback_t=time.monotonic(),
            raw={"cmd": self.CMD_READ_ENCODER_DATA},
        )

    def _decode_mit_params(self, payload8: bytes) -> ActuatorState:
        params = self._parse_mit_params(payload8)
        return ActuatorState(
            mode="MIT_PARAMS",
            v_max_rad_s=params.v_max_rad_s,
            tau_max_nm=params.tau_max_nm,
            kt_out_nm_per_a=params.kt_out_nm_per_a,
            gear_ratio=params.gear_ratio,
            last_feedback_t=time.monotonic(),
            raw={"cmd": self.CMD_READ_MIT_PARAMS},
        )

    def _decode_ack_like_frame(self, payload8: bytes) -> ActuatorState:
        cmd = payload8[0]
        state = ActuatorState(
            is_enabled=_ack_enabled_hint(cmd),
            mode=_ack_mode(cmd),
            fault_code=int(payload8[1]) if cmd == self.CMD_CLEAR_ERROR_FLAG else None,
            last_feedback_t=time.monotonic(),
            raw={"cmd": cmd},
        )
        if cmd == self.CMD_MIT_SET_ZERO:
            state.zero_offset_deg = struct.unpack("<h", payload8[6:8])[0] * 0.01
        if cmd in (self.CMD_WRITE_CURRENT_POS_AS_ZERO, self.CMD_WRITE_ENCODER_OFFSET):
            state.encoder_offset_u16 = struct.unpack("<H", payload8[6:8])[0]
        return state

    @classmethod
    def _parse_mit_status_v14(cls, payload8: bytes) -> SPGMITStatus:
        if len(payload8) != 8 or payload8[0] != cls.CMD_MIT_CONTROL:
            raise ValueError("Invalid SPG MIT v14 status response")
        return SPGMITStatus(
            temp_c=struct.unpack("b", payload8[1:2])[0],
            iq_counts=struct.unpack("<h", payload8[2:4])[0],
            speed_dps=struct.unpack("<h", payload8[4:6])[0],
            mit_position_i16=struct.unpack("<h", payload8[6:8])[0],
        )

    @classmethod
    def _parse_encoder_data(cls, payload8: bytes) -> SPGEncoderData:
        if len(payload8) != 8 or payload8[0] != cls.CMD_READ_ENCODER_DATA:
            raise ValueError("Invalid SPG encoder data response")
        return SPGEncoderData(
            temp_c=struct.unpack("b", payload8[1:2])[0],
            encoder_position_u16=wrap_u14(struct.unpack("<H", payload8[2:4])[0]),
            encoder_original_u16=wrap_u14(struct.unpack("<H", payload8[4:6])[0]),
            encoder_offset_u16=wrap_u14(struct.unpack("<H", payload8[6:8])[0]),
        )

    @classmethod
    def _parse_mit_params(cls, payload8: bytes) -> SPGMITParams:
        if len(payload8) != 8 or payload8[0] != cls.CMD_READ_MIT_PARAMS:
            raise ValueError("Invalid SPG MIT params response")
        return SPGMITParams(
            v_max_rad_s=int(payload8[1]),
            tau_max_nm=int(payload8[2]),
            kt_out_nm_per_a=struct.unpack("<H", payload8[3:5])[0] * 0.001,
            gear_ratio=struct.unpack("<H", payload8[5:7])[0] * 0.01,
        )

    def _pack_mit_payload(
        self,
        *,
        position_rad: float,
        velocity_rad_s: float,
        kp: float,
        kd: float,
        torque_ff_nm: float,
    ) -> bytes:
        cfg = self.mit_config
        require_range(position_rad, -cfg.p_max, cfg.p_max, "position_rad")
        require_range(velocity_rad_s, -cfg.v_max, cfg.v_max, "velocity_rad_s")
        require_range(kp, 0.0, cfg.kp_max, "kp")
        require_range(kd, 0.0, cfg.kd_max, "kd")
        require_range(torque_ff_nm, -cfg.tau_max, cfg.tau_max, "torque_ff_nm")

        p_u = float_to_uint(position_rad, -cfg.p_max, cfg.p_max, 16)
        v_u = float_to_uint(velocity_rad_s, -cfg.v_max, cfg.v_max, 12)
        kp_u = float_to_uint(kp, 0.0, cfg.kp_max, 12)
        kd_u = float_to_uint(kd, 0.0, cfg.kd_max, 8)
        t_u = float_to_uint(torque_ff_nm, -cfg.tau_max, cfg.tau_max, 8)

        data = bytearray(8)
        data[0] = self.CMD_MIT_CONTROL
        data[1] = (p_u >> 8) & 0xFF
        data[2] = p_u & 0xFF
        data[3] = (v_u >> 4) & 0xFF
        data[4] = ((v_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F)
        data[5] = kp_u & 0xFF
        data[6] = kd_u & 0xFF
        data[7] = t_u & 0xFF
        return bytes(data)

    def _pack_mit_set_zero_payload(self, offset_deg: float = 0.0) -> bytes:
        offset_raw = round_half_away_from_zero(offset_deg * 100.0)
        if not (-32768 <= offset_raw <= 32767):
            raise ValueError("offset_deg is out of int16 range for 0.01 deg/LSB encoding")
        data = bytearray(8)
        data[0] = self.CMD_MIT_SET_ZERO
        struct.pack_into("<h", data, 6, offset_raw)
        return bytes(data)

    def _frame(self, payload: bytes) -> CANFrame:
        return CANFrame(can_id=self.command_id, data=payload)


def require_range(x: float, lo: float, hi: float, field: str) -> None:
    if x < lo or x > hi:
        raise ValueError(f"{field} out of range: {x} not in [{lo}, {hi}]")


def round_half_away_from_zero(x: float) -> int:
    if x >= 0.0:
        return int(math.floor(x + 0.5))
    return int(math.ceil(x - 0.5))


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    require_range(x, x_min, x_max, "MIT field")
    uint_max = (1 << bits) - 1
    raw = round_half_away_from_zero((x - x_min) * uint_max / (x_max - x_min))
    return max(0, min(raw, uint_max))


def wrap_u14(x: int) -> int:
    return x & 0x3FFF


def u14_count_to_rad(cnt: int) -> float:
    return wrap_u14(cnt) * CNT2RAD


def rad_to_u14_count(rad: float) -> int:
    return round_half_away_from_zero((rad % (2.0 * math.pi)) / (2.0 * math.pi) * ENC_MOD) % ENC_MOD


def signed_u14_count_to_rad(cnt: int) -> float:
    value = wrap_u14(cnt)
    if value >= ENC_HALF:
        value -= ENC_MOD
    return value * CNT2RAD


def _ack_enabled_hint(cmd: int) -> bool | None:
    if cmd == SPGActuatorDriver.CMD_MIT_ENTER:
        return True
    if cmd in (SPGActuatorDriver.CMD_MIT_EXIT, SPGActuatorDriver.CMD_CLEAR_ERROR_FLAG):
        return False
    return None


def _ack_mode(cmd: int) -> str | None:
    modes = {
        SPGActuatorDriver.CMD_MIT_ENTER: "MIT_ENTER_ACK",
        SPGActuatorDriver.CMD_MIT_EXIT: "MIT_EXIT_ACK",
        SPGActuatorDriver.CMD_MIT_SET_ZERO: "MIT_SET_ZERO_ACK",
        SPGActuatorDriver.CMD_CLEAR_ERROR_FLAG: "CLEAR_ERROR_ACK",
        SPGActuatorDriver.CMD_WRITE_CURRENT_POS_AS_ZERO: "WRITE_CURRENT_POS_AS_ZERO_ACK",
        SPGActuatorDriver.CMD_WRITE_ENCODER_OFFSET: "WRITE_ENCODER_OFFSET_ACK",
    }
    return modes.get(cmd)
