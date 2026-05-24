"""
SPG actuator CAN protocol.

This module implements SPG-specific CAN frame encoding and decoding.
It is compatible with the generic ActuatorDriver / ActuatorProtocolBase
structure.

Responsibilities:
- Define SPG CAN command opcodes.
- Encode SPG MIT-style command frames.
- Decode SPG feedback/status frames into ActuatorState.
- Keep SocketCAN, CANDaemon, and CANDispatcher out of the protocol layer.
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass

from hal.can_bus import CANFrame

from hal.hardware.can.actuator.protocol import ActuatorProtocolBase
from hal.hardware.can.actuator.state import ActuatorState


ENC_MOD = 16384
ENC_HALF = ENC_MOD // 2
CNT2RAD = 2.0 * math.pi / ENC_MOD


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = clamp(x, x_min, x_max)
    span = x_max - x_min
    max_int = (1 << bits) - 1
    return int(round((x - x_min) * max_int / span))


def wrap_u14(x: int) -> int:
    return x & 0x3FFF


def shortest_delta_u14(curr_u16: int, prev_u16: int) -> int:
    curr = wrap_u14(curr_u16)
    prev = wrap_u14(prev_u16)

    delta = curr - prev

    if delta > ENC_HALF:
        delta -= ENC_MOD
    elif delta < -ENC_HALF:
        delta += ENC_MOD

    return delta


def u14_count_to_rad(cnt: int) -> float:
    return wrap_u14(cnt) * CNT2RAD


def rad_to_u14_count(rad: float) -> int:
    rad_wrapped = rad % (2.0 * math.pi)
    return int(round(rad_wrapped / (2.0 * math.pi) * ENC_MOD)) % ENC_MOD


def signed_u14_count_to_rad(cnt: int) -> float:
    value = wrap_u14(cnt)

    if value >= ENC_HALF:
        value -= ENC_MOD

    return value * CNT2RAD


# ---------------------------------------------------------------------------
# SPG protocol data containers
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SPGMITConfig:
    """
    MIT-style command packing limits.

    These limits must match the actuator firmware configuration.
    """

    p_max: float = 12.5
    v_max: float = 45.0
    kp_max: float = 500.0
    kd_max: float = 5.0
    tau_max: float = 33.0


@dataclass(frozen=True)
class SPGMotorStatus:
    temp_c: int
    iq_counts: int
    speed_dps: int
    enc_u16: int


@dataclass(frozen=True)
class SPGEncoderData:
    temp_c: int
    encoder_position_u16: int
    encoder_original_u16: int
    encoder_offset_u16: int


# ---------------------------------------------------------------------------
# SPG actuator protocol
# ---------------------------------------------------------------------------

class SPGActuatorProtocol(ActuatorProtocolBase):
    """
    SPG actuator protocol implementation.

    This class only handles frame-level encoding/decoding. It does not send
    frames and does not keep long-term actuator state.
    """

    CMD_MIT_CONTROL = 0xC0
    CMD_MIT_ENTER = 0xC1
    CMD_MIT_EXIT = 0xC2
    CMD_MIT_SET_ZERO = 0xC3

    CMD_READ_ENCODER_DATA = 0x90
    CMD_WRITE_CURRENT_POS_AS_ZERO = 0x19
    CMD_WRITE_ENCODER_OFFSET = 0x91

    def __init__(
        self,
        command_id: int,
        feedback_id: int | None = None,
        mit_config: SPGMITConfig | None = None,
        gear_ratio: float = 1.0,
        feedback_speed_is_motor_side: bool = True,
        expose_single_turn_position: bool = False,
        iq_count_to_amp: float | None = None,
        torque_constant_nm_per_a: float | None = None,
        gear_efficiency: float = 1.0,
    ):
        """
        Args:
            command_id:
                CAN ID used for command transmission.

            feedback_id:
                CAN ID used for feedback reception. If omitted, command_id is used.

            mit_config:
                MIT-style command packing limits.

            gear_ratio:
                Motor-to-output gear ratio. Used only for optional output-side
                velocity/position conversion.

            feedback_speed_is_motor_side:
                If True, decoded speed_dps is divided by gear_ratio before being
                exposed as velocity_rad_s.

            expose_single_turn_position:
                If True, 14-bit single-turn encoder position is exposed as
                ActuatorState.position_rad. For continuous joint position,
                use a driver-side unwrap estimator instead.

            iq_count_to_amp:
                Optional scale from iq_counts to phase current [A].

            torque_constant_nm_per_a:
                Optional motor torque constant. If both iq_count_to_amp and this
                value are provided, torque_nm is estimated.

            gear_efficiency:
                Optional gear efficiency multiplier for torque estimation.
        """
        if gear_ratio <= 0.0:
            raise ValueError("gear_ratio must be positive")

        self.command_id = command_id
        self.feedback_id = feedback_id if feedback_id is not None else command_id
        self.mit_config = mit_config or SPGMITConfig()

        self.gear_ratio = gear_ratio
        self.feedback_speed_is_motor_side = feedback_speed_is_motor_side
        self.expose_single_turn_position = expose_single_turn_position

        self.iq_count_to_amp = iq_count_to_amp
        self.torque_constant_nm_per_a = torque_constant_nm_per_a
        self.gear_efficiency = gear_efficiency

    # ------------------------------------------------------------------
    # RX
    # ------------------------------------------------------------------

    def rx_can_ids(self) -> list[int]:
        return [self.feedback_id]

    def decode_frame(self, frame: CANFrame) -> ActuatorState | None:
        if frame.can_id != self.feedback_id:
            return None

        if len(frame.data) != 8:
            raise ValueError("SPG feedback payload must be 8 bytes")

        cmd = frame.data[0]

        if cmd == self.CMD_MIT_CONTROL:
            return self._decode_mit_status(frame.data)

        if cmd == self.CMD_READ_ENCODER_DATA:
            return self._decode_encoder_data(frame.data)

        if cmd in (
            self.CMD_WRITE_CURRENT_POS_AS_ZERO,
            self.CMD_WRITE_ENCODER_OFFSET,
            self.CMD_MIT_SET_ZERO,
        ):
            return self._decode_ack_like_frame(frame.data)

        return None

    # ------------------------------------------------------------------
    # Generic actuator command encoders
    # ------------------------------------------------------------------

    def encode_enable_frame(self) -> CANFrame:
        return CANFrame(
            can_id=self.command_id,
            data=bytes([self.CMD_MIT_ENTER, 0, 0, 0, 0, 0, 0, 0]),
        )

    def encode_disable_frame(self) -> CANFrame:
        return CANFrame(
            can_id=self.command_id,
            data=bytes([self.CMD_MIT_EXIT, 0, 0, 0, 0, 0, 0, 0]),
        )

    def encode_torque_command_frame(self, torque_nm: float) -> CANFrame:
        payload = self._pack_mit_payload(
            position_rad=0.0,
            velocity_rad_s=0.0,
            kp=0.0,
            kd=0.0,
            torque_ff_nm=torque_nm,
        )
        return CANFrame(can_id=self.command_id, data=payload)

    def encode_impedance_command_frame(
        self,
        position_rad: float,
        velocity_rad_s: float,
        kp: float,
        kd: float,
        torque_ff_nm: float = 0.0,
    ) -> CANFrame:
        payload = self._pack_mit_payload(
            position_rad=position_rad,
            velocity_rad_s=velocity_rad_s,
            kp=kp,
            kd=kd,
            torque_ff_nm=torque_ff_nm,
        )
        return CANFrame(can_id=self.command_id, data=payload)

    def encode_zero_position_frame(self) -> CANFrame:
        return self.encode_mit_set_zero_frame(offset_deg=0.0)

    # ------------------------------------------------------------------
    # SPG-specific command encoders
    # ------------------------------------------------------------------

    def encode_mit_set_zero_frame(self, offset_deg: float = 0.0) -> CANFrame:
        payload = self._pack_mit_set_zero_payload(offset_deg=offset_deg)
        return CANFrame(can_id=self.command_id, data=payload)

    def encode_read_encoder_data_frame(self) -> CANFrame:
        return CANFrame(
            can_id=self.command_id,
            data=bytes([self.CMD_READ_ENCODER_DATA, 0, 0, 0, 0, 0, 0, 0]),
        )

    def encode_write_current_position_as_zero_frame(self) -> CANFrame:
        return CANFrame(
            can_id=self.command_id,
            data=bytes([self.CMD_WRITE_CURRENT_POS_AS_ZERO, 0, 0, 0, 0, 0, 0, 0]),
        )

    def encode_write_encoder_offset_frame(self, offset_u16: int) -> CANFrame:
        offset_u16 &= 0xFFFF

        payload = bytes([
            self.CMD_WRITE_ENCODER_OFFSET,
            0,
            0,
            0,
            0,
            0,
            offset_u16 & 0xFF,
            (offset_u16 >> 8) & 0xFF,
        ])

        return CANFrame(can_id=self.command_id, data=payload)

    def encode_set_current_position_as_rad_frame(self, original_u16: int, desired_rad: float) -> CANFrame:
        """
        Create encoder-offset write frame so the current physical position is
        interpreted as desired_rad.

        This only creates the write frame. Reading original_u16 and waiting for
        ACK should be handled by a manager or driver-level transaction flow.
        """
        original_cnt = wrap_u14(original_u16)
        desired_cnt = rad_to_u14_count(desired_rad)
        offset_cnt = (desired_cnt - original_cnt) % ENC_MOD

        return self.encode_write_encoder_offset_frame(offset_cnt)

    # ------------------------------------------------------------------
    # Decode helpers
    # ------------------------------------------------------------------

    def _decode_mit_status(self, payload8: bytes) -> ActuatorState:
        status = self._parse_status_common(
            payload8=payload8,
            expected_cmd=self.CMD_MIT_CONTROL,
        )

        velocity_rad_s = math.radians(status.speed_dps)

        if self.feedback_speed_is_motor_side:
            velocity_rad_s /= self.gear_ratio

        single_turn_motor_rad = u14_count_to_rad(status.enc_u16)
        single_turn_output_rad = single_turn_motor_rad / self.gear_ratio

        current_a = None
        torque_nm = None

        if self.iq_count_to_amp is not None:
            current_a = status.iq_counts * self.iq_count_to_amp

        if current_a is not None and self.torque_constant_nm_per_a is not None:
            torque_nm = (
                current_a
                * self.torque_constant_nm_per_a
                * self.gear_ratio
                * self.gear_efficiency
            )

        position_rad = single_turn_output_rad if self.expose_single_turn_position else None

        return ActuatorState(
            position_rad=position_rad,
            velocity_rad_s=velocity_rad_s,
            torque_nm=torque_nm,
            current_a=current_a,
            temperature_c=float(status.temp_c),
            last_feedback_t=time.monotonic(),
            raw={
                "cmd": self.CMD_MIT_CONTROL,
                "iq_counts": status.iq_counts,
                "speed_dps": status.speed_dps,
                "enc_u16": status.enc_u16,
                "single_turn_motor_rad": single_turn_motor_rad,
                "single_turn_output_rad": single_turn_output_rad,
            },
        )

    def _decode_encoder_data(self, payload8: bytes) -> ActuatorState:
        enc = self._parse_encoder_data(payload8)

        position_rad = None

        if self.expose_single_turn_position:
            position_rad = signed_u14_count_to_rad(enc.encoder_position_u16) / self.gear_ratio

        return ActuatorState(
            position_rad=position_rad,
            temperature_c=float(enc.temp_c),
            last_feedback_t=time.monotonic(),
            raw={
                "cmd": self.CMD_READ_ENCODER_DATA,
                "encoder_position_u16": enc.encoder_position_u16,
                "encoder_original_u16": enc.encoder_original_u16,
                "encoder_offset_u16": enc.encoder_offset_u16,
                "encoder_position_rad_signed_motor": signed_u14_count_to_rad(
                    enc.encoder_position_u16
                ),
                "encoder_original_rad_motor": u14_count_to_rad(
                    enc.encoder_original_u16
                ),
                "encoder_offset_rad_motor": u14_count_to_rad(
                    enc.encoder_offset_u16
                ),
            },
        )

    def _decode_ack_like_frame(self, payload8: bytes) -> ActuatorState:
        if len(payload8) != 8:
            raise ValueError("SPG ACK payload must be 8 bytes")

        cmd = payload8[0]
        offset_u16 = struct.unpack("<H", payload8[6:8])[0]

        return ActuatorState(
            last_feedback_t=time.monotonic(),
            raw={
                "cmd": cmd,
                "ack_offset_u16": offset_u16,
                "ack_offset_rad_motor": u14_count_to_rad(offset_u16),
            },
        )

    @staticmethod
    def _parse_status_common(payload8: bytes, expected_cmd: int) -> SPGMotorStatus:
        if len(payload8) != 8 or payload8[0] != expected_cmd:
            raise ValueError("Invalid SPG status response")

        temp_c = struct.unpack("b", payload8[1:2])[0]
        iq_counts = struct.unpack("<h", payload8[2:4])[0]
        speed_dps = struct.unpack("<h", payload8[4:6])[0]
        enc_u16 = struct.unpack("<H", payload8[6:8])[0]

        return SPGMotorStatus(
            temp_c=temp_c,
            iq_counts=iq_counts,
            speed_dps=speed_dps,
            enc_u16=wrap_u14(enc_u16),
        )

    @staticmethod
    def _parse_encoder_data(payload8: bytes) -> SPGEncoderData:
        if len(payload8) != 8 or payload8[0] != SPGActuatorProtocol.CMD_READ_ENCODER_DATA:
            raise ValueError("Invalid SPG encoder data response")

        temp_c = struct.unpack("b", payload8[1:2])[0]
        encoder_position_u16 = struct.unpack("<H", payload8[2:4])[0]
        encoder_original_u16 = struct.unpack("<H", payload8[4:6])[0]
        encoder_offset_u16 = struct.unpack("<H", payload8[6:8])[0]

        return SPGEncoderData(
            temp_c=temp_c,
            encoder_position_u16=wrap_u14(encoder_position_u16),
            encoder_original_u16=wrap_u14(encoder_original_u16),
            encoder_offset_u16=wrap_u14(encoder_offset_u16),
        )

    # ------------------------------------------------------------------
    # Encode helpers
    # ------------------------------------------------------------------

    def _pack_mit_payload(
        self,
        position_rad: float,
        velocity_rad_s: float,
        kp: float,
        kd: float,
        torque_ff_nm: float,
    ) -> bytes:
        cfg = self.mit_config

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
        offset_raw = int(round(offset_deg * 100.0))

        if not (-32768 <= offset_raw <= 32767):
            raise ValueError(
                f"offset_deg={offset_deg} is out of int16 range "
                "for 0.01 deg/LSB encoding"
            )

        data = bytearray(8)
        data[0] = self.CMD_MIT_SET_ZERO

        offset_bytes = offset_raw.to_bytes(2, byteorder="little", signed=True)
        data[6] = offset_bytes[0]
        data[7] = offset_bytes[1]

        return bytes(data)