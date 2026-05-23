import math
import struct
import time

from hal.can_bus import CANFrame
from hal.hardware.can.imu import IMUProtocolBase
from hal.hardware.can.imu import IMUState
from .robot_state import RobotPoseState


class E2BoxIMUProtocol(IMUProtocolBase):
    REQ_ID = 0x221
    QUAT_ID = 0x2A1
    GYRO_ID = 0x321

    CMD_GET_QUAT = 0x01
    CMD_GET_GYRO = 0x02
    CMD_GET_ALL = 0x03

    def rx_can_ids(self) -> list[int]:
        return [self.QUAT_ID, self.GYRO_ID]

    def encode_request_quat(self) -> CANFrame:
        return CANFrame(can_id=self.REQ_ID, data=bytes([self.CMD_GET_QUAT]))

    def encode_request_gyro(self) -> CANFrame:
        return CANFrame(can_id=self.REQ_ID, data=bytes([self.CMD_GET_GYRO]))

    def encode_request_all(self) -> CANFrame:
        return CANFrame(can_id=self.REQ_ID, data=bytes([self.CMD_GET_ALL]))

    def decode_frame(self, frame: CANFrame) -> RobotPoseState | None:
        if frame.can_id == self.QUAT_ID:
            return self._decode_quat(frame.data)

        if frame.can_id == self.GYRO_ID:
            return self._decode_gyro(frame.data)

        return None

    def _decode_quat(self, data: bytes) -> RobotPoseState:
        if len(data) != 8:
            raise ValueError(f"Quaternion payload must be 8 bytes, got {len(data)}")

        qz_raw, qy_raw, qx_raw, qw_raw = struct.unpack("<hhhh", data)

        qz = qz_raw / 10000.0
        qy = qy_raw / 10000.0
        qx = qx_raw / 10000.0
        qw = qw_raw / 10000.0

        # E2Box convention correction.
        qx = -qx

        quat_xyzw = self._normalize_quat((qx, qy, qz, qw))
        projected_gravity_b = self._projected_gravity_from_xyzw(quat_xyzw)

        return RobotPoseState(
            quat_xyzw=quat_xyzw,
            projected_gravity_b=projected_gravity_b,
            last_quat_t=time.monotonic(),
        )

    def _decode_gyro(self, data: bytes) -> RobotPoseState:
        if len(data) != 8:
            raise ValueError(f"Gyro payload must be 8 bytes, got {len(data)}")

        gx_raw, gy_raw, gz_raw, _reserved = struct.unpack("<hhhh", data)

        gx = (gx_raw / 100.0) * math.pi / 180.0
        gy = (gy_raw / 100.0) * math.pi / 180.0
        gz = (gz_raw / 100.0) * math.pi / 180.0

        # E2Box convention correction: swap x/y.
        gx, gy = gy, gx

        return RobotPoseState(
            angular_velocity_rad_s=(gx, gy, gz),
            last_gyro_t=time.monotonic(),
        )

    @staticmethod
    def _normalize_quat(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        qx, qy, qz, qw = q
        n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if n < 1e-12:
            return 0.0, 0.0, 0.0, 1.0
        return qx / n, qy / n, qz / n, qw / n

    @staticmethod
    def _projected_gravity_from_xyzw(
        q: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        qx, qy, qz, qw = q

        vx, vy, vz = 0.0, 0.0, -1.0

        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)

        vpx = vx - qw * tx + (qy * tz - qz * ty)
        vpy = vy - qw * ty + (qz * tx - qx * tz)
        vpz = vz - qw * tz + (qx * ty - qy * tx)

        # !!! E2Box/robot convention correction. !!!
        return vpy, vpx, vpz #becareful for the order