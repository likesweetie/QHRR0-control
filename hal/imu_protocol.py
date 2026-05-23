import socket
import struct
import time
import signal
import math
from dataclasses import dataclass
from typing import Dict, Optional, Set, Tuple

IFACE = "can0"
FRAME_FMT = "=IB3x8s"
CAN_RAW_FILTER = 1

g_run = True


def sigint_handler(signum, frame):
    global g_run
    g_run = False


def open_can(iface: str) -> socket.socket:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    return s

def set_can_filters(sock: socket.socket, can_ids: Tuple[int, ...]) -> None:
    flt = b"".join(struct.pack("=II", can_id, 0x7FF) for can_id in can_ids)
    sock.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_FILTER, flt)

def flush_rx(sock: socket.socket, max_frames: int = 256) -> int:
    flushed = 0
    for _ in range(max_frames):
        rx = recv_frame(sock, timeout=0.0)
        if rx is None:
            break
        flushed += 1
    return flushed

def send_frame(sock: socket.socket, can_id: int, data: bytes) -> None:
    if len(data) > 8:
        raise ValueError("CAN payload must be <= 8 bytes")
    payload = data.ljust(8, b"\x00")
    frame = struct.pack(FRAME_FMT, can_id, len(data), payload)
    sock.send(frame)


def recv_frame(sock: socket.socket, timeout: float = 0.0) -> Optional[Tuple[int, int, bytes]]:
    sock.settimeout(timeout)
    try:
        frame = sock.recv(16)
    except (TimeoutError, OSError):
        return None

    can_id, dlc, data = struct.unpack(FRAME_FMT, frame)
    can_id &= socket.CAN_EFF_MASK
    return can_id, dlc, data[:dlc]


@dataclass
class QuaternionFrame:
    qz: float
    qy: float
    qx: float
    qw: float
    """
    due to E2Box IMU convention,
    first, we need to reflect y and z, and swap roll(y) and pitch(x) to match with typical robot convention roll(x) pitch(y)
    """
    def __post_init__(self):
        self.qx = -self.qx
        
    def norm(self) -> float:
        return math.sqrt(self.qz**2 + self.qy**2 + self.qx**2 + self.qw**2)

    def as_xyzw(self) -> Tuple[float, float, float, float]:
        """
        Convert stored order [qz, qy, qx, qw]
        to standard math order [qx, qy, qz, qw].
        """
        #Here we reflect y and z which means 180 degree rotated x
        return self.qx, self.qy, self.qz, self.qw  #this is intentional!

    def normalized_xyzw(self) -> Tuple[float, float, float, float]:
        qx, qy, qz, qw = self.as_xyzw()
        n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if n < 1e-12:
            return 0.0, 0.0, 0.0, 1.0
        return qx / n, (qy / n), (qz / n), qw / n

    def projected_gravity(self) -> Tuple[float, float, float]:
        """
        Compute projected gravity in body frame.

        Definition:
            g_b = R(q)^T * g_w
        with
            g_w = [0, 0, -1]

        Returns:
            (gx_b, gy_b, gz_b)
        """
        qx, qy, qz, qw = self.normalized_xyzw()


        # world gravity (unit vector)
        vx, vy, vz = 0.0, 0.0, -1.0

        # Rotate world vector into body frame using inverse quaternion:
        # v_body = q_conj * v_world * q
        # Optimized quaternion-vector inverse rotation
        # q_vec = [qx, qy, qz], q_w = qw

        # t = 2 * cross(q_vec, v)
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)

        # v' = v - qw * t + cross(q_vec, t)
        # note: inverse rotation uses q_conj, so sign differs from forward rotate
        vpx = vx - qw * tx + (qy * tz - qz * ty)
        vpy = vy - qw * ty + (qz * tx - qx * tz)
        vpz = vz - qw * tz + (qx * ty - qy * tx)
        #here and swap x and y
        return vpy, vpx, vpz #this is intentional!!!
    
@dataclass
class GyroFrame:
    gx_dps: float
    gy_dps: float
    gz_dps: float
    reserved: int = 0

    def __post_init__(self):
        """
        Due to E2Box IMU convention,
        swap x/y so that:
          - IMU pitch(x) -> robot gy
          - IMU roll(y)  -> robot gx
        """
        self.gx_dps, self.gy_dps = self.gy_dps, self.gx_dps
        
@dataclass
class IMUState:
    quat: Optional[QuaternionFrame] = None
    gyro: Optional[GyroFrame] = None
    last_quat_t: float = 0.0
    last_gyro_t: float = 0.0


class IMUCanClient:
    REQ_ID = 0x221
    QUAT_ID = 0x2A1
    GYRO_ID = 0x321

    CMD_GET_QUAT = 0x01
    CMD_GET_GYRO = 0x02
    CMD_GET_ALL = 0x03

    def __init__(self, iface: str = IFACE):
        self.iface = iface
        self.sock = open_can(iface)
        set_can_filters(self.sock, (self.QUAT_ID, self.GYRO_ID))
        self.state = IMUState()

    def close(self):
        self.sock.close()

    def send_request(self, cmd: int) -> None:
        send_frame(self.sock, self.REQ_ID, bytes([cmd]))

    def request_quat(self) -> None:
        self.send_request(self.CMD_GET_QUAT)

    def request_gyro(self) -> None:
        self.send_request(self.CMD_GET_GYRO)

    def request_all(self) -> None:
        self.send_request(self.CMD_GET_ALL)

    @staticmethod
    def parse_quaternion_payload(data: bytes) -> QuaternionFrame:
        if len(data) != 8:
            raise ValueError(f"Quaternion payload must be 8 bytes, got {len(data)}")
        qz_raw, qy_raw, qx_raw, qw_raw = struct.unpack("<hhhh", data)
        # print(qz_raw, qy_raw, qx_raw, qw_raw)
        return QuaternionFrame(
            qz=qz_raw / 10000.0,
            qy=qy_raw / 10000.0,
            qx=qx_raw / 10000.0,
            qw=qw_raw / 10000.0,
        )

    @staticmethod
    def parse_gyro_payload(data: bytes) -> GyroFrame:
        if len(data) != 8:
            raise ValueError(f"Gyro payload must be 8 bytes, got {len(data)}")
        gx_raw, gy_raw, gz_raw, reserved = struct.unpack("<hhhh", data)
        #here we reverse x and y
        return GyroFrame(
            gx_dps= (gx_raw / 100.0)*(3.141592/180), #this is intentional!!
            gy_dps= (gy_raw / 100.0)*(3.141592/180), #this is intentional!!
            gz_dps= (gz_raw / 100.0)*(3.141592/180),
            reserved=reserved,
        )

    def recv_one(self, timeout: float = 0.0) -> Optional[Tuple[int, object]]:
        rx = recv_frame(self.sock, timeout=timeout)
        if rx is None:
            return None

        can_id, dlc, data = rx

        if can_id == self.QUAT_ID:
            quat = self.parse_quaternion_payload(data)
            self.state.quat = quat
            self.state.last_quat_t = time.time()
            return can_id, quat

        if can_id == self.GYRO_ID:
            gyro = self.parse_gyro_payload(data)
            self.state.gyro = gyro
            self.state.last_gyro_t = time.time()
            return can_id, gyro

        return can_id, data

    def recv_until_timeout(self, timeout_each: float = 0.01, max_frames: int = 64) -> Dict[int, object]:
        got: Dict[int, object] = {}
        for _ in range(max_frames):
            item = self.recv_one(timeout=timeout_each)
            if item is None:
                break
            can_id, obj = item
            got[can_id] = obj
        return got

    def drain_rx_buffer(
        self,
        max_frames: int = 64,
        expected_ids: Optional[Set[int]] = None,
        timeout=0.0,
    ) -> Tuple[IMUState, int]:
        latest = IMUState()
        recv_count = 0
        expected_id_set = (
            expected_ids
            if expected_ids is not None
            else {self.QUAT_ID, self.GYRO_ID}
        )

        for _ in range(max_frames):
            rx = recv_frame(self.sock, timeout=timeout)
            if rx is None:
                break

            can_id, dlc, data = rx
            if can_id not in expected_id_set:
                continue

            if dlc != 8 or len(data) != 8:
                continue
            try:
                if can_id == self.QUAT_ID:
                    quat = self.parse_quaternion_payload(data)
                    now_t = time.time()
                    self.state.quat = quat
                    self.state.last_quat_t = now_t
                    latest.quat = quat
                    latest.last_quat_t = now_t
                    recv_count += 1
                elif can_id == self.GYRO_ID:
                    gyro = self.parse_gyro_payload(data)
                    now_t = time.time()
                    self.state.gyro = gyro
                    self.state.last_gyro_t = now_t
                    latest.gyro = gyro
                    latest.last_gyro_t = now_t
                    recv_count += 1
            except ValueError:
                continue

        return latest, recv_count

    def request_and_wait(
        self,
        cmd: int,
        timeout_total: float = 0.2,
    ) -> IMUState:
        self.send_request(cmd)
        t0 = time.perf_counter()

        need_quat = cmd in (self.CMD_GET_QUAT, self.CMD_GET_ALL)
        need_gyro = cmd in (self.CMD_GET_GYRO, self.CMD_GET_ALL)

        got_quat = False
        got_gyro = False

        while (time.perf_counter() - t0) < timeout_total:
            item = self.recv_one(timeout=0.005)
            if item is None:
                continue

            can_id, obj = item
            if can_id == self.QUAT_ID:
                got_quat = True
            elif can_id == self.GYRO_ID:
                got_gyro = True

            if ((not need_quat or got_quat) and
                (not need_gyro or got_gyro)):
                break

        return self.state
