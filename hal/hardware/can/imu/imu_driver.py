import logging
import threading
from copy import deepcopy

from hal.can_bus import CANFrame, CANDaemon
from imu import IMUProtocolBase
from imu import IMUState, RobotPoseState


logger = logging.getLogger(__name__)


class CANIMUDriver:
    """
    Generic IMU driver for CAN-based IMU protocols.

    Responsibilities:
    - Own IMU protocol
    - Register CAN RX callbacks to CANDaemon
    - Send IMU request frames through CANDaemon
    - Decode received CAN frames using protocol
    - Maintain latest robot pose state
    """

    def __init__(
        self,
        name: str,
        protocol: IMUProtocolBase,
    ):
        self.name = name
        self.protocol = protocol

        self._state = RobotPoseState()
        self._lock = threading.Lock()

        self._quat_event = threading.Event()
        self._gyro_event = threading.Event()

    def rx_can_ids(self) -> list[int]:
        return self.protocol.rx_can_ids()

    def make_request_quat_frame(self) -> CANFrame:
        self._quat_event.clear()
        return self.protocol.encode_request_quat()

    def make_request_gyro_frame(self) -> CANFrame:
        self._gyro_event.clear()
        return self.protocol.encode_request_gyro()

    def make_request_all_frame(self) -> CANFrame:
        self._quat_event.clear()
        self._gyro_event.clear()
        return self.protocol.encode_request_all()

    def on_frame(self, frame: CANFrame) -> None:
        """
        Callback registered to CANDaemon dispatcher.
        """
        try:
            partial_state = self.protocol.decode_frame(frame)
        except Exception:
            logger.exception(
                "[%s] Failed to decode IMU frame: can_id=0x%X",
                self.name,
                frame.can_id,
            )
            return

        if partial_state is None:
            return

        self._merge_state(partial_state)

    def _merge_state(self, partial_state: RobotPoseState) -> None:
        with self._lock:
            if partial_state.quat_xyzw is not None:
                self._state.quat_xyzw = partial_state.quat_xyzw
                self._state.projected_gravity_b = partial_state.projected_gravity_b
                self._state.last_quat_t = partial_state.last_quat_t
                self._quat_event.set()

            if partial_state.angular_velocity_rad_s is not None:
                self._state.angular_velocity_rad_s = partial_state.angular_velocity_rad_s
                self._state.last_gyro_t = partial_state.last_gyro_t
                self._gyro_event.set()

    def get_state(self) -> RobotPoseState:
        with self._lock:
            return deepcopy(self._state)

    def wait_for_quat(self, timeout: float = 0.1) -> bool:
        return self._quat_event.wait(timeout=timeout)

    def wait_for_gyro(self, timeout: float = 0.1) -> bool:
        return self._gyro_event.wait(timeout=timeout)

    def wait_for_all(self, timeout: float = 0.1) -> bool:
        quat_ok = self._quat_event.wait(timeout=timeout)
        gyro_ok = self._gyro_event.wait(timeout=timeout)
        return quat_ok and gyro_ok