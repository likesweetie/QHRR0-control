import logging
import threading
from copy import deepcopy

from hal.can_bus import CANFrame
from hal.hardware.can.device_driver import CANDeviceDriverBase
from hal.hardware.can.device_comm_manager import BestEffortCommManager

from imu import IMUProtocolBase
from imu import RobotPoseState


logger = logging.getLogger(__name__)


class IMUDriver(CANDeviceDriverBase):
    """
    Generic CAN IMU driver.

    Product-specific details should be implemented in IMUProtocolBase subclasses.
    This driver only manages:
    - RX callback
    - state update
    - freshness / stale status
    - request frame generation
    """

    def __init__(
        self,
        name: str,
        protocol: IMUProtocolBase,
        quat_timeout: float,
        gyro_timeout: float,
    ):
        super().__init__(name=name, protocol=protocol)

        self.quat_comm = BestEffortCommManager(timeout=quat_timeout)
        self.gyro_comm = BestEffortCommManager(timeout=gyro_timeout)

        self._state = RobotPoseState()
        self._lock = threading.Lock()

    def rx_can_ids(self) -> list[int]:
        return self.protocol.rx_can_ids()

    def make_request_quat_frame(self) -> CANFrame:
        return self.protocol.encode_request_quat()

    def make_request_gyro_frame(self) -> CANFrame:
        return self.protocol.encode_request_gyro()

    def make_request_all_frame(self) -> CANFrame:
        return self.protocol.encode_request_all()

    def on_frame(self, frame: CANFrame) -> None:
        try:
            partial_state = self.protocol.decode_frame(frame)
        except Exception:
            logger.exception(
                "[%s] Failed to decode IMU frame: can_id=0x%X",
                self.name,
                frame.can_id,
            )
            self._mark_decode_error(frame.can_id)
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
                self.quat_comm.mark_rx(partial_state.last_quat_t)

            if partial_state.angular_velocity_rad_s is not None:
                self._state.angular_velocity_rad_s = partial_state.angular_velocity_rad_s
                self._state.last_gyro_t = partial_state.last_gyro_t
                self.gyro_comm.mark_rx(partial_state.last_gyro_t)

    def _mark_decode_error(self, can_id: int) -> None:
        if can_id in self.protocol.rx_can_ids():
            # 정확히 quat/gyro ID를 구분하고 싶으면 protocol에 is_quat_id(), is_gyro_id()를 추가하는 것이 좋습니다.
            self.quat_comm.mark_decode_error()
            self.gyro_comm.mark_decode_error()

    def update_fault_flags(self) -> None:
        self.quat_comm.update_fault_flags()
        self.gyro_comm.update_fault_flags()

    def is_fresh(self) -> bool:
        quat_status = self.quat_comm.update_fault_flags()
        gyro_status = self.gyro_comm.update_fault_flags()

        return (not quat_status.is_stale) and (not gyro_status.is_stale)

    def get_state(self) -> RobotPoseState:
        with self._lock:
            return deepcopy(self._state)

    def get_comm_status(self):
        return {
            "quat": self.quat_comm.get_status(),
            "gyro": self.gyro_comm.get_status(),
        }