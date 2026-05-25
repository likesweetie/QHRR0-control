from __future__ import annotations

import time
from dataclasses import dataclass

from hal.hardware.can.imu.driver import IMUDriver

from robot_controller.QHRR0_HW.e2box_imu.e2box_imu_protocol import E2BoxIMUProtocol
from robot_controller.core.config import RobotControllerConfig

from .can_transport import CanTransport


@dataclass(frozen=True)
class ImuFeedback:
    state: object
    quat_comm: object
    gyro_comm: object
    quat_age_s: float | None
    gyro_age_s: float | None
    quat_online: bool
    gyro_online: bool
    quat_stale: bool
    gyro_stale: bool

    @property
    def has_stale_feedback(self) -> bool:
        return self.quat_stale or self.gyro_stale

    @property
    def stale_reason(self) -> str:
        names = []
        if self.quat_stale:
            names.append("quat")
        if self.gyro_stale:
            names.append("gyro")
        return "IMU feedback stale: " + ", ".join(names)


class ImuBus:
    def __init__(self, config: RobotControllerConfig, transport: CanTransport):
        self.config = config
        self.transport = transport
        self.driver = IMUDriver(
            name="e2box_imu",
            protocol=E2BoxIMUProtocol(),
            quat_timeout=self.config.can.command_timeout_s,
            gyro_timeout=self.config.can.command_timeout_s,
        )

    def register_callbacks(self) -> None:
        for can_id in self.driver.rx_can_ids():
            self.transport.register_callback(can_id, self.driver.on_frame)

    def bringup(self) -> None:
        imu_config = self.config.can.imu
        if not imu_config.enabled or not imu_config.request_all_on_start:
            return
        for _ in range(int(imu_config.startup_request_count)):
            self.transport.send_frame(self.driver.make_request_all_frame())
            time.sleep(float(imu_config.startup_request_delay_s))

    def request_on_tick(self, now: float) -> None:
        del now
        imu_config = self.config.can.imu
        if imu_config.enabled and imu_config.request_all_each_tick:
            self.transport.send_frame(self.driver.make_request_all_frame())

    def read_feedback(self, now: float | None = None) -> ImuFeedback:
        now = time.monotonic() if now is None else now
        self.driver.update_fault_flags()
        state = self.driver.get_state()
        comm = self.driver.get_comm_status()
        quat_comm = comm["quat"]
        gyro_comm = comm["gyro"]
        return ImuFeedback(
            state=state,
            quat_comm=quat_comm,
            gyro_comm=gyro_comm,
            quat_age_s=None if state.last_quat_t <= 0.0 else max(0.0, now - state.last_quat_t),
            gyro_age_s=None if state.last_gyro_t <= 0.0 else max(0.0, now - state.last_gyro_t),
            quat_online=bool(quat_comm.is_online),
            gyro_online=bool(gyro_comm.is_online),
            quat_stale=bool(quat_comm.is_stale),
            gyro_stale=bool(gyro_comm.is_stale),
        )

    def get_state(self):
        return self.driver.get_state()
