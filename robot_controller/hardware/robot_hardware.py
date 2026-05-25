from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from robot_controller.core.config import RobotControllerConfig

from .can_transport import CanTransport
from .imu_bus import ImuBus, ImuFeedback
from .motor_bus import MotorBus, MotorFeedback


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class RobotFeedback:
    motors: MotorFeedback
    imu: ImuFeedback


@dataclass(frozen=True)
class HardwareStatus:
    can_connected: bool


class RobotHardware:
    def __init__(self, config: RobotControllerConfig):
        self.config = config
        self.transport = CanTransport(config)
        self._motors = MotorBus(config, self.transport)
        self._imu = ImuBus(config, self.transport)

    @property
    def motors(self) -> MotorBus:
        return self._motors

    @property
    def imu(self) -> ImuBus:
        return self._imu

    def connect_can_daemon(self) -> None:
        self.transport.connect()
        self.motors.register_callbacks()
        self.imu.register_callbacks()
        registered_ids = self.transport.registered_ids()
        logger.info(
            "Registered CAN RX callbacks: %s",
            ", ".join(f"0x{can_id:03X}" for can_id in registered_ids),
        )

    def bringup(self) -> None:
        self.imu.bringup()
        self.motors.bringup()

    def read_feedback(self, now: float | None = None) -> RobotFeedback:
        now = time.monotonic() if now is None else now
        return RobotFeedback(
            motors=self.motors.read_feedback(now),
            imu=self.imu.read_feedback(now),
        )

    def status(self) -> HardwareStatus:
        return HardwareStatus(can_connected=self.transport.is_connected())

    def close(self) -> None:
        self.transport.close()
