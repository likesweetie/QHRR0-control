from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


class RobotControllerState(Enum):
    CREATED = auto()
    INIT_SHM = auto()
    START_CAN_DAEMON = auto()
    BRINGUP_IMU = auto()
    BRINGUP_MOTORS = auto()
    START_CHILD_PROCESSES = auto()
    RUNNING = auto()
    SHUTTING_DOWN = auto()
    STOPPED = auto()
    ERROR = auto()


@dataclass(frozen=True)
class MitTarget:
    can_id: int
    position_rad: float
    velocity_rad_s: float
    kp: float
    kd: float
    torque_ff_nm: float


@dataclass(frozen=True)
class MitCommandBatch:
    source: str
    timestamp: float
    targets: list[MitTarget]
