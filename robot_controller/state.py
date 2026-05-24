from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto


class RobotControllerState(Enum):
    CREATED = auto()
    INIT_SHM = auto()
    START_CAN_DAEMON = auto()
    BRINGUP_MOTORS = auto()
    START_CHILD_PROCESSES = auto()
    RUNNING = auto()
    SHUTTING_DOWN = auto()
    STOPPED = auto()
    ERROR = auto()


@dataclass(frozen=True)
class MitTarget:
    motor_id: int
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


@dataclass
class MotorState:
    motor_id: int
    enabled: bool = False
    position_rad: float | None = None
    velocity_rad_s: float | None = None
    torque_nm: float | None = None
    temperature_c: float | None = None
    fault_code: int | None = None


@dataclass
class RobotStateSnapshot:
    timestamp: float
    motor_states: dict[int, MotorState] = field(default_factory=dict)
    imu_state: object | None = None
    can_alive: bool = False
