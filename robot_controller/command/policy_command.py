from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


@dataclass(frozen=True)
class JointCommand:
    can_id: int
    position_rad: float
    velocity_rad_s: float
    kp: float
    kd: float
    torque_ff_nm: float


@dataclass(frozen=True)
class PolicyCommand:
    source: str
    timestamp: float
    targets: list[JointCommand]


JointCommandBatch = PolicyCommand


class CommandReadStatus(Enum):
    AVAILABLE = auto()
    NO_COMMAND = auto()
    READ_COLLISION = auto()
    BAD_FORMAT = auto()
    SHM_UNAVAILABLE = auto()


@dataclass(frozen=True)
class CommandReadResult:
    command: PolicyCommand | None
    status: CommandReadStatus
    reason: str
    timestamp: float | None
