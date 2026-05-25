from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


class SafetyState(Enum):
    CREATED = auto()
    DISARMED = auto()
    ARMING = auto()
    READY = auto()
    RUNNING = auto()
    DAMPING = auto()
    FAULT_LATCHED = auto()
    ESTOP = auto()
    SHUTTING_DOWN = auto()
    STOPPED = auto()


class ControlAction(Enum):
    NO_OUTPUT = auto()
    SEND_POLICY_COMMAND = auto()
    SEND_DAMPING = auto()
    DISABLE_MOTORS = auto()
    SHUTDOWN = auto()


@dataclass(frozen=True)
class SafetyDecision:
    state: SafetyState
    action: ControlAction
    reason: str
    fault_code: str | None = None
