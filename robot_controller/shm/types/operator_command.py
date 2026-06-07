from __future__ import annotations

import ctypes
from enum import IntEnum

from robot_controller.shm.cstruct_type import CStructShm


class OperatorCommandCode(IntEnum):
    NONE = 0
    ENABLE = 1
    DISABLE = 2
    DAMPING = 3
    ZERO_SET = 4
    ESTOP = 5
    RESET_FAULT = 6
    RUN = 7


OPERATOR_ZERO_TARGET_CAPACITY = 12
OPERATOR_ZERO_TARGET_MAGIC = 0x5A45524F


class OperatorZeroTargetC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("can_id", ctypes.c_uint32),
        ("offset_count", ctypes.c_int16),
        ("reserved", ctypes.c_uint16),
    ]


class OperatorCommandC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("command", ctypes.c_uint32),
        ("target_mask", ctypes.c_uint32),
        ("zero_target_count", ctypes.c_uint32),
        ("zero_target_magic", ctypes.c_uint32),
        ("zero_targets", OperatorZeroTargetC * OPERATOR_ZERO_TARGET_CAPACITY),
    ]

    def valid_zero_target_count(self) -> int:
        return max(0, min(int(self.zero_target_count), OPERATOR_ZERO_TARGET_CAPACITY))

    def valid_zero_targets(self) -> tuple[OperatorZeroTargetC, ...]:
        return tuple(self.zero_targets[: self.valid_zero_target_count()])


OPERATOR_COMMAND_SIZE = ctypes.sizeof(OperatorCommandC)


class OperatorCommandShm(CStructShm[OperatorCommandC]):
    struct_type = OperatorCommandC
