from __future__ import annotations

import ctypes

from enum import IntEnum

from qhrr0.app.robot_controller.shm.types.cstruct_type import CStructShm
from .consts import *

class OperatorCommandCode(IntEnum):
    NONE = 0
    ENABLE = 1
    DISABLE = 2
    DAMPING = 3
    ZERO_SET = 4
    ESTOP = 5
    RESET_FAULT = 6
    RUN = 7


class ControlTargetC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("can_id", ctypes.c_uint32),
        ("q", ctypes.c_float),
        ("dq", ctypes.c_float),
        ("kp", ctypes.c_float),
        ("kd", ctypes.c_float),
        ("tau", ctypes.c_float),
    ]

class ControlCommandC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("num_targets", ctypes.c_uint32),
        ("targets", ControlTargetC * MAX_ROBOT_ACTUATORS),
    ]

    def valid_target_count(self) -> int:
        return max(0, min(int(self.num_targets), MAX_ROBOT_ACTUATORS))

    def valid_targets(self) -> tuple[ControlTargetC, ...]:
        return tuple(self.targets[: self.valid_target_count()])


class AuxCommandC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("lin_vel_target", ctypes.c_float * 3),
        ("ang_vel_target", ctypes.c_float * 3),
        ("button_mask", ctypes.c_uint32),
    ]


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
        ("zero_targets", OperatorZeroTargetC * MAX_ROBOT_ACTUATORS),
    ]

    def valid_zero_target_count(self) -> int:
        return max(0, min(int(self.zero_target_count), MAX_ROBOT_ACTUATORS))

    def valid_zero_targets(self) -> tuple[OperatorZeroTargetC, ...]:
        return tuple(self.zero_targets[: self.valid_zero_target_count()])



class AuxCommandShm(CStructShm[AuxCommandC]):
    struct_type = AuxCommandC

class OperatorCommandShm(CStructShm[OperatorCommandC]):
    struct_type = OperatorCommandC

class ControlCommandShm(CStructShm[ControlCommandC]):
    struct_type = ControlCommandC
