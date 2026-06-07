from __future__ import annotations

import ctypes

from robot_controller.shm.cstruct_type import CStructShm


MAX_CONTROL_TARGETS = 12


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
        ("targets", ControlTargetC * MAX_CONTROL_TARGETS),
    ]

    def valid_target_count(self) -> int:
        return max(0, min(int(self.num_targets), MAX_CONTROL_TARGETS))

    def valid_targets(self) -> tuple[ControlTargetC, ...]:
        return tuple(self.targets[: self.valid_target_count()])


CONTROL_COMMAND_SIZE = ctypes.sizeof(ControlCommandC)


class ControlCommandShm(CStructShm[ControlCommandC]):
    struct_type = ControlCommandC
