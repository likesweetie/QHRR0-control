from __future__ import annotations

import ctypes

from robot_controller.shm.cstruct_type import CStructShm


class AuxCommandC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("lin_vel_target", ctypes.c_float * 3),
        ("ang_vel_target", ctypes.c_float * 3),
        ("button_mask", ctypes.c_uint32),
    ]


AUX_COMMAND_SIZE = ctypes.sizeof(AuxCommandC)


class AuxCommandShm(CStructShm[AuxCommandC]):
    struct_type = AuxCommandC
