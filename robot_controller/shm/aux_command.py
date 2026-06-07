from __future__ import annotations

import ctypes
import time

from robot_controller.shm.cstruct import CStructShm


class AuxCommandC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("lin_vel_target", ctypes.c_float * 3),
        ("ang_vel_target", ctypes.c_float * 3),
        ("button_mask", ctypes.c_uint32),
    ]


AUX_COMMAND_SIZE = ctypes.sizeof(AuxCommandC)


BUTTON_FIELDS = (
    "a_button",
    "b_button",
    "x_button",
    "y_button",
    "lb_button",
    "rb_button",
    "back_button",
    "start_button",
    "guide_button",
    "l3_button",
    "r3_button",
)


class AuxCommandShm(CStructShm[AuxCommandC]):
    struct_type = AuxCommandC

    def publish(
        self,
        lin_vel_target: list[float],
        ang_vel_target: list[float],
        buttons: dict[str, bool],
    ) -> int:
        command = AuxCommandC()
        command.timestamp_ns = time.time_ns()
        for index in range(3):
            command.lin_vel_target[index] = float(lin_vel_target[index])
            command.ang_vel_target[index] = float(ang_vel_target[index])
        command.button_mask = buttons_to_mask(buttons)
        self.write(command)
        return int(command.timestamp_ns)


def buttons_to_mask(buttons: dict[str, bool]) -> int:
    mask = 0
    for index, name in enumerate(BUTTON_FIELDS):
        if bool(buttons.get(name, False)):
            mask |= 1 << index
    return mask


def mask_to_buttons(mask: int) -> dict[str, bool]:
    return {
        name: bool(int(mask) & (1 << index))
        for index, name in enumerate(BUTTON_FIELDS)
    }
