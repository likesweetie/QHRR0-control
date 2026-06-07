from __future__ import annotations

import ctypes
import time

from robot_controller.shm.cstruct_type import CStructShm


MAX_ROBOT_STATE_ACTUATORS = 12
COMMAND_OUTPUT_SOURCE_NAMES = {
    0: "NONE",
    1: "ENABLE",
    2: "DISABLE",
    3: "ZERO_SET",
    4: "DAMPING",
    5: "POLICY",
}
COMMAND_OUTPUT_SOURCE_VALUES = {
    name: value
    for value, name in COMMAND_OUTPUT_SOURCE_NAMES.items()
}


class ActuatorStateC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("can_id", ctypes.c_uint32),
        ("position_rad", ctypes.c_float),
        ("velocity_rad_s", ctypes.c_float),
        ("torque_nm", ctypes.c_float),
        ("current_a", ctypes.c_float),
        ("temperature_c", ctypes.c_float),
        ("fault_code", ctypes.c_int32),
        ("is_enabled", ctypes.c_int32),
        ("last_feedback_t", ctypes.c_double),
        ("age_s", ctypes.c_float),
        ("online", ctypes.c_uint8),
        ("stale", ctypes.c_uint8),
        ("_pad", ctypes.c_uint8 * 2),
    ]


class ImuStateC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("quat_wxyz", ctypes.c_float * 4),
        ("projected_gravity_b", ctypes.c_float * 3),
        ("angular_velocity_rad_s", ctypes.c_float * 3),
        ("last_quat_t", ctypes.c_double),
        ("last_gyro_t", ctypes.c_double),
        ("quat_online", ctypes.c_uint8),
        ("gyro_online", ctypes.c_uint8),
        ("quat_stale", ctypes.c_uint8),
        ("gyro_stale", ctypes.c_uint8),
    ]


class CommandTargetStateC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("can_id", ctypes.c_uint32),
        ("p_target_rad", ctypes.c_float),
        ("v_target_rad_s", ctypes.c_float),
        ("kp", ctypes.c_float),
        ("kd", ctypes.c_float),
        ("tau_target_nm", ctypes.c_float),
    ]


class CommandOutputStateC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_monotonic", ctypes.c_double),
        ("source", ctypes.c_uint32),
        ("target_count", ctypes.c_uint32),
        ("targets", CommandTargetStateC * MAX_ROBOT_STATE_ACTUATORS),
    ]

    def valid_target_count(self) -> int:
        return max(0, min(int(self.target_count), MAX_ROBOT_STATE_ACTUATORS))

    def valid_targets(self) -> tuple[CommandTargetStateC, ...]:
        return tuple(self.targets[: self.valid_target_count()])


class RobotStateC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("timestamp_monotonic", ctypes.c_double),
        ("timestamp_unix", ctypes.c_double),
        ("controller_mode", ctypes.c_uint32),
        ("actuator_count", ctypes.c_uint32),
        ("imu", ImuStateC),
        ("actuators", ActuatorStateC * MAX_ROBOT_STATE_ACTUATORS),
        ("command_output", CommandOutputStateC),
    ]

    def is_initialized(self) -> bool:
        return int(self.timestamp_ns) != 0

    def valid_actuator_count(self) -> int:
        return max(0, min(int(self.actuator_count), MAX_ROBOT_STATE_ACTUATORS))

    def valid_actuators(self) -> tuple[ActuatorStateC, ...]:
        return tuple(self.actuators[: self.valid_actuator_count()])

    def actuator_by_can_id(self, can_id: int) -> ActuatorStateC | None:
        can_id_int = int(can_id)
        for actuator in self.valid_actuators():
            if int(actuator.can_id) == can_id_int:
                return actuator
        return None


ROBOT_STATE_SIZE = ctypes.sizeof(RobotStateC)


class RobotStateShm(CStructShm[RobotStateC]):
    struct_type = RobotStateC


def new_robot_state(mode: int) -> RobotStateC:
    state = RobotStateC()
    state.timestamp_ns = time.time_ns()
    state.timestamp_monotonic = time.monotonic()
    state.timestamp_unix = time.time()
    state.controller_mode = int(mode)
    return state


RobotStateShmReader = RobotStateShm
RobotStateShmWriter = RobotStateShm
