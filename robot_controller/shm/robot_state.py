from __future__ import annotations

import ctypes
import time
from typing import Any

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
        ("quat_xyzw", ctypes.c_float * 4),
        ("projected_gravity_b", ctypes.c_float * 3),
        ("angular_velocity_rad_s", ctypes.c_float * 3),
        ("last_quat_t", ctypes.c_double),
        ("last_gyro_t", ctypes.c_double),
        ("quat_online", ctypes.c_uint8),
        ("gyro_online", ctypes.c_uint8),
        ("quat_stale", ctypes.c_uint8),
        ("gyro_stale", ctypes.c_uint8),
    ]

    def quat_wxyz(self) -> tuple[float, float, float, float]:
        quat = self.quat_xyzw
        return (
            float(quat[3]),
            float(quat[0]),
            float(quat[1]),
            float(quat[2]),
        )


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

    def read_latest(self) -> dict[str, Any] | None:
        state = self.read_relaxed()
        if not state.is_initialized():
            return None
        return robot_state_to_dict(state)


def robot_state_to_dict(state: RobotStateC) -> dict[str, Any]:
    mode_name = _mode_name(int(state.controller_mode))
    imu = state.imu
    command_output = state.command_output
    return {
        "schema": "qhrr.robot_state.cstruct.v1",
        "timestamp_monotonic": float(state.timestamp_monotonic),
        "timestamp_unix": float(state.timestamp_unix),
        "controller_state": mode_name,
        "imu": {
            "quat_xyzw": [float(value) for value in imu.quat_xyzw],
            "projected_gravity_b": [float(value) for value in imu.projected_gravity_b],
            "angular_velocity_rad_s": [float(value) for value in imu.angular_velocity_rad_s],
            "last_quat_t": float(imu.last_quat_t),
            "last_gyro_t": float(imu.last_gyro_t),
            "quat_online": bool(imu.quat_online),
            "gyro_online": bool(imu.gyro_online),
            "quat_stale": bool(imu.quat_stale),
            "gyro_stale": bool(imu.gyro_stale),
        },
        "actuators": [
            _actuator_to_dict(item)
            for item in state.valid_actuators()
        ],
        "command_output": _command_output_to_dict(command_output),
    }


def _actuator_to_dict(item: ActuatorStateC) -> dict[str, Any]:
    has_feedback = float(item.last_feedback_t) > 0.0
    return {
        "can_id": int(item.can_id),
        "position_rad": float(item.position_rad) if has_feedback else None,
        "velocity_rad_s": float(item.velocity_rad_s) if has_feedback else None,
        "torque_nm": float(item.torque_nm) if has_feedback else None,
        "current_a": float(item.current_a) if has_feedback else None,
        "temperature_c": float(item.temperature_c) if has_feedback else None,
        "fault_code": None if int(item.fault_code) < 0 else int(item.fault_code),
        "is_enabled": None if int(item.is_enabled) < 0 else bool(item.is_enabled),
        "last_feedback_t": float(item.last_feedback_t),
        "age_s": None if float(item.age_s) < 0.0 else float(item.age_s),
        "online": bool(item.online),
        "stale": bool(item.stale),
    }


def _command_output_to_dict(item: CommandOutputStateC) -> dict[str, Any]:
    target_count = item.valid_target_count()
    timestamp_monotonic = float(item.timestamp_monotonic)
    age_s = None
    if timestamp_monotonic > 0.0:
        age_s = max(0.0, time.monotonic() - timestamp_monotonic)
    return {
        "status": "online" if timestamp_monotonic > 0.0 else "waiting",
        "source": COMMAND_OUTPUT_SOURCE_NAMES.get(int(item.source), f"SOURCE_{int(item.source)}"),
        "timestamp_monotonic": timestamp_monotonic,
        "age_s": age_s,
        "target_count": target_count,
        "targets": [
            _command_target_to_dict(target)
            for target in item.valid_targets()
        ],
        "error": None,
    }


def _command_target_to_dict(item: CommandTargetStateC) -> dict[str, Any]:
    can_id = int(item.can_id)
    return {
        "can_id": f"0x{can_id:X}",
        "p_target_rad": float(item.p_target_rad),
        "v_target_rad_s": float(item.v_target_rad_s),
        "kp": float(item.kp),
        "kd": float(item.kd),
        "tau_target_nm": float(item.tau_target_nm),
    }


def _mode_name(value: int) -> str:
    names = {
        0: "DISABLED",
        1: "ENABLING",
        2: "NORMAL",
        3: "DAMPING",
        4: "ZERO_SETTING",
        5: "ESTOP",
    }
    return names.get(value, f"MODE_{value}")


def new_robot_state(mode: int) -> RobotStateC:
    state = RobotStateC()
    state.timestamp_ns = time.time_ns()
    state.timestamp_monotonic = time.monotonic()
    state.timestamp_unix = time.time()
    state.controller_mode = int(mode)
    return state


RobotStateShmReader = RobotStateShm
RobotStateShmWriter = RobotStateShm
