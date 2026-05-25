from __future__ import annotations

import time
from typing import Any, Callable

import numpy as np

from robot_controller.core.robot_state_shm import RobotStateShmReader


class ControlStateNotReady(RuntimeError):
    pass


class ControlStateReader:
    def __init__(self, shm_name: str) -> None:
        self.reader = RobotStateShmReader(shm_name)

    def close(self) -> None:
        self.reader.close()

    def latest(self, read_timeout_s: float) -> dict[str, Any]:
        deadline = time.monotonic() + read_timeout_s
        last_error = "control_state SHM has no readable payload"
        while time.monotonic() < deadline:
            state = self.reader.read_latest()
            if state is None:
                last_error = "control_state SHM read collided with writer"
                time.sleep(0.001)
                continue
            return state
        raise RuntimeError(last_error)

    def latest_ready(self, can_ids: list[int]) -> dict[str, Any]:
        state = self.reader.read_latest()
        if state is None:
            raise ControlStateNotReady("control_state SHM has no committed payload")
        error = control_state_numeric_error(state, can_ids)
        if error is not None:
            raise ControlStateNotReady(f"control_state SHM is not ready for policy input: {error}")
        return state

    def wait_until_available(
        self,
        *,
        poll_period_s: float,
        should_run: Callable[[], bool],
        can_ids: list[int],
    ) -> dict[str, Any]:
        last_error: ControlStateNotReady | None = None
        last_report_t = 0.0
        last_report = ""
        print("[task_controller] waiting for numeric control_state", flush=True)
        while should_run():
            try:
                state = self.latest_ready(can_ids)
                print("[task_controller] numeric control_state received", flush=True)
                return state
            except ControlStateNotReady as exc:
                last_error = exc
                now = time.monotonic()
                message = str(exc)
                if message != last_report or now - last_report_t >= 1.0:
                    print(f"[task_controller] still waiting: {message}", flush=True)
                    last_report = message
                    last_report_t = now
                time.sleep(poll_period_s)
        if last_error is not None:
            raise RuntimeError(f"task_controller stopped while waiting for control_state: {last_error}")
        raise RuntimeError("task_controller stopped while waiting for control_state")


class AuxCommandReader:
    def __init__(self, shm_name: str) -> None:
        self.reader = RobotStateShmReader(shm_name)

    def close(self) -> None:
        self.reader.close()

    def latest(self, read_timeout_s: float) -> tuple[list[float], list[float], dict[str, bool]]:
        deadline = time.monotonic() + read_timeout_s
        state = None
        while time.monotonic() < deadline:
            state = self.reader.read_latest()
            if state is not None:
                break
            time.sleep(0.001)
        if state is None:
            raise RuntimeError("aux command SHM has no readable payload")
        return (
            [float(value) for value in state["lin_vel_target"]],
            [float(value) for value in state["ang_vel_target"]],
            {str(key): bool(value) for key, value in state.get("buttons", {}).items()},
        )


def state_vectors(control_state: dict[str, Any], can_ids: list[int]) -> tuple[np.ndarray, np.ndarray, list[float], np.ndarray]:
    error = control_state_numeric_error(control_state, can_ids)
    if error is not None:
        raise RuntimeError(error)
    actuators = {
        int(item["can_id"]): item
        for item in control_state["actuators"]
    }
    positions = [float(actuators[can_id]["position_rad"]) for can_id in can_ids]
    velocities = [float(actuators[can_id]["velocity_rad_s"]) for can_id in can_ids]
    imu = control_state["imu"]
    quat_xyzw = imu["quat_xyzw"]
    gyro = imu["angular_velocity_rad_s"]
    quat_wxyz = [float(quat_xyzw[3]), float(quat_xyzw[0]), float(quat_xyzw[1]), float(quat_xyzw[2])]
    return (
        np.asarray(positions, dtype=np.float64),
        np.asarray(velocities, dtype=np.float64),
        quat_wxyz,
        np.asarray([float(value) for value in gyro], dtype=np.float64),
    )


def control_state_numeric_error(control_state: dict[str, Any], can_ids: list[int]) -> str | None:
    try:
        actuator_items = control_state["actuators"]
        actuators = {int(item["can_id"]): item for item in actuator_items}
    except (KeyError, TypeError, ValueError) as exc:
        return f"invalid actuator list: {exc}"

    for can_id in can_ids:
        item = actuators.get(can_id)
        if item is None:
            return f"missing actuator CAN ID 0x{can_id:X}"
        for field in ("position_rad", "velocity_rad_s"):
            value = item.get(field)
            if value is None:
                return f"actuator 0x{can_id:X} {field} is not available yet"
            try:
                float(value)
            except (TypeError, ValueError) as exc:
                return f"actuator 0x{can_id:X} {field} is not numeric: {exc}"

    try:
        imu = control_state["imu"]
        quat_xyzw = imu["quat_xyzw"]
        gyro = imu["angular_velocity_rad_s"]
    except (KeyError, TypeError) as exc:
        return f"invalid IMU state: {exc}"

    if not isinstance(quat_xyzw, list) or len(quat_xyzw) != 4:
        return "IMU quat_xyzw is not available yet"
    if not isinstance(gyro, list) or len(gyro) != 3:
        return "IMU angular_velocity_rad_s is not available yet"

    for index, value in enumerate(quat_xyzw):
        if value is None:
            return f"IMU quat_xyzw[{index}] is not available yet"
        try:
            float(value)
        except (TypeError, ValueError) as exc:
            return f"IMU quat_xyzw[{index}] is not numeric: {exc}"

    for index, value in enumerate(gyro):
        if value is None:
            return f"IMU angular_velocity_rad_s[{index}] is not available yet"
        try:
            float(value)
        except (TypeError, ValueError) as exc:
            return f"IMU angular_velocity_rad_s[{index}] is not numeric: {exc}"

    return None
