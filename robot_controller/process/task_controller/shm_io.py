from __future__ import annotations

import time
from typing import Any, Callable

import numpy as np

from robot_controller.core.robot_state_shm import RobotStateShmReader


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

    def wait_until_available(
        self,
        *,
        read_timeout_s: float,
        startup_timeout_s: float,
        should_run: Callable[[], bool],
    ) -> dict[str, Any]:
        deadline = time.monotonic() + startup_timeout_s
        last_error: Exception | None = None
        print("[task_controller] waiting for control_state", flush=True)
        while should_run() and time.monotonic() < deadline:
            try:
                state = self.latest(read_timeout_s)
                print("[task_controller] control_state received", flush=True)
                return state
            except RuntimeError as exc:
                last_error = exc
                time.sleep(0.02)
        if last_error is not None:
            raise RuntimeError(f"control_state was not readable during startup: {last_error}")
        raise RuntimeError("control_state was not readable during startup")


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
