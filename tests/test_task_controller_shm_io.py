from __future__ import annotations

import contextlib
import io
import unittest

from robot_controller.subprocesses.task_controller.shm_io import (
    ControlStateNotReady,
    ControlStateReader,
    control_state_numeric_error,
    state_vectors,
)


def control_state(*, position=0.1, velocity=0.2, quat=None, gyro=None):
    return {
        "actuators": [
            {
                "can_id": 0x141,
                "position_rad": position,
                "velocity_rad_s": velocity,
            },
        ],
        "imu": {
            "quat_xyzw": [0.0, 0.0, 0.0, 1.0] if quat is None else quat,
            "angular_velocity_rad_s": [0.0, 0.0, 0.0] if gyro is None else gyro,
        },
    }


class FakeRobotStateReader:
    def __init__(self, states):
        self.states = list(states)

    def read_latest(self):
        if len(self.states) > 1:
            return self.states.pop(0)
        return self.states[0]


class TaskControllerShmIoTest(unittest.TestCase):
    def test_control_state_rejects_missing_actuator_position(self) -> None:
        error = control_state_numeric_error(control_state(position=None), [0x141])
        self.assertIn("position_rad is not available yet", str(error))

    def test_control_state_rejects_missing_imu_quaternion(self) -> None:
        error = control_state_numeric_error(control_state(quat=[0.0, None, 0.0, 1.0]), [0x141])
        self.assertIn("IMU quat_xyzw[1] is not available yet", str(error))

    def test_state_vectors_convert_numeric_state(self) -> None:
        dof_pos, dof_vel, quat_wxyz, gyro = state_vectors(control_state(), [0x141])
        self.assertEqual(dof_pos.tolist(), [0.1])
        self.assertEqual(dof_vel.tolist(), [0.2])
        self.assertEqual(quat_wxyz, [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(gyro.tolist(), [0.0, 0.0, 0.0])

    def test_latest_ready_reports_not_ready_without_zero_fallback(self) -> None:
        reader = ControlStateReader.__new__(ControlStateReader)
        reader.reader = FakeRobotStateReader([control_state(position=None)])
        with self.assertRaisesRegex(ControlStateNotReady, "position_rad is not available yet"):
            reader.latest_ready([0x141])

    def test_wait_until_available_keeps_waiting_for_numeric_state(self) -> None:
        reader = ControlStateReader.__new__(ControlStateReader)
        reader.reader = FakeRobotStateReader([
            control_state(position=None),
            control_state(),
        ])
        with contextlib.redirect_stdout(io.StringIO()):
            state = reader.wait_until_available(
                poll_period_s=0.001,
                should_run=lambda: True,
                can_ids=[0x141],
            )
        self.assertEqual(state["actuators"][0]["position_rad"], 0.1)


if __name__ == "__main__":
    unittest.main()
