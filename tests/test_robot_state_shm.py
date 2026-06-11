from __future__ import annotations

import unittest
import uuid

from qhrr0.app.robot_controller.shm.types.robot_state import RobotStateShm, new_robot_state
from qhrr0.app.robot_controller.state_machine import ControllerMode


class RobotStateShmTest(unittest.TestCase):
    def test_empty_initialized_segment_has_no_payload(self) -> None:
        name = f"qhrr_test_state_{uuid.uuid4().hex}"
        shm = RobotStateShm.create(name)
        try:
            reader = RobotStateShm.open(name)
            try:
                self.assertFalse(reader.read_relaxed().is_initialized())
            finally:
                reader.close()
        finally:
            shm.close()
            shm.unlink()

    def test_writer_publishes_readable_payload(self) -> None:
        name = f"qhrr_test_state_{uuid.uuid4().hex}"
        shm = RobotStateShm.create(name)
        try:
            writer = RobotStateShm.open(name)
            reader = RobotStateShm.open(name)
            try:
                writer.write(new_robot_state(ControllerMode.NORMAL))
                payload = reader.read_relaxed()
                self.assertTrue(payload.is_initialized())
                self.assertEqual(payload.controller_mode, ControllerMode.NORMAL)
            finally:
                writer.close()
                reader.close()
        finally:
            shm.close()
            shm.unlink()


if __name__ == "__main__":
    unittest.main()
