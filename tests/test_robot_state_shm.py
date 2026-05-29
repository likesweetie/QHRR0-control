from __future__ import annotations

import unittest
import uuid

from robot_controller.shm.robot_state import RobotStateShm, new_robot_state
from robot_controller.state_machine import ControllerMode


class RobotStateShmTest(unittest.TestCase):
    def test_empty_initialized_segment_has_no_payload(self) -> None:
        name = f"qhrr_test_state_{uuid.uuid4().hex}"
        shm = RobotStateShm.create(name)
        try:
            reader = RobotStateShm.open_reader(name)
            try:
                self.assertIsNone(reader.read_latest())
            finally:
                reader.close()
        finally:
            shm.close()
            shm.unlink()

    def test_writer_publishes_readable_payload(self) -> None:
        name = f"qhrr_test_state_{uuid.uuid4().hex}"
        shm = RobotStateShm.create(name)
        try:
            writer = RobotStateShm.open_writer(name)
            reader = RobotStateShm.open_reader(name)
            try:
                writer.write(new_robot_state(ControllerMode.NORMAL))
                payload = reader.read_latest()
                self.assertIsNotNone(payload)
                assert payload is not None
                self.assertEqual(payload["controller_state"], "NORMAL")
            finally:
                writer.close()
                reader.close()
        finally:
            shm.close()
            shm.unlink()


if __name__ == "__main__":
    unittest.main()
