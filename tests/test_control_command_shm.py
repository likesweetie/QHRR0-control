from __future__ import annotations

import unittest
import uuid

from robot_controller.shm.control_command import ControlCommandShm, ControlTarget


class ControlCommandShmTest(unittest.TestCase):
    def test_relaxed_read_allows_direct_struct_payload(self) -> None:
        name = f"qhrr_test_control_command_{uuid.uuid4().hex}"
        shm = ControlCommandShm.create(name)
        try:
            writer = ControlCommandShm.open_writer(name)
            reader = ControlCommandShm.open_reader(name)
            try:
                writer.write_targets(
                    [
                        ControlTarget(
                            can_id=0x141,
                            q=1.0,
                            dq=2.0,
                            kp=3.0,
                            kd=4.0,
                            tau=5.0,
                        )
                    ]
                )
                command = reader.read_relaxed()
                self.assertEqual(command.num_targets, 1)
                self.assertEqual(command.targets[0].can_id, 0x141)
                self.assertAlmostEqual(command.targets[0].q, 1.0)
            finally:
                writer.close()
                reader.close()
        finally:
            shm.close()
            shm.unlink()


if __name__ == "__main__":
    unittest.main()

