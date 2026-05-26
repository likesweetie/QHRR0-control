from __future__ import annotations

import time
import unittest
import uuid

from robot_controller.command import CommandReadStatus, ShmPolicyCommandSource
from robot_controller.core.config import MitCommandShmConfig
from robot_controller.shm.control_command import ControlCommandShm, ControlTarget


class ShmPolicyCommandSourceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = MitCommandShmConfig(
            name=f"test_control_command_{uuid.uuid4().hex}",
            target_count=1,
        )
        self.segment = ControlCommandShm.create(self.config.name)

    def tearDown(self) -> None:
        self.segment.close()
        self.segment.unlink()

    def test_empty_segment_reports_no_command(self) -> None:
        source = ShmPolicyCommandSource(self.config)
        try:
            result = source.read_latest()
            self.assertEqual(result.status, CommandReadStatus.NO_COMMAND)
            self.assertIsNone(result.command)
        finally:
            source.close()

    def test_writer_published_command_is_available(self) -> None:
        source = ShmPolicyCommandSource(self.config)
        try:
            self.segment.write_targets(
                [
                    ControlTarget(
                        can_id=0x141,
                        q=0.0,
                        dq=0.0,
                        kp=0.0,
                        kd=0.5,
                        tau=0.0,
                    )
                ]
            )
            result = source.read_latest()
            self.assertEqual(result.status, CommandReadStatus.AVAILABLE)
            self.assertIsNotNone(result.command)
            assert result.command is not None
            self.assertEqual(result.command.source, "control_command_shm")
            self.assertLess(abs(time.time() - result.command.timestamp), 1.0)
            self.assertEqual(result.command.targets[0].can_id, 0x141)
        finally:
            source.close()


if __name__ == "__main__":
    unittest.main()
