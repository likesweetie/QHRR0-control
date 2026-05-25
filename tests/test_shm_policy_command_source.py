from __future__ import annotations

import struct
import time
import unittest
import uuid
from multiprocessing import shared_memory

from robot_controller.command import CommandReadStatus, ShmPolicyCommandSource
from robot_controller.core.config import MitCommandShmConfig
from robot_controller.core.state import MitTarget
from robot_controller.utils.shm_command_router import ShmMitCommandWriter
from robot_controller.utils.shm_manager import (
    MIT_COMMAND_MAGIC,
    MIT_COMMAND_VERSION,
    MIT_HEADER_FMT,
    mit_command_shm_size,
)


class ShmPolicyCommandSourceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = MitCommandShmConfig(
            name=f"test_mit_{uuid.uuid4().hex}",
            target_count=1,
        )
        self.segment = shared_memory.SharedMemory(
            name=self.config.name,
            create=True,
            size=mit_command_shm_size(self.config.target_count),
        )
        self.segment.buf[:] = b"\x00" * len(self.segment.buf)
        struct.pack_into(
            MIT_HEADER_FMT,
            self.segment.buf,
            0,
            MIT_COMMAND_MAGIC,
            MIT_COMMAND_VERSION,
            self.config.target_count,
            0,
            0,
            0,
        )

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
        writer = ShmMitCommandWriter(self.config, source_id=7)
        source = ShmPolicyCommandSource(self.config)
        try:
            writer.publish(
                [
                    MitTarget(
                        can_id=0x141,
                        position_rad=0.0,
                        velocity_rad_s=0.0,
                        kp=0.0,
                        kd=0.5,
                        torque_ff_nm=0.0,
                    )
                ]
            )
            result = source.read_latest()
            self.assertEqual(result.status, CommandReadStatus.AVAILABLE)
            self.assertIsNotNone(result.command)
            assert result.command is not None
            self.assertEqual(result.command.source, "shm:7")
            self.assertLess(abs(time.time() - result.command.timestamp), 1.0)
            self.assertEqual(result.command.targets[0].can_id, 0x141)
        finally:
            writer.close()
            source.close()


if __name__ == "__main__":
    unittest.main()
